// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PITCH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

#define chassis_remote_ratio 10.0
#define chassis_power_buffer 2.0

/* cmd应用包含的模块实例指针和交互信息存储*/
static Publisher_t* chassis_cmd_pub; // 底盘控制消息发布者
static Subscriber_t* chassis_feed_sub; // 底盘反馈信息订阅者

static Chassis_Ctrl_Cmd_s chassis_cmd_send; // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t* rc_data; // 遥控器数据,初始化时返回
static Vision_Recv_s* vision_recv_data; // 视觉接收数据指针,初始化时返回

static Publisher_t* gimbal_cmd_pub; // 云台控制消息发布者
static Subscriber_t* gimbal_feed_sub; // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send; // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t* shoot_cmd_pub; // 发射控制消息发布者
static Subscriber_t* shoot_feed_sub; // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send; // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度

    float offset_angle = angle - YAW_ALIGN_ANGLE; // 计算初步偏差
#if YAW_ECD_GREATER_THAN_4096
    if (angle > 180.0f + YAW_ALIGN_ANGLE)
        offset_angle -= 360.0f;
#else
    if (angle < YAW_ALIGN_ANGLE - 180.0f)
        offset_angle += 360.0f;
#endif
    chassis_cmd_send.offset_angle = offset_angle;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    // 云台参数, 遥控器[左]侧开关确定云台控制数据
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;

    // 遥控器[左]侧开关在[上]，遥控器控制云台
    gimbal_cmd_send.yaw -= 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
    gimbal_cmd_send.pitch -= 0.001f * (float)rc_data[TEMP].rc.rocker_l1;

    chassis_cmd_send.vx = chassis_remote_ratio * (float)rc_data[TEMP].rc.rocker_r_;
    chassis_cmd_send.vy = chassis_remote_ratio * (float)rc_data[TEMP].rc.rocker_r1;
    // 遥控器[左]侧开关在[中]，视觉控制云台移动
    if (switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        if (vision_recv_data->target_state != NO_TARGET)
        {
            // 存在目标时视觉控制云台移动
            gimbal_cmd_send.yaw = vision_recv_data->yaw;
            gimbal_cmd_send.pitch = vision_recv_data->pitch;
        }
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_left))
    {
        // 开！
        shoot_cmd_send.friction_mode = FRICTION_ON;
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
    }
}

/**
 * @brief 控制输入为键鼠的模式和控制量设置
 *
 */
static void MouseKeyControlSet()
{
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    // 底盘和云台控制可能需要添加系数
    chassis_cmd_send.vx -= (float)(rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d);
    chassis_cmd_send.vy = (float)(rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s);
    gimbal_cmd_send.yaw -= (float)rc_data[TEMP].mouse.x * 0.005f;
    gimbal_cmd_send.pitch -= (float)rc_data[TEMP].mouse.y * 0.001f;

    if (rc_data[TEMP].mouse.press_r)
    {
        // 按住鼠标右键，视觉控制云台
        if (vision_recv_data->target_state != NO_TARGET)
        {
            gimbal_cmd_send.yaw = vision_recv_data->yaw;
            gimbal_cmd_send.pitch = vision_recv_data->pitch;
        }
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2)
    {
    // F键开关摩擦轮
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_X] % 2)
    {
    // X切换自瞄射击模式，
    case 0:
        vision_recv_data->fire_mode = AUTO_AIM;
        break;
    default:
        vision_recv_data->fire_mode = AUTO_FIRE;
        break;
    }

    if (shoot_cmd_send.friction_mode == FRICTION_ON)
    {
        // 左键单击
        switch (rc_data[TEMP].mouse.press_l)
        {
        case 0:
            shoot_cmd_send.load_mode = LOAD_STOP;
            break;
        default:
            if (!rc_data[TEMP].mouse.press_r || vision_recv_data->target_state == NO_TARGET)
            {
                shoot_cmd_send.load_mode = LOAD_1_BULLET;
            }
            else
            {
                if (vision_recv_data->fire_mode == AUTO_AIM)
                {
                    shoot_cmd_send.load_mode = LOAD_1_BULLET;
                }
                else if (vision_recv_data->fire_mode == AUTO_FIRE)
                {
                    shoot_cmd_send.load_mode = vision_recv_data->target_state == READY_TO_FIRE ? LOAD_1_BULLET : LOAD_STOP;
                }
            }
            break;
        }
    }
    else
    {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }

    if (rc_data[TEMP].key[KEY_PRESS].shift)
    {
        chassis_cmd_send.vx *= chassis_power_buffer;
        chassis_cmd_send.vy *= chassis_power_buffer;
    }
}

/**
 * @brief  遥控器离线后紧急停止
 *
 */
static void EmergencyHandler()
{
    // 遥控器离线或者右侧开关状态[上]
    if (!RemoteControlIsOnline() || switch_is_up(rc_data[TEMP].rc.switch_right) || robot_state == ROBOT_STOP)
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器在线且右侧开关状态不为[上]
    if (RemoteControlIsOnline() && !switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

static void ControlTask()
{
    // 根据遥控器右侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        // 遥控器右侧开关状态为[中]，遥控器控制
        RemoteControlSet();
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        // 遥控器右侧开关状态为[下]，键鼠控制
        MouseKeyControlSet();
    }

    // 软件限位
    if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
    {
        gimbal_cmd_send.pitch = -30;
    }
    else if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
    {
        gimbal_cmd_send.pitch = 30;
    }
}

static void PublishMessageTask()
{
    // 设置视觉发送数据,还需增加加速度和角速度数据
    RefreeSetAltitude(
        chassis_fetch_data.current_HP,
        chassis_fetch_data.stage_remain_time,
        chassis_fetch_data.game_progress,
        chassis_fetch_data.current_enemy_sentry_hp,
        chassis_fetch_data.current_enemy_base_hp,
        chassis_fetch_data.current_shield_hp,
        chassis_fetch_data.current_base_hp
    );
    VisionSetFlag(chassis_fetch_data.enemy_color, VISION_MODE_AIM, chassis_fetch_data.bullet_speed);
    SlamSetAltitude(chassis_fetch_data.real_vx, chassis_fetch_data.real_vy, gimbal_fetch_data.gimbal_imu_data.Gyro[2]);
    VisionSetAltitude(gimbal_cmd_send.yaw, -gimbal_cmd_send.pitch, 0.0f, 1.0f);

    PubPushMessage(chassis_cmd_pub, (void*)&chassis_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void*)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void*)&gimbal_cmd_send);
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    SubGetMessage(chassis_feed_sub, (void*)&chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();

    ControlTask();

    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    PublishMessageTask();
}
