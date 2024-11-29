#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define  VISION_RECV_SIZE 34u
#define VISION_SEND_SIZE 128u

#pragma pack(1)
typedef enum
{
    NO_FIRE = 0,
    AUTO_FIRE = 1,
    AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
    NO_TARGET = 0,
    TARGET_CONVERGING = 1,
    READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
    NO_TARGET_NUM = 0,
    HERO1 = 1,
    ENGINEER2 = 2,
    INFANTRY3 = 3,
    INFANTRY4 = 4,
    INFANTRY5 = 5,
    OUTPOST = 6,
    SENTRY = 7,
    BASE = 8
} Target_Type_e;

typedef struct
{
    Fire_Mode_e fire_mode;
    Target_State_e target_state;
    Target_Type_e target_type;

    float pitch;
    float yaw;
    float vel_x;
    float vel_y;
    float angle;
    float gimbal_select;
} Vision_Recv_s;

typedef enum
{
    COLOR_NONE = 0,
    COLOR_BLUE = 2,
    COLOR_RED = 1,
} Enemy_Color_e;

typedef enum
{
    VISION_MODE_AIM = 0,
    VISION_MODE_SMALL_BUFF = 1,
    VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
    Enemy_Color_e enemy_color;
    Work_Mode_e work_mode;
    Bullet_Speed_e bullet_speed;

    float yaw;
    float pitch;
    float roll;
    float vel_x;
    float vel_y;
    float gimabl_angle;
    float gimbal_send;
    float game_progress;
    float stage_remain_time;
    float current_HP;
    float current_enemy_sentry_hp;
    float current_enemy_base_hp;
    float current_shield_hp;
    float current_base_hp;
} Vision_Send_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s* VisionInit(UART_HandleTypeDef* _handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

void SlamSetAltitude(float x, float y, float gimabl_angle);

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll, float GimbalSend);

void RefreeSetAltitude(float current_HP, float stage_remain_time, float game_progress, float enemy_sentry_HP,
                       float enemy_base_HP, float shield_HP, float current_base_HP);

#endif // !MASTER_PROCESS_H
