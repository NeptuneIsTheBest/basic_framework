#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "crc_ref.h"

#if REMOTE_SOURCE == REMOTE_SOURCE_VIDEO
#define REMOTE_CONTROL_FRAME_SIZE 21u // 图传接收端输出21字节固定帧
#else
#define REMOTE_CONTROL_FRAME_SIZE 18u // DR16/DBUS固定18字节
#endif

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

static void UpdateKeyState(uint16_t key_value)
{
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = key_value;
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }
}

static void UpdateButtonCount()
{
    if (rc_ctrl[TEMP].button.pause && !rc_ctrl[LAST].button.pause)
        rc_ctrl[TEMP].button_count[RC_BUTTON_PAUSE]++;
    if (rc_ctrl[TEMP].button.fn_left && !rc_ctrl[LAST].button.fn_left)
        rc_ctrl[TEMP].button_count[RC_BUTTON_FN_LEFT]++;
    if (rc_ctrl[TEMP].button.fn_right && !rc_ctrl[LAST].button.fn_right)
        rc_ctrl[TEMP].button_count[RC_BUTTON_FN_RIGHT]++;
    if (rc_ctrl[TEMP].button.trigger && !rc_ctrl[LAST].button.trigger)
        rc_ctrl[TEMP].button_count[RC_BUTTON_TRIGGER]++;
}

#if REMOTE_SOURCE == REMOTE_SOURCE_VIDEO
static uint32_t ExtractBitsLE(const uint8_t *frame, uint16_t bit_offset, uint8_t bit_len)
{
    uint32_t value = 0;

    for (uint8_t i = 0; i < bit_len; ++i)
    {
        uint16_t current_bit = bit_offset + i;
        if ((frame[current_bit / 8u] >> (current_bit % 8u)) & 0x01u)
            value |= (1u << i);
    }

    return value;
}

static uint8_t VideoModeSwitchToRC(uint8_t mode_switch)
{
    switch (mode_switch)
    {
    case 0u:
        return RC_SW_UP; // C
    case 1u:
        return RC_SW_MID; // N
    case 2u:
        return RC_SW_DOWN; // S
    default:
        return RC_SW_MID;
    }
}

static uint8_t ParseRemoteFrame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame_len != REMOTE_CONTROL_FRAME_SIZE)
        return 0;

    if (frame[0] != 0xA9u || frame[1] != 0x53u)
        return 0;

    if (!Verify_CRC16_Check_Sum((uint8_t *)frame, frame_len))
        return 0;

    rc_ctrl[TEMP].rc.rocker_r_ = (int16_t)ExtractBitsLE(frame, 16u, 11u) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.rocker_r1 = (int16_t)ExtractBitsLE(frame, 27u, 11u) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.rocker_l_ = (int16_t)ExtractBitsLE(frame, 38u, 11u) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.rocker_l1 = (int16_t)ExtractBitsLE(frame, 49u, 11u) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.dial = (int16_t)ExtractBitsLE(frame, 65u, 11u) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.switch_left = VideoModeSwitchToRC((uint8_t)ExtractBitsLE(frame, 60u, 2u));
    rc_ctrl[TEMP].rc.switch_right = RC_SW_MID;
    RectifyRCjoystick();

    rc_ctrl[TEMP].button.pause = (uint8_t)ExtractBitsLE(frame, 62u, 1u);
    rc_ctrl[TEMP].button.fn_left = (uint8_t)ExtractBitsLE(frame, 63u, 1u);
    rc_ctrl[TEMP].button.fn_right = (uint8_t)ExtractBitsLE(frame, 64u, 1u);
    rc_ctrl[TEMP].button.trigger = (uint8_t)ExtractBitsLE(frame, 76u, 1u);

    rc_ctrl[TEMP].mouse.x = (int16_t)ExtractBitsLE(frame, 80u, 16u);
    rc_ctrl[TEMP].mouse.y = (int16_t)ExtractBitsLE(frame, 96u, 16u);
    rc_ctrl[TEMP].mouse.press_l = (uint8_t)(ExtractBitsLE(frame, 128u, 2u) != 0u);
    rc_ctrl[TEMP].mouse.press_r = (uint8_t)(ExtractBitsLE(frame, 130u, 2u) != 0u);

    UpdateKeyState((uint16_t)ExtractBitsLE(frame, 136u, 16u));
    UpdateButtonCount();
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));
    return 1;
}
#else
static uint8_t ParseRemoteFrame(const uint8_t *sbus_buf, uint16_t frame_len)
{
    if (frame_len != REMOTE_CONTROL_FRAME_SIZE)
        return 0;

    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
    RectifyRCjoystick();

    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8));
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8));
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];

    memset(&rc_ctrl[TEMP].button, 0, sizeof(rc_ctrl[TEMP].button));
    UpdateKeyState((uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8)));
    UpdateButtonCount();
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));
    return 1;
}
#endif

/**
 * @brief 对协议解析的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    if (ParseRemoteFrame(rc_usart_instance->recv_buff, rc_usart_instance->recv_size))
        DaemonReload(rc_daemon_instance);
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}
