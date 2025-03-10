/**
 * @file referee_protocol.h
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

#ifndef referee_protocol_H
#define referee_protocol_H

#include "stdint.h"

/****************************宏定义部分****************************/

#define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
#define Robot_Red 0
#define Robot_Blue 1
#define Communicate_Data_LEN 5  // 自定义交互数据长度，该长度决定了我方发送和他方接收，自定义交互数据协议更改时只需要更改此宏定义即可

#pragma pack(1)

/****************************通信协议格式****************************/

/* 通信协议格式偏移，枚举类型,代替#define声明 */
typedef enum
{
	FRAME_HEADER_Offset = 0,
	CMD_ID_Offset = 5,
	DATA_Offset = 7,
} JudgeFrameOffset_e;

/* 通信协议长度 */
typedef enum
{
	LEN_HEADER = 5, // 帧头长
	LEN_CMDID = 2,	// 命令码长度
	LEN_TAIL = 2,	// 帧尾CRC16

	LEN_CRC8 = 4, // 帧头CRC8校验长度=帧头+数据长+包序号
} JudgeFrameLength_e;

/****************************帧头****************************/
/****************************帧头****************************/

/* 帧头偏移 */
typedef enum
{
	SOF = 0,		 // 起始位
	DATA_LENGTH = 1, // 帧内数据长度,根据这个来获取数据长度
	SEQ = 3,		 // 包序号
	CRC8 = 4		 // CRC8
} FrameHeaderOffset_e;

/* 帧头定义 */
typedef struct
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
} xFrameHeader;

/****************************cmd_id命令码说明****************************/
/****************************cmd_id命令码说明****************************/

/* 命令码ID,用来判断接收的是什么数据 */
typedef enum
{
	ID_game_state = 0x0001,				                 // 比赛状态数据
	ID_game_result = 0x0002,			                 // 比赛结果数据
	ID_game_robot_survivors = 0x0003,	                 // 比赛机器人血量数据
	ID_event_data = 0x0101,				                 // 场地事件数据
	ID_supply_projectile_action = 0x0102,                // 场地补给站动作标识数据
	ID_referee_warning_t = 0x0104,                     // 裁判警告数据,已方判罚/判负时触发发送
    ID_dart_info_t=0x0105,                  // 飞镖发射相关数据
	ID_robot_status_t = 0x0201,		         // 机器人性能体系数据
	ID_power_heat_data = 0x0202,		                 // 实时底盘功率和枪口热量数据
	ID_game_robot_pos = 0x0203,			                 // 机器人位置数据
	ID_buff_t = 0x0204,				         // 机器人增益数据
	ID_air_support_data_t = 0x0205,	                 // 空中支援时间数据
	ID_robot_hurt = 0x0206,				                 // 伤害状态数据
	ID_shoot_data = 0x0207,                              // 实时射击数据
    ID_projectile_allowance=0x0208,                      // 允许发弹量
    ID_rfid_status_t=0x0209,                        // 机器人 RFID 模块状态
    ID_darts_client_cmd=0x020A,                 // 飞镖选手端指令数据
    ID_ground_robot_position_t=0x020B,                       // 地面机器人位置数据
    ID_radar_mark_data_t=0x020C,                 // 雷达标记进度数据
    ID_sentry_info_t=0x020D,                             // 哨兵自主决策信息同步
    ID_radar_info_t=0x020E,                              // 雷达自主决策信息同步

	ID_student_interactive = 0x0301,	                 // 机器人间交互数据
    ID_custom_robot_data_t=0x0302,      // 自定义控制器与机器人交互数据，发送方触发发送
    ID_map_command_t=0x0303,                           // 选手端小地图交互数据，选手端触发发送
    ID_remote_control_t=0x0304,                         // 键鼠遥控数据
    ID_map_robot_data_t=0x0305,                     // 选手端小地图接收雷达数据
    ID_custom_client_data_t=0x0306,     // 自定义控制器与选手端交互数据，发送方触发发送
    ID_map_data_t=0x0307,                    // 选手端小地图接收哨兵数据
    ID_custom_info_t=0x0308                      // 选手端小地图接收机器人数据
} CmdID_e;

/* 命令码数据段长,根据官方协议来定义长度，还有自定义数据长度 */
typedef enum
{
    LEN_game_state = 11,                                 // 0x0001
    LEN_game_result = 1,                                 // 0x0002
    LEN_game_robot_survivors = 32,                       // 0x0003
    LEN_event_data = 4,                                 // 0x0101
    LEN_supply_projectile_action = 4,                   // 0x0102
    LEN_referee_warning_t = 3,                         // 0x0104,
    LEN_dart_info_t = 3,                   // 0x0105
    LEN_robot_status_t = 13,                    // 0x0201
    LEN_power_heat_data = 16,                             // 0x0202
    LEN_game_robot_pos = 16,                             // 0x0203
    LEN_buff_t = 6,                            // 0x0204
    LEN_air_support_data_t = 2,                         // 0x0205
    LEN_robot_hurt = 1,                                 // 0x0206
    LEN_shoot_data = 7,                                  // 0x0207
    LEN_projectile_allowance= 6,                          //0x0208
    LEN_rfid_status_t = 4,                           // 0x0209
    LEN_darts_client_cmd = 6,                  // 0x020A
    LEN_ground_robot_position_t = 40,                       // 0x020B
    LEN_radar_mark_data_t = 6,                   // 0x020C
    LEN_sentry_info_t = 4,       // 0x020D
    LEN_radar_info_t = 1,        // 0x020E
    LEN_student_interactive = 6 + Communicate_Data_LEN,      // 0x0301
    LEN_custom_robot_data_t = 30,        // 0x0302
    LEN_map_command_t = 15,                           // 0x0303
    LEN_remote_control_t = 12,                             // 0x0304
    LEN_map_robot_data_t = 10,                              // 0x0305
    LEN_custom_client_data_t = 8,             // 0x0306
    LEN_map_data_t = 103,                    // 0x0307
    LEN_custom_info_t = 34,                      // 0x0308

} JudgeDataLength_e;
/*
              */
/****************************接收数据的详细说明****************************/
/****************************接收数据的详细说明****************************/

/* ID: 0x0001  Byte:  11    比赛状态数据 */
typedef  struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
} ext_game_state_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct
{
	uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct
{
	uint32_t event_type;
} ext_event_data_t;

/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef struct
{
	uint8_t reserved;//supply_projectile_id
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* ID: 0X0104  Byte: 3    裁判警告数据 */
typedef  struct
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
}ext_referee_warning_t;

/* ID: 0X0105  Byte: 3    飞镖发射相关数据 */
typedef  struct
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
}ext_dart_info_t;

/* ID: 0X0201  Byte: 13    机器人状态数据 */
typedef struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
} ext_game_robot_state_t;

/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef struct
{
	uint16_t chassis_voltage;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef struct
{
	float x;
	float y;
	float angle;
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  6    机器人增益数据 */
typedef struct
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
} ext_buff_musk_t;

/* ID: 0x0205  Byte:  2    空中机器人能量状态数据 */
typedef struct
{
	uint8_t airforce_status;
 	uint8_t time_remain;
} ext_air_support_data_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;

/* ID: 0x0208  Byte:  6    允许发弹量 */
typedef struct
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
}ext_projectile_allowance_t;

/* ID: 0x0209  Byte:  4    机器人 RFID 模块状态 */
typedef struct{
    uint32_t rfid_status;
}ext_rfid_status_t;

/* ID: 0x020A  Byte:  6    飞镖选手端指令数据 */
typedef struct{
        uint8_t dart_launch_opening_status;
        uint8_t reserved;
        uint16_t target_change_time;
        uint16_t latest_launch_cmd_time;
}ext_dart_client_cmd_t;

/* ID: 0x020B  Byte:  40    地面机器人位置数据 */
typedef struct{
        float hero_x;
        float hero_y;
        float engineer_x;
        float engineer_y;
        float standard_3_x;
        float standard_3_y;
        float standard_4_x;
        float standard_4_y;
        float standard_5_x;
        float standard_5_y;
}ext_ground_robot_position_t;

/* ID: 0x020C  Byte:  6   雷达标记进度数据 */
typedef struct{
        uint8_t mark_hero_progress;
        uint8_t mark_engineer_progress;
        uint8_t mark_standard_3_progress;
        uint8_t mark_standard_4_progress;
        uint8_t mark_standard_5_progress;
        uint8_t mark_sentry_progress;
}ext_radar_mark_data_t;

/* ID: 0x020D  Byte:  4    哨兵自主决策信息同步 */
typedef struct{
        uint32_t sentry_info;
}ext_sentry_info_t;

/* ID: 0x020E  Byte:  1    雷达自主决策信息同步 */
typedef struct{
        uint8_t radar_info;
}ext_radar_info_t;



/* ID: 0x0302  Byte:  30    自定义控制器与机器人交互数据，发送方触发发送 */
typedef struct{
        uint8_t data[30];
}ext_custom_robot_data_t;

/* ID: 0x0303  Byte:  15   选手端小地图交互数据，选手端触发发送 */
typedef struct{
        float target_position_x;
        float target_position_y;
        uint8_t cmd_keyboard;
        uint8_t target_robot_id;
        uint8_t cmd_source;
}ext_map_command_t;

/* ID: 0x0304  Byte:  12    键鼠遥控数据 */
typedef struct{
        int16_t mouse_X;
        int16_t mouse_y;
        int16_t mouse_z;
        int8_t left_button_down;
        int8_t right_button_down;
        uint16_t keyboard_value;
        uint16_t reserved;
}ext_remote_control_t;

/* ID: 0x0305  Byte:  10    选手端小地图接收雷达数据 */
typedef struct{
        uint16_t target_robot_id;
        float target_position_x;
        float target_position_y;
}ext_map_robot_data_t;

/* ID: 0x0306  Byte:  8    自定义控制器与选手端交互数据，发送方触发发送*/
typedef struct{
            uint16_t key_value;
            uint16_t x_position:12;
            uint16_t mouse_left:4;
            uint16_t y_position:12;
            uint16_t mouse_right:4;
            uint16_t reserved;
}ext_custom_client_data_t;

/* ID: 0x0307  Byte:  103    选手端小地图接收哨兵数据 */
typedef struct{
        uint8_t intention;
        uint16_t start_position_x;
        uint16_t start_position_y;
        int8_t delta_x[49];
        int8_t delta_y[49];
        uint16_t sender_id;
}ext_map_data_t;

/* ID: 0x0308  Byte:  34    选手端小地图接收 */
typedef struct{
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[30];
}ext_custom_info_t;
/****************************机器人交互数据****************************/
/****************************机器人交互数据****************************/
/* 发送的内容数据段最大为 112 检测是否超出大小限制?实际上图形段不会超，数据段最多30个，也不会超//0x0200~0x02FF */
/* 交互数据头结构 */
/* ID: 0x0301  Byte:  6+ Communicate_Data_LEN    机器人间交互数据 */
typedef struct
{
	uint16_t data_cmd_id; // 由于存在多个内容 ID，但整个cmd_id 上行频率最大为 10Hz，请合理安排带宽。注意交互部分的上行频率
	uint16_t sender_ID;
	uint16_t receiver_ID;
    //uint8_t user_data[Communicate_Data_LEN]; 自定义数据
} ext_robot_interaction_data_t;
/* 机器人id */
typedef enum
{
	// 红方机器人ID
	RobotID_RHero = 1,
	RobotID_REngineer = 2,
	RobotID_RStandard1 = 3,
	RobotID_RStandard2 = 4,
	RobotID_RStandard3 = 5,
	RobotID_RAerial = 6,
	RobotID_RSentry = 7,
    RobotID_Rdart =8,
	RobotID_RRadar = 9,
	// 蓝方机器人ID
	RobotID_BHero = 101,
	RobotID_BEngineer = 102,
	RobotID_BStandard1 = 103,
	RobotID_BStandard2 = 104,
	RobotID_BStandard3 = 105,
	RobotID_BAerial = 106,
	RobotID_BSentry = 107,
    RobotID_Bdart =108,
	RobotID_BRadar = 109,
} Robot_ID_e;

/* 交互数据ID */
typedef enum
{
	UI_Data_ID_Del = 0x100,
	UI_Data_ID_Draw1 = 0x101,
	UI_Data_ID_Draw2 = 0x102,
	UI_Data_ID_Draw5 = 0x103,
	UI_Data_ID_Draw7 = 0x104,
	UI_Data_ID_DrawChar = 0x110,
    UI_Data_ID_sentry=0x120,
    UI_Data_ID_radar=0x121,
	/* 自定义交互数据部分 */
	Communicate_Data_ID = 0x0200,//0x0200~0x02FF

} Interactive_Data_ID_e;
/* 交互数据长度 */
typedef enum
{
	Interactive_Data_LEN_Head = 6,
	UI_Operate_LEN_Del = 2,
	UI_Operate_LEN_PerDraw = 15,
	UI_Operate_LEN_DrawChar = 15 + 30,
    UI_Operate_LEN_Decision_sentry=4,
    UI_Operate_LEN_Decision_radar=1,
	/* 自定义交互数据部分 */
	 //Communicate_Data_LEN = 5,//自定义0x0200~0x02FF

} Interactive_Data_Length_e;

/****************************自定义交互数据****************************/
/*
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	自定义交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz
*/
// 自定义交互数据协议，可更改，更改后需要修改最上方宏定义数据长度的值
typedef struct
{
	uint8_t data[Communicate_Data_LEN]; // 数据段,n需要小于113
} robot_interactive_data_t;

// 机器人交互信息_发送
typedef struct
{
	xFrameHeader FrameHeader;
	uint16_t CmdID;
	ext_robot_interaction_data_t datahead;
	robot_interactive_data_t Data; // 数据段
	uint16_t frametail;
} Communicate_SendData_t;
// 机器人交互信息_接收
typedef struct
{
	ext_robot_interaction_data_t datahead;
	robot_interactive_data_t Data; // 数据段
} Communicate_ReceiveData_t;

/****************************UI交互数据****************************/



/* 图形数据 */
typedef struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t figure_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t details_a : 9;
	uint32_t details_b : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
    uint32_t details_c: 10;
    uint32_t details_d: 11;
    uint32_t details_e: 11;
} interaction_figure_t;

typedef struct
{
	interaction_figure_t Graph_Control;
	uint8_t show_Data[30];
} String_Data_t; // 打印字符串数据

/* 删除操作 */
typedef enum
{
	UI_Data_Del_NoOperate = 0,
	UI_Data_Del_Layer = 1,
	UI_Data_Del_ALL = 2, // 删除全部图层，后面的参数已经不重要了。
} UI_Delete_Operate_e;

/* 图形配置参数__图形操作 */
typedef enum
{
	UI_Graph_ADD = 1,
	UI_Graph_Change = 2,
	UI_Graph_Del = 3,
} UI_Graph_Operate_e;

/* 图形配置参数__图形类型 */
typedef enum
{
	UI_Graph_Line = 0,		// 直线
	UI_Graph_Rectangle = 1, // 矩形
	UI_Graph_Circle = 2,	// 整圆
	UI_Graph_Ellipse = 3,	// 椭圆
	UI_Graph_Arc = 4,		// 圆弧
	UI_Graph_Float = 5,		// 浮点型
	UI_Graph_Int = 6,		// 整形
	UI_Graph_Char = 7,		// 字符型

} UI_Graph_Type_e;

/* 图形配置参数__图形颜色 */
typedef enum
{
	UI_Color_Main = 0, // 红蓝主色
	UI_Color_Yellow = 1,
	UI_Color_Green = 2,
	UI_Color_Orange = 3,
	UI_Color_Purplish_red = 4, // 紫红色
	UI_Color_Pink = 5,
	UI_Color_Cyan = 6, // 青色
	UI_Color_Black = 7,
	UI_Color_White = 8,

} UI_Graph_Color_e;

typedef  struct//哨兵自主决策指令：0x0120
{
    uint32_t sentry_cmd;
} sentry_cmd_t;

typedef struct//雷达自主决策指令：0x0121
{
    uint8_t radar_cmd;
} radar_cmd_t;

#pragma pack()

#endif
