/**
 * @file referee_protocol.h
 * @brief RoboMaster 2026 referee system protocol definitions
 */

#ifndef referee_protocol_H
#define referee_protocol_H

#include "stdint.h"

#define REFEREE_SOF 0xA5
#define Robot_Red 0
#define Robot_Blue 1
#define REFEREE_UI_MAX_STRING_LEN 30u
#define REFEREE_INTERACTIVE_MAX_DATA_LEN 112u
#define REFEREE_MAX_DATA_LEN 300u
#define REFEREE_MAX_FRAME_SIZE (LEN_HEADER + LEN_CMDID + REFEREE_MAX_DATA_LEN + LEN_TAIL)

#pragma pack(1)

typedef enum
{
	FRAME_HEADER_Offset = 0,
	CMD_ID_Offset = 5,
	DATA_Offset = 7,
} JudgeFrameOffset_e;

typedef enum
{
	LEN_HEADER = 5,
	LEN_CMDID = 2,
	LEN_TAIL = 2,
	LEN_CRC8 = 4,
} JudgeFrameLength_e;

typedef enum
{
	SOF = 0,
	DATA_LENGTH = 1,
	SEQ = 3,
	CRC8 = 4
} FrameHeaderOffset_e;

typedef struct
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
} xFrameHeader;

typedef enum
{
	ID_game_state = 0x0001,
	ID_game_result = 0x0002,
	ID_game_robot_survivors = 0x0003,
	ID_event_data = 0x0101,
	ID_referee_warning = 0x0104,
	ID_dart_info = 0x0105,
	ID_game_robot_state = 0x0201,
	ID_power_heat_data = 0x0202,
	ID_game_robot_pos = 0x0203,
	ID_buff = 0x0204,
	ID_robot_hurt = 0x0206,
	ID_shoot_data = 0x0207,
	ID_projectile_allowance = 0x0208,
	ID_rfid_status = 0x0209,
	ID_dart_client_cmd = 0x020A,
	ID_ground_robot_position = 0x020B,
	ID_sentry_info = 0x020D,
	ID_student_interactive = 0x0301,
	ID_custom_controller_robot_interaction = 0x0302,
	ID_map_command = 0x0303,
	ID_custom_controller_client_interaction = 0x0306,
	ID_map_data = 0x0307,
	ID_custom_info = 0x0308,
	ID_robot_custom_controller_data = 0x0309,
	ID_robot_custom_client_data = 0x0310,
	ID_custom_client_robot_cmd = 0x0311,
} CmdID_e;

typedef enum
{
	LEN_game_state = 11,
	LEN_game_result = 1,
	LEN_game_robot_HP = 16,
	LEN_event_data = 4,
	LEN_referee_warning = 3,
	LEN_dart_info = 3,
	LEN_game_robot_state = 13,
	LEN_power_heat_data = 14,
	LEN_game_robot_pos = 12,
	LEN_buff = 8,
	LEN_robot_hurt = 1,
	LEN_shoot_data = 7,
	LEN_projectile_allowance = 8,
	LEN_rfid_status = 5,
	LEN_dart_client_cmd = 6,
	LEN_ground_robot_position = 40,
	LEN_sentry_info = 6,
	LEN_student_interactive_head = 6,
	LEN_student_interactive_max = LEN_student_interactive_head + REFEREE_INTERACTIVE_MAX_DATA_LEN,
	LEN_custom_controller_robot_interaction = 30,
	LEN_map_command = 12,
	LEN_custom_controller_client_interaction = 8,
	LEN_map_data = 105,
	LEN_custom_info = 34,
	LEN_robot_custom_controller_data = 30,
	LEN_robot_custom_client_data = 300,
	LEN_custom_client_robot_cmd = 30,
} JudgeDataLength_e;

typedef struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_state_t;

typedef struct
{
	uint8_t winner;
} ext_game_result_t;

typedef struct
{
	uint16_t ally_1_robot_HP;
	uint16_t ally_2_robot_HP;
	uint16_t ally_3_robot_HP;
	uint16_t ally_4_robot_HP;
	uint16_t reserved;
	uint16_t ally_7_robot_HP;
	uint16_t ally_outpost_HP;
	uint16_t ally_base_HP;
} ext_game_robot_HP_t;

typedef struct
{
	uint32_t event_type;
} ext_event_data_t;

typedef struct
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
} ext_referee_warning_t;

typedef struct
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} ext_dart_info_t;

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
	uint8_t reserved : 5;
} ext_game_robot_state_t;

typedef struct
{
	uint16_t reserved_voltage;
	uint16_t reserved_current;
	float reserved_power;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

typedef struct
{
	float x;
	float y;
	float angle;
} ext_game_robot_pos_t;

typedef struct
{
	uint8_t recovery_buff;
	uint16_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy;
} ext_buff_t;

typedef struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef struct
{
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
} ext_shoot_data_t;

typedef struct
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
	uint16_t projectile_allowance_fortress;
} ext_projectile_allowance_t;

typedef struct
{
	uint32_t rfid_status;
	uint8_t rfid_status_2;
} ext_rfid_status_t;

typedef struct
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;

typedef struct
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float reserved_x;
	float reserved_y;
} ext_ground_robot_position_t;

typedef struct
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
} ext_sentry_info_t;

typedef struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

typedef struct
{
	ext_student_interactive_header_data_t datahead;
	uint16_t data_len;
	uint8_t data[REFEREE_INTERACTIVE_MAX_DATA_LEN];
} Communicate_ReceiveData_t;

typedef struct
{
	uint8_t data[LEN_custom_controller_robot_interaction];
} ext_custom_controller_robot_interaction_t;

typedef struct
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint16_t cmd_source;
} ext_map_command_t;

typedef struct
{
	uint8_t data[LEN_custom_controller_client_interaction];
} ext_custom_controller_client_interaction_t;

typedef struct
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
} ext_map_data_t;

typedef struct
{
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} ext_custom_info_t;

typedef struct
{
	uint8_t data[LEN_robot_custom_controller_data];
} ext_robot_custom_controller_data_t;

typedef struct
{
	uint8_t data[LEN_robot_custom_client_data];
} ext_robot_custom_client_data_t;

typedef struct
{
	uint8_t data[LEN_custom_client_robot_cmd];
} ext_custom_client_robot_cmd_t;

typedef enum
{
	RobotID_RHero = 1,
	RobotID_REngineer = 2,
	RobotID_RStandard1 = 3,
	RobotID_RStandard2 = 4,
	RobotID_RStandard3 = 5,
	RobotID_RAerial = 6,
	RobotID_RSentry = 7,
	RobotID_RDart = 8,
	RobotID_RRadar = 9,
	RobotID_BHero = 101,
	RobotID_BEngineer = 102,
	RobotID_BStandard1 = 103,
	RobotID_BStandard2 = 104,
	RobotID_BStandard3 = 105,
	RobotID_BAerial = 106,
	RobotID_BSentry = 107,
	RobotID_BDart = 108,
	RobotID_BRadar = 109,
} Robot_ID_e;

typedef enum
{
	UI_Data_ID_Del = 0x0100,
	UI_Data_ID_Draw1 = 0x0101,
	UI_Data_ID_Draw2 = 0x0102,
	UI_Data_ID_Draw5 = 0x0103,
	UI_Data_ID_Draw7 = 0x0104,
	UI_Data_ID_DrawChar = 0x0110,
	UI_Data_ID_SentryCmd = 0x0120,
	Communicate_Data_ID = 0x0200,
} Interactive_Data_ID_e;

typedef enum
{
	Interactive_Data_LEN_Head = LEN_student_interactive_head,
	UI_Operate_LEN_Del = 2,
	UI_Operate_LEN_PerDraw = 15,
	UI_Operate_LEN_DrawChar = 45,
} Interactive_Data_Length_e;

typedef struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t radius : 10;
	uint32_t end_x : 11;
	uint32_t end_y : 11;
} Graph_Data_t;

typedef struct
{
	Graph_Data_t Graph_Control;
	uint8_t show_Data[REFEREE_UI_MAX_STRING_LEN];
} String_Data_t;

typedef enum
{
	UI_Data_Del_NoOperate = 0,
	UI_Data_Del_Layer = 1,
	UI_Data_Del_ALL = 2,
} UI_Delete_Operate_e;

typedef enum
{
	UI_Graph_ADD = 1,
	UI_Graph_Change = 2,
	UI_Graph_Del = 3,
} UI_Graph_Operate_e;

typedef enum
{
	UI_Graph_Line = 0,
	UI_Graph_Rectangle = 1,
	UI_Graph_Circle = 2,
	UI_Graph_Ellipse = 3,
	UI_Graph_Arc = 4,
	UI_Graph_Float = 5,
	UI_Graph_Int = 6,
	UI_Graph_Char = 7,
} UI_Graph_Type_e;

typedef enum
{
	UI_Color_Main = 0,
	UI_Color_Yellow = 1,
	UI_Color_Green = 2,
	UI_Color_Orange = 3,
	UI_Color_Purplish_red = 4,
	UI_Color_Pink = 5,
	UI_Color_Cyan = 6,
	UI_Color_Black = 7,
	UI_Color_White = 8,
} UI_Graph_Color_e;

_Static_assert(sizeof(xFrameHeader) == LEN_HEADER, "referee frame header size mismatch");
_Static_assert(sizeof(ext_game_robot_HP_t) == LEN_game_robot_HP, "0x0003 size mismatch");
_Static_assert(sizeof(ext_referee_warning_t) == LEN_referee_warning, "0x0104 size mismatch");
_Static_assert(sizeof(ext_dart_info_t) == LEN_dart_info, "0x0105 size mismatch");
_Static_assert(sizeof(ext_game_robot_state_t) == LEN_game_robot_state, "0x0201 size mismatch");
_Static_assert(sizeof(ext_power_heat_data_t) == LEN_power_heat_data, "0x0202 size mismatch");
_Static_assert(sizeof(ext_game_robot_pos_t) == LEN_game_robot_pos, "0x0203 size mismatch");
_Static_assert(sizeof(ext_buff_t) == LEN_buff, "0x0204 size mismatch");
_Static_assert(sizeof(ext_projectile_allowance_t) == LEN_projectile_allowance, "0x0208 size mismatch");
_Static_assert(sizeof(ext_rfid_status_t) == LEN_rfid_status, "0x0209 size mismatch");
_Static_assert(sizeof(ext_dart_client_cmd_t) == LEN_dart_client_cmd, "0x020A size mismatch");
_Static_assert(sizeof(ext_ground_robot_position_t) == LEN_ground_robot_position, "0x020B size mismatch");
_Static_assert(sizeof(ext_sentry_info_t) == LEN_sentry_info, "0x020D size mismatch");
_Static_assert(sizeof(ext_map_command_t) == LEN_map_command, "0x0303 size mismatch");
_Static_assert(sizeof(ext_map_data_t) == LEN_map_data, "0x0307 size mismatch");
_Static_assert(sizeof(ext_custom_info_t) == LEN_custom_info, "0x0308 size mismatch");
_Static_assert(sizeof(ext_robot_custom_client_data_t) == LEN_robot_custom_client_data, "0x0310 size mismatch");
_Static_assert(sizeof(Graph_Data_t) == UI_Operate_LEN_PerDraw, "UI graph size mismatch");
_Static_assert(sizeof(String_Data_t) == UI_Operate_LEN_DrawChar, "UI char size mismatch");

#pragma pack()

#endif
