/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "crc_ref.h"
#include "daemon.h"
#include "string.h"
#include "task.h"

#define RE_RX_BUFFER_SIZE REFEREE_MAX_FRAME_SIZE // 裁判系统接收缓冲区大小

static USARTInstance *referee_usart_instance; // 裁判系统串口实例
static DaemonInstance *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

static uint8_t RefereeCopyData(void *dst, const uint8_t *src, uint16_t expected_len, uint16_t actual_len)
{
	if (actual_len != expected_len)
		return 0;

	memcpy(dst, src, expected_len);
	return 1;
}

static void RefereeParseInteractiveData(const uint8_t *data, uint16_t data_len)
{
	if (data_len < LEN_student_interactive_head)
		return;

	memset(&referee_info.ReceiveData, 0, sizeof(referee_info.ReceiveData));
	memcpy(&referee_info.ReceiveData.datahead, data, LEN_student_interactive_head);
	referee_info.ReceiveData.data_len = data_len - LEN_student_interactive_head;
	if (referee_info.ReceiveData.data_len > REFEREE_INTERACTIVE_MAX_DATA_LEN)
		referee_info.ReceiveData.data_len = REFEREE_INTERACTIVE_MAX_DATA_LEN;
	memcpy(referee_info.ReceiveData.data, data + LEN_student_interactive_head, referee_info.ReceiveData.data_len);
}

static void RefereeParseFrame(uint16_t cmd_id, const uint8_t *data, uint16_t data_len)
{
	switch (cmd_id)
	{
	case ID_game_state:
		RefereeCopyData(&referee_info.GameState, data, LEN_game_state, data_len);
		break;
	case ID_game_result:
		RefereeCopyData(&referee_info.GameResult, data, LEN_game_result, data_len);
		break;
	case ID_game_robot_survivors:
		RefereeCopyData(&referee_info.GameRobotHP, data, LEN_game_robot_HP, data_len);
		break;
	case ID_event_data:
		RefereeCopyData(&referee_info.EventData, data, LEN_event_data, data_len);
		break;
	case ID_referee_warning:
		RefereeCopyData(&referee_info.RefereeWarning, data, LEN_referee_warning, data_len);
		break;
	case ID_dart_info:
		RefereeCopyData(&referee_info.DartInfo, data, LEN_dart_info, data_len);
		break;
	case ID_game_robot_state:
		RefereeCopyData(&referee_info.GameRobotState, data, LEN_game_robot_state, data_len);
		break;
	case ID_power_heat_data:
		RefereeCopyData(&referee_info.PowerHeatData, data, LEN_power_heat_data, data_len);
		break;
	case ID_game_robot_pos:
		RefereeCopyData(&referee_info.GameRobotPos, data, LEN_game_robot_pos, data_len);
		break;
	case ID_buff:
		RefereeCopyData(&referee_info.Buff, data, LEN_buff, data_len);
		break;
	case ID_robot_hurt:
		RefereeCopyData(&referee_info.RobotHurt, data, LEN_robot_hurt, data_len);
		break;
	case ID_shoot_data:
		RefereeCopyData(&referee_info.ShootData, data, LEN_shoot_data, data_len);
		break;
	case ID_projectile_allowance:
		RefereeCopyData(&referee_info.ProjectileAllowance, data, LEN_projectile_allowance, data_len);
		break;
	case ID_rfid_status:
		RefereeCopyData(&referee_info.RFIDStatus, data, LEN_rfid_status, data_len);
		break;
	case ID_dart_client_cmd:
		RefereeCopyData(&referee_info.DartClientCmd, data, LEN_dart_client_cmd, data_len);
		break;
	case ID_ground_robot_position:
		RefereeCopyData(&referee_info.GroundRobotPosition, data, LEN_ground_robot_position, data_len);
		break;
	case ID_sentry_info:
		RefereeCopyData(&referee_info.SentryInfo, data, LEN_sentry_info, data_len);
		break;
	case ID_student_interactive:
		if (data_len <= LEN_student_interactive_max)
			RefereeParseInteractiveData(data, data_len);
		break;
	case ID_custom_controller_robot_interaction:
		RefereeCopyData(&referee_info.CustomControllerRobotData, data, LEN_custom_controller_robot_interaction, data_len);
		break;
	case ID_map_command:
		RefereeCopyData(&referee_info.MapCommand, data, LEN_map_command, data_len);
		break;
	case ID_custom_controller_client_interaction:
		RefereeCopyData(&referee_info.CustomControllerClientData, data, LEN_custom_controller_client_interaction, data_len);
		break;
	case ID_map_data:
		RefereeCopyData(&referee_info.MapData, data, LEN_map_data, data_len);
		break;
	case ID_custom_info:
		RefereeCopyData(&referee_info.CustomInfo, data, LEN_custom_info, data_len);
		break;
	case ID_robot_custom_controller_data:
		RefereeCopyData(&referee_info.RobotCustomControllerData, data, LEN_robot_custom_controller_data, data_len);
		break;
	case ID_robot_custom_client_data:
		RefereeCopyData(&referee_info.RobotCustomClientData, data, LEN_robot_custom_client_data, data_len);
		break;
	case ID_custom_client_robot_cmd:
		RefereeCopyData(&referee_info.CustomClientRobotCmd, data, LEN_custom_client_robot_cmd, data_len);
		break;
	default:
		break;
	}
}

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @param  buff_len: 实际收到的数据长度
 */
static void JudgeReadData(uint8_t *buff, uint16_t buff_len)
{
	uint16_t offset = 0;

	if (buff == NULL || buff_len < (LEN_HEADER + LEN_CMDID + LEN_TAIL))
		return;

	while ((offset + LEN_HEADER + LEN_CMDID + LEN_TAIL) <= buff_len)
	{
		uint16_t data_length;
		uint16_t frame_length;
		uint16_t cmd_id;

		if (buff[offset + SOF] != REFEREE_SOF)
		{
			offset++;
			continue;
		}

		if (Verify_CRC8_Check_Sum(buff + offset, LEN_HEADER) != TRUE)
		{
			offset++;
			continue;
		}

		data_length = (uint16_t)buff[offset + DATA_LENGTH] | ((uint16_t)buff[offset + DATA_LENGTH + 1] << 8);
		frame_length = LEN_HEADER + LEN_CMDID + LEN_TAIL + data_length;
		if (frame_length > (uint16_t)(buff_len - offset))
			break;

		if (Verify_CRC16_Check_Sum(buff + offset, frame_length) != TRUE)
		{
			offset++;
			continue;
		}

		memcpy(&referee_info.FrameHeader, buff + offset, LEN_HEADER);
		cmd_id = (uint16_t)buff[offset + CMD_ID_Offset] | ((uint16_t)buff[offset + CMD_ID_Offset + 1] << 8);
		referee_info.CmdID = cmd_id;
		RefereeParseFrame(cmd_id, buff + offset + DATA_Offset, data_length);
		offset += frame_length;
	}
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback()
{
	DaemonReload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff, referee_usart_instance->recv_size);
}
// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USARTServiceInit(referee_usart_instance);
	LOGWARNING("[rm_ref] lost referee data");
}

/* 裁判系统通信初始化 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = RefereeRxCallback;
	conf.usart_handle = referee_usart_handle;
	conf.recv_buff_size = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = DaemonRegister(&daemon_conf);

	return &referee_info;
}

/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
	while (!USARTIsReady(referee_usart_instance))
	{
	}
	USARTSend(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	while (!USARTIsReady(referee_usart_instance))
	{
	}
}

uint8_t RefereeRobotInteractiveSend(referee_id_t *_id, uint16_t receiver_id, uint16_t data_cmd_id, const uint8_t *data, uint16_t data_len)
{
	static uint8_t send_buffer[LEN_HEADER + LEN_CMDID + LEN_student_interactive_max + LEN_TAIL];
	xFrameHeader *frame_header;
	ext_student_interactive_header_data_t *interactive_header;
	uint16_t frame_len;

	if (_id == NULL || data_len > REFEREE_INTERACTIVE_MAX_DATA_LEN || (data == NULL && data_len > 0))
		return 0;

	while (!USARTIsReady(referee_usart_instance))
	{
	}

	frame_header = (xFrameHeader *)send_buffer;
	frame_header->SOF = REFEREE_SOF;
	frame_header->DataLength = Interactive_Data_LEN_Head + data_len;
	frame_header->Seq = UI_Seq;
	frame_header->CRC8 = Get_CRC8_Check_Sum(send_buffer, LEN_CRC8, 0xFF);

	send_buffer[CMD_ID_Offset] = (uint8_t)(ID_student_interactive & 0x00FF);
	send_buffer[CMD_ID_Offset + 1] = (uint8_t)(ID_student_interactive >> 8);

	interactive_header = (ext_student_interactive_header_data_t *)(send_buffer + DATA_Offset);
	interactive_header->data_cmd_id = data_cmd_id;
	interactive_header->sender_ID = _id->Robot_ID;
	interactive_header->receiver_ID = receiver_id;
	if (data_len > 0)
		memcpy(send_buffer + DATA_Offset + Interactive_Data_LEN_Head, data, data_len);

	frame_len = LEN_HEADER + LEN_CMDID + frame_header->DataLength + LEN_TAIL;
	Append_CRC16_Check_Sum(send_buffer, frame_len);
	RefereeSend(send_buffer, frame_len);
	UI_Seq++;
	return 1;
}
