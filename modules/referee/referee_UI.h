#ifndef REFEREE_UI_H
#define REFEREE_UI_H

#include "stdarg.h"
#include "usart.h"
#include "stdint.h"
#include "referee_def.h"
#pragma pack(1)                           //按1字节对齐

// #define NULL 0
#define __FALSE 100

/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A

typedef struct
{
	xFrameHeader FrameHeader; 
	uint16_t CmdID;
   ext_student_interactive_header_data_t datahead;
   uint8_t Delete_Operate;         //删除操作
   uint8_t Layer; 
   uint16_t frametail;
} UI_delete_t;

typedef struct
{
	xFrameHeader FrameHeader; 
	uint16_t CmdID;
   ext_student_interactive_header_data_t datahead;
   uint16_t frametail;
} UI_GraphReFresh_t;


typedef struct
{
  	xFrameHeader FrameHeader; 
	uint16_t CmdID;
   ext_student_interactive_header_data_t datahead;
   String_Data_t String_Data;
   uint16_t frametail; 
} UI_CharReFresh_t;                  //打印字符串数据


#pragma pack()

void UI_Delete(uint8_t Del_Operate,uint8_t Del_Layer);

void Line_Draw(Graph_Data_t *graph,char graphname[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y);
void Circle_Draw(Graph_Data_t *graph,char graphname[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t Graph_Radius);
void Elliptical_Draw(Graph_Data_t *graph,char graphname[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t end_x,uint32_t end_y);
void Rectangle_Draw(Graph_Data_t *graph,char graphname[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t End_x,uint32_t End_y);
void Arc_Draw(Graph_Data_t *graph,char graphname[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_StartAngle,uint32_t Graph_EndAngle,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,uint32_t end_x,uint32_t end_y);
void Float_Draw(Graph_Data_t *graph,char graphname[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Digit,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,int32_t Graph_Float);
void Integer_Draw(Graph_Data_t *graph,char graphname[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y,int32_t Graph_Integer);
int UI_ReFresh(int cnt,...);


void Char_Draw(String_Data_t *graph,char graphname[3],uint32_t Graph_Operate,uint32_t Graph_Layer,uint32_t Graph_Color,uint32_t Graph_Size,uint32_t Graph_Width,uint32_t Start_x,uint32_t Start_y);
void Char_Write(String_Data_t *graph,char* fmt, ...);
int Char_ReFresh(String_Data_t string_Data);

#endif
