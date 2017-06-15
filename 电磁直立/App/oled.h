#ifndef __OLED_H
#define __OLED_H			  	 
#include "common.h"
#include "stdlib.h"	    
/**************************************************************************
作者：平衡小车之家 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/

//OLED模式设置
//0:4线串行模式
//1:并行8080模式
#define OLED_MODE 0
		    						  
//-----------------OLED端口定义----------------  
#define OLED_RST_Clr() PTC10_OUT=0
#define OLED_RST_Set() PTC10_OUT=1

#define OLED_RS_Clr() PTC11_OUT=0
#define OLED_RS_Set() PTC11_OUT=1

#define OLED_SCLK_Clr()  PTC8_OUT=0
#define OLED_SCLK_Set()  PTC8_OUT=1

#define OLED_SDIN_Clr()  PTC9_OUT=0
#define OLED_SDIN_Set()  PTC9_OUT=1

#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
//OLED控制用函数

void oled_show(void);
void OLED_WR_Byte(uint8 dat,uint8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   						   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8 x,uint8 y,uint8 t);
void OLED_Fill(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 dot);
void OLED_ShowChar(uint8 x,uint8 y,uint8 chr,uint8 size,uint8 mode);
void OLED_ShowNumber(uint8 x,uint8 y,uint32 num,uint8 len,uint8 size);
void OLED_ShowString(uint8 x,uint8 y,const uint8 *p);	 
#endif  
	 



