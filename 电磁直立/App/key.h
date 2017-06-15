#ifndef __KEY_H
#define __KEY_H	 
#include "common.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//按键输入驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

/*下面的方式是通过直接操作库函数方式读取IO*/
#define KEY0 		PTB17_IN //PE4
#define KEY1 		PTB16_IN	//PE3 
#define KEY2 		PTB20_IN
#define KEY3 	        PTB22_IN
#define KEY4 	        PTB23_IN
#define KEY5            PTB21_IN

#define Bm0 	        PTA17_IN
#define Bm1 	        PTA16_IN
#define Bm2 	        PTA14_IN
#define Bm3 	        PTA15_IN


#define KEY0_PRES 	1
#define KEY1_PRES	2
#define KEY2_PRES	3
#define KEY3_PRES	4
#define KEY4_PRES	5
#define KEY5_PRES	6

#define BM0_ON	0
#define BM0_OFF	1
#define BM1_ON	2
#define BM1_OFF	3
#define BM2_ON	4
#define BM2_OFF	5
#define BM3_ON	6
#define BM3_OFF	7

void KEY_Init(void);	//IO初始化
uint8 KEY_Scan_bm0(void);
uint8 KEY_Scan_bm1(void);
uint8 KEY_Scan_bm2(void);
uint8 KEY_Scan_bm3(void);
uint8 KEY_Scan(uint8);  		//按键扫描函数	

#endif
