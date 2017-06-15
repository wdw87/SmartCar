#ifndef __KEY_H
#define __KEY_H	 
#include "common.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//����������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

/*����ķ�ʽ��ͨ��ֱ�Ӳ����⺯����ʽ��ȡIO*/
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

void KEY_Init(void);	//IO��ʼ��
uint8 KEY_Scan_bm0(void);
uint8 KEY_Scan_bm1(void);
uint8 KEY_Scan_bm2(void);
uint8 KEY_Scan_bm3(void);
uint8 KEY_Scan(uint8);  		//����ɨ�躯��	

#endif
