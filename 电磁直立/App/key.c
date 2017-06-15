#include "key.h"
#include "include.h" 
#include "filter.h"
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

//������ʼ������
void KEY_Init(void)
{
	gpio_init(PTB16,GPI,0);
        port_init_NoALT(PTB16,PULLUP);
        gpio_init(PTB17,GPI,0); 
        port_init_NoALT(PTB17,PULLUP);
        gpio_init(PTB20,GPI,0);
        port_init_NoALT(PTB20,PULLUP);
        gpio_init(PTB21,GPI,0);
        port_init_NoALT(PTB21,PULLUP);
        gpio_init(PTB22,GPI,0);
        port_init_NoALT(PTB22,PULLUP);
        gpio_init(PTB23,GPI,0);
        port_init_NoALT(PTB23,PULLUP);             
        
        
        gpio_init(PTA16,GPI,0);
        port_init_NoALT(PTA16,PULLUP);       
        gpio_init(PTA17,GPI,0);
        port_init_NoALT(PTA17,PULLUP);  
        gpio_init(PTA15,GPI,0);
        port_init_NoALT(PTA15,PULLUP);       
        gpio_init(PTA14,GPI,0);
        port_init_NoALT(PTA14,PULLUP);
 
}

//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� 
//4��WKUP���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>WK_UP!!
uint8 KEY_Scan(uint8 mode)
{	 
	static uint8 key_up=1;//�������ɿ���־
	if(mode) key_up=1;  //֧������		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||KEY3==0||KEY4==0||KEY5==0))
	{
		//DELAY_MS(10);//ȥ���� ��ȥ��������Ӱ��ֱ��
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(KEY2==0)return 3;
                else if(KEY3==0)return 4;
                else if(KEY4==0)return 5;
                else if(KEY5==0)return 6;
              
		
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1&&KEY5==1) key_up=1; 	    
 	return 0;// �ް�������
}

uint8 KEY_Scan_bm0(void)
{
    if(Bm0==0) return 0;
    else return 1;
}
uint8 KEY_Scan_bm1(void)
{
    if(Bm1==0) return 2;
    else return 3;
}
uint8 KEY_Scan_bm2(void)
{
    if(Bm2==0) return 4;
    else return 5;
}
uint8 KEY_Scan_bm3(void)
{
    if(Bm3==0) return 6;
    else return 7;
}




















