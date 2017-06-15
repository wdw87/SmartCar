#include "key.h"
#include "include.h" 
#include "filter.h"
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

//按键初始化函数
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

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 
//4，WKUP按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
uint8 KEY_Scan(uint8 mode)
{	 
	static uint8 key_up=1;//按键按松开标志
	if(mode) key_up=1;  //支持连按		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||KEY3==0||KEY4==0||KEY5==0))
	{
		//DELAY_MS(10);//去抖动 。去掉，避免影响直立
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
		else if(KEY2==0)return 3;
                else if(KEY3==0)return 4;
                else if(KEY4==0)return 5;
                else if(KEY5==0)return 6;
              
		
	}else if(KEY0==1&&KEY1==1&&KEY2==1&&KEY3==1&&KEY4==1&&KEY5==1) key_up=1; 	    
 	return 0;// 无按键按下
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




















