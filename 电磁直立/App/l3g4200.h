#ifndef __L3G4200_H
#define __L3G4200_H
#include "common.h"
//**********L3G4200D内部寄存器地址*********
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define REFERENCE 0x25
#define OUT_TEMP 0x26
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG 0x30
#define INT1_SRC 0x31
#define INT1_TSH_XH 0x32
#define INT1_TSH_XL 0x33
#define INT1_TSH_YH 0x34
#define INT1_TSH_YL 0x35
#define INT1_TSH_ZH 0x36
#define INT1_TSH_ZL 0x37
#define INT1_DURATION 0x38
//****************************************

#define	L3G4200_ADRESS   0x69	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

#define L3G4200_DEVICE I2C1 //定义L3G4200 所用的接口 为 I2C1
//宏定义调用底层的I2C接口 
#define L3G4200_OPEN(baud) i2c_init(L3G4200_DEVICE,baud) 
#define L3G4200_WR(reg,value) i2c_write_reg(L3G4200_DEVICE,L3G4200_ADRESS,reg,value) //L3G4200 写寄存器 
#define L3G4200_RD(reg) i2c_read_reg(L3G4200_DEVICE,L3G4200_ADRESS,reg) //L3G4200 读寄存器

extern uint8  L3g4200_Reedbuf[6];
extern int16   GYRO_OFFSET_X ,GYRO_OFFSET_Y ,GYRO_OFFSET_Z ;
/************************************/
extern uint8 KEY_Scan_bm0(void);
extern uint8 KEY_Scan_bm1(void);
/***********************************/
void L3g4200_Init(void);
uint8 L3g4200_Reedx_H(void);
uint8 L3g4200_Reedx_L(void);
uint8 L3g4200_Reedy_H(void);
uint8 L3g4200_Reedy_L(void);
uint8 L3g4200_Reedz_H(void);
uint8 L3g4200_Reedz_L(void);
void GYRO_SET_OFFSET(void);
#endif