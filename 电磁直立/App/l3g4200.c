#include "l3g4200.h"
#include "include.h"


uint8  L3g4200_Reedbuf[6];                        //接收数据缓存区   
int16   GYRO_OFFSET_X ,GYRO_OFFSET_Y ,GYRO_OFFSET_Z ;
void L3g4200_Init(void)
{
   L3G4200_OPEN(400 * 1000); //初始化L3G4200接口，设置波特率 400k
   L3G4200_WR(CTRL_REG1, 0x0f);   //
   L3G4200_WR(CTRL_REG2, 0x00);   //
   L3G4200_WR(CTRL_REG3, 0x08);   //
   L3G4200_WR(CTRL_REG4, 0x30);  //+-2000dps
   L3G4200_WR(CTRL_REG5, 0x00);
}
//////////////////////////////////////
uint8 L3g4200_Reedx_H(void)
{
  L3g4200_Reedbuf[0]=L3G4200_RD(OUT_X_H); //读取X轴数据
  return L3g4200_Reedbuf[0];
}

uint8 L3g4200_Reedx_L(void)
{
  L3g4200_Reedbuf[1]=L3G4200_RD(OUT_X_L); //读取X轴数据
  return L3g4200_Reedbuf[1];
}
////////////////////////////////////
uint8 L3g4200_Reedy_H(void)
{
  L3g4200_Reedbuf[2]=L3G4200_RD(OUT_Y_H); //读取y轴数据
  return L3g4200_Reedbuf[2];
}

uint8 L3g4200_Reedy_L(void)
{
  L3g4200_Reedbuf[3]=L3G4200_RD(OUT_Y_L); //读取y轴数据
  return L3g4200_Reedbuf[3];
}
///////////////////////////////////////////////////
uint8 L3g4200_Reedz_H(void)
{
  L3g4200_Reedbuf[4]=L3G4200_RD(OUT_Z_H); //读取z轴数据
  return L3g4200_Reedbuf[4];
}

uint8 L3g4200_Reedz_L(void)
{
  L3g4200_Reedbuf[5]=L3G4200_RD(OUT_Z_L); //读取z轴数据
  return L3g4200_Reedbuf[5];
}


void GYRO_SET_OFFSET(void)
{
uint8 times=0;
int32 gyro_x=0,gyro_y=0,gyro_z=0;


//陀螺仪零偏校准 -- START --
    for(times=0;times<200;times++)
    {
        
        L3g4200_Reedx_H();
        L3g4200_Reedx_L();
        L3g4200_Reedy_H();
        L3g4200_Reedy_L();
        L3g4200_Reedz_H();
        L3g4200_Reedz_L();
        
        gyro_x += (L3g4200_Reedbuf[0]<<8)+L3g4200_Reedbuf[1];
        gyro_y += (L3g4200_Reedbuf[2]<<8)+L3g4200_Reedbuf[3];
        gyro_z += (L3g4200_Reedbuf[4]<<8)+L3g4200_Reedbuf[5];
     }

        GYRO_OFFSET_X = gyro_x/200;
        GYRO_OFFSET_Y = gyro_y/200;
        GYRO_OFFSET_Z = gyro_z/200;
    //陀螺仪零偏校准 -- END --

}
