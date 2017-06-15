
#ifndef _MYFLASH_H_
#define _MYFLASH_H_
#include "include.h"
#include "common.h"
void My_flash_read();
void My_flash_write();
void My_flash_init();

extern float Movement;
extern float Q_fBouterraceP,Q_fBinnerraceP,Q_fBinnerraceD,Q_fBinnerraceI;
extern int B_Ctr_flag;
extern int V_Ctr_flag;
extern float V_P,V_D,V_I;
extern int T_Ctr_flag;
extern float T_P,T_D,T_Gyro,T_Zero,T_P_FACTOR,T_P_BASE;
extern float Angle_zero;
extern int po_L_value;
extern int po_angle;
extern int po_slowtim;
extern int po_ctrtim;
extern int Po_Ctr_flag;
#endif
