
#include "common.h"
#include "include.h"
#include "math.h"
#include "l3g4200.h"
#include "filter.h"
#include "oled.h"   
#include "key.h"
#include "IIC.h"
#include "myflash.h" 
    
    
/*!
 *  @brief      main函数
 *  @since      v5.0
 *  @note       山外 ADC 实验
 */

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define NOstoptim 2000
#define NOpotim 1200
///////////////////////与坡道相关变量
int po;
int po_num;
int potim;
int po_tim;
int Po_Ctr_flag=1;
int po_slowtim=350/5;
int po_ctrtim=1200/5;
int po_L_value=1000;
int po_angle=20;
int beepflag;
uint8 outflag=0;
uint8 stoptim;
uint8 finishtim;
uint8 finishflag;
uint32 timevar;     //程序执行时间测试
/////////////////////速度控制相关变量
uint8 V_StopFlag=0;
uint16 V_StopTim=0;
uint8 StopFlag=0;
uint16 StopFlagTim=0;
uint8 buchang=1;
float Buchang_slow;
float Buchang_fast;
float k_fast=0.0005;
float k_slow=0.001;
///////////////////错误与停车
uint8 outlarm=0;
uint8 Error_Flag=0;
uint8 Flag_stop=1;
int key;
//////////////////平衡控制相关
float angle_buchang;
float angle_zero=-10;
float angle_stop;
float Angle_zero=-10;
int bm_key0,bm_key1,bm_key2,bm_key3;
int16 Accel_X,Accel_Y,Accel_Z;
int16 Gyro_X,Gyro_Y,Gyro_Z;
float accel_Y,gyro_Y;
uint16 Accel_X_zero=32208;
int16 L_Error;
float Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int32 Moto1,Moto2;  
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
int16 Angle_Up;
int16 turnbase_up;

uint16  oled_hang;
uint32 Count;
/********************速度控制相关变量********************/
uint8 SpeedCF=0;
float SpeedControlOut_Old,SpeedControlOut;//本次速度 上次速度
uint8 TurnCF=0;
float TurnControlOut_Old,TurnControlOut;
float TurnNow,TurnOld;
float Encoder_D=0;

int16 go_delay;
float move=0;
float Movement=4800;





/******************参数****************/

float J_fAngleerror,J_fAnglepre,J_fAnglenow,J_fAngleDelta;//串级PID参数
float Q_fBouterraceP=735,Q_fBinnerraceP=2.38,Q_fBinnerraceD=1.02,Q_fBinnerraceI=0;//0.09
float Error=0,Error_Last=0,Q_fI=0;

int B_Ctr_flag=1;
float B_P=000,B_D=1.5;

int V_Ctr_flag=1;
float V_P=9,V_D=0,V_I=0.05;
float Slow=0.5;

int T_Ctr_flag=1;
float T_P_BASE=950;
float T_P_FACTOR=1.0;
float T_P,T_D=6900,T_Gyro=1.4;
float T_Zero=1.5;
/*************************************/



extern float K1;
int16 numl[10];
int16 numr[10];
int16 Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int32 Encoder_Sum;
int flagL,flagR,flagM;
          
int32 Inductor[4];
int32 Inductor_sum[4];
int32 min_v[4]={150,23,6,150},max_v[4]={1700,1184,1031,1700};
int16 AD_Reed_Buff[4];

float Turn_bias;


float  p1 =      15.34; 
float  p2 =     -0.0361;
float  p3 =      7.693;
float  p4 =       1.664;

     

float  pl1 =  -291;
float  pl2 =   652.7  ;
float  pl3 =     -505.8  ;
float  pl4 =      105.6  ;




float  pr1 =       197.5 ;
float  pr2 =      -464.5 ;
float  pr3 =       377.8 ;
float  pr4 =      -76.04 ;


float pm1=31.74;
float pm2=0.1602;
     




uint8 tim;


float Inductor_Left,Inductor_Right;
float InductorM;
float InductorM_Left,InductorM_Right;
uint8 data_to_send[30];

int16 myabs(int16 a);

float balance(float Angle,float Gyro);
void velocity(int encoder_left,int encoder_right);
float turn(int encoder_left,int encoder_right,float gyro);
void PWM_Init(void);
void Set_Pwm(int moto1,int moto2);
void readEncoder(void);
void Get_Senser(void);
void Get_Senser_L(void);
void Get_Senser_L_Init(void);
void PIT1_IRQHandler(void);
void Xianfu_Pwm(void);
void oled_show(void);
int16 myabs(int16 a);
void GYRO_SET_OFFSET1(void);
void OLED_ShowSignedNumber(uint8 x,uint8 y,int32 num,uint8 len,uint8 size);
void Error_larm(void);
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int16 myabs(int16 a)
{ 		   
	 if (a<0)
         {
           
           a=-a;
         }
         
             return a;
     
 
}
void Error_larm(void)
{
  Flag_stop=1;
  PTB9_OUT=1;
  key=KEY_Scan (0);
  DELAY_MS(50);
  key=KEY_Scan (0);
  DELAY_MS(50);
  key=KEY_Scan (0);
  DELAY_MS(50);
  key=KEY_Scan (0);
  DELAY_MS(50);
  key=KEY_Scan (0);
  DELAY_MS(500);
  key=KEY_Scan (0);
  PTB9_OUT=0;
  key=KEY_Scan (0);
  DELAY_MS(50);
  key=KEY_Scan (0);
  DELAY_MS(50);
  key=KEY_Scan (0);
  DELAY_MS(50);
  key=KEY_Scan (0);
  DELAY_MS(50);
  key=KEY_Scan (0);
  DELAY_MS(500);
  key=KEY_Scan (0);
}
void PIT1_IRQHandler(void)      //618us
{
  //lptmr_time_start_us(); //开始计时
  
    PIT_Flag_Clear(PIT1);
    Get_Senser();
    
    SpeedCF++;              //速度控制周期标志
    TurnCF++;
    if(B_Ctr_flag==1)
    {
      if(Flag_stop==0)  go_delay++;
      else              go_delay=0;
      if(go_delay>8000) go_delay=8000;
      
      
      if(go_delay==500) angle_zero+=25;
      if((move< Movement)&&(go_delay>500))
      {
        if(angle_zero>Angle_zero)
        {
           angle_zero-=0.0525;
        }
        else angle_zero=Angle_zero;
        
        move+=10;
      }
      else if((move>=Movement)&&(go_delay>500))
      {
        move=Movement;
        angle_zero=Angle_zero;
      }
      else move=0;
      if(B_Ctr_flag==1)
      {
        Balance_Pwm =balance(Angle_Balance,Gyro_Balance); 
      }
      if(SpeedCF==19)
      {
        readEncoder();
        velocity(Encoder_Left,Encoder_Right);       //===速度环PID控制
       
      }
      Get_Senser_L();
      ///////////坡道控制
      if(KEY_Scan_bm2()==4)
      {
        //Po_Ctr_flag=1;
        if((Po_Ctr_flag==1)&&(Inductor[1]>1800)&&(((Angle_Balance+angle_zero)>-16))&&(Inductor[0]<po_L_value)&&(Inductor[0]<po_L_value)&&(myabs(Inductor[0]-Inductor[3])<100))
        {
          po_num+=1;
          if(po_num>10) po=1;
          if(po==1)
          {
          V_Ctr_flag=0;
          angle_zero=-po_angle;
          k_slow=0;
          PTB9_OUT=1;
          }
        }else po_num=0;

        if(po==1)
        {
          po_tim++;
        }

         

        if(po_tim>po_slowtim)
        {
          if(angle_zero<Angle_zero) angle_zero+=0.1;
          else angle_zero=Angle_zero; 
          
        }
        if(po_tim>po_ctrtim)
        {
          po=0;
          po_tim=0;
          PTB9_OUT=0;
          V_Ctr_flag=1;
          k_slow=0.001;
          Movement=3800;
        }
      }else Po_Ctr_flag=0;

      ///////////出跑道停车
      if((Inductor[0]<60)&&(Inductor[3]<60))
      {
        Flag_stop=1;
        outflag=1;
        //Error_Flag=1;//error
      }
      else
      {
        outflag=0;
      }
      if(TurnCF==1)
      {
        
        TurnControlOut=turn(Encoder_Left,Encoder_Right,Gyro_Turn);
      }

      if(V_Ctr_flag==1)
      {
        Velocity_Pwm=((float)SpeedControlOut_Old+(float)(SpeedControlOut-SpeedControlOut_Old)*((float)SpeedCF+1)/20);       //===速度环PID控制
      }
      else
      {
        Velocity_Pwm=0;
      }
      if(T_Ctr_flag==1)
      {
        Turn_Pwm    =((float)TurnControlOut_Old+(float)(TurnControlOut-TurnControlOut_Old)*((float)TurnCF+1)/2); //===转向环PID控制
      }
      else
      {
        Turn_Pwm=0;
      }

        
      Moto1=(int32)(Balance_Pwm+Velocity_Pwm+Turn_Pwm);                 //===计算左轮电机最终PWM
      Moto2=(int32)(Balance_Pwm+Velocity_Pwm-Turn_Pwm);                 //===计算右轮电机最终PWM
      Xianfu_Pwm();
    }
    else
    {
      Moto1=1000;
      Moto2=1000;
    }
    if(SpeedCF==19)  SpeedCF=0;
    if(TurnCF==1) TurnCF=0;
    //if((Inductor[1]<180)&(Inductor[1]<180)) Flag_stop=1;
    if(Flag_stop==0) 
    {
      if(StopFlagTim<NOstoptim)
      {
        StopFlagTim++;
        Po_Ctr_flag=0;
      }
      Set_Pwm(Moto1,Moto2);
    
    }
    else 
    {
      
      if(finishflag==1)
      {
        finishtim++;
      }
      if((finishtim!=0)&&(finishtim<=70))
      {
        Moto1=4000;
        Moto2=4000;
        Set_Pwm(Moto1,Moto2);
        move=0;
      }
      else 
      {
      Moto1=0;
      Moto2=0;
      Set_Pwm(Moto1,Moto2);
      finishtim=0;
      finishflag=0;
      move=0;
      }
    }
    
    //timevar = lptmr_time_get_us(); //停止计时，获取计时时间

}


void finish(void)
{
  if(PTE26_IN==0)
  {
    DELAY_MS(3);
    if(PTE26_IN==0)
    {
        finishflag=1;
        Flag_stop=1;
        //PTB9_OUT=1;
     }
  }
}
/////直立PD控制
float balance(float Angle,float Gyro)     //6us
{
  //lptmr_time_start_us(); //开始计时
  Q_fBinnerraceI=0;
          //static float Sum_Bias;
         float Bias;
	 float balance;
	 Bias=Angle+angle_zero-angle_buchang;              //===求出平衡的角度中值 和机械相关 -0意味着身重中心在0度附近 如果身重中心在5度附近 那就应该减去5
          J_fAngleDelta = Q_fBouterraceP*Bias;// 外环系数P*角度误差

         Error_Last=Error;
         Error=Gyro_X;
    
         balance = (Q_fBinnerraceP*(J_fAngleDelta+Gyro_X)- Q_fBinnerraceD*(Error-Error_Last));//改极性了
 
         
        // timevar = lptmr_time_get_us(); //停止计时，获取计时时间
      
	 return balance;
}

/******************速度PI控制 修改前进后退速度，修改Movement的值************/

void velocity(int encoder_left,int encoder_right)      //5us
{  
	  static float Encoder_Least;
	  static long Encoder_Integral;
          
//lptmr_time_start_us(); //开始计时
   //=============速度PI控制器======================//
          
                SpeedControlOut_Old=SpeedControlOut;
		Encoder_Least =(Encoder_Left+Encoder_Right)/2-(move);  //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此次为零） 	                             //   
                if(Flag_stop==0)
                {
                  Encoder_Sum=Encoder_Sum+(Encoder_Left+Encoder_Right)/2;
                }
                
                 
	        if(Flag_stop==0)        //电机开启的时候才开始积分
		{
                  Encoder_Integral +=Encoder_Least;                     //===积分出位移 积分时间：100ms
		
		}
              //  Encoder_D=(SpeedControlOut-SpeedControlOut_Old);
          	if(Encoder_Integral>8500)  	Encoder_Integral=8500;          
		if(Encoder_Integral<-8500)	Encoder_Integral=-8500;         
               // SpeedControlOut=(Encoder_Least*(-V_P)+Encoder_D*(-V_D)+Encoder_Integral*(-V_I));             //===速度PI控制器	
                SpeedControlOut=(Encoder_Least*(-V_P)+Encoder_Integral*(-V_I))-(((Encoder_Left+Encoder_Right)/2)-Movement+2900)*5;             //===速度PI控制器	
		if(Flag_stop==1)   
                {
                  Encoder_Integral=0;          //===电机关闭后清除积分
                  
                }
	//************超速与慢速  进行角度补偿
        if(buchang==1)
        {
            if(Encoder_Least<-100) Buchang_slow=-(Encoder_Least+100)*k_slow;//慢速补偿
            else Buchang_slow=0;
            if((((Encoder_Left+Encoder_Right)/2)>Movement-2900)&&(go_delay>2000))
            {
              Buchang_fast=-(((Encoder_Left+Encoder_Right)/2)-Movement+2900)*k_fast;//超速补偿
              
            }
            else 
            {
              Buchang_fast=0;
              
            }
            if((Encoder_Least>-2900)&&(Flag_stop==0))
            {
              PTB9_OUT=1;
            }
            else
            {
              PTB9_OUT=0;
            }
            angle_buchang=Buchang_slow+Buchang_fast;
        }
          else
          {
            Buchang_fast=0;
            Buchang_slow=0;
          }
            
          
//timevar = lptmr_time_get_us(); //停止计时，获取计时时间
	
                
}          
                
float turn(int encoder_left,int encoder_right,float gyro)//转向控制    103us
{

          static float Turn;
	  static float Turn_Bias;
          static float Turn_Bias1;
          float Inductor_left,Inductor_right;
          float Turn_Err;
          static float P;
             //lptmr_time_start_us(); //开始计时
                TurnOld=TurnNow;
                  beepflag+=1;
                  if(beepflag==4)
                  {
                    beepflag=0;
                  }
                  if(((Inductor_Left-Inductor_Right)<-0.03)&&(Inductor_Left<0.03))   ///情况3
                 {
                  
                  if(flagR!=1)
                  {
                    flagL=1;  //左出标志
                  }else flagL=0;

                }
                else if(((Inductor_Left-Inductor_Right)>0.03)&&(Inductor_Right<0.03))   ///情况2
                {
                  
                  if(flagL!=1)
                  {
                    flagR=1;  //右出标志
                  }else flagR=0;
                }
                else if((Inductor_Right>0.02)&&(Inductor_Left>0.03))     ///情况1
                {
                  if(Inductor_Left>Inductor_Right) flagM=1;else if(Inductor_Left<Inductor_Right) flagM=2;else flagM=0;    //remember
                  flagL=0;
                  flagR=0;
                }
                else if((Inductor_Left<0.03)&&((Inductor_Left-Inductor_Right)>-0.03))//情况4
                {
                  if(flagM==2) flagL=1;
                }
                else if((Inductor_Right<0.02)&&((Inductor_Left-Inductor_Right)<0.03))//情况5
                {
                  if(flagM==1) flagR=1;
                }
                else
                {
                  Error_Flag=1;//error
                }
                //计算打角
                if((flagR==1)&&(flagL!=1))
                {
                Inductor_left=1.12-Inductor_Left;
                Turn_Bias1=pr1*pow(Inductor_left,3) + pr2*pow(Inductor_left,2) + pr3*Inductor_left + pr4;  //f(x) = pr1*x^3 + pr2*x^2 + pr3*x + pr4
                }
                
                else if((flagL==1)&&(flagR!=1))
                {
                Inductor_right=1.133-Inductor_Right;
                Turn_Bias1=pl1*pow(Inductor_right,3) + pl2*pow(Inductor_right,2) + pl3*Inductor_right + pl4 ;   //     f(x) = pl1*x^3 + pl2*x^2 + pl3*x + pl4
                }
                else if((flagL==0)&&(flagR==0))
                {
                P=(Inductor_Left-Inductor_Right)/(Inductor_Left+Inductor_Right);
                
                Turn_Bias1= p1*pow(P,3) + p2*pow(P,2) + p3*P + p4-T_Zero;          // f(x) = p1*x^3 + p2*x^2 + p3*x + p4
                }
                else
                {
                  Error_Flag=1;//error
                }
                
                if((outflag==0)&&(outlarm==1))
                {
                  if(flagL==1)
                  {
                    if(beepflag>0)
                    {
                      PTB9_OUT=1;
                    }
                    else
                    {
                      PTB9_OUT=0;
                    }
                  }
                  else if(flagR==1)
                  {
                    PTB9_OUT=1;
                  }
                  else
                    PTB9_OUT=0;
                }
                
                /*
                
                Pm=(InductorM_Left-InductorM_Right)/(InductorM_Left+InductorM_Right);
                Turn_Bias2=pm1*Pm + pm2; //f(x) = p1*x + p2
                */
                Turn_Bias=Turn_Bias1;//*1+Turn_Bias2*0.00;

                TurnNow=Turn_Bias;
                  TurnControlOut_Old=Turn;
                  Turn_Err=TurnNow-TurnOld;
                 if((flagL==0)&&(flagR==0))
                 {
                   T_P=(T_P_BASE+pow(Turn_Bias,2)*T_P_FACTOR);   
                 }
                 else
                 {
                   T_P=(T_P_BASE); 
                 }
                  Turn=Turn_Bias*  T_P + Turn_Err*(T_D)+gyro*(-T_Gyro );  
                  
                Turn_bias=Turn_Bias;
                turnbase_up=(int16)(Turn_Bias*100);
          if(Turn>8600) Turn=8600;
          else if(Turn<-8600) Turn=-8600;
        //timevar = lptmr_time_get_us(); //停止计时，获取计时时间
	  return Turn;
}


///赋值给PWM寄存器

void PWM_Init(void)
{
  ftm_pwm_init(FTM0, FTM_CH0,18*1000,0);
  ftm_pwm_init(FTM0, FTM_CH1,18*1000,0);
  ftm_pwm_init(FTM0, FTM_CH2,18*1000,0);
  ftm_pwm_init(FTM0, FTM_CH3,18*1000,0);
  
}
void Set_Pwm(int moto1,int moto2)
{
  
  
  
	if(moto1>0)	
        {
          ftm_pwm_duty(FTM0, FTM_CH0,10000-myabs(moto1));  ftm_pwm_duty(FTM0, FTM_CH1,10000);
        }
	else 	       
        {
          ftm_pwm_duty(FTM0, FTM_CH0,10000);  ftm_pwm_duty(FTM0, FTM_CH1,10000-myabs(moto1));
        }
	if(moto2>0)	
        {
          ftm_pwm_duty(FTM0, FTM_CH2,10000-(myabs(moto2)));  ftm_pwm_duty(FTM0, FTM_CH3,10000);
        }
	else            
        {
          ftm_pwm_duty(FTM0, FTM_CH2,10000);  ftm_pwm_duty(FTM0, FTM_CH3,10000-(myabs(moto2)));
        }
	

	
}
/********************函数功能：读取编码器的数据并进行数据类型转换****************************/
void readEncoder(void)
{
  
    Encoder_Left =  -(ftm_quad_get(FTM1));          //获取FTM 正交解码 的脉冲数(负数表示反方向)
    //if(PTA13_IN==1) Encoder_Left=-Encoder_Left;
    Encoder_Right = (ftm_quad_get(FTM2));          //获取FTM 正交解码 的脉冲数(负数表示反方向)
    //if(PTB19_IN==1) Encoder_Right=-Encoder_Right;
    ftm_quad_clean(FTM1);
    ftm_quad_clean(FTM2);
}

void Xianfu_Pwm(void)
{	
    int Amplitude=9600;    //===PWM满幅是10000 限制在900
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
    if(Moto1>Amplitude)  Moto1=Amplitude;	
    if(Moto2<-Amplitude) Moto2=-Amplitude;	
    if(Moto2>Amplitude)  Moto2=Amplitude;		
	
}



void GYRO_SET_OFFSET1(void)
{
uint8 times=0;
int32 gyro_x=0,gyro_y=0,gyro_z=0;


//陀螺仪零偏校准 -- START --
    for(times=0;times<200;times++)
    {
        
       
        gyro_x += Get_Gyro(1,'X');
        gyro_y += Get_Gyro(1,'Y');
        gyro_z += Get_Gyro(1,'Z');
     }

        GYRO_OFFSET_X = gyro_x/200;
        GYRO_OFFSET_Y = gyro_y/200;
        GYRO_OFFSET_Z = gyro_z/200;
    //陀螺仪零偏校准 -- END --

}


void Get_Senser(void)     //300us
{
    lptmr_time_start_us(); //开始计时
       Accel_X = adc_once (ADC1_SE8, ADC_16bit);
       Accel_X-=Accel_X_zero;
       Accel_X=Accel_X;
      // Accel_Y = adc_once (ADC0_SE9, ADC_16bit);
       //Accel_Y-=35968;
       //Accel_Y= Accel_Y;
       Accel_Z = adc_once (ADC1_SE9, ADC_16bit);
       Accel_Z-=29000;
     
       Gyro_X=Get_Gyro(1,'X')-(-26);//GYRO_OFFSET_X;
       //Gyro_Y=Get_Gyro(1,'Y')-(-13);//GYRO_OFFSET_Y;  //+267.5    读取Y轴陀螺仪
       Gyro_Z=Get_Gyro(1,'Z')-(8);//GYRO_OFFSET_Z;    //读取Z轴陀螺仪
       
       Gyro_Balance=-Gyro_X;

      
  	
	accel_Y = atan2((float)Accel_X,(float)Accel_Z)*57.2957795;//(180/3.14159265);		   			//反正切计算

       gyro_Y=Gyro_X*0.07;//70.0/1000.0;                                   //陀螺仪量程转换
       
       Kalman_Filter(accel_Y,gyro_Y);    //二阶互补滤波
       
       Angle_Balance = angle;      //平衡角度
       Gyro_Turn=Gyro_Z;                                      //更新转向角速度
       
       Angle_Up=(int16)(angle*300);
       
       //Angle_Balance = angle;       
       timevar = lptmr_time_get_us(); //停止计时，获取计时时间
} 
void Get_Senser_L(void)       //93us
{
  //lptmr_time_start_us(); //开始计时
  int i;

  int32 max[3]={0,0,0},min[3]={65535,65535,65535};
   for(i=0;i<12;i++)
   {

            Inductor[0]=adc_once (ADC0_SE13, ADC_16bit);
            Inductor[1]=adc_once (ADC1_SE14, ADC_16bit);
            //Inductor[2]+=adc_once (ADC1_SE14, ADC_16bit);
            Inductor[3]=adc_once (ADC0_SE12, ADC_16bit);
     if(Inductor[0]>max[0]) max[0]=Inductor[0];
     if(Inductor[1]>max[1]) max[1]=Inductor[1];
     if(Inductor[3]>max[2]) max[2]=Inductor[3];
     
     if(Inductor[0]<min[0]) min[0]=Inductor[0];
     if(Inductor[1]<min[1]) min[1]=Inductor[1];
     if(Inductor[3]<min[2]) min[2]=Inductor[3];
            Inductor_sum[0]+=Inductor[0];
            Inductor_sum[1]+=Inductor[1];
            //Inductor[2]+=adc_once (ADC1_SE14, ADC_16bit);
            Inductor_sum[3]+=Inductor[3];
     
   }
   Inductor[0]=(int32)((float)(Inductor_sum[0]-max[0]-min[0])*0.005);
   Inductor[1]=(int32)((float)(Inductor_sum[1]-max[1]-min[1])*0.005);    
   Inductor[3]=(int32)((float)(Inductor_sum[3]-max[2]-min[2])*0.005);  

            Inductor_sum[0]=0;
            Inductor_sum[1]=0;
            //Inductor[2]+=adc_once (ADC1_SE14, ADC_16bit);
            Inductor_sum[3]=0;
   if(bm_key1==2)
   {
   Inductor_Left= ((float)(Inductor[3]-min_v[3]))/((float)(max_v[3]-min_v[3]));
   if(Inductor_Left<=0.0) Inductor_Left=0.00000000001;
   if(Inductor_Left>1.0) Inductor_Left=1.0;
   
   Inductor_Right=((float)(Inductor[0]-min_v[0]))/((float)(max_v[0]-min_v[0]));
   if(Inductor_Right<=0.0) Inductor_Right=0.00000000001;
   if(Inductor_Right>1.0) Inductor_Right=1.0;
   }
   else if(bm_key1==3)
   {
   Inductor_Left= ((float)(Inductor[0]-min_v[0]))/((float)(max_v[0]-min_v[0]));
   if(Inductor_Left<=0.0) Inductor_Left=0.00000000001;
   if(Inductor_Left>1.0) Inductor_Left=1.0;
   
   Inductor_Right=((float)(Inductor[3]-min_v[3]))/((float)(max_v[3]-min_v[3]));
   if(Inductor_Right<=0.0) Inductor_Right=0.00000000001;
   if(Inductor_Right>1.0) Inductor_Right=1.0;
   }
   
   InductorM=Inductor[1];
/*  
   InductorM_Left= ((float)(Inductor[2]-min_v[2]))/((float)(max_v[2]-min_v[2]));
   if(InductorM_Left<=0.0) InductorM_Left=0.00000000001;
   if(InductorM_Left>1.0) InductorM_Left=1.0;
   
   InductorM_Right=((float)(Inductor[1]-min_v[1]))/((float)(max_v[1]-min_v[1]));
   if(InductorM_Right<=0.0) InductorM_Right=0.00000000001;
   if(InductorM_Right>1.0) InductorM_Right=1.0;
   
   */
   //timevar = lptmr_time_get_us(); //停止计时，获取计时时间
   
   
   
}
void Get_Senser_L_Init(void)   //读取归一化最大最小值    持续16s
{
  int i;
  for(i=0;i<8000;i++)
  {
    
    
    AD_Reed_Buff[0]=(int16)((float)adc_once (ADC1_SE15, ADC_16bit)*0.05);
    //AD_Reed_Buff[1]=(int16)((float)adc_once (ADC0_SE16, ADC_16bit)*0.05);
    //AD_Reed_Buff[2]=(int16)((float)adc_once (ADC1_SE14, ADC_16bit)*0.05);
    AD_Reed_Buff[3]=(int16)((float)adc_once (ADC1_SE14, ADC_16bit)*0.05);
    if(AD_Reed_Buff[0]>max_v[0]) max_v[0]=AD_Reed_Buff[0];
    if(AD_Reed_Buff[1]>max_v[1]) max_v[1]=AD_Reed_Buff[1];
    if(AD_Reed_Buff[2]>max_v[2]) max_v[2]=AD_Reed_Buff[2];
    if(AD_Reed_Buff[3]>max_v[3]) max_v[3]=AD_Reed_Buff[3];
    DELAY_MS(2);
  }
  
}

/**************************发数据给上位机*******************************/

void Send_Data(uint8 *dataToSend , uint8 length)
{
	uart_putbuff(UART4,dataToSend,length);
}
void Send_Senser(void )
{
	uint8 i, sum = 0,  _cnt=0;
	

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=14;

	data_to_send[_cnt++]=BYTE1(Accel_X);
	data_to_send[_cnt++]=BYTE0(Accel_X);
        data_to_send[_cnt++]=BYTE1(Accel_Y);
	data_to_send[_cnt++]=BYTE0(Accel_Y);
        data_to_send[_cnt++]=BYTE1(Accel_Z);
	data_to_send[_cnt++]=BYTE0(Accel_Z);
        
	data_to_send[_cnt++]=BYTE1(Gyro_X);
	data_to_send[_cnt++]=BYTE0(Gyro_X);
        data_to_send[_cnt++]=BYTE1(Gyro_Y);
	data_to_send[_cnt++]=BYTE0(Gyro_Y);
        data_to_send[_cnt++]=BYTE1(Gyro_Z);
	data_to_send[_cnt++]=BYTE0(Gyro_Z);
        
        
	data_to_send[_cnt++]=BYTE1(Angle_Up);
	data_to_send[_cnt++]=BYTE0(Angle_Up);
        
        /*
	data_to_send[_cnt++]=BYTE1(HMC5883L.Fifter.Z);
	data_to_send[_cnt++]=BYTE0(HMC5883L.Fifter.Z);
        */
	
        
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
        
}
void Send_UserData(void )
{
	uint8 i, sum = 0,  _cnt=0;
	

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xf1;
	data_to_send[_cnt++]=14;

	/*data_to_send[_cnt++]=(MMA8452_Reedbuf[0]);
	data_to_send[_cnt++]=(MMA8452_Reedbuf[1]);
	data_to_send[_cnt++]=(MMA8452_Reedbuf[2]);
	data_to_send[_cnt++]=(MMA8452_Reedbuf[3]);
	data_to_send[_cnt++]=(MMA8452_Reedbuf[4]);
	data_to_send[_cnt++]=(MMA8452_Reedbuf[5]);
        */
        
	data_to_send[_cnt++]=(L3g4200_Reedbuf[0]);
	data_to_send[_cnt++]=(L3g4200_Reedbuf[1]);
	data_to_send[_cnt++]=(L3g4200_Reedbuf[2]);
	data_to_send[_cnt++]=(L3g4200_Reedbuf[3]);
	data_to_send[_cnt++]=(L3g4200_Reedbuf[4]);
	data_to_send[_cnt++]=(L3g4200_Reedbuf[5]);
        
	//data_to_send[_cnt++]=BYTE3(Angle_Balance);
	//data_to_send[_cnt++]=BYTE2(Angle_Balance);
        data_to_send[_cnt++]=BYTE1(Angle_Balance);
        data_to_send[_cnt++]=BYTE0(Angle_Balance);
        /*
	data_to_send[_cnt++]=BYTE1(HMC5883L.Fifter.Y);
	data_to_send[_cnt++]=BYTE0(HMC5883L.Fifter.Y);
	data_to_send[_cnt++]=BYTE1(HMC5883L.Fifter.Z);
	data_to_send[_cnt++]=BYTE0(HMC5883L.Fifter.Z);
        */
	
        
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
        
}

//显示2个数字
//x,y :起点坐标	 
//len :数字的位数
//size:字体大小
//mode:模式	0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowSignedNumber(uint8 x,uint8 y,int32 num,uint8 len,uint8 size)
{
  if(num<0) 
  {
    OLED_ShowNumber(x,y,(uint32)(-num),len,size);
    OLED_ShowString(x-10,y,"-");
  }
  else 
  {
    OLED_ShowNumber(x,y,(uint32)num,len,size);
    OLED_ShowString(x-10,y,"+");
  }
}

void OLED_MYShowString(void)
{
        OLED_Clear();
        if(oled_hang<60)       
        {
        //OLED_ShowString(120,oled_hang,"*");   //显示选择标志
	OLED_ShowString(00,00,"Move"); 
        OLED_ShowString(00,10,"Po_Cty_flag");   
        OLED_ShowString(00,20,"outlarm");
        OLED_ShowString(00,30,"buchang");
        OLED_ShowString(00,40,"Angle");    //=============显示角度
        
        }
         //第二页
        else if(oled_hang<120)
        {
         // OLED_ShowString(120,(oled_hang-60),"*");   //显示选择标志      
          OLED_ShowString(00,0,"B_Ctr_flag");        //                 60      
          OLED_ShowString(00,10,"BO_P");       
          OLED_ShowString(00,20,"BI_P");        
          OLED_ShowString(00,30,"B_D");         
        }
          
         //第三页
        else if(oled_hang<180)
        {
          //OLED_ShowString(120,(oled_hang-120),"*");   //显示选择标志 
          OLED_ShowString(00,00,"V_Ctr_flag");
          OLED_ShowString(00,10,"V_P");
          OLED_ShowString(00,20,"V_I");
          OLED_ShowString(00,30,"PST");
          OLED_ShowString(00,40,"PCT");
        }
          //第四页
        else if(oled_hang<240)
        {  
          //OLED_ShowString(120,(oled_hang-180),"*");   //显示选择标志     
          OLED_ShowString(00,00,"T_Ctr_flag");        
          OLED_ShowString(00,10,"T_P");     
          OLED_ShowString(00,20,"T_D");       
          OLED_ShowString(00,30,"T_Gyro");     
          OLED_ShowString(00,40,"T_PF"); 
        }
        else if(oled_hang<300)
        {  
          //OLED_ShowString(120,(oled_hang-180),"*");   //显示选择标志     
          OLED_ShowString(00,00,"Angle_Zero");        
          OLED_ShowString(00,10,"po_L_value");     
          OLED_ShowString(00,20,"po_angle");       

        }
          //第五页
        else if(oled_hang<360)
          
        {
          //OLED_ShowString(120,(oled_hang-180),"*");   //显示选择标志     
          OLED_ShowString(00,00,"T_Zero");        
          OLED_ShowString(00,10,"Inductor[0]");     
          OLED_ShowString(00,20,"Inductor[3]");       
          OLED_ShowString(00,30,"Turn_bias");     
          OLED_ShowString(00,40,"Inductor[1]");
        }
        OLED_Refresh_Gram();
        
        

}
void oled_show(void)
{
	Count=0;
        //uint32 Angle_zero;
	//OLED_Display_On();  //??ê??á′ò?a
        //OLED_Clear();
//=============??ê???2¨?÷=======================//
        
        //μúò?ò3      ×′ì???ê?
        if(oled_hang<60)       
        {
        OLED_ShowString(120,oled_hang,"*");   //??ê?????±ê??
        OLED_ShowSignedNumber(85,0, (int32)Movement,4,12);
        OLED_ShowSignedNumber(85,10, Po_Ctr_flag,4,12);
        //OLED_ShowSignedNumber(35,10, (int32)(Turn_bias),4,12);;
        OLED_ShowSignedNumber(85,20,outlarm,4,12); 
       // OLED_ShowSignedNumber(85,20, (uint32)Inductor[2],4,12);
        OLED_ShowSignedNumber(85,30, buchang,4,12);
	OLED_ShowSignedNumber(85,40,(int32)(Angle_Balance+angle_zero),4,12);
        
        }
         //μú?tò3
        else if(oled_hang<120)
        {
          OLED_ShowString(120,(oled_hang-60),"*");   //??ê?????±ê??           //                 60
          OLED_ShowNumber(85,0, (uint32)B_Ctr_flag,4,12); 
          OLED_ShowSignedNumber(85,10, (int32)Q_fBouterraceP,4,12);
          OLED_ShowSignedNumber(85,20, (int32)(Q_fBinnerraceP*10),4,12);
          OLED_ShowSignedNumber(85,30, (int32)(Q_fBinnerraceD*10),4,12);
           OLED_ShowSignedNumber(10,40, (int32)(Encoder_Sum),6,12);
        }
          
         //μúèyò3
        else if(oled_hang<180)
        {
          OLED_ShowString(120,(oled_hang-120),"*");   //??ê?????±ê??
          OLED_ShowNumber(85,00, (uint32)V_Ctr_flag,4,12);
          OLED_ShowSignedNumber(85,10, (int32)(V_P*10),4,12);
          OLED_ShowSignedNumber(85,20, (int32)(V_I*100),4,12);
          OLED_ShowSignedNumber(85,30, (int32)(po_slowtim),4,12);
          OLED_ShowSignedNumber(85,40, (int32)(po_ctrtim),4,12);
        }
          //μú??ò3
        else if(oled_hang<240)
        {  
          OLED_ShowString(120,(oled_hang-180),"*");   //??ê?????±ê??
          OLED_ShowNumber(85,00, (uint32)T_Ctr_flag,4,12);
          OLED_ShowSignedNumber(85,10, (int32)(T_P_BASE),4,12);
          OLED_ShowSignedNumber(85,20, (int32)(T_D),5,12);
          OLED_ShowSignedNumber(85,30, (int32)(T_Gyro*10),4,12);
          OLED_ShowSignedNumber(85,40, (int32)(T_P_FACTOR*10),4,12);
         // OLED_ShowSignedNumber(85,40, (int32)(T_Zero*10),4,12);
          
        }
          //第五页
        else if(oled_hang<300)
        {  
          OLED_ShowString(120,(oled_hang-240),"*");   //显示选择标志
          
          OLED_ShowSignedNumber(85,00, (int32)(Angle_zero),4,12);
          OLED_ShowSignedNumber(85,10, po_L_value,4,12);
          OLED_ShowSignedNumber(85,20, po_angle,4,12);

          
        }
        else if(oled_hang<360)
        {  
          OLED_ShowString(120,(oled_hang-300),"*");   //显示选择标志
          
          OLED_ShowSignedNumber(85,00, (int32)(T_Zero),4,12);
          OLED_ShowSignedNumber(85,10, (uint32)Inductor[0],4,12);
          OLED_ShowSignedNumber(85,20, (uint32)Inductor[3],4,12);
          OLED_ShowSignedNumber(85,30, (int32)Turn_bias,4,12);
          OLED_ShowSignedNumber(85,40, (uint32)Inductor[1],4,12);
          
        }
         OLED_ShowString(00,50,"Write flash");
	OLED_Refresh_Gram();	
}

void main()
{
  /*sto flag init*/
	gpio_init(PTE26,GPI,1);
        port_init_NoALT(PTE26,PULLUP);
        
	gpio_init(PTB9,GPO,0);
        port_init_NoALT(PTB9,PULLUP);        
    PTB9_OUT=1;    
    uart_init (UART4,115200);
    adc_init(ADC1_SE8);
    adc_init(ADC1_SE9);
    
    adc_init(ADC0_SE13);              //ADC初始化
    adc_init(ADC1_SE14); 
    
      adc_init(ADC0_SE12);
    

    flash_init();                                       //初始化flash
    Init_L3G4200D();
    OLED_Init();	                  //=====初始化OLED 模拟SPI 
    PTB9_OUT=0;
    DELAY_MS(200);

    
    OLED_Display_On();  //显示屏打开
    OLED_MYShowString();  
    ftm_quad_init(FTM1);                                    //FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）
    ftm_quad_init(FTM2);                                    //FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）
   // gpio_init(PTA13,GPI,0);
   // gpio_init(PTB19,GPI,0);
    PWM_Init();
    KEY_Init();
    pit_init_ms(PIT1,5);  
    //初始化PIT0，定时时间为： 1000ms
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
                            //使能PIT0中断
   // DELAY_MS(500);
    //GYRO_SET_OFFSET1();
    bm_key0=KEY_Scan_bm0();
    bm_key1=KEY_Scan_bm1();
    bm_key2=KEY_Scan_bm2();
    bm_key3=KEY_Scan_bm3();
    
    if(KEY_Scan_bm0()==0)
    {
      My_flash_read();
    }
    enable_irq(PIT1_IRQn); 
    angle_zero=Angle_zero;
    
    while(1)
    {
      if(Error_Flag==1)
      {
        Error_larm();
      }
      key=KEY_Scan (0);
      if((StopFlagTim>=NOstoptim)&&(KEY_Scan_bm3()==6))
      {
        finish();
        Po_Ctr_flag=1;
      }
      if(key)
      {
              switch(key)
              {

                      case KEY0_PRES :     //??
                              
                              if(oled_hang>340) oled_hang=0;       
                              else oled_hang+=10;
                              OLED_MYShowString();
                              oled_show();
                              break;
                      case KEY1_PRES :     //
                              if(oled_hang==0)         Movement-=50;
                              else if(oled_hang==10)  Po_Ctr_flag=!Po_Ctr_flag;  
                              else if(oled_hang==20)  outlarm=!outlarm;
                              else if(oled_hang==30)  buchang=!buchang;
                              else if(oled_hang==40)  ;
                              else if(oled_hang==50)  My_flash_write();
                              else if(oled_hang==60)  B_Ctr_flag=!B_Ctr_flag;
                              else if(oled_hang==70)  Q_fBouterraceP-=50;
                              else if(oled_hang==80)  Q_fBinnerraceP-=0.1;
                              else if(oled_hang==90)  Q_fBinnerraceD-=0.1;
                              else if(oled_hang==100) ;
                              else if(oled_hang==110) My_flash_write();
                              else if(oled_hang==120) V_Ctr_flag=!V_Ctr_flag;
                              else if(oled_hang==130) V_P-=0.1;
                              else if(oled_hang==140) V_I-=0.01;
                              else if(oled_hang==150) po_slowtim-=1;
                              else if(oled_hang==160) po_ctrtim-=1;
                              else if(oled_hang==170) My_flash_write();
                              else if(oled_hang==180) T_Ctr_flag=!T_Ctr_flag;
                              else if(oled_hang==190) T_P_BASE-=10;
                              else if(oled_hang==200) T_D-=100;
                              else if(oled_hang==210) T_Gyro-=0.1;
                              else if(oled_hang==220) T_P_FACTOR-=0.1;
                              else if(oled_hang==230) My_flash_write();
                              else if(oled_hang==240) Angle_zero-=0.5;
                              else if(oled_hang==250) po_L_value-=50;
                              else if(oled_hang==260) po_angle-=1;
                              else if(oled_hang==270) ;
                              else if(oled_hang==280) ;
                              else if(oled_hang==290) My_flash_write();
                              else if(oled_hang==300) T_Zero-=0.1;
                              else if(oled_hang==310) ;
                              else if(oled_hang==320) ;
                              else if(oled_hang==330) ;
                              else if(oled_hang==340) ;
                              else if(oled_hang==350) My_flash_write();
                              
                              OLED_MYShowString();
                              oled_show();
                              break;

                      case KEY2_PRES :    //     é?
                              
                              if(oled_hang==0) oled_hang=350;
                              else oled_hang-=10;
                              OLED_MYShowString();
                                oled_show();
                              break;
                      case KEY3_PRES :
                              Error_Flag=0;
                              StopFlag=0;
                              StopFlagTim=0;
                              PTA17_OUT=1;
                              finishflag=0;
                              Flag_stop=!Flag_stop;
                              OLED_MYShowString();
                              break;	
                      case KEY4_PRES :    //óò?ü
                         
                              if(oled_hang==0)        Movement+=50.0001;
                              else if(oled_hang==10)  Po_Ctr_flag=!Po_Ctr_flag;  
                              else if(oled_hang==20)  outlarm=!outlarm;
                              else if(oled_hang==30)  buchang=!buchang;
                              else if(oled_hang==40)  ;
                              else if(oled_hang==50)  My_flash_write();
                              else if(oled_hang==60)  B_Ctr_flag=!B_Ctr_flag;
                              else if(oled_hang==70)  Q_fBouterraceP+=5;
                              else if(oled_hang==80)  Q_fBinnerraceP+=0.1;
                              else if(oled_hang==90)  Q_fBinnerraceD+=0.1;
                              else if(oled_hang==100) ;
                              else if(oled_hang==110) My_flash_write();
                              else if(oled_hang==120) V_Ctr_flag=!V_Ctr_flag;
                              else if(oled_hang==130) V_P+=0.1;
                              else if(oled_hang==140) V_I+=0.01;
                              else if(oled_hang==150) po_slowtim+=1;
                              else if(oled_hang==160) po_ctrtim+=1;
                              else if(oled_hang==170) My_flash_write();
                              else if(oled_hang==180) T_Ctr_flag=!T_Ctr_flag;
                              else if(oled_hang==190) T_P_BASE+=10;
                              else if(oled_hang==200) T_D+=100;
                              else if(oled_hang==210) T_Gyro+=0.1;
                              else if(oled_hang==220) T_P_FACTOR+=0.1;
                              else if(oled_hang==230) My_flash_write();
                              else if(oled_hang==240) Angle_zero+=0.5;
                              else if(oled_hang==250) po_L_value+=50;
                              else if(oled_hang==260) po_angle+=1;
                              else if(oled_hang==270) ;
                              else if(oled_hang==280) ;
                              else if(oled_hang==290) My_flash_write();
                              else if(oled_hang==300) T_Zero-=0.1;
                              else if(oled_hang==310) ;
                              else if(oled_hang==320) ;
                              else if(oled_hang==330) ;
                              else if(oled_hang==340) ;
                              else if(oled_hang==350) My_flash_write();
                              OLED_MYShowString();
                              oled_show();
                              break;
              case KEY5_PRES:

                              if(oled_hang>=300) oled_hang-=300;       
                              else oled_hang+=60;
                              OLED_MYShowString(); 
                              oled_show();
              }
	}
		else DELAY_MS(5);
                

                if(Flag_stop==1)
                {
                  oled_show();
                }
        
    }
}





