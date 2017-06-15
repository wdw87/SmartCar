#include "Myflash.h"
#include "include.h"
#include "common.h"

#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)


void My_flash_init()
{
  flash_init();
}

void My_flash_write()
{ 
   flash_erase_sector(SECTOR_NUM); 

   flash_write(SECTOR_NUM, 0, (uint32)Movement);
     
   flash_write(SECTOR_NUM, 1*4, (uint32)B_Ctr_flag);
     
   flash_write(SECTOR_NUM, 2*4, (uint32)(Q_fBouterraceP*1000));
     
   flash_write(SECTOR_NUM, 3*4, (uint32)(Q_fBinnerraceP*1000));
     
   flash_write(SECTOR_NUM, 4*4, (uint32)(Q_fBinnerraceD*1000)); 
     
   flash_write(SECTOR_NUM, 5*4, (uint32)V_Ctr_flag);    
     
   flash_write(SECTOR_NUM, 6*4, (uint32)(V_P*1000));  
   
   flash_write(SECTOR_NUM, 7*4, (uint32)(V_I*1000));  
   
   flash_write(SECTOR_NUM, 8*4, (uint32)T_Ctr_flag);  
    
   flash_write(SECTOR_NUM, 9*4, (uint32)(T_P_BASE));  
    
   flash_write(SECTOR_NUM, 10*4,(uint32)(T_D*1000));  
   
   flash_write(SECTOR_NUM, 11*4,(uint32)(T_Gyro*1000));  
    
   flash_write(SECTOR_NUM, 12*4, (uint32)(po_slowtim));  
    
   flash_write(SECTOR_NUM, 13*4, (uint32)po_ctrtim);  
    
   //flash_write(SECTOR_NUM, 14*4, (uint32)max_v[2]);  
    
  // flash_write(SECTOR_NUM, 15*4, (uint32)max_v[3]);  
    
   flash_write(SECTOR_NUM, 16*4, (uint32)(T_Zero*1000));
    
   flash_write(SECTOR_NUM, 17*4, (uint32)(T_P_FACTOR*1000));
     
   flash_write(SECTOR_NUM, 18*4, po_angle);
     
   flash_write(SECTOR_NUM, 19*4, po_L_value);
     
   flash_write(SECTOR_NUM, 20*4, (uint32)(-Angle_zero*1000));
   
   flash_write(SECTOR_NUM, 21*4, Po_Ctr_flag);
     

    
}

void My_flash_read(void)
{
 //  My_flash_init();
    
   Movement=flash_read(SECTOR_NUM, 0, int);
   
   B_Ctr_flag=flash_read(SECTOR_NUM, 1*4, int);
    
   Q_fBouterraceP=flash_read(SECTOR_NUM, 2*4, int)/1000.0;
     
   Q_fBinnerraceP=flash_read(SECTOR_NUM, 3*4, int)/1000.0;
      
   Q_fBinnerraceD=flash_read(SECTOR_NUM, 4*4, int)/1000.0;
  
   V_Ctr_flag=flash_read(SECTOR_NUM, 5*4, int);
      
   V_P=flash_read(SECTOR_NUM, 6*4, int)/1000.0;
   
   V_I=flash_read(SECTOR_NUM, 7*4, int)/1000.0; 
     
   T_Ctr_flag=flash_read(SECTOR_NUM, 8*4, int);  
    
   T_P_BASE = flash_read(SECTOR_NUM, 9*4,int);  
    
   T_D = flash_read(SECTOR_NUM, 10*4,int)/1000.0;  
    
   T_Gyro = flash_read(SECTOR_NUM, 11*4,int)/1000.0;  
    
   po_slowtim = flash_read(SECTOR_NUM, 12*4,int);  
    
   po_ctrtim = flash_read(SECTOR_NUM, 13*4,int);  
    
   //po_L_value = flash_read(SECTOR_NUM, 14*4,int);  
    
  // Angle_zero = -flash_read(SECTOR_NUM, 15*4,int)/1000.0;  
   
   T_Zero  =  flash_read(SECTOR_NUM, 16*4,int)/1000.0;  
   
     T_P_FACTOR=flash_read(SECTOR_NUM, 17*4,int)/1000.0; 
    
    po_angle=flash_read(SECTOR_NUM, 18*4,int); 
     
   po_L_value = flash_read(SECTOR_NUM, 19*4,int);  
   
   Angle_zero = -(flash_read(SECTOR_NUM, 20*4,int)/1000.0);  
 
     Po_Ctr_flag=flash_read(SECTOR_NUM, 21*4,int); 

}

//需要用Flash 的参数:
/*
      0             1 
g_nCarSpeed_P,g_nCarSpeed_I
      2              3            4
g_nMotorSpeedSet, casu_weight, js_weight
      5               6
g_nCarRoute_P,g_nCarRoute_D

*/


