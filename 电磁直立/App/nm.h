#ifndef __NM_H
#define __NM_H
static void Uart1_Put_Char(unsigned char DataToSend);
static void Recive_Analyse(uint8_t *data_buf,uint8_t num);
void Send_Data(uint8_t *dataToSend , uint8_t length);

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))




#endif