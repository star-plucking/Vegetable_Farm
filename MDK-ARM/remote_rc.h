#ifndef REMOTE_RC_H_
#define REMOTE_RC_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "can_config.h"


#define rxBufferLen 18      // 接收的长 
#define BufferLen 18        // 缓冲区长 
#define Starting_num  0
#define MOTOR_RATIO 10


typedef struct // 11+11 + 11 +11 +2 +2+16+16+16+8+8+16
{
  int16_t Channel_1;
  int16_t Channel_2;
  int16_t Channel_3;
  int16_t Channel_4;
  uint8_t Switch_1;
  uint8_t Switch_2;
  uint8_t Mouse_Left;
  uint8_t Mouse_Right;
  uint16_t Mouse_X;
  uint16_t Mouse_Y;
  uint16_t Mouse_Z;
  uint8_t KeyBoard;
} dataBuffer_t;


static uint8_t rxBuffer[rxBufferLen]; // 用于存放接收到的数据
static int dataLen;


inline int CalLast(int i);
int CalNext(int i);
void DecodeData(uint8_t *data,dataBuffer_t*dataBuffer);
#endif