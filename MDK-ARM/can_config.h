#ifndef can_config_H_
#define can_config_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"

#define FilterLength 5


typedef struct
{
	short Mechnical_angle;
	short Speed;
	short Torque_current;
	uint8_t Temprature;
	uint16_t Last_speed[FilterLength];
} CAN_Feedback_t;

static CAN_TxHeaderTypeDef CAN_values;
#endif

#define CAN_ID    0x1ff     
#define CAN_DLC   0x08 

static uint8_t can_data_buffer[8];
extern CAN_Feedback_t Can_Feedback[2];

void can_filter_init(void);
void CAN_ctrl(int16_t motor1,int16_t motor2);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

