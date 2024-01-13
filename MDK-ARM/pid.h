#ifndef PID_H_
#define PID_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "remote_rc.h"
#include "stm32f4xx_hal_tim.h"

#define Kp1 30
#define Ki1 0.1
#define Kd1 0

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	int OUT;
	float ERR;
	float Last_ERR;
}PID_t;


static int target_Speed[2]={0,0};
static int target_Angle[2]={4096,4096};
static PID_t PID;
static int Interal=0;

extern dataBuffer_t dataBuffer;
extern CAN_Feedback_t Can_Feedback[2];

void Pid_init();
#endif
