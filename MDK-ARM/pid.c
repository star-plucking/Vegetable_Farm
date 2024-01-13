#include "pid.h"


extern TIM_HandleTypeDef htim3;



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim3)
	{
	target_Speed[0]= (float)dataBuffer.Channel_1/33*16;
	target_Speed[1]= (float)dataBuffer.Channel_2/33*16;
	target_Angle[0]= (float)dataBuffer.Channel_1*5+4096;
	uint16_t Current_Speed=0;
		
	for(int i=0;i<FilterLength-1;i++)  //filter
		{
			Current_Speed+=Can_Feedback[0].Last_speed[i]/FilterLength;
		}
		
	PID.ERR=target_Speed[0]-Current_Speed;
	Interal+=PID.ERR;
	PID.OUT = PID.Kp * (float)PID.ERR + PID.Ki * (float)Interal + PID.Kd * (PID.ERR-PID.Last_ERR);
	if(PID.OUT>=25000){
		PID.OUT=25000;
	}
	else if(PID.OUT<=-25000){
		PID.OUT = -25000;
	}
	PID.Last_ERR=PID.ERR;
	CAN_ctrl(PID.OUT,dataBuffer.Channel_2 * MOTOR_RATIO);
	}
}

void Pid_init()
{
	 PID.Kp=Kp1;
	 PID.Ki=Ki1;
	 PID.Kd=Kd1;
}

