#include "can_config.h"

extern CAN_HandleTypeDef hcan1;

//can_filter_init
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
}

//can_ctrl


void CAN_ctrl(int16_t motor1,int16_t motor2)
{
	uint32_t mail_box;
	CAN_values.StdId=CAN_ID;
	CAN_values.IDE=CAN_ID_STD;
	CAN_values.RTR = CAN_RTR_DATA;
  CAN_values.DLC = CAN_DLC;
	can_data_buffer[0]=motor1>>8;
	can_data_buffer[1]=motor1;
	can_data_buffer[2]=motor2>>8;
	can_data_buffer[3]=motor2;
	can_data_buffer[4]=0;
	can_data_buffer[5]=0;
	can_data_buffer[6]=0;
	can_data_buffer[7]=0;
	HAL_CAN_AddTxMessage(&hcan1, &CAN_values, can_data_buffer, &mail_box); 
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan==&hcan1)
	{
		CAN_RxHeaderTypeDef CAN_rx;
		uint8_t data_rx[8];
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_rx,data_rx);
		if(CAN_rx.StdId==0x205)
		{
			for(int i=FilterLength-1;i>0;i--)
			{
				Can_Feedback[0].Last_speed[i]=Can_Feedback[0].Last_speed[i-1];
			}
			Can_Feedback[0].Mechnical_angle=((uint16_t) data_rx[0] <<8) | (uint16_t)data_rx[1];
			Can_Feedback[0].Speed=((uint16_t) data_rx[2] <<8) | (uint16_t)data_rx[3];
			Can_Feedback[0].Last_speed[0]=Can_Feedback[0].Speed;
			Can_Feedback[0].Torque_current=((uint16_t) data_rx[4] <<8) | (uint16_t)data_rx[5];
			Can_Feedback[0].Temprature=data_rx[6];
		}
		else if (CAN_rx.StdId==0x206)
		{
			Can_Feedback[1].Last_speed[0]=Can_Feedback[1].Speed;
			Can_Feedback[1].Mechnical_angle=((uint16_t) data_rx[0] <<8) | (uint16_t)data_rx[1];
			Can_Feedback[1].Speed=((uint16_t) data_rx[2] <<8) | (uint16_t)data_rx[3];
			Can_Feedback[1].Torque_current=((uint16_t) data_rx[4] <<8) | (uint16_t)data_rx[5];
			Can_Feedback[1].Temprature=data_rx[6];
		}
	}
	return;
}

//can_rxit_callback    fifo0

