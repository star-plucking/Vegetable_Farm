#include "remote_rc.h"
//--------------------------------------------------------------------------------

//decode数据计算s所使用的函数
inline int CalLast(int i)
{
	if(i==0)
		return 17;
	else 
		return --i;
}

int CalNext(int i)
{
   if(i>17)
		 return i-18;
	 else
		 return i;
}


//dbus协议解码
void DecodeData(uint8_t *data,dataBuffer_t*dataBuffer) // 11+11+11+11+2+2+16+16+16+8+8+16
{
		dataBuffer->Channel_1 = (((uint16_t)data[Starting_num])| (((uint16_t)data[CalNext(Starting_num+1)] << 8) & 0x0700))-1024;
		dataBuffer->Channel_2 = ((((uint16_t)data[CalNext(Starting_num+1)]) >> 3) |(((uint16_t)data[CalNext(Starting_num+2)] & 0x3f)<<5))-1024;
		dataBuffer->Channel_3 = ((((uint16_t)data[CalNext(Starting_num+2)]) >> 6) |((uint16_t)data[CalNext(Starting_num+3)]<<2)  | (((uint16_t)data[Starting_num+4] & 0x01)<<10))-1024;
		dataBuffer->Channel_4 = (((uint16_t)data[CalNext(Starting_num+4)]>>1) | (((uint16_t)data[CalNext(Starting_num+5)]&0x0f)<<7))-1024;
		dataBuffer->Switch_1 = (data[CalNext(Starting_num+5)] & 0xC0)>>6;
		dataBuffer->Switch_2 = (data[CalNext(Starting_num+5)] & 0x30)>>4;
}

//--------------------------------------------------------------------------

//remote it & decodedata


//---------------------------------------------------------------------------

