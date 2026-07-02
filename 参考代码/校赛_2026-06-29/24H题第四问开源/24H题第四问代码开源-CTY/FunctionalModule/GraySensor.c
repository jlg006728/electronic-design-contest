#include "ti_msp_dl_config.h"
#include "GraySensor.h"
extern Car_t Car ;


void GraySensorInit(GraySensor_t** GraySensor)
{
	 static GraySensor_t graysensor;
		*GraySensor = &graysensor;
		graysensor.bit0 = 0;
		graysensor.bit1 = 0;
		graysensor.bit2 = 0;
		graysensor.bit3 = 0;
		graysensor.bit4 = 0;
		graysensor.bit5 = 0;
		graysensor.bit6 = 0;
		graysensor.bit7 = 0;
		graysensor.BinaryData = 0xFF;
		graysensor.GraySensorNoData = 0xFF;
}

#define read_gray_bit0   ((GraySensor_PIN_0_PORT->DIN31_0 & GraySensor_PIN_0_PIN ) ? 0x01 : 0x00)
#define read_gray_bit1   ((GraySensor_PIN_1_PORT->DIN31_0 & GraySensor_PIN_1_PIN ) ? 0x01 : 0x00)
#define read_gray_bit2   ((GraySensor_PIN_2_PORT->DIN31_0 & GraySensor_PIN_2_PIN ) ? 0x01 : 0x00)
#define read_gray_bit3   ((GraySensor_PIN_3_PORT->DIN31_0 & GraySensor_PIN_3_PIN ) ? 0x01 : 0x00)
#define read_gray_bit4   ((GraySensor_PIN_4_PORT->DIN31_0 & GraySensor_PIN_4_PIN ) ? 0x01 : 0x00)
#define read_gray_bit5   ((GraySensor_PIN_5_PORT->DIN31_0 & GraySensor_PIN_5_PIN ) ? 0x01 : 0x00)
#define read_gray_bit6   ((GraySensor_PIN_6_PORT->DIN31_0 & GraySensor_PIN_6_PIN ) ? 0x01 : 0x00)
#define read_gray_bit7   ((GraySensor_PIN_7_PORT->DIN31_0 & GraySensor_PIN_7_PIN ) ? 0x01 : 0x00)

void GraySensorDataUpdate (GraySensor_t* GraySensor)
{
	GraySensor->bit0 = read_gray_bit0;
	GraySensor->bit1 = read_gray_bit1;
	GraySensor->bit2 = read_gray_bit2;
	GraySensor->bit3 = read_gray_bit3;
	GraySensor->bit4 = read_gray_bit4;
	GraySensor->bit5 = read_gray_bit5;
	GraySensor->bit6 = read_gray_bit6;
	GraySensor->bit7 = read_gray_bit7;
	
	GraySensor->BinaryData = 0x00 ;
	
	GraySensor->BinaryData |= GraySensor->bit0 << 0 ;
	GraySensor->BinaryData |= GraySensor->bit1 << 1 ;
	GraySensor->BinaryData |= GraySensor->bit2 << 2 ;
	GraySensor->BinaryData |= GraySensor->bit3 << 3 ;
	GraySensor->BinaryData |= GraySensor->bit4 << 4 ;
	GraySensor->BinaryData |= GraySensor->bit5 << 5 ;
	GraySensor->BinaryData |= GraySensor->bit6 << 6 ;
	GraySensor->BinaryData |= GraySensor->bit7 << 7 ;
	
	if((GraySensor->BinaryData == 0xFF||GraySensor->BinaryData == 0x00)&&GraySensor->GraySensorNoData!= 0xFF) GraySensor->GraySensorNoData++;
	else if(GraySensor->GraySensorNoData!= 0x00)GraySensor->GraySensorNoData = GraySensor->GraySensorNoData/2;
}

float GraySensorToTurnAngle(GraySensor_t* GraySensor)
{
//速度50时的参数
//	if(!(GraySensor->BinaryData&0x01)) return +35;
//	else if(!(GraySensor->BinaryData&0x80)) return -35;
//	else if(!(GraySensor->BinaryData&0x02)) return +25;
//	else if(!(GraySensor->BinaryData&0x40)) return -25;
//	else if(!(GraySensor->BinaryData&0x04)) return +15;
//	else if(!(GraySensor->BinaryData&0x20)) return -15;
//	else if((!(GraySensor->BinaryData&0x08))&!(GraySensor->BinaryData&0x10)) return 0;
//	else if(!(GraySensor->BinaryData&0x08)) return +5;
//	else if(!(GraySensor->BinaryData&0x10)) return -5 ;
//	else return 0;
//	
//速度80时的参数
//	if(!(GraySensor->BinaryData&0x01)) return +60;
//	else if(!(GraySensor->BinaryData&0x80)) return -60;
//	else if(!(GraySensor->BinaryData&0x02)) return +40;
//	else if(!(GraySensor->BinaryData&0x40)) return -40;
//	else if(!(GraySensor->BinaryData&0x04)) return +20;
//	else if(!(GraySensor->BinaryData&0x20)) return -20;
//	else if((!(GraySensor->BinaryData&0x08))&!(GraySensor->BinaryData&0x10)) return 0;
//	else if(!(GraySensor->BinaryData&0x08)) return +10;
//	else if(!(GraySensor->BinaryData&0x10)) return -10 ;
//	else return 0;
//速度100时的参数
//	if(!(GraySensor->BinaryData&0x01)) return +90;
//	else if(!(GraySensor->BinaryData&0x80)) return -90;
//	else if(!(GraySensor->BinaryData&0x02)) return +60;
//	else if(!(GraySensor->BinaryData&0x40)) return -60;
//	else if(!(GraySensor->BinaryData&0x04)) return +35;
//	else if(!(GraySensor->BinaryData&0x20)) return -35;
//	else if((!(GraySensor->BinaryData&0x08))&!(GraySensor->BinaryData&0x10)) return 0;
//	else if(!(GraySensor->BinaryData&0x08)) return +15;
//	else if(!(GraySensor->BinaryData&0x10)) return -15 ;
//	else return 0;
//速度130时的参数
	if(!(GraySensor->BinaryData&0x01)) return +40;
	else if(!(GraySensor->BinaryData&0x80)) return -40;
	else if(!(GraySensor->BinaryData&0x02)) return +25;
	else if(!(GraySensor->BinaryData&0x40)) return -25;
	else if(!(GraySensor->BinaryData&0x04)) return +15;
	else if(!(GraySensor->BinaryData&0x20)) return -15;
	else if((!(GraySensor->BinaryData&0x08))&!(GraySensor->BinaryData&0x10)) return 0;
	else if(!(GraySensor->BinaryData&0x08)) return +5;
	else if(!(GraySensor->BinaryData&0x10)) return -5 ;
	else return 0;
}
