#include "ti_msp_dl_config.h"
#include "headfile.h"
Car_t Car;

int main(void)
{
	SYSCFG_DL_init();
	
	UsartInit();
	TimerInit();
	
	TaskInit(&Car.Tasks);
	MotorInit(&Car.Motors);
	GraySensorInit(&Car.GraySensor);
	EncodersInit (Car.Motors);
	HiPnucInit(&Car.HiPnuc);
	BuzzerInit(&Car.Buzzer);
  OLED_Init();
	
//	usart0_send_string("OK");
	
    while(1) ;
}

void SysTick_Handler(void)
{

}
void duty_1000hz(void)
{
    
}

void duty_200hz(void)
{
	HipnucDataUpdate(Car.HiPnuc);
	
	if(Car.Tasks->CarStartFlag)
	{
		EncoderDataUpdate(Car.Motors);
		GraySensorDataUpdate(Car.GraySensor);
		Task4(Car);
		MotorPidCtrl(Car.Motors ,Car.Tasks->Deltayaw ,Car.Tasks->AverageSpeed);
		MotorDataUpdate (Car.Motors);
	}
	
}

void duty_100hz(void)
{
	if(!Car.Tasks->CarStartFlag){
		MotorStop(Car.Motors);
	}
}
void duty_10hz(void)
{
		OledDataUpdate();
		BuzzerDataUpdate(Car.Buzzer);
}
