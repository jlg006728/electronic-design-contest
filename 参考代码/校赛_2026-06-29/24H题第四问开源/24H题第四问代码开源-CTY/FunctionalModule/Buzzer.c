#include "ti_msp_dl_config.h"
#include "Buzzer.h"

void BuzzerInit(Buzzer_t** Buzzer)
{
	static Buzzer_t buzzer;
	*Buzzer = &buzzer;
	buzzer.BuzzerFlag = 0 ;
	buzzer.BuzzerCount = 0 ;
}

void BuzzerDataUpdate(Buzzer_t* Buzzer)
{
	if(Buzzer->BuzzerFlag){
		PWMStart(PWM_Buzzer_INST);
		Buzzer->BuzzerCount--;
		if(Buzzer->BuzzerCount == 0)
			Buzzer->BuzzerFlag = 0;
	}
	else
		PWMStop(PWM_Buzzer_INST);
}

void BuzzerBee(Buzzer_t* Buzzer)
{
	Buzzer->BuzzerCount = 2;
	Buzzer->BuzzerFlag = 1;
}