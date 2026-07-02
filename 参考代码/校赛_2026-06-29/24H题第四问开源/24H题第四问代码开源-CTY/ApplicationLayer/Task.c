#include "ti_msp_dl_config.h"
#include "Task.h"

void TaskInit(Task_t** Task)
{
	static Task_t task;
	*Task = &task;
	task.TaskFlag = 0;
	task.TaskFlag = 0;
	task.TaskState = 0;
	task.LastTaskState = 0 ;
	task.StTrPreFlag = 0;
	task.AverageSpeed = 0;
	task.Deltayaw = 0 ;
	task.Datumyaw = 0 ;
}
void Task1(Car_t car)
{

}

void Task2(Car_t car)
{

}

void Task3(Car_t car)
{

}

uint16_t ProtectFlag = 0;        //进入巡线状态，防止冲出黑线保护
uint16_t ProtectCount = 0;
uint16_t BeginProtectFlag = 0;		//启动时加速度过快车辆翘头，灰度传感器不准，进行发车启动保护
uint16_t BeginProtectCount = 0;

void Task4StateUpdate(Car_t car)
{
	if(car.GraySensor->GraySensorNoData<=0x20&&car.Tasks->TaskState == 0&&BeginProtectFlag == 0)
		{car.Tasks->TaskState = 1;	ProtectFlag = 1;	ProtectCount = 100;	EncoderLengthClear(car.Motors);BuzzerBee(car.Buzzer);}
		
		
	else if(car.GraySensor->GraySensorNoData>=0x20&&car.Tasks->TaskState == 1&&ProtectFlag == 0)
		{car.Tasks->TaskState = 2;																				EncoderLengthClear(car.Motors);BuzzerBee(car.Buzzer);}
		
		
	else if(car.GraySensor->GraySensorNoData<=0x20&&car.Tasks->TaskState == 2)
		{car.Tasks->TaskState = 3;	ProtectFlag = 1;	ProtectCount = 100;	EncoderLengthClear(car.Motors);BuzzerBee(car.Buzzer);}
		
		
	else if(car.GraySensor->GraySensorNoData>=0x20&&car.Tasks->TaskState == 3&&ProtectFlag == 0)
		{car.Tasks->TaskState = 0;																				EncoderLengthClear(car.Motors);BuzzerBee(car.Buzzer);}
}

void Task4(Car_t car)
{
		Task4StateUpdate(car);
	
//	uart0_send_char(car.Tasks->TaskState);

		if(BeginProtectFlag) {
			BeginProtectCount--;
			car.Tasks->TaskState = 0;									//强制维持状态0
			car.GraySensor->GraySensorNoData = 0xFF;	//强制忽略灰度传感器读数
			if(BeginProtectCount == 0)
				BeginProtectFlag = 0 ;
		}
		
	if(car.Tasks->TaskState == 0){
		if(car.Motors->EncoderLeftFront->X + car.Motors->EncoderRightFront->X <= 2*StaightWayLength)
		{
			car.Tasks->Deltayaw = car.HiPnuc->yaw  +Task34TurnAngle;				
			car.Tasks->AverageSpeed = 130;
		}
		
		if(car.Motors->EncoderLeftFront->X + car.Motors->EncoderRightFront->X >= 2*StaightWayLength)
		{
			car.Tasks->Deltayaw = car.HiPnuc->yaw ;
			car.Tasks->AverageSpeed = 60;
		}
	}
	
	else if(car.Tasks->TaskState == 1&&car.GraySensor->GraySensorNoData==0x00){								//避免出线后遇到黑点，整车失控甩飞，只有高置信度时处理逻辑
		if(car.Motors->EncoderLeftFront->X + car.Motors->EncoderRightFront->X <= 2*BendWayLength)
		{
			car.Tasks->Deltayaw = GraySensorToTurnAngle(car.GraySensor); 	
			car.Tasks->AverageSpeed = 130;
		}
		if(car.Motors->EncoderLeftFront->X + car.Motors->EncoderRightFront->X >= 2*BendWayLength)
		{
			car.Tasks->Deltayaw = GraySensorToTurnAngle(car.GraySensor); 	
			car.Tasks->AverageSpeed = 60;
		}
	}
	
	else if(car.Tasks->TaskState == 2){
		if(car.Motors->EncoderLeftFront->X + car.Motors->EncoderRightFront->X <= 2*StaightWayLength)
		{
			car.Tasks->Deltayaw = car.HiPnuc->yaw  +180-Task34TurnAngle;		
			car.Tasks->AverageSpeed = 130;
		}
		if(car.Motors->EncoderLeftFront->X + car.Motors->EncoderRightFront->X >= 2*StaightWayLength)
		{
			car.Tasks->Deltayaw = car.HiPnuc->yaw  +180;
			car.Tasks->AverageSpeed = 60;
		}
	}
	
	else if(car.Tasks->TaskState == 3&&car.GraySensor->GraySensorNoData==0x00){
		if(car.Motors->EncoderLeftFront->X + car.Motors->EncoderRightFront->X <= 2*BendWayLength)
		{
			car.Tasks->Deltayaw = GraySensorToTurnAngle(car.GraySensor); 	
			car.Tasks->AverageSpeed = 130;
		}
		if(car.Motors->EncoderLeftFront->X + car.Motors->EncoderRightFront->X >= 2*BendWayLength)
		{
			car.Tasks->Deltayaw = GraySensorToTurnAngle(car.GraySensor); 	
			car.Tasks->AverageSpeed = 60;
		}
	}
	
	else
		car.Tasks->Deltayaw = 0 ;
	
	
	if(car.Tasks->Deltayaw >180)  car.Tasks->Deltayaw  -= 360;//输出限幅
	if(car.Tasks->Deltayaw <-180) car.Tasks->Deltayaw  += 360;
	
	
	if(ProtectFlag)		//入弯保护
	{
		ProtectCount--;
		if(ProtectCount == 0)
			ProtectFlag = 0;
		if(car.GraySensor->BinaryData == 0xFF&&car.Tasks->TaskState == 1)//在入弯保护状态下且无灰度传感器数据时，强制转向寻找黑线
			car.Tasks->Deltayaw = -50;//uart0_send_char(0x05);
		if(car.GraySensor->BinaryData == 0xFF&&car.Tasks->TaskState == 3)
			car.Tasks->Deltayaw = 50;//uart0_send_char(0x06);
	}	
	
	
	car.Tasks->LastTaskState = car.Tasks->TaskState;
}

void CarStart(Car_t car)
{
	car.Tasks->CarStartFlag = !car.Tasks->CarStartFlag;
	BeginProtectFlag = 1;
	BeginProtectCount = 100;
	BuzzerBee(car.Buzzer);
}