#ifndef __HEADFILE_H
#define __HEADFILE_H
#include "headfile.h"
#include "datatype.h"
#include "main.h"
/*********************************************************/
#include "system.h"

#include "timer.h"
#include "pwm.h"

#include "Uart.h" 
#include "IIC.h" 

#include "pid.h"
#include "hipnuc_dec.h"

#include "MotorsCtrl.h"
#include "oled.h" 
#include "GraySensor.h"
#include "BMI088driver.h"
#include "Encoder.h"
#include "HiPnuc.h"
#include "Buzzer.h"

#include "Task.h"

#define PI 	   3.14159265358979323846l

#define BrushMotor_PID_mode		  PID_POSITION
#define BrushMotor_PID_KP   		10.0f
#define BrushMotor_PID_KI   		0.8f
#define BrushMotor_PID_KD   		5.0f
#define BrushMotor_PID_Maxout   1000.0f
#define BrushMotor_PID_MaxIout  1000.0f

#define SelfTurn_PID_mode 			PID_POSITION
#define SelfTurn_PID_KP   			3.0f
#define SelfTurn_PID_KI   			0.0f
#define SelfTurn_PID_KD   			2.0f
#define SelfTurn_PID_Maxout   	200.0f
#define SelfTurn_PID_MaxIout 	  10.0f

#define EncoderReadingFrequency 200.0f
#define EncoderLines 260.0f
#define TireRadius 3.0f

#define Task34TurnAngle 				50.0f
#define StaightWayLength 				90.0f
#define BendWayLength    				100.0f
#endif
