#ifndef __DATATYPE_H
#define __DATATYPE_H

// 标准库头文件包含
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/*------------------------- 类型重定义 -------------------------*/
// 8位整数类型重定义
typedef   signed char 			int8;       // 有符号8位整型
typedef unsigned char 			_u8;        // 无符号8位整型（不同命名风格）
typedef unsigned char 			u8;         // 无符号8位整型
typedef unsigned char 			uint8;      // 无符号8位整型
typedef unsigned char 			byte;       // 字节类型（8位）

// 16位整数类型重定义
typedef   signed short int  int16;       // 有符号16位整型
typedef unsigned short int  uint16;      // 无符号16位整型
typedef unsigned short int  _u16;        // 无符号16位整型（不同命名风格）
typedef unsigned short int  u16;         // 无符号16位整型

// 32位整数类型重定义
typedef unsigned long int 	_u32;        // 无符号32位整型
typedef unsigned long int 	u32;        // 无符号32位整型
typedef float 							fp32;
typedef double 							fp64;

/*------------------------- 常用数学宏 -------------------------*/
#define ABS(X)  (((X)>0)?(X):-(X))         // 绝对值计算
#define MAX(a,b)  ((a)>(b)?(a):(b))        // 最大值
#define MIN(a,b)  ((a)>(b)?(b):(a))        // 最小值

/*------------------------- 对象实例化 -------------------------*/

typedef struct
{
    /* 原始传感器数据 */
    float Accel[3];         //!< 三轴加速度计数据（单位：m/s2），顺序[X,Y,Z]
    float Gyro[3];          //!< 三轴陀螺仪数据（单位：deg/s），顺序[X,Y,Z]
    
    /* 温度相关 */
    float TempWhenCali;     //!< 校准时的环境温度（单位：℃），用于温度补偿参考
    float Temperature;      //!< 当前传感器温度（单位：℃），用于实时补偿
    
    /* 校准参数 */
    float AccelScale;       //!< 加速度计比例因子（无量纲），校准公式：实际值 = 原始值 × AccelScale + AccelOffset
    float GyroOffset[3];    //!< 陀螺仪零偏校准值（单位：deg/s），静止状态下各轴的理论输出
    
    float AccelOffset[3];   //!< 加速度计零偏校准值（单位：m/s2），静止状态下各轴的理论输出
    
    /* 环境参数 */
    float gNorm;            //!< 当地重力加速度标量（单位：m/s2），默认9.80665，用于姿态解算
} BMI088_Data_t;

typedef struct
{
    /* 原始传感器数据 */
    float Accel[3];         //!< 三轴加速度计数据（单位：m/s2），顺序[X,Y,Z]
    float Gyro[3];          //!< 三轴陀螺仪数据（单位：deg/s），顺序[X,Y,Z]
    
    /* 温度相关 */
    float TempWhenCali;     //!< 校准时的环境温度（单位：℃），用于温度补偿参考
    float Temperature;      //!< 当前传感器温度（单位：℃），用于实时补偿
    
    /* 校准参数 */
    float AccelScale;       //!< 加速度计比例因子（无量纲），校准公式：实际值 = 原始值 × AccelScale + AccelOffset
    float GyroOffset[3];    //!< 陀螺仪零偏校准值（单位：deg/s），静止状态下各轴的理论输出
    
    float AccelOffset[3];   //!< 加速度计零偏校准值（单位：m/s2），静止状态下各轴的理论输出
    
    /* 环境参数 */
    float gNorm;            //!< 当地重力加速度标量（单位：m/s2），默认9.80665，用于姿态解算
} MPU6050_Data_t;

typedef struct __attribute__((__packed__)) {
    uint8_t  tag;            ///< 数据包标识 (0x91=有效数据, 其他值需过滤)
    uint16_t sttaus;         ///< 保留字段 
    int8_t   temp;           ///< 板载温度原始值 (℃, 范围: -40~85，精度 ±1℃)
    float    air_pressure;   ///< 气压值 (Pa, 已进行温度补偿)
    uint32_t system_time;    ///< 系统运行时间戳 (ms, 从设备上电开始累计)
    float    acc[3];         ///< 三轴加速度 (m/s2, 已校准，范围 ±16g)
    float    gyr[3];         ///< 三轴角速度 (deg/s, 已校准，范围 ±2000dps)
    float    mag[3];         ///< 三轴磁场强度 (μT, 已硬铁校准)
    float    roll;           ///< 横滚角 (-180° ~ 180°, 绕X轴旋转)
    float    pitch;          ///< 俯仰角 (-90° ~ 90°, 绕Y轴旋转)
    float    yaw;            ///< 航向角 (0° ~ 360°, 绕Z轴旋转，真北参考)
    float    quat[4];        ///< 姿态四元数 [w, x, y, z], 已归一化
} HiPnuc_t;

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
		float YawAngleLast;
		float YawRoundCount;
		
		float v_n;//绝对系沿着水平运动方向的速度
		float x_n;//绝对系沿着水平运动方向的位移
		
		uint8_t ins_flag;
} INS_t;

typedef struct
{
		uint8_t bit0	:1;
		uint8_t bit1	:1;
		uint8_t bit2	:1;
		uint8_t bit3	:1;
		uint8_t bit4	:1;
		uint8_t bit5	:1;
		uint8_t bit6	:1;
		uint8_t bit7	:1;
		uint8_t BinaryData ;
		uint8_t GraySensorNoData ;
}GraySensor_t;

typedef struct
{
	int8_t 	BuzzerFlag;
	int16_t BuzzerCount;
}Buzzer_t;

typedef struct {
    /*---------- 原始整型数据 ----------*/
    int v_int;   ///< 速度原始整型值 		(12位有符号整型，范围: -2048~2047)
    int t_int;   ///< 占空比整型值 			(12位有符号整型，范围: -2048~2047)
    int EncoderCount;///< 编码器中断计数

    /*---------- 转换后浮点数据 ----------*/
    float pos;   ///< 转子位置，单位：弧度 (范围: -π~π，由p_int转换得到)
    float vel;   ///< 转速，单位：rad/s (由v_int转换，比例因子0.01)
    float V;   		///< 对地移速
		float X;   		///< 对地位移
} Encoder_t;

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} Pid_t;


typedef enum {
  Enable   = 1,
  Disable  = 0,   
} Motor_State;

typedef struct
{
    /*---------- 基本信息 ----------*/
    Motor_State state;  ///< 电机状态字 

    /*---------- 控制参数 ----------*/
		int 	ERF;   //EncoderReadingFrequency
		int 	EL ;	 //EncoderLines
		float TR ;	 //TireRadius
	
		int   ExpectOutput;
		int 	Output ;
}BrushMotor_t;

typedef struct
{
		Pid_t*					PidSelfTurn;
	
		BrushMotor_t* 	MotorLeftFront;
		Encoder_t* 			EncoderLeftFront;
		Pid_t*					PidLeftFront;
	
		BrushMotor_t* 	MotorLeftRear; 
		Encoder_t* 			EncoderLeftRear;
		Pid_t*					PidLeftRear;	
	
		BrushMotor_t* 	MotorRightFront; 
		Encoder_t* 			EncoderRightFront;
		Pid_t*					PidRightFront;	
	
		BrushMotor_t* 	MotorRightRear;
		Encoder_t*	 		EncoderRightRear;  
		Pid_t*					PidRightRear;
}Motors_t;

typedef struct
{
	int8_t 	CarStartFlag;
	int16_t TaskFlag;
	int16_t TaskState;
	int16_t LastTaskState;
	float   AverageSpeed;
	float		StTrPreFlag;	//State transition preparation Flag
	float   Deltayaw;
	float   Datumyaw;
}Task_t;


typedef struct {
		BMI088_Data_t* 		BMI088;				//BMI088传感器数据
		MPU6050_Data_t* 	MPU6050;			//MPU6050传感器数据
		HiPnuc_t* 				HiPnuc;				//HiPnuc传感器数据
		INS_t*      			INS;					//惯导数据
		GraySensor_t* 		GraySensor;		//灰度传感器数据
		Buzzer_t*					Buzzer;				//蜂鸣器数据
		Motors_t* 				Motors; 			//电机数据
		Task_t*						Tasks;				//赛题阶段数据
}Car_t;

#endif


