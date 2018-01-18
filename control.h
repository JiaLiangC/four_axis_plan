#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "r_cg_userdefine.h"
//extern u16 moto1,moto2,moto3,moto4;

//∂®“ÂPID
typedef struct
{
	float P;
	float pout;
	
	float I;
	float IMAX;
	float iout;
	
	float D;
	float dout;
	
	float OUT;
}PID;

///extern u8 ARMED;
//extern PID PID_ROL,PID_PIT,PID_YAW;

void PID_Init(void);
void Serial_PID_Init(void);
void CONTROL(float rol_now, float pit_now, float yaw_now, unsigned int throttle, float rol_tar, float pit_tar, int yaw_gyro_tar);
void Serial_CONTROL(float rol_now, float pit_now, float yaw_now, unsigned int throttle, float rol_tar, float pit_tar, int yaw_gyro_tar);
unsigned char Is_Armed(unsigned int CH3,unsigned int CH4);//
unsigned char Is_DisArmed(unsigned int CH3,unsigned int CH4);//


#endif


