#include "rc.h"
#include "r_cg_userdefine.h"
//#include "r_cg_macrodriver.h"

unsigned int  CH1_MxMi[2] = {2000,1000};
unsigned int  CH2_MxMi[2] = {2000,1000};
unsigned int  CH3_MxMi[2] = {2000,1000};
unsigned int  CH4_MxMi[2] = {2000,1000};

extern unsigned int rc_data[6];

unsigned int  Get_RightCH_Value(unsigned int  num,unsigned int  max,unsigned int  min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}


unsigned int  Value_2_Thr(void)
{
	return rc_data[2];
}


float Value_2_Roll(void)
{
	int value;
	
	value =-(float)MAX_ANGLE*(float) ((int)rc_data[1]-1500)/500;// (CH3_MxMi[0]+CH3_MxMi[1])/2;
	
	return value;
	
}

float Value_2_Pitch(void)
{
	int value;
	
	value=-(float)MAX_ANGLE*(float)((int)rc_data[0]-1500)/500;
	return value;	

}

int Vaule_2_Gyro(void)
{
	int value;
	value=MAX_GYRO*(((int)rc_data[3])-1500)/500;
	return value;
	
}