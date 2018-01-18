#include "control.h"
#include "math.h"
#include "r_cg_userdefine.h"
#define Gyro_Gain 		2000/32767;
unsigned int moto1,moto2,moto3,moto4;

extern  float Gyro[3];

unsigned char ARMED;

PID PID_ROL={0,0,0,0,0,0,0,0},PID_PIT={0,0,0,0,0,0,0,0},PID_YAW={0,0,0,0,0,0,0,0};
PID  PID_CORE_ROL={0,0,0,0,0,0,0,0},PID_CORE_PIT={0,0,0,0,0,0,0,0},PID_CORE_YAW={0,0,0,0,0,0,0,0};
extern void Stop_Motor();
extern void transport(unsigned int x);
extern  void Set_Motor(Motor1,Motor2,Motor3,Motor4);

void PID_Init(void)
{
	//P
	PID_ROL.P = 13;
	PID_PIT.P =PID_ROL.P;// 4.15;
	PID_YAW.P = 0;//4.15;
	
	//I
	PID_ROL.I = 0;//.01;//0;
	PID_PIT.I =0.008;//0;
	PID_YAW.I = 0;
	
	//IMAX
	PID_ROL.IMAX = 80;
	PID_PIT.IMAX =80;
	PID_YAW.IMAX = 80;

	//D
	PID_ROL.D =2.8;//1.8;
	PID_PIT.D = PID_ROL.D;
	PID_YAW.D = 0;
	
}

void Serial_PID_Init(void)
{
//P
	PID_ROL.P = 4.2;
	PID_PIT.P =PID_ROL.P;// 4.15;
	PID_YAW.P =0;//4.15;
	
	//I
	PID_ROL.I = 0.01;//.008;
	PID_PIT.I =0.01;//0.008;
	PID_YAW.I = 0;
	
	//IMAX
	PID_ROL.IMAX = 10;
	PID_PIT.IMAX =10;
	PID_YAW.IMAX = 0;	
	//D
	PID_ROL.D = 0;//1.8;
	PID_PIT.D = PID_ROL.D;
	PID_YAW.D = 0;
	
	PID_CORE_ROL.P = 3.4;//
	PID_CORE_PIT.P =PID_CORE_ROL.P;// 4.15;
	PID_CORE_YAW.P =3.4;//
	
	//I
	PID_CORE_ROL.I = 0.01;//.008;
	PID_CORE_PIT.I =0.01;//0.008;
	PID_CORE_YAW.I =0.01;
	//IMAX
	PID_CORE_ROL.IMAX = 35;
	PID_CORE_PIT.IMAX =35;
	PID_CORE_YAW.IMAX = 35;

	//D
	PID_CORE_ROL.D =10;//10;
	PID_CORE_PIT.D = PID_CORE_ROL.D;
	PID_CORE_YAW.D =PID_CORE_ROL.D;


}

float Get_MxMi(float num,float max,float min)
{
	if(num>max)
		return max;
	else if(num<min)
		return min;
	else
		return num;
}

void CONTROL(float rol_now, float pit_now, float yaw_now, unsigned int throttle, float rol_tar, float pit_tar, int yaw_gyro_tar)
{       
	
	float rol_error,pit_error,yaw_error,yaw_gyro_now;

	rol_error = rol_now - rol_tar;
	 pit_error = pit_now - pit_tar;	
	 yaw_error = yaw_now - yaw_gyro_tar;
	 
	// transport((unsigned int)rol_error );
	 
	PID_ROL.pout = PID_ROL.P * rol_error;
	PID_PIT.pout = PID_PIT.P * pit_error;
	
	PID_ROL.iout += PID_ROL.I * rol_error;
	PID_ROL.iout = Get_MxMi(PID_ROL.iout,PID_ROL.IMAX,-PID_ROL.IMAX);
	
	PID_PIT.iout += PID_PIT.I * pit_error;
	PID_PIT.iout = Get_MxMi(PID_PIT.iout,PID_PIT.IMAX,-PID_PIT.IMAX);
	
       if(throttle<1100)
	{
		PID_ROL.iout = 0;
		PID_PIT.iout = 0;
	}

	PID_ROL.dout  = PID_ROL.D * (-Gyro[0]);
	PID_PIT.dout 	  =  PID_PIT.D * Gyro[1];
	
	PID_YAW.pout = PID_YAW.P * yaw_error;//P0;//
	PID_YAW.iout  = 0;//I
	PID_YAW.dout =  PID_YAW.D * (yaw_gyro_now-yaw_gyro_tar);//Gyro[2];0;//
		
	PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;
	PID_PIT.OUT   = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
	PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;//0
	
	if(ARMED == 1)
	{
		if(throttle>=1100)
		{
			moto1 = throttle + PID_ROL.OUT -  PID_PIT.OUT + PID_YAW.OUT;  moto1 = Get_MxMi(moto1,2000,1000);//
			moto2 = throttle -  PID_ROL.OUT -  PID_PIT.OUT  - PID_YAW.OUT;  moto2 = Get_MxMi(moto2,2000,1000);
			moto3 = throttle -  PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT;  moto3 = Get_MxMi(moto3,2000,1000);
			moto4 = throttle + PID_ROL.OUT + PID_PIT.OUT -   PID_YAW.OUT;  moto4 = Get_MxMi(moto4,2000,1000);			
			Set_Motor(moto1,moto2,moto3,moto4);
		}
		else
		{
			Stop_Motor();
		}
	}
	else
	{
		//moto1 =0;moto2 =0;moto3 =0;moto4 =0;
		Stop_Motor();
	}
	
}


//change the symbol of rol to minus
void Serial_CONTROL(float rol_now, float pit_now, float yaw_now, unsigned int throttle, float rol_tar, float pit_tar, int yaw_gyro_tar)
{
        //PID_CORE_ROL
	static float rol_core_error,pit_core_error,yaw_core_error;
        static float rol_error,pit_error; 
	static float Pitch_old,Roll_old,Yaw_old;
        static float  Gyro_old_y, Gyro_old_x, Gyro_old_z;
	
  rol_error = rol_now - rol_tar;
  rol_error=Get_MxMi(rol_error,40,-40);
	
  pit_error = pit_now - pit_tar;	
  pit_error=Get_MxMi(pit_error,40,-40);
 ////////////////////////outloop angle loop(PID)///////////////////////////////
  PID_PIT.iout+=pit_error;
//-------------Pitch integral amplitude limit----------------//
  PID_PIT.iout =Get_MxMi(PID_PIT.iout,PID_PIT.IMAX,-PID_PIT.IMAX);
 if(throttle<=1100)
 {
  PID_PIT.iout=0;
 }
//-------------Pitch differential--------------------//
 PID_PIT.dout= pit_now-Pitch_old;
//-------------Pitch  PID-------------------//
  PID_PIT.OUT= PID_PIT.P*pit_error + PID_PIT.I*PID_PIT.iout + PID_PIT.D*PID_PIT.dout;
//------------Pitch history value save------------------//
  Pitch_old= pit_now;
/*********************************************************/




  PID_ROL.iout+=rol_error;
//-------------Roll integral amplitude limit----------------//
  PID_ROL.iout=Get_MxMi(PID_ROL.iout,PID_ROL.IMAX,-PID_ROL.IMAX);
if(throttle<=1100)
 {
  PID_ROL.iout=0;
 }
//-------------Roll differential--------------------//
  PID_ROL.dout=rol_now -Roll_old;
//-------------Roll  PID-------------------//
  PID_ROL.OUT = PID_ROL.P*rol_error + PID_ROL.I*PID_ROL.iout+ PID_ROL.D* PID_ROL.dout;
//------------Roll history value save------------------//
  Roll_old=rol_now;
        
  
////////////////////////coreloop angular loop(PID)//////////////////////////////////////////////////////////////////////      
	
   rol_core_error=PID_ROL.OUT    +Gyro[0];
   pit_core_error=PID_PIT.OUT    +Gyro[1];
   yaw_core_error=0   +Gyro[2];   
	
   rol_core_error=Get_MxMi(rol_core_error,200,-200);
   pit_core_error=Get_MxMi(pit_core_error,200,-200);
   yaw_core_error=Get_MxMi(yaw_core_error,90,-90);
   
   PID_CORE_PIT.pout        =  PID_CORE_PIT.P * pit_core_error;
   PID_CORE_PIT.iout 	   +=	PID_CORE_PIT.I  * pit_core_error;
   /////////////////////////////////// amplitude limit /////////////////////////////////////////////////////////////////////////////
   PID_CORE_PIT.iout =Get_MxMi(PID_CORE_PIT.iout ,PID_CORE_PIT.IMAX,-PID_CORE_PIT.IMAX);
  
   PID_CORE_PIT.dout    =   PID_CORE_PIT.D * (Gyro_old_y+Gyro[1]);
  
   
   PID_CORE_ROL.pout     =  PID_CORE_ROL.P  * rol_core_error;
   PID_CORE_ROL.iout    +=  PID_CORE_ROL.I  * rol_core_error;
   PID_CORE_ROL.iout     =  Get_MxMi(PID_CORE_ROL.iout ,PID_CORE_ROL.IMAX,-PID_CORE_ROL.IMAX);
   
   PID_CORE_ROL.dout     =  PID_CORE_ROL.D  * (Gyro_old_x+Gyro[0]);

   
   PID_CORE_YAW.pout  =   PID_CORE_YAW.P  * yaw_core_error;
   PID_CORE_YAW.iout  +=  PID_CORE_YAW.I * yaw_core_error;
   PID_CORE_YAW.iout  =   Get_MxMi(PID_CORE_YAW.iout ,PID_CORE_YAW.IMAX,-PID_CORE_YAW.IMAX);
   
   PID_CORE_YAW.dout  =   PID_CORE_YAW.D * (Gyro_old_z+Gyro[2]);
        
        
   if(throttle<=1100)
 {
  PID_CORE_ROL.iout =0;
  PID_CORE_YAW.iout =0;
  PID_CORE_PIT.iout =0;
 }
 
  PID_CORE_PIT.OUT = PID_CORE_PIT.pout + PID_CORE_PIT.iout+PID_CORE_PIT.dout;
  PID_CORE_ROL.OUT = PID_CORE_ROL.pout +PID_CORE_ROL.iout +PID_CORE_ROL.dout ;
  PID_CORE_YAW.OUT = PID_CORE_YAW.pout  +PID_CORE_YAW.iout + PID_CORE_YAW.dout;

  Gyro_old_y = 	(-Gyro[1]);
  Gyro_old_x =  (-Gyro[0]); 
  Gyro_old_z =  (-Gyro[2]);   //save history value
        
//--------------------output the value to four motors--------------------------------//
       
        if(ARMED == 1)
        {
    		if(throttle>=1100)
       		{                  
        	moto1=1.06*throttle  -  PID_CORE_ROL.OUT - PID_CORE_PIT.OUT- PID_CORE_YAW.OUT; Get_MxMi(moto1,2000,1000);
        	moto2=throttle  +  PID_CORE_ROL.OUT - PID_CORE_PIT.OUT+ PID_CORE_YAW.OUT;Get_MxMi(moto2,2000,1000);
        	moto3=throttle  +  PID_CORE_ROL.OUT + PID_CORE_PIT.OUT- PID_CORE_YAW.OUT;Get_MxMi(moto3,2000,1000);
        	moto4=throttle  -  PID_CORE_ROL.OUT+ PID_CORE_PIT.OUT+ PID_CORE_YAW.OUT;Get_MxMi(moto4,2000,1000);
		Set_Motor(moto1,moto2,moto3,moto4);                               
    		}
        	else
        	{
			Stop_Motor();            
        	}
 	}
	else
	{
	Stop_Motor();
	}
}
unsigned char Is_Armed(unsigned int CH3,unsigned int CH4)
{
	if( (CH3<1100 && CH3>900) && (CH4<2100 && CH4>1900) )
	{	ARMED = 1;
		return 1;
	}
	else 
	{
		return 0;
	}
}


unsigned char Is_DisArmed(unsigned int CH3,unsigned int CH4)
{
	if( (CH3<1100 && CH3>900) && (CH4<1100 && CH4>900) )
	{
		ARMED = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}