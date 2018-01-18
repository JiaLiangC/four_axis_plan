/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under 
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING 
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT 
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR 
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE 
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software 
* and to discontinue the availability of this software.  By using this software, 
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2013 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_main.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements main function.
* Creation Date: 2015-4-19
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_timer.h"
/* Start user code for include. Do not edit comment generated here */
#include"dmac.h"
#include "r_cg_serial.h"
#include "control.h"
#include "rc.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */

/****************************************
*Receive control data
****************************************/
volatile uint16_t PPMbuf[6];
unsigned int  rc_data[6]={0};//roll pitch thro yaw aux1
  
 uint16_t PPM_Temp;//transport()fun
 uint8_t tbuf[20];//transport()fun
 int16_t ANGZ=0;
 uint8_t ANGZFLAG=0;
 /******************************
 *DMA ReciveData
 ******************************/
 extern unsigned char ARMED;
 volatile unsigned char tflag=1;
  
 volatile  uint8_t DMAC_DataBuff[44];//0xfefea
 volatile uint8_t DataBuff[44];
 volatile float Angle[3]={0};
 float Gyro[3]={0};
 
 /******************************
 * declare timebase
 ******************************/
  uint32_t current_time=0;
  static uint32_t send_time=0;
  static uint32_t rc_time=0;
  static uint32_t ctrl_time=0;
  static uint32_t pose_time=0;
  volatile uint32_t  tickus=0;
/******************************/
extern unsigned int moto1,moto2,moto3,moto4;

 void Delay(uint16_t x);
 void transport(unsigned int x);
 void compute_rc();
 unsigned long micros() ;
 void Data_Send_Senser(void);
 extern void PID_Init(void);
 extern  void Data_Send_Status(void);
 extern void Data_Send_RCData(uint16_t roll,uint16_t pitch,uint16_t thro,uint16_t yaw,uint16_t aux1,uint16_t aux2);
 extern void Data_Send_MotoPWM(uint16_t Moto_PWM_1,uint16_t Moto_PWM_2,uint16_t Moto_PWM_3,uint16_t Moto_PWM_4);
 uint8_t compute_imu1(void);
void filter_Gyr( uint8_t tms);
 void Set_Motor(Motor1,Motor2,Motor3,Motor4)
{
	
		PWM_CH1=Motor1*6.5;	//2 6000+
		PWM_CH2=Motor2*6.5;	//3 6000+
		PWM_CH3=Motor3*6.5;	//4 6000+
		PWM_CH4=Motor4*6.5;
}
void Stop_Motor()
{
	
		PWM_CH1=0;	//2
		PWM_CH2=0;	//3
		PWM_CH3=0;	//4
		PWM_CH4=0;	//1
}

/* End user code. Do not edit comment generated here */
void R_MAIN_UserInit(void);

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    /* Start user code. Do not edit comment generated here */
     R_TAU0_Channel0_Start();//pwm
     
     R_SAU0_Create();//
     R_UART0_Create();//serial
     R_UART0_Start();//
     //PID_Init();//PID
     
     Serial_PID_Init();
     R_TAU0_Channel6_Start();//1us timer high
     R_TAU0_Channel5_Start();//PPM
     DMA1_USART0_RCV(DMAC_DataBuff,44); //R_DMAC1_Start();
     while (1U)
     {        
	if(current_time>rc_time)
	{     		
	rc_time=current_time+20000;//20ms
	 compute_rc();//PPM	
	}
	//current_time=micros();
	if(current_time>send_time)
	{	
	send_time=current_time+30000;//30ms	
	if(ARMED == 1)
			{
				Is_DisArmed(rc_data[2],rc_data[3]);
			}
			else
			{
				Is_Armed(rc_data[2],rc_data[3]);
			}
	/*Data_Send_Status();
	Delay(3);
	Data_Send_RCData(rc_data[0],rc_data[1],rc_data[2],rc_data[3],0,0);
	Delay(3);
        Data_Send_MotoPWM(moto1-1000,moto2-1000,moto3-1000,moto4-1000);
	Delay(3);
	Data_Send_Senser();*/
	}
	
	
	if(current_time>ctrl_time)
	{
	 ctrl_time=current_time+4000;//5ms
	 Serial_CONTROL(Angle[0] ,Angle[1] ,-Angle[2],Value_2_Thr(),Value_2_Roll(),Value_2_Pitch(),Vaule_2_Gyro());
	}
	compute_imu1();
	current_time=micros();
     } 
     
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
    EI();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
void transport(unsigned int x)
{
    uint8_t vt;
	PPM_Temp=x;//rc_data[2]*0.0625;//x
      		 for(vt=0;vt<5;vt++)
      		      { 
      			tbuf[4-vt] = (uint8_t)(PPM_Temp%10+48);
      		        PPM_Temp=PPM_Temp/10;
      		      }
		 tbuf[5]=13;
		 tbuf[6]=10;		
		DMA0_USART0_SEND(tbuf,7);
}

void Delay(uint16_t x)
{
uint16_t y;
for(;x>0;x--)
for(y=0;y<1200;y++);

}
unsigned long micros() 
{
    unsigned long m;
    DI();
     m=tickus;
   EI();
    return m ; 
} 

void compute_rc( )
{

	//rc_data[0] = PPMbuf[0]*0.0625+400;
	rc_data[0] = PPMbuf[0]*0.03125+400;
	if(1495<rc_data[0] && rc_data[0]<1505) rc_data[0]=1500;
	
	rc_data[1] = PPMbuf[1]*0.03125+400;
	//rc_data[1] = PPMbuf[1]*0.0625+400;
	if(1495<rc_data[1] && rc_data[1]<1505) rc_data[1]=1500;
	
	rc_data[2] = PPMbuf[2]*0.03125+400;
	//rc_data[2] = PPMbuf[2]*0.0625+400;
	//if(1495<rc_data[2] && rc_data[2]<1505) rc_data[2]=1500;
	
	rc_data[3] = PPMbuf[3]*0.03125+400;
	//rc_data[3] = PPMbuf[3]*0.0625+400;
	if(1495<rc_data[3] && rc_data[3]<1505) rc_data[3]=1500;
}

uint8_t compute_imu1(void)//
{ 
volatile uint8_t i=0;
   static uint32_t time_interval;
  time_interval=micros();
 if((micros()-time_interval)>2000)
     {
   	return 0;
     }
   else
     {
	while((micros()-time_interval)<2000);//read 166 times persecond 166hz
     }
     
     for(i=0;DataBuff[i]!=0x55;i++);
     if(i>33) return 0;
     	 tflag=0;
/*****************************************************************************************
* when flow have not calculate these data,the DMA
*made the another data overlap the old data,so there is a error data sometimes
******************************************************************************************/
     switch(DataBuff[i+1])
		{
		case 0x52:
		  Gyro[0] =((int)DataBuff[i+3] << 8 | DataBuff[i+2])*0.06;
                  Gyro[1] =((int)DataBuff[i+5] << 8 | DataBuff[i+4])*0.06;
                  Gyro[2] =((int)DataBuff[i+7] << 8 | DataBuff[i+6])*0.06;
		    if(-1<Gyro[0] && Gyro[0]<1)Gyro[0]=0;
		    if(-1<Gyro[1] && Gyro[1]<1)Gyro[1]=0;
		    if(-1<Gyro[2] && Gyro[2]<1)Gyro[2]=0;
		break;
		case 0x53:
                  Angle[0] = ((int)DataBuff[i+3]<<8  | DataBuff[i+2])*0.0055+0.4;
                  Angle[1] = ((int)DataBuff[i+5]<<8  | DataBuff[i+4])*0.0055-0.2;
                  Angle[2] = ((int)DataBuff[i+7]<<8  | DataBuff[i+6])*0.0055-ANGZ;
		   if(ANGZFLAG==0){ANGZ=Angle[2];ANGZFLAG=1;}
		   if(-1<Angle[2] && Angle[2]<1)Angle[2]=0;
		break; 
		} 		
		tflag=1;
		
      	return 1;
}
/*void filter_Gyr( uint8_t tms)
{
uint8_t i=0;
static uint16_t FiltBuf1[20],FiltBuf2[20],FiltBuf3[20];
int32_t temp1=0,temp2=0,temp3=0;
static uint8_t filter_cnt=0;

	FiltBuf1[filter_cnt] = GYR[0];
	FiltBuf2[filter_cnt] = GYR[1];
	FiltBuf3[filter_cnt] = GYR[2];
	for(i=0;i<tms;i++)
	{
		temp1 += FiltBuf1[i];
		temp2 += FiltBuf2[i];
		temp3 += FiltBuf3[i];
	}
	Gyro[0] = temp1 / tms;
	Gyro[1] = temp2 / tms;
	Gyro[2] = temp3 / tms;
	filter_cnt++;
	if(filter_cnt==tms)	filter_cnt=0;

}*/
/* End user code. Do not edit comment generated here */
