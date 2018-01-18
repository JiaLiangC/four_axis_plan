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
* File Name    : r_cg_timer_user.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for TAU module.
* Creation Date: 2015-4-19
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt INTTM05 r_tau0_channel5_interrupt
#pragma interrupt INTTM06 r_tau0_channel6_interrupt
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_timer.h"
/* Start user code for include. Do not edit comment generated here */
#include "r_cg_serial.h"
#include"stdio.h"
#include "r_cg_intc.h"
#include "dmac.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* For TAU0_ch5 pulse measurement */
volatile uint32_t g_tau0_ch5_width = 0U;
/* Start user code for global. Do not edit comment generated here */

uint8_t PPM_Channel;
extern volatile uint16_t PPMbuf[6];
uint8_t data_to_send[32];
extern float Gyro[3];
extern volatile uint32_t  tickus;
extern  void transport(void);
void Data_Send_Status(void);
extern float volatile Angle[3];
void Data_Send_MotoPWM(uint16_t Moto_PWM_1,uint16_t Moto_PWM_2,uint16_t Moto_PWM_3,uint16_t Moto_PWM_4);

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_tau0_channel5_interrupt
* Description  : This function is INTTM05 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_tau0_channel5_interrupt(void)
{
    if ((TSR05 & _0001_TAU_OVERFLOW_OCCURS) == 1U)    /* overflow occurs */
    {            
        g_tau0_ch5_width = (uint32_t)(TDR05 + 1U) + 0x10000U;
    }
    else
    {
        g_tau0_ch5_width = (uint32_t)(TDR05 + 1U);
    }

    /* Start user code. Do not edit comment generated here */
    if(g_tau0_ch5_width>65535 || (TSR05 & _0001_TAU_OVERFLOW_OCCURS) == 1U)  //4ms headertime 100000
      {
      PPM_Channel=0;
      }
      
      switch(PPM_Channel)
                {
                        case 1 :PPMbuf[0] =  g_tau0_ch5_width; break;
                        case 2 :PPMbuf[1] =  g_tau0_ch5_width; break;
                        case 3 :PPMbuf[2] =  g_tau0_ch5_width; break;
                        case 4 :PPMbuf[3] =  g_tau0_ch5_width; break;
                        case 5 :PPMbuf[4] =  g_tau0_ch5_width; break;
                        case 6 :PPMbuf[5] =  g_tau0_ch5_width; break;      
                        default: break;
                }
	
       	PPM_Channel ++;   
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_tau0_channel6_interrupt
* Description  : This function is INTTM06 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_tau0_channel6_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
tickus++;
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */

void Data_Send_Status(void)
{
	unsigned char  _cnt=0;
	unsigned char sum = 0;
	int16_t temp;
	unsigned char i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	temp = (int)(Angle[0]*100);//roll
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	temp = (int)(Angle[1]*100);//pit
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	temp = (int)(Angle[2]*100);

	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;

	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0xA1;
	
	data_to_send[3] = _cnt-4;
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	DMA0_USART0_SEND(data_to_send,_cnt);
	//R_UART0_Send(data_to_send,_cnt);
}
void Data_Send_RCData(uint16_t roll,uint16_t pitch,uint16_t thro,uint16_t yaw,uint16_t aux1,uint16_t aux2)
{
	uint8_t  _cnt=0;
	uint16_t temp;
	uint8_t sum = 0;
	uint8_t i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	temp=thro;
	data_to_send[_cnt++]=temp>>8;//BYTE1(Rc_D.THROTTLE);
	data_to_send[_cnt++]=temp&0xff;//BYTE0(Rc_D.THROTTLE);
	temp=yaw;
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	temp=roll;
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	temp=pitch;
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	temp=aux1;
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	temp=aux2;
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	for( i=0;i<_cnt;i++)
	sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	DMA0_USART0_SEND(data_to_send,_cnt);
}
void Data_Send_MotoPWM(uint16_t Moto_PWM_1,uint16_t Moto_PWM_2,uint16_t Moto_PWM_3,uint16_t Moto_PWM_4)
{
	uint8_t _cnt=0;
	uint16_t temp;
	uint8_t sum = 0;
	uint8_t i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	temp=Moto_PWM_1;
	data_to_send[_cnt++]=temp>>8;		//(Moto_PWM_1);
	data_to_send[_cnt++]=temp&0xff;	//(Moto_PWM_1);
	temp=Moto_PWM_2;
	data_to_send[_cnt++]=temp>>8;		//(Moto_PWM_2);
	data_to_send[_cnt++]=temp&0xff;	//(Moto_PWM_2);
	temp=Moto_PWM_3;
	data_to_send[_cnt++]=temp>>8;//(Moto_PWM_3);
	data_to_send[_cnt++]=temp&0xff;//BYTE0(Moto_PWM_3);
	temp=Moto_PWM_4;
	data_to_send[_cnt++]=temp>>8;//(Moto_PWM_4);
	data_to_send[_cnt++]=temp&0xff;//BYTE0(Moto_PWM_4);
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
	
	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	DMA0_USART0_SEND(data_to_send,_cnt);
}

void Data_Send_Senser(void)
{
	unsigned char _cnt=0;
	unsigned char sum = 0;
	unsigned char i;
	int16_t temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;  //¸ß8Î»
	data_to_send[_cnt++]=0;  //µÍ8Î»
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	 temp=(int)Gyro[0];  
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	 temp=(int)Gyro[1];
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	 temp=(int)Gyro[2];
	data_to_send[_cnt++]=temp>>8;
	data_to_send[_cnt++]=temp&0xff;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=0;
	
	data_to_send[3] = _cnt-4;
	
 	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;

	DMA0_USART0_SEND(data_to_send,_cnt);
}

/* End user code. Do not edit comment generated here */
