/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#pragma interrupt INTDMA0 r_dmac0_interrupt
#pragma interrupt INTDMA1 r_dmac1_interrupt
#include "r_cg_macrodriver.h"
#include "dmac.h"
#include "r_cg_userdefine.h"
#include "r_cg_serial.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern volatile uint8_t DMAC_DataBuff[44];
extern volatile uint8_t DataBuff[44];
extern volatile uint8_t Dat[44];
extern volatile unsigned char tflag;
extern void Delay(uint16_t x);
/***********************************************************************************************************************
* Function Name: R_DMAC0_Create
* Description  : This function initializes the DMA0 transfer.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: R_DMAC0_Start
* Description  : This function enables DMA0 transfer.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void 		DMAC0_Start(void)
{
    DMAIF0 = 0U; /* clear INTDMA0 interrupt flag */
    DMAMK0 = 0U; /* enable INTDMA0 interrupt */
    DEN0 = 1U;
    DST0 = 1U;
}

/***********************************************************************************************************************
* Function Name: R_DMAC0_Stop
* Description  : This function disables DMA0 transfer.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void 		DMAC0_Stop(void)
{
    if (DST0 != 0U)
    {
        DST0 = 0U;
    }
    
    NOP();
    NOP();
    DEN0 = 0U; /* disable DMA0 operation */
    DMAMK0 = 1U; /* disable INTDMA0 interrupt */
    DMAIF0 = 0U; /* clear INTDMA0 interrupt flag */
}


/***********************************************************************************************************************
* Function Name: R_DMAC1_Start
* Description  : This function enables DMA1 transfer.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void DMAC1_Start(void)
{
    DMAIF1 = 0U; /* clear INTDMA1 interrupt flag */
    DMAMK1 = 0U; /* enable INTDMA1 interrupt */
    DEN1 = 1U;
    DST1 = 1U;
}

/***********************************************************************************************************************
* Function Name: R_DMAC1_Stop
* Description  : This function disables DMA1 transfer.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void	DMAC1_Stop(void)
{
    if (DST1 != 0U)
    {
        DST1 = 0U;
    }
    
    NOP();
    NOP();
    DEN1 = 0U; /* disable DMA1 operation */
    DMAMK1 = 1U; /* disable INTDMA1 interrupt */
    DMAIF1 = 0U; /* clear INTDMA1 interrupt flag */
}


void DMA1_Config(uint16_t ct,uint16_t *ADDRESS)
{
 DRC1 = _80_DMA_OPERATION_ENABLE;
    NOP();
    NOP();
    DMAMK1 = 1U; /* disable INTDMA1 interrupt */
    DMAIF1 = 0U; /* clear INTDMA1 interrupt flag */
    /* Set INTDMA1 low priority */
    DMAPR11 = 1U;
    DMAPR01 = 1U;
    DMC1 = _00_DMA_TRANSFER_DIR_SFR2RAM | _00_DMA_DATA_SIZE_8 |_07_DMA_TRIGGER_SR0_CSI01 ;//_00_DMA_TRIGGER_SOFTWARE;
    DSA1 = 0x12;//0x12
    DRA1 =ADDRESS;//0xefea;
    DBC1 =ct;
    DEN1 = 0U; /* disable DMA1 operation */

}


void DMA_Config(uint16_t *RAM_ADDRESS,uint16_t cnt)
{
  DRC0 = _80_DMA_OPERATION_ENABLE;
    NOP();
    NOP();
    DMAMK0 = 1U; //disable INTDMA0 interrupt 
    DMAIF0 = 0U; //clear INTDMA0 interrupt flag 
    // Set INTDMA0 high priority 
    DMAPR10 = 0U;
    DMAPR00 = 0U;		//DMA priority is low
    DMC0 = _40_DMA_TRANSFER_DIR_RAM2SFR | _00_DMA_DATA_SIZE_8 | _06_DMA_TRIGGER_ST0_CSI00;//
    DSA0 = 0x10;	// DMA SFR_ADDRESS TXD0
    DRA0 =RAM_ADDRESS;// 0xefc7;		//DMA Ram address
    DBC0 =cnt;//cnt;// _0020_DMA0_BYTE_COUNT; DMA Buffer size
    DEN0 = 0U; // disable DMA0 operation 
	  	
} 
void DMA1_USART0_RCV(uint8_t   *Buff,uint16_t len)
{
	DMA1_Config(len,Buff);//DMA Channel 0,44
	DMAC1_Start();		//start DMA1 
}
/*******************************************************************************************************************************
*DMA->USART
*
*
********************************************************************************************************************************/
void DMA0_USART0_SEND(uint8_t   *Buff,uint16_t len)
{	
  	DMA_Config(Buff,len);//DMA Channel 0,
	DMAC0_Start();		//start DMA 0
	STG0=1;
	/* Wait for transfers to complete */
}

__interrupt static void r_dmac0_interrupt(void)
{
   
    // Set flag to indicate end of transfers   
    	DMAIF0 = 0; 
    //R_DMAC0_Stop();	//transfer complete	,stop DMA
 
}
__interrupt static void r_dmac1_interrupt(void)
{	
	
    	DMA1_USART0_RCV(DMAC_DataBuff,44);// DMAC1_Start();
	if(tflag==1)
  	{
	memcpy(DataBuff,DMAC_DataBuff,44);
	}
	DMAIF1 = 0; 
}