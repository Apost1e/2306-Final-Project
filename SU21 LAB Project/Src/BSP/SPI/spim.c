/*****************************************************************************
 @Project	: 
 @File 		: spim.c
 @Details  	: SPI Master hardware layer                    
 @Author	: lcgan
 @Hardware	: 
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  Name     XXXX-XX-XX  		Initial Release
   
******************************************************************************/
#include <Common.h>
#include "math.h"
#include "Hal.h"
#include "spim.h"


/*****************************************************************************
 Define
******************************************************************************/
#define TOTAL_SPI			4U


/*****************************************************************************
 Type definition
******************************************************************************/


/*****************************************************************************
 Global Variables
******************************************************************************/

/*****************************************************************************
 Local Variables
******************************************************************************/
static volatile PSPIM_HANDLE g_pSpiIrqhandles[TOTAL_SPI] = { 0, 0, 0, 0 };


/*****************************************************************************
 Local Helpers
******************************************************************************/
static void SPI_Handler( PSPIM_HANDLE pHandle );


/*****************************************************************************
 Export functions
******************************************************************************/;


/*****************************************************************************
 Implementation
******************************************************************************/
int
SpimInit(
	void 		*pHandle,
	char 		nPort,
	int		 	nSpeed,
	SPIM_CFG	ClkInactive,
	SPIM_CFG	ClkEdge,
	SPIM_CFG	DATA_SIZE )
{
	IRQn_Type		irq;
	SSI0_Type 		*spi;
	int				clock;
    int				scr;
	int				dvsr;
	int				prescale;
	PSPIM_HANDLE 	handle = (PSPIM_HANDLE)pHandle;
	
	ASSERT( 0 != pHandle );
	ASSERT( 0 != nSpeed );
	
	/* Check port */
	switch( nPort )
	{
		case 0:
			spi = SSI0;
			irq = SSI0_IRQn;
		break;
		
		case 1:
			spi = SSI1;
			irq = SSI1_IRQn;
		break;

		case 2:
			spi = SSI2;
			irq = SSI1_IRQn;
		break;

		case 3:
			spi = SSI3;
			irq = SSI1_IRQn;
		break;
		
		default:
			return SPIM_E_INVALID_PORT;
	}
	
	handle->pSpi = spi;
	handle->Irq = irq;
	
	g_pSpiIrqhandles[nPort] = handle;		/* keep the handle for interrupt use */
	
	spi->CR1 &= ~SSI_CR1_SSE;

	
	spi->CR1 &= ~SSI_CR1_MS;		/* Master mode */
	
	/* Set clock polarity */
	switch( ClkInactive )
	{
		case SPI_CLK_INACT_LOW:
			spi->CR0 &= ~SSI_CR0_SPO;
		break;
		
		case SPI_CLK_INACT_HIGH:
			spi->CR0 |= SSI_CR0_SPO;
		break;
		
		default:
			return SPIM_E_INVALID_CLK_POLARITY;
	}
	
	/* Set clock edge */
	switch( ClkEdge )
	{
		case SPI_CLK_RISING_EDGE:
			spi->CR0 &= ~SSI_CR0_SPH;
		break;
		
		case SPI_CLK_FALLING_EDGE:
			spi->CR0 |= SSI_CR0_SPH;
		break;
		
		default:
			return SPIM_E_INVALID_CLK_EDGE;
	}

	/* Speed - Use system core clock */
	spi->CC &= ~SSI_CC_CS_M;
	spi->CC |= SSI_CC_CS_SYSPLL;
	
    clock = SystemCoreClock;

	prescale = clock / nSpeed;
	
	if( prescale < 2 )
	{
		return SPIM_E_INVALID_SPEED;
	}
	
	if( prescale<=254 )
	{
		dvsr = prescale;
		scr = 0;
	}
	else
	{
		dvsr = 254;
		scr = (prescale/254) - 1;
		
		if( scr > 256 )
		{
			return SPIM_E_INVALID_SPEED;
		}
	}
	
	spi->CR0 &= ~SSI_CR0_SCR_M;
	spi->CR0 |= (scr<<SSI_CR0_SCR_S);
	spi->CPSR = dvsr;
	
	spi->CR0 &= ~SSI_CR0_DSS_M;
	spi->CR0 = scr;
	
	handle->bTransferWord = FALSE;
	
	switch( DATA_SIZE )
	{
		case SPI_DATA_SIZE_4:
			spi->CR0 |= SSI_CR0_DSS_4;
			handle->Datasize = 4;
		break;
		
		case SPI_DATA_SIZE_5:
			spi->CR0 |= SSI_CR0_DSS_5;
			handle->Datasize = 5;
		break;
		
		case SPI_DATA_SIZE_6:
			spi->CR0 |= SSI_CR0_DSS_6;
			handle->Datasize = 6;
		break;
		
		case SPI_DATA_SIZE_7:
			spi->CR0 |= SSI_CR0_DSS_7;
			handle->Datasize = 7;
		break;
		
		case SPI_DATA_SIZE_8:
			spi->CR0 |= SSI_CR0_DSS_8;
			handle->Datasize = 8;
		break;
		
		case SPI_DATA_SIZE_9:
			spi->CR0 |= SSI_CR0_DSS_9;
			handle->Datasize = 9;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_10:
			spi->CR0 |= SSI_CR0_DSS_10;
			handle->Datasize = 10;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_11:
			spi->CR0 |= SSI_CR0_DSS_11;
			handle->Datasize = 11;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_12:
			spi->CR0 |= SSI_CR0_DSS_12;
			handle->Datasize = 12;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_13:
			spi->CR0 |= SSI_CR0_DSS_13;
			handle->Datasize = 13;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_14:
			spi->CR0 |= SSI_CR0_DSS_14;
			handle->Datasize = 14;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_15:
			spi->CR0 |= SSI_CR0_DSS_15;
			handle->Datasize = 15;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_16:
			spi->CR0 |= SSI_CR0_DSS_16;
			handle->Datasize = 16;
			handle->bTransferWord = TRUE;
		break;
		
		default:
			return SPIM_E_INVALID_DATASIZE;
	}
	
	spi->CR1 |= SSI_CR1_EOT;
	
	spi->CR1 |= SSI_CR1_SSE;
	
	NVIC_EnableIRQ( irq );
	
	return SPIM_STS_OK;
}


int SpimSetSpeed( void *pHandle, int nSpeed )
{
	int				clock;
    int				scr;
	int				dvsr;
	int				prescale;
    PSPIM_HANDLE 	handle = (PSPIM_HANDLE)pHandle;
    SSI0_Type 		*spi 	= handle->pSpi;

	ASSERT( 0 != pHandle );

	spi->CR1 &= ~SSI_CR1_SSE;
	
	/* Speed - Use system core clock */
	spi->CC &= ~SSI_CC_CS_M;
	spi->CC |= SSI_CC_CS_SYSPLL;
	
    clock = SystemCoreClock;

	prescale = clock / nSpeed;
	
	if( prescale < 2 )
	{
		return SPIM_E_INVALID_SPEED;
	}
	
	if( prescale<=254 )
	{
		dvsr = prescale;
		scr = 0;
	}
	else
	{
		dvsr = 254;
		scr = (prescale/254) - 1;
		
		if( scr > 256 )
		{
			return SPIM_E_INVALID_SPEED;
		}
	}
	
	spi->CR0 &= ~SSI_CR0_SCR_M;
	spi->CR0 |= (scr<<SSI_CR0_SCR_S);
	spi->CPSR = dvsr;

	spi->CR1 |= SSI_CR1_SSE;
	
	return clock;
}


int SpimSetDataSize( void *pHandle, SPIM_CFG DATA_SIZE )
{
	PSPIM_HANDLE 	handle = (PSPIM_HANDLE)pHandle;
	SSI0_Type 		*spi	= handle->pSpi;
	
	spi->CR1 &= ~SSI_CR1_SSE;
	spi->CR0 &= ~SSI_CR0_DSS_M;
	
	handle->bTransferWord = FALSE;
	
	switch( DATA_SIZE )
	{
		case SPI_DATA_SIZE_4:
			spi->CR0 |= SSI_CR0_DSS_4;
			handle->Datasize = 4;
		break;
		
		case SPI_DATA_SIZE_5:
			spi->CR0 |= SSI_CR0_DSS_5;
			handle->Datasize = 5;
		break;
		
		case SPI_DATA_SIZE_6:
			spi->CR0 |= SSI_CR0_DSS_6;
			handle->Datasize = 6;
		break;
		
		case SPI_DATA_SIZE_7:
			spi->CR0 |= SSI_CR0_DSS_7;
			handle->Datasize = 7;
		break;
		
		case SPI_DATA_SIZE_8:
			spi->CR0 |= SSI_CR0_DSS_8;
			handle->Datasize = 8;
		break;
		
		case SPI_DATA_SIZE_9:
			spi->CR0 |= SSI_CR0_DSS_9;
			handle->Datasize = 9;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_10:
			spi->CR0 |= SSI_CR0_DSS_10;
			handle->Datasize = 10;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_11:
			spi->CR0 |= SSI_CR0_DSS_11;
			handle->Datasize = 11;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_12:
			spi->CR0 |= SSI_CR0_DSS_12;
			handle->Datasize = 12;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_13:
			spi->CR0 |= SSI_CR0_DSS_13;
			handle->Datasize = 13;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_14:
			spi->CR0 |= SSI_CR0_DSS_14;
			handle->Datasize = 14;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_15:
			spi->CR0 |= SSI_CR0_DSS_15;
			handle->Datasize = 15;
			handle->bTransferWord = TRUE;
		break;
		
		case SPI_DATA_SIZE_16:
			spi->CR0 |= SSI_CR0_DSS_16;
			handle->Datasize = 16;
			handle->bTransferWord = TRUE;
		break;
		
		default:
			return SPIM_E_INVALID_DATASIZE;
	}
	
	spi->CR1 |= SSI_CR1_SSE;
	
	return SPIM_STS_OK;
}



void
SpimAddCallbackTransferDone(
	void 				*phandle,
	SPIM_CB_TRFR_DONE 	*pfDone )
{
	PSPIM_HANDLE handle = (PSPIM_HANDLE)phandle;	
	
	ASSERT( 0 != phandle );
	ASSERT( 0 != pfDone );	
	
	handle->pfDone = pfDone;
}


void
SpimTransfer(
	void 		*phandle,
	void const 	*pTxBuf,
	void 		*pRxBuf,
	int 		nSize
	)
{
	PSPIM_HANDLE handle 	= (PSPIM_HANDLE)phandle;
	SSI0_Type *spi 	= handle->pSpi;
	
	ASSERT( 0 != phandle );
	ASSERT( 0 != pTxBuf );
	ASSERT( 0 != pRxBuf );
	ASSERT( 0 != nSize );
	
	
	handle->nTxCount	= 0;
	handle->nRxCount	= 0;
	
	/* This driver need to modify if datasize more than 8 bits */
	if( FALSE == handle->bTransferWord )
	{
		handle->pRxBuf 	= (char *)pRxBuf;
		handle->pTxBuf 	= (char *)pTxBuf;
		handle->nSize	= nSize;
		spi->DR = handle->pTxBuf[0];
	}
	else
	{
		handle->pRxWordBuf 	= (uint16_t *)pRxBuf;
		handle->pTxWordBuf 	= (uint16_t *)pTxBuf;
		handle->nSize	= nSize>>1;
		spi->DR = handle->pTxWordBuf[0];
	}

	handle->nTxCount++;
	
	
	spi->IM = SSI_IM_TXIM | SSI_IM_RXIM;	/* Enable interrupts */
	spi->CR1 |= SSI_CR1_SSE;
}



/*****************************************************************************
 Callback functions
******************************************************************************/


/*****************************************************************************
 Local functions
******************************************************************************/
static void SPI_Handler( PSPIM_HANDLE pHandle )
{
	SSI0_Type *spi = pHandle->pSpi;
	int status = spi->SR;
	int isr = spi->RIS;
	uint16_t tmp;
	
	ASSERT( 0 != pHandle );
	
	if( 0 != (status & SSI_SR_RNE) )
	{
		tmp = spi->DR; 
		
		if( FALSE == pHandle->bTransferWord )
		{
			if( 0 != pHandle->pRxBuf )
			{
				pHandle->pRxBuf[pHandle->nRxCount] = tmp;
			}
		}
		else
		{
			if( 0 != pHandle->pRxWordBuf )
			{
				pHandle->pRxWordBuf[pHandle->nRxCount] = tmp;
			}
		}
		
		pHandle->nRxCount++;
		
		if( pHandle->nRxCount >= pHandle->nSize )
		{
			spi->IM &= ~SSI_IM_RXIM;	/* Disable interrupts */
			spi->CR1 &= ~SSI_CR1_SSE;
			
			/* Callback to application */
			if( 0 != pHandle->pfDone )	/* The pointer maybe 0 */
			{
				pHandle->pfDone();
			}
		}
	}

			
	if( (0 != (status & SSI_SR_TNF)) || (0 != (status & SSI_SR_TFE)) )
	{
		if( pHandle->nTxCount >= pHandle->nSize )
		{
			/* Transfer has completed */
			spi->IM &= ~SSI_IM_TXIM;
		}
		else
		{
			if( FALSE == pHandle->bTransferWord )
			{
				/* Write to data register to transfer */
				spi->DR = pHandle->pTxBuf[pHandle->nTxCount];
			}
			else
			{
				/* Write to data register to transfer */
				spi->DR = pHandle->pTxWordBuf[pHandle->nTxCount];
			}
			
			pHandle->nTxCount++;
		}
	}
}



/*****************************************************************************
 Interrupt
******************************************************************************/
void SSI0_Handler( void )
{
	ASSERT( 0 != g_pSpiIrqhandles[0] );
	
	SPI_Handler( g_pSpiIrqhandles[0] );
}


void SSI1_Handler( void )
{
	ASSERT( 0 != g_pSpiIrqhandles[1] );
	
	SPI_Handler( g_pSpiIrqhandles[1] );
}


void SSI2_Handler( void )
{
	ASSERT( 0 != g_pSpiIrqhandles[2] );
	
	SPI_Handler( g_pSpiIrqhandles[2] );
}


void SSI3_Handler( void )
{
	ASSERT( 0 != g_pSpiIrqhandles[3] );
	
	SPI_Handler( g_pSpiIrqhandles[3] );
}
























