/*****************************************************************************
 @Project	: SEM2306 Lab Assignment
 @File 		: i2c_sensor.c
 @Details : 
 @Author	: FongFH
 @Hardware: Tiva LaunchPad
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  Fong FH     4 Jun 21  		Initial Release
   
******************************************************************************/

#include <Common.h>
#include "BSP.h"
#include "IRQ.h"
#include "i2c.h"
#include "i2c_sensors.h"
#include <math.h>

static PI2C_HANDLE g_pI2cHandle;
extern volatile BOOL g_bI2C0IsBusy;

/*****************************************************************************
 Local function
*****************************************************************************/
void delay_ms( int ms );

/*****************************************************************************
 Implementation
******************************************************************************/
void AHT10_Init( PI2C_HANDLE pHandle, AHT10 *pAHT10 )
{
	uint8_t data[2];
	
  g_pI2cHandle = pHandle;
	pAHT10->I2C_adr = AHT10_I2C_ADDRESS;
	
	data[0] = AHT10_INIT_CAL_ENABLE;	
	data[1] = AHT10_DATA_NOP;		// 0x00
	I2CWrite( g_pI2cHandle, pAHT10->I2C_adr, AHT10_INIT_CMD, data, 2U);  
  g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);
}

void AHT10Trigger( PI2C_HANDLE pHandle, AHT10 *pAHT10 )
{
	uint8_t data[2];
	
	g_pI2cHandle = pHandle;
	pAHT10->I2C_adr = AHT10_I2C_ADDRESS;
	
	data[0] = AHT10_DATA_MEASURMENT_CMD;
	data[1] = AHT10_DATA_NOP;
	I2CWrite( g_pI2cHandle, pAHT10->I2C_adr, AHT10_START_MEASURMENT_CMD, data, 2U);
	g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);
}

void AHT10ReadRawdata( PI2C_HANDLE pHandle, AHT10 *pAHT10 )
{    
    uint8_t data[6];
    g_pI2cHandle = pHandle;
    pAHT10->I2C_adr = AHT10_I2C_ADDRESS;
    I2CRead( g_pI2cHandle, pAHT10->I2C_adr, 0, data, 6U);
    g_bI2C0IsBusy = TRUE;
    while( TRUE == g_bI2C0IsBusy);
    
    
    uint32_t rawData= ((uint32_t)(data[1] & 0x0F) << 16) | ((uint16_t)data[2] << 8) | data[3]>>4;
    pAHT10->fHumidity = rawData * 0.000095;
		if( pAHT10->fHumidity > 100 ) { pAHT10->fHumidity = 99.9;}
		if( pAHT10->fHumidity < 0 ) { pAHT10->fHumidity = 0;}
    uint32_t temperature = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint16_t)data[4] << 8) | data[5]; 
    pAHT10->fTemperature = temperature * 0.000191 - 50 ;
    
}

void I2CExpander_Init (PI2C_HANDLE pHandle, MCP23017 *pMCP23017)
{
	uint8_t data[2] = {0,0};
	
	g_pI2cHandle = pHandle;
	
	pMCP23017->I2C_addr = MCP23017_ADDR;
	data[0] = 0x00;	// 16-bit mode (BANK = 0), Sequential Operation (SEQOP = 0)
	I2CWrite( g_pI2cHandle, pMCP23017->I2C_addr, MCP23017_IOCONA, data, 1U);  
  g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);
	
	data[0] = 0x00;	// data = 0x00 (port A as output)
	data[1] = 0x00;	// data = 0x00 (port B as output)
	I2CWrite( g_pI2cHandle, MCP23017_ADDR, MCP23017_IODIRA, data, 2U);
  g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);
}


/* function to write to the MCP23017 Output Latch registers    */
/* outputs from the MC23017 will drive the 7-segment displays  */
void write_I2CExpander (PI2C_HANDLE pHandle, MCP23017 *pMCP23017 )
{
	uint8_t data2[2] = {pMCP23017->Digit1, pMCP23017->Digit2};
	I2CWrite( g_pI2cHandle, pMCP23017->I2C_addr, MCP23017_OLATA, data2, 2U);  
	g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);
}



void delay_ms( int ms )
{
	volatile int cnt = (SystemCoreClock/1000)*ms;
	
	while( cnt-- )
	{
		__NOP();
	}
}
