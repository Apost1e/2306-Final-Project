/*****************************************************************************
 @Project	: 	ECE300 - Tiva Base Board
 @File : 			i2c.c
 @Details : 	TM4C123G I2C driver               
 @Author	: 	ldenissen
 @Hardware	: TM4C123G
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  ldenissen   2019-05-22 		Initial Release
	 1.1  ldenissen   2020-08-10 		Bug Fix on callback for writing
	 1.2  ldenissen   2020-08-31 		Bug Fix on callback for reading
	 1.3  ldenissen   2020-09-12 		Workaround for Master Write Error Errata 
******************************************************************************/

#ifndef __I2C_DOC_H__
#define __I2C_DOC_H__

#include <common.h>

/*****************************************************************************
 Define
******************************************************************************/

#define I2C_OK					0
#define I2C_ERR					-1

#define I2C_100K				0U
#define I2C_400K				1U
#define I2C_600K				2U
#define I2C_800K				3U
#define I2C_1000K				4U

#define I2C_0						0U
#define I2C_1						1U
#define I2C_2						2U
#define I2C_3						3U


/*****************************************************************************
 Type definition
******************************************************************************/
typedef void I2C_CB_UPDATE( void );

typedef struct _tagI2C_Handle
{
	void 				*pI2C;
	int					Irq;
	uint8_t 		*data; 
	uint32_t 		nBytes;	
	uint32_t		nByteCnt;
	BOOL				bBusy;
	BOOL				bRead;
	BOOL				bError;
	BOOL				bAddressNack;
	I2C_CB_UPDATE  *pfCallback;
}I2C_HANDLE, 	*PI2C_HANDLE;


/******************************************************************************
 Global functions
******************************************************************************/


/******************************************************************************
 @Description 	: Intialize specified timer

 @param			: pHandle - To an empty handle pointer
							nTimer - Timer index e.g: 0A, 0B, 1A ... 5B (for 32 bit always use A) 
							nFreq - Timer tick speed in Hz
							bIs32Bit - Set TRUE for 32 bit mode, otherwise timer will work in 16 bit mode
 
 @revision		: 1.0.0
 
******************************************************************************/
int I2CInit( PI2C_HANDLE pHandle, uint8_t nI2C, uint8_t nI2C_speed);

/******************************************************************************
 @Description 	: Write bytes to I2C device

 @param			: pHandle - To an intialized handle pointer
							Address - Address of the I2C device to write to
							Reg_data - Register on I2C device to write to
							*data - Pointer to buffer that contains the data to be send
							nBytes - Number of bytes that should be transmitted
 
 @revision		: 1.0.0
 
******************************************************************************/
void I2CWrite( PI2C_HANDLE pHandle, uint8_t Address, uint8_t Reg_data, uint8_t *data, uint32_t nBytes );

/******************************************************************************
 @Description 	: Read bytes from I2C device

 @param			: pHandle - To an intialized handle pointer
							Address - Address of the I2C device to read from
							Reg_data - Register on I2C device to read from
							*data - Pointer to buffer that will store the received data
							nBytes - Number of bytes requested
 
 @revision		: 1.0.0
 
******************************************************************************/
void I2CRead( PI2C_HANDLE pHandle, uint8_t Address, uint8_t Reg_data, uint8_t *data, uint32_t nBytes );

/******************************************************************************
 @Description 	: Add callback to a I2C

 @param			: pHandle - To an intialized handle pointer
							pfUpdate - callback function pointer
				   
 @revision		: 1.0.0
 
******************************************************************************/
void 
I2CAddCallback(
	PI2C_HANDLE 		pHandle,
	I2C_CB_UPDATE 	*pfUpdate );

#endif /* __I2C_DOC_H__ */


