/*****************************************************************************
 @Project	: SEM2306 Lab Assignment
 @File 		: i2c_sensor.h
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
#include "i2c.h"

/*****************************************************************************
 Type definition
******************************************************************************/

/*** Definitions for MCP23017      ***/

#define MCP23017_ADDR 			0x20 // MCP23017 Address

// registers
#define MCP23017_IODIRA 0x00   // I/O direction register A
#define MCP23017_IPOLA 0x02    // Input polarity port register A
#define MCP23017_GPINTENA 0x04 // Interrupt-on-change pins A
#define MCP23017_DEFVALA 0x06  // Default value register A
#define MCP23017_INTCONA 0x08  // Interrupt-on-change control register A
#define MCP23017_IOCONA 0x0A   // I/O expander configuration register A
#define MCP23017_GPPUA 0x0C    // GPIO pull-up resistor register A
#define MCP23017_INTFA 0x0E    // Interrupt flag register A
#define MCP23017_INTCAPA 0x10  // Interrupt captured value for port register A
#define MCP23017_GPIOA 0x12    // General purpose I/O port register A
#define MCP23017_OLATA 0x14    // Output latch register 0 A

#define MCP23017_IODIRB 0x01   // I/O direction register B
#define MCP23017_IPOLB 0x03    // Input polarity port register B
#define MCP23017_GPINTENB 0x05 // Interrupt-on-change pins B
#define MCP23017_DEFVALB 0x07  // Default value register B
#define MCP23017_INTCONB 0x09  // Interrupt-on-change control register B
#define MCP23017_IOCONB 0x0B   // I/O expander configuration register B
#define MCP23017_GPPUB 0x0D    // GPIO pull-up resistor register B
#define MCP23017_INTFB 0x0F    // Interrupt flag register B
#define MCP23017_INTCAPB 0x11  // Interrupt captured value for port register B
#define MCP23017_GPIOB 0x13    // General purpose I/O port register B
#define MCP23017_OLATB 0x15    // Output latch register 0 B

#define MCP23017_INT_ERR 255   // Interrupt error


/*** Definitions for AHT10    ***/

#define AHT10_I2C_ADDRESS						0x38

#define AHT10_INIT_CMD							0xE1	//initialization command for AHT10
#define AHT10_START_MEASURMENT_CMD	0xAC	//start measurment command
#define AHT10_NORMAL_CMD           0xA8  //normal cycle mode command
#define AHT10_SOFT_RESET_CMD        0xBA  //soft reset command

#define AHT10_INIT_NORMAL_MODE			0x00  //enable normal mode
#define AHT10_INIT_CYCLE_MODE				0x20  //enable cycle mode
#define AHT10_INIT_CMD_MODE					0x40  //enable command mode
#define AHT10_INIT_CAL_ENABLE				0x08  //load factory calibration coeff

#define AHT10_DATA_MEASURMENT_CMD   0x33  //no info in datasheet!!! my guess it is DAC resolution, saw someone send 0x00 instead
#define AHT10_DATA_NOP              0x00  //no info in datasheet!!!

// delay values are in ms
#define AHT10_MEASURMENT_DELAY     80 	//at least 75 milliseconds
#define AHT10_POWER_ON_DELAY       40 	//at least 20..40 milliseconds
#define AHT10_CMD_DELAY            350	//at least 300 milliseconds
#define AHT10_SOFT_RESET_DELAY     20		//less than 20 milliseconds


/******************************************************************************
 Data Structures & Definitions
******************************************************************************/

typedef struct tag_AHT10 {
		BOOL bPresent;
		uint8_t I2C_adr;
		uint8_t Status;
		float fTemperature;
		float fHumidity;
} AHT10; 

typedef enum {SET, COUNT_DOWN, STOP} Mode;

typedef struct tag_MCP23017 {
		uint8_t I2C_addr; 	// MCP23017 I2c addr
		uint8_t Mode;    		// Count-down timer Mode 
		uint8_t Digit1; 		// left 7-segment digit
		uint8_t Digit2; 		// right 7-segment digit
} MCP23017; 


/******************************************************************************
 Functions
******************************************************************************/

void AHT10_Init(PI2C_HANDLE pHandle, AHT10 *pAHT10);

/* sends trigger command to AHT10 to initate measurments  */
void AHT10Trigger(PI2C_HANDLE pHandle, AHT10 *pAHT10);

/* read temp & humidity data from AHT10                          */
/* this function should be called after Trigger command is sent  */
void AHT10ReadRawdata(PI2C_HANDLE pHandle, AHT10 *pAHT10);

void I2CExpander_Init (PI2C_HANDLE pHandle, MCP23017 *pMCP23017);

/* function to write to the MCP23017 Output Latch registers   */
void write_I2CExpander (PI2C_HANDLE pHandle, MCP23017 *pMCP23017);

/* test functions */
void I2C_Write_Test1 (PI2C_HANDLE pHandle, MCP23017 *pMCP23017);
void I2C_Write_Test2 (PI2C_HANDLE pHandle, MCP23017 *pMCP23017);
