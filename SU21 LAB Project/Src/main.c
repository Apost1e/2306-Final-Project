/*****************************************************************************
 @Project		: SEM2306 Lab Assignment
 @File 			: main.c
 @Details  	:
 @Author		: Sirius Phua
 @Hardware	: Tiva LaunchPAd TM4C123
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0       			26 May 21  		Initial Release
   
******************************************************************************/

#include "Common.h"
#include "Hal.h"
#include "BSP.h"
#include "LED.h"
#include "IRQ.h"
#include "spim.h"
#include "LCD_ST7735R.h"
#include "gui.h"
#include "i2c.h"
#include "i2c_sensors.h"
#include "timer.h"

/*****************************************************************************
 Define
******************************************************************************/
#define LCD_BUF_SIZE				4096
#define LCD_UPDATE_MS				10U
#define KEYPAD_UPDATE_MS	50U
#define BUZZER_MS					100U

#define SHORT_BEEP					30U        /* short beep duration in ms  */
#define I2C_UPDATE_MS				1000U
#define AHT10_UPDATE_MS			5000U			/* for I21C module            */
#define SW_DEBOUNCE_INTRV 	10U       /* SW debounce period in ms   */
#define AHT10_DELAY					80U				/* Measurment Delay in ms */

/* defines states in the FSM array  */
#define S6  &fsm[0]
#define S2  &fsm[1]
#define S3  &fsm[2]
#define S1  &fsm[3]
#define S9  &fsm[4]
#define S8  &fsm[5]
#define S12 &fsm[6]
#define S4  &fsm[7]

#define MOTOR_STEP_TIME  	10  		/* time in ms between steps */
#define MOTOR_ANGLE				360			/* motor angle to turn */

/*****************************************************************************
 Type definition
******************************************************************************/



/*****************************************************************************
 Global Variables
******************************************************************************/
void GUI_AppDraw( BOOL bFrameStart );

/*****************************************************************************
 Local const Variables
******************************************************************************/
/* 7 Segment binary values, Active Low */
static uint8_t LCD_0		=	0x40;
static uint8_t LCD_1		=	0xF9;
static uint8_t LCD_2		=	0xA4;
static uint8_t LCD_3		=	0x30;
static uint8_t LCD_4		=	0x19;
static uint8_t LCD_5		=	0x12;
static uint8_t LCD_6		=	0x02;
static uint8_t LCD_7		=	0x78;
static uint8_t LCD_8		=	0x00;
static uint8_t LCD_9		=	0x10;

/* structure represents a State of the FSM   */ 
struct State
{
  uint8_t Out;            
  const struct State *Next; 
};
typedef const struct State StateType;

StateType fsm[8]=
{
  {0x60, S2 },   /* step 0, current state S6  */
	{0x20, S3 },   /* step 1, current state S2  */
  {0x30, S1 },   /* step 2, current state S3  */
	{0x10, S9 },   /* step 3, current state S1  */
	{0x90, S8 },   /* step 4, current state S9  */
	{0x80, S12 },  /* step 5, current state S8  */
	{0xC0, S4 },   /* step 6, current state S12 */
	{0x40, S6 },   /* step 7, current state S4  */
};

const struct State *Pt;  // Current State

unsigned char cState; 

enum Rotation {CW , ACW} ;

static char KEY_DECODE_TABLE[4][3] = 
{
	{ 1, 2, 3 },
	{ 4, 5, 6 },
	{ 7, 8, 9 },
	{ '*', 0, '#' }
};

/*****************************************************************************
 Local Variables
******************************************************************************/
/* SysTick   */
static volatile BOOL	g_bSystemTick = FALSE;
static volatile BOOL 	g_bSystemTick1000 = FALSE; /* flag changes after 1000 ticks */
static volatile BOOL	g_bSecTick = FALSE;
static int            g_nCount = 0;
static volatile BOOL	g_bToggle = FALSE;
static unsigned int		g_nTimeSec = 0;

/* LCD */
static volatile int 	g_bSpiDone = FALSE;
static SPIM_HANDLE		g_SpimHandle;
static GUI_DATA				g_aBuf[LCD_BUF_SIZE];
static GUI_MEMDEV			g_MemDev;
static volatile BOOL 	g_bLCDUpdate = FALSE;
static volatile int 	g_nLCD = LCD_UPDATE_MS;
static volatile BOOL	g_bBacklightOn = TRUE;

static volatile BOOL 	g_bLcdFree  = TRUE;

static volatile int   g_nDelay = 0;

/* I2C0     */
static I2C_HANDLE			g_I2C0Handle;
volatile BOOL	g_bI2C0IsBusy = FALSE;

/* AHT10 */
static AHT10 g_AHT10;																				// data structure
static volatile uint8_t 	g_nAHT10Delay;										// measurement delay
static BOOL								g_bAHT10Read = FALSE;							// AHT10 Read flag - asserted after Measurement Delay
static volatile int				g_nAHT10Update = AHT10_UPDATE_MS;	// controls AHT10 update interv 
static volatile BOOL			g_bAHT10Update = FALSE;   				// AHT10 Update flag

/* MCP23017 */
static MCP23017 					g_MCP23017;												// data structure

/** Buzzer     **/
static volatile uint16_t	g_nBeep_Count = 0U;
static volatile uint32_t 	g_nBuzzerONCountDn;
static volatile uint32_t 	g_nBuzzerOFFCountDn;
static volatile BOOL			g_bBuzzerON = FALSE;

/** I2c Expander     **/
static volatile uint8_t	dig1 = 0;
static volatile uint8_t	dig2 = 0;

/** 7-segment */
static uint16_t						g_n7SegCount = 0;

/* Debounce */
static volatile int 			g_debounce = 0;
static volatile BOOL 			g_bInitialState = TRUE;

/* LED */
static volatile BOOL			g_bLED_Toggle;

/** keypad 					**/
static volatile BOOL 	g_nKeypadScan = FALSE;
static volatile int 	g_nKeypad = KEYPAD_UPDATE_MS;
static unsigned char 	g_cKey = 0;
static volatile BOOL 	g_bKeyPressed = FALSE;	
static volatile int		g_BounceThresh = 0;
static int						g_DebounceValue = 150;
static volatile int numPos = 0;

static volatile int 			_4DigitPin = 0;

/** stepper motor   **/
static volatile int   		g_nMotorCount = MOTOR_STEP_TIME;
static volatile BOOL  		g_bmotor_move = FALSE;
int steps;
static volatile int motor_angle = 0;
static int Rotation =	CW;
static volatile BOOL g_bDirectional_Op = FALSE;

/** Buttons 				**/
static BOOL								SW1_Pressed = FALSE;  /* switch-pressed status  */
static BOOL								SW2_Pressed = FALSE; 

static volatile int				KI_Digits_Counter = 0;	//Keyin Digits counter
static volatile int				KI_Digits = 0;	//Keyed-in Digits (Set up)
static volatile int				Validate_KI_Digits = 0;	//Keyed-in Digits (Unlocking and Locking of Door)
static volatile int				Tries = 5;	//Number of tries
static volatile int				Tries_Counter = 0;	//Tryouts
static volatile BOOL  		SetUpDone = FALSE;
static unsigned char 			g_BackSpaceKey = 0;


/*****************************************************************************
 Callbacks Prototypes
******************************************************************************/
void main_cbI2C0Isdone( void );
uint8_t LCD_Count ( uint16_t );
/*****************************************************************************
 Local Functions Prototypes
******************************************************************************/
static void main_LcdInit( void );
static void main_KeyScan( void );
static void motor_output (int);

void GUI_AppDraw( BOOL bFrameStart );

/*****************************************************************************
 Implementation
******************************************************************************/
int main()
{
	Port_Init();
	SystemCoreClockUpdate ();
	SysTick_Config( SystemCoreClock/1000 );  
	
	/* SSI initialization */
	NVIC_SetPriority( SSI0_IRQn, 0 );
	
	// initialize SSI port for TFT LCD
	SpimInit(
		&g_SpimHandle,
		0U,
		25000000U,
		SPI_CLK_INACT_LOW,
		SPI_CLK_RISING_EDGE,
		SPI_DATA_SIZE_8 );
	
	main_LcdInit();
	
	/* Initialise I2C0 bus   */
	I2CInit(&g_I2C0Handle, I2C_0,I2C_400K);
	I2CAddCallback(&g_I2C0Handle, main_cbI2C0Isdone); // add Callback for I2C bus transaction
	NVIC_SetPriority( I2C0_IRQn, 1 );
	
	AHT10_Init (&g_I2C0Handle, &g_AHT10); 							// initialize AHT10 module
	I2CExpander_Init (&g_I2C0Handle, &g_MCP23017); 		// initialize MCP23017
	
	IRQ_Init();
	
	/* print to Virtual COM port temrinal */
	printf ("\n\rHello World! \n\r"); // display to virtual COM port
	printf ("Count Mode: %d\n\r", g_MCP23017.Mode);
	
	for(;;)
  {
		if(g_bInitialState)
		{
				g_MCP23017.Digit1 = 0x7f;		/* Stores the first value which is in multiples of 10 */
				g_MCP23017.Digit2 = 0x7f;		/* Stores the remainder of divide by 10 (modulo) */
				
				write_I2CExpander(&g_I2C0Handle, &g_MCP23017);			/* Updates displayed value */
		}
		
		//if(g_n7SegCount > 99) g_n7SegCount = 99; 								/* Prevents overcounting outside of 99*/
		
		if( FALSE != g_bSystemTick1000 )
    {
			g_bSystemTick1000 = FALSE;	
			/* Count down mode must happen in 1 second loop */
			if(g_MCP23017.Mode == COUNT_DOWN && !g_bInitialState)						/* Check for Count Down Mode*/
			{
				if(g_n7SegCount>0)												/* While 7 Segment stored value is greater than 0, this also must be top of function or the 7 segment value will update 1sec late */
				{
					--g_n7SegCount;
				
				g_MCP23017.Digit1 = LCD_Count( g_n7SegCount/10 );		/* Stores the first value which is in multiples of 10 */
				g_MCP23017.Digit2 = LCD_Count( g_n7SegCount%10 );		/* Stores the remainder of divide by 10 (modulo) */
				
				write_I2CExpander(&g_I2C0Handle, &g_MCP23017);			/* Updates displayed value */
				g_bBuzzerON = TRUE;																	/* Sets Buzzer flag to true */
				}			
			}
		}
		
	
		if( FALSE != g_bLCDUpdate )
		{
			if( 0 != g_bLcdFree )
			{
				g_bLCDUpdate = FALSE;
				g_bLcdFree = FALSE;
				/* Draw every block. Consumes less time  */
				GUI_Draw_Exe(); 
			}
		} /* LCD update */
		
		/* AHT Read */
		if(g_bAHT10Update)
		{
		AHT10Trigger(&g_I2C0Handle, &g_AHT10);			/* AHT10 is set to read data */
		g_nAHT10Delay = AHT10_DELAY;								/* Delay is set for reading to stabilise */
		}
			
		if(g_bAHT10Update && g_bAHT10Read)					/* Flag is triggered in SysTick */
		{
			g_bAHT10Read = FALSE;											/* Set flags to false */
			g_bAHT10Update = FALSE;
			AHT10ReadRawdata(&g_I2C0Handle, &g_AHT10);/* Read raw data and process data */
		}
		
		LED_BLUE_SET ( g_bLED_Toggle );
		
		if (g_bDirectional_Op)													/* Checks motor start flag*/
		{	
			if (FALSE != g_bmotor_move && 0 != steps)			/* While the motor should be moving and steps is still counting down*/
			{			
				g_bmotor_move = FALSE; 											/* Resets flag */
				motor_output (cState);											/* moves in current state*/
				if(Rotation == CW)													/* If Direction is set to CW */
					{
						/* Clockwise moves in incremental direction according to defined FSM */
						/* Once the final state is reached, resets to initial set and continue counting until stopped */
						cState++;
						if (cState == 8)
							cState = 0;
					}
					if( Rotation == ACW)											/* If Direction is set to ACW */
					{
						/* Anti-Clockwise moves in opposite direction, thus it decrements and is able to still utilise the FSM meant for CW */
						/* Once CW initial state is reached, resets to final state of CW */
						cState--;
						if(cState == 0)
							cState = 8;
					}
				/* Steps is constantly decremented, once steps is 0, the flag to run the motor is stopped and new value input is enabled.*/
				steps--;
				if (steps == 0)
				{
					g_bDirectional_Op = FALSE;
					numPos = 0;
				}
			}
		}
	
	if(SetUpDone == FALSE)
	{
		if( FALSE != g_nKeypadScan )
		{
			g_nKeypadScan = FALSE;
			main_KeyScan();
		}
	}
	
	
		
	
	}
	
}	

/*****************************************************************************
 Callback functions
******************************************************************************/
void SysTick_Handler( void )  
{
	g_bSystemTick = TRUE;
		
	/* Provide system tick */
  g_nCount++;
	g_nBuzzerOFFCountDn--;
	g_debounce--;
	g_nAHT10Delay--;
	g_nAHT10Update--;
	
	/* LED */
	if(g_nCount >= 100)
		g_bLED_Toggle = FALSE;
	
	/* Delay for AHT Trigger */
	if(g_nAHT10Update == 0) 
	{
		g_nAHT10Update = AHT10_UPDATE_MS;				/* Sets the flag to trigger AHT10 Module every 5 seconds */
		g_bAHT10Update = TRUE;
	}
	
	/* Check if Delay is up and AHT has not yet read */
	if(g_nAHT10Delay == 0 && !g_bAHT10Update)
	{
		g_bAHT10Read = TRUE;
	}
	
	/* Buzzer Flag */
	/*
	if(g_nBuzzerOFFCountDn == 0 && !g_bBuzzerON) // Turns off Buzzer when Counter hits 0 and Flag is false 
		{

			BUZZER_SET (g_bBuzzerON);								
		}
		if(g_bBuzzerON)
		{	
			BUZZER_SET ( g_bBuzzerON );								//Turns on Buzzer 
			g_bBuzzerON = FALSE;											//Sets flag to False 
			g_nBuzzerOFFCountDn = SHORT_BEEP;					//Count down duration 
			
		}
	*/
  if (g_nCount == 1000)
  {
    g_bSecTick = TRUE;
		g_bSystemTick1000 = TRUE;
		g_nCount=0;
		g_bLED_Toggle = TRUE;
		
		
		/* Keep track of time based on 1 sec interval */ 
		g_nTimeSec++;
		if(g_nTimeSec > 24*60*60)
		{
			g_nTimeSec = 0;
		}
  } // g_nCount

	if( 0 != g_nLCD )
	{
		g_nLCD--;
		if( 0 == g_nLCD )
		{
			g_nLCD = LCD_UPDATE_MS;
			g_bLCDUpdate = TRUE;
		}
	}
	
	/**  Keypad flag	**/
	if( 0 != g_nKeypad )
	{
		g_nKeypad--;
		if( 0 == g_nKeypad )
		{
			g_nKeypad = KEYPAD_UPDATE_MS;
			g_nKeypadScan = TRUE;
		}
	}
	
	/** Motor flag																	**/
	/*
	if (0 != g_nMotorCount)
	{
		g_nMotorCount--;
		if (0 == g_nMotorCount)
		{
			g_bmotor_move = TRUE;
			g_nMotorCount = MOTOR_STEP_TIME;
		}		
	}
	*/
	
}

void GUI_AppDraw( BOOL bFrameStart )
{
	/* This function is invoked from the GUI library */
	char buf[128];
		
	GUI_Clear( ClrBlue ); /* Set background to blue.Refer to gui.h */
	GUI_SetFont( &FONT_Arialbold16 );
	GUI_SetFontBackColor( ClrBlue );
	GUI_PrintString( "SEM2306 SU21", ClrYellow,8,8 );
	GUI_SetFont( &FONT_Arialbold12 );
	GUI_PrintString( "MORI", ClrWhite,8,28 );

	GUI_SetColor(ClrYellow);
	GUI_DrawFilledRect(0,55,127,158);
	GUI_SetFont( &g_sFontCalibri10 );	
	
	sprintf( buf, "TEMP: %4.2f C", g_AHT10.fTemperature);
	GUI_PrintString( buf, ClrBlack, 10, 60);
	sprintf( buf, "HUMD: %3.1f %%", g_AHT10.fHumidity);
	GUI_PrintString( buf, ClrBlack, 10, 72);
	
	sprintf( buf, "COUNT: %d", g_n7SegCount);
	GUI_PrintString( buf, ClrDarkBlue, 10, 84);
	
	//Setup
	if(SetUpDone == FALSE)
	{
		GUI_SetFont( &FONT_Arialbold12 );
		sprintf( buf, "Pin Entered %i", KI_Digits);
		GUI_PrintString( buf, ClrBlack, 15, 108 );
		sprintf( buf, "Setup Failed"); //Default Door is Locked
		GUI_PrintString( buf, ClrBlack, 15, 120 );
		
	}
		
	//After Setup
	if(SetUpDone == TRUE)
	{
		GUI_SetFont( &FONT_Arialbold12 );
		sprintf( buf, "Pin Entered %i", Validate_KI_Digits);
		GUI_PrintString( buf, ClrBlack, 15, 108 );
		sprintf( buf, "Door is Currently Locked"); 
		GUI_PrintString( buf, ClrBlack, 15, 120 );
	}
	
	/*
	//Pins Matched	
	if(Validate_KI_Digits == KI_Digits && Validate_DoorStatus == DoorStatus)
	{
		GUI_PrintString( buf, ClrBlack, 15, 108 );
		if (Validate_DoorStatus == CW && g_bDirectional_Op ==TRUE)
		sprintf( buf, "Door is Locked");
		else if (Validate_DoorStatus == ACW && g_bDirectional_Op == TRUE)
		sprintf( buf, "Door is Unlocked");
		GUI_PrintString( buf, ClrBlack, 15, 120 );
	}
*/
	
	GUI_SetColor( ClrLightCyan );
	GUI_DrawFilledRect( 0, 140, 127, 159);
	GUI_SetFont( &FONT_Arialbold16 );
	GUI_SetFontBackColor( ClrLightCyan );
	sprintf( buf, "%02uMHz %02u:%02u:%02u", SystemCoreClock/1000000, (g_nTimeSec/3600)%24, (g_nTimeSec/60)%60, g_nTimeSec%60 );
	GUI_PrintString( buf, ClrBlack,4,143 );
}

static void main_cbLcdTransferDone( void )
{
	g_bLcdFree = TRUE;
}

static void main_cbGuiFrameEnd( void )
{
	g_bLcdFree = TRUE;
}

void main_cbI2C0Isdone ( void )
{
	g_bI2C0IsBusy = FALSE; 
}

/*****************************************************************************
 Local functions
******************************************************************************/
static void main_LcdInit( void )
{
	int screenx;
	int screeny;
	
	/* g_SpimHandle shall be initialized before use */
	
	/* Choosing a landscape orientation */
	LcdInit( &g_SpimHandle, LCD_POTRAIT_180 );
	
	/* Get physical LCD size in pixels */
	LCD_GetSize( &screenx, &screeny );
	
	/* Initialize GUI */
	GUI_Init(
		&g_MemDev,
		screenx,
		screeny,
		g_aBuf,
		sizeof(g_aBuf) );
	
	/* Switch to transfer word for faster performance */
	SpimSetDataSize( &g_SpimHandle, SPI_DATA_SIZE_16 );
	GUI_16BitPerPixel( TRUE );
	
	/* Clear LCD screen to Blue */
	GUI_Clear( ClrBlue );

  /* set font color background */
  GUI_SetFontBackColor( ClrBlue );
    
  /* Set font */
	GUI_SetFont( &g_sFontCalibri10 );
	
	LCD_AddCallback( main_cbLcdTransferDone );
	
	GUI_AddCbFrameEnd( main_cbGuiFrameEnd );
	
	/* Backlight ON */
	LCD_BL_ON();
}

uint8_t LCD_Count ( uint16_t count )		/* Switch case for 7 segment display */
{
	switch (count)
	{
		case 0:
			return LCD_0;
		case 1:
			return LCD_1;
		case 2:
			return LCD_2;
		case 3:
			return LCD_3;
		case 4:
			return LCD_4;
		case 5:
			return LCD_5;
		case 6:
			return LCD_6;
		case 7:
			return LCD_7;
		case 8:
			return LCD_8;
		case 9:
			return LCD_9;
	}
	return 0;
}



	static void main_KeyScan( void )
	{
		int nRow, nCol, input;
		static BOOL bKeyPressed = FALSE;

	

			//Start up and User keys in the pin number
			KEYPAD_ALL_ROWS_ON();

			for( nRow=0; nRow<4; nRow++ )
			{
				/* Pull row by row low to determined which button is pressed */
				KEPAD_ROW_MASKED &= ~(1U << nRow);
		
				/* Short delay to stabalize row that has just been pulled low */
				__nop(); __nop(); __nop();
		
				/* Read input */
				input = KEYPAD_COL_IN();	
		
				if(input!=0x07)																//If any bit is pulled low
				{
					if( input == 0x06 ) 									//If Bit 0 (Col 0) is pulled low 
					{
						g_cKey = KEY_DECODE_TABLE[nRow][0];	//Update g_cKey	
					}
					else if( input == 0x05 ) 								//If Bit 1 (Col 1) is pulled low 
					{    
						g_cKey = KEY_DECODE_TABLE[nRow][1];	//Update g_cKey
					}
					else if( input == 0x03 ) 								//If Bit 2 (Col 2) is pulled low 
					{
						g_cKey = KEY_DECODE_TABLE[nRow][2];	//Update g_cKey
					}
						g_bKeyPressed = TRUE;								//Change Flag to True
						bKeyPressed = TRUE;
						break;
				}
						
				/* If a column is asserted, then look for the corresponding key 
				Make use of the KEY_DECODE_TABLE, exit loop once key found!  */	
			}
		if(SetUpDone == FALSE)
		{
				if( KI_Digits_Counter < 5 && !g_bKeyPressed )  /* Data in managed in 2 parts, checking for rotational direction, then saving the turn angle */
				{  
					if(bKeyPressed)									/* local variable is used to prevent multiple inputs while the key is being held down */
					{
						bKeyPressed = FALSE;
						
						//BackSpace
						if(g_cKey == '*')
						{
							KI_Digits_Counter--;
									
							if(KI_Digits_Counter >= 0)
							{
								KI_Digits/=10;
							}
							
						}
						//4 Digits 
						if( KI_Digits_Counter < 4)
						{
							if( g_cKey != '#' && g_cKey != '*')				/* Guard statement*/
							{
								KI_Digits*=10;												/* Shifts the numbers left in decimal form (factor 10)*/
								KI_Digits += g_cKey;									/* Adds current value to angle */
								KI_Digits_Counter++;									/* Tracks current digit length */
							}
						}
					
								
					}
				}
		}
		else //Unlock and Lock
		{
			if( numPos < 5 && !g_bKeyPressed )  /* Data in managed in 2 parts, checking for rotational direction, then saving the turn angle */
			{  
				if(bKeyPressed)									/* local variable is used to prevent multiple inputs while the key is being held down */
				{
					bKeyPressed = FALSE;
						
					//BackSpace
					if(g_cKey == '*')
					{
						numPos--;
									
						if(numPos >= 0)
						{
							Validate_KI_Digits/=10;
						}
							
					}
					//4 Digits 
					if( numPos < 4)
					{
						if( g_cKey != '#' && g_cKey != '*')				/* Guard statement*/
						{
							Validate_KI_Digits*=10;									/* Shifts the numbers left in decimal form (factor 10)*/
							Validate_KI_Digits += g_cKey;						/* Adds current value to angle */
							numPos++;																/* Tracks current digit length */
						}
					}
					
								
				}
			}
		}
				
		
		
				
				
				/* Check if key is released */
			if(input == 0x07)
			{
				g_bKeyPressed = FALSE;						//Change Flag to False	
			}
			/* Reset all rows for next key detection */
			KEYPAD_ALL_ROWS_OFF();
				

				
				
				
				
				/* Check if key is released */
				if(input == 0x07)
				{
					g_bKeyPressed = FALSE;						//Change Flag to False		
				}
			
				/* Reset all rows for next key detection */
				KEYPAD_ALL_ROWS_OFF();		
}

void motor_output (int state)
{
	GPIOC->DATA = fsm [state].Out;
}

/*****************************************************************************
 Interrupt functions
******************************************************************************/
void GPIOF_Button_IRQHandler( uint32_t Status )
{
	if( 0 != (Status & BIT(PF_SW2 ) ))
		{
			KI_Digits_Counter = 0;
			KI_Digits = 0;
			SetUpDone = FALSE;
		}
		if( 0 != (Status & BIT(PF_SW1) ))
	{
		if(g_debounce <= 0)
		{
			g_debounce = 150;
			SetUpDone = TRUE;	
		}
	}
	GPIOF->ICR |= BIT(PF_SW1)|BIT(PF_SW2); /* clear intr  */	
}  /* GPIOF_Button_IRQHandler */

void GPIOE_Button_IRQHandler( uint32_t Status )
{

	if( 0 != (Status&BIT(PE_KEYPAD_COL0)) )
	{
		if( g_BounceThresh == 0)
		{
			g_BounceThresh = g_DebounceValue;
			GPIOE->ICR = BIT(PE_KEYPAD_COL0);
		}
	}
	
	if( 0 != (Status&BIT(PE_KEYPAD_COL1)) )
	{
		if( g_BounceThresh == 0)
		{
			g_BounceThresh = g_DebounceValue;
		GPIOE->ICR = BIT(PE_KEYPAD_COL1);
		}
	}
	if( 0 != (Status&BIT(PE_KEYPAD_COL2)) )
	{
		if( g_BounceThresh == 0)
		{
			g_BounceThresh = g_DebounceValue;
		GPIOE->ICR = BIT(PE_KEYPAD_COL2);
		}
	}
}

 void TIMER0A_IRQHandler( uint32_t Status )
	{
		
		/* Checks for any timer interrupt */
		if( 0 != (Status & TIMER_RIS_TATORIS) )
		{
			g_bToggle = !g_bToggle;
			PB6_Test(g_bToggle);	/* Weirdly AFSEL for PB6 cannot be written into  */
			PB5_Test(g_bToggle);	/* Outputs normally */
			TIMER0->ICR |= TIMER_ICR_TATOCINT; /* clear intr */
			/** interrupt processing codes **/
		}
		
	}
