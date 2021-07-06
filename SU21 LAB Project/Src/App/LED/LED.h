
/*****************************************************************************
 @Project	: 
 @File 		: LED.c
 @Details  	: Functions to display LED colours                    
 @Author	: 
 @Hardware	: 
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  LucD     		XXXX-XX-XX  	Initial Release
   
******************************************************************************/


/*****************************************************************************
 Define
******************************************************************************/
#define RGB_OFF			0x00
#define RGB_RED			0x02
#define RGB_GREEN		0x08
#define RGB_BLUE		0x04
#define RGB_MAGENTA	0x06
#define RGB_YELLOW	0x0A
#define RGB_CYAN		0x0C
#define RGB_WHITE		0x0E

/*****************************************************************************
 Macro
******************************************************************************/
#define LED_RGB_SET( x )    ((*((volatile unsigned long *)(GPIOF_BASE + (0x0E << 2)))) = (x)%0xFF )
#define LED_RGB_GET()				(GPIOF->DATA & 0x0E);

#define LED_RED_ON()				(GPIOF->DATA |= BIT(PF_LED_RED))
#define LED_RED_OFF()				(GPIOF->DATA &= ~BIT(PF_LED_RED))
#define LED_RED_SET( x )		((x)? LED_RED_ON() : LED_RED_OFF())

#define LED_BLUE_ON()				(GPIOF->DATA |= BIT(PF_LED_BLUE))
#define LED_BLUE_OFF()			(GPIOF->DATA &= ~BIT(PF_LED_BLUE))
#define LED_BLUE_SET( x )		((x)? LED_BLUE_ON() : LED_BLUE_OFF())

#define LED_GREEN_ON()			(GPIOF->DATA |= BIT(PF_LED_GREEN))
#define LED_GREEN_OFF()			(GPIOF->DATA &= ~BIT(PF_LED_GREEN))
#define LED_GREEN_SET( x )	((x)? LED_GREEN_ON() : LED_GREEN_OFF())

/*****************************************************************************
 Global Variables
******************************************************************************/


/*****************************************************************************
 Local Variables
******************************************************************************/



