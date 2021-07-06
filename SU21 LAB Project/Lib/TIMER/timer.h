/*****************************************************************************
 @Project		: ECE300 LABS 
 @File 			: Timer.c
 @Details  	: TM4C123G Timer driver               
 @Author		: ldenissen
 @Hardware	: TM4C123G
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  ldenissen   2018-09-11 		Initial Release
   
******************************************************************************/

#ifndef __TIMER_DOC_H__
#define __TIMER_DOC_H__

/*****************************************************************************
 Define
******************************************************************************/

#define TIMER_OK				0
#define TIMER_ERR_INVALID 		-1

#define TIMER0_A				0U
#define TIMER0_B				1U
#define TIMER1_A				2U
#define TIMER1_B				3U
#define TIMER2_A				4U
#define TIMER2_B				5U
#define TIMER3_A				6U
#define TIMER3_B				7U
#define TIMER4_A				8U
#define TIMER4_B				9U
#define TIMER5_A				10U
#define TIMER5_B				11U


#define TIMER_CON32				0U
#define TIMER_SUB_A				1U
#define TIMER_SUB_B				2U


/*****************************************************************************
 Type definition
******************************************************************************/
typedef void TIMER_CB_UPDATE( void );

typedef struct _tagTimer_Hook
{
	TIMER_CB_UPDATE 		*pfUpdate;
	struct _tagTimer_Hook 	*pNext;
}TIMER_HOOK, *PTIMER_HOOK;


typedef struct _tagTimer_Handle
{
	void 					*pTimer;
	int						Irq;
	BOOL					bTimerB;
	BOOL					bIs32Bit;
	BOOL					bPWM;
	PTIMER_HOOK		pHeadHook;
}TIMER_HANDLE, 	*PTIMER_HANDLE;


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
int TimerInit( PTIMER_HANDLE pHandle, uint8_t nTimer, uint32_t nFreq, BOOL bIs32Bit, BOOL bPWM );

/******************************************************************************
 @Description 	: Set PWM duty cycle for the specified timer

 @param			: pHandle - To an intialized handle pointer
							nDutyCycle - Duty cycle in % from 1 to 100
							bInverse - Inverts the behaviour of the PWM module
 
 @revision		: 1.0.0
 
******************************************************************************/
void TimerSetPWM( PTIMER_HANDLE pHandle, double dDutyCycle, BOOL bInverse );

/******************************************************************************
 @Description 	: Start specified timer

 @param			: pHandle - To an intialized handle pointer
 
 @revision		: 1.0.0
 
******************************************************************************/
void TimerStart( PTIMER_HANDLE 	pHandle );

/******************************************************************************
 @Description 	: Stop specified timer

 @param			: pHandle - To an intialized handle pointer
 
 @revision		: 1.0.0
 
******************************************************************************/
void TimerStop( PTIMER_HANDLE 	pHandle );

/******************************************************************************
 @Description 	: Add callback to a timer

 @param			: pHandle - To an intialized handle pointer
				  pHook   - To an empty hook pointer
				  pfUpdate - callback function pointer
				   
 @revision		: 1.0.0
 
******************************************************************************/
void 
TimerAddHook(
	PTIMER_HANDLE 		pHandle,
	PTIMER_HOOK 		pHook,
	TIMER_CB_UPDATE 	*pfUpdate );


#endif /* __TIMER_DOC_H__ */


