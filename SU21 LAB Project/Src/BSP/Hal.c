/*****************************************************************************
 @Project		: SEM2306
 @File 			: Hal.c
 @Details  	: All Ports and peripherals configuration                    
 @Author		: fongfh
 @Hardware	: Tiva LaunchPad 
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  Name     XXXX-XX-XX  		Initial Release
   
******************************************************************************/
#include <Common.h>
#include "Hal.h"


/*****************************************************************************
 Define
******************************************************************************/


/*****************************************************************************
 Type definition
******************************************************************************/


/*****************************************************************************
 Global Variables
******************************************************************************/


/*****************************************************************************
 Local Variables
******************************************************************************/


/*****************************************************************************
 Implementation
******************************************************************************/

void Port_Init( void )
{
	/* Enable Clock for SSI0 module */
	SYSCTL->RCGCSSI |= SYSCTL_RCGCSSI_R0;
	/* enable clock to UART0   */
	SYSCTL->RCGCUART |= SYSCTL_RCGCUART_R0;
	
	/* Waiting clock ready */
	while( 0 == (SYSCTL->PRSSI & SYSCTL_PRSSI_R0) );
	/* Wait for UART to be ready */
	while( 0 == (SYSCTL->PRUART & SYSCTL_PRUART_R0) ); 
	
	/* Enable Clocks Gate for Timers */
	SYSCTL->RCGCTIMER	|= SYSCTL_RCGCTIMER_R0  
					| SYSCTL_RCGCTIMER_R1 
					| SYSCTL_RCGCTIMER_R2 
					| SYSCTL_RCGCTIMER_R3 
					| SYSCTL_RCGCTIMER_R4 
					| SYSCTL_RCGCTIMER_R5; 
	
	/* Waiting clock ready */
	while( 0 == (SYSCTL->PRTIMER & SYSCTL_PRTIMER_R0) );
	while( 0 == (SYSCTL->PRTIMER & SYSCTL_PRTIMER_R1) );
	while( 0 == (SYSCTL->PRTIMER & SYSCTL_PRTIMER_R2) );
	while( 0 == (SYSCTL->PRTIMER & SYSCTL_PRTIMER_R3) );
	
	/* Enable Clocks Gate for GPIO */
	SYSCTL->RCGCGPIO |= SYSCTL_RCGCGPIO_R0
					 | SYSCTL_RCGCGPIO_R1
					 | SYSCTL_RCGCGPIO_R2
					 | SYSCTL_RCGCGPIO_R3
					 | SYSCTL_RCGCGPIO_R4
					 | SYSCTL_RCGCGPIO_R5;
	
	/* Waiting clock ready */
	while( 0 == (SYSCTL->PRGPIO & SYSCTL_PRGPIO_R0) );
	while( 0 == (SYSCTL->PRGPIO & SYSCTL_PRGPIO_R1) );
	while( 0 == (SYSCTL->PRGPIO & SYSCTL_PRGPIO_R2) );
	while( 0 == (SYSCTL->PRGPIO & SYSCTL_PRGPIO_R3) );
	while( 0 == (SYSCTL->PRGPIO & SYSCTL_PRGPIO_R4) );
	while( 0 == (SYSCTL->PRGPIO & SYSCTL_PRGPIO_R5) );
	
	/* Verification Output */
	GPIOB->DIR |= BIT(PB_T3CCP0);
	GPIOB->DEN |= BIT(PB_T3CCP0);	
	GPIOB->AFSEL &= ~BIT(PB_T3CCP0);
	
	/* Enable Clocks Gate for I2C0 */
	SYSCTL->RCGCI2C |= SYSCTL_PRI2C_R0;

	/* Wait of clock to be ready */
	while( 0 == (SYSCTL->PRI2C & SYSCTL_PRI2C_R0) );
	
		/* Enable Clocks Gate for ADC0 */
	SYSCTL->RCGCADC |= SYSCTL_RCGCADC_R0; 
	
	/* Wait of clock to be ready */
	while( 0 == (SYSCTL->PRADC & SYSCTL_PRADC_R0) );
	
	/* Configure PE4 - AIN9 and PE5 - AIN8 as Analog In */
	GPIOE->CR |= (BIT(PE_ADC_AIN8) | BIT(PE_ADC_AIN9));
	GPIOE->AFSEL |= (BIT(PE_ADC_AIN8) | BIT(PE_ADC_AIN9));
	GPIOE->DEN &= ~(BIT(PE_ADC_AIN8) | BIT(PE_ADC_AIN9));
	GPIOE->AMSEL |= (BIT(PE_ADC_AIN8) | BIT(PE_ADC_AIN9));

	/* Configure PB4 - AIN10 as Analog In */
	GPIOB->CR |= BIT(PB_ADC_AIN10);
	GPIOB->AFSEL |= BIT(PB_ADC_AIN10);
	GPIOB->DEN &= ~BIT(PB_ADC_AIN10);
	GPIOB->AMSEL |= BIT(PB_ADC_AIN10);

	/* LED */
	GPIOF->CR |= BIT(PF_LED_RED) | BIT(PF_LED_GREEN) | BIT(PF_LED_BLUE);
	GPIOF->DIR |= BIT(PF_LED_RED) | BIT(PF_LED_GREEN) | BIT(PF_LED_BLUE);
	GPIOF->DEN |= BIT(PF_LED_RED) | BIT(PF_LED_GREEN) | BIT(PF_LED_BLUE);
	GPIOF->AFSEL &= ~(BIT(PF_LED_RED) | BIT(PF_LED_GREEN) | BIT(PF_LED_BLUE));

	/* Enable Buzzer (PA4) pin as output */
	GPIOA->DIR |= BIT(PA_BUZZER);
	GPIOA->DEN |= BIT(PA_BUZZER);	
	
	/* User button SW1 & SW2 */
	GPIOF->LOCK = GPIO_LOCK_KEY;
	
	GPIOF->CR |= (BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->DIR &= ~(BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->DEN |= (BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->AFSEL &= ~(BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->PCTL &= ~(BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->PUR |= (BIT(PF_SW1) | BIT(PF_SW2));
	
	GPIOF->IM &= ~(BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->IBE &= ~(BIT(PF_SW1) | BIT(PF_SW2)); /* Disable both edge */
	GPIOF->IS &= ~(BIT(PF_SW1) | BIT(PF_SW2)); /* edge detection */
	GPIOF->IEV = ~(BIT(PF_SW1) | BIT(PF_SW2)); /* Falling edge */
	GPIOF->IM |= (BIT(PF_SW1) | BIT(PF_SW2));
	
	/* SSI & LCD */
	GPIOA->LOCK = GPIO_LOCK_KEY;
	
	GPIOA->DIR |= BIT(PA_LCD_SSI0_CS) | BIT(PA_LCD_DC) | BIT(PA_LCD_RESET);
	GPIOA->DEN |= BIT(PA_LCD_SSI0_CS) | BIT(PA_LCD_DC) | BIT(PA_LCD_RESET);
	GPIOA->AFSEL &= ~(BIT(PA_LCD_SSI0_CS) | BIT(PA_LCD_DC) | BIT(PA_LCD_RESET));
	GPIOA->PCTL &= ~( GPIO_PCTL_PA3_M | GPIO_PCTL_PA6_M | GPIO_PCTL_PA7_M );
	
	GPIOA->DEN |= BIT(PA_LCD_SSI0_SCK) | BIT(PA_LCD_SSI0_MOSI);
	GPIOA->AFSEL |= (BIT(PA_LCD_SSI0_SCK) | BIT(PA_LCD_SSI0_MOSI) );
	GPIOA->PCTL &= ~(GPIO_PCTL_PA2_M | GPIO_PCTL_PA5_M);
	GPIOA->PCTL |= (GPIO_PCTL_PA2_SSI0CLK | GPIO_PCTL_PA5_SSI0TX);
	
	GPIOB->DIR |= BIT(PB_LCD_BL);
	GPIOB->DEN |= BIT(PB_LCD_BL);
	
	/* Configure PB2 & PB3 as I2C0 */
	GPIOB->CR |= BIT(PB_I2C_SCL) | BIT(PB_I2C_SDA);
	GPIOB->AFSEL |= BIT(PB_I2C_SCL) | BIT(PB_I2C_SDA);
	GPIOB->DEN |= BIT(PB_I2C_SCL) | BIT(PB_I2C_SDA);
	GPIOB->DIR |= BIT(PB_I2C_SCL) | BIT(PB_I2C_SDA);
	GPIOB->DATA &= ~(BIT(PB_I2C_SCL) | BIT(PB_I2C_SDA));
	GPIOB->ODR |= BIT(PB_I2C_SDA);
	GPIOB->PCTL |= GPIO_PCTL_PB2_I2C0SCL | GPIO_PCTL_PB3_I2C0SDA;
	GPIOB->DR8R |= BIT(PB_I2C_SCL) | BIT(PB_I2C_SDA);
	
	/* initialize GPIO PA0 (UART0_RX) & PA1 (UART0_TX)   */
	GPIOA->LOCK = GPIO_LOCK_KEY; 
	GPIOA->CR |= BIT(PA_UART0_RX) | BIT(PA_UART0_TX); 
	
	GPIOA->AFSEL |= BIT(PA_UART0_RX) | BIT(PA_UART0_TX);
	GPIOA->DEN |= BIT(PA_UART0_RX) | BIT(PA_UART0_TX);
	GPIOA->AMSEL &= ~( BIT(PA_UART0_RX) | BIT(PA_UART0_TX) );
	GPIOA->PCTL &= ~( GPIO_PCTL_PA0_M | GPIO_PCTL_PA1_M ); /* clear Port C config bits  */
	GPIOA->PCTL |= GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX;
	
	/** initialize UART0   **/	
	UART0->CTL &= ~UART_CTL_UARTEN; 					/* disable UART during initialization  	*/
	UART0->CC &= ~UART_CC_CS_M; 							//  clock source mask
	UART0->CC |= UART_CC_CS_SYSCLK; 					// set to system clock
	
	UART0->CTL &= ~UART_CTL_HSE;											/* use 16X CLKDIV  							*/
	UART0->IBRD = 5;	 					/* int (80,000,000 / (16 * 921,600)) = 5.425347    */
	UART0->FBRD = 27;  					/* int (5.425347 * 64 + 0.5 = int (27.72) = 27   */
	
	UART0->LCRH &= ~UART_LCRH_WLEN_M; 	
	UART0->LCRH |= UART_LCRH_WLEN_7 | UART_LCRH_PEN; 	/* 7 data bits, parity enable  	*/
	UART0->LCRH |= UART_LCRH_EPS ;   									/* even parity  								*/ 	
	UART0->LCRH &= ~UART_LCRH_STP2;  									/* 1 stop bit 									*/ 
	
	UART0->LCRH |= UART_LCRH_FEN; 										/* enable FIFO*/
	
	UART0->CTL |= UART_CTL_TXE; 											/* transmit enable 							*/
	UART0->CTL |= UART_CTL_UARTEN; 										/* enable UART0   							*/
	
	/* User button SW1 & SW2 */
	GPIOF->LOCK = GPIO_LOCK_KEY;
	
	GPIOF->CR |= (BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->DIR &= ~(BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->DEN |= (BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->AFSEL &= ~(BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->PCTL &= ~(BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->PUR |= (BIT(PF_SW1) | BIT(PF_SW2));
	
	GPIOF->IM &= ~(BIT(PF_SW1) | BIT(PF_SW2));
	GPIOF->IBE &= ~(BIT(PF_SW1) | BIT(PF_SW2)); /* Disable both edge */
	GPIOF->IS &= ~(BIT(PF_SW1) | BIT(PF_SW2)); /* edge detection */
	GPIOF->IEV = ~(BIT(PF_SW1) | BIT(PF_SW2)); /* Falling edge */
	GPIOF->IM |= (BIT(PF_SW1) | BIT(PF_SW2));
	
	/* Keypad 3x4 matrix */
	GPIOD->DIR |= BIT(PD_KEYPAD_ROW0) | BIT(PD_KEYPAD_ROW1) | BIT(PD_KEYPAD_ROW2) | BIT(PD_KEYPAD_ROW3);
	GPIOD->DEN |=  BIT(PD_KEYPAD_ROW0) | BIT(PD_KEYPAD_ROW1) | BIT(PD_KEYPAD_ROW2) | BIT(PD_KEYPAD_ROW3);
	GPIOD->AFSEL &= ~(BIT(PD_KEYPAD_ROW0) | BIT(PD_KEYPAD_ROW1) | BIT(PD_KEYPAD_ROW2) | BIT(PD_KEYPAD_ROW3));
	GPIOD->PCTL &= ~( GPIO_PCTL_PD0_M | GPIO_PCTL_PD1_M | GPIO_PCTL_PD2_M | GPIO_PCTL_PD3_M );

	GPIOE->DIR &= ~(BIT(PE_KEYPAD_COL0) | BIT(PE_KEYPAD_COL1) | BIT(PE_KEYPAD_COL2));
	GPIOE->DEN |= (BIT(PE_KEYPAD_COL0) | BIT(PE_KEYPAD_COL1) | BIT(PE_KEYPAD_COL2));
	GPIOE->AFSEL &= ~(BIT(PE_KEYPAD_COL0) | BIT(PE_KEYPAD_COL1) | BIT(PE_KEYPAD_COL2));
	GPIOE->PCTL &= ~( GPIO_PCTL_PE2_M | GPIO_PCTL_PE3_M | GPIO_PCTL_PE4_M );
	GPIOE->PUR |= (BIT(PE_KEYPAD_COL0) | BIT(PE_KEYPAD_COL1) | BIT(PE_KEYPAD_COL2));
	
	/* Initialize Port C4-C7 for STEPPER output */
	GPIOC->DIR |= (BIT(PC_STEPPER0) | BIT(PC_STEPPER1) | BIT(PC_STEPPER2) | BIT(PC_STEPPER3));
	GPIOC->DEN |= (BIT(PC_STEPPER0) | BIT(PC_STEPPER1) | BIT(PC_STEPPER2) | BIT(PC_STEPPER3));
	GPIOC->AFSEL &= ~(BIT(PC_STEPPER0) | BIT(PC_STEPPER1) | BIT(PC_STEPPER2) | BIT(PC_STEPPER3));
	GPIOC->PCTL &= ~( GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M | GPIO_PCTL_PC6_M | GPIO_PCTL_PC7_M );
	
	
	
	/** setup GPIO pins in alternate mode **/
	// other GPIO initialization codes
	GPIOB->AFSEL |= BIT(PB6_TESTPIN); /* enable alternate function*/
	GPIOB->PCTL &= ~GPIO_PCTL_PB6_M; /* reset mask for PB6 */
	GPIOB->PCTL |= GPIO_PCTL_PB6_T0CCP0; /* enable timer 0 in PB6 */
	

		
	/** setup Timer 0A in 16-bits operation **/
	TIMER0->CTL &= ~TIMER_CTL_TAEN; /* 1. disable timer 0 during setup */
	TIMER0->CFG |= TIMER_CFG_16_BIT; /* 2. set to 16-bit mode */
	TIMER0->TAMR |= TIMER_TAMR_TAMR_PERIOD ; /* 3. periodic mode */
	TIMER0->TAMR &= ~TIMER_TAMR_TACDIR; /* 4. count down */
	TIMER0->TAILR = 0xFDE8 ; /* 5. reload value; Interval 39*2 = 78ms */
	TIMER0->TAPR = 0x5F; /* 6. prescaler (8 bits) */
	TIMER0->IMR |= TIMER_IMR_TATOIM; /* 7. enable timeout intr */
	TIMER0->ICR |= TIMER_ICR_TATOCINT; /* 8. clear timeout flag */
	TIMER0->CTL |= TIMER_CTL_TAEN; /* 9. enable timer 0 */
		
	/* Initialize Port PB5 as TESTPIN output */
	GPIOB->DIR |= BIT(PB5);
	GPIOB->DEN |= BIT(PB5);
	GPIOB->AFSEL &= ~BIT(PB5);
	
	/* Initialize Port PB6 as TESTPIN output */
	/* Reason why PB6 may not work as I/O pin for some may be that PB6 was initialised I/O pin */
	/* before Timer0A was initialised, might be something about resetting of the bits */
	GPIOB->DIR |= BIT(PB6_TESTPIN);
	GPIOB->DEN |= BIT(PB6_TESTPIN);
	GPIOB->AFSEL &= ~BIT(PB6_TESTPIN);
	
}
	
/** Write an ASCII character to UART0				**/
/** character = ASCII to write 							**/
void write_ASCII_UART0 (char character )
{
	while( 0 != (UART0->FR & UART_FR_TXFF) ){}; 	/* wait if TX FIFO full					*/
	UART0->DR = character; 												/* write character to UART data reg */
}
















