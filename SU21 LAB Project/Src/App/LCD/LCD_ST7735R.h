/*****************************************************************************
 @Project	: 
 @File 		: 
 @Details  	:                   
 @Author	: 
 @Hardware	: 
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  Name     XXXX-XX-XX  		Initial Release
   
******************************************************************************/

#ifndef __LCD_DRIVER_DOT_H__
#define __LCD_DRIVER_DOT_H__


/*****************************************************************************
 Define
******************************************************************************/


/*****************************************************************************
 Type definiton
******************************************************************************/
typedef void LCD_CB_DONE( void );

typedef enum _tagLCD_ORIENTATION
{
	LCD_POTRAIT,
	LCD_LANDSCAPE,
	LCD_POTRAIT_180,
	LCD_LANDSCAPE_180
}
LCD_ORIENTATION;


/*****************************************************************************
 Macro
******************************************************************************/


/******************************************************************************
 Global functions
******************************************************************************/


/******************************************************************************
 @Description 	: 

 @param			: 
 
 @revision		: 1.0.0
 
******************************************************************************/
void LcdInit( PSPIM_HANDLE pSpimHAndle, LCD_ORIENTATION Orientation );


/******************************************************************************
 @Description 	: 

 @param			: 
 
 @revision		: 1.0.0
 
******************************************************************************/
void LCD_GetSize( int *pScreenX, int *pScreenY );


/******************************************************************************
 @Description 	: 

 @param			: 
 
 @revision		: 1.0.0
 
******************************************************************************/
void LCD_Reset( void );


/******************************************************************************
 @Description 	: 

 @param			: 
 
 @revision		: 1.0.0
 
******************************************************************************/
void LCD_WriteData8( uint8_t Data );


/******************************************************************************
 @Description 	: 

 @param			: 
 
 @revision		: 1.0.0
 
******************************************************************************/
void LCD_WriteDataPixel( void const *pData, int nSize );



/******************************************************************************
 @Description 	: 

 @param			: 
 
 @revision		: 1.0.0
 
******************************************************************************/
void LCD_AddCallback( LCD_CB_DONE *pDone );

#endif /* __LCD_DRIVER_DOT_H__ */









