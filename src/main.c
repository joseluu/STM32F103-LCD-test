/*
 * A quick test program to test the LCD on the dev board mentioned in 
 * the readme. Also blinks an LED and does some floating point calcs for 
 * fun and testing :D.
 * 
 * If this helps you do something cool let me know!
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "lcd_control.h"

int main(int argc, char *argv[])
{

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
  /* Enable hi resolution counter */
	DWT->CTRL &= ~0x00000001;
	DWT->CTRL |= 0x00000001;

		Clr_Sync;       // Flag de synchro sur bouton 3
	LCD_Configuration();

	
	LCD_Initialization();
	

	u32 delay;


//  	Set_Sync;       // Flag de synchro sur bouton 3
//		LCD_Delay(1);
//		Clr_Sync;       // Flag de synchro sur bouton 3
		LCD_Clear (0x0000);     // Black
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);			
		LCD_Clear (0x001F);			// Blue
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);			
		LCD_Clear (0xF800);			// Red
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);			
		LCD_Clear (0x07E0);			// Green 
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);			
		

		LCD_Test();
	
		Set_Sync;       // Flag de synchro sur bouton 3
		LCD_Delay(1);
		Clr_Sync;       // Flag de synchro sur bouton 3
//		Clr_Rs
//		LCD_Delay(1);
//		Set_Rs;
		
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		
		LCD_Clear (0xFFFF);     // White
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);	
		LCD_Delay(50000);		
		LCD_Delay(50000);			
		

		LCD_Test();
		

		
//		Set_Sync;       // Flag de synchro sur bouton 3
//		Clr_Sync;       // Flag de synchro sur bouton 3

	while(1)
	{


	
	}
}
