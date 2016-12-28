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


	LCD_Configuration();
	LCD_Initialization();

	u32 delay;





	LCD_Test();

	while(1)
	{
		LCD_Clear(LCD_Blue);
		/* make some float calculations */
		float x = 42,y = 23,z = 7;
		int i = 0;
		for (i = 0;i<6;i++)
		{
			z = (x*y)/z;
		};

		/* GPIO PA0 set, pin=high,LED2 off */
		GPIOA->BSRR = GPIO_BSRR_BS2;
		//GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_SET);

		/*delay ->> compiler optimizer settings must be "-O0" */
		delay = 500000;
		while(delay)
		{
			delay--;
		}

		/* GPIO PA0 reset pin=low, LED2 on */
		GPIOA->BSRR = GPIO_BSRR_BR2;
		//GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_RESET);

		/* delay --> blah */
		delay = 500000;
		while(delay)
		{
			delay--;
		}
	}
}
