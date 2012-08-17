/*
 * This is a series of functions to communicate between an ILI9320 LCD 
 * controller and an STM32F1xx series ARM microcontroller. The required 
 * pinout can be found in the pinout.txt file at the root of the project.
 *
 * This code borrows a lot from other similar libraries, I hope it helps 
 * you as much as they have helped me.
 */

#include "stm32f1x.h"
#include "lcd_control.h"

u16 DeviceCode;

static LCD_OrientationMode_t orientation_mode = LCD_ORIENTATION_DEFAULT;

void LCD_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// Open the clock we want
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC,ENABLE);

	// Configure the LCD pins for push-pull output
	// This is just the lower 8 bits of data
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	// Now for the high 8 bits
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	// Control pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	// Backlight control
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	LCD_Light_On;
}

/*
 * Name: void LCD_Initialization()
 * Function: initialize ILI9320 controller(may work with 9325 and 9328, untested)
 * Input: none
 * Output: none
 * Call: LCD_Initialization();
 */
void LCD_Initialization()
{
	u16 i;

	// Set all bits
	LCD_WriteData(0xffff);
	// Set various pins
	Set_Rst;
	Set_nWr;
	Set_Cs;
	Set_Rs;
	Set_nRd;
	Set_Rst;
	LCD_Reset();	// reset LCD

	// Wait a bit
	for (i = 50000;i > 0;i--);
	for (i = 50000;i > 0;i--);
	for (i = 50000;i > 0;i--);
	LCD_WriteRegister(0x0000,0x0001);
	LCD_Delay(10000);
	for (i = 50000;i > 0;i--);
	for (i = 50000;i > 0;i--);
	LCD_DeviceCode = LCD_ReadRegister(0x0000);
	if (LCD_DeviceCode == 0x9320)
	{
		LCD_WriteRegister(0x00,0x0000);		// Start Oscillation
		LCD_WriteRegister(0x01,0x0100);		// Driver output control (makes it scan top left to bottom right, odd for top half, even for bottom)
		LCD_WriteRegister(0x02,0x0700);		// Line inversion on the LCD Driving Wave Control
		LCD_WriteRegister(0x03,0x1018);		// Entry mode set(may need changing
		LCD_WriteRegister(0x04,0x0000);		// Resizing stuff(not needed by me)
		LCD_WriteRegister(0x08,0x0202);		// (Display control 2) Set 2 lines for for both front and back porch(minimum number, helps VSYNC)
		LCD_WriteRegister(0x09,0x0000);		// (Display control 3) Sets 0 frame scan cycle(not needed for this communication) and normal scan
		LCD_WriteRegister(0x0a,0x0000);		// Not necessary(FMARK signal stuff)
		LCD_WriteRegister(0x0c,0x0001);		// 16-bit RGB interface(1 transfer/pixel), using internal system clock and system interface
		LCD_WriteRegister(0x0d,0x0000);		// Frame Marker at position 0(starts outputting frame right away
		LCD_WriteRegister(0x0f,0x0000);		// Data input on rising edge of DOTCLK, data written when ENABLE=0,HSPL/VSPL(H/VSYNC pins) Low polarit = active

		for (i = 50000;i > 0;i--);
		for (i = 50000;i > 0;i--);
		LCD_WriteRegister(0x07,0x0101);		// (Display control 1) Set display to operate, VCOM output to GND
		for (i = 50000;i > 0;i--);
		for (i = 50000;i > 0;i--);

		LCD_WriteRegister(0x10,0x10c0);		// (Power control 1) Enable power supply
		LCD_WriteRegister(0x11,0x0007);		// (Power control 2) Ref. voltage(VciOUT) = Vci1, operating frequency = Fosc in circuit 1 and Fosc/16 in circuit 2
		LCD_WriteRegister(0x12,0x0110);		// (Power control 3) Enable VGL output, use internal electric volume(VCM) to set VcomH(Vcom center voltage level
		LCD_WriteRegister(0x13,0x0a00);		// (Power control 4) VCOM Amplitude = VREG1OUT x 0.90
		LCD_WriteRegister(0x29,0x0000);		// (Power control 7) VcomH voltage = VREG1OUT x .69

		LCD_WriteRegister(0x2b,0x0008);		// 90Hz frame rate, internal resistor. (maybe should be 0x0408)

		// Horiz. and Vert. RAM Address Position. Set up for 320x240
		LCD_WriteRegister(0x50,0);			// X starts at 0
		LCD_WriteRegister(0x51,239);		// X ends at 239
		LCD_WriteRegister(0x52,0);			// Y starts at 0
		LCD_WriteRegister(0x53,319);		// Y ends at 319

		LCD_WriteRegister(0x60,0x2700);		// (Driver output control) Gate scans at 0, ends after 320 lines
		LCD_WriteRegister(0x61,0x0001);		// (Driver output control) grayscale inversion
		LCD_WriteRegister(0x62,0x0000);		// (Vertical scroll control) not used

		LCD_WriteRegister(0x80,0x0000);		// Display position for partial image 1
		LCD_WriteRegister(0x81,0x0000);		// RAM address for start of partial image 1
		LCD_WriteRegister(0x82,0x0000);		// RAM address for end of partial image 1
		LCD_WriteRegister(0x83,0x0000);		// Display position for partial image 2
		LCD_WriteRegister(0x84,0x0000);		// RAM address for start of partial image 2
		LCD_WriteRegister(0x85,0x0000);		// RAM address for end of partial image 2

		LCD_WriteRegister(0x90,0x0010);		// (Panel interface control 1) Sets clock number for internal clock
		LCD_WriteRegister(0x92,0x0000);		// (Panel interface control 2) 0 Clocks of overlap when synced
		LCD_WriteRegister(0x93,0x0001);		// (Panel interface control 3) 1 Clock output position when synced
		LCD_WriteRegister(0x95,0x0110);		// (Panel interface control 4) Division ration of 1/4, 16 clocks per horizontal line
		LCD_WriteRegister(0x97,0x0000);		// (Panel interface control 5) 0 Clock non-overlap
		LCD_WriteRegister(0x98,0x0000);		// (Panel interface control 6) Source output position 0 clocks

		LCD_WriteRegister(0x07,0x0133);		// (Display control 1) Set display to operate, base image display, normal display (maybe should be 0x0173)
	}
	for (i = 50000;i > 0;i--);
	LCD_Clear(LCD_Red);
}

/*
 * Name: void LCD_SetCursor(u16 x,u16 y)
 * Function: Set the screen coordinates
 * Input: x,y-positions
 * Output: none
 * Call: LCD_SetCursor(50,50);
 */
__inline void LCD_SetCursor(u16 x,u16 y)
{
	LCD_WriteRegister(0x20,y);		// Y position register
	LCD_WriteRegister(0x21,319 - x);	// X position register
}

/*
 * Name: void LCD_SetWindow(u16 Startx,u16 Starty,u16 Endx,u16 Endy)
 * Function: Set window region
 * Input: Start/end x,y-position of window
 * Output: none
 * Call: LCD_SetWindow(0,0,240,320);
 */
__inline void LCD_SetWindow(u16 Startx,u16 Starty,u16 Endx,u16 Endy)
{
	LCD_SetCursor(Startx,Starty);
	LCD_WriteRegister(0x50,Startx);
	LCD_WriteRegister(0x52,Starty);
	LCD_WriteRegister(0x51,Endx);
	LCD_WriteRegister(0x53,Endy);
}

/*
 * Name: void LCD_Clear(u16 Color)
 * Feature: fill screen to specified color 
 * Input: fill color
 * Output: none
 * Call: LCD_Clear (0xffff);
 */
void LCD_Clear(u16 Color)
{
	u32 i;
	LCD_SetCursor(0,0);
	Clr_Cs;
	LCD_WriteIndex(0x22);		// GRAM access port
	Set_Rs;
	for (i = 0;i < 76800;i++)
	{
		LCD_WriteData(Color);
		Clr_nWr;
		Set_nWr;
	}

	Set_Cs;
}

/*
 * Name: void LCD_SetPoint(u16 x,u16 y,u16 Color)
 * Function: Draw point at x,y
 * Input: x,y-position, point color
 * Output: None
 * Call: LCD_SetPoint(50,50,0x0000);
 */
void LCD_SetPoint(u16 x,u16 y,u16 Color)
{
	if ((x > 320) || (y > 240))
	{
		return;
	}
	LCD_SetCursor(x,y);
	Clr_Cs;
	LCD_WriteIndex(0x22);
	Set_Rs;
	LCD_WriteData(Color);
	Clr_nWr; Set_nWr;
	Set_Cs;
}

/*
 * Name: void LCD_DrawPicture(u16 Startx,u16 Starty,u16 Endx,u16 Endy,u16 *pic)
 * Function: display a picture
 * Input: starting/ending x,y-positions, picture head pointer
 * Output: none
 * Call: LCD_DrawPicture(0,0,100,100,(u16*) demo);
 */
void LCD_DrawPicture(u16 Startx,u16 Starty,u16 Endx,u16 Endy,u16 *pic)
{
	u16 i;
	LCD_SetCursor(Startx,Starty);
	Clr_cs;
	LCD_WriteIndex(0x0022);
	Set_Rs;
	for (i = 0;i < (Endx * Endy);i++)
	{
		LCD_WriteData(*pic++);
		Clr_nWr;Set_nWr;
	}
	Set_Cs;
}

/*
 * Name: void LCD_Test()
 * Function: test the LCD
 * Input: none
 * Output: none
 * Call: LCD_Test();
 */
void LCD_Test()
{
	u8 R_data,G_data,B_data,i,j;

	LCD_SetCursor(0,0);
	LCD_WriteRegister(0x50,0);		// Set horizontal start of window at 0
	LCD_WriteRegister(0x51,239);	// Set horizontal end of window at 239
	LCD_WriteRegister(0x52,0);		// Set vertical start of window at 0
	LCD_WriteRegister(0x53,319);	// Set vertical end of window at 319
	Clr_Cs;
	LCD_WriteIndex(0x22);
	Set_Rs;
	R_data = 0;G_data = 0;B_data = 0;
	/********** RED **********/
	for (j = 0;j < 50;j++)
	{
		for (i = 0;i < 240;i++)
		{
			R_data = i / 8;
			LCD_WriteData(R_data << 11 | G_data << 5 | B_data);
			Clr_nWr;Set_nWr;
		}
	}
	R_data = 0x1f;G_data = 0x3f;B_data = 0x1f;
	for (j = 0;j < 50;j++)
	{
		for (i = 0; i < 240;i++)
		{
			G_data = 0x3f - (i / 4);
			B_data = 0x1f - (i / 8);
			LCD_WriteData(R_data << 11 | G_data << 5 | B_data);
			Clr_nWr;Set_nWr;
		}
	}
	/********** GREEN **********/
	R_data = 0;G_data = 0;B_data = 0;
	for (j = 0;j < 50;j++)
	{
		for (i = 0;i < 240;i++)
		{
			G_data = i / 4;
			LCD_WriteData(R_data << 11 | G_data << 5 | B_data);
			Clr_nWr;Set_nWr;
		}
	}
	R_data = 0x1f;G_data = 0x3f;B_data = 0x1f;
	for (j = 0;j < 50;j++)
	{
		for (i = 0;i < 240;i++)
		{
			R_data = 0x1f - (i / 8);
			B_data = 0x1f - (i / 8);
			LCD_WriteData(R_data << 11 | G_data << 5 | B_data);
			Clr_nWr;Set_nWr;
		}
	}
	/********** BLUE **********/
	R_data = 0;G_data = 0;B_data = 0;
	for (j = 0;j < 60; j++)
	{
		for(i = 0;i < 240;i++)
		{
			B_data = i / 8;
			LCD_WriteData(R_data << 11 | G_data << 5 | B_data);
			Clr_nWr;Set_nWr;
		}
	}
	R_data = 0x1f;G_data = 0x3f;B_data = 0x1f;
	for (j = 0;j < 60;j++)
	{
		for (i = 0;i < 240;i++)
		{
			G_data = 0x3f - (i / 4);
			R_data = 0x1f - (i / 8);
			LCD_WriteData(R_data << 11 | G_data << 5 | B_data);
			Clr_nWr;Set_nWr;
		}
	}
	Set_Cs;
}

/*
 * Name: u16 LCD_BGR2RGB(u16 color)
 * Function: RRRRRGGGGGGBBBBB to BBBBBGGGGGGRRRRR format
 * Input: BRG color
 * Output: RGB color
 * Call: LCD_BGR2RGB(0xfefefe);
 */
u16 LCD_BGR2RGB(u16 color)
{
	u16 r,g,b,rgb;
	b = (color >> 0) & 0x1f;
	g = (color >> 5) & 0x3f;
	r = (color >> 11) & 0x1f;

	rgb = (b << 11) + (g << 5) + (r << 0);
	return(rgb);
}

/*
 * Name: void LCD_WriteIndex(u16 idx)
 * Function: write register address
 * Input: register
 * Output: none
 * Call: LCD_WriteIndex(0x20);
 */
__inline void LCD_WriteIndex(u16 idx)
{
	Clr_Rs;
	Set_nRd;
	LCD_WriteData(idx);
	Clr_nWr;
	Set_nWr;
	Set_Rs;
}

/*
 * Name: void LCD_WriteData(u16 data)
 * Function: write register data
 * Input: register data
 * Output: none
 * Call: LCD_WriteData(0x0000);
 */
void LCD_WriteData(u16 data)
{
	GPIOB->ODR = ((GPIOB->ODR&0x00ff) | (data << 8));
	GPIOC->ODR = ((GPIOC->ODR&0xff00) | (data >> 8));
}

/*
 * Name: u16 LCD_ReadData(void)
 * Function: read controller data
 * Input: none
 * Output: data
 * Call: x = LCD_ReadData();
 */
__inline u16 LCD_ReadData(void)
{
	u16 temp;
	GPIOB->CRH = (GPIOB->CRH & 0x00000000) | 0x44444444;		// configure pins for reading
	GPIOC->CRL = (GPIOC->CRL & 0x00000000) | 0x44444444;
	temp = (((GPIOB->IDR) >> 8) | ((GPIOC->IDR) << 8));			// read in the data
	GPIOB->CRH = (GPIOB->CRH & 0x00000000) | 0x44444444;		// reconfigure back to normal operation
	GPIOC->CRL = (GPIOC->CRL & 0x00000000) | 0x44444444;
	return temp;
}

/*
 * Name: u16 LCD_ReadRegister(u16 index)
 * Function Read the value of the address register
 * Input: index of the register you want to read
 * Output: Register value
 * Call: x = LCD_ReadRegister(0x22);
 */
__inline u16 LCD_ReadRegister(u16 index)
{
	Clr_Cs;
	LCD_WriteIndex(index);
	Clr_nRd;
	index = LCD_ReadData();
	Set_nRd;
	Set_Cs;
	return index;
}

/*
 * MAY BE WRONG
 * Name: void LCD_WriteRegister(u16 index,u16 data)
 * Function: Write data to register
 * Input: index of register and data to go there
 * Output: none
 * Call: LCD_WriteRegister(0x00,0x0001);
 */
__inline void LCD_WriteRegister(u16 index,u16 data)
{
	Clr_Cs;
	Clr_Rs;
	Set_nRd;
	LCD_WriteData(index);

	Clr_nWr;Set_nWr;
	Set_Rs;
	LCD_WriteData(data);

	Clr_nWr;Set_nWr;
	Set_Cs;
}

/*
 * Name: void LCD_Reset()
 * Function: reset ili9320 controller
 * Input: none
 * Output: none
 * Call: LCD_Reset();
 */
void LCD_Reset()
{
	Set_Rst;
	LCD_Delay(50000);
	Clr_Rst;
	LCD_Delay(50000);
	Set_Rst;
	LCD_Delay(50000);
}

/*
 * Name: LCD_Backlight(bool status)
 * Function: turn on or off backlight
 * Input: 1 for on, 0 for off
 * Output: none
 * Call: LCD_Backlight(1);
 */
void LCD_Backlight(bool status)
{
	if(status)
	{
		LCD_Light_On;
	}
	else
	{
		LCD_Light_Off;
	}
}

/*
 * Name: void LCD_Delay(vu32 nCount)
 * Function: Delay nCount many counts
 * Input: nCount, number of counts
 * Output: none
 * Call: LCD_Delay(50000);
 */
void LCD_Delay(vu32 nCount)
{
	for(;nCount != 0;nCount--);
}

/*
 * Name: void LCD_SetOrienatation(LCD_OrientationMode_t m)
 * Function: sets the orientation of the LCD
 * Input: orientation mode
 * Output: none
 * Call: LCD_SetOrientation(mode);
 */
void LCD_SetOrientation(LCD_OrientationMode_t m)
{
	uint16_t em;
	switch (m)
	{
		case LCD_PORTRAIT_TOP_DOWN:
			em = 0x1030;
			break;
		case LCD_PORTRAIT_BOTTOM_UP:
			em = 0x1010;
			break;
		case LCD_LANDSCAPE_TOP_DOWN:
			em = 0x1018;
			break;
		case LCD_LANDSCAPE_BOTTOM_UP:
			em = 0x1008;
			break;
		default:
			em = 0x0130;
			break;
	}
	LCD_WriteRegister(0x0003,em);
	orientation_mode = m;
	LCD_SetCursor(0,0);
}

/*
 * Name: LCD_OrientationMode_t LCD_GetOrientation(void)
 * Function: returns the current orientation of the LCD
 * Input: none
 * Output: orientation
 * Call: x = LCD_GetOrientation();
 */
LCD_Orientation_Mode_t LCD_GetOrientation(void)
{
	return orientation_mode;
}

/*
 * Name: uint16_t LCD_GetWidth(void)
 * Function: Get the width of the LCD in pixels in the current orientation
 * Input: none
 * Output: width
 * Call: x = LCD_GetWidth();
 */
uint16_t LCD_GetWidth(void)
{
	switch (orientation_mode)
	{
		case LCD_LANDSCAPE_TOP_DOWN:
		case LCD_LANDSCAPE_BOTTOM_UP:
			return LCD_HEIGHT_HW;
		case LCD_PORTRAIT_TOP_DOWN:
		case LCD_PORTRAIT_BOTTOM_UP:
		default:
			return LCD_WIDTH_HW;
	}
}

/*
 * Name: uint16_t LCD_GetHeight(void)
 * Function: Get the height of the LCD in pixels in the current orientation
 * Input: none
 * Output: height
 * Call: y = LCD_GetHeight();
 */
uint16_t LCD_GetHeight(void)
{
	switch (orientation_mode)
	{
		case LCD_LANDSCAPE_TOP_DOWN:
		case LCD_LANDSCAPE_BOTTOM_UP:
			return LCD_WIDTH_HW;
		case LCD_PORTRAIT_TOP_DOWN:
		case LCD_PORTRAIT_BOTTOM_UP:
		default:
			return LCD_HEIGHT_HW;
	}
}
