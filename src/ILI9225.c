/*
 * ILI9225.c
 *
 *  Created on: Nov 25, 2018
 *      Author: kasik
 */

#include "stm32f4xx_conf.h"
#include "ILI9225.h"
#include "spi.h"
#include "delay.h"
#include "ILI9225_registers.h"
#include "ILI9225_colours.h"

uint8_t  ILI9225_orientation = 0;
uint8_t  ILI9225_maxX = 0;
uint8_t  ILI9225_maxY = 0;


void ILI9225_initCtrlGPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //how to make this generic?
	RCC_ClearFlag();

	// Configure pins in alternate function
	GPIO_InitStructure.GPIO_Pin = RESET|RS|LED;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(dispCtrlPort, &GPIO_InitStructure);

	GPIO_WriteBit(dispCtrlPort, RESET, Bit_SET);
	GPIO_WriteBit(dispCtrlPort, RS, Bit_RESET);
	GPIO_WriteBit(dispCtrlPort, LED, Bit_SET);
}

void ILI9225_reset(void)
{
	GPIO_WriteBit(dispCtrlPort, RESET, Bit_SET);
	Delay(1);
	GPIO_WriteBit(dispCtrlPort, RESET, Bit_RESET);
	Delay(10);
	GPIO_WriteBit(dispCtrlPort, RESET, Bit_SET);
	Delay(50);
}

void ILI9225_writeIndex(uint16_t address, int cs)
{
	if(cs)
		GPIO_WriteBit(CS_port, CS, Bit_RESET);

	GPIO_WriteBit(dispCtrlPort, RS, Bit_RESET);

	spi_write16(address, 0);

	if(cs)
		GPIO_WriteBit(CS_port, CS, Bit_SET);
}

void ILI9225_writeRegister(uint16_t data, int cs)
{
	if(cs)
		GPIO_WriteBit(CS_port, CS, Bit_RESET);

	GPIO_WriteBit(dispCtrlPort, RS, Bit_SET);

	spi_write16(data, 0);

	if(cs)
		GPIO_WriteBit(CS_port, CS, Bit_SET);
}


void ILI9225_write(uint16_t address, uint16_t data)
{
	GPIO_WriteBit(CS_port, CS, Bit_RESET);

	ILI9225_writeIndex(address, 0);
	ILI9225_writeRegister(data, 0);

	GPIO_WriteBit(CS_port, CS, Bit_SET);
}


void ILI9225_setOrientation(uint8_t orientation)
{

	ILI9225_orientation = orientation % 4;

    switch (ILI9225_orientation) {
    case 0:
    	ILI9225_maxX = LCD_WIDTH;
    	ILI9225_maxY = LCD_HEIGHT;
        break;
    case 1:
    	ILI9225_maxX = LCD_HEIGHT;
    	ILI9225_maxY = LCD_WIDTH;
        break;
    case 2:
    	ILI9225_maxX = LCD_WIDTH;
    	ILI9225_maxY = LCD_HEIGHT;
        break;
    case 3:
    	ILI9225_maxX = LCD_HEIGHT;
    	ILI9225_maxY = LCD_WIDTH;
        break;
    }
}


void ILI9225_init(void)
{
	ILI9225_initCtrlGPIO();
	Delay(50);
	ILI9225_reset();

/* Start Initial Sequence */
   /* Set SS bit and direction output from S528 to S1 */
	ILI9225_write(POWER_CTRL1, 0x0000); // Set SAP,DSTB,STB
	ILI9225_write(POWER_CTRL2, 0x0000); // Set APON,PON,AON,VCI1EN,VC
	ILI9225_write(POWER_CTRL3, 0x0000); // Set BT,DC1,DC2,DC3
	ILI9225_write(POWER_CTRL4, 0x0000); // Set GVDD
	ILI9225_write(POWER_CTRL5, 0x0000); // Set VCOMH/VCOML voltage
	Delay(40);

   // Power-on sequence
	ILI9225_write(POWER_CTRL2, 0x0018); // Set APON,PON,AON,VCI1EN,VC
	ILI9225_write(POWER_CTRL3, 0x6121); // Set BT,DC1,DC2,DC3
	ILI9225_write(POWER_CTRL4, 0x006F); // Set GVDD   /*007F 0088 */
	ILI9225_write(POWER_CTRL5, 0x495F); // Set VCOMH/VCOML voltage
	ILI9225_write(POWER_CTRL1, 0x0800); // Set SAP,DSTB,STB
	Delay(10);
	ILI9225_write(POWER_CTRL2, 0x103B); // Set APON,PON,AON,VCI1EN,VC
	Delay(50);

	ILI9225_write(DRIVER_OUTPUT_CTRL, 0x011C); // set the display line number and display direction
	ILI9225_write(LCD_AC_DRIVING_CTRL, 0x0100); // set 1 line inversion
	ILI9225_write(ENTRY_MODE, 0x1030); // set GRAM write direction and BGR=1. //0x1030
	ILI9225_write(DISP_CTRL1, 0x0000); // Display off
	ILI9225_write(BLANK_PERIOD_CTRL1, 0x0808); // set the back porch and front porch
	ILI9225_write(FRAME_CYCLE_CTRL, 0x1100); // set the clocks number per line
	ILI9225_write(INTERFACE_CTRL, 0x0000); // CPU interface
	ILI9225_write(OSC_CTRL, 0x0D01); // Set Osc  /*0e01*/
	ILI9225_write(VCI_RECYCLING, 0x0020); // Set VCI recycling
	ILI9225_write(RAM_ADDR_SET1, 0x0000); // RAM Address
	ILI9225_write(RAM_ADDR_SET2, 0x0000); // RAM Address

/* Set GRAM area */
	ILI9225_write(GATE_SCAN_CTRL, 0x0000);
	ILI9225_write(VERTICAL_SCROLL_CTRL1, 0x00DB);
	ILI9225_write(VERTICAL_SCROLL_CTRL2, 0x0000);
	ILI9225_write(VERTICAL_SCROLL_CTRL3, 0x0000);
	ILI9225_write(PARTIAL_DRIVING_POS1, 0x00DB);
	ILI9225_write(PARTIAL_DRIVING_POS2, 0x0000);
	ILI9225_write(HORIZONTAL_WINDOW_ADDR1, 0x00AF);
	ILI9225_write(HORIZONTAL_WINDOW_ADDR2, 0x0000);
	ILI9225_write(VERTICAL_WINDOW_ADDR1, 0x00DB);
	ILI9225_write(VERTICAL_WINDOW_ADDR2, 0x0000);

/* Set GAMMA curve */
	ILI9225_write(GAMMA_CTRL1, 0x0000);
	ILI9225_write(GAMMA_CTRL2, 0x0808);
	ILI9225_write(GAMMA_CTRL3, 0x080A);
	ILI9225_write(GAMMA_CTRL4, 0x000A);
	ILI9225_write(GAMMA_CTRL5, 0x0A08);
	ILI9225_write(GAMMA_CTRL6, 0x0808);
	ILI9225_write(GAMMA_CTRL7, 0x0000);
	ILI9225_write(GAMMA_CTRL8, 0x0A00);
	ILI9225_write(GAMMA_CTRL9, 0x0710);
	ILI9225_write(GAMMA_CTRL10, 0x0710);

	ILI9225_write(DISP_CTRL1, 0x0012);
	Delay(50);
	ILI9225_write(DISP_CTRL1, 0x1017);


	ILI9225_setOrientation(0);

	// Initialize variables
	//setBackgroundColor(COLOR_BLACK);

	ILI9225_clear();
}

void ILI9225_swap(uint16_t *a, uint16_t *b)
{
    uint16_t w = *a;
    *a = *b;
    *b = w;
}

void ILI9225_orientCoordinates(uint16_t *x1, uint16_t *y1)
{

    switch (ILI9225_orientation) {
    case 0:  // ok
        break;
    case 1: // ok
        *y1 = ILI9225_maxY - *y1 - 1;
        ILI9225_swap(x1, y1);
        break;
    case 2: // ok
        *x1 = ILI9225_maxX - *x1 - 1;
        *y1 = ILI9225_maxY - *y1 - 1;
        break;
    case 3: // ok
        *x1 = ILI9225_maxX - *x1 - 1;
        ILI9225_swap(x1, y1);
        break;
    }
}

void ILI9225_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	ILI9225_orientCoordinates(&x0, &y0);
	ILI9225_orientCoordinates(&x1, &y1);

    if (x1 < x0) ILI9225_swap(&x0, &x1);
    if (y1 < y0) ILI9225_swap(&y0, &y1);

    ILI9225_write(HORIZONTAL_WINDOW_ADDR1, x1);
    ILI9225_write(HORIZONTAL_WINDOW_ADDR2, x0);

    ILI9225_write(VERTICAL_WINDOW_ADDR1, y1);
    ILI9225_write(VERTICAL_WINDOW_ADDR2, y0);

    ILI9225_write(RAM_ADDR_SET1, x0);
    ILI9225_write(RAM_ADDR_SET2, y0);

    ILI9225_writeIndex(0x0022, 1);
}


void ILI9225_drawPixel(uint16_t x1, uint16_t y1, uint16_t color)
{
    if ((x1 >= ILI9225_maxX) || (y1 >= ILI9225_maxY)) return;

    ILI9225_orientCoordinates(&x1, &y1);
    ILI9225_write(RAM_ADDR_SET1,x1);
    ILI9225_write(RAM_ADDR_SET2,y1);
    ILI9225_write(GRAM_DATA_REG,color);
}

void ILI9225_drawBitmap(uint16_t x0, uint16_t y0, uint16_t columns, uint16_t rows, uint16_t *data)
{
	int i = 0;
	uint16_t size = rows*columns;
    if ((x0 >= ILI9225_maxX) || (y0 >= ILI9225_maxY) || (x0+columns-1 >= ILI9225_maxX) || (y0+rows-1 >= ILI9225_maxY)   ) return;

    ILI9225_orientCoordinates(&x0, &y0);
    ILI9225_setWindow(x0, y0, x0 + columns-1, y0 + rows-1);

    ILI9225_writeIndex(GRAM_DATA_REG,1);

    for(i=0; i< size; i++)
    {
//    	ILI9225_write(GRAM_DATA_REG,data[i]);
    	ILI9225_writeRegister(data[i],1);

    }
}

int stopDMA;

void ILI9225_startdBitmapDMA(uint16_t *data)
{
	int i = 0;
	uint16_t size = LCD_WIDTH*LCD_HEIGHT;

	spi_DMA_Init(data, size);
	ILI9225_setWindow(0, 0, LCD_WIDTH-1, LCD_HEIGHT-1); //set full screen window

	GPIO_WriteBit(dispCtrlPort, RS, Bit_SET);
	ILI9225_writeIndex(GRAM_DATA_REG,0);				//write gram reg address

	spi_DMA_Transfer();

	stopDMA = 0;
}


void ILI9225_stopdBitmapDMA(void)
{
	stopDMA = 1;
}


void ILI9225_clear(void)
{
	uint16_t size = LCD_WIDTH*LCD_HEIGHT;
	int i = 0;
	uint16_t buff[size];

	for (i = 0; i < size; i++)
	{
		buff[i] = COLOR_SNOW;
	}

	ILI9225_drawBitmap(0,0,LCD_WIDTH,LCD_HEIGHT, buff);
}
