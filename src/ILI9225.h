/*
 * ILI9225.h
 *
 *  Created on: Nov 25, 2018
 *      Author: kasik
 */

#ifndef SRC_ILI9225_H_
#define SRC_ILI9225_H_

// control pins
#define dispCtrlPort			GPIOA
#define RESET					GPIO_Pin_6
#define RS						GPIO_Pin_7
#define LED						GPIO_Pin_5

// ILI9225 screen size
#define LCD_WIDTH  				176
#define LCD_HEIGHT 				220

void ILI9225_init(void);
void ILI9225_setOrientation(uint8_t orientation);
void ILI9225_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ILI9225_drawPixel(uint16_t x1, uint16_t y1, uint16_t color);
void ILI9225_drawBitmap(uint16_t x0, uint16_t y0, uint16_t columns, uint16_t rows, uint16_t *data);

void ILI9225_initBitmapDMA(uint16_t *data, uint16_t columns, uint16_t rows);
void ILI9225_startBitmapDMA(void);
void ILI9225_reconfBitmapDMA(uint16_t *data, uint16_t columns, uint16_t rows);


#endif /* SRC_ILI9225_H_ */
