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

// ILI9225 LCD Registers
#define DRIVER_OUTPUT_CTRL      0x01  // Driver Output Control
#define LCD_AC_DRIVING_CTRL     0x02  // LCD AC Driving Control
#define ENTRY_MODE              0x03  // Entry Mode
#define DISP_CTRL1              0x07  // Display Control 1
#define BLANK_PERIOD_CTRL1      0x08  // Blank Period Control
#define FRAME_CYCLE_CTRL        0x0B  // Frame Cycle Control
#define INTERFACE_CTRL          0x0C  // Interface Control
#define OSC_CTRL                0x0F  // Osc Control
#define POWER_CTRL1             0x10  // Power Control 1
#define POWER_CTRL2             0x11  // Power Control 2
#define POWER_CTRL3             0x12  // Power Control 3
#define POWER_CTRL4             0x13  // Power Control 4
#define POWER_CTRL5             0x14  // Power Control 5
#define VCI_RECYCLING           0x15  // VCI Recycling
#define RAM_ADDR_SET1           0x20  // Horizontal GRAM Address Set
#define RAM_ADDR_SET2           0x21  // Vertical GRAM Address Set
#define GRAM_DATA_REG           0x22  // GRAM Data Register
#define GATE_SCAN_CTRL          0x30  // Gate Scan Control Register
#define VERTICAL_SCROLL_CTRL1   0x31  // Vertical Scroll Control 1 Register
#define VERTICAL_SCROLL_CTRL2   0x32  // Vertical Scroll Control 2 Register
#define VERTICAL_SCROLL_CTRL3   0x33  // Vertical Scroll Control 3 Register
#define PARTIAL_DRIVING_POS1    0x34  // Partial Driving Position 1 Register
#define PARTIAL_DRIVING_POS2    0x35  // Partial Driving Position 2 Register
#define HORIZONTAL_WINDOW_ADDR1 0x36  // Horizontal Address Start Position
#define HORIZONTAL_WINDOW_ADDR2 0x37  // Horizontal Address End Position
#define VERTICAL_WINDOW_ADDR1   0x38  // Vertical Address Start Position
#define VERTICAL_WINDOW_ADDR2   0x39  // Vertical Address End Position
#define GAMMA_CTRL1             0x50  // Gamma Control 1
#define GAMMA_CTRL2             0x51  // Gamma Control 2
#define GAMMA_CTRL3             0x52  // Gamma Control 3
#define GAMMA_CTRL4             0x53  // Gamma Control 4
#define GAMMA_CTRL5             0x54  // Gamma Control 5
#define GAMMA_CTRL6             0x55  // Gamma Control 6
#define GAMMA_CTRL7             0x56  // Gamma Control 7
#define GAMMA_CTRL8             0x57  // Gamma Control 8
#define GAMMA_CTRL9             0x58  // Gamma Control 9
#define GAMMA_CTRL10 			0x59  // Gamma Control 10


/* RGB 24-bits color table definition (RGB888). */
#define RGB888_RGB565(color) ((((color) >> 19) & 0x1f) << 11) | ((((color) >> 10) & 0x3f) << 5) | (((color) >> 3) & 0x1f)

#define COLOR_BLACK          RGB888_RGB565(0x000000u)
#define COLOR_WHITE          RGB888_RGB565(0xFFFFFFu)
#define COLOR_BLUE           RGB888_RGB565(0x0000FFu)
#define COLOR_GREEN          RGB888_RGB565(0x00FF00u)
#define COLOR_RED            RGB888_RGB565(0xFF0000u)
#define COLOR_NAVY           RGB888_RGB565(0x000080u)
#define COLOR_DARKBLUE       RGB888_RGB565(0x00008Bu)
#define COLOR_DARKGREEN      RGB888_RGB565(0x006400u)
#define COLOR_DARKCYAN       RGB888_RGB565(0x008B8Bu)
#define COLOR_CYAN           RGB888_RGB565(0x00FFFFu)
#define COLOR_TURQUOISE      RGB888_RGB565(0x40E0D0u)
#define COLOR_INDIGO         RGB888_RGB565(0x4B0082u)
#define COLOR_DARKRED        RGB888_RGB565(0x800000u)
#define COLOR_OLIVE          RGB888_RGB565(0x808000u)
#define COLOR_GRAY           RGB888_RGB565(0x808080u)
#define COLOR_SKYBLUE        RGB888_RGB565(0x87CEEBu)
#define COLOR_BLUEVIOLET     RGB888_RGB565(0x8A2BE2u)
#define COLOR_LIGHTGREEN     RGB888_RGB565(0x90EE90u)
#define COLOR_DARKVIOLET     RGB888_RGB565(0x9400D3u)
#define COLOR_YELLOWGREEN    RGB888_RGB565(0x9ACD32u)
#define COLOR_BROWN          RGB888_RGB565(0xA52A2Au)
#define COLOR_DARKGRAY       RGB888_RGB565(0xA9A9A9u)
#define COLOR_SIENNA         RGB888_RGB565(0xA0522Du)
#define COLOR_LIGHTBLUE      RGB888_RGB565(0xADD8E6u)
#define COLOR_GREENYELLOW    RGB888_RGB565(0xADFF2Fu)
#define COLOR_SILVER         RGB888_RGB565(0xC0C0C0u)
#define COLOR_LIGHTGREY      RGB888_RGB565(0xD3D3D3u)
#define COLOR_LIGHTCYAN      RGB888_RGB565(0xE0FFFFu)
#define COLOR_VIOLET         RGB888_RGB565(0xEE82EEu)
#define COLOR_AZUR           RGB888_RGB565(0xF0FFFFu)
#define COLOR_BEIGE          RGB888_RGB565(0xF5F5DCu)
#define COLOR_MAGENTA        RGB888_RGB565(0xFF00FFu)
#define COLOR_TOMATO         RGB888_RGB565(0xFF6347u)
#define COLOR_GOLD           RGB888_RGB565(0xFFD700u)
#define COLOR_ORANGE         RGB888_RGB565(0xFFA500u)
#define COLOR_SNOW           RGB888_RGB565(0xFFFAFAu)
#define COLOR_YELLOW         RGB888_RGB565(0xFFFF00u)

// ILI9225 screen size
#define LCD_WIDTH  				176
#define LCD_HEIGHT 				220

//make it more beautiful?
#define START_INDEX_WRITE			0b01110000
#define START_REGISTER_WRITE		0b01110010


void ILI9225_initCtrlGPIO(void);
void ILI9225_reset(void);
void ILI9225_init(void);
void ILI9225_setOrientation(uint8_t orientation);
void ILI9225_swap(uint16_t *a, uint16_t *b);
void ILI9225_orientCoordinates(uint16_t *x1, uint16_t *y1);
void ILI9225_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ILI9225_drawPixel(uint16_t x1, uint16_t y1, uint16_t color);

#endif /* SRC_ILI9225_H_ */
