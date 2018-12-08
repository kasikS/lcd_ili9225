/*
 * ILI9225_registers.c
 *
 *  Created on: Dec 8, 2018
 *      Author: kasik
 */

#ifndef SRC_ILI9225_REGISTERS_C_
#define SRC_ILI9225_REGISTERS_C_

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



#endif /* SRC_ILI9225_REGISTERS_C_ */
