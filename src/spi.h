/*
 * spi.h
 *
 *  Created on: Feb 17, 2018
 *      Author: kasik
 */

#ifndef SRC_SPI_H
#define SRC_SPI_H

#include <inttypes.h>
#include <stdbool.h>

#define CS						GPIO_Pin_4
#define CS_port					GPIOA

void spi_init(void);
void spi_write(uint8_t address, uint8_t data);
uint8_t spi_read(uint8_t address);
void spi_writeBit(uint8_t address, uint8_t bitNum, uint8_t data);
void spi_writeBits(uint8_t address,uint8_t bitStart, uint8_t length, uint8_t data);
void spi_write16(uint16_t data, int cs);
void spi_writeBuffer(uint8_t * data, uint16_t length, int cs);
void spi_writeCommand(uint16_t address, int cs);
void spi_writeData(uint16_t data, int cs);
void spi_writeRegister(uint16_t address, uint16_t data);
void spi_DMA_Init(uint16_t *data, uint32_t bufferSize);
void spi_DMA_Transfer(void);

extern int stopDMA;
//extern uint16_t bitmap[400];
#endif /* SRC_SPI_H */
