/*
 * spi.c
 *
 *  Created on: Feb 17, 2018
 *      Author: kasik
 */
#include <stdio.h>


#include "stm32f4xx_conf.h"
#include "spi.h"
#include "stm32f4xx_spi.h"

#include "ILI9225.h" //for the trials now only!!!

#define CS						GPIO_Pin_4

void spi_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	// enable peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* configure pins used by SPI2
	 * PB15 = MOSI
	 * PB14 = MISO
	 * PB13 = SCK
	 */

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// connect SPI2 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = CS;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_SetBits(GPIOA, CS);

	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32; // SPI frequency is APB2 frequency / 32
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI2, &SPI_InitStruct);

	SPI_Cmd(SPI2, ENABLE); // enable SPI2
}

void spi_write(uint8_t address, uint8_t data)
{
	GPIO_ResetBits(GPIOA, CS);

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, address);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	GPIO_SetBits(GPIOA, CS);
}

void spi_write16(uint16_t address, uint16_t data)
{
	uint8_t buffer = 0;


	GPIO_WriteBit(GPIOA, CS, Bit_RESET);
	GPIO_WriteBit(dispCtrlPort, RS, Bit_RESET);

	//send higher byte of address
	buffer= address >> 8;
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, buffer);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	//send lower byte of address
	buffer= (uint8_t) (0x00ff & address);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, buffer);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);


	GPIO_WriteBit(dispCtrlPort, RS, Bit_SET);

	//send higher byte of data
	buffer= data >> 8;
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	//send lower byte of data
	buffer= (uint8_t) (0x00ff & data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);


	GPIO_WriteBit(GPIOA, CS, Bit_SET);


}

void spi_writeCommand(uint16_t address, int cs)
{
	uint8_t buffer = 0;

	GPIO_WriteBit(dispCtrlPort, RS, Bit_RESET);
	if(cs)
		GPIO_WriteBit(GPIOA, CS, Bit_RESET);


	//send higher byte of address
	buffer= address >> 8;
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, buffer);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	//send lower byte of address
	buffer= (uint8_t) (0x00ff & address);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, buffer);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	if(cs)
		GPIO_WriteBit(GPIOA, CS, Bit_SET);
}

void spi_writeData(uint16_t data, int cs)
{
	uint8_t buffer = 0;

	GPIO_WriteBit(dispCtrlPort, RS, Bit_SET);
	if(cs)
		GPIO_WriteBit(GPIOA, CS, Bit_RESET);

	//send higher byte of data
	buffer= data >> 8;
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, buffer);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	//send lower byte of data
	buffer= (uint8_t) (0x00ff & data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, buffer);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	if(cs)
		GPIO_WriteBit(GPIOA, CS, Bit_SET);

}

void spi_writeBuffer(uint16_t address, uint8_t * data, uint16_t length)
{
	int i=0;
	uint8_t buffer = 0;
	GPIO_ResetBits(GPIOC, GPIO_Pin_5);

	//send higher byte of address
	buffer= address >> 8;
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, buffer);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	//send lower byte of address
	buffer= (uint8_t) (0x00ff & address);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, buffer);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	for (i=0; i<length; i++)
	{
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		SPI_I2S_SendData(SPI2, data[i]);
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
		SPI_I2S_ReceiveData(SPI2);
	}
}

void spi_writeStart(uint8_t data, int cs)
{
	return;
	if(cs)
		GPIO_WriteBit(GPIOA, CS, Bit_RESET);

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	if(cs)
		GPIO_WriteBit(GPIOA, CS, Bit_SET);
}

uint8_t spi_read(uint8_t address)
{
	address=0x80|address;

	GPIO_ResetBits(GPIOC, GPIO_Pin_5);

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, address);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2); //Clear RXNE bit

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, 0x00); //Dummy byte to generate clock
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));

	GPIO_SetBits(GPIOC, GPIO_Pin_5);

	return  SPI_I2S_ReceiveData(SPI2);
}

void spi_writeBit(uint8_t address, uint8_t bitNum, uint8_t data)
{
    uint8_t b;

    b = spi_read(address);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));

    spi_write(address, b);

}

void spi_writeBits(uint8_t address, uint8_t bitStart, uint8_t length, uint8_t data)
{
	//      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value

    //if(length > 0) {
        uint8_t b = 0;

        if(spi_read(address) != 0)
        {
            uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
            data <<= (bitStart - length + 1); // shift data into correct position
            data &= mask; // zero all non-important bits in data
            b &= ~(mask); // zero all important bits in existing byte
            b |= data; // combine data with existing byte
            spi_write(address, b);
        }
    //}
}

void spi_writeRegister(uint16_t address, uint16_t data)
{
	GPIO_WriteBit(GPIOA, CS, Bit_RESET);

	GPIO_WriteBit(dispCtrlPort, RS, Bit_RESET);
	spi_writeStart(START_INDEX_WRITE, 0);
	spi_writeCommand(address, 0);

	GPIO_WriteBit(dispCtrlPort, RS, Bit_SET);
	spi_writeStart(START_REGISTER_WRITE, 0);
	spi_writeData(data, 0);
	GPIO_WriteBit(GPIOA, CS, Bit_SET);
}

