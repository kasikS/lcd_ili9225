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
#include "serial.h"


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

    GPIO_SetBits(CS_port, CS);

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

void spi_DMA_Init(uint16_t *data, uint32_t bufferSize)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;

	DMA_Cmd(DMA1_Stream4, DISABLE);
	while(DMA_GetCmdStatus(DMA1_Stream4)); //

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_FEIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_TCIF4);

	DMA_DeInit(DMA1_Stream4); //reset DMA1 channe1 to default values;

	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;	//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_Memory0BaseAddr = (int32_t)data; //variable to write data from

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI2->DR)); // : address of data reading register -not needed
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	//setting normal mode (non circular)
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;	//medium priority
	DMA_InitStructure.DMA_BufferSize = bufferSize;	//number of data to be transfered
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //automatic memory increment disable for peripheral
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//automatic memory increment enable for memory
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//source peripheral data size = 8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//destination memory data size = 8bit
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
}

void spi_DMA_irqInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void spi_DMA_irqEnable(void)
{
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);
}

void spi_DMA_irqDisable(void)
{
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, DISABLE);
}

void spi_DMA_Enable(void)
{
	displayFree=0;
	GPIO_WriteBit(CS_port, CS, Bit_RESET);

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

	DMA_Cmd(DMA1_Stream4, ENABLE); /* Enable the DMA SPI TX Stream */

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);

}

void spi_DMA_Disable(void)
{

//	DMA_Cmd(DMA1_Stream4, DISABLE);
//	while(DMA_GetCmdStatus(DMA1_Stream4));
//	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);

	GPIO_WriteBit(CS_port, CS, Bit_SET);

	displayFree=1;
}

void spi_write(uint8_t address, uint8_t data)
{
	GPIO_ResetBits(CS_port, CS);

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, address);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, data);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY));

	GPIO_SetBits(CS_port, CS);
}

void spi_write16(uint16_t data, int cs)
{
	uint8_t buffer = 0;
	if(cs)
		GPIO_WriteBit(CS_port, CS, Bit_RESET);


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
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY));

	if(cs)
		GPIO_WriteBit(CS_port, CS, Bit_SET);
}

void spi_writeBuffer(uint8_t * data, uint16_t length, int cs)
{
	int i=0;
	uint8_t buffer = 0;

	if(cs)
		GPIO_ResetBits(CS_port, CS);

	for (i=0; i<length; i++)
	{
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		SPI_I2S_SendData(SPI2, data[i]);
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
		SPI_I2S_ReceiveData(SPI2);
	}

	if(cs)
		GPIO_WriteBit(CS_port, CS, Bit_SET);
}

uint8_t spi_read(uint8_t address)
{
	address=0x80|address;

	GPIO_ResetBits(CS_port, CS);

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, address);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));
	SPI_I2S_ReceiveData(SPI2); //Clear RXNE bit

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI2, 0x00); //Dummy byte to generate clock
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));

	GPIO_SetBits(CS_port, CS);

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

void DMA1_Stream4_IRQHandler(void)
{
		while(!DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4))

		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
		/* Clear transmission complete flag */
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);

		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
		while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);

		spi_DMA_Disable();
}
