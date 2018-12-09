/*
 * Copyright (C) 2014 Maciej Suminski <orson@orson.net.pl>, Katarzyna Stachyra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "stm32f4xx_conf.h"

void serial_init(unsigned int baud_rate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    USART_StructInit(&USART_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    USART_InitStructure.USART_BaudRate            = baud_rate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE); // enable USART2
}

void serial_putc(const char c)
{
	USART_SendData(USART2, c);
}

void serial_puts(const char *string)
{
	while(*string)
	{
		// wait until data register is empty
		while( !(USART2->SR & 0x00000040) );
		serial_putc(*string);
		++string;
	}
}

void serial_write(const char *string, int len)  //what was the difference between serial_puts and serial_write?
{
    // TODO semaphore to sync data from different tasks?
    for(int i = 0; i < len; ++i)
    {
    	serial_putc(*string++);
    }

}

//int serial_getc(char *c)  //no need; will be done in interrupt
//{
//    return xQueueReceive(serial_rx_queue, (void*) c, SERIAL_TICKS_WAIT);
//}
//
//static void serial_tx_task(void *parameters) // no need
//{
//    char c;
//
//    while(1)
//    {
//        while(xQueueReceive(serial_tx_queue, &c, portMAX_DELAY))
//        {
//            /*HAL_UART_Transmit(&uart, (uint8_t*) &c, 1, 0);*/
//           // uart.Instance->DR = c;
//        	while( !(USART2->SR & 0x00000040) );
//        	USART2->DR=c;
//
//            // Wait until the transmission is finished
//          //  while(!__HAL_UART_GET_FLAG(&uart, UART_FLAG_TXE));
//        }
//    }
//}
//
//void USART2_IRQHandler(void)
//{
//    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//
//    if( USART_GetITStatus(USART2, USART_IT_RXNE) ){
//        char t = USART2->DR; // the character from the USART1 data register is saved in t
//        xQueueSendFromISR(serial_rx_queue, &t, &xHigherPriorityTaskWoken);
//    }
//    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//}

