/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include "stm32f4xx_conf.h"
#include "delay.h"
#include "ILI9225.h"
#include "spi.h"

/*#define controlPort		GPIOC
#define addressPort		GPIOA
#define dataPort		GPIOB
#define MERQ			GPIO_Pin_0
#define CS				GPIO_Pin_1
#define RD				GPIO_Pin_2
#define WR 				GPIO_Pin_3
*/




/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

int main(void) {

/*	uint16_t controlPins;
	uint16_t addressPins;
	uint16_t dataPins;

	uint16_t address;

	GPIO_InitTypeDef GPIO_InitStructure;
*/

//	!< At this stage the microcontroller clock setting is already configured,
//	 this is done through SystemInit() function which is called from startup
//	 files before to branch to application main.
//	 To reconfigure the default setting of SystemInit() function,
//	 refer to system_stm32f4xx.c file


	DelayInit();

	/* Add your application code here */
	/* Insert 50 ms delay */
/*	Delay(50);

	// Enable the GPIOA, GPIOB, GPIOC peripheral
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_ClearFlag();

	// Configure pins in alternate function
	addressPins = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Pin = dataPins; //GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(addressPort, &GPIO_InitStructure);

	dataPins = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Pin =  addressPins; //GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(dataPort, &GPIO_InitStructure);

	controlPins = MERQ|CS|WR|RD; //GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //MERQ, CS,RD,WR
	GPIO_InitStructure.GPIO_Pin = controlPins; //GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(controlPort, &GPIO_InitStructure);

	GPIO_Write(controlPort, controlPins);

	for(address = 0; address < 256; address++)
	{
		GPIO_Write(addressPort, address);
		GPIO_WriteBit(controlPort, CS, Bit_RESET);
		GPIO_WriteBit(controlPort, MERQ, Bit_RESET);

		Delay(1);

		GPIO_WriteBit(controlPort, WR, Bit_RESET);
		GPIO_Write(dataPort, 0x000);

		Delay(1);

		GPIO_WriteBit(controlPort, CS, Bit_SET);
		GPIO_WriteBit(controlPort, MERQ, Bit_SET);
		GPIO_WriteBit(controlPort, WR, Bit_SET);

		Delay(1);
	}
*/

	ILI9225_initCtrlGPIO();
	Delay(50);
	ILI9225_reset();

	spi_init(); //init spi2

	ILI9225_init();
	Delay(10);
	uint16_t x1 = 0;
	uint16_t y1 = 0;
	int  i = 0;

//    spi_writeRegister(RAM_ADDR_SET1,0);
//	spi_writeRegister(RAM_ADDR_SET2,0);
//
//	GPIO_WriteBit(dispCtrlPort, RS, Bit_SET);
//	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
//	spi_writeStart(START_REGISTER_WRITE, 0);

	for (y1 = 0; y1 < 150; y1++)
	{
		for (x1 = 0; x1 < 150; x1++)
		{
			//spi_writeData(1 << (x1 % 16), 0);
			ILI9225_drawPixel(x1, y1, RGB888_RGB565(0xFCC9B9u));
		}
	}

//	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);

//	spi_writeStart(START_WRITE_GRAM);
//	spi_write16(DISP_CTRL1, 0x0000); // Display off

	while (1) {
		//GPIO_ToggleBits(GPIOA, GPIO_Pin_5);


		Delay(100);
	}
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
