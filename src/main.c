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
#include "ILI9225_colours.h"
#include "spi.h"
#include "serial.h"
#include "ILI9225_registers.h"
#include "unicorn.h"


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


//volatile uint16_t bitmap[176*220];


int main(void)
{
uint16_t coloursArr[8] = {COLOR_BLACK, COLOR_RED, COLOR_PINK, COLOR_INDIGO, COLOR_WHITE, COLOR_GOLD, COLOR_DARKGRAY, COLOR_TURQUOISE};

	DelayInit();
	spi_init(); //init spi2

	serial_init(460800);
	ILI9225_init();
	Delay(10);

	uint16_t x1 = 0;
	uint16_t y1 = 0;
	int  i = 0;

//	spi_write16(DISP_CTRL1, 0x0000); // Display off

	int counter=0;

	if(!ILI9225_isBusy())
	{
		ILI9225_drawBitmapDMA(0, 0, 176, 176, unicorn);
	}

	while (1)
	{

/*
			for (i = 0; i<176*110; i ++)
			{
				bitmap[i] = coloursArr[counter%4];
			}

			for (i = 176*110; i<176 * 220; i ++)
			{
				bitmap[i] = coloursArr[counter%4 + 4];
			}


			if(!ILI9225_isBusy())
			{
				ILI9225_drawBitmapDMA(0, 0, 176, 220, bitmap);
				counter++;
			}
*/
		Delay(1000);
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
