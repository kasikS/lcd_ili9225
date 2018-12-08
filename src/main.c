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


	DelayInit();
	spi_init(); //init spi2
	ILI9225_init();
	Delay(10);

	uint16_t x1 = 0;
	uint16_t y1 = 0;
	int  i = 0;

	uint16_t green[200];
	uint16_t red[200];
	uint16_t mixed[400];

for (i = 0; i<200; i ++)
{
	green[i] = COLOR_GREEN;
	red[i] = COLOR_RED;
}

for (i = 0; i<200; i ++)
{
	mixed[i] = COLOR_VIOLET;
}

for (i = 200; i<400; i ++)
{
	mixed[i] = COLOR_GOLD;
}


//	for (y1 = 0; y1 < 50; y1++)
//	{
//		for (x1 = 0; x1 < 50; x1++)
//		{
//			ILI9225_drawPixel(x1, y1, COLOR_GREEN);
//		}
//	}


	ILI9225_drawBitmap(x1, y1, 100, 2, red);


	y1 = 5;

	ILI9225_drawBitmap(x1, y1, 100, 4, mixed);



//	spi_write16(DISP_CTRL1, 0x0000); // Display off

	while (1)
	{
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
