#include "stm32f4xx_conf.h"

static __IO uint32_t uwTimingDelay;

void DelayInit(void) {
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	// Set SysTick freq to 1 kHz
	SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
	SystemCoreClockUpdate();
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(uint32_t nTime) {
	uwTimingDelay = nTime;

	while (uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void) {
	if (uwTimingDelay != 0x00) {
		uwTimingDelay--;
	}
}
