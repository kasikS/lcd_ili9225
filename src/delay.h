#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

void DelayInit(void);
void Delay(uint32_t nTime);
void TimingDelay_Decrement(void);

#endif /* DELAY_H */
