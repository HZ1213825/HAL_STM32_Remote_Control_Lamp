#ifndef __DELAY_H
#define __DELAY_H
#include "main.h"
extern TIM_HandleTypeDef htim3;
#define Delay_Tim htim3

void Delay_us(int time);
void Delay_ms(int time);
void Delay_s(int time);
void Delay_min(int time);
#endif
