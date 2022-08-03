#ifndef _MY_
#define _MY_
#include "main.h"
#include "IR_NEC.h"
#include "Steering_Engine.h"
extern ADC_HandleTypeDef hadc1;
extern uint8_t STOP_Ctl;
void OPEN(void);
void CLOSE(void);
void INIT_LED(void);
void Voltage_detection(void);
void LED_Ctrl(void);
void enter_stop_mode(void);
void exit_stop_mode(void);
#endif
