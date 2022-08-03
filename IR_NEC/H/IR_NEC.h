#ifndef _IR_NEC_H_
#define _IR_NEC_H_
#include "main.h"
#include "Delay.h"
extern TIM_HandleTypeDef htim1;
// extern TIM_HandleTypeDef htim3;

#define u8 uint8_t
#define u32 uint32_t
/*
接收部分需要
1.定时器（无需中断）
    分频后周期需要为1us
    计数最大值需大于15ms
2.外部中断
    下降沿触发

需要修改外部中断回调函数，在C文件中
*/
#define IR_NEC_Read_TIM htim1

/*
发送部分需要
1.发送的GPIO
    设置为发送模式
    推挽/开漏上拉，根据发送功率选择
2.定时器
    需要开启中断（在定时器初始化函数里调用  HAL_TIM_Base_Start_IT  ）
    中断频率为 38 * 2 KHz

需要修改定时器中断回调函数，在C文件中
*/
// #define IR_NEC_Send_GPIOx GPIOA
// #define IR_NEC_Send_GPIO_Pin GPIO_PIN_0

// #define IR_NEC_Send_TIM htim3

#define N 4
extern u8 IR_NEC_Read_Dat[N]; //解码的数据
extern u8 IR_NEC_Read_OK;     //解码成功标志

void air(void);
void IR_NEC_Read_Init(void);
void IR_NEC_Read_Decode(void (*val)(void));
void IR_NEC_Send_Init(void);
void IR_NEC_Send_Code(u8 *Dat, u32 Len);
void IR_NEC_Send_Repect(void);
#endif
