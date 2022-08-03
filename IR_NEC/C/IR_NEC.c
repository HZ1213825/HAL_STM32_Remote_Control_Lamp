#include "IR_NEC.h"
u8 IR_NEC_Read_ins = 0;      //指示解码类型 引导码/数据码
u8 IR_NEC_Read_Decode_i = 0; //解码循环变量
u8 IR_NEC_Read_Decode_j = 0; //解码循环变量
u8 IR_NEC_Read_OK = 0;       //解码成功标志
u32 IR_NEC_Read_Time = 0;    //下降沿时间差
u8 IR_NEC_Read_zj = 0;
u8 IR_NEC_Send_Square = 0; //方波
u8 IR_NEC_Read_Dat[N] = {0};
u8 IR_NEC_Read_Dat2[N] = {0};
void air(void)
{
}

// NEC解码函数，外部中断下降沿调用
void IR_NEC_Read_Decode(void (*val)(void))
{
    if (IR_NEC_Read_ins == 0) //检测初始低电平
    {
        IR_NEC_Read_ins = 1;
        HAL_TIM_Base_Start(&IR_NEC_Read_TIM);
        __HAL_TIM_SetCounter(&IR_NEC_Read_TIM, 0);
    }
    else if (IR_NEC_Read_ins == 1) //判断初始低电平到第二个低电平时间
    {
        IR_NEC_Read_Time = __HAL_TIM_GetCounter(&IR_NEC_Read_TIM);
        if (IR_NEC_Read_Time > 13500 - 1000 && IR_NEC_Read_Time < 13500 + 1000) // 13.5ms左右 引导码
        {

            IR_NEC_Read_ins = 2;
            __HAL_TIM_SetCounter(&IR_NEC_Read_TIM, 0);
            IR_NEC_Read_OK = 0;
            for (int i = 0; i < N; i++)
                IR_NEC_Read_Dat[i] = 0;
            IR_NEC_Read_zj = 0;
        }
        else if (IR_NEC_Read_Time > 11250 - 1000 && IR_NEC_Read_Time < 11250 + 1000) // 11.25ms左右 重复码
        {
            IR_NEC_Read_ins = 0;
            IR_NEC_Read_OK = 2;
            for (int i = 0; i < N; i++)
                IR_NEC_Read_Dat[i] = IR_NEC_Read_Dat2[i];

            __HAL_TIM_SetCounter(&IR_NEC_Read_TIM, 0);
        }
        else //超时或时间过短 复位
        {
            IR_NEC_Read_ins = 0;
        }
    }
    else if (IR_NEC_Read_ins == 2) //开始解码
    {
        IR_NEC_Read_Time = __HAL_TIM_GetCounter(&IR_NEC_Read_TIM);
        if (IR_NEC_Read_Time > 1120 - 500 && IR_NEC_Read_Time < 1120 + 500) // 1.12ms 写入0
        {
            IR_NEC_Read_zj <<= 1;   //向左移位
            IR_NEC_Read_zj &= 0xfe; //最低位置零
            IR_NEC_Read_Decode_i++;
        }
        else if (IR_NEC_Read_Time > 2250 - 500 && IR_NEC_Read_Time < 2250 + 500) // 2.25ms 写入1
        {
            IR_NEC_Read_zj <<= 1;
            IR_NEC_Read_zj |= 0x01;
            IR_NEC_Read_Decode_i++;
        }
        else //出错复位
        {
            IR_NEC_Read_ins = 0;
            IR_NEC_Read_Decode_i = 0;
            IR_NEC_Read_Decode_j = 0;
            IR_NEC_Read_zj = 0;
        }

        if (IR_NEC_Read_Decode_i >= 8) // uchar每位写入数据
        {
            IR_NEC_Read_Decode_i = 0;
            IR_NEC_Read_Dat[IR_NEC_Read_Decode_j] = IR_NEC_Read_zj;
            IR_NEC_Read_Decode_j++;
            IR_NEC_Read_zj = 0;
        }
        if (IR_NEC_Read_Decode_j >= N) //数据数组的不同位写入数据
        {
            IR_NEC_Read_Decode_i = 0;
            IR_NEC_Read_Decode_j = 0;
            IR_NEC_Read_ins = 0;
            IR_NEC_Read_OK = 1;
            IR_NEC_Read_zj = 0;
            for (int i = 0; i < N; i++)
                IR_NEC_Read_Dat2[i] = IR_NEC_Read_Dat[i];
            HAL_TIM_Base_Stop(&IR_NEC_Read_TIM);
            val();
        }

        __HAL_TIM_SetCounter(&IR_NEC_Read_TIM, 0);
    }
}

// //发送引导码
// __STATIC_INLINE void IR_NEC_Send_Guide(void)
// {
//     HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
//     __HAL_TIM_SetCounter(&IR_NEC_Send_TIM, 0);
//     HAL_TIM_Base_Start(&IR_NEC_Send_TIM);
//     Delay_us(9000); // 9ms闪
//     HAL_TIM_Base_Stop(&IR_NEC_Send_TIM);
//     HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
//     Delay_us(4500); // 4.5ms灭
//     //共13.5ms
// }
// //发送数据0
// __STATIC_INLINE void IR_NEC_Send_0(void)
// {
//     HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
//     __HAL_TIM_SetCounter(&IR_NEC_Send_TIM, 0);
//     HAL_TIM_Base_Start(&IR_NEC_Send_TIM);
//     Delay_us(620); // 0.56ms闪
//     HAL_TIM_Base_Stop(&IR_NEC_Send_TIM);
//     HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
//     Delay_us(500); // 0.56ms灭
//     //共1.12ms
// }
// //发送数据1
// __STATIC_INLINE void IR_NEC_Send_1(void)
// {
//     HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
//     __HAL_TIM_SetCounter(&IR_NEC_Send_TIM, 0);
//     HAL_TIM_Base_Start(&IR_NEC_Send_TIM);
//     Delay_us(625); // 0.56ms闪
//     HAL_TIM_Base_Stop(&IR_NEC_Send_TIM);
//     HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
//     Delay_us(1600); // 1.69ms灭
//     //共2.25ms
// }
// //结束位
// void IR_NEC_Send_End(void)
// {
//     HAL_TIM_Base_Start(&IR_NEC_Send_TIM);
//     Delay_us(650); // 6ms亮
//     HAL_TIM_Base_Stop(&IR_NEC_Send_TIM);
//     HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
// }
// //发送重复码
// void IR_NEC_Send_Repect(void)
// {
//     HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
//     __HAL_TIM_SetCounter(&IR_NEC_Send_TIM, 0);
//     HAL_TIM_Base_Start(&IR_NEC_Send_TIM);
//     Delay_us(9000); // 9ms亮
//     HAL_TIM_Base_Stop(&IR_NEC_Send_TIM);
//     HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
//     Delay_us(2250); // 2.25ms灭
//                     //共11.25ms
//     IR_NEC_Send_End();
//     Delay_ms(100);
// }
// // NEC编码发送
// void IR_NEC_Send_Code(u8 *Dat, u32 Len)
// {
//     u32 zj;
//     IR_NEC_Send_Guide();          //引导码
//     for (int j = 0; j < Len; j++) //循环数组
//     {
//         zj = Dat[j];
//         for (int i = 0; i < 8; i++) //从高到低
//         {
//             if (zj & (0X80))
//             {
//                 IR_NEC_Send_1();
//             }
//             else
//             {
//                 IR_NEC_Send_0();
//             }
//             zj <<= 1;
//         }
//     }
//     IR_NEC_Send_End(); //结束位
//     Delay_ms(40);
// }

// //产生38kHz方波
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim == &IR_NEC_Send_TIM)
//     {
//         if (IR_NEC_Send_Square)
//             HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_SET);
//         else
//             HAL_GPIO_WritePin(IR_NEC_Send_GPIOx, IR_NEC_Send_GPIO_Pin, GPIO_PIN_RESET);
//         IR_NEC_Send_Square++;
//         if (IR_NEC_Send_Square >= 2)
//             IR_NEC_Send_Square = 0;
//     }
// }
