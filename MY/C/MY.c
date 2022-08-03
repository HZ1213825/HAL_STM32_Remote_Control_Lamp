#include "MY.h"
uint8_t LED_Ctl = 0;  // LED控制计时
uint8_t STOP_Ctl = 0; //进入休眠模式计时
uint8_t ADC_ins = 0;  // adc去抖指示
/**
 * @brief 开灯
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-08-03 16:43:01
 */
void OPEN(void)
{
    Steering_Engine_360(0, 30);
    HAL_Delay(400);
    Steering_Engine_360(1, 50);
    HAL_Delay(100);
    Steering_Engine_Stop();
    HAL_Delay(2000);
}
/**
 * @brief 关灯
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-08-03 16:43:14
 */
void CLOSE(void)
{
    Steering_Engine_360(1, 30);
    HAL_Delay(400);
    Steering_Engine_360(0, 50);
    HAL_Delay(180);
    Steering_Engine_Stop();
    HAL_Delay(2000);
}
/**
 * @brief 上电时LED闪
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-08-03 16:43:29
 */
void INIT_LED(void)
{

    if (LED_Ctl % 2)
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
    LED_Ctl++;
}
/**
 * @brief 电压检测
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-08-03 16:44:06
 */
void Voltage_detection(void)
{
    uint16_t V = 0;
    HAL_ADC_Start(&hadc1);                                               //开启ADC
    HAL_ADC_PollForConversion(&hadc1, 100);                              //开启电压检测
    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) //转化完成
    {
        V = HAL_ADC_GetValue(&hadc1); //获取电压值
        if (V < 2600)
        {
            HAL_Delay(100);
            HAL_ADC_PollForConversion(&hadc1, 100); //去抖
            if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
            {
                V = HAL_ADC_GetValue(&hadc1);
                if (V < 2600)
                {
                    HAL_Delay(100);
                    HAL_ADC_PollForConversion(&hadc1, 100); //再去抖
                    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
                    {
                        V = HAL_ADC_GetValue(&hadc1);
                        if (V < 2600)
                        {
                            // LED_Ctl = 0;
                            STOP_Ctl = 0;
                            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); //点亮灯
                        }
                    }
                }
            }
        }
        else
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); //关闭led
        }
    }
    HAL_ADC_Stop(&hadc1); //关闭adc
}
/**
 * @brief led控制
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-08-03 16:46:48
 */
void LED_Ctrl(void)
{
    if (LED_Ctl < 6)
    {
        INIT_LED(); //前6秒闪烁
    }
    else
    {
        STOP_Ctl++;
        Voltage_detection();
    }

    if (STOP_Ctl >= 6)
    {
        STOP_Ctl = 0;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); //关灯
        enter_stop_mode();                                         //休眠
        exit_stop_mode();                                          //退出休眠
    }
}
/**
 * @brief 进入休眠模式
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-08-03 16:47:42
 */
void enter_stop_mode(void)
{
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}
