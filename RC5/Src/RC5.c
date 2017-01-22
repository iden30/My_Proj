


#include "RC5.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#define  PWM_FREG        37600 // частоа ШИМ в герцах
#define  PWM_PRECENT        25 // скважность в процентах
#define  PRESC_PERIOD      900 // период одного полубита для протокола RC5 в микросекундах
#define  ACCEPT             11 // допуск в процентах


extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
uint32_t pwm_period_get(uint32_t ir_clk_hZ);
uint32_t pwm_duty_get(uint32_t duty_precent);

typedef enum {STATE_RC5_NONE, STATE_RC5_SEND, STATE_RC5_GET} state_rc5_status_t; // перечисление состоянй автомата

static uint16_t get_data_buf[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
static uint16_t Z[]            = {1,1,0,0,0,0,0,0,0,0,1,1,0,0};
//static uint16_t P[]            = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint16_t cnt_data_bit = 0;
static uint16_t point_1  = 0;
static uint16_t point_2  = 0;
static uint16_t period   = 0; 
static uint8_t  puls = 0;
static uint8_t  f_period = 0; // Флаг измеренного периода IC
static uint8_t  cnt_nibble = 0; // Для подсчета полученных полубит

uint32_t pwm_period_get(uint32_t ir_clk_hZ)
{
  uint32_t tim_clk = 0;
  
  if (htim3.Init.Prescaler == 0) // проверяем значение предделителя на 0
  {
    htim3.Init.Prescaler = 1;
  }
  
  tim_clk = (HAL_RCC_GetPCLK2Freq() / htim3.Init.Prescaler); // вычисляем частоту тактирования таймера
  return (tim_clk / ir_clk_hZ); // вычисляем период PWM для получения заданной частоты PWM
}

uint32_t pwm_duty_get(uint32_t duty_precent)
{
  return ((pwm_period_get(PWM_FREG) * duty_precent) / 100);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = pwm_period_get(PWM_FREG);
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    //Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
//    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
//    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
//    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pwm_duty_get(PWM_PRECENT);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
  //  Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 36;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
   // Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
   // Error_Handler();
  }

  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
   // Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
  //  Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
   // Error_Handler();
  }

}

void rc5_init(void)
{
  MX_TIM3_Init();
  MX_TIM8_Init();
  HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
} 

void rc5_send(uint32_t *data)
{
  uint8_t i = 0;
  volatile uint32_t j = 0;
  
  for( i=0; i<14; i++)
  {
    switch (data[i])
    {
      case 0: 
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        for (j=0; j < (10*911); j++){};
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        for (j=0; j < (10*911); j++){};
        break;
      
      case 1: 
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        for (j=0; j < (10*911); j++){};
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        for (j=0; j < (10*911); j++){};
        break;
      
      default:
        break;
    }
    
  }  
  
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}








uint16_t rc5_get(void)
{
      
//>>>>>>>>>>>>>>>>>>>>>>>>>> Пишем значения CNT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
    if (puls == 0)
    {
        point_1 = TIM8->CNT;
        //HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    } 
    else
    {
        if (!f_period) f_period = 1;
        point_2 = TIM8->CNT;
        //HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
    }
    
 //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    
    //>>>>>>>>>>>>>>>>>>>>>> Декодируем данные >>>>>>>>>>>>>>>>>>>>>>>>>
    
    if (f_period != 0) // Если обе точки измерялись
    {
      period = (point_2 - point_1); // Расчитываем период
 
      if (  // Проверяем, поподает ли период в рамки 
             (period >= (PRESC_PERIOD - (PRESC_PERIOD * ACCEPT) / 100)) // >= period - 11%
          || (period <= (PRESC_PERIOD + (PRESC_PERIOD * ACCEPT) / 100)) // <= period + 11%
         )
         { 
           // Если да, заполняем массив принятых данных
           if (cnt_data_bit == 0) // Если это первый принятый бит, записываем в нулевую ячейку = 1
           {
             get_data_buf[cnt_data_bit] = 1;
             if (cnt_data_bit < 13)
             {
               cnt_data_bit++;
             }
             else
             {
               cnt_data_bit = 0;
             }
           } 
           else
           {
             if (cnt_nibble == 0) // Если это первый принятый болубит
             {
               cnt_nibble = 1;  // меняем флаг
             }
             else
             {
               cnt_nibble = 0;  // меняем флаг
               if (puls == 0)
               {
                  get_data_buf[cnt_data_bit] = 0;
               }
                 if (cnt_data_bit < 13)
                 {
                   cnt_data_bit++;
                 }
                 else
                 {
                   cnt_data_bit = 0;
                 }
             }
           } 
         } 
         else if (  // Проверяем, поподает ли период в рамки 
                    (period >= ((PRESC_PERIOD * 2) - (PRESC_PERIOD * ACCEPT) / 100)) // >= (period * 2) - 11%
                  ||(period <= ((PRESC_PERIOD * 2) + (PRESC_PERIOD * ACCEPT) / 100)) // <= (period * 2) + 11%
                 )
         { 
             if (puls == 1)
             {
               if (cnt_nibble == 0) // Если это первый принятый болубит
               {
                 cnt_nibble = 1;  // меняем флаг
                 get_data_buf[cnt_data_bit] = 1;
               }
               else
               {
                 cnt_nibble = 0;  // меняем флаг
                 get_data_buf[cnt_data_bit] = 1;
            
                 if (cnt_data_bit < 13)
                 {
                   cnt_data_bit++;
                 }
                 else
                 {
                   cnt_data_bit = 0;
                 }
               }
             } 
             else 
             {
               if (cnt_nibble == 0) // Если это первый принятый болубит
               {
                 cnt_nibble = 1;  // меняем флаг
                 get_data_buf[cnt_data_bit] = 1;
               }
               else
               {
                 cnt_nibble = 0;  // меняем флаг
                 get_data_buf[cnt_data_bit] = 0;
                 if (cnt_data_bit < 13)
                 {
                   cnt_data_bit++;
                 }
                 else
                 {
                   cnt_data_bit = 0;
                 }
               }
             } 
        } 
    }
    puls ^= 1;
    return (period);
}


__weak void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t i = 0;
    static uint16_t a = 0;
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_IC_CaptureCallback could be implemented in the user file
    */
    
    
 
    rc5_get();    
        
    if ( i == 23)
    {
        for (i = 0; i < 13; i++)
        {
            
            if (get_data_buf[i] == Z[i])
            {
              a ++;
            }
            else
            {
              if (a > 0) a--;
            }
        }
        
        if (a == 13)
        {
            a = 0;
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        }
        else
        {
            a = 0;
        }
        i = 0;
    }
    else
    {  
        i++;
    }
}
