


#include "RC5.h"
#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_tim.h"

#define  PWM_FREG        37600 // частоа ШИМ в герцах
#define  PWM_PRECENT        25 // скважность в процентах
#define  PRESC_PERIOD      900 // период одного полубита для протокола RC5 в микросекундах
#define  ACCEPT             11 // допуск в процентах
#define  COUNT_PERIOD       22


extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

static uint16_t cnt_period = 0;
static uint16_t previous_value = 0;

uint16_t get_period_buf[26] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t get_data_buf[15];

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
uint32_t pwm_period_get(uint32_t ir_clk_hZ);
uint32_t pwm_duty_get(uint32_t duty_precent);


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

extern void rc5_init(void)
{
  MX_TIM3_Init();
  MX_TIM8_Init();
  HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
} 

extern void rc5_send(uint16_t * data)
{
  uint8_t i = 0;
  volatile uint16_t j = 0;
  
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


extern void rc5_get(uint16_t * data)
{
               
//>>>>>>>>>>>>>>>>>>>>>>>>>> Пишем значения CNT >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 
   
    if (cnt_period < COUNT_PERIOD)
    {
        if (previous_value == 0)
        {
            previous_value = TIM8->CNT;
        }
        else
        {
            data[cnt_period] = (TIM8->CNT - previous_value);
            previous_value = TIM8->CNT;
            cnt_period++;
        }
        
    }  
    else
    {
        data[cnt_period] = (TIM8->CNT - previous_value); 
        if (data[25] == 0)
        {
           previous_value = 0; 
        }
        cnt_period = 0;
    }        
}       
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

extern void calc_rc5 (uint16_t *period_buf, uint16_t *data_buf)
{
    static uint16_t delta_p = 0;
    uint16_t cnt_period = 0;
    uint16_t cnt_bit = 0;
    uint16_t bit = 0;
    #define TOLERANCE     5 // Допуск в %
    delta_p = ((period_buf[0] + period_buf[1]) / 2); // усредненная длительность импульса
    
   if (period_buf[0] != 0)
   {       
        for (cnt_period = 0; cnt_period < 25; cnt_period++)
        {
            if (
                    (period_buf[cnt_period] > (delta_p - ((delta_p * TOLERANCE) / 100)))
                 && (period_buf[cnt_period] < (delta_p + ((delta_p * TOLERANCE) / 100)))
               )
            {
                if (cnt_period == 0) 
                {
                    data_buf[cnt_bit] = 1;
                    cnt_bit++;
                }
                else if (
                            ((cnt_period % 2) == 0)
                         && (bit == 0)
                        )
                        {
                            data_buf[cnt_bit] = 1;
                            cnt_bit++;
                        }
                        else if (
                                    ((cnt_period % 2) == 0)
                                 && (bit == 1)
                                )
                        {
                            data_buf[cnt_bit] = 0;
                            cnt_bit++;
                        }
                
            }
            else if (
                         (period_buf[cnt_period] > ((delta_p * 2) - (((delta_p * 2) * TOLERANCE) / 100)))
                      && (period_buf[cnt_period] < ((delta_p * 2) + (((delta_p * 2) * TOLERANCE) / 100)))
                    )
            {
                
                if (
                       ((cnt_period % 2) != 0)
                    && (bit == 0)
                   )
                {
                    data_buf[cnt_bit] = 1;
                    cnt_bit++;
                }
                else if (
                            ((cnt_period % 2) != 0)
                         && (bit == 1)
                        )
                {
                    data_buf[cnt_bit] = 0;
                    cnt_bit++;
                }
                bit ^= 1;
            }     
        }
    }
}


__weak void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(htim);
    /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_IC_CaptureCallback could be implemented in the user file
    */
  rc5_get(&get_period_buf[0]);
}
