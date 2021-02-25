/* This is a simple application we made for my Embedded Systems course to blink a few LEDs
 * using timers and display some text on the LCD of an STM32 discovery board */
#include "main.h"

TIM_HandleTypeDef Tim2_Handle,Tim4_Handle;
TIM_OC_InitTypeDef Tim4_OCInitStructure;
uint16_t Tim2_PrescalerValue,Tim4_PrescalerValue;

__IO uint16_t Tim4_CCR; // the pulse of the TIM4
__O uint8_t factor = 0;
uint8_t     JOYSTICK_SEL_INT_FIRED = 0;
uint8_t     counter = 0;

//for using LCD
Point_Typedef singlePoint;
DoublePoint_Typedef doublePoint;
DigitPosition_Typedef charPosition;

static void SystemClock_Config(void);
static void EXTI0_Config(void);
static void Error_Handler(void);

void TIM2_Config(void);
void TIM4_Config(void);
void TIM4_OC_Config(void);

int main(void)
{
  uint8_t aChar='a';

  singlePoint=POINT_ON ;
  doublePoint=DOUBLEPOINT_OFF;
  charPosition=LCD_DIGIT_POSITION_1;

  HAL_Init();

  SystemClock_Config();

  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);

  BSP_LCD_GLASS_Init();

  BSP_JOY_Init(JOY_MODE_EXTI);


  TIM2_Config();

  TIM4_Config();

  Tim4_CCR=20000;       //2 s to fire an interrupt.
  TIM4_OC_Config();


  BSP_LCD_GLASS_ScrollSentence((uint8_t*) "3ta4 lab1 starter", 2, 200);

  /* Infinite loop */
  while (1)
  {
    if(JOYSTICK_SEL_INT_FIRED)
    {

    }
  }
}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* The following clock configuration sets the Clock configuration sets after System reset                */
  /* It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig */
  /* and to be eventually adapted to new clock configuration                                               */

  /* MSI is enabled after System reset at 4Mhz, PLL not used */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  /* Set 0 Wait State flash latency for 4Mhz */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
}

void  TIM2_Config(void)
{
  /* Compute the prescaler value to have TIM2 counter clock equal to 10 KHz */
  Tim2_PrescalerValue = (uint16_t) (SystemCoreClock/ 10000) - 1;

  Tim2_Handle.Instance = TIM2;

  Tim2_Handle.Init.Period = 5000 - 1;
  Tim2_Handle.Init.Prescaler = Tim2_PrescalerValue;
  Tim2_Handle.Init.ClockDivision = 0;
  Tim2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

  if(HAL_TIM_Base_Init(&Tim2_Handle) != HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_TIM_Base_Start_IT(&Tim2_Handle) != HAL_OK)
  {
    Error_Handler();
  }
}


// configure Timer4 base.
void  TIM4_Config(void)
{

  /* Compute the prescaler value to have TIM2 counter clock equal to 10 KHz */
  Tim4_PrescalerValue = (uint16_t) (SystemCoreClock/ 10000) - 1;

  /* Set TIM2 instance */
  Tim4_Handle.Instance = TIM4;
  Tim4_Handle.Init.Period = 40000;
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
}

void  TIM4_OC_Config(void)
{
  Tim4_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
  Tim4_OCInitStructure.Pulse=Tim4_CCR;
  Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;

  HAL_TIM_OC_Init(&Tim4_Handle);
  HAL_TIM_OC_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
    case GPIO_PIN_0:     //SEL_JOY_PIN
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t*)"select");
      JOYSTICK_SEL_INT_FIRED = 1;
      BSP_LED_Off(LED5);
      factor = (factor+1)%2;
      __HAL_TIM_SET_COMPARE(&Tim4_Handle, TIM_CHANNEL_1, Tim4_CCR/(factor+1));
      break;

    case GPIO_PIN_1: //LEFT_JOY_PIN
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t*)"left");
      break;

    case GPIO_PIN_2: //RIGHT_JOY_PIN
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t*)"right");
      break;

    case GPIO_PIN_3:  //UP_JOY_PIN
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t*)"up");
      break;

    case GPIO_PIN_5:  //DOWN_JOY_PIN
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t*)"down");
      break;

    default:
      BSP_LCD_GLASS_Clear();
      BSP_LCD_GLASS_DisplayString((uint8_t*)"OTHER");
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (JOYSTICK_SEL_INT_FIRED)
  {
    if ( counter < 2)
    {
      BSP_LED_Toggle(LED4);
    }
    else
    {
      BSP_LED_Toggle(LED5);
    }
    // This is just a faster modulus 4
    counter = (counter + 1) & 0x3;
  }
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim)
{
  if (!JOYSTICK_SEL_INT_FIRED)
  {
    BSP_LED_Toggle(LED5);
  }

  __HAL_TIM_SET_COUNTER(htim, 0x0000);
}


static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
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
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
