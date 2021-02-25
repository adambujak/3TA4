
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/

#define pincnt_t                    uint8_t
#define flag_t                      uint8_t

#define FLAG_ACTIVE                 1
#define FLAG_INACTIVE               0

#define STEP_TIMER                  TIM3
#define DEFAULT_ROTATION_PERIOD     47
#define STEP_CNT                    48
#define POLE_CNT                    4

#define FULL_STEP                   1
#define HALF_STEP                   0

#define CLOCKWISE                   0
#define COUNTER_CLOCKWISE           1



/* Private typedef -----------------------------------------------------------*/

/* Application States Enum */
typedef enum
{
  APP_STATE_HALF_STEPS = 0,             // Measures temperature and controls fan
  APP_STATE_FULL_STEPS,                 // Allows user to temperature setpoint
  APP_STATE_CNT                         // Number of states
} app_state_e;

typedef struct
{
  GPIO_PinState * pinStates;
  GPIO_TypeDef  * port;
  uint16_t      * pins;
  pincnt_t        size;
  pincnt_t        currentIndex;
} PinStateTracker;


/* Private variables ---------------------------------------------------------*/

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY

GPIO_PinState     pinStatesArray[POLE_CNT];
uint16_t          pins[POLE_CNT]             = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
PinStateTracker   pinStateTracker            = {pinStatesArray, GPIOE, pins, POLE_CNT, 0};

volatile uint8_t  cycleDirection             = CLOCKWISE;


volatile flag_t   stepTimerFlag              = FLAG_INACTIVE;
volatile flag_t   joystickPressedFlag        = FLAG_INACTIVE;
volatile flag_t   speedChangedFlag           = FLAG_INACTIVE;
volatile flag_t   directionChangedFlag       = FLAG_INACTIVE;

volatile uint8_t  rotationPeriod             = DEFAULT_ROTATION_PERIOD;

TIM_HandleTypeDef stepTimerHandle;

app_state_e       state;



char              lcd_buffer[6];


/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config   ( void );
static void Error_Handler        ( void );

static void GPIO_Config          ( void );
static void initPinStateTracker  ( PinStateTracker * pinStateTracker );
static void halfStep             ( PinStateTracker * pinStateTracker );
static void fullStep             ( PinStateTracker * pinStateTracker );
static void setGPIOPins          ( PinStateTracker * pinStateTracker );

static void startHalfStepState   ( void );
static void startFullStepState   ( void );

static void startTimer           ( uint8_t stepSize );

static void EXTILine14_Config(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  HAL_Init();


  SystemClock_Config();   //sysclock is 80Hz. HClkm apb1 an apb2 are all 80Mhz.

  HAL_InitTick(0x0000); // set systick's priority to the highest.


  BSP_LED_Init(LED4);
  BSP_LED_Init(LED5);

  BSP_LCD_GLASS_Init();

  BSP_JOY_Init(JOY_MODE_EXTI);

  GPIO_Config();

  /* Start Half Step State (default) */
  startHalfStepState();

  while (1)
  {
    if (directionChangedFlag == FLAG_ACTIVE)
    {
      directionChangedFlag = FLAG_INACTIVE;
      initPinStateTracker(&pinStateTracker);
      setGPIOPins(&pinStateTracker);
    }
    switch (state)
    {
      case (APP_STATE_HALF_STEPS):
        /* If joystick is pressed switch states */
        if (joystickPressedFlag == FLAG_ACTIVE)
        {
          joystickPressedFlag = FLAG_INACTIVE;
          startFullStepState();
          break;
        }
        /* If timer interrupted, step */
        if (stepTimerFlag == FLAG_ACTIVE)
        {
          stepTimerFlag = FLAG_INACTIVE;
          halfStep(&pinStateTracker);
          setGPIOPins(&pinStateTracker);
        }
        /* If speed is changed, reconfigure half step timer */
        if (speedChangedFlag == FLAG_ACTIVE)
        {
          speedChangedFlag = FLAG_INACTIVE;
          startTimer(HALF_STEP);
        }
        break;
      case (APP_STATE_FULL_STEPS):
        /* If joystick is pressed switch states */
        if (joystickPressedFlag == FLAG_ACTIVE)
        {
          joystickPressedFlag = FLAG_INACTIVE;
          startHalfStepState();
          break;
        }
        /* If timer interrupted, step */
        if (stepTimerFlag == FLAG_ACTIVE)
        {
          stepTimerFlag = FLAG_INACTIVE;
          fullStep(&pinStateTracker);
          setGPIOPins(&pinStateTracker);
        }
        /* If speed is changed, reconfigure full step timer */
        if (speedChangedFlag == FLAG_ACTIVE)
        {
          speedChangedFlag = FLAG_INACTIVE;
          startTimer(FULL_STEP);
        }
        break;
      case (APP_STATE_CNT):
        while(1); // Something went wrong
        break;
    }
  }
  return 0;
}

/**
  * @brief  Configure Timer
  * @param  stepSize - Half Step or Full Step
  * @retval None
  */
static void startTimer( uint8_t stepSize )
{
  #define DIVISOR 5000
  /* Compute the prescaler value to have TIM3 counter clock equal to 5 kHz */
  uint16_t prescalerValue = (uint16_t) (SystemCoreClock / DIVISOR) - 1;

  /* Set TIM2 instance */
  stepTimerHandle.Instance = STEP_TIMER;
  uint32_t period = (rotationPeriod * DIVISOR) / STEP_CNT;
  if (stepSize == HALF_STEP)
  {
    period /= 2;
  }
  stepTimerHandle.Init.Period = ((uint16_t)(period)) - 1;
  stepTimerHandle.Init.Prescaler = prescalerValue;
  stepTimerHandle.Init.ClockDivision = 0;
  stepTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&stepTimerHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  if(HAL_TIM_Base_Start_IT(&stepTimerHandle) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
}

/**
  * @brief  Start Half Step State
  * @param  None
  * @retval None
  */
static void startHalfStepState ( void )
{
  state = APP_STATE_HALF_STEPS;
  stepTimerFlag = FLAG_INACTIVE;
  joystickPressedFlag = FLAG_INACTIVE;
  initPinStateTracker(&pinStateTracker);
  setGPIOPins(&pinStateTracker);
  BSP_LCD_GLASS_DisplayString((uint8_t*)"HALF");
  startTimer(HALF_STEP);
}

/**
  * @brief  Start Full Step State
  * @param  None
  * @retval None
  */
static void startFullStepState ( void )
{
  state = APP_STATE_FULL_STEPS;
  stepTimerFlag = FLAG_INACTIVE;
  joystickPressedFlag = FLAG_INACTIVE;
  initPinStateTracker(&pinStateTracker);
  setGPIOPins(&pinStateTracker);
  BSP_LCD_GLASS_DisplayString((uint8_t*)"FULL");
  startTimer(FULL_STEP);
}
/**
  * @brief  Initialize Pin State Tracker
  * @param  Pointer to pin state tracker
  * @retval None
  */
static void initPinStateTracker ( PinStateTracker * pinStateTracker )
{
  pinStateTracker->currentIndex = 0;
  for (uint8_t i = 0; i < pinStateTracker->size; i++)
  {
    pinStateTracker->pinStates[i] = GPIO_PIN_RESET;
  }
}

/**
  * @brief  Sets GPIO pins corresponding to pinStateTracker
  * @param  Pointer to pin state tracker
  * @retval None
  */
static void setGPIOPins ( PinStateTracker * pinStateTracker )
{
  for (pincnt_t i = 0; i < pinStateTracker->size; i++)
  {
    HAL_GPIO_WritePin(pinStateTracker->port, pinStateTracker->pins[i], pinStateTracker->pinStates[i]);
  }
}

/**
  * @brief  Get next index in pinstate array
  * @param  currentIndex - current index in pinstate array
  * @retval next index in pinstate array
  */
static pincnt_t getNextIndex ( pincnt_t currentIndex, pincnt_t size, uint8_t direction )
{
  if (direction == COUNTER_CLOCKWISE)
  {
    if (currentIndex == 0)
    {
      return (size-1);
    }
    return (currentIndex - 1);
  }
  if (direction == CLOCKWISE)
  {
    if (currentIndex == (size - 1))
    {
      return 0;
    }
    return (currentIndex + 1);
  }
}



/**
  * @brief  Half Step
  * @param  Pointer to pin state tracker
  * @retval None
  */
static void halfStep ( PinStateTracker * pinStateTracker )
{
  pincnt_t nextIndex = getNextIndex(pinStateTracker->currentIndex, pinStateTracker->size, (cycleDirection));

  /* Base Case - nothing set for any state */
  if (pinStateTracker->pinStates[pinStateTracker->currentIndex] == GPIO_PIN_RESET)
  {
    pinStateTracker->pinStates[pinStateTracker->currentIndex] = GPIO_PIN_SET;
  }
  /* Half Step State - if next pin is clear, set the next pin */
  else if (pinStateTracker->pinStates[nextIndex] == GPIO_PIN_RESET)
  {
    pinStateTracker->pinStates[nextIndex] = GPIO_PIN_SET;
  }
  /* Full Step State - if next pin is set, clear current pin */
  else
  {
    pinStateTracker->pinStates[pinStateTracker->currentIndex] = GPIO_PIN_RESET;
    pinStateTracker->currentIndex = nextIndex;
  }
}

/**
  * @brief  Full Step
  * @param  Pointer to pin state tracker
  * @retval None
  */
static void fullStep ( PinStateTracker * pinStateTracker )
{
  pincnt_t nextIndex = getNextIndex(pinStateTracker->currentIndex, pinStateTracker->size, cycleDirection);

  /* Base Case - nothing set for any state */
  if (pinStateTracker->pinStates[pinStateTracker->currentIndex] == GPIO_PIN_RESET)
  {
    pinStateTracker->pinStates[pinStateTracker->currentIndex] = GPIO_PIN_SET;
  }
  /* If current pin is set, set the next one and clear current one */
  else if (pinStateTracker->pinStates[pinStateTracker->currentIndex] == GPIO_PIN_SET)
  {
    pinStateTracker->pinStates[pinStateTracker->currentIndex] = GPIO_PIN_RESET;
    pinStateTracker->pinStates[nextIndex] = GPIO_PIN_SET;
    pinStateTracker->currentIndex = nextIndex;
  }
}

/**
  * @brief  GPIO Configuration
  *         Pins:
  *           PE12
  *           PE13
  *           PE14
  *           PE15
  * @param  None
  * @retval None
  */
static void GPIO_Config ( void )
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
  //reading from RTC requires the APB clock is 7 times faster than HSE clock,
  //so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8
  //the PLL will be MSI (4Mhz)*N /M/R =

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers
  // Set 0 Wait State flash latency for 4Mhz
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;


  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
  {
    // Initialization Error
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.

  // Enable Power Control clock
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz??





/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case GPIO_PIN_0:                   //SELECT button
      joystickPressedFlag = FLAG_ACTIVE;
      break;
    case GPIO_PIN_1:     //left button
      cycleDirection ^= 1;    //Toggle Direction
      directionChangedFlag = FLAG_ACTIVE;
      break;
    case GPIO_PIN_2:     //right button
      break;

    case GPIO_PIN_3:     //up button
      if (rotationPeriod >= 5)
      {
        rotationPeriod -= 5;
      }
      speedChangedFlag = FLAG_ACTIVE;
      break;

    case GPIO_PIN_5:     //down button
      if (rotationPeriod <= 250)
      {
        rotationPeriod += 5;
      }
      speedChangedFlag = FLAG_ACTIVE;
      break;

    default:
      break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  stepTimerFlag = FLAG_ACTIVE;
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
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

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
