/******************************************************************************
 * @file    main.c
 * @author  3TA4 Lab 2 Application
 * @version V1.0
 * @date    01-October-2019
 * @brief   Reaction time tester
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedefs ----------------------------------------------------------*/

/* Application States Enum */
typedef enum
{
  APP_STATE_PREGAME = 0,     // LED blinking state, waiting for user input
  APP_STATE_START, // Start game state, wait random amount of time to turn LED on
  APP_STATE_GAME,            // Game in session state
  APP_STATE_RESULT, // Result state - display result, and store result if neccessary
  APP_STATE_CNT              // Number of states
} app_state_e;

/* Timer Types Enum */
typedef enum
{
  TIMER_TYPE_CONTINUOUS = 0, // Timer continuously generates interrupts
  TIMER_TYPE_ONE_SHOT,       // Timer generates interrupt once
  TIMER_TYPE_CNT
} timer_type_e;

/* Private defines -----------------------------------------------------------*/

#define TIMER_IRQ_TRIGGERED       1         // Value of timer flag once triggered
#define TIMER_IRQ_NOT_TRIGGERED   0         // Value of timer flag when not yet triggered

#define BUTTON_IRQ_TRIGGERED      1         // Value of button flag once triggered
#define BUTTON_IRQ_NOT_TRIGGERED  0         // Value of button flag when not yet triggered

#define GENERAL_TIMER             TIM3      // Timer used to wait certain amount of time
#define GAME_TIMER                TIM2      // Timer used to measure user reaction time

#define INDICATOR_LED             LED4      // Indicator LED

#define GAME_TIMER_PRESCALER      100       // Prescaler for game timer

#define MIN_REACTION_TICK_CNT     10        // Minimum reaction time tick count
#define MAX_REACTION_TIME         10000     // Maximum reaction time in ms

#define PREGAME_TIMER_PERIOD      250       // delay time between LED toggles in pregame state in ms

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Application variables */

app_state_e state = APP_STATE_PREGAME;                    // App state variable

uint16_t reflexRecord = MAX_REACTION_TIME-1;        			// Record reflex time value

/* To calculate reflex time, we can take the difference between the timers counts */

uint32_t initialTimerValue = 0; // Initial timer tick count - set when LED is turned on 
uint32_t finalTimerValue = 0; // Final timer tick count - set when user releases button

/* Timer Interrupt Flags */

uint8_t GENERAL_TIMER_FLAG = TIMER_IRQ_NOT_TRIGGERED;     // General timer interrupt flag
uint8_t GAME_TIMER_FLAG = TIMER_IRQ_NOT_TRIGGERED;        // Reaction timer interrupt flag

TIM_HandleTypeDef GeneralTimer_Handle;                    // General Timer Handle
TIM_HandleTypeDef GameTimer_Handle;                       // Game (Reflex) Timer Handle

/* Button Interrupt Flags */

uint8_t USER_BUTTON_FLAG     = BUTTON_IRQ_NOT_TRIGGERED;  // User button interrupt flag
uint8_t USER_RST_BUTTON_FLAG = BUTTON_IRQ_NOT_TRIGGERED;  // User button interrupt flag


/* EEPROM Declarations */
uint16_t EE_status=0;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777}; // the emulated EEPROM can save 3 varibles, at these three addresses.
uint16_t EEREAD;                                          // to practice reading the BESTRESULT save in the EE, for EE read/write, require uint16_t type

/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config ( void );                  // Configure System Clock
static void Error_Handler ( void );                       // Error Handler

static void PregameState_Start ( void );                  // Start Pregame State
static void StartState_Start ( void );                    // Start Game Start State
static void GameState_Start ( void );                     // Start Game State
static void ResultState_Start ( uint32_t );               // Start Result State

static void GeneralTimer_Config ( uint16_t timeout_val ); // Configure General Timer
static void GameTimer_Config ( void );                    // Configure Game Timer

static void EEPROM_Config ( void );                       // Configure EEPROM
static uint16_t getReflexRecord ( void );                 // Get reflex record
static void setReflexRecord ( void );                     // Set reflex record

static void updateRecord ( uint16_t val );                // Update reflex record time
static void resetReflexRecord ( void );                   // Reset reflex value

static void drawTimeToScreen ( uint16_t );                // Draw time value to LCD

static uint32_t getGameTimerValue ( void );               // Get Timer Total Count
static uint16_t getRandomValue ( void );                  // Get random value from RNG

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main Function - handles FSM
 * @param  None
 * @retval None
 */
int main ( void )
{
  /* Initialize Hardware Abstraction Layer */
  HAL_Init();

  /* Configure the system clock to 4 MHz */
  SystemClock_Config();

  /* Set the systick interrupt priority to the highest */
  HAL_InitTick(0x0000);

  /* Configure LEDs */
  BSP_LED_Init(INDICATOR_LED);
  BSP_LED_Off(INDICATOR_LED);

  /* Configure LCD */
  BSP_LCD_GLASS_Init();

  /* Configure User Input Joystick */
  BSP_JOY_Init(JOY_MODE_EXTI);
	
	/* Start EEPROM */
	EEPROM_Config();

  /* Start PREGAME State */
  PregameState_Start();

  /* Main while loop - FSM */
  while (1)
  {
    switch (state)
    {
      case APP_STATE_PREGAME:
        /* If timer interrupt fired */
        if (GENERAL_TIMER_FLAG == TIMER_IRQ_TRIGGERED)
        {
          /* Reset timer flag */
          GENERAL_TIMER_FLAG = TIMER_IRQ_NOT_TRIGGERED;
          /* Toggle LED */
          BSP_LED_Toggle(INDICATOR_LED);
        }
        /* If user button pressed */
        if (USER_BUTTON_FLAG == BUTTON_IRQ_TRIGGERED)
        {
          /* Clear button flag */
          USER_BUTTON_FLAG = BUTTON_IRQ_NOT_TRIGGERED;
          /* Stop Timer */
          HAL_TIM_Base_Stop(&GeneralTimer_Handle);
          /* Clear timer flag */
          GENERAL_TIMER_FLAG = TIMER_IRQ_NOT_TRIGGERED;
          /* Go to START state */
          StartState_Start();
          break;
        }
				/* If user reset button pressed */
        if (USER_RST_BUTTON_FLAG == BUTTON_IRQ_TRIGGERED)
        {
          /* Claer reset button flag */
          USER_RST_BUTTON_FLAG = BUTTON_IRQ_NOT_TRIGGERED;
          /* Reset Record */
					resetReflexRecord();
          break;
        }
        break;
      case APP_STATE_START:
        /* If timer interrupt fired */
        if (GENERAL_TIMER_FLAG == TIMER_IRQ_TRIGGERED)
        {
          /* Reset timer flag */
          GENERAL_TIMER_FLAG = TIMER_IRQ_NOT_TRIGGERED;
          /* Stop timer */
          HAL_TIM_Base_Stop(&GeneralTimer_Handle);
          /* Go to GAME State */
          GameState_Start();
          break;
        }
        break;
      case APP_STATE_GAME:
        /* If user button pressed */
        if (USER_BUTTON_FLAG == BUTTON_IRQ_TRIGGERED)
        {
          /* Reset button flag */
          USER_BUTTON_FLAG = BUTTON_IRQ_NOT_TRIGGERED;

          finalTimerValue = getGameTimerValue();

          /* Go to RESULT state */
          ResultState_Start(finalTimerValue - initialTimerValue);
          break;
        }
        /* If timer interrupt fired */
        if (GAME_TIMER_FLAG == TIMER_IRQ_TRIGGERED)
        {
          /* Reset timer flag */
          GENERAL_TIMER_FLAG = TIMER_IRQ_NOT_TRIGGERED;
          /* Stop timer */
          HAL_TIM_Base_Stop(&GameTimer_Handle);
          /* Go to Result State - pass timeout value*/
          ResultState_Start(0);
          break;
        }
        break;
      case APP_STATE_RESULT:
        /* If user button pressed */
        if (USER_BUTTON_FLAG == BUTTON_IRQ_TRIGGERED)
        {
          /* Reset button flag */
          USER_BUTTON_FLAG = BUTTON_IRQ_NOT_TRIGGERED;

          /* Go to PREGAME state */
          PregameState_Start();
          break;
        }
        break;
      case APP_STATE_CNT:
        while (1)
          ; // Something has gone wrong
    }
  }
}

/* General functions ---------------------------------------------------------*/

/**
 * @brief  Update record - write to EEPROM
 * @param  None
 * @retval None
 */
static void updateRecord ( uint16_t val )
{
  reflexRecord = val;
	setReflexRecord();
}

/**
 * @brief  Reset Reflex Record
 * @param  None
 * @retval None
 */
static void resetReflexRecord ( void )
{
	/* Reset value */
	reflexRecord = MAX_REACTION_TIME - 1;
	setReflexRecord();
}



/**
 * @brief  Draw time in ms to LCD
 * @param  None
 * @retval None
 */
void drawTimeToScreen ( uint16_t val )
{
  val %= MAX_REACTION_TIME;
  char str[6] = "      ";
  snprintf(str, 7, "%dMS   ", val);
  BSP_LCD_GLASS_DisplayString((uint8_t*) str);
}

/**
 * @brief  Get GameTimer main count in ticks
 * @param  None
 * @retval (uint32_t) GameTimer main count
 */
uint32_t getGameTimerValue ( void )
{
  return GAME_TIMER->CNT;
}

/**
 * @brief  Get random value from RNG
 * @param  None
 * @retval (uint32_t) random value
 */
uint16_t getRandomValue ( void )
{
  return 0;
}

/* State managing functions -------------------------------------------------*/

/**
 * @brief  Starts PREGAME State
 * @param  None
 * @retval None
 */
void PregameState_Start ( void )
{
  state = APP_STATE_PREGAME;

  /* Draw Record to LCD */
  drawTimeToScreen(reflexRecord);

  /* Configure Timers */
  GeneralTimer_Config(PREGAME_TIMER_PERIOD);
}

/**
 * @brief  Starts START State
 * @param  None
 * @retval None
 */
void StartState_Start ( void )
{
  state = APP_STATE_START;

  uint32_t delayTime = 2000 + getRandomValue();

  /* Configure Timers */
  GeneralTimer_Config(delayTime);

  GENERAL_TIMER_FLAG = TIMER_IRQ_NOT_TRIGGERED;

  BSP_LED_Off(INDICATOR_LED);
}

/**
 * @brief  Starts GAME State
 * @param  None
 * @retval None
 */
void GameState_Start ( void )
{
  state = APP_STATE_GAME;

  /* Configure Timers */
  GameTimer_Config();

  /* Clear Game Timer Flag */
  GAME_TIMER_FLAG = TIMER_IRQ_NOT_TRIGGERED;

  /* Set initial timer value - used for calculating reflex time */
  initialTimerValue = getGameTimerValue();

  /* Turn Indicator LED on to indicate start of game */
  BSP_LED_On(INDICATOR_LED);

  /* If button is already pressed, they are cheating, return to PREGAME state */
  if (BSP_JOY_GetState() != JOY_NONE)
  {
    PregameState_Start();
  }
}

/**
 * @brief  Starts RESULT State
 * @param  None
 * @retval None
 */
void ResultState_Start ( uint32_t reactionTickCount )
{
  state = APP_STATE_RESULT;

  /* If reaction tick count is too small return to PREGAME state */
  if (reactionTickCount < MIN_REACTION_TICK_CNT)
  {
    PregameState_Start();
    return;
  }

  /* Calculate reaction time in seconds */
  /*
   * Formula:
   * tickCount = number of timer ticks between game start and button release
   * TimerClockFrequency = SystemClock / Prescaler
   * time (in ms) = 1000 * tickCount / (TimerClockFrequency)
   */
  uint32_t reactionTime = reactionTickCount * 1000
      / (SystemCoreClock / GAME_TIMER_PRESCALER);

  /* If current reaction time is less than record, set record */
  if (reactionTime < reflexRecord)
  {
    updateRecord(reactionTime);
  }

  /* Update LCD */
  drawTimeToScreen(reactionTime);
}


/* EEPROM functions -------------------------------------------------*/

/**
 * @brief  Configure EEPROM
 * @param  None
 * @retval None
 */
void EEPROM_Config ( void )
{
	/* Initialize EEPROM */
  EE_status=EE_Init();
	if(EE_status != HAL_OK)
  {
		Error_Handler();
  }
  
	/* Get reflex record, if not found set to existing value */
	if (getReflexRecord())
	{
		return;
	}
	else 
	{
		setReflexRecord();
	}
}

/**
 * @brief  Get Reflex Record from EEPROM
 * @param  None
 * @retval 1 = Variable found, 0 = not found
 */
static uint16_t getReflexRecord ( void )
{
	/* Read variable from EEPROM */
	return (EE_ReadVariable(VirtAddVarTab[0], &reflexRecord) == 0);
}

/**
 * @brief  Set Reflex Record in EEPROM
 * @param  None
 * @retval None
 */
static void setReflexRecord ( void )
{
	/* Write to EEPROM */
	uint16_t i = EE_WriteVariable(VirtAddVarTab[0],  reflexRecord);
	char str[6] = "      ";
  snprintf(str, 7, "%dsetr", i);
	
  BSP_LCD_GLASS_DisplayString((uint8_t*) str);
	HAL_Delay(1000);
}


/* Timer config functions -------------------------------------------------*/

/**
 * @brief  Configure general timer
 * @param  timeout_val - number of ms to trigger interrupt
 * @retval None
 */
void GeneralTimer_Config ( uint16_t timeout_val )
{
  /* Compute the prescaler value to have GeneralTimer counter clock equal to 1 KHz */
  uint16_t PrescalerValue = (uint16_t)(SystemCoreClock / 1000) - 1;

  /* Set GeneralTimer instance */
  GeneralTimer_Handle.Instance = GENERAL_TIMER;

  GeneralTimer_Handle.Init.Period = timeout_val - 1;
  GeneralTimer_Handle.Init.Prescaler = PrescalerValue;
  GeneralTimer_Handle.Init.ClockDivision = 0;
  GeneralTimer_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&GeneralTimer_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&GeneralTimer_Handle) != HAL_OK) //the TIM_XXX_Start_IT function enable IT, and also enable Timer
  //so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
}

/**
 * @brief  Configure game timer
 * @param  None
 * @retval None
 */
void GameTimer_Config ( void )
{
  /* Compute the prescaler value to have PreGameTimer counter clock equal to 10 KHz */
  uint32_t PrescalerValue = GAME_TIMER_PRESCALER;

  /* Set GeneralTimer instance */
  GameTimer_Handle.Instance = GAME_TIMER;
  GameTimer_Handle.Init.Period = (uint32_t) 0xFFFFFFFF; // Set to max value
  GameTimer_Handle.Init.Prescaler = PrescalerValue;
  GameTimer_Handle.Init.ClockDivision = 0;
  GameTimer_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&GameTimer_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&GameTimer_Handle) != HAL_OK) //the TIM_XXX_Start_IT function enable IT, and also enable Timer
  //so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
}

/* Hardware functions -------------------------------------------------*/

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin )
{
  switch (GPIO_Pin) 
	{
		case GPIO_PIN_0:                       //SELECT button                  
				USER_BUTTON_FLAG = BUTTON_IRQ_TRIGGERED;
			break;  
		case GPIO_PIN_1:     //left button                      
								USER_RST_BUTTON_FLAG = BUTTON_IRQ_TRIGGERED;
			break;
		case GPIO_PIN_2:    //right button                        to play again.
						
			break;
		case GPIO_PIN_3:    //up button                         
						
			break;
		
		case GPIO_PIN_5:    //down button                       
				
			break;
		default://
								//default
			break;
	} 
	
	
}

/**
 * @brief Timer interrupt callbacks
 * @param htim - timer handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef * htim )
{
  /* If general timer set corresponding flag */
  if (htim->Instance == GENERAL_TIMER)
  {
    GENERAL_TIMER_FLAG = TIMER_IRQ_TRIGGERED;
  }
  /* If game timer set corresponding flag */
  else if (htim->Instance == GAME_TIMER)
  {
    GAME_TIMER_FLAG = TIMER_IRQ_TRIGGERED;
  }
}

static void Error_Handler ( void )
{
  /* Loop here forever if error occurs*/
  while (1)
    ;
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

void SystemClock_Config ( void )
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };

  // The following clock configuration sets the Clock configuration sets after System reset                
  // It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig 
  // and to be eventually adapted to new clock configuration                                               

  // MSI is enabled after System reset at 4Mhz, PLL not used 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;

//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4; //2, 4,6, 0r 8  ===the clock for RNG will be 4Mhz *N /M/Q =40Mhz. which is nearly 48

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while (1)
      ;
  }

  // Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    // Initialization Error 
    while (1)
      ;
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while (1)
      ;
  }

  // Disable Power Control clock 
  __HAL_RCC_PWR_CLK_DISABLE();
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz

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
