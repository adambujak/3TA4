/******************************************************************************
 * @file    main.c
 * @author  3TA4 Lab 4 Application
 * @version V1.0
 * @date    11-November-2019
 * @brief   Fan Controller - Temperature Reader
 *****************************************************************************/
 
/* Includes ------------------------------------------------------------------*/

#include "main.h"


/* Private define ------------------------------------------------------------*/

#define PWM_TIMER                          TIM1

#define POLLING_TIMER                      TIM3
#define TEMP_POLLING_PERIOD                10000   // Poll every 250 ms 

#define __SII                              static inline int

#define MAX_ADC_VAL                        65535
#define REF_ADC_VAL                        3.3

#define adc_val_t                          uint16_t
#define flag_t                             volatile uint8_t
#define fan_speed_t                        uint8_t 

#define FLAG_ACTIVE                        1
#define FLAG_INACTIVE                      0

#define FAN_OFF                            0
#define SLOW_SPEED                         19
#define MEDIUM_SPEED                       38
#define FULL_SPEED                         100


#define MEDIUM_SPEED_DIFFERENCE_THRESHOLD  3
#define FULL_SPEED_DIFFERENCE_THRESHOLD    5

/* Private typedef -----------------------------------------------------------*/


/* Application States Enum */
typedef enum
{
  APP_STATE_MAIN = 0,             // Measures temperature and controls fan 
  APP_STATE_SET_TEMP,             // Allows user to temperature setpoint
  APP_STATE_CNT                   // Number of states
} app_state_e;


/* Joystick flag type struct */
typedef struct 
{
  flag_t left_triggered;
  flag_t right_triggered;
  flag_t up_triggered;
  flag_t down_triggered;
  flag_t sel_triggered;
  flag_t selHeld_triggered;
} joystick_flags_t;




/* Private variables ---------------------------------------------------------*/

/* Application Variables */
app_state_e          state;                                             // Main application state
joystick_flags_t     joystickFlags;                                     // Joystick flags instance
double               temperatureSetpoint;                               // Temperature setpoint variable
double               currentTemperature;                                // Current temperature variable
                     
                     
/* Flags */          
                     
flag_t               TEMP_ABOVE_SETPOINT_FLAG  = FLAG_INACTIVE;         // Temperature is above setpoint flag
flag_t               TEMP_POLL_TIMER_FLAG      = FLAG_INACTIVE;         // Temperature poll flag
                     
uint8_t              currentSpeed              = 0;                     // Current Fan Speed
                     
char                 lcd_buffer[85];                                    // LCD display buffer
  
ADC_HandleTypeDef    Adc_Handle;

TIM_HandleTypeDef    PWMTimer_Handle;                                   // Timer used to output PWM signal
TIM_HandleTypeDef    PollingTimer_Handle;                               // Timer used to time polling
//TIM_OC_InitTypeDef   Tim3_OCInitStructure;
//TIM_OC_InitTypeDef   Tim4_OCInitStructure;
TIM_OC_InitTypeDef sConfig;

/* Private function prototypes -----------------------------------------------*/

static void   SystemClock_Config         (void);
static void   Error_Handler              (void);

static void   displayTemp                (void);                  
static void   displayTempSetpoint        (void);         
static double getTemp                    (void);
static void   controlFan                 (void);
static void   setFanSpeed                (fan_speed_t);
static void   clearJoystickFlags         (void);

static void   PollingTimer_Config        (uint16_t);
static void   PWMTimer_Config            (uint8_t);

/* State navigation functions */
static void   startMainState             (void);
static void   startSetTempSetpointState  (void);


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Set application state to display time state */
  state = APP_STATE_MAIN;

  /* Clear all Joystick Flags */
  clearJoystickFlags();

  /* Initialize HAL */
  HAL_Init();

  /* Initialize LEDS */
  BSP_LED_Init(LED4);

  SystemClock_Config();   
                                          
  HAL_InitTick(0x0000); 

  BSP_LCD_GLASS_Init();

  BSP_JOY_Init(JOY_MODE_EXTI);


  //PollingTimer_Config(TEMP_POLLING_PERIOD);

  
  
  /* Initialize setpoint to current temp on reset */
  currentTemperature  = getTemp();
  temperatureSetpoint = currentTemperature;  

  /* Display the current temperature */ 
  displayTemp();
    
  /* Infinite loop */
  while (1)
  {     
    switch (state) 
    {
      case APP_STATE_MAIN:
        /* If temp poll timer interrupt has been triggered */
        if (TEMP_POLL_TIMER_FLAG == FLAG_ACTIVE)
        {
          TEMP_POLL_TIMER_FLAG = FLAG_INACTIVE;
          BSP_LED_Toggle(LED4);
          currentTemperature = getTemp();
        }
        /* If select pressed, direct user to temp setpoint setting state */
        if (joystickFlags.sel_triggered)
        {
          joystickFlags.sel_triggered = FLAG_INACTIVE;
          startSetTempSetpointState();
        }
        /* Control fan speed depending on current temperature */
        controlFan();
        /* Display current temperature */
        displayTemp();
        break;
      
      case APP_STATE_SET_TEMP:           
        /* If select pressed, direct user to temp setpoint setting state */
        if (joystickFlags.sel_triggered)
        {
          joystickFlags.sel_triggered = FLAG_INACTIVE;
          startMainState();
        }
        /* If up pressed, increment setpoint by 0.5C */
        if (joystickFlags.up_triggered)
        {
          joystickFlags.up_triggered = FLAG_INACTIVE;
          temperatureSetpoint = temperatureSetpoint + 0.5;
        }
        /* If down pressed, decrement setpoint by 0.5C */
        if (joystickFlags.down_triggered)
        {
          joystickFlags.down_triggered = FLAG_INACTIVE;
          temperatureSetpoint = temperatureSetpoint - 0.5;
        }
        /* Display current temperature setpoint */
        displayTempSetpoint();
        break;

      case APP_STATE_CNT:
        while(1);
        //Something went wrong
    }
  }
}


/******************* State Navigation Functions **********************/

/* Go to main state */
static void startMainState(void)
{
    clearJoystickFlags();
    state = APP_STATE_MAIN;
}

/* Go to set temp setpoint state */
static void startSetTempSetpointState(void)
{
    clearJoystickFlags();
    state = APP_STATE_SET_TEMP;
}


/******************* Display Functions **********************/

/* Display current time to screen */
static void displayTemp(void) 
{
  BSP_LCD_GLASS_Clear();
  snprintf(lcd_buffer, 7, "%04.1fC", currentTemperature);
  lcd_buffer[2] = 'P';
  BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);  
}

/* Display current date to screen */
static void displayTempSetpoint(void) 
{
  BSP_LCD_GLASS_Clear();
  snprintf(lcd_buffer, 7, "-%04.1fC", temperatureSetpoint);
  lcd_buffer[3] = 'P';
  BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);  
}

/***************** Fan Control Functions ********************/

/* Sets fan speed according to current temperature */
static void controlFan(void)
{
  double tempDiff = currentTemperature - temperatureSetpoint;
  if (tempDiff > FULL_SPEED_DIFFERENCE_THRESHOLD)
  {
    if (currentSpeed != FULL_SPEED)
    {
      setFanSpeed(FULL_SPEED);
    }
    return;
  }
  if (tempDiff > MEDIUM_SPEED_DIFFERENCE_THRESHOLD)
  {
    if (currentSpeed != MEDIUM_SPEED)
    {
      setFanSpeed(MEDIUM_SPEED);
    }
    return;
  }
  if (tempDiff > 0)
  {
    if (currentSpeed != SLOW_SPEED)
    {
      setFanSpeed(SLOW_SPEED);
    }
    return;
  }
  if (currentSpeed != FAN_OFF)
  {
    setFanSpeed(FAN_OFF);
  }
  return;
}

/* Set PWM signal for controlling fan speed */
static void setFanSpeed(fan_speed_t speed) 
{
  currentSpeed = speed;
  PWMTimer_Config(speed);
}


/******************* Temp Read Functions ********************/

static uint16_t readADCVal(void)
{
  // ToDo: Implement
  return 1400;
}

static double getTemp(void) 
{
  double ADCVal = (double) readADCVal();
  return ((ADCVal * 100 / MAX_ADC_VAL) * REF_ADC_VAL);
}


/******************* General Functions **********************/

/* Clear all joystick flags */
static void clearJoystickFlags(void)
{
  joystickFlags.left_triggered    = FLAG_INACTIVE;
  joystickFlags.right_triggered   = FLAG_INACTIVE;
  joystickFlags.up_triggered      = FLAG_INACTIVE;
  joystickFlags.down_triggered    = FLAG_INACTIVE;
  joystickFlags.sel_triggered     = FLAG_INACTIVE;
  joystickFlags.selHeld_triggered = FLAG_INACTIVE;
}

/* GPIO Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
    case GPIO_PIN_0:     //SELECT button          
      joystickFlags.sel_triggered   = FLAG_ACTIVE;
      break;  
    case GPIO_PIN_1:     //left button            
      joystickFlags.left_triggered  = FLAG_ACTIVE;
      break;
    case GPIO_PIN_2:    //right button              
      joystickFlags.right_triggered = FLAG_ACTIVE;
      break;
    case GPIO_PIN_3:    //up button             
      joystickFlags.up_triggered    = FLAG_ACTIVE;
      break;
    case GPIO_PIN_5:    //down button           
      joystickFlags.down_triggered  = FLAG_ACTIVE;
      break;  
    default://
      //default
      break;
  } 
}

static void Error_Handler(void)
{
  /* Loop here forever if error occurs*/
  while (1);
}

/******************* Timer  Functions *********************/

/**
 * @brief  Configure polling timer
 * @param  timeout_val - number of ms to trigger interrupt
 * @retval None
 */
void PollingTimer_Config(uint16_t timeout_val)
{
  /* Compute the prescaler value to have PollingTimer counter clock equal to 1 KHz */
  uint16_t PrescalerValue = (uint16_t)(SystemCoreClock / 1000) - 1;

  /* Set PollingTimer instance */
  PollingTimer_Handle.Instance = POLLING_TIMER;

  PollingTimer_Handle.Init.Period = timeout_val - 1;
  PollingTimer_Handle.Init.Prescaler = PrescalerValue;
  PollingTimer_Handle.Init.ClockDivision = 0;
  PollingTimer_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&PollingTimer_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&PollingTimer_Handle) != HAL_OK) //the TIM_XXX_Start_IT function enable IT, and also enable Timer
  //so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
}

/**
 * @brief  Configure pwm timer
 * @param  duty cycle
 * @retval None
 */
void PWMTimer_Config(uint8_t dutyCycle)
{
  #define  PERIOD_VALUE       (uint32_t)(666 - 1)               /* Period Value  */
 
  if (dutyCycle <= 0) 
  {
    HAL_TIM_PWM_Stop(&PollingTimer_Handle, TIM_CHANNEL_2);
    return;
  }
  
  uint32_t PULSE_VALUE; 
  
  if (dutyCycle >= 100) 
  {
    PULSE_VALUE = PERIOD_VALUE - 1;
  }
  else 
  {
    PULSE_VALUE = (uint32_t)(PERIOD_VALUE*dutyCycle/100); /* Capture Compare 2 Value  */
  }
  
  /* Timer Output Compare Configuration Structure declaration */
  
  uint32_t uhPrescalerValue                  = (uint32_t)(SystemCoreClock / 16000000) - 1;
  
  /* Set PWM Timer instance */
  PollingTimer_Handle.Instance               = PWM_TIMER;

  PollingTimer_Handle.Init.Prescaler         = uhPrescalerValue;
  PollingTimer_Handle.Init.Period            = PERIOD_VALUE;
  PollingTimer_Handle.Init.ClockDivision     = 0;
  PollingTimer_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  PollingTimer_Handle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&PollingTimer_Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

  /* Set the pulse value for channel 2 */
  sConfig.Pulse = PULSE_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&PollingTimer_Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Start channel 2 */
  if (HAL_TIM_PWM_Start(&PollingTimer_Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
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
  BSP_LED_Toggle(LED4);
  if (htim->Instance == POLLING_TIMER)
  {
    TEMP_POLL_TIMER_FLAG = FLAG_ACTIVE;
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM4_pwm
    
    __HAL_TIM_SET_COUNTER(htim, 0x0000);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{

}


/******************* System Configuration Functions **********************/

void SystemClock_Config(void)
{ 
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
  //reading from RTC requires the APB clock is 7 times faster than HSE clock, 
  //so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;     //RTC need either HSE, LSE or LSI           
  
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;
  
    //RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;//RCC_PLL_NONE;

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
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz

