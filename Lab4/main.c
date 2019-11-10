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

#define TEMP_POLLING_PERIOD                25   // Poll every 25 ms 

#define __SII                              static inline int

#define MAX_ADC_VAL                        65535
#define REF_ADC_VAL                        3.3

#define adc_val_t                          uint16_t
#define flag_t                             volatile uint8_t
#define fan_speed_t                        uint8_t 

#define FLAG_ACTIVE                        1
#define FLAG_INACTIVE                      0

#define FAN_OFF                            0
#define SLOW_SPEED                         45
#define MEDIUM_SPEED                       127
#define FULL_SPEED                         255


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
app_state_e       state;                                             // Main application state
joystick_flags_t  joystickFlags;                                     // Joystick flags instance
float             temperatureSetpoint;                               // Temperature setpoint variable
float             currentTemperature;                                // Current temperature variable


/* Flags */

flag_t            TEMP_ABOVE_SETPOINT_FLAG  = FLAG_INACTIVE;         // Temperature is above setpoint flag
flag_t            TEMP_POLL_TIMER_FLAG      = FLAG_INACTIVE;         // Temperature poll flag
               
               
char              lcd_buffer[85];                                    // LCD display buffer
 


/* Private function prototypes -----------------------------------------------*/

static void  SystemClock_Config         (void);
static void  Error_Handler              (void);

static void  displayTemp                (void);                  
static void  displayTempSetpoint        (void);         
static float getTemp                    (void);
static void  controlFan                 (void);
static void  setFanSpeed                (fan_speed_t);
static void  clearJoystickFlags         (void);

/* State navigation functions */
static void  startMainState             (void);
static void  startSetTempSetpointState  (void);


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

  BSP_LCD_GLASS_DisplayString((uint8_t*)"LAB 4"); 
  HAL_Delay(1000);



  /* Initialize setpoint to current temp on reset */
  currentTemperature  = getTemp();
  temperatureSetpoint = currentTemperature();  

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
          temperatureSetpoint += 0.5;
        }
        /* If down pressed, decrement setpoint by 0.5C */
        if (joystickFlags.down_triggered)
        {
          joystickFlags.down_triggered = FLAG_INACTIVE;
          temperatureSetpoint -= 0.5;
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
    state = APP_STATE_DISPLAY_TIME;
}

/* Go to set temp setpoint state */
static void startSetTempSetpointState(void)
{
    clearJoystickFlags();
    state = APP_STATE_SET_DATE;
}


/******************* Display Functions **********************/

/* Display current time to screen */
static void displayTemp(void) 
{
  BSP_LCD_GLASS_Clear();
  snprintf(lcd_buffer, 7, "%4.1f", currentTemperature);
  BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);  
}

/* Display current date to screen */
static void displayTempSetpoint(void) 
{
  BSP_LCD_GLASS_Clear();
  snprintf(lcd_buffer, 7, "%4.1f", temperatureSetpoint);
  BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);  
}

/***************** Fan Control Functions ********************/

/* Sets fan speed according to current temperature */
static void controlFan(void)
{
  float tempDiff = currentTemperature - temperatureSetpoint;
  if (tempDiff > FULL_SPEED_DIFFERENCE_THRESHOLD)
  {
    return setFanSpeed(FULL_SPEED);
  }
  if (tempDiff > MEDIUM_SPEED_DIFFERENCE_THRESHOLD)
  {
    return setFanSpeed(MEDIUM_SPEED);
  }
  if (tempDiff > 0)
  {
    return setFanSpeed(SLOW_SPEED);
  }
  return setFanSpeed(FAN_OFF);
}

/* Set PWM signal for controlling fan speed */
static void setFanSpeed(fan_speed_t speed) 
{
  // ToDo: Implement
}


/******************* Temp Read Functions ********************/

static uint16_t readADCVal(void)
{
  // ToDo: Implement
  return 1400;
}

static float getTemp(void) 
{
  uint16_t ADCVal = readADCVal();
  return ((ADCVal / MAX_ADC_VAL) * REF_ADC_VAL);
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

