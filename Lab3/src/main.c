/******************************************************************************
 * @file    main.c
 * @author  3TA4 Lab 3 Application
 * @version V1.0
 * @date    28-October-2019
 * @brief   I2C EEPROM Application
 *****************************************************************************/
 
/* Includes ------------------------------------------------------------------*/

#include "main.h"


/* Private define ------------------------------------------------------------*/

#define FLAG_ACTIVE   1
#define FLAG_INACTIVE 0
#define flag_t volatile uint8_t


/* Private typedef -----------------------------------------------------------*/


/* Application States Enum */
typedef enum
{
  APP_STATE_DISPLAY_TIME = 0,     // Displays time - updates once / second
  APP_STATE_DISPLAY_DATE,         // Displays date
  APP_STATE_DISPLAY_PAST_TIMES,   // Displays past two times select was pressed
  APP_STATE_SET_DATE,             // Allows user to set date
  APP_STATE_CNT                   // Number of states
} app_state_e;

typedef enum
{
  TIME_SET_STATE_YEAR = 0,     // Set year state
  TIME_SET_STATE_MONTH,        // Set month state
  TIME_SET_STATE_DAY,          // Set day state
  TIME_SET_STATE_HOUR,         // Set hour state
	TIME_SET_STATE_MINUTE,       // Set minute state
	TIME_SET_STATE_SECOND,	     // Set second state
  TIME_SET_STATE_CNT           // Number of states
} time_set_state_e;


typedef struct 
{
	flag_t left_triggered;
	flag_t right_triggered;
	flag_t up_triggered;
	flag_t down_triggered;
	flag_t sel_triggered;
	flag_t selHeld_triggered;
} joystick_flags_t;

typedef struct 
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} time_t;


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Application Variables */
app_state_e            state;
time_set_state_e       timeSetState = TIME_SET_STATE_YEAR;
joystick_flags_t       joystickFlags;

/* Strings */

char                   yearStr[]   = {"Yr"};
char                   monthStr[]  = {"Mon"};
char                   dayStr[]    = {"Day"};
char                   hourStr[]   = {"Hr"};
char                   minuteStr[] = {"Min"};
char                   secondStr[] = {"Sec"};

/* Flags */

flag_t RTC_INT_TRIGGERED_FLAG = FLAG_ACTIVE;



I2C_HandleTypeDef      pI2c_Handle;

RTC_HandleTypeDef      RTCHandle;
RTC_DateTypeDef        RTC_DateStructure;
RTC_TimeTypeDef        RTC_TimeStructure;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

//memory location to write to in the device
__IO uint16_t          memLocation = 0x000A; //pick any location within range

  

char                   lcd_buffer[85];       // LCD display buffer
char                   timestring[10]={0};  //   
char                   datestring[6]={0};


__IO uint32_t SEL_Pressed_StartTick;   //sysTick when the User button is pressed


/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config         (void);
static void Error_Handler              (void);
																	    
void        RTC_Config                 (void);
void        RTC_AlarmAConfig           (void);
void        displayTime                (void);
void        displayDate                (void);
void        displayPastTimes           (void);
void        getPastTimes               (time_t *, time_t *);
void        saveCurrentTime            (void);
void        incrementTimeSettingState  (time_set_state_e *);
void        incrementTimeSetting       (time_set_state_e *);
void        updateTimeSetScreen        (time_set_state_e *);
																			 
																			 
void        startDisplayTimeState      (void);
void        startDisplayDateState      (void);
void        startDisplayPastTimesState (void);
void        startSetTimeState          (void);



/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	state = APP_STATE_DISPLAY_TIME;
	
	joystickFlags.left_triggered    = FLAG_INACTIVE;
	joystickFlags.right_triggered   = FLAG_INACTIVE;
	joystickFlags.up_triggered      = FLAG_INACTIVE;
	joystickFlags.down_triggered    = FLAG_INACTIVE;
	joystickFlags.sel_triggered     = FLAG_INACTIVE;
	joystickFlags.selHeld_triggered = FLAG_INACTIVE;
		

	HAL_Init();
	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
  
	SystemClock_Config();   
											
	HAL_InitTick(0x0000); 

	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);

	BSP_LCD_GLASS_DisplayString((uint8_t*)"LAB 3");	
	HAL_Delay(1000);


  /* Configure Real Time Clock */
	RTC_Config();
	
	RTC_AlarmAConfig();
	
	I2C_Init(&pI2c_Handle);


	
  /* Infinite loop */
  while (1)
  {
		//the joystick is pulled down. so the default status of the joystick is 0, when pressed, get status of 1. 
		//while the interrupt is configured at the falling edge---the moment the pressing is released, the interrupt is triggered.
		//therefore, the variable "selpressed==1" can not be used to make choice here.
		if (BSP_JOY_GetState() == JOY_SEL) {
			SEL_Pressed_StartTick=HAL_GetTick(); 
			while(BSP_JOY_GetState() == JOY_SEL) {  //while the selection button is pressed)	
				if ((HAL_GetTick()-SEL_Pressed_StartTick)>800) {	
					joystickFlags.sel_triggered = FLAG_INACTIVE;
					joystickFlags.selHeld_triggered = FLAG_ACTIVE;
				} 
			}
		}					

		
		switch (state) 
		{
			case APP_STATE_DISPLAY_TIME:
				/* If RTC interrupt has been triggered */
				if (RTC_INT_TRIGGERED_FLAG == FLAG_ACTIVE)
				{
					if (HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BCD) == HAL_OK)
					{
						BSP_LED_Toggle(LED5);
						displayTime();
					} 
					HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BCD);	
					RTC_INT_TRIGGERED_FLAG = FLAG_INACTIVE;
				}				
				/* If select pressed, save current time to eeprom */
				if (joystickFlags.sel_triggered)
				{
					joystickFlags.sel_triggered = FLAG_INACTIVE;
					saveCurrentTime();
				}
				/* If select held, display date */
				if (joystickFlags.selHeld_triggered) 
				{
					joystickFlags.selHeld_triggered = FLAG_INACTIVE;
					startDisplayDateState();
				}
				/* If left pressed show previous times sel was pressed */
				else if (joystickFlags.left_triggered) 
				{
					joystickFlags.left_triggered = FLAG_INACTIVE;
					startDisplayPastTimesState();
				}
				/* If right pressed go to set date and time state */
				else if (joystickFlags.right_triggered) 
				{
					joystickFlags.right_triggered = FLAG_INACTIVE;
					startSetTimeState();
				}
				break;
			
			case APP_STATE_DISPLAY_DATE:
				/* If RTC Interrupt is triggered update the screen */
				if (RTC_INT_TRIGGERED_FLAG == FLAG_ACTIVE)
				{
					HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BCD);	
					displayDate();
					RTC_INT_TRIGGERED_FLAG = FLAG_INACTIVE;
				}				
				/* If joystick sel is pressed or held, go back to display time state */
				if (joystickFlags.sel_triggered == FLAG_ACTIVE)
				{
					startDisplayTimeState();
				}
				else if (joystickFlags.selHeld_triggered == FLAG_ACTIVE)
				{
					startDisplayTimeState();
				}
				break;
			
			case APP_STATE_DISPLAY_PAST_TIMES:
				/* If left button pressed go back to display time state */
				if (joystickFlags.left_triggered) 
				{
					joystickFlags.left_triggered = FLAG_INACTIVE;
					startDisplayTimeState();
				}
				break;
			
			case APP_STATE_SET_DATE:
				
				/* If left pressed, change setting field */
			  if (joystickFlags.left_triggered) 
				{
					joystickFlags.left_triggered = FLAG_INACTIVE;
					incrementTimeSettingState(&timeSetState);
					updateTimeSetScreen(&timeSetState);
				}
			
				/* If sel pressed, increment data in field */
			  if (joystickFlags.sel_triggered) 
				{
					joystickFlags.sel_triggered = FLAG_INACTIVE;
					incrementTimeSetting(&timeSetState);
					updateTimeSetScreen(&timeSetState);
				}
				
				/* If right button pressed go back to display time state */
				if (joystickFlags.right_triggered) 
				{
					joystickFlags.right_triggered = FLAG_INACTIVE;
					startDisplayTimeState();
				}
				
				break;
			
			case APP_STATE_CNT:
				while(1);
				//Something went wrong
		}
	}
}
void clearJoystickFlags(void)
{
	joystickFlags.left_triggered    = FLAG_INACTIVE;
	joystickFlags.right_triggered   = FLAG_INACTIVE;
	joystickFlags.up_triggered      = FLAG_INACTIVE;
	joystickFlags.down_triggered    = FLAG_INACTIVE;
	joystickFlags.sel_triggered     = FLAG_INACTIVE;
	joystickFlags.selHeld_triggered = FLAG_INACTIVE;
}


/* State Functions */

void startDisplayTimeState(void)
{
	clearJoystickFlags();
	state = APP_STATE_DISPLAY_TIME;
	displayTime();
}

void startDisplayDateState(void)
{
	clearJoystickFlags();
	state = APP_STATE_DISPLAY_DATE;
	displayDate();
}


void startDisplayPastTimesState(void)
{
	clearJoystickFlags();
	state = APP_STATE_DISPLAY_PAST_TIMES;
	displayPastTimes();
}

void startSetTimeState(void)
{
	clearJoystickFlags();
	state = APP_STATE_SET_DATE;
	updateTimeSetScreen(&timeSetState);
}


static inline int bcd_decimal(uint8_t hex)
{
  int dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
  return dec;
}      

static inline int decimal_bcd(unsigned int num) // say num is now 100
{
    unsigned int ones = 0;
    unsigned int tens = 0;
    unsigned int temp = 0;

    ones = num%10; // 100%10 = 0
    temp = num/10; // 100/10 = 10, or 0x0A
    tens = temp<<4;  
    return (tens + ones);// so the result is A0
}


void displayTime(void) 
{
	BSP_LCD_GLASS_Clear();
	snprintf(lcd_buffer, 7, "%02d%02d%02d  ", bcd_decimal(RTC_TimeStructure.Hours), bcd_decimal(RTC_TimeStructure.Minutes), bcd_decimal(RTC_TimeStructure.Seconds));
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);	
}

void displayDate(void) 
{
	snprintf(lcd_buffer, 12, "%02d/%02d/%02d - ", bcd_decimal(RTC_DateStructure.Year), bcd_decimal(RTC_DateStructure.Month), bcd_decimal(RTC_DateStructure.Date));
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)lcd_buffer, 1, 800);	
}

void displayPastTimes(void) 
{
	BSP_LCD_GLASS_Clear();
	time_t time1;
	time_t time2;
	getPastTimes(&time1, &time2);
	snprintf(lcd_buffer, 32, "%02d %02d %02d - %02d %02d %02d  -- ", time1.hour, time1.minute, time1.second, time2.hour, time2.minute, time2.second);
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)lcd_buffer, 1, 800);	
}

void displaySetTime(char * label, uint8_t val)
{
	BSP_LCD_GLASS_Clear();
	snprintf(lcd_buffer, 12, "%s %d", label, bcd_decimal(val));
	BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);	
}

void getPastTimes (time_t * time1, time_t * time2)
{
	time1->hour   = 3;
	time1->minute = 32;
	time1->second = 36;
	
	time2->hour   = 4;
	time2->minute = 42;
	time2->second = 46;
}

void incrementTimeSettingState (time_set_state_e * state)
{
	(*state)++;
	if ((*state) == TIME_SET_STATE_CNT)
	{
		(*state) = TIME_SET_STATE_YEAR;
	}
	
}

void incrementYear(void)
{
	uint16_t num = bcd_decimal((uint16_t) (RTC_DateStructure.Year)) + 1;
	if (num >= 100 )
	{
		num = 0;
	}
	RTC_DateStructure.Year = decimal_bcd(num);
}

void incrementMonth(void)
{
	uint16_t num = bcd_decimal((uint16_t) (RTC_DateStructure.Month)) + 1;
	if (num >= 12 )
	{
		num = 0;
	}
	RTC_DateStructure.Month = decimal_bcd(num);
}

void incrementDay(void)
{
	uint16_t num = bcd_decimal((uint16_t) (RTC_DateStructure.Date)) + 1;
	if (num >= 31 )
	{
		num = 0;
	}
	RTC_DateStructure.Date = decimal_bcd(num);
}


void incrementHour(void)
{
	uint16_t num = bcd_decimal((uint16_t) (RTC_TimeStructure.Hours)) + 1;
	if (num >= 12 )
	{
		num = 0;
	}
	RTC_TimeStructure.Hours = decimal_bcd(num);
}
void incrementMinute(void)
{
	uint16_t num = bcd_decimal((uint16_t) (RTC_TimeStructure.Minutes)) + 1;
	if (num >= 60 )
	{
		num = 0;
	}
	RTC_TimeStructure.Minutes = decimal_bcd(num);
}
void incrementSecond(void)
{
	uint16_t num = bcd_decimal((uint16_t) (RTC_TimeStructure.Seconds)) + 1;
	if (num >= 60 )
	{
		num = 0;
	}
	RTC_TimeStructure.Seconds = decimal_bcd(num);
}

void incrementTimeSetting (time_set_state_e * state)
{
	switch ((*state))
	{
		case TIME_SET_STATE_YEAR: 
			incrementYear();
			break;
		case TIME_SET_STATE_MONTH: 
			incrementMonth();
			break;
		case TIME_SET_STATE_DAY: 
			incrementDay();
			break;
		case TIME_SET_STATE_HOUR: 
			incrementHour();
			break;
		case TIME_SET_STATE_MINUTE: 
			incrementMinute();
			break;
		case TIME_SET_STATE_SECOND: 
			incrementSecond();
			break;
		case TIME_SET_STATE_CNT: 
			while (1); //something went wrong
	}
	HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BCD);
	HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BCD);
	HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BCD);	
	updateTimeSetScreen(state);
	
}
void updateTimeSetScreen (time_set_state_e * state)
{
	switch ((*state))
	{
		case TIME_SET_STATE_YEAR: 
			displaySetTime(yearStr, (uint8_t) RTC_DateStructure.Year);
			break;
		case TIME_SET_STATE_MONTH: 
			displaySetTime(monthStr, (uint8_t) RTC_DateStructure.Month);
			break;
		case TIME_SET_STATE_DAY: 
			displaySetTime(dayStr, (uint8_t) RTC_DateStructure.Date);
			break;
		case TIME_SET_STATE_HOUR: 
			displaySetTime(hourStr, (uint8_t) RTC_TimeStructure.Hours);
			break;
		case TIME_SET_STATE_MINUTE: 
			displaySetTime(minuteStr, (uint8_t) RTC_TimeStructure.Minutes);
			break;
		case TIME_SET_STATE_SECOND: 
			displaySetTime(secondStr, (uint8_t) RTC_TimeStructure.Seconds);
			break;
		case TIME_SET_STATE_CNT: 
			while (1); //something went wrong
		
	}
}
void saveCurrentTime(void)
{
	
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


void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	

	//****1:***** Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE();    
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();    
			//1.3: Select the RTC clock source
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);    
				//RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
				
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC --see note for the Macro in _hal_rcc.h---using this Marco requires 
																//the above three lines.
			
	
			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {}    //defind in rcc.c
	
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
				
				
				
	//****2.*****  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        

			RTCHandle.Instance = RTC;
			RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
			
			RTCHandle.Init.AsynchPrediv = 127; 
			RTCHandle.Init.SynchPrediv = 255; 
			
			
			RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
			RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
			RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
			
		
			if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
			{
				BSP_LCD_GLASS_Clear(); 
				BSP_LCD_GLASS_DisplayString((uint8_t *)"RT I X"); 	
			}

			
			

			RTC_DateStructure.Year = 19;
			RTC_DateStructure.Month = RTC_MONTH_OCTOBER;
			RTC_DateStructure.Date = 27;
			RTC_DateStructure.WeekDay = RTC_WEEKDAY_MONDAY;
			
			if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better 
														//before, must set in BCD format and read in BIN format!!
			{
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"D I X");
			} 


			RTC_TimeStructure.Hours = 2;  
			RTC_TimeStructure.Minutes = 59;
			RTC_TimeStructure.Seconds = 55;
			RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_PM;
			RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
			RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;
			
			if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better
																																				//before, must set in BCD format and read in BIN format!!
			{
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"T I X");
			}	
	




			
		__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
		__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
			//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
			//Timestamp on tamper event
			//With TAMPTS set to ‘1 , any tamper event causes a timestamp to occur. In this case, either
			//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
			//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
			//same time that TSF or TSOVF is set. ---P802, about Tamper detection
			//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
			//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
			//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
			//---function. 
			
		HAL_RTC_WaitForSynchro(&RTCHandle);	
		//To read the calendar through the shadow registers after Calendar initialization,
		//		calendar update or after wake-up from low power modes the software must first clear
		//the RSF flag. The software must then wait until it is set again before reading the
		//calendar, which means that the calendar registers have been correctly copied into the
		//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
		//implements the above software sequence (RSF clear and RSF check).	
}


void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;

  RTC_Alarm_Structure.Alarm = RTC_ALARM_A;
  RTC_Alarm_Structure.AlarmMask = RTC_ALARMMASK_ALL;
	
  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
			BSP_LCD_GLASS_Clear(); 
			BSP_LCD_GLASS_DisplayString((uint8_t *)"A S X");
  }

	__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //without this line, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		

		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);  //not important ,but it is better not use the same prio as the systick
	
}

//You may need to disable and enable the RTC Alarm at some moment in your application
HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

}



/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
		case GPIO_PIN_0: 		 //SELECT button					
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



void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	RTC_INT_TRIGGERED_FLAG = FLAG_ACTIVE;
}

static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}

void eepromTest( void )
{

	//the following variables are for testging I2C_EEPROM
	uint8_t data1 =0x67,  data2=0x68;
	uint8_t readData=0x00;
	uint16_t EE_status;


	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, data1);

  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 X");

	HAL_Delay(1000);
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1 , data2);
	
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 X");

	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);

	BSP_LCD_GLASS_Clear();
	if (data1 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	
	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);

	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	

	HAL_Delay(1000);
	
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
