//this file provides read, write, start ,init and stop functionality for master i2c send and receive
#ifndef _I2C_H
#define _I2C_H

#include "stm32L4xx.h"
#include "stm32L4xx_hal_gpio.h"
#include "stm32L4xx_hal_gpio_ex.h"
#include "stm32l476g_discovery.h"
#include "stm32L4xx_hal_i2c.h"
#include "stm32L4xx_hal_rcc.h"
#include "stm32L4xx_hal_rcc_ex.h"


// the following are all defined in _discovery.h
/*
//clock and register pins
#define	I2C_SCL_PIN			GPIO_PIN_6
#define	I2C_SDA_PIN			GPIO_PIN_7
//#define	I2C_SMBA_PIN		GPIO_PIN_5   //STM32L476_DISCO DOES NOT HAVE THIS HEADER PIN

//#define I2C_GPIO_PORT		GPIOB
#define I2C_SCL_GPIO_PORT		GPIOB
#define I2C_SDA_GPIO_PORT		GPIOB


//#define I2C_AF_PORT			GPIO_AF_I2C3
#define I2C_SCL_SDA_AF               GPIO_AF4_I2C1

*/   

// #define I2C_TIMEOUT			1000000//HSI_VALUE

#define PAGE_SIZE 32    // for AT24C64

void I2C_Init(I2C_HandleTypeDef * pI2c_Handle);
HAL_StatusTypeDef I2C_ByteWrite(I2C_HandleTypeDef * pI2c_Handle,uint8_t EEPROM_Addr, uint16_t Mem_Addr, uint8_t Data);
HAL_StatusTypeDef I2C_PageWrite(I2C_HandleTypeDef * pI2c_Handle,uint8_t EEPROM_Addr, uint16_t Mem_Addr, uint8_t * Data, uint8_t datalen);
HAL_StatusTypeDef I2C_BufferWrite(I2C_HandleTypeDef * pI2c_Handle,uint8_t EEPROM_Addr, uint16_t Mem_Addr, uint8_t * dataBuffer, uint16_t datalen);

uint8_t I2C_ByteRead(I2C_HandleTypeDef * pI2c_Handle,uint8_t Addr, uint16_t Reg);
void I2C_Error(I2C_HandleTypeDef * pI2c_Handle);















#ifdef __cplusplus
	extern "C" {
#endif
		

//void NVIC_I2Cx_Config(void);
#ifdef __cplusplus
	}
#endif

#endif
