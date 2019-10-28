#include "i2c_at24c64.h"

/******************Robert Li *****************
This library file is for STM32L476g-doscovery board working with AT24c64 EEPROM
1. the stm32l476_discovery.c file has some library functions for I2Cx, but there is no byte write and no byte read functions for I2C1
   while the board stm32l476g_discovery board can only use I2C1, because only PB7 and PB6 are available for I2C alternative functions
	PB7 and PB6 are for I2C1. 
2. At24C64 needs to delay 10 ms after writting to EEPROM. while the I2C functions in stm32l476_discovery.c does not provide this delay. 
3. stm32l476g_discovery.c did not export the I2C functions. -----Still need this file in lab3 project.
3. functions in this file are for I2C1.

************************************/


//static I2C_HandleTypeDef * pI2c_Handle;

HAL_StatusTypeDef status = HAL_OK;


// defined in the stm32l476_discovery.c

static void I2C_MspInit(void)
{
		GPIO_InitTypeDef  GPIO_InitStruct;  
		RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
		
		// IOSV bit MUST be set to access GPIO port G[2:15]   //??
		__HAL_RCC_PWR_CLK_ENABLE();
		HAL_PWREx_EnableVddIO2();
	

    //##-1- Configure the Discovery I2C clock source. The clock is derived from the SYSCLK #
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
		
		
		// Configure the GPIOs ---------------------------------------------------
    // Enable GPIO clock 
				
    __HAL_RCC_GPIOB_CLK_ENABLE();    //for i2c1 SCL & sda line , 
      
    // Configure I2C Tx as alternate function  
    GPIO_InitStruct.Pin       = DISCOVERY_I2C1_SCL_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;  //GPIO_SPEED_FREQ_VERY_HIGH
    GPIO_InitStruct.Alternate = DISCOVERY_I2C1_SCL_SDA_AF;
    HAL_GPIO_Init(DISCOVERY_I2C1_SCL_GPIO_PORT , &GPIO_InitStruct);
      
    // Configure I2C Rx as alternate function  
    GPIO_InitStruct.Pin = DISCOVERY_I2C1_SDA_PIN;
    HAL_GPIO_Init(DISCOVERY_I2C1_SDA_GPIO_PORT , &GPIO_InitStruct);
    
    
		// Configure the Discovery I2Cx peripheral -------------------------------
    // Enable I2C1 clock 
    __HAL_RCC_I2C1_CLK_ENABLE(); 
    
    // Force the I2C Peripheral Clock Reset   
    __HAL_RCC_I2C1_FORCE_RESET();
      
    // Release the I2C Peripheral Clock Reset 
   __HAL_RCC_I2C1_RELEASE_RESET(); 
    
    // Enable and set Discovery I2Cx Interrupt to the highest priority 
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x01, 0);   //HIGHEST?? LOWER THAN SYSTICK??
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    
    // Enable and set Discovery I2Cx Interrupt to the highest priority 
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0x01, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);  

}



// defined in the stm32l476_discovery.c

void I2C_Init(I2C_HandleTypeDef * pI2c_Handle){
  //I2C_HandleTypeDef* pI2c_Handle = I2C_GetHandle(pI2c_Handle);
	
		
	if(HAL_I2C_GetState(pI2c_Handle) == HAL_I2C_STATE_RESET)
  {
		pI2c_Handle->Instance              = I2C1;
    //pI2c_Handle->Init.ClockSpeed       = 100000; //400000 also OK
    //pI2c_Handle->Init.DutyCycle        = I2C_DUTYCYCLE_2;  //16_9  also OK
    pI2c_Handle->Init.Timing						 = DISCOVERY_I2C1_TIMING; 
		pI2c_Handle->Init.OwnAddress1      = 0;
    pI2c_Handle->Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    pI2c_Handle->Init.DualAddressMode  = I2C_DUALADDRESS_DISABLED;
    pI2c_Handle->Init.OwnAddress2      = 0;
    pI2c_Handle->Init.GeneralCallMode  = I2C_GENERALCALL_DISABLED;
    pI2c_Handle->Init.NoStretchMode    = I2C_NOSTRETCH_DISABLED;  
    //
		
    I2C_MspInit();
    HAL_I2C_Init(pI2c_Handle);
		while(pI2c_Handle->State != HAL_I2C_STATE_READY) {}
  }
}



HAL_StatusTypeDef I2C_ByteWrite(I2C_HandleTypeDef * pI2c_Handle,uint8_t EEPROM_Addr, uint16_t Mem_Addr, uint8_t Data)
{
	//	HAL_StatusTypeDef status = HAL_OK;
  
		//status = HAL_I2C_Mem_Write(pI2c_Handle, EEPROM_Addr, Mem_Addr, I2C_MEMADD_SIZE_16BIT, &Data, 1, DISCOVERY_I2C1_TIMEOUT_MAX);  //I2Cx_TIMEOUT_MAX is defined in _discovery.h 	
		if (pI2c_Handle->State != HAL_I2C_STATE_READY) {
			return HAL_ERROR;
		}
		
		status = HAL_I2C_Mem_Write(pI2c_Handle, EEPROM_Addr, Mem_Addr, I2C_MEMADD_SIZE_16BIT, &Data, 1, DISCOVERY_I2C1_TIMEOUT_MAX);  //I2Cx_TIMEOUT_MAX is defined in _discovery.h
		/* Check the communication status */
		if(status != HAL_OK)
		{
				/* Re-Initialize the BUS */
				I2C_Error(pI2c_Handle);
				return status;
		} 

		while (HAL_I2C_GetState(pI2c_Handle)!=HAL_I2C_STATE_READY) {}

		HAL_Delay(10);  //essential for AT24C64, The T_WR is 10 ms for AT2C64.
	
		return status;		
}


//At24c64 may begin to write at the top of the page if page boundary is reached, and data may be "roll over" if more than 32 byts data are page_write
//user need to make use the datalen is appropriate to avoid the above situation happen. otherwise, data read back may have trouble.
HAL_StatusTypeDef I2C_PageWrite(I2C_HandleTypeDef * pI2c_Handle,uint8_t EEPROM_Addr, uint16_t Mem_Addr, uint8_t * PageData, uint8_t datalen)
{
		/*	uint8_t byte_num;	
		  if(datalen > PAGE_SIZE){ 
				datalen = PAGE_SIZE;
			}
		*/	
			status = HAL_I2C_Mem_Write(pI2c_Handle, EEPROM_Addr, Mem_Addr, I2C_MEMADD_SIZE_16BIT, PageData, datalen, DISCOVERY_I2C1_TIMEOUT_MAX);  //I2Cx_TIMEOUT_MAX is defined in _discovery.h
			while (HAL_I2C_GetState(pI2c_Handle)!=HAL_I2C_STATE_READY) {}

			HAL_Delay(10);  //essential for AT24C64, The T_WR is 10 ms for AT2C64.
			
/*
			for(byte_num=0; byte_num < datalen; byte_num++){
				status = HAL_I2C_Mem_Write(pI2c_Handle, EEPROM_Addr, Mem_Addr++, I2C_MEMADD_SIZE_16BIT, PageData+byte_num , 1, DISCOVERY_I2C1_TIMEOUT_MAX);  //I2Cx_TIMEOUT_MAX is defined in _discovery.h
  	
				// Check the communication status 
				if(status != HAL_OK)
					{
							// Re-Initialize the BUS 
							I2C_Error(pI2c_Handle);
							return status;
					} 

				while (HAL_I2C_GetState(pI2c_Handle)!=HAL_I2C_STATE_READY) {}

				HAL_Delay(10);  //essential for AT24C64, The T_WR is 10 ms for AT2C64.
			}	
	*/
		return status;		
}


HAL_StatusTypeDef I2C_BufferWrite(I2C_HandleTypeDef * pI2c_Handle, uint8_t EEPROM_Addr, uint16_t Mem_Addr, uint8_t * dataBuffer, uint16_t datalen)
{
  uint16_t numofpage = 0, numofsingle = 0, count = 0;
  uint16_t addr = 0;


  addr = Mem_Addr % PAGE_SIZE;
  count = PAGE_SIZE - addr;
  numofpage =  datalen / PAGE_SIZE;
  numofsingle = datalen % PAGE_SIZE;
 
  // If Mem_Addr is  aligned with EEPROM page  
  if(addr == 0) 
  {
    // If data length < EEPROM_PAGESIZE 
    if(numofpage == 0) 
    {
      //status = BSP_EEPROM_WritePage(pBuffer, WriteAddr, (uint8_t*)(&dataindex));
      status=I2C_PageWrite(pI2c_Handle,EEPROM_Addr, Mem_Addr, dataBuffer, (uint8_t)numofsingle);
			if (status != HAL_OK)
      {
        return status;
      }
    }
    // If datalen> EEPROM_PAGESIZE 
    else  
    {
      while(numofpage--)
      {
        status=I2C_PageWrite(pI2c_Handle,EEPROM_Addr, Mem_Addr, dataBuffer, (uint8_t)PAGE_SIZE);
				if (status != HAL_OK)
        {
          return status;
        }
        
        Mem_Addr +=  PAGE_SIZE;
        dataBuffer += PAGE_SIZE;
      }
      
      if(numofsingle!=0)
      {   
        status=I2C_PageWrite(pI2c_Handle,EEPROM_Addr, Mem_Addr, dataBuffer, (uint8_t)numofsingle);
				if (status != HAL_OK)
        {
          return status;
        }
      }
    }
  }
  // If Mem_Addr is not aligned with EEPROM_PAGESIZE
  else 
  {
    // If datalen < EEPROM_PAGESIZE
    if(numofpage== 0) 
    {
      //If the number of data to be written is more than the remaining space in the current page
      if (datalen > count)
      {
				status=I2C_PageWrite(pI2c_Handle,EEPROM_Addr, Mem_Addr, dataBuffer, (uint8_t)count);
        if (status != HAL_OK)
        {
          return status;
        }

				status=I2C_PageWrite(pI2c_Handle,EEPROM_Addr, Mem_Addr+count, (uint8_t*)(dataBuffer+count), (uint8_t)(datalen-count));
        if (status != HAL_OK)
        {
          return status;
        }
      }      
      else   //if number of data to be written is less than the remaining space in the current page   
      {   
				status=I2C_PageWrite(pI2c_Handle,EEPROM_Addr, Mem_Addr, dataBuffer, (uint8_t)datalen);
        if (status != HAL_OK)
        {
          return status;
        }
      }     
    }   
    else   // If datalen > EEPROM_PAGESIZE 
    {
      datalen -= count;
      numofpage =  datalen / PAGE_SIZE;
      numofsingle = datalen % PAGE_SIZE;
      
      if(count != 0)
      {  
       //write the first part-of-Page
				status=I2C_PageWrite(pI2c_Handle, EEPROM_Addr, Mem_Addr, dataBuffer, (uint8_t)count);
        if (status != HAL_OK)
        {
          return status;
        }
        Mem_Addr += count;
        dataBuffer += count;
      } 
      
      while(numofpage--)
      {
				status=I2C_PageWrite(pI2c_Handle,EEPROM_Addr, Mem_Addr, dataBuffer, PAGE_SIZE);
        if (status != HAL_OK)
        {
          return status;
        }
        Mem_Addr +=  PAGE_SIZE;
        dataBuffer += PAGE_SIZE;  
      }
      if(numofsingle != 0)
      {
				status=I2C_PageWrite(pI2c_Handle,EEPROM_Addr, Mem_Addr, dataBuffer, (uint8_t)numofsingle);
        if (status != HAL_OK)
        {
          return status;
        }
      }
    }
  }  
                                   
  /* If all operations OK, return EEPROM_OK (0) */
  return HAL_OK;
}


uint8_t I2C_ByteRead(I2C_HandleTypeDef * pI2c_Handle,uint8_t EEPROM_Addr, uint16_t Mem_Addr)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  
  status = HAL_I2C_Mem_Read(pI2c_Handle, EEPROM_Addr, Mem_Addr, I2C_MEMADD_SIZE_16BIT, &value, 1, DISCOVERY_I2C1_TIMEOUT_MAX);
 
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the BUS */
    I2C_Error(pI2c_Handle);
  
  }
  return value;
}


//defined in stm32l476g_discovery.h

void I2C_Error(I2C_HandleTypeDef * pI2c_Handle)
{
  // De-initialize the I2C communication BUS 
  HAL_I2C_DeInit(pI2c_Handle);
  
  // Re-Initialize the I2C communication BUS 
  I2C_Init(pI2c_Handle);
}




























