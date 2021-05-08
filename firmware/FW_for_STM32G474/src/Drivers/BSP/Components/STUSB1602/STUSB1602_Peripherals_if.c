/**
**************************************************************************************************
* @file    STUSB1602_Peripherals_if.c
* @author  AMG -  Application Team
* @brief   This file provides a set of functions needed to manage the interface STUSB1602 and peripherals 
I2C for Registers , SPI DMA CRC Timers for Phy .
**************************************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics International N.V.
* All rights reserved.</center></h2>
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted, provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 3. Neither the name of STMicroelectronics nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************************************
*/


#define __USBPD_PERIPHERAL_C_
/* Includes ------------------------------------------------------------------*/
#include "User_BSP.h"
#include "STUSB1602_Peripherals_if.h"
#undef __USBPD_PERIPHERAL_C_



void HW_IF_PWR_DigitalGPIO_Init(void);    

extern STUSB16xx_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT];
extern uint8_t nvm_flash(uint8_t Addr);
extern uint8_t nvm_flash_recup(uint8_t Addr);
/**
* @brief  Global hardware initialization
* @retval None
*/
void USBPD_HW_IF_GlobalHwInit(void)
{
  uint8_t PortNum ;  
  uint8_t nvm_read = 0;                   /*!< Variable used to check if NVM has been loaded correctly */  
  
  /* Configure the ADCx peripheral */
#if defined (_ADC_MONITORING)
  HW_IF_ADC_Init();
#endif

  HW_IF_PWR_DigitalGPIO_Init();
  
  /* used for the PRL/PE timing */
  USBPD_TIM_Init();
  /* Init of all IOs for the specified AFE port*/
  for (PortNum =0 ; PortNum < USBPD_PORT_COUNT ;PortNum++)
  {
#if defined(_BUCKCV) || defined( BOARD_100W)
    HW_DAC_init(PortNum);
#endif  /* defined(_BUCKCV) || defined( BOARD_100W)*/
    HW_IF_STUSB1602_IO_Init(PortNum);
        
    /* Init peripherals required by the specified port*/
    STUSB1602_Driver_Init(PortNum);
    HW_IF_STUSB16xx_I2C_Init(PortNum);
#ifdef _VVAR_FLASH  
    /* if sommething was written in NVM , reset must be performe to take it into account */
    /*   to update NVM content */
    if ( nvm_flash(STUSB1602_I2C_Add(PortNum)) == 4)   
      /* Or initial content of Nvm*/
      /*     if ( nvm_flash_recup(STUSB1602_I2C_Add(PortNum)) == 4) */
    {
      HW_IF_STUSB16xx_Reset(PortNum);
    }
#endif
    
    
    /* check init phase is completed on STUSB1602*/
    nvm_read = STUSB1602_NVM_OK_Get(STUSB1602_I2C_Add(PortNum));
    while (nvm_read != 2)
    {
      /*NVM not ready*/
      nvm_read = STUSB1602_NVM_OK_Get(STUSB1602_I2C_Add(PortNum));
    }
    /* Add check of chip ID*/
    Ports[PortNum].Device_cut = STUSB1602_DEVICE_CUT_Get(STUSB1602_I2C_Add(PortNum));
    
    STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum),VBUS_Discharge_Path_Disable); 
    STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), No_SW_RST); /* to avoid STUSB under SW reset after a reboot */

    HAL_GPIO_WritePin(TX_EN_GPIO_PORT(PortNum),TX_EN_GPIO_PIN(PortNum),GPIO_PIN_RESET);
    /* SPI and DMA init */
    HW_IF_DMA_Init(PortNum);
    HW_IF_SPI_Init(PortNum);
    /* clear Alert status bits */
  } 
}

/**
* @brief  Inititialization of the Power Pins.
* @retval HAL Status
*/
void HW_IF_PWR_DigitalGPIO_Init()
{
#ifdef MB1303   
  GPIO_InitTypeDef  GPIO_InitStruct;
  uint8_t index = 0;
  for(index=0;index<USBPD_POWSELn;index++)
  {
    USBPD_BSP_GPIOPins_TypeDef gpio = USBPD_POWSELs[index];

#ifdef _GPIO_FOR_SRC  
   /* Configure the powsels pin */
    GPIO_InitStruct.Pin = gpio.GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(gpio.GPIOx, &GPIO_InitStruct);
    
    /* Turn the pin off */
    USBPD_HW_IF_GPIO_On(gpio);
#else  
    /* Configure the powsels pin */
    GPIO_InitStruct.Pin = gpio.GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    HAL_GPIO_Init(gpio.GPIOx, &GPIO_InitStruct);
    
    /* Turn the pin off */
    USBPD_HW_IF_GPIO_Off(gpio);
#endif
  }
#endif
}
/**
* @brief  Configuration of STUSB1602 GPIO pins
* @param  PortNum The port index
* @retval None
*/ 
void HW_IF_STUSB1602_IO_Init(uint8_t PortNum)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Configure GPIO pin : ALERT */
  GPIO_InitStruct.Pin = ALERT_GPIO_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ALERT_GPIO_PORT(PortNum), &GPIO_InitStruct);
  
#ifdef MB1303
  /* Configure GPIO pin : A_B_SIDE */
  GPIO_InitStruct.Pin = A_B_Side_GPIO_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(A_B_Side_GPIO_PORT(PortNum), &GPIO_InitStruct);  
#endif  
  /* Configure GPIO pin : TX_EN */
  HAL_GPIO_WritePin(TX_EN_GPIO_PORT(PortNum),TX_EN_GPIO_PIN(PortNum),GPIO_PIN_RESET); 
  GPIO_InitStruct.Pin = TX_EN_GPIO_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TX_EN_GPIO_PORT(PortNum), &GPIO_InitStruct);  
  
  /* Configure GPIO pins : RESET */
  HAL_GPIO_WritePin(RESET_GPIO_PORT(PortNum),RESET_GPIO_PIN(PortNum),GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = RESET_GPIO_PIN(PortNum);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_GPIO_PORT(PortNum), &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ALERT_GPIO_IRQHANDLER(PortNum), ALERT_GPIO_IRQPRIORITY(PortNum), 1);
  HAL_NVIC_EnableIRQ(ALERT_GPIO_IRQHANDLER(PortNum));
}


/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef STUSB16xx_I2CxHandle;         /*!< I2C Handle for port 0 */

/* Functions -----------------------------------------------------------------*/
/**
* @brief  I2C init function
* @param  PortNum The port index
* @retval None
*/ 
void HW_IF_STUSB16xx_I2C_Init(uint8_t PortNum)
{
  
  Ports[PortNum].hi2c->Instance =  I2C_INSTANCE(PortNum);
#if defined (I2C_CLOCKSPEED)
Ports[PortNum].hi2c->Init.ClockSpeed = I2C_CLOCKSPEED(PortNum);                                        
Ports[PortNum].hi2c->Init.DutyCycle       = I2C_DUTYCYCLE_2;
#else  /*FO72 / F051*/     
Ports[PortNum].hi2c->Init.Timing =  I2C_TIMING(PortNum);
#endif  /*defined (I2C_CLOCKSPEED)*/
  
  
  Ports[PortNum].hi2c->Init.OwnAddress1 = 0;
  Ports[PortNum].hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  Ports[PortNum].hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  Ports[PortNum].hi2c->Init.OwnAddress2 = 0;
#if defined (I2C_OA2_NOMASK)                                            
  Ports[PortNum].hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
#endif  /*defined (I2C_OA2_NOMASK)*/  
  
  Ports[PortNum].hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  Ports[PortNum].hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  
  HAL_I2C_Init(Ports[PortNum].hi2c);
#ifdef HAL_I2CEx_ConfigAnalogFilter
  HAL_I2CEx_ConfigAnalogFilter(Ports[PortNum].hi2c, I2C_ANALOGFILTER_ENABLE);
#endif
#ifdef HAL_I2CEx_ConfigDigitalFilter  
  HAL_I2CEx_ConfigDigitalFilter(Ports[PortNum].hi2c, 0)
#endif    
}
/* Private function prototypes -----------------------------------------------*/
/* COMM functions*/
#if defined (_RTOS)
osMutexDef(mut_i2c_1602);
osMutexId mut_i2c_1602_id;
#else   
static STUSB1602_StatusTypeDef HW_IF_COMM_WAIT(uint32_t * I2C_Lock, uint32_t Timeout) ;
static STUSB1602_StatusTypeDef HW_IF_COMM_RELEASE(uint32_t * I2C_Lock);
uint32_t I2C_LockCount ;
uint32_t duration;   
extern uint32_t HAL_GetTick(void);
#endif

uint8_t semaphore_Timeout_event_nb ;
/* Private functions ---------------------------------------------------------*/

/* Imported function prototypes ----------------------------------------------*/
/* Functions -----------------------------------------------------------------*/


/** @defgroup USBPD_DEVICE_STUSB1602_LIBRARY_Exported_Functions USBPD DEVICE STUSB1602 LIBRARY Exported functions
* @{
*/

/**
* @brief  I2C handle initialization
* @param  PortNum     Port number value
* @param  I2CxHandle  External I2C handle
* @retval None
*/
void STUSB1602_Driver_Init(uint8_t PortNum)
{
  
  Ports[PortNum].hi2c = &STUSB16xx_I2CxHandle  ;
#if defined (_RTOS)
#if (USBPD_PORT_COUNT == 2)
  if (PortNum == 1)
  {
  mut_i2c_1602_id = osMutexCreate(osMutex(mut_i2c_1602));
  __enable_irq();
  }
#else
  mut_i2c_1602_id =  osMutexCreate(osMutex(mut_i2c_1602)); 
#endif
#else
  I2C_LockCount = 0 ;
#endif
  semaphore_Timeout_event_nb = 0;
}

#ifndef _RTOS

/**
* @brief   It waits for the communication sequence with the device is completed
* @param   PortNum The port index
* @param   Timeout The timeout time
* @retval  USBPD status 
*/
static STUSB1602_StatusTypeDef HW_IF_COMM_WAIT(uint32_t * I2C_Lock, uint32_t Timeout)
{
  uint32_t timeout ;
  
  timeout = HAL_GetTick();
  while (1)
  {  
    I2C_ENTER_CRITICAL_SECTION() ;
    if (*I2C_Lock == 0)
    {
      /* the resource is free */
      
      *I2C_Lock = 1;
      I2C_LEAVE_CRITICAL_SECTION();
      return STUSB1602_OK;
    }
    I2C_LEAVE_CRITICAL_SECTION();
    duration =  (HAL_GetTick() - timeout)  ;
    if (duration > Timeout)
    {
      semaphore_Timeout_event_nb++;
      return STUSB1602_ERROR;
    }
    
  }
}


/**
* @brief   It releases the communication resources
* @param   PortNum The port index
* @retval  USBPD status 
*/
static STUSB1602_StatusTypeDef HW_IF_COMM_RELEASE(uint32_t * I2C_Lock)
{
  I2C_ENTER_CRITICAL_SECTION() ;
  if (*I2C_Lock == 0)
  {  
    I2C_LEAVE_CRITICAL_SECTION();
    /* no change, the resource is already free */
    return STUSB1602_ERROR;
  }
  
  /* release the resource */
  *I2C_Lock = 0;
  I2C_LEAVE_CRITICAL_SECTION();
  return STUSB1602_OK;
  
}
#endif



/**
* @brief STUSB1602 registers reading function
* @param pBuffer      Pointer to data buffer
* @param Addr         I2C address of port controller device
* @param Reg          Address of first register to be read
* @param Size         Amount of bytes to be read
* @retval STUSB1602_StatusTypeDef Allowed values are STUSB1602_OK, STUSB1602_ERROR, STUSB1602_BUSY, STUSB1602_TIMEOUT
*/
STUSB1602_StatusTypeDef STUSB1602_ReadReg(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size)
{
  STUSB1602_StatusTypeDef status = STUSB1602_OK;
#if defined (_RTOS)
  /* try to acquire the communication resource to avoid the conflict */
  /* check to avoid count before OSKernel Start */
  if (uxTaskGetNumberOfTasks() > 2 ) 
  {
    if (osMutexWait(mut_i2c_1602_id, osWaitForever) != osErrorOS)  /*osWait 10*/ 
    {
      status = (STUSB1602_StatusTypeDef) HAL_I2C_Mem_Read(&STUSB16xx_I2CxHandle, (Addr<<1), (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size, TIMEOUT_MAX);
      osMutexRelease(mut_i2c_1602_id);
    }
    else 
    {
      semaphore_Timeout_event_nb ++;
      status = STUSB1602_TIMEOUT ;
    }
  }
  else  status = (STUSB1602_StatusTypeDef) HAL_I2C_Mem_Read(&STUSB16xx_I2CxHandle, (Addr<<1), (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size, TIMEOUT_MAX);
  return  status;
#else
  status = HW_IF_COMM_WAIT(&I2C_LockCount, 10);
  if (status == STUSB1602_OK)
  {
    status = (STUSB1602_StatusTypeDef) HAL_I2C_Mem_Read(&STUSB16xx_I2CxHandle, (Addr<<1), (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size, TIMEOUT_MAX);
    HW_IF_COMM_RELEASE(&I2C_LockCount);
    
  }  
  else 
  {
    semaphore_Timeout_event_nb ++;
    status = STUSB1602_TIMEOUT ;
  } 
  return  status;
#endif
}



/**
* @brief STUSB1602 single register reading function
* @param Addr         I2C address of port controller device
* @param Reg          Address of register to be read
* @retval uint8_t     Register value
*/
uint8_t STUSB1602_ReadRegSingle(uint8_t Addr, uint8_t Reg)
{
  uint8_t value = 0x00;
  STUSB1602_ReadReg(&value, Addr, Reg, 1);
  return value;
}


/**
* @brief STUSB1602 registers writing function
* @param pBuffer      Pointer to data buffer
* @param Addr         I2C address of port controller device
* @param Reg          Address of first register to be write
* @param Size         Amount of bytes to be write
* @retval STUSB1602_StatusTypeDef Allowed values are STUSB1602_OK, STUSB1602_ERROR, STUSB1602_BUSY, STUSB1602_TIMEOUT
*/
STUSB1602_StatusTypeDef STUSB1602_WriteReg(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size)
{
  STUSB1602_StatusTypeDef status = STUSB1602_OK;
#if defined (_RTOS)
  /* try to acquire the communication resource to avoid the conflict */
  /* check to avoid count before OSKernel Start */
  if (uxTaskGetNumberOfTasks() > 2 )
  {
    if (osMutexWait(mut_i2c_1602_id, osWaitForever) != osErrorOS)  /*osWait 10*/ 
    {
      status = (STUSB1602_StatusTypeDef)HAL_I2C_Mem_Write(&STUSB16xx_I2CxHandle, (Addr<<1), (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,TIMEOUT_MAX);
      osMutexRelease(mut_i2c_1602_id);
    }
    else 
    {
      semaphore_Timeout_event_nb ++;
      status = STUSB1602_TIMEOUT ;
    }
  }
  else  status = (STUSB1602_StatusTypeDef)HAL_I2C_Mem_Write(&STUSB16xx_I2CxHandle, (Addr<<1), (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,TIMEOUT_MAX); 
  return  status;
#else
  status = HW_IF_COMM_WAIT(&I2C_LockCount, 10);
  if (status == STUSB1602_OK)
  {
    status = (STUSB1602_StatusTypeDef)HAL_I2C_Mem_Write(&STUSB16xx_I2CxHandle, (Addr<<1), (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,TIMEOUT_MAX); 
    HW_IF_COMM_RELEASE(&I2C_LockCount);
    
  }  
  else 
  {
    semaphore_Timeout_event_nb ++;
    status = STUSB1602_TIMEOUT ;
  }  
  return status ;     
#endif
  
}


/**
* @brief STUSB1602 single register writing function
* @param Value        Value to write
* @param Addr         I2C address of port controller device
* @param Reg          Address of register to be write
* @retval STUSB1602_StatusTypeDef Allowed values are STUSB1602_OK, STUSB1602_ERROR, STUSB1602_BUSY, STUSB1602_TIMEOUT
*/
STUSB1602_StatusTypeDef STUSB1602_WriteRegSingle(const uint8_t Value, uint8_t Addr, uint8_t Reg)
{
  
  STUSB1602_StatusTypeDef status = STUSB1602_OK;
#if defined (_RTOS)
  /* try to acquire the communication resource to avoid the conflict */
  /* check to avoid count before OSKernel Start */
 if (uxTaskGetNumberOfTasks() > 2 ) 
  {
    if (osMutexWait(mut_i2c_1602_id, osWaitForever) != osErrorOS)  /*osWait30*/ 
    {
      uint8_t value = Value;
      status = (STUSB1602_StatusTypeDef)HAL_I2C_Mem_Write(&STUSB16xx_I2CxHandle, (Addr<<1), (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, TIMEOUT_MAX);
      osMutexRelease(mut_i2c_1602_id);
      return  status;
    }
    else 
    {
      semaphore_Timeout_event_nb ++;
      status = STUSB1602_TIMEOUT ;
    }        
  }
  else  
  {
    uint8_t value = Value;
    status = (STUSB1602_StatusTypeDef)HAL_I2C_Mem_Write(&STUSB16xx_I2CxHandle, (Addr<<1), (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, TIMEOUT_MAX);
    return  status;
  }
#else
  status = HW_IF_COMM_WAIT(&I2C_LockCount, 10);
  if (status == STUSB1602_OK)
  {
    uint8_t value = Value;
    status = (STUSB1602_StatusTypeDef)HAL_I2C_Mem_Write(&STUSB16xx_I2CxHandle, (Addr<<1), (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, TIMEOUT_MAX);
    HW_IF_COMM_RELEASE(&I2C_LockCount);
    return  status;
  }  
  else 
  {
    semaphore_Timeout_event_nb ++;
    status = STUSB1602_TIMEOUT ;
  }       
#endif
  
  
  return status;
}


/* PHY Peripherals init functions */

/**
* @brief  SPI init function
* @param  PortNum The port index
* @retval None
*/ 
void HW_IF_SPI_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  SPI_HandleTypeDef*           phspi = &(Ports[PortNum].hspi);
  
  phspi->Instance =           SPI_Instance(PortNum);
  phspi->Init.Mode =           SPI_MODE_SLAVE;
#if !defined (SPI_ONE_LINE)
  phspi->Init.Direction =  SPI_DIRECTION_2LINES;
#else
    phspi->Init.Direction =  SPI_DIRECTION_1LINE;
#endif
  phspi->Init.DataSize =   SPI_DATASIZE_8BIT;
  phspi->Init.CLKPolarity =     SPI_POLARITY_HIGH;
  phspi->Init.CLKPhase =   SPI_PHASE_1EDGE;
  phspi->Init.NSS =     SPI_NSS_HARD_INPUT;
  phspi->Init.FirstBit =   SPI_FIRSTBIT_LSB;
  phspi->Init.TIMode =     SPI_TIMODE_DISABLE;
  phspi->Init.CRCCalculation =  SPI_CRCCALCULATION_DISABLE;
  phspi->Init.CRCPolynomial =   8;
  phspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
#if defined(SPI_CRC_LENGTH_DATASIZE)
  phspi->Init.CRCLength =   SPI_CRC_LENGTH_DATASIZE; 
#endif
#if defined(SPI_NSS_PULSE_DISABLE)
  phspi->Init.NSSPMode =   SPI_NSS_PULSE_DISABLE;
#endif  
  
  HAL_SPI_Init(phspi);
}


/**
* @brief  It changes SPI configuartion according to trasmission or reception mode requirements
* @param  PortNum The port index
* @param  mode Two allowed values:  STUSB16xx_SPI_Mode_TX or STUSB16xx_SPI_Mode_RX
* @retval None
*/ 
void HW_IF_Switch_Mode(uint8_t PortNum, STUSB1602_SPI_Mode_TypeDef mode)
{
  
  /* Set the data sampling edge according to mode */
  /*Here we made hypothesis that DMA and SPI are stopped then we can change mode */
  SPI_HandleTypeDef*   phspi = &(Ports[PortNum].hspi);
  
  /* SPI NSS software or hardware according to the mode value */
  /* when TX  NSS is Soft SSM= 1 RXONLY = 0 */
  /* when RX NSS is Hard SSM = 0 RXONLY = 1 */
  phspi->Instance->CR1 &= ~(1<<SPI_CR1_SSM_Pos);
  phspi->Instance->CR1 |= ((((~mode) & 1)<<SPI_CR1_SSM_Pos) & SPI_CR1_SSM) ;
#if !defined(SPI_ONE_LINE)
  phspi->Instance->CR1 &= ~(1<<SPI_CR1_RXONLY_Pos);
  phspi->Instance->CR1 |= ((((mode) & 1)<<SPI_CR1_RXONLY_Pos) & SPI_CR1_RXONLY) ;
#endif
#if defined(SPI_ONE_LINE)
    GPIO_InitTypeDef GPIO_InitStruct;

  if (~mode)
  {
     GPIO_InitStruct.Pin = SPI_MISO_PIN(PortNum);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = SPI_MISO_ALTERNATE(PortNum);
    HAL_GPIO_Init(SPI_MISO_PORT(PortNum), &GPIO_InitStruct);
  LL_SPI_SetTransferDirection(phspi->Instance,LL_SPI_HALF_DUPLEX_TX);
  }
  else
  {
    
    GPIO_InitStruct.Pin = SPI_MISO_PIN(PortNum);
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = SPI_MISO_ALTERNATE(PortNum);
    HAL_GPIO_Init(SPI_MISO_PORT(PortNum), &GPIO_InitStruct);
  LL_SPI_SetTransferDirection(phspi->Instance,LL_SPI_HALF_DUPLEX_RX);
  }

#endif
  /* Enable/Disable RX NSS EXT Interrupt */
  HW_IF_NSS_RisingFalling_Interrupt (PortNum, mode == STUSB16xx_SPI_Mode_RX ? ENABLE : DISABLE);
      
      
}

/**
* @brief  DMA init function
* @param  PortNum The port index
* @retval None
*/ 
void HW_IF_DMA_Init(uint8_t PortNum)
{
  /* DMA controller clock enable */
    DMA_CLK_ENABLE(PortNum);
  
  
  /* NVIC configuration for DMA */
  
#if defined(DMAMUX_CxCR_DMAREQ_ID) 
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
#endif     

  HAL_NVIC_SetPriority(TXDMACHIRQ(PortNum), TXDMACHIRQ_PRIO(PortNum), TXDMACHIRQ_SUB_PRIO(PortNum));
  
//#ifdef RX_DMACH
//  HAL_NVIC_SetPriority(RXDMACHIRQ(PortNum), RXDMACHIRQ_PRIO(PortNum), RXDMACHIRQ_SUB_PRIO(PortNum));
//#endif
}


/**
* @brief  It enables or disables the rising and falling interrupt on NSS line
* @param  PortNum The port index
* @param  status Two allowed values: ENABLE or DISABLE
* @retval None
*/ 
void HW_IF_NSS_RisingFalling_Interrupt (uint8_t PortNum ,FunctionalState status)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  
  if (status == ENABLE)
  {
    /* NVIC configuration*/
    NVIC_EnableIRQ(SPI_NSS_LL_IRQHANDLER(PortNum));
    NVIC_SetPriority(SPI_NSS_LL_IRQHANDLER(PortNum),SPI_NSS_LL_IRQPRIORITY(PortNum));
    
    /* External Line initialization */

    SPI_NSS_LL_APB_EN_CLK(PortNum);

    LL_SYSCFG_SetEXTISource(SPI_NSS_LL_PORT(PortNum), SPI_NSS_LL_SYS_EXTI(PortNum));
    
    EXTI_InitStruct.Line_0_31 = SPI_NSS_LL_EXTI(PortNum);
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);
  }
  else
  {
    /* External Line deinitialization */
    SPI_NSS_LL_APB_EN_CLK(PortNum);
    LL_SYSCFG_SetEXTISource(SPI_NSS_LL_PORT(PortNum), SPI_NSS_LL_SYS_EXTI(PortNum));
  
    EXTI_InitStruct.Line_0_31 = SPI_NSS_LL_EXTI(PortNum);
    EXTI_InitStruct.LineCommand = DISABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_NONE;
    LL_EXTI_Init(&EXTI_InitStruct);
  }  
}

/* Private functions ---------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_STUSB16XX_HW_IF_Private_Functions USBPD DEVICE STUSB16XX HW IF Private functions
* @details Private functions can be used at hardware interface level
* @{
*/

/**
* @brief  Initialization of DMA for transmission
* @param  PortNum The port index
* @retval None
*/ 
void STUSB16xx_HW_IF_TX_DMA_Init(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_tx_spi = &(Ports[PortNum].hdmatx);
  
  /* Set the DMA handler of the peripheral handler */
  Ports[PortNum].hspi.hdmatx = hdma_tx_spi;
  
  /* Peripheral DMA init*/
#if defined(TX_DMASTREAM)
hdma_tx_spi->Instance =                   TX_DMASTREAM(PortNum);
#if defined(TX_DMACH)
hdma_tx_spi->Init.Channel =               TX_DMACH(PortNum);
#endif
#else   
  hdma_tx_spi->Instance =                   TX_DMACH(PortNum);
#endif
#if defined(DMA_REQUEST_SPI_TX  )
  hdma_tx_spi->Init.Request = DMA_REQUEST_SPI_TX(PortNum);
#endif
  hdma_tx_spi->Init.Direction =             DMA_MEMORY_TO_PERIPH;
  hdma_tx_spi->Init.PeriphInc =             DMA_PINC_DISABLE;
  hdma_tx_spi->Init.MemInc =                DMA_MINC_ENABLE;
  hdma_tx_spi->Init.PeriphDataAlignment =   DMA_PDATAALIGN_BYTE;
  hdma_tx_spi->Init.MemDataAlignment =      DMA_MDATAALIGN_BYTE;
#ifdef DMA_PFCTRL
  hdma_tx_spi->Init.Mode =  DMA_NORMAL;               /*  DMA_PFCTRL; */
  hdma_tx_spi->Init.FIFOMode = DMA_FIFOMODE_DISABLE;    /* FIFO mode enabled       DMA_FIFOMODE_ENABLE  */
  hdma_tx_spi->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL; /* FIFO threshold: 1/4 full   */
  hdma_tx_spi->Init.MemBurst = DMA_MBURST_SINGLE;              /* Memory burst                     */
  hdma_tx_spi->Init.PeriphBurst = DMA_PBURST_SINGLE;           /* Peripheral burst                 */
#else  
  hdma_tx_spi->Init.Mode =                  DMA_NORMAL;
#endif  
  hdma_tx_spi->Init.Priority =              DMA_PRIORITY_LOW;
  HAL_DMA_Init(hdma_tx_spi);
  

#if defined (DMA1_Channel7)  
  if ( TX_DMACH(PortNum) == DMA1_Channel7)
#if defined(__HAL_DMA_REMAP_CHANNEL_ENABLE)
    __HAL_DMA_REMAP_CHANNEL_ENABLE(DMA_REMAP_SPI2_DMA_CH67);
#elif defined (HAL_REMAPDMA_SPI1_RX_DMA1_CH7)
	__HAL_DMA_REMAP_CHANNEL_ENABLE(HAL_REMAPDMA_SPI1_RX_DMA1_CH7);
    
#endif
#endif

  __HAL_LINKDMA((&Ports[PortNum].hspi),hdmatx,(*hdma_tx_spi));
  
  /* Enable IRQ DMA */
  HAL_NVIC_EnableIRQ(TXDMACHIRQ(PortNum));
#if defined (SPI_SR_FTLVL)  
  __HAL_DMA_DISABLE_IT(hdma_tx_spi, DMA_IT_HT);
#endif 
}


/**
* @brief  Initialization DMA for reception
* @param  PortNum The port index
* @retval None
*/ 
void STUSB16xx_HW_IF_RX_DMA_Init(uint8_t PortNum)
{
  #ifdef RX_DMACH
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_rx_spi = &(Ports[PortNum].hdmarx);
  
  /* Peripheral DMA init*/
#if defined(RX_DMASTREAM)
  hdma_rx_spi->Instance =                   RX_DMASTREAM(PortNum);
#if defined(RX_DMACH)  
  hdma_rx_spi->Init.Channel =               RX_DMACH(PortNum);
#endif
#else 
  hdma_rx_spi->Instance =                   RX_DMACH(PortNum);
#endif
#if defined(DMA_REQUEST_SPI_RX)
      hdma_rx_spi->Init.Request = DMA_REQUEST_SPI_RX(PortNum);
#endif
  hdma_rx_spi->Init.Direction =             DMA_PERIPH_TO_MEMORY;
  hdma_rx_spi->Init.PeriphInc =             DMA_PINC_DISABLE;
  hdma_rx_spi->Init.MemInc =                DMA_MINC_ENABLE;
  hdma_rx_spi->Init.PeriphDataAlignment =   DMA_PDATAALIGN_BYTE;
  hdma_rx_spi->Init.MemDataAlignment =      DMA_MDATAALIGN_BYTE;
#ifdef DMA_PFCTRL
  hdma_rx_spi->Init.Mode =  DMA_NORMAL;               /*  DMA_PFCTRL; */
  hdma_rx_spi->Init.FIFOMode = DMA_FIFOMODE_DISABLE;            /* FIFO mode enabled       DMA_FIFOMODE_ENABLE         */
  hdma_rx_spi->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL; /* FIFO threshold: 1/4 full   */
  hdma_rx_spi->Init.MemBurst = DMA_MBURST_SINGLE;              /* Memory burst                     */
  hdma_rx_spi->Init.PeriphBurst = DMA_PBURST_SINGLE;           /* Peripheral burst                 */
#else  
  hdma_rx_spi->Init.Mode =                  DMA_NORMAL;
#endif   
  hdma_rx_spi->Init.Priority =              DMA_PRIORITY_HIGH;
  HAL_DMA_Init(hdma_rx_spi);

  /* Set the DMA handler of the peripheral handler */

#if defined (DMA1_Channel6) 
  if ( RX_DMACH(PortNum) == DMA1_Channel6)
#if defined(__HAL_DMA_REMAP_CHANNEL_ENABLE)
    __HAL_DMA_REMAP_CHANNEL_ENABLE(DMA_REMAP_SPI2_DMA_CH67);
#elif defined (HAL_REMAPDMA_SPI1_RX_DMA1_CH6)
	__HAL_DMA_REMAP_CHANNEL_ENABLE(HAL_REMAPDMA_SPI1_RX_DMA1_CH6);
#endif 
#endif
  __HAL_LINKDMA((&Ports[PortNum].hspi),hdmarx,(*hdma_rx_spi));
 
  /* NVIC configuration for DMA & SPI */
  HAL_NVIC_SetPriority(SPI_IRQn(PortNum), SPIx_IRQ_PRIO(PortNum), 0);
  
#endif 
   __HAL_SPI_DISABLE_IT(&Ports[PortNum].hspi, (SPI_IT_RXNE | SPI_IT_ERR | SPI_IT_TXE ));
#ifdef RX_DMACH   
   /* disabling DMA IT at this point is mandatory to avoid propagating IT from DMA)*/
   __HAL_DMA_DISABLE_IT(hdma_rx_spi,  DMA_IT_HT );
   __HAL_DMA_DISABLE_IT(hdma_rx_spi,  DMA_IT_TC );
   __HAL_DMA_DISABLE_IT(hdma_rx_spi,  DMA_IT_TE );
#endif
}


/**
* @brief  It switches SPI DMA in normal mode
* @param  PortNum The port index
* @retval None
*/
void STUSB16xx_HW_IF_Set_DMA_Normal_Mode(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_tx_spi = &(Ports[PortNum].hdmatx);
  
  hdma_tx_spi->Init.Mode =                  DMA_NORMAL;
  HAL_DMA_Init(hdma_tx_spi);
  
  __HAL_LINKDMA((&Ports[PortNum].hspi),hdmatx,(*hdma_tx_spi));
}


/**
* @brief  It switches SPI DMA in circular mode
* @param  PortNum The port index
* @retval None
*/
void STUSB16xx_HW_IF_Set_DMA_Circular_Mode(uint8_t PortNum)
{
  /* Get the peripheral handler variable */
  DMA_HandleTypeDef* hdma_tx_spi = &(Ports[PortNum].hdmatx);
  
  hdma_tx_spi->Init.Mode =                  DMA_CIRCULAR;
  HAL_DMA_Init(hdma_tx_spi);
  
  __HAL_LINKDMA((&Ports[PortNum].hspi),hdmatx,(*hdma_tx_spi));
  
  /* DMA interrupt init */

  HAL_NVIC_SetPriority(TXDMACHIRQ(PortNum), TXDMACHIRQ_PRIO(PortNum),TXDMACHIRQ_SUB_PRIO(PortNum));
}

/** @}*/ 


/* Private functions ---------------------------------------------------------*/


/** @addtogroup USBPD_DEVICE_PHY_HW_IF_Private_Functions USBPD DEVICE PHY HW IF Private functions
* @details Private functions can be used at hardware interface level
* @{
*/






