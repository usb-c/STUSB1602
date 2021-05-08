/**
  ******************************************************************************
  * @file    STUSB1602_Peripherals_if.h
  * @author  AMG Application Team
  * @brief   This file contains the headers of usbpd_porthandle.h.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/*#warning "Peripheral header sof"*/
#ifndef __USBPD_PERIPHERAL_H_
#define __USBPD_PERIPHERAL_H_
/*#warning "Peripheral header sof 2"*/
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "User_BSP.h"

   /**
  * @brief SPI Mode for communication
  */
typedef enum
{
  STUSB16xx_SPI_Mode_TX = 0,  /*!< SPI used for transmission */
  STUSB16xx_SPI_Mode_RX = 1,  /*!< SPI used for reception    */
} STUSB1602_SPI_Mode_TypeDef;

/**
  * @}
  */
  
/** @defgroup USBPD_DEVICE_PORTHANDLE_Exported_Defines USBPD DEVICE PORTHANDLE Exported Defines
  * @{
  */


#define I2C_ENTER_CRITICAL_SECTION()  uint32_t primask= __get_PRIMASK();\
                                        __disable_irq();

#define I2C_LEAVE_CRITICAL_SECTION() __set_PRIMASK(primask)
/** @addtogroup STUSB1602_LIBRARY
 * @{
 */

/* Exported defines ----------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_STUSB1602_LIBRARY_Exported_Defines USBPD DEVICE STUSB1602 LIBRARY Exported defines
* @{
*/
  
/** @defgroup USBPD_DEVICE_STUSB1602_LIBRARY_I2C_Defines USBPD DEVICE STUSB1602 LIBRARY I2C defines
* @{
*/


/**
  * @}
  */

/* Prototypes*/
                                        
void USBPD_HW_IF_GlobalHwInit(void);

                                        
void STUSB1602_Driver_Init(uint8_t PortNum);
STUSB1602_StatusTypeDef STUSB1602_ReadReg(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);
uint8_t STUSB1602_ReadRegSingle(uint8_t Addr, uint8_t Reg);
STUSB1602_StatusTypeDef STUSB1602_WriteReg(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);
STUSB1602_StatusTypeDef STUSB1602_WriteRegSingle(const uint8_t Value, uint8_t Addr, uint8_t Reg);
   

    
void HW_IF_STUSB1602_IO_Init(uint8_t PortNum);

void HW_IF_STUSB16xx_I2C_Init(uint8_t PortNum);

void HW_IF_SPI_Init(uint8_t PortNum);
void HW_IF_Switch_Mode(uint8_t PortNum, STUSB1602_SPI_Mode_TypeDef mode);
void HW_IF_DMA_Init(uint8_t PortNum);
void HW_IF_NSS_RisingFalling_Interrupt (uint8_t PortNum ,FunctionalState status);
void STUSB16xx_HW_IF_TX_DMA_Init(uint8_t PortNum);
void STUSB16xx_HW_IF_RX_DMA_Init(uint8_t PortNum);
void STUSB16xx_HW_IF_Set_DMA_Normal_Mode(uint8_t PortNum);
void STUSB16xx_HW_IF_Set_DMA_Circular_Mode(uint8_t PortNum);

#endif
