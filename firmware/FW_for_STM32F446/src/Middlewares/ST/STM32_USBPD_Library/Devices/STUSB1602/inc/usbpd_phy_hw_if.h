/**
  ******************************************************************************
  * @file    usbpd_phy_hw_if.h
  * @author  AMG Application Team
  * @brief   This file contains the headers of usbpd_hw_if.h for USB-PD Hardwer
             Interface layer. This file is specific for each device.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics International N.V.
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
/*#warning "usbpd_hw_if header bof"*/
#ifndef __STM32F072_USBPD_PHY_HW_IF_H_
#define __STM32F072_USBPD_PHY_HW_IF_H_

/*#warning "usbpd_hw_if header bof 2"*/
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "usbpd_porthandle.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_HW_IF
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_DEVICE_HW_IF_Exported_Types USBPD DEVICE HW_IF Exported Types
* @{
*/

/**
  * @brief Mode how to access functions containing a list of actions
  */
typedef enum
{
  ACKNOWLEDGE = 0,
  REQUEST = 1
} USBPD_HRPRS_Mode_TypeDef;

/**
  * @brief 
  */
typedef enum 
{
  EV_EX_NONE = 0,
  EV_EX_MONITOR = 1,
  EV_EX_FAULT = 2
} USBPD_HW_IF_Ex_Event_TypeDef;

/**
  * @brief 
  */
typedef struct
{
  /**
   * @brief  Notify an event occurred in the HW layer .
   * @param  PortNum:  The handle of the port
   * @param  Event:    The occured event
   * @param  Param:    a pointer to the parameter, the type is defined by the 
                       event, if none it is NULL
   * @retval None
   * @note Received data are stored inside PortNum->pRxBuffPtr
   */
  void (*USBPD_HW_IF_Ex_Notification)(uint8_t PortNum, USBPD_HW_IF_Ex_Event_TypeDef Event, void * Param);
} USBPD_HW_IF_Ex_Callbacks;

/**
  * @brief 
  */
typedef struct
{
  USBPD_HW_IF_Ex_Callbacks *cbs; /*!< @ref HW_IF_Ex callbacks         */
} HW_IF_Ex_HandleTypeDef;
/**
* @}
*/

/* Exported defines ----------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_HW_IF_Exported_Defines USBPD DEVICE HW_IF Exported Defines
* @{
*/

/* Exported constants --------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_HW_IF_Exported_Constants USBPD DEVICE HW_IF Exported Constants
* @{
*/
#define TX_BUFFER_LEN         ( 28 )          /*!< Size of Tx Buffer           */
#define TX_FREQ               ( 300000 )      /*!< Freq of USBPD transmission [Hz]  */
#define BMC_TX_FREQ           ( 2 * TX_FREQ ) /*!< Freq of BMC transmission   [Hz]  */
#define CAD_threshold_VBus    4750            /*!< Vbus Threshold           */
#define BIST_MAX_LENGTH       ( 60 )          /*!< 28 Bytes 60 times *    */
/**
* @}
*/

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_HW_IF_Exported_Macros USBPD DEVICE HW_IF Exported Macros
* @{
*/
#define USBPD_PORT_IsValid(__Port__)    ((__Port__) < (USBPD_PORT_COUNT))            /*!< Macro to identify valid Port Number*/

/* DRP */
#define DRP_SET_PROVIDER(X)             HAL_GPIO_WritePin(DRP_PORT, DRP_PIN, GPIO_PIN_SET)    /*!< Macro to set DRP as Provider     */
#define DRP_SET_CONSUMER(X)             HAL_GPIO_WritePin(DRP_PORT, DRP_PIN, GPIO_PIN_RESET)  /*!< Macro to set DRP as Consumer     */

/**
* @}
*/

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_HW_IF_Exported_Functions USBPD DEVICE HW_IF Exported Functions
* @{
*/
/* Called by interrupts */
void USBPD_DMA_PORTX_IRQHandler(uint8_t PortNum);
void USBPD_RX_PORT0_Interrupt_IRQHandler(void);
#if (USBPD_PORT_COUNT == 2)
void USBPD_RX_PORT1_Interrupt_IRQHandler(void);
#endif
void USBPD_RX_PORT0_COUNTTIM_IRQHandler(void);
#if (USBPD_PORT_COUNT == 2)
void USBPD_RX_PORT1_COUNTTIM_IRQHandler(void);
#endif
void USBPD_PE_PRL_TIMER_IRQHandler(void);

void PHY_HW_IF_RX_Start(uint8_t PortNum);
void PHY_HW_IF_RX_Stop(uint8_t PortNum);
void RX_ByteReceiverHandler(uint8_t PortNum);
USBPD_StatusTypeDef STUSB16xx_HW_IF_Send_Packet(uint8_t PortNum, uint8_t *pData, uint16_t Size);
void HW_IF_STUSB1602_Interrupt_CC_Detection(uint8_t PortNum, FunctionalState status);

void PHY_HW_IF_TX_Done(uint8_t PortNum);
void STUSB16xx_HW_IF_Alert_Check(uint8_t PortNum);
USBPD_StatusTypeDef STUSB16xx_HW_IF_Alert_Manager(uint8_t PortNum);
void STUSB16xx_HW_IF_TX_EN_Status(uint8_t PortNum, GPIO_PinState status);
void STUSB16xx_HW_IF_Set_DMA_Normal_Mode(uint8_t PortNum);
void STUSB16xx_HW_IF_Set_DMA_Circular_Mode(uint8_t PortNum);
USBPD_StatusTypeDef STUSB16xx_HW_IF_Set_VBus_Monitoring(uint8_t PortNum, uint16_t VBus, uint8_t Hset, uint8_t Lset);
void HW_IF_RESET_CTRL(uint8_t PortNum);
void HW_IF_RX_Enable(uint8_t PortNum);
void HW_IF_RX_Disable(uint8_t PortNum);
USBPD_StatusTypeDef STUSB16xx_HW_IF_HardReset(uint8_t PortNum, USBPD_HRPRS_Mode_TypeDef Mode);
USBPD_StatusTypeDef STUSB16xx_HW_IF_VConnSwap(uint8_t PortNum);
USBPD_StatusTypeDef STUSB16xx_HW_IF_DataRoleSwap(uint8_t PortNum);
void USBPDM1_AssertRp(uint8_t PortNum);
void USBPDM1_AssertRd(uint8_t PortNum);
void USBPDM1_DeAssertRp(uint8_t PortNum);
void USBPDM1_DeAssertRd(uint8_t PortNum);
FunctionalState HW_IF_PWR_IsEnabled(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, uint8_t *pBuffer, uint32_t Bitsize);
USBPD_StatusTypeDef USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum);
HAL_StatusTypeDef HW_IF_PWR_Enable(uint8_t PortNum, USBPD_FunctionalState state, CCxPin_TypeDef Cc, uint32_t VconnState, USBPD_PortPowerRole_TypeDef role);
USBPD_FunctionalState HW_IF_PWR_VBUSIsEnabled(uint8_t PortNum);
void USBPD_SINGLE_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t tim_it);
void USBPD_HW_IF_Reset(uint8_t PortNum, USBPD_HRPRS_Mode_TypeDef Mode);
void USBPD_HW_IF_EXTI_Callback(uint16_t GPIO_Pin);
uint8_t USBPD_16xx_IsResistor_SinkTxOk(uint8_t PortNum);
void PHY_HW_IF_TX_ABORT(uint8_t PortNum);
/**
* @}
*/

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif
/*#warning "usbpd_hw_if header eof 2"*/
#endif /* __STM32F072_USBPD_HW_IF_H_ */
/*#warning "usbpd_hw_if header eof"*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
