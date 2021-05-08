/**
  ******************************************************************************
  * @file    tracer_emb_conf.h
  * @author  MCD Application Team
  * @brief   This file contains the Trace HW related defines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#ifndef TRACER_EMB_CONF_H
#define TRACER_EMB_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "User_BSP.h"
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* -----------------------------------------------------------------------------
      Definitions for TRACE feature
-------------------------------------------------------------------------------*/
#define TRACER_EMB_BAUDRATE                          921600UL

#define TRACER_EMB_DMA_MODE                          1UL
#define TRACER_EMB_IT_MODE                           0UL

#define TRACER_EMB_BUFFER_SIZE                       4096UL

/* -----------------------------------------------------------------------------
      Definitions for TRACE Hw information
-------------------------------------------------------------------------------*/

/*definition for USART1 on Nucleo 144 STM32L4R5ZI*/
#define TRACER_EMB_USART_INSTANCE                    USART1

#define TRACER_EMB_TX_GPIO                           GPIOA
#define TRACER_EMB_TX_PIN                            LL_GPIO_PIN_9
#define TRACER_EMB_TX_AF                             LL_GPIO_AF_7
#define TRACER_EMB_TX_GPIO_ENABLE_CLOCK()            LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA)

#define TRACER_EMB_RX_GPIO                           GPIOA
#define TRACER_EMB_RX_PIN                            LL_GPIO_PIN_10
#define TRACER_EMB_RX_AF                             LL_GPIO_AF_7
#define TRACER_EMB_RX_GPIO_ENABLE_CLOCK()            LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA)
  
#define TRACER_EMB_ENABLE_CLK_USART()                LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define TRACER_EMB_DISABLE_CLK_USART()               LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_USART1)
#define TRACER_EMB_SET_CLK_SOURCE_USART()            LL_RCC_USART1_CLKSOURCE
#define TRACER_EMB_USART_IRQ                         USART1_IRQn
#define TRACER_EMB_USART_IRQHANDLER                  USART1_IRQHandler
#define TRACER_EMB_TX_AF_FUNCTION                    LL_GPIO_SetAFPin_8_15
#define TRACER_EMB_RX_AF_FUNCTION                    LL_GPIO_SetAFPin_8_15
#define TRACER_EMB_TX_IRQ_PRIORITY                   9
#if TRACER_EMB_DMA_MODE == 1UL
#define TRACER_EMB_DMA_INSTANCE                      DMA1
#define TRACER_EMB_ENABLE_CLK_DMA()                  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
#if defined (DMAMUX_CxCR_DMAREQ_ID)
#define TRACER_EMB_TX_DMA_REQUEST                    LL_DMAMUX_REQ_USART1_TX 
#endif  /* DMAMUX_CxCR_DMAREQ_ID */
#define TRACER_EMB_ENABLECHANNEL                     LL_DMA_EnableChannel
#define TRACER_EMB_DISABLECHANNEL                    LL_DMA_DisableChannel

//#if USBPD_PORT_COUNT == 2
#define TRACER_EMB_TX_DMA_CHANNEL                    LL_DMA_CHANNEL_5
#define TRACER_EMB_TX_DMA_IRQ                        DMA1_Channel5_IRQn 
#define TRACER_EMB_TX_DMA_IRQHANDLER                 DMA1_Channel5_IRQHandler
#define TRACER_EMB_TX_DMA_ACTIVE_FLAG                LL_DMA_IsActiveFlag_TC5
#define TRACER_EMB_TX_DMA_CLEAR_FLAG                 LL_DMA_ClearFlag_GI5
#define TRACER_EMB_TX_DMA_PRIORITY                   8

#endif
#ifdef __cplusplus
}
#endif

#endif /* TRACER_EMB_CONF_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

