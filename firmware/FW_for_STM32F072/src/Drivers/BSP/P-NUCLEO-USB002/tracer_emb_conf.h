/**
  ******************************************************************************
  * @file    usbpd_devices_conf.h
  * @author  MCD Application Team
  * @brief   This file contains the device define.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

#define TRACER_EMB_BUFFER_SIZE                       512UL

/* -----------------------------------------------------------------------------
      Definitions for TRACE Hw information
-------------------------------------------------------------------------------*/

/* USART instance is used.*/
#define TRACER_EMB_IS_INSTANCE_LPUART_TYPE           0UL /* set to 1UL if LPUART is used instead of USART */
#define TRACER_EMB_USART_INSTANCE                    USART1

#define TRACER_EMB_TX_GPIO                           GPIOA
#define TRACER_EMB_TX_PIN                            LL_GPIO_PIN_9
#define TRACER_EMB_TX_AF                             LL_GPIO_AF_1
#define TRACER_EMB_TX_GPIO_ENABLE_CLOCK()            LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)
#define TRACER_EMB_TX_GPIO_DISABLE_CLOCK()           LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOA)
#define TRACER_EMB_RX_GPIO                           GPIOA
#define TRACER_EMB_RX_PIN                            LL_GPIO_PIN_10
#define TRACER_EMB_RX_AF                             LL_GPIO_AF_1
#define TRACER_EMB_RX_GPIO_ENABLE_CLOCK()            LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)
#define TRACER_EMB_RX_GPIO_DISABLE_CLOCK()           LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_GPIOA)

#define TRACER_EMB_ENABLE_CLK_USART()                LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1)
#define TRACER_EMB_DISABLE_CLK_USART()               LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_USART1)
#define TRACER_EMB_SET_CLK_SOURCE_USART()            
#define TRACER_EMB_USART_IRQ                         USART1_IRQn
#define TRACER_EMB_USART_IRQHANDLER                  USART1_IRQHandler
#define TRACER_EMB_TX_AF_FUNCTION                    LL_GPIO_SetAFPin_8_15
#define TRACER_EMB_RX_AF_FUNCTION                    LL_GPIO_SetAFPin_8_15
#define TRACER_EMB_TX_IRQ_PRIORITY                   3
#if TRACER_EMB_DMA_MODE == 1UL
#define TRACER_EMB_DMA_INSTANCE                      DMA1
#define TRACER_EMB_ENABLE_CLK_DMA()                  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);\
                                                     if (  TRACER_EMB_TX_DMA_CHANNEL ==  LL_DMA_CHANNEL_4){ \
                                                     LL_SYSCFG_SetRemapDMA_USART(LL_SYSCFG_USART1TX_RMP_DMA1CH4);  }
#if defined (DMAMUX_CxCR_DMAREQ_ID)
#define TRACER_EMB_TX_DMA_REQUEST                    LL_DMAMUX_REQ_USART1_TX         /* This define is needed only in case of DMAMUX management */
#endif  /* DMAMUX_CxCR_DMAREQ_ID */
#if defined(DMA_SxCR_CHSEL)
#define TRACER_EMB_TX_DMA_STREAM		     LL_DMA_STREAM_NODMA                 /* This define is needed only in case of use of a DMA IP supporting Streams */
#endif  /* DMA_SxCR_CHSEL */
#define TRACER_EMB_TX_DMA_CHANNEL                    LL_DMA_CHANNEL_4
#if defined(DMA_SxCR_CHSEL)
/* Those defines are needed only in case of use of a DMA IP supporting Streams */
#define TRACER_EMB_ENABLESTREAM                      LL_DMA_EnableStream
#define TRACER_EMB_DISABLESTREAM                     LL_DMA_DisableStream
#else
/* Those defines are needed only in case of use of a DMA IP not supporting Streams */
#define TRACER_EMB_ENABLECHANNEL                     LL_DMA_EnableChannel
#define TRACER_EMB_DISABLECHANNEL                    LL_DMA_DisableChannel
#endif  /* DMA_SxCR_CHSEL */
#define TRACER_EMB_TX_DMA_IRQ                        DMA1_Channel4_5_6_7_IRQn // DMA1_Channel2_DMAMUX1_OVR_IRQn
#define TRACER_EMB_TX_DMA_IRQHANDLER                 DMA1_Channel4_5_6_7_IRQHandler
#define TRACER_EMB_TX_DMA_ACTIVE_FLAG                LL_DMA_IsActiveFlag_TC4
#define TRACER_EMB_TX_DMA_CLEAR_FLAG                 LL_DMA_ClearFlag_GI4
#define TRACER_EMB_TX_DMA_PRIORITY                   3
#endif
#ifdef __cplusplus
}
#endif

#endif /* USBPD_DEVICE_CONF_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

