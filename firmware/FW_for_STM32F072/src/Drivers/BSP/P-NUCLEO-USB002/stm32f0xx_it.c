/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#if defined(USBPD_TCPM_MODULE_ENABLED)
#include "usbpd_tcpci.h"
#endif /*USBPD_TCPM_MODULE_ENABLED*/

#include "stm32f0xx_it.h"


#if defined(_TRACE)
#include "usbpd_core.h"
#include "tracer_emb.h"
#endif /* _TRACE */
#if defined(_GUI_INTERFACE)
#include "gui_api.h"
#endif /* _GUI_INTERFACE */

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#if defined(USBPD_USBDATA)
extern PCD_HandleTypeDef hpcd;
#endif /*USBPD_USBDATA*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
#if defined (_BUCKCV) || defined(_ADC_MONITORING)
uint8_t index_meas = 0;
extern uint16_t current_meas[USBPD_PORT_COUNT][5]; 
#endif
/**
  * @brief  This function handles SysTick Handler.
  * @retval None
  */
void SysTick_Handler(void)
{
#if defined (_BUCKCV)  || defined(_ADC_MONITORING)
    current_meas[0][index_meas] = (uint16_t) APPLI_GetIvbus(0);
#if USBPD_PORT_COUNT == 2
    current_meas[1][index_meas] = (uint16_t) APPLI_GetIvbus(1);
#endif 
  index_meas++;  
  index_meas = index_meas % 5;
#endif
#if defined(_OPTIM_CONSO)
#else
  HAL_IncTick();
  USBPD_DPM_TimerCounter();
#if defined(_GUI_INTERFACE)
  GUI_TimerCounter();
#endif /* _GUI_INTERFACE */
#endif /* _OPTIM_CONSO */
}


#ifdef MB1303
/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/ 
/**
  * @brief  This function handles EXTI line 4_15 interrupts.
  * @retval None
  */
void EXTI0_1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ALERT_GPIO_PIN(0));

}

#if USBPD_PORT_COUNT == 2
/**
  * @brief  This function handles EXTI line 2 to 3 interrupts.
  * @retval None
  */
void EXTI2_3_IRQHandler(void)
{  
  HAL_GPIO_EXTI_IRQHandler(ALERT_GPIO_PIN(1));
}
#endif /* (USBPD_PORT_COUNT == 2)*/
#endif /* MB1303 */

/**
  * @brief  This function handles EXTI line 4_15 interrupts.
  * @retval None
  */
void EXTI4_15_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(SPI_NSS_PIN(0)); 
#if defined(MB1303) 
#if USBPD_PORT_COUNT == 2
  HAL_GPIO_EXTI_IRQHandler(SPI_NSS_PIN(1));
#endif /*(USBPD_PORT_COUNT == 2) */                                         
  HAL_GPIO_EXTI_IRQHandler(USER_BUTTON_PIN);
#endif /* MB1303 */  
}

/**
  * @brief  This function handles DMA Channel 2 and 3 interrupts.
  * @param  None
  * @retval None
  */
void DMA1_Channel2_3_IRQHandler(void)
{ 
#if defined(MB1303)
#if (USBPD_PORT_COUNT == 2)
  USBPD_DMA_PORTX_IRQHandler(1);
#endif 
#endif 
}

/**
  * @brief  This function handles DMA Channel 4 to 7 interrupts.
  * @param  None
  * @retval None
  */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  
 USBPD_DMA_PORTX_IRQHandler(0);
#if defined(_TRACE)
#if TRACER_EMB_DMA_MODE == 1UL  
   TRACER_EMB_IRQHandlerDMA();
#endif
#endif

}

#if defined(USBPD_CLI) || defined(USBPD_TERMINAL_OUTPUT) || defined(_TRACE) || defined(_GUI_INTERFACE)
/**
  * @brief  This function handles USART exception.
  * @param  None
  * @retval None        */
#if defined(_TRACE) || defined(_GUI_INTERFACE)
#if defined (MB1303) 
void USART1_IRQHandler(void )
#elif defined (_STUSB4761)||defined(_BUCKCV) || defined(BOARD_100W)
void USART2_IRQHandler(void )
#else 
void USART1_IRQHandler(void )
#endif
{ 
    HW_TRACER_EMB_IRQHandlerUSART();
}
#else

#if defined (MB1303) 
void USART1_IRQHandler(void )
#elif defined (_STUSB4761)||defined(_BUCKCV) 
void USART2_IRQHandler(void )
#else 
void USART1_IRQHandler(void )
#endif
{  
 HAL_UART_IRQHandler(&huart_handle);
}
/*_GUI_INTERFACE*/
#endif /*_TRACE */
/*_GUI_INTERFACE || USBPD_CLI*/
#endif /* USBPD_TERMINAL_OUTPUT || _TRACE */


/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
#if defined (MB1303) || defined (_STUSB4761) || defined(_BUCKCV)
void SPI2_IRQHandler(void)
{                                                 
  
  RX_ByteReceiverHandler(0);
  
}

#if (USBPD_PORT_COUNT == 2)

void SPI1_IRQHandler(void)
{
  RX_ByteReceiverHandler(1);
}
#endif /* USBPD_PORT_COUNT == 2 */
#elif defined(BOARD_100W)
void SPI1_IRQHandler(void)
{
  RX_ByteReceiverHandler(0);
}
#endif /*MB1303*/

#if defined(USBPD_USBDATA)
/**
  * @brief  This function handles USB Handler.
  * @param  None
  * @retval None
  */
void USB_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}
#endif /*USBPD_USBDATA*/

#if defined(_OPTIM_CONSO)
/**
  * @brief  This function handles RTC interrupt.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
  HAL_IncTick();
  USBPD_DPM_TimerCounter();

  /* Reset Internal Wake up flag */
  LL_RTC_ClearFlag_WUT(RTC);

  /* Clear the EXTI's line Flag for RTC WakeUpTimer */
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_20);
}
#endif /* _OPTIM_CONSO */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
