/**
  ******************************************************************************
  * @file    usbpd_timersserver.c
  * @author  MCD Application Team
  * @brief   This file contains timer server functions.
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

#include "User_BSP.h"

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_TIMESERVER
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Definition for TIMx clock resources */
/* clock source is defined in BSP .h file */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialization of TIMERSERVER, used for CRC and RETRY operations
  * @retval None
  */
void USBPD_TIM_Init(void)
{
  TIMx_CLK_ENABLE;
 #if (USBPD_PORT_COUNT == 2)
  TIM_PE_CLK_ENABLE;
#endif  
  /***************************/
  /* Time base configuration */
  /***************************/

  /* Counter mode: select up-counting mode */
  LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP);
#if (USBPD_PORT_COUNT == 2)
  LL_TIM_SetCounterMode(TIM_PE, LL_TIM_COUNTERMODE_UP);
#endif
  /* Set the pre-scaler value to have TIMx counter clock equal to 1 MHz */
  LL_TIM_SetPrescaler(TIMx, __LL_TIM_CALC_PSC(TIMx_Clock_Freq, 1000000));
#if (USBPD_PORT_COUNT == 2)  
  LL_TIM_SetPrescaler(TIM_PE, __LL_TIM_CALC_PSC(TIM_PE_Clock_Freq, 1000000));
#endif  
  /* Set the auto-reload value to have a counter frequency of 100Hz */
  LL_TIM_SetAutoReload(TIMx, __LL_TIM_CALC_ARR(TIMx_Clock_Freq, LL_TIM_GetPrescaler(TIMx), 100u));
#if (USBPD_PORT_COUNT == 2)  
  LL_TIM_SetAutoReload(TIM_PE, __LL_TIM_CALC_ARR(TIM_PE_Clock_Freq, LL_TIM_GetPrescaler(TIM_PE), 100u));
#endif 
  /*********************************/
  /* Output waveform configuration */
  /*********************************/

  /* Set output compare mode: TOGGLE */
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_TOGGLE);
#if (USBPD_PORT_COUNT == 2)
  LL_TIM_OC_SetMode(TIM_PE, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_TOGGLE);
  LL_TIM_OC_SetMode(TIM_PE, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_TOGGLE);
#endif 

  /* Set output channel polarity: OC is active high */
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
#if (USBPD_PORT_COUNT == 2)
  LL_TIM_OC_SetPolarity(TIM_PE, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM_PE, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
#endif 
  /* Enable counter */
  LL_TIM_EnableCounter(TIMx);
#if (USBPD_PORT_COUNT == 2)
  LL_TIM_EnableCounter(TIM_PE);
#endif 
}


/**
  * @brief  Start TIMERSERVER (CRC and RETRY operation)
  * @param  id Timer Operation Identifier
  * @param  us_time time in micro-seconds
  * @retval None
  */
void USBPD_TIM_Start(TIM_identifier id, uint16_t us_time)
{
  switch (id)
  {
  case TIM_PORT0_CA:
    LL_TIM_OC_SetCompareCH1(TIMx, (us_time + TIMx->CNT) % TIM_MAX_TIME);
    LL_TIM_ClearFlag_CC1(TIMx);
    break;
  case TIM_PORT0_RETRY:
    LL_TIM_OC_SetCompareCH2(TIMx, (us_time + TIMx->CNT) % TIM_MAX_TIME);
    LL_TIM_ClearFlag_CC2(TIMx);
    break;
#if (USBPD_PORT_COUNT == 2)
  case TIM_PORT1_CA:
    LL_TIM_OC_SetCompareCH3(TIMx, (us_time + TIMx->CNT) % TIM_MAX_TIME);
    LL_TIM_ClearFlag_CC3(TIMx);
    break;
  case TIM_PORT1_RETRY:
      LL_TIM_OC_SetCompareCH4(TIMx, (us_time + TIMx->CNT) % TIM_MAX_TIME);
      LL_TIM_ClearFlag_CC4(TIMx);
      break;
  case TIM_PORT0_CRC:
    LL_TIM_OC_SetCompareCH1(TIM_PE, (us_time + TIM_PE->CNT) % TIM_MAX_TIME);
    LL_TIM_ClearFlag_CC1(TIM_PE);
    break;
  case TIM_PORT1_CRC:
    LL_TIM_OC_SetCompareCH2(TIM_PE, (us_time + TIM_PE->CNT) % TIM_MAX_TIME);
    LL_TIM_ClearFlag_CC2(TIM_PE);
    break;
#else
  case TIM_PORT0_CRC:
    LL_TIM_OC_SetCompareCH3(TIMx, (us_time + TIMx->CNT) % TIM_MAX_TIME);
    LL_TIM_ClearFlag_CC3(TIMx);
    break;
#endif    
    default:
      break;
  }
}


/**
  * @brief  Retrieve the TIMERSERVER status for a specified id
  * @param  id Timer Operation Identifier
  * @retval State of bit (1 or 0).
  */
uint8_t USBPD_TIM_IsExpired(TIM_identifier id)
{
  switch (id)
  {
  case TIM_PORT0_CA:
    return LL_TIM_IsActiveFlag_CC1(TIMx);
  case TIM_PORT0_RETRY:
    return LL_TIM_IsActiveFlag_CC2(TIMx);
#if (USBPD_PORT_COUNT == 2)
  case TIM_PORT1_CA:
    return LL_TIM_IsActiveFlag_CC3(TIMx);
  case TIM_PORT1_RETRY:
    return LL_TIM_IsActiveFlag_CC4(TIMx);
  case TIM_PORT0_CRC:
    return LL_TIM_IsActiveFlag_CC1(TIM_PE);
  case TIM_PORT1_CRC:
    return LL_TIM_IsActiveFlag_CC2(TIM_PE);    
#else
  case TIM_PORT0_CRC:
    return LL_TIM_IsActiveFlag_CC3(TIMx);
#endif     
  default:
    break;
  }
  return 1;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

