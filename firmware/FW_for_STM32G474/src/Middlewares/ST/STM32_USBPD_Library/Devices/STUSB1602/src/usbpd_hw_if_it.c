/**
******************************************************************************
* @file    usbpd_hw_if_it.c
* @author  AMG Application Team
* @brief   This file contains HW interface interrupt routines.
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


#include "string.h"

#include "STUSB1602_Peripherals_if.h"


extern STUSB16xx_PORT_HandleTypeDef Ports[];

/** @addtogroup STM32_USBPD_LIBRARY
* @{
*/

/** @addtogroup USBPD_DEVICE
* @{
*/

/** @addtogroup USBPD_DEVICE_HW_IF
* @{
*/

/** @addtogroup USBPD_DEVICE_HW_IF_IT
* @{
*/


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
* @var     GPIO_PinState RxNSSStatus[2]
* @brief   SPI NSS status in reception phase
* @details It is an array that stores the previous value of the SPI NSS pins to check if a falling or rising event is occurring
*/
static GPIO_PinState RxNSSStatus[2] = {GPIO_PIN_SET,GPIO_PIN_SET};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Optimized functions */

/**
* @brief   EXTI line detection callback
* @param   GPIO_Pin It specifies the pins connected to EXTI line
* @note    The parameter can be one of the following values:
* @arg     USER_BUTTON_PIN      ** Defined in stm32f0xx_nucleo.h **
* @arg     ALERT_GPIO_PIN(0)    ** Defined in usbpd_porthandle.h **
* @arg     SPI_NSS_PIN(0)       ** Defined in usbpd_porthandle.h **
* @arg     ALERT_GPIO_PIN(1)    ** Defined in usbpd_porthandle.h **
* @arg     SPI_NSS_PIN(1)       ** Defined in usbpd_porthandle.h **
* @retval None
*/
void USBPD_HW_IF_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t PortNum=0;
  GPIO_PinState NSSCurrentState;
  switch (GPIO_Pin)
  {
  case ALERT_GPIO_PIN(0):
    STUSB16xx_HW_IF_Alert_Check(ALERT_PORT_INDEX(0));
    break;
    
  case SPI_NSS_PIN(0):
    PortNum = 0;
    
    /* Check the NSS current state and compare it with the stored value*/
    NSSCurrentState = HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum));
    
    if( NSSCurrentState == GPIO_PIN_SET || RxNSSStatus[PortNum] == GPIO_PIN_RESET) 
    {
      if ( HAL_GPIO_ReadPin(TX_EN_GPIO_PORT(PortNum), TX_EN_GPIO_PIN(PortNum)) == GPIO_PIN_SET)
      {
        PHY_HW_IF_TX_ABORT(PortNum);
      }
      else
      {
        /* End of reception */
        PHY_HW_IF_RX_Stop(PortNum);
      }
    }
    if(( NSSCurrentState == GPIO_PIN_RESET ) && ( HAL_GPIO_ReadPin(TX_EN_GPIO_PORT(PortNum), TX_EN_GPIO_PIN(PortNum)) == GPIO_PIN_RESET))
      {
        PHY_HW_IF_RX_Start(PortNum);     
      }

    
    /* store the current NSS status */
    RxNSSStatus[PortNum] = NSSCurrentState;
    
    break;    
    
#if defined(MB1303) && (USBPD_PORT_COUNT == 2)
  case ALERT_GPIO_PIN(1):
    STUSB16xx_HW_IF_Alert_Check(ALERT_PORT_INDEX(1));
    break;
    
  case SPI_NSS_PIN(1):
    PortNum = 1;
    
    /* Check the NSS current state and compare it with the stored value*/

   NSSCurrentState = HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum));

    
    if( NSSCurrentState == GPIO_PIN_SET || RxNSSStatus[PortNum] == GPIO_PIN_RESET) 
    {
      if ( HAL_GPIO_ReadPin(TX_EN_GPIO_PORT(PortNum), TX_EN_GPIO_PIN(PortNum)) == GPIO_PIN_SET)
      {
        PHY_HW_IF_TX_ABORT(PortNum);
      }
      else
      {
        /* End of reception */
        PHY_HW_IF_RX_Stop(PortNum);
      }
    }
    if( NSSCurrentState == GPIO_PIN_RESET )
      {
        PHY_HW_IF_RX_Start(PortNum);     
      }

    /* store the current NSS status */
    RxNSSStatus[PortNum] = NSSCurrentState;
    
    break;
#endif
    
  default:
    __NOP();
    break;
  }
}


/**
* @brief   This function handles DMA interrupts on channels 4, 5, 6 and 7
* @retval  None
*/
void USBPD_DMA_PORTX_IRQHandler(uint8_t PortNum)
{
  uint8_t *end;
  
  /* Handler DMA TX PORT   */
  if(__HAL_DMA_GET_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[PortNum].hdmatx)) != RESET)
  {
    /* Transfer complete interrupt is used to end the transmission */
    if (Ports[PortNum].State!=HAL_USBPD_PORT_STATE_BIST)
    {
      __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx,__HAL_DMA_GET_TC_FLAG_INDEX(&Ports[PortNum].hdmatx));
      PHY_HW_IF_TX_Done(PortNum);
    }    
    else
    { 
      Ports[PortNum].BIST_index+=1;
      __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx,__HAL_DMA_GET_TC_FLAG_INDEX(&Ports[PortNum].hdmatx));
#if !defined (SPI_SR_FTLVL)   /*to drive SPI without fifo*/
      if(Ports[PortNum].BIST_index == BIST_MAX_LENGTH+1) 
#else
      if(Ports[PortNum].BIST_index == BIST_MAX_LENGTH)
#endif
        {
          /* Get the address of the transmission buffer*/
          end = Ports[PortNum].pTxRxBuffPtr;
          
          end[TX_BUFFER_LEN - 1] = 0;
#if !defined (SPI_SR_FTLVL)   /*to drive SPI without fifo*/
          PHY_HW_IF_TX_Done(PortNum);
#endif 
          STUSB16xx_HW_IF_Set_DMA_Normal_Mode(PortNum); 
        }
      if(Ports[PortNum].BIST_index > BIST_MAX_LENGTH) 
      {
        PHY_HW_IF_TX_Done(PortNum);
        Ports[PortNum].BIST_index=0;
        
      }
    }
  }
  if(__HAL_DMA_GET_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(&Ports[PortNum].hdmatx)) != RESET)
  {
    if(HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum))== GPIO_PIN_SET )
      PHY_HW_IF_TX_ABORT(PortNum);
    __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(&Ports[PortNum].hdmatx)); 
  }
  if(__HAL_DMA_GET_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(&Ports[PortNum].hdmatx)) != RESET)
  {
    if(HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum))== GPIO_PIN_SET )
      PHY_HW_IF_TX_ABORT(PortNum);
    __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(&Ports[PortNum].hdmatx)); 
  }
#ifdef __HAL_DMA_GET_FE_FLAG_INDEX 
  if(__HAL_DMA_GET_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(&Ports[PortNum].hdmatx)) != RESET)
    __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(&Ports[PortNum].hdmatx));
#endif
#ifdef __HAL_DMA_GET_DME_FLAG_INDEX 
  if(__HAL_DMA_GET_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(&Ports[PortNum].hdmatx)) != RESET)
    __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(&Ports[PortNum].hdmatx)); 
  
#endif 
  
  
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

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
