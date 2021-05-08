/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#ifdef _RTOS
#include "FreeRTOS.h"
#include "task.h"
#endif
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbpd_core.h"
#if defined(_TRACE)
#include "tracer_emb.h"
#endif /* _TRACE */
#if defined(_GUI_INTERFACE)
#include "gui_api.h"
#endif /* _GUI_INTERFACE */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#if defined(USBPD_USBDATA)
extern PCD_HandleTypeDef hpcd;
void SystemClock_Config(void);
#endif /*USBPD_USBDATA*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
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
  index_meas++ ; 
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

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
#if defined(MB1303) && defined (RX_DMACH)
#if USBPD_PORT_COUNT == 2
void DMA2_Stream2_IRQHandler(void)
{
  USBPD_DMA_PORTX_IRQHandler(1);
}
#endif
#endif

#if defined(MB1303)
#if USBPD_PORT_COUNT == 2
void DMA2_Stream3_IRQHandler(void)
{
  USBPD_DMA_PORTX_IRQHandler(1);
}
#endif
#endif
/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  USBPD_DMA_PORTX_IRQHandler(0);
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
#if defined(MB1303) 
void DMA1_Stream3_IRQHandler(void) 
#else
void DMA1_Stream6_IRQHandler(void)
#endif
{
#if defined (RX_DMACH)
  USBPD_DMA_PORTX_IRQHandler(0);
#endif

}
#if defined(_TRACE)
void DMA2_Stream7_IRQHandler(void)
{
#if TRACER_EMB_DMA_MODE == 1UL  
   TRACER_EMB_IRQHandlerDMA();
#endif
} 
#endif
/**
  * @brief This function handles SPI2 global interrupt.
  */
#if defined (MB1303) || defined(_BUCKCV)
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
#endif
/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
#if defined(MB1303)
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ALERT_GPIO_PIN(0));
}
#if USBPD_PORT_COUNT == 2
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(ALERT_GPIO_PIN(1));
 }
#endif
#endif


void EXTI15_10_IRQHandler(void)
{
//#ifndef MB1303 
//  HAL_GPIO_EXTI_IRQHandler(ALERT_GPIO_PIN(0));
//#endif  
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(SPI_NSS_PIN(0));     
#if defined(MB1303) 
#if USBPD_PORT_COUNT == 2
  HAL_GPIO_EXTI_IRQHandler(SPI_NSS_PIN(1));
#endif /*(USBPD_PORT_COUNT == 2) */                                         
  HAL_GPIO_EXTI_IRQHandler(USER_BUTTON_PIN);
#endif /* MB1303 */ 
}
#if defined(_TRACE)
void USART1_IRQHandler(void )
//#endif
{ 
    TRACER_EMB_IRQHandlerUSART();
}
#endif
#if defined(USBPD_USBDATA)
/**
  * @brief  This function handles USB Handler.
  * @param  None
  * @retval None
  */
void OTG_FS_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_WKUP_IRQn 0 */

  /* USER CODE END OTG_FS_WKUP_IRQn 0 */
  if ((&hpcd)->Init.low_power_enable) {
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB->SCR &= (uint32_t)~((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
    SystemClock_Config();
  }
  __HAL_PCD_UNGATE_PHYCLOCK(&hpcd);
  /* Clear EXTI pending bit */
  __HAL_USB_OTG_FS_WAKEUP_EXTI_CLEAR_FLAG();
  /* USER CODE BEGIN OTG_FS_WKUP_IRQn 1 */

  /* USER CODE END OTG_FS_WKUP_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}
#endif /*USBPD_USBDATA*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
