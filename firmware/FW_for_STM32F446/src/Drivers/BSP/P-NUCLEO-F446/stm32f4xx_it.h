/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);
#if defined(USBPD_USBDATA)
void OTG_FS_WKUP_IRQHandler(void);
void OTG_FS_IRQHandler(void);
#endif
#if defined(MB1303) && defined (RX_DMACH)
#if USBPD_PORT_COUNT == 2
void DMA2_Stream2_IRQHandler(void);
#endif 
#endif 
#if defined(MB1303) 
void DMA1_Stream3_IRQHandler(void);    // UART3 debug trace  + SPIDMARX Port 0 if needed
#else
void DMA1_Stream6_IRQHandler(void);   //UART2 debug trace (buckcv)
#endif

void DMA1_Stream4_IRQHandler(void);   //SPIDMATX port0
#if defined(_TRACE)//&& defined(STM32F401xE)
void DMA2_Stream7_IRQHandler(void);
#endif
#if defined(MB1303)
#if USBPD_PORT_COUNT == 2
void DMA2_Stream3_IRQHandler(void);    //SPIDMATX port1
#endif
#endif 
#if defined(MB1303)
//#if defined(STM32F446xx)
//void USART3_IRQHandler(void );
//#else
void USART1_IRQHandler(void );
//#endif
//#else
//void USART2_IRQHandler(void );
#endif
void SPI2_IRQHandler(void);
#if USBPD_PORT_COUNT == 2
void SPI1_IRQHandler(void);
#endif
void EXTI15_10_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);

void DMA2_Stream0_IRQHandler(void); // DMA ADC1

#if defined(MB1303)
void EXTI1_IRQHandler(void);
#if USBPD_PORT_COUNT == 2
void EXTI2_IRQHandler(void);
#endif
#endif
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
