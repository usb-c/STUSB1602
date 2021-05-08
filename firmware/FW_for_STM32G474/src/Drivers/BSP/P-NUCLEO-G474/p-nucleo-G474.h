/**
  ******************************************************************************
  * @file    p-nucleo-G474.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of p-nucleo-usb002.c file.
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
/*#warning "P-Nucleo header bof"*/
#ifndef __P_NUCLEO_G4_H_
#define __P_NUCLEO_G4_H_
/*#warning "P-Nucleo header bof2"*/
#ifdef __cplusplus
extern "C"
{
#endif
 
/**
  * @addtogroup BSP
  * @{
  * */

/**
  * @addtogroup P-NUCLEO-USB002
  * @{
  * */

/* Includes ------------------------------------------------------------------*/
#if defined(STM32G474xx)

#include "stm32g4xx.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_exti.h" 
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_crc.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_cortex.h"  
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"  
#include "stm32g4xx_ll_pwr.h"  
#if defined(_OPTIM_CONSO)
#include "stm32g4xx_ll_rtc.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_adc.h"
#endif /* _OPTIM_CONSO_ */  
#else
#error "Add include for the family!"
#endif

  
#define TIMx                           TIM2
#define TIMx_Clock_Freq                (SystemCoreClock *2) /4       /* depend on APB clock freq timer is connected */   
#define TIMx_CLK_ENABLE                __HAL_RCC_TIM2_CLK_ENABLE()
#if (USBPD_PORT_COUNT == 2)
#define TIM_PE                         TIM3
#define TIM_PE_Clock_Freq              (SystemCoreClock *2)/4        // depend on APB clock freq timer is connected 
#define TIM_PE_CLK_ENABLE              __HAL_RCC_TIM3_CLK_ENABLE()
#endif  

  
#define FLASH_BANK1_END FLASH_END 
//#define FLASH_PAGE_SIZE 0x1FFFF
/** @defgroup P-NUCLEO-USB001_Exported_Macros Exported Macros
  * @{
  */
#define USBPD_BSP_PIN(PORT,PIN)    { PORT, GPIO_PIN_ ## PIN, 0 }
#define USBPD_BSP_ADC(PORT,PIN,HANDLE,CH,RANK) { PORT, GPIO_PIN_ ## PIN,HANDLE ,CH, RANK }
  
#define USBPD_LOWEST_IRQ_PRIO   15   /*!< Lowest priority                    */
#define USBPD_LOW_IRQ_PRIO      7   /*!< High priority shift value          */ //nathe//
#define USBPD_HIGH_IRQ_PRIO     5   /*!< Low priority shift value           */
#define RX_IRQ_PRIO             6   /*!< Rx priority for first interrupt @note Communication is half duplex so @ref TX_IRQ_PRIO = @ref RX_IRQ_PRIO */
#define TX_IRQ_PRIO             6   /*!< Tx interrupt priority @note Communication is half duplex so @ref TX_IRQ_PRIO = @ref RX_IRQ_PRIO */

#define SPI_ENTER_CRITICAL_SECTION()// uint32_t primask= __get_PRIMASK();\ 
                                    //   __disable_irq();

#define SPI_LEAVE_CRITICAL_SECTION() // __set_PRIMASK(primask)

/**
  * @brief This struct contains parameter used to define array of pins
  */
typedef struct
{
      GPIO_TypeDef*   GPIOx;      /*!< The GPIO Port of the Pin */
      uint16_t        GPIO_Pin;   /*!< The GPIO_Pin */
      uint32_t        ADCCH;      /*!< The ADC Channel if used */
} USBPD_BSP_GPIOPins_TypeDef;

typedef struct
{
      GPIO_TypeDef*   GPIOx;      /*!< The GPIO Port of the Pin */
      uint16_t        GPIO_Pin;   /*!< The GPIO_Pin */
      ADC_HandleTypeDef * hadc ;
      uint32_t        ADCCH;      /*!< The ADC Channel if used */
      uint32_t        RANK;       /*!< The ADC Rank if used */
} USBPD_BSP_GPIOADCPins_TypeDef;

void USBPD_HW_IF_GPIO_Set(USBPD_BSP_GPIOPins_TypeDef gpio, GPIO_PinState PinState);
void USBPD_HW_IF_GPIO_On(USBPD_BSP_GPIOPins_TypeDef gpio);
void USBPD_HW_IF_GPIO_Off(USBPD_BSP_GPIOPins_TypeDef gpio);
void USBPD_HW_IF_GPIO_Toggle(USBPD_BSP_GPIOPins_TypeDef gpio);

/**
  * @}
  */

/** @defgroup P-NUCLEO-USB002_Exported_Types Exported Types
  * @{
  */
/** @defgroup USBPD_DEVICE_PORTHANDLE_STUSB1602_IOs STUSB1602 IOs
  *   @brief All those macro are used to define the HW setup on MCU side
  * @{
  */

/* GPIOs */
#define TX_EN_GPIO_PORT(__PORT__)           ( GPIOC )                                       /*!< TX_EN GPIO Port           */
#define TX_EN_GPIO_PIN(__PORT__)            ((__PORT__ == 0) ? GPIO_PIN_2 : GPIO_PIN_3 )    /*!< TX_EN GPIO Pin            */
#define RESET_GPIO_PORT(__PORT__)           ( GPIOC )                                       /*!< RESET GPIO Port           */
#define RESET_GPIO_PIN(__PORT__)            ((__PORT__ == 0) ? GPIO_PIN_6 : GPIO_PIN_7 )    /*!< RESET GPIO PIN            */
#define A_B_Side_GPIO_PORT(__PORT__)        ( GPIOA )                                       /*!< A_B Side GPIO Port        */
#define A_B_Side_GPIO_PIN(__PORT__)         ((__PORT__ == 0) ? GPIO_PIN_0 : GPIO_PIN_3 )    /*!< A_B Side GPIO Pin         */
#define ALERT_PORT_INDEX(__PORT__)          ((__PORT__ == 0) ? 0 : 1 )                      /*!< ALERT Pin index           */
#define ALERT_GPIO_PORT(__PORT__)           ( GPIOA )                                       /*!< ALERT GPIO Port           */
#define ALERT_GPIO_PIN(__PORT__)            ((__PORT__ == 0) ? GPIO_PIN_1 : GPIO_PIN_2 )    /*!< ALERT GPIO Pin            */
#define ALERT_GPIO_IRQHANDLER(__PORT__)     ((__PORT__ == 0) ? EXTI1_IRQn : EXTI2_IRQn )/*!< ALERT GPIO IRQn           */
#define ALERT_GPIO_IRQPRIORITY(__PORT__)    ( USBPD_LOW_IRQ_PRIO )                          /*!< ALERT IRQ priority        */

/* I2C */
#define I2C_INSTANCE(__PORT__)              ( I2C3 )                    /*!< I2C: Device           */
#define I2C_TIMING(__PORT__)                (0x00E057FD )               /*!< I2C: Timing value     */
#define I2C_PORT(__PORT__)                  ( GPIOC )                   /*!< I2C: GPIO Port        */
#define I2C_SCL_PIN(__PORT__)               ( GPIO_PIN_8 )              /*!< I2C: SCL Pin          */
#define I2C_SCL_PORT(__PORT__)              ( GPIOC )                   /*!< I2C: SCL Port         */
#define I2C_SDA_PIN(__PORT__)               ( GPIO_PIN_9 )              /*!< I2C: SDA Pin          */
#define I2C_SDA_PORT(__PORT__)              ( GPIOC )                   /*!< I2C: SDA Port         */
#define I2C_MODE(__PORT__)                  ( GPIO_MODE_AF_OD )         /*!< I2C: GPIO Mode        */
#define I2C_PULL(__PORT__)                  ( GPIO_NOPULL )             /*!< I2C: Pull setup       */
#define I2C_ALTERNATE(__PORT__)             ( GPIO_AF8_I2C3 )           /*!< I2C: GPIO AF          */
#define I2C_IP_SPEED(__PORT__)              ( GPIO_SPEED_FREQ_HIGH )	/*!< I2C: GPIO Speed       */


/* SPI */
#define SPI_Instance(__PORT__)                  ((__PORT__ == 0) ? SPI2                  : SPI1 )                   /*!< SPI: Device           */
#define SPI_NSS_PORT(__PORT__)                  ((__PORT__ == 0) ? GPIOB                 : GPIOA )                  /*!< SPI: NSS Port         */
#define SPI_NSS_PIN(__PORT__)                   ((__PORT__ == 0) ? GPIO_PIN_12           : GPIO_PIN_15 )            /*!< SPI: NSS Pin          */
#define SPI_NSS_LL_APB(__PORT__)                ((__PORT__ == 0) ? LL_APB1_GRP1_PERIPH_SPI2  : LL_APB2_GRP1_PERIPH_SPI1 )  /*!< SPI: NSS LL Group     */
#define SPI_NSS_LL_PORT(__PORT__)               ((__PORT__ == 0) ? LL_SYSCFG_EXTI_PORTB  : LL_SYSCFG_EXTI_PORTA )   /*!< SPI: NSS LL Port      */
#define SPI_NSS_LL_SYS_EXTI(__PORT__)           ((__PORT__ == 0) ? LL_SYSCFG_EXTI_LINE12 : LL_SYSCFG_EXTI_LINE15 )  /*!< SPI: NSS LL CFG EXTI Line */
#define SPI_NSS_LL_EXTI(__PORT__)               ((__PORT__ == 0) ? LL_EXTI_LINE_12 : LL_EXTI_LINE_15 )              /*!< SPI: NSS LL EXTI Line */
                                               /*!< SPI: NSS LL IRQn      */
#define SPI_NSS_LL_IRQPRIORITY(__PORT__)        ( RX_IRQ_PRIO )                            /*!< SPI: NSS LL IRQ priority */
#define SPI_NSS_LL_APB_EN_CLK(__PORT__)         ((__PORT__ == 0) ? LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2)  : LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1) )
#define SPI_NSS_LL_APB_DIS_CLK(__PORT__)        ((__PORT__ == 0) ? LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI2)  : LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI1) )

#define SPI_CLK_PORT(__PORT__)                  ( GPIOB )                                                           /*!< SPI: CLK Port         */
#define SPI_MISO_PORT(__PORT__)                 ( GPIOB )                                                           /*!< SPI: MISO Port        */
#define SPI_CLK_PIN(__PORT__)                   ((__PORT__ == 0) ? GPIO_PIN_13           : GPIO_PIN_3 )             /*!< SPI: CLK Pin          */
#define SPI_MISO_PIN(__PORT__)                  ((__PORT__ == 0) ? GPIO_PIN_14           : GPIO_PIN_4 )             /*!< SPI: MISO Pin         */
#if !defined(SPI_ONE_LINE)
#define SPI_MOSI_PORT(__PORT__)                 ( GPIOB )                                                           /*!< SPI: MOSI Port        */
#define SPI_MOSI_PIN(__PORT__)                  ((__PORT__ == 0) ? GPIO_PIN_15           : GPIO_PIN_5 )             /*!< SPI: MOSI Pin         */
#define SPI_MOSI_ALTERNATE(__PORT__)            ((__PORT__ == 0) ? GPIO_AF5_SPI2         : GPIO_AF5_SPI1 )          /*!< SPI: MOSI Alternate Function */
#endif
#define SPI_NSS_LL_IRQHANDLER(__PORT__)         ( EXTI15_10_IRQn ) 
#define SPI_CLK_ALTERNATE(__PORT__)             ((__PORT__ == 0) ? GPIO_AF5_SPI2         : GPIO_AF5_SPI1 )          /*!< SPI: CLK Alternate Function */
#define SPI_NSS_ALTERNATE(__PORT__)             ((__PORT__ == 0) ? GPIO_AF5_SPI2         : GPIO_AF5_SPI1 )          /*!< SPI: NSS Alternate Function */
#define SPI_MISO_ALTERNATE(__PORT__)            ((__PORT__ == 0) ? GPIO_AF5_SPI2         : GPIO_AF5_SPI1 )          /*!< SPI: MISO Alternate Function */
/* Definition for SPIx's NVIC */
#define SPI_IRQn(__PORT__)                       ((__PORT__ == 0) ? SPI2_IRQn            : SPI1_IRQn )
#define SPIx_IRQHandler(__PORT__)                ((__PORT__ == 0) ? SPI2_IRQHandler      : SPI1_IRQHandler )
#define SPIx_IRQ_PRIO(__PORT__)                 ( USBPD_HIGH_IRQ_PRIO )    /*!< SPI: IRQ priority  must be higher    */


/* DMA */
#define TX_DMACH(__PORT__)                       ((__PORT__ == 0) ? DMA1_Channel4 : DMA1_Channel2)       /*!< DMA: Tx Channel  */
#define TXDMACHIRQ(__PORT__)                     ((__PORT__ == 0) ? DMA1_Channel4_IRQn : DMA1_Channel2_IRQn)         /*!< DMA: IRQn        */

#if defined(DMAMUX_CxCR_DMAREQ_ID)
#define DMA_REQUEST_SPI_TX(__PORT__)              ((__PORT__ == 0) ? DMA_REQUEST_SPI2_TX : DMA_REQUEST_SPI1_TX) /*!< DMA: Requestr           */
#endif


#define TXDMACHIRQ_PRIO(__PORT__)                  (TX_IRQ_PRIO )                              /*!< DMA: IRQ priority     */
#define TXDMACHIRQ_SUB_PRIO(__PORT__)              (0)                              /*!< DMA: IRQ priority     */

#define SPARE_BIT_SUB_CORRECTION                 0
#define PHY_TXRX_BYTE_TIMEOUT                    50  /*!< Max time to recieve or send a byte Time Elapsed value [us]       */
#define PHY_TX_BYTE_TIMEOUT                      50
#define SWCALL_RX_STOP(PORT)  LL_EXTI_GenerateSWI_0_31( SPI_NSS_PIN(PORT)) /* Macro Generating SWIE */

/**
  * @}
  */

/** @defgroup USBPD_DEVICE_PORTHANDLE_STUSB1602_MSP_MACRO MSP MACRO
  * @brief MACRO used by HAL MSP function to identify peripherals
 * @{
 */
#define GET_PORT_FROM_I2C(hi2c)  (0)//\
//( (uint8_t)( hi2c->Instance == I2C3)? 0 : 1 )   /*!< GET Port Number from I2C peripheral   */

#define I2C_CLK_ENABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_I2C3_CLK_ENABLE(); \
else \
  __HAL_RCC_I2C3_CLK_ENABLE();  \
} while(0)                                      /*!< Enables Clock of I2C peripheral for the specified port  */

#define I2C_CLK_DISABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_I2C3_CLK_DISABLE(); \
else \
  __HAL_RCC_I2C3_CLK_DISABLE();  \
} while(0)                                      /*!< Disables Clock of I2C peripheral for the specified port */

#define GET_PORT_FROM_SPI(hspi) \
( (uint8_t)( hspi->Instance == SPI_Instance(0) )? 0 : 1 )  /*!< GET Port Number from SPI peripheral   */

#define SPI_CLK_ENABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_SPI2_CLK_ENABLE(); \
else \
  __HAL_RCC_SPI1_CLK_ENABLE();  \
} while(0)                                      /*!< Enables Clock of SPI peripheral for the specified port  */

#define SPI_CLK_DISABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_SPI2_CLK_DISABLE(); \
else \
  __HAL_RCC_SPI1_CLK_DISABLE();  \
} while(0)                                      /*!< Disables Clock of SPI peripheral for the specified port */

#define SPI_FORCE_RESET(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_SPI2_FORCE_RESET(); \
else \
  __HAL_RCC_SPI1_FORCE_RESET();  \
} while(0) 

#define SPI_RELEASE_RESET(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_SPI2_RELEASE_RESET(); \
else \
  __HAL_RCC_SPI1_RELEASE_RESET();  \
} while(0) 
#define DMA_CLK_ENABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_DMA1_CLK_ENABLE(); \
else \
  __HAL_RCC_DMA1_CLK_ENABLE();   \
} while(0)                                      /*!< Enables Clock of DMA peripheral for the specified port  */

#define DMA_CLK_DISABLE(PortNum) do { \
if (PortNum==0) \
  __HAL_RCC_DMA1_CLK_DISABLE(); \
else \
  __HAL_RCC_DMA1_CLK_DISABLE();   \
} while(0)                                      /*!< Disables Clock of DMA peripheral for the specified port */


#define TIM_CRC(TIM_identifier) ((TIM_identifier == TIM_PORT0_CRC )? TIM_PORT0_CRC:TIM_PORT1_CRC)

#define STUSB1602_I2C_Add_0     					0x28 	/*<! Address 0 */
#define STUSB1602_I2C_Add_1     					0x29	/*<! Address 1 */
#define TIMEOUT_MAX             2000 /*<! The value of the maximal timeout for BUS waiting loops */

#define STUSB1602_I2C_Add(__PORT__)		((__PORT__ == 0) ? STUSB1602_I2C_Add_0 : STUSB1602_I2C_Add_1 )	/*!< I2C Address of the STUSB1602 device */


/**
  * @brief Leds on board P_NUCLEO_USB002
  *
  */ 
  typedef enum
  {
    ELED  = -1,
    ELED0 = 0,       /**<LED1 (green);  D107 of MB1303 X-NUCLEO */
    
    ELED1 = 1,       /**<LED01 (blue);  D100 of MB1303 X-NUCLEO */
    ELED2 = 2,       /**<LED02 (green); D101 of MB1303 X-NUCLEO */
    ELED3 = 3,       /**<LED03 (red);   D102 of MB1303 X-NUCLEO */
    
    ELED4 = 4,       /**<LED11 (blue);  D103 of MB1303 X-NUCLEO */
    ELED5 = 5,       /**<LED12 (green); D104 of MB1303 X-NUCLEO */
    ELED6 = 6,       /**<LED13 (red);   D105 of MB1303 X-NUCLEO */

    /* BSP name list */
    GREEN_USER_LED      = ELED0,

    LED_PORT0_CC        = ELED3,
    LED_PORT0_VBUS      = ELED2,
#if defined(__AUTHENTICATION__)
    /* In case of Authentication, LED01 and LED13 are mapped on 
       GPIOs used for I2C ST-SAFE communication. As only one port is handled,
       PORT0 Role is remapped on original PORT1 Role => ELED4 */
    LED_PORT0_ROLE      = ELED4,
#else
    LED_PORT0_ROLE      = ELED1,
#endif  /* __AUTHENTICATION__ */

    LED_PORT1_CC        = ELED6,
    LED_PORT1_VBUS      = ELED5,
    LED_PORT1_ROLE      = ELED4
  } USBPD_BSP_Led_TypeDef;

#define LED_PORT_ROLE(__PORT__) (__PORT__ == USBPD_PORT_0 ? LED_PORT0_ROLE : LED_PORT1_ROLE)  
#define LED_PORT_CC(__PORT__)   (__PORT__ == USBPD_PORT_0 ? LED_PORT0_CC : LED_PORT1_CC)
#define LED_PORT_VBUS(__PORT__) (__PORT__ == USBPD_PORT_0 ? LED_PORT0_VBUS : LED_PORT1_VBUS) 

#define DRP_pin                                 GPIO_PIN_8
#define DRP_port                                GPIOA


/**
  * @}
  */ 

#define BUTTONn                            1

/**
  * @brief User push-button
  */
#define USER_BUTTON_PIN                         GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT                   GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()   
#define USER_BUTTON_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()  
#define USER_BUTTON_EXTI_LINE                   GPIO_PIN_13
#define USER_BUTTON_EXTI_IRQn                   EXTI15_10_IRQn
/* Aliases */
#define KEY_BUTTON_PIN                        USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT                  USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()          USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()         USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_LINE                  USER_BUTTON_EXTI_LINE
#define KEY_BUTTON_EXTI_IRQn                  USER_BUTTON_EXTI_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == 0) USER_BUTTON_GPIO_CLK_ENABLE();} while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0)
/**
  * @}
  */ 

typedef enum 
{  
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY  = BUTTON_USER
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 
  
/* Exported define -----------------------------------------------------------*/
/** @defgroup P-NUCLEO-USB002_Exported_Constants Exported Constants     */

#if (USBPD_PORT_COUNT == 1)
#define HW_IF_ADC_MASTER        ADC2
#elif (USBPD_PORT_COUNT == 2)
#define HW_IF_ADC_MASTER       ADC1
#define HW_IF_ADC_SLAVE        ADC2
#endif

/* ADC Exported define for MB1303 board */
#if (USBPD_PORT_COUNT == 1)
#define USBPD_ADCn    2                                    /*!< Number of Analog channel used by ADC */
#define USBPD_ADC_INT_CHANNEL    0          /*!< Number of internal channel used */     
#define ADCCONVERTEDVALUES_BUFFER_SIZE    USBPD_ADCn + USBPD_ADC_INT_CHANNEL /*!< Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */
#define VBUS_INDEX(__PORT__)    ( 0 )                      /*!< Index of the VBus channel for the specified Port inside @ref ADCxConvertedValues */
#define IBUS_INDEX(__PORT__)    ( 1 )                      /*!< Index of the IBus channel for the specified Port inside @ref ADCxConvertedValues */
//#define VSRC_INDEX(__PORT__)    ( 2 )
#elif (USBPD_PORT_COUNT == 2)
#define USBPD_ADCn    4                                     /*!< Number of Analog channel used by ADC */
#define USBPD_ADC_INT_CHANNEL    0          /*!< Number of internal channel used */     
#define ADCCONVERTEDVALUES_BUFFER_SIZE 6   /*!< Size of array containing ADC converted values: stm32G4 has slave/master ADC. in this conf 3 channels of slave + 1 channel of master. Need to have same number of channels means 3+3 (and not 4) */
#define VBUS_INDEX(__PORT__)    ((__PORT__ == 0) ? 1 : 0)  /*!< Index of the VBus channel for the specified Port inside @ref ADCxConvertedValues */
#define IBUS_INDEX(__PORT__)    ((__PORT__ == 0) ? 3 : 5)  /*!< Index of the IBus channel for the specified Port inside @ref ADCxConvertedValues */
//#define VSRC_INDEX(__PORT__)     (4)
#endif


#if defined (P_NUCLEO_G4_C)
ADC_HandleTypeDef  hadc_master;           /*!< Handle of ADC peripheral */
#if (USBPD_PORT_COUNT == 2)
ADC_HandleTypeDef  hadc_slave;           /*!< Handle of ADC peripheral */
#endif
DMA_HandleTypeDef   DmaHandle;              /*!< Handle of DMA peripheral */
#if (USBPD_PORT_COUNT == 1)
uint32_t ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE]; /*!< Array that stores ADC sampled values */
#else
__IO uint16_t ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE]; /*!< Array that stores ADC sampled values */
#endif
#else 

extern ADC_HandleTypeDef  hadc_master;           /*!< Handle of ADC peripheral */
#if (USBPD_PORT_COUNT == 2)
extern ADC_HandleTypeDef  hadc_slave;           /*!< Handle of ADC peripheral */
#endif
extern DMA_HandleTypeDef   DmaHandle;              /*!< Handle of DMA peripheral */

#if (USBPD_PORT_COUNT == 1)
extern uint32_t ADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE]; /*!< Array that stores ADC sampled values */
#else
extern __IO uint16_t ADCxConvertedValues[]; /*!< Array that stores ADC sampled values */
#endif 
#endif 

static const USBPD_BSP_GPIOADCPins_TypeDef USBPD_ADCs[USBPD_ADCn] =
{ 
#if (USBPD_PORT_COUNT == 1)
  USBPD_BSP_ADC(GPIOC, 4,&hadc_master ,ADC_CHANNEL_5,ADC_REGULAR_RANK_1), /*ADC0 on schematic - VBUS port0  (ADC 2 in5)*/
  USBPD_BSP_ADC(GPIOA, 7,&hadc_master , ADC_CHANNEL_4,ADC_REGULAR_RANK_2),  /*ADC1 on schematic - IBUS port0 (ADC 2 in4)*/
//  USBPD_BSP_ADC(GPIOC, 0,&hadc_master , ADC_CHANNEL_6,ADC_REGULAR_RANK_3),  /*ADC4 on schematic - PowConn17 (ADC 2 in6)*/
  
#elif (USBPD_PORT_COUNT == 2)
  USBPD_BSP_ADC(GPIOC, 4,&hadc_slave , ADC_CHANNEL_5,ADC_REGULAR_RANK_1), /*ADC0 on schematic - VBUS port0 (ADC 2 in5)*/
  USBPD_BSP_ADC(GPIOA, 7,&hadc_slave , ADC_CHANNEL_4,ADC_REGULAR_RANK_2),  /*ADC1 on schematic - IBUS port0 (ADC 2 in4)*/

  USBPD_BSP_ADC(GPIOB, 0,&hadc_master , ADC_CHANNEL_15,ADC_REGULAR_RANK_1),  /*ADC2 on schematic - VBUS port1 (ADC 2 in15)*/
  USBPD_BSP_ADC(GPIOA, 4,&hadc_slave , ADC_CHANNEL_17,ADC_REGULAR_RANK_3),  /*ADC3 on schematic - IBUS port1 (ADC 1 in15)*/
//  USBPD_BSP_ADC(GPIOC, 0,&hadc_slave , ADC_CHANNEL_6 ,ADC_REGULAR_RANK_5),  /*ADC4 on schematic - PowConn17*/
#endif

} ;
/* Macro for ADCx DMA resources */
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC12_CLK_ENABLE()         /*!< Enables clock of ADC device  */
#define ADCx_CLK_DISABLE()              __HAL_RCC_ADC12_CLK_DISABLE()        /*!< Disables clock of ADC device  */
#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()        /*!< Forces reset of ADC device   */
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()      /*!< Releases reset of ADC device   */
#define ADCx_DMA_CLK_ENABLE()           __HAL_RCC_DMA1_CLK_ENABLE()         /*!< Enables clock of DMA device   */
#define ADCx_DMAMUX_CLK_ENABLE()        __HAL_RCC_DMAMUX1_CLK_ENABLE()
#if (USBPD_PORT_COUNT == 1)
#define ADCx_DMA                        DMA1_Channel7                       /*!< DMA channel for ADC       */
#define ADCx_DMA_IRQn                   DMA1_Channel7_IRQn
#define ADCx_DMA_IRQHandler             DMA1_Channel7_IRQHandler
#define ADCx_DMA_REQUEST                DMA_REQUEST_ADC2
#elif (USBPD_PORT_COUNT == 2)
#define ADCx_DMA                        DMA1_Channel6                       /*!< DMA channel for ADC       */
#define ADCx_DMA_IRQn                   DMA1_Channel6_IRQn
#define ADCx_DMA_IRQHandler             DMA1_Channel6_IRQHandler
#define ADCx_DMA_REQUEST                DMA_REQUEST_ADC1
#endif


#define TEMP(X)     ( ( ((uint32_t)X)*1024 )>>10 ) 
#define MVOLTAGE(X)  ( ( ( ( (uint32_t)X) * 3300 * (82+15)) >> 12 ) / 15 ) 

#define MVOLTVSRC(X)  ( ( ( ( (uint32_t)X) * 11 * 3300 ) ) >> 12 )
#define    codemax  (uint16_t) (4096*225/330)
#define MAMPIBUS(X)           (uint32_t) (X * 3000  / codemax)
#define MVOLTVINADC(X) ( ( ( ( (uint32_t)X) * 20151 ) ) >> 12 )
/**
 * 50Vs - VDDA/2 = Vadcin
 * */
#define MAMP(X)    (( (int32_t)((X*3300)>>10)-6600 ))      /*!< Macro to convert ADC value into mA  */

#if defined(USBPD_CLI)
#if defined (HAL_UART_MODULE_ENABLED) 
#if defined (P_NUCLEO_G4_C)
UART_HandleTypeDef huart_handle;
void USBPD_BSP_UART_Init(void);
#else 
extern UART_HandleTypeDef huart_handle;
extern void USBPD_BSP_UART_Init(void);
#endif 
#endif /* HAL_UART_MODULE_ENABLED */
#define USBPD_BSP_USART_IRQHandler              USART3_IRQHandler
#endif /* USBPD_CLI */

#define USBPD_BSP_LEDn                          7
#define USBPD_BSP_LED_LEN                       USBPD_BSP_LEDn

/**
  * @}
  */

#define USBPD_POWSELn    ((USBPD_PORT_COUNT) * 2)

static const USBPD_BSP_GPIOPins_TypeDef USBPD_POWSELs[USBPD_POWSELn] =
{
  USBPD_BSP_PIN(GPIOB,7),
  USBPD_BSP_PIN(GPIOB,6),
#if USBPD_PORT_COUNT == 2
  USBPD_BSP_PIN(GPIOF,0),
  USBPD_BSP_PIN(GPIOF,1),
#endif
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
    
/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup P-NUCLEO-USB002_Exported_Functions
  * @{
  */
#if defined(_GUI_INTERFACE)
const uint8_t*          BSP_GetHWBoardVersionName(void);
const uint8_t*          BSP_GetPDTypeName(void);
#endif /* _GUI_INTERFACE */

    /**
      * @brief  Configures P_NUCLEO_USB002 LED GPIO.
      * @param  None
      * @retval None
      */
    void USBPD_BSP_LED_Init(void);

    /**
      * @brief  Turns selected LED On.
      * @param  Led: Specifies the Led to be set on.
      * @retval None
      */
    void USBPD_BSP_LED_On(USBPD_BSP_Led_TypeDef Led);

    /**
      * @brief  Turns selected LED On or Off.
      * @param  Led: Specifies the Led to be set on.
      * @param  Value: value to set the led on or off.
      * @retval None
      */
    void USBPD_BSP_LED_Set(USBPD_BSP_Led_TypeDef Led, uint8_t Value);    
    
    /**
      * @brief  Turns selected LED Off.
      * @param  Led: Specifies the Led to be set off.
      * @retval None
      */
    void USBPD_BSP_LED_Off(USBPD_BSP_Led_TypeDef Led);

    /**
      * @brief  Toggles the selected LED.
      * @param  Led: Specifies the Led to be toggled.
      * @retval None
      */
    void USBPD_BSP_LED_Toggle(USBPD_BSP_Led_TypeDef Led);
    HAL_StatusTypeDef APPLI_Set_Current_Limit(uint8_t PortNum ,uint32_t IbusInmA);

void      BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void      BSP_PB_DeInit(Button_TypeDef Button);
uint32_t  BSP_PB_GetState(Button_TypeDef Button);
    


/* STUSB16xx_EVAL exported function prototypes */
void            HW_IF_ADC_Init(void);
void            HW_IF_ADCAnalogGPIO_Init(void);
void            HW_IF_ADCAnalogGPIO_DeInit(void);
void            HW_IF_ADCDMA_Init(void);
void            HW_IF_ADCDMA_DeInit(void);



#ifndef MB1303
uint16_t APPLI_GetTemp(uint8_t PortNum);
uint16_t APPLI_GetVsrc(uint8_t PortNum);
#endif /*MB1303*/
int16_t APPLI_GetIvbus(uint8_t PortNum);
uint16_t APPLI_GetVBUS(uint8_t PortNum);
uint16_t APPLI_GetVINADC(uint8_t PortNum);

HAL_StatusTypeDef APPLI_SetVoltage(uint8_t PortNum ,uint32_t vbus_vvar);
void BSP_USBPD_SetVoltage(uint32_t Port, uint32_t Voltage);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#else
/*#warning "P-Nucleo header eof2"*/
#endif /* __P_NUCLEO_USB_G4_H_ */  
/*#warning "P-Nucleo header eof"*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
