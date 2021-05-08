/**
******************************************************************************
* @file    p-nucleo-F446.c
* @author  System Lab
* @brief   This file provides set of functions to manage peripherals on
*          P_NUCLEO_USB002 board.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Unless required by applicable law or agreed to in writing, software 
* distributed under the License is distributed on an "AS IS" BASIS, 
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
******************************************************************************
*/
#ifndef P_NUCLEO_G4_C
#define P_NUCLEO_G4_C 

/* Includes ------------------------------------------------------------------*/ 
#include "User_BSP.h"

#if _ADC_MONITORING
extern USBPD_HandleTypeDef DPM_Ports[USBPD_PORT_COUNT];
#endif
/**
* @addtogroup P_NUCLEO_USB002
* @{
* */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup P-NUCLEO-USB002
  * @{
  */   

/** @defgroup P-NUCLEO-USB002_Private_Defines Private Defines
  * @{
  */

#if defined(USBPD_CLI)
#define BAUDRATE	115200		/* BaudRate for the UART */

/**
* @brief Usart used by P_NUCLEO_USB002
* */
#define USBPD_BSP_USART			  USART3

/**
* @brief Clock Enable Macro
* */
#define USBPD_BSP_USARTCLK_ENABLE	        __HAL_RCC_USART3_CLK_ENABLE

/**
* @brief PIN RELATED MACROS
* @{
* */
#define USART_TX_PORT			        GPIOD
#define USART_TX_PIN			        GPIO_PIN_8
#define USART_RX_PORT			        GPIOA
#define USART_RX_PIN			        GPIO_PIN_9
#define USART_PIN_GPIOAF		        GPIO_AF7_USART3
#define USART_IRQ			        USART3_IRQn
#endif /*defined(USBPD_CLI)*/
/**
* @}
* */

/* 
* NOTE: BAUDRATE, P_NUCLEO_USB002_USART have been defined but they haven't used yet.
*       usart.c and usart.h files have to be modified according to these definitions
*/

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup P-NUCLEO-USB002_Private_Variables Private Variables
  * @{
  */ 

/**
* @brief Vector storing informations on pins controlling leds of P_NUCLEO_USB002
* */

USBPD_BSP_GPIOPins_TypeDef USBPD_BSP_LEDs[USBPD_BSP_LEDn] =
{
  USBPD_BSP_PIN(GPIOA,5),           /* LED1: generic */
  
  USBPD_BSP_PIN(GPIOC,5),           /* LED01: PORT0_ROLE */
  USBPD_BSP_PIN(GPIOB,1),           /* LED02: PORT0_VBUS */
  USBPD_BSP_PIN(GPIOB,2),           /* LED03: PORT0_CCx  */
  
  USBPD_BSP_PIN(GPIOC,14),          /* LED11: PORT1_ROLE */
  USBPD_BSP_PIN(GPIOA,6),           /* LED12: PORT1_VBUS */
  USBPD_BSP_PIN(GPIOC,15)           /* LED13: PORT1_CCx  */
     
} ;

#if defined(_GUI_INTERFACE)
const uint8_t HWBoardVersionName[] = "P-Nucleo-F4";
#endif /* _GUI_INTERFACE */

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#if defined(_GUI_INTERFACE)
/**
  * @brief  This method returns HW board version name
  * @retval HW Board version name
  */
const uint8_t* BSP_GetHWBoardVersionName(void)
{
  return HWBoardVersionName;
}

/**
  * @brief  This method returns HW PD Type name
  * @retval HW Board version name
  */
const uint8_t* BSP_GetPDTypeName(void)
{
    return "MB1303B";
}
#endif /* _GUI_INTERFACE */

/* ************************************************************************* */
/* Led Procedures and functions                                              */

/**
* @brief  Configures P_NUCLEO_USB002 LED GPIO.
* @param  None
* @retval None
*/
void USBPD_BSP_LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  uint8_t led=0;
  
  /* Common values for Leds GPIO */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    
  
  for(led=0;led<USBPD_BSP_LEDn;led++)
  {
    /* Configure the GPIO pin */
    GPIO_InitStruct.Pin = USBPD_BSP_LEDs[led].GPIO_Pin;
    
    /* Init the associated GPIO */
    HAL_GPIO_Init(USBPD_BSP_LEDs[led].GPIOx, &GPIO_InitStruct);
    /* Turn the led off */
    USBPD_BSP_LED_Off((USBPD_BSP_Led_TypeDef)led);
  }

    GPIO_InitStruct.Pin = DRP_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DRP_port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DRP_port ,DRP_pin, GPIO_PIN_SET);

}

/**
* @brief  Turns selected LED On or Off.
* @param  Led: Specifies the Led to be set on.
* @param  Value: value to set the led on or off.
* @retval None
*/
void USBPD_BSP_LED_Set(USBPD_BSP_Led_TypeDef Led, uint8_t Value)
{
  HAL_GPIO_WritePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin, Value ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
* @brief  Turns selected LED On.
* @param  Led: Specifies the Led to be set on.
* @retval None
*/  
void USBPD_BSP_LED_On(USBPD_BSP_Led_TypeDef Led)
{
  HAL_GPIO_WritePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin, GPIO_PIN_RESET);
}

/**
* @brief  Turns selected LED Off.
* @param  Led: Specifies the Led to be set off.
* @retval None
*/  
void USBPD_BSP_LED_Off(USBPD_BSP_Led_TypeDef Led)
{
  HAL_GPIO_WritePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin, GPIO_PIN_SET);
}

/**
* @brief  Toggles the selected LED.
* @param  Led: Specifies the Led to be toggled.
* @retval None
*/ 
void USBPD_BSP_LED_Toggle(USBPD_BSP_Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin);
}


GPIO_TypeDef*  BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT};
const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN};
const uint8_t  BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn };  
/**
* @brief  Configures Button GPIO and EXTI Line.
* @param  Button: Specifies the Button to be configured.
*   This parameter should be: BUTTON_USER
* @param  ButtonMode: Specifies Button mode.
*   This parameter can be one of following parameters:   
*     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
*     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
*                            generation capability  
* @retval None
*/
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpioinitstruct;
  
  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);
  
  gpioinitstruct.Pin = BUTTON_PIN[Button];
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
  if(ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    gpioinitstruct.Mode = GPIO_MODE_INPUT;
    
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);
  }
  
  if(ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    gpioinitstruct.Mode = GPIO_MODE_IT_FALLING;
    
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x03, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
* @brief  Push Button DeInit.
* @param  Button: Button to be configured
*   This parameter should be: BUTTON_USER
* @note PB DeInit does not disable the GPIO clock
* @retval None
*/
void BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef gpio_init_structure;
  
  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);
}

/**
* @brief  Returns the selected Button state.
* @param  Button: Specifies the Button to be checked.
*   This parameter should be: BUTTON_USER
* @retval Button state.
*/
uint32_t BSP_PB_GetState(Button_TypeDef Button)
{
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}
/**
* @}
*/


#if defined(USBPD_CLI)
#if defined(HAL_UART_MODULE_ENABLED)
/**
* @brief  Configures the UART used by the P_NUCLEO_USB002
* @retval None
*/

void USBPD_BSP_UART_Init(void)
{
  /* Init huart_usbpdm1 */
  huart_handle.Instance = USBPD_BSP_USART;
  huart_handle.Init.BaudRate = BAUDRATE;
  huart_handle.Init.WordLength = UART_WORDLENGTH_8B;
  huart_handle.Init.StopBits = UART_STOPBITS_1;
  huart_handle.Init.Parity = UART_PARITY_NONE;
  huart_handle.Init.Mode = UART_MODE_TX_RX;
  huart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart_handle.Init.OverSampling = UART_OVERSAMPLING_16;
  huart_handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart_handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  /* Init of the peripheral */
  HAL_UART_Init(&huart_handle);
}

/*
* @brief Init the low level hardware : GPIO, CLOCK
* */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  /* Peripheral clock enable */
  USBPD_BSP_USARTCLK_ENABLE();
  
  /* USART GPIO Configuration */
  GPIO_InitStruct.Pin = USART_TX_PIN | USART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = USART_PIN_GPIOAF;
  HAL_GPIO_Init(USART_TX_PORT, &GPIO_InitStruct);
  
  /* Peripheral interrupt init*/
  HAL_NVIC_SetPriority(USART_IRQ, 3, 0);
  HAL_NVIC_EnableIRQ(USART_IRQ);
}
#endif /* HAL_UART_MODULE_ENABLED */
#endif   /*USBPD_CLI*/


void USBPD_HW_IF_GPIO_Set(USBPD_BSP_GPIOPins_TypeDef gpio, GPIO_PinState PinState)
{
  HAL_GPIO_WritePin(gpio.GPIOx, gpio.GPIO_Pin, PinState);
}
void USBPD_HW_IF_GPIO_On(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Sets the pin */
  USBPD_HW_IF_GPIO_Set(gpio, GPIO_PIN_SET);
}

void USBPD_HW_IF_GPIO_Off(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Resets the pin */
  USBPD_HW_IF_GPIO_Set(gpio, GPIO_PIN_RESET);
}

void USBPD_HW_IF_GPIO_Toggle(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Toggle the pin */
  HAL_GPIO_TogglePin(gpio.GPIOx, gpio.GPIO_Pin);
}


/** @addtogroup USBPD_DEVICE_PHY_HW_IF_Private_Functions USBPD DEVICE PHY HW IF Private functions
* @details Private functions can be used at hardware interface level
* @{
*/


/**
* @brief  ADC init function
* @param  None
* @retval None
*/
void HW_IF_ADC_Init(void)
{
  
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
//  ADC_InjectionConfTypeDef IConfig = {0};
  uint8_t ch = 0;
  
  
  /* Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) */
 
  hadc_master.Instance = HW_IF_ADC_MASTER;  /*ADC1 if #2 ADC2 if 1 */
  hadc_master.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc_master.Init.Resolution = ADC_RESOLUTION_12B;
  hadc_master.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc_master.Init.GainCompensation = 0;  
  hadc_master.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc_master.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  hadc_master.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  hadc_master.Init.ContinuousConvMode = ENABLE;
 #if (USBPD_PORT_COUNT == 2)
hadc_master.Init.NbrOfConversion = 3;
#else
  hadc_master.Init.NbrOfConversion = USBPD_ADCn + USBPD_ADC_INT_CHANNEL;
#endif
  hadc_master.Init.DiscontinuousConvMode = DISABLE;
  hadc_master.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc_master.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc_master.Init.DMAContinuousRequests = ENABLE;
  hadc_master.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
//  hadc_master.OversamplingMode = ENABLE;
//  hadc_master.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_2;
//  hadc_master.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
//  hadc_master.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
//  hadc_master.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  
  
  HAL_ADC_Init(&hadc_master);
  
  

#if (USBPD_PORT_COUNT == 2)
  hadc_slave.Instance = HW_IF_ADC_SLAVE; /*ADC1  that must become master */
  
  hadc_slave.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc_slave.Init = hadc_master.Init;
  hadc_master.Init.NbrOfConversion = 3 ; // max number channel of the 2 ADCs :1 on ADC1 3 on ADC2 //  USBPD_ADCn + USBPD_ADC_INT_CHANNEL;
  hadc_slave.Init.ContinuousConvMode    = DISABLE;
  hadc_slave.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  
  
  HAL_ADC_Init(&hadc_slave);  

  if (HAL_ADCEx_Calibration_Start(&hadc_slave, ADC_SINGLE_ENDED)  != HAL_OK )
  {
    while(1);
  }      
 
  
#endif 
  /* Configuration of the selected ADC regular channel to be converted */
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
 #if (USBPD_PORT_COUNT == 1)  
  for(ch=0;ch<(USBPD_ADCn - USBPD_ADC_INT_CHANNEL);ch++)
  {
    sConfig.Channel = USBPD_ADCs[ch].ADCCH;
    sConfig.Rank = USBPD_ADCs[ch].RANK;
    
    HAL_ADC_ConfigChannel(USBPD_ADCs[ch].hadc, &sConfig);// USBPD_ADCs[ch].hadc

  }
#endif
  if (USBPD_ADC_INT_CHANNEL != 0)
  {
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    HAL_ADC_ConfigChannel(USBPD_ADCs[ch].hadc, &sConfig);
  }
#if (USBPD_PORT_COUNT == 2)
  
    sConfig.Channel = USBPD_ADCs[2].ADCCH;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(&hadc_master, &sConfig);
    sConfig.Channel = USBPD_ADCs[2].ADCCH;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc_master, &sConfig);    
    sConfig.Channel = USBPD_ADCs[2].ADCCH;
    sConfig.Rank = ADC_REGULAR_RANK_3; 
    HAL_ADC_ConfigChannel(&hadc_master, &sConfig);    

    sConfig.Channel = USBPD_ADCs[0].ADCCH;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(&hadc_slave, &sConfig);
    sConfig.Channel = USBPD_ADCs[1].ADCCH;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc_slave, &sConfig);
    sConfig.Channel = USBPD_ADCs[3].ADCCH;
    sConfig.Rank = ADC_REGULAR_RANK_3;  
    HAL_ADC_ConfigChannel(&hadc_slave, &sConfig);    

    
  multimode.Mode = ADC_DUALMODE_REGSIMULT;//;ADC_DUALMODE_INTERL
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE ;
  
//#else
//    multimode.Mode = ADC_MODE_INDEPENDENT;
//#endif  

  if (HAL_ADCEx_MultiModeConfigChannel(&hadc_master, &multimode) != HAL_OK)
  {
    while(1);
  }
  
#endif 
  
#if (USBPD_PORT_COUNT == 1)
    if (HAL_ADCEx_Calibration_Start(&hadc_master, ADC_SINGLE_ENDED) != HAL_OK)
  {
 while(1);

  } 

  if (HAL_ADC_Start_DMA(&hadc_master,(uint32_t *) &ADCxConvertedValues[0], ADCCONVERTEDVALUES_BUFFER_SIZE ) != HAL_OK)
  {
     while(1);
  }
#elif (USBPD_PORT_COUNT == 2)  
  
  if (HAL_ADCEx_MultiModeStart_DMA(&hadc_master,(uint32_t *) &ADCxConvertedValues[0], ADCCONVERTEDVALUES_BUFFER_SIZE) != HAL_OK)
  {
    while(1);
  }   
#endif
 // HAL_ADC_Start(&hadc_master);

}

/**
* @brief  Initialization of ADC analog GPIOs
* @retval None
*/
void HW_IF_ADCAnalogGPIO_Init(void)
{
  GPIO_InitTypeDef      GPIO_InitStruct;
  uint8_t ch = 0;
  
  /* Configure all GPIO port pins in Analog mode */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  
  for(ch=0;ch<USBPD_ADCn;ch++)
  {
    GPIO_InitStruct.Pin = USBPD_ADCs[ch].GPIO_Pin;
    HAL_GPIO_Init(USBPD_ADCs[ch].GPIOx, &GPIO_InitStruct);
  }
}


/**
* @brief  Deinitialization of ADC analog GPIOs.
* @retval None
*/
void HW_IF_ADCAnalogGPIO_DeInit(void)
{
  uint8_t ch = 0;
  /* De-initialize GPIO pin of the selected ADC channel */
  
  for(ch=0;ch<USBPD_ADCn;ch++)
  {
    HAL_GPIO_DeInit(USBPD_ADCs[ch].GPIOx, USBPD_ADCs[ch].GPIO_Pin);
  }
}


/**
* @brief  Initialization of the ADC DMA
* @retval None
*/
void HW_IF_ADCDMA_Init(void)
{
  /* Configuration of DMA parameters */
  ADCx_CLK_ENABLE() ;
  ADCx_DMA_CLK_ENABLE() ;
  ADCx_DMAMUX_CLK_ENABLE();
  
#if (USBPD_PORT_COUNT == 1)  
  DmaHandle.Instance = ADCx_DMA;
//  DmaHandle.Init.Channel = DMA_CHANNEL_0;
#if defined(DMAMUX_CxCR_DMAREQ_ID)  
  DmaHandle.Init.Request = ADCx_DMA_REQUEST;
#endif 
#elif (USBPD_PORT_COUNT == 2)
  DmaHandle.Instance = ADCx_DMA;
//  DmaHandle.Init.Channel = DMA_CHANNEL_0;
#if defined(DMAMUX_CxCR_DMAREQ_ID)  
  DmaHandle.Init.Request = ADCx_DMA_REQUEST;
#endif 
#endif  
  DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
  DmaHandle.Init.MemInc = DMA_MINC_ENABLE;
  DmaHandle.Init.Mode = DMA_CIRCULAR;
  DmaHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
  
  
  
  /* Initialization of the DMA associated to the peripheral */

  HAL_DMA_Init(&DmaHandle);
  

  /* Association of the initialized DMA handle to the ADC handle */
  __HAL_LINKDMA(&hadc_master, DMA_Handle, DmaHandle);
 
}


/**
* @brief  Deinitialization of the ADC DMA
* @retval None
*/
void HW_IF_ADCDMA_DeInit(void)
{
  /* Deinitialization of the DMA associated to the peripheral */
  if(hadc_master.DMA_Handle != NULL)
  {
    HAL_DMA_DeInit(hadc_master.DMA_Handle);
  }

}


void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  /* Enable clock of ADCx peripheral */
  ADCx_CLK_ENABLE();
  HW_IF_ADCAnalogGPIO_Init();
  
  /* Enable clock of DMA associated to the peripheral */
  ADCx_DMA_CLK_ENABLE();
  HW_IF_ADCDMA_Init();
}

/**
  * @brief ADC MSP De-initialization
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  /* Reset peripherals */
    /* Peripheral clock disable */
  ADCx_CLK_DISABLE();
  /* De-initialize the GPIO pins of the selected ADC */
  HW_IF_ADCAnalogGPIO_DeInit();
  /* De-Initialize the DMA related to the ADC */
  HW_IF_ADCDMA_DeInit();
}

/*
*  }@ 
*/
/**
* define temperature calculation based on appli
*/



#ifndef MB1303
/**
* @brief  Get temp value from ADC.
* @param  PortNum.
* @param  Value: value to set the GPO on or off.
* @retval None
*/
uint16_t APPLI_GetTemp(uint8_t PortNum)
{
#warning "[NB] need to adapt TEMP calculation vs application to get correct temparature info"
  return (uint16_t)TEMP( ADCxConvertedValues[TEMP_INDEX(PortNum)] );
}

uint16_t Vsrc ; 
uint16_t APPLI_GetVsrc(uint8_t PortNum)
{
#warning "[NB] Vsrc measurement not used actually"
  Vsrc = (uint16_t)MVOLTAGE( (ADCxConvertedValues[VSRC_INDEX(PortNum)])  );
  return Vsrc;
}
#endif
uint8_t vbus_index,vsrc_index, vin_index;
uint32_t vbus_board[USBPD_PORT_COUNT];

uint16_t APPLI_GetVBUS(uint8_t PortNum)
{
#if defined (_ADC_MONITORING)
  vbus_index = VBUS_INDEX(PortNum);
  vbus_board[PortNum] = (uint16_t)MVOLTAGE( ADCxConvertedValues[VBUS_INDEX(PortNum)] );
  return vbus_board[PortNum];
#else 
  /* check if Vbus is within Valid range */
  /* USBPD_HW_IF_CheckVbusValid should be better but during HWreset flag is not updated */
  if(USBPD_HW_IF_CheckVbusValid(PortNum ,10) == USBPD_OK)
  {
#ifdef _DEBUG_TRACE
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "valid range", sizeof("valid range"));
#endif
  return (uint16_t)(STUSB1602_VBUS_Select_Status_Get(STUSB1602_I2C_Add(PortNum)));
}
else 
{  /* check is Vbus is bellow 0.8 V */
if (USBPD_HW_IF_CheckVbusVSafe0V(PortNum, 10) == USBPD_OK )
{
#ifdef _DEBUG_TRACE
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "valid vsafe0", sizeof("valid vsafe0"));
#endif
  return 600;
}

  else 
  {
    if (USBPD_HW_IF_CheckVBusPresence(PortNum ,0) == USBPD_OK)
    {
      USBPD_HW_IF_HR_End(PortNum, USBPD_PORTPOWERROLE_SNK);
    }
  #ifdef _DEBUG_TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "valid range KO", sizeof("valid range KO"));
  #endif
    return 3000;
  }
}       
#endif   
  
}

HAL_StatusTypeDef APPLI_Set_Current_Limit(uint8_t PortNum ,uint32_t IbusInmA) 
{
  return HAL_OK;
} 

uint16_t Ibus[USBPD_PORT_COUNT];

int16_t APPLI_GetIvbus(uint8_t PortNum)
{
  /*  #warning "[NB] Ivbus measurement not used actually"*/
  int16_t signed_current;
#if _ADC_MONITORING
  signed_current = (int16_t)(MAMP( ADCxConvertedValues[IBUS_INDEX(PortNum)] ));
#else
  signed_current = 100;
#endif

  if (signed_current < 0)
    return (- signed_current);
  else
    return signed_current;
}


HAL_StatusTypeDef APPLI_SetVoltage(uint8_t PortNum ,uint32_t vbus_vvar) 
{
   HAL_StatusTypeDef ret = HAL_OK;

  /* configuration for STCH2 Board, enabled only profile 5V and 9V
   * Connections
   * +----------------+-------------------------------------------+
   * |   J4 STCH2     |     C4 MB1303/MB1257                      |
   * +----+-----------+----+--------------------------------------+
   * |PIN#| PIN Name  |PIN#|PIN Name                              |
   * +----+-----------+----+--------------------------------------+
   * | 1  | Vout      |1-3 |POWCNN1 (VBus)                        |
   * | 2  | SEL1      |9   |POWCNN9 / CN7.21 (PB7)    (open drain)|
   * | 3  | SEL2      |11  |POWCNN11 / CN10.17 (PB6)  (open drain)|
   * | 4  | GND       |5-7 |GND                                   |
   * +----+-----------+----+--------------------------------------+
   *
   * STCH2 Voltage Output
   * +------+------+------+
   * | SEL2 | SEL1 | VOUT |
   * +------+------+------+
   * |  0   |  0   |  5V  |
   * |  0   |  1   |  9V  |
   * |  1   |  -   | 12V  |
   * +------+------+------+
   */

  uint32_t offset_port = (PortNum == 0) ? 0 : 2;


#ifdef CONF_NORMAL
  /* force low both sel pins */
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);
  USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);
#endif

  // STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);
  /* VBUS_Monitoring functional adjust monitoring and set Vbus output target */
  if (5000 == vbus_vvar)
  {
      (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, vbus_vvar+100, 5, 10);
   ret = (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, vbus_vvar, 5, 10);
#if USBPD_USBDATA
extern USBPD_ParamsTypeDef DPM_Params[USBPD_PORT_COUNT];
    if (USBPD_PORTPOWERROLE_SNK == DPM_Params[PortNum].PE_PowerRole)
   {
         STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);
         STUSB1602_VBUS_Presence_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Presence_Disable);
#ifdef _DEBUG_TRACE
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "VBUS_Range_Disable 5V", sizeof("VBUS_Range_Disable 5V"));
#endif
   }
   else
   {
       (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, vbus_vvar+100, 10, 10);
  ret= (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, vbus_vvar, 10, 10);
       STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
       STUSB1602_VBUS_Presence_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Presence_Enable);
#ifdef _DEBUG_TRACE
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "VBUS_Range_enable1", sizeof("VBUS_Range_enable1"));
#endif
   }
  }
  else /* VBUS above 5 V */
  {
      (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, vbus_vvar+100, 10, 10);
  ret= (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, vbus_vvar, 10, 10);
   STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
   STUSB1602_VBUS_Presence_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Presence_Enable);
#ifdef _DEBUG_TRACE
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "VBUS_Range_enable2", sizeof("VBUS_Range_enable2"));
#endif
//   ret = (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, vbus_vvar, 10, 10);
  }
#else
  }
   else /* VBUS above 5 V */
   {
   ret = (HAL_StatusTypeDef)STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, vbus_vvar, 10, 10);
   }
#endif
//  ret=(HAL_StatusTypeDef)STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum),vbus_vvar);
#ifdef _GPIO_FOR_SRC
   /* force low both sel pins */
  if(vbus_vvar == 15000)
  {
    USBPD_HW_IF_GPIO_On(USBPD_POWSELs[offset_port]);
    USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port+1]);   
  }
  else if(vbus_vvar == 9000)
  {
    USBPD_HW_IF_GPIO_Off(USBPD_POWSELs[offset_port]);
    USBPD_HW_IF_GPIO_On(USBPD_POWSELs[offset_port+1]);   
  }
  else
  {
    USBPD_HW_IF_GPIO_On(USBPD_POWSELs[offset_port]);
    USBPD_HW_IF_GPIO_On(USBPD_POWSELs[offset_port+1]); 
  }
#endif
  return ret;
 
}

uint32_t vin_board,code_adc;
#ifndef MB1303

uint16_t APPLI_GetVINADC(uint8_t PortNum)
{
  vin_index= VINADC_INDEX(PortNum);
  vin_board = (uint16_t)MVOLTAGE( ADCxConvertedValues[VINADC_INDEX(PortNum)] );
#warning "[NB] Vbus measurement not used actually"
  return (uint16_t)MVOLTAGE( ADCxConvertedValues[VINADC_INDEX(PortNum)] );
}

#endif
#undef P_NUCLEO_G4_C
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
