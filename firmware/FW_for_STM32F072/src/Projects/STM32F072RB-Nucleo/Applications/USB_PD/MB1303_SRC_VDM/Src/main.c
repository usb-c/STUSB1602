/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   USBPD demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "User_BSP.h"
#include "usbpd_dpm_user.h"

#ifdef USBPD_TERMINAL_OUTPUT
#define TERM_OUTPUT_MAX_SIZE 100 
char TerminalOutputbuffer[TERM_OUTPUT_MAX_SIZE];
void USBPD_TerminalOutput_Async_Notify(char *string);
void USBPD_TerminalOutput_Async_PrintBuffer(uint16_t Length, uint8_t * pData);
#endif /* USBPD_TERMINAL_OUTPUT */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined(_OPTIM_CONSO)
#define _HSE_ENABLE 0
#else
    #define _HSE_ENABLE 1
#endif /* _OPTIM_CONSO_ */


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
#if defined(_OPTIM_CONSO)
void     Configure_RTC(void);
#endif /* _OPTIM_CONSO */

/* Private functions ---------------------------------------------------------*/
void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName );

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName )
{
  while(1);
} 

/**
  * @brief  Main program
  * @retval None
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize BSP functionalities */
  USBPD_BSP_LED_Init();
  BSP_PB_Init(BUTTON_USER , BUTTON_MODE_EXTI);
  /* Global Init of USBPD HW */
  USBPD_HW_IF_GlobalHwInit();


#if defined(_OPTIM_CONSO)
  /* Configure RTC to use WUT */
  Configure_RTC();
#endif /* _OPTIM_CONSO */


  /* Initialize the Device Policy Manager */
  if( USBPD_OK != USBPD_DPM_InitCore())
  {
    /* error on core init  */
    while(1);
  }


  /* Initialise the DPM application */
  if (USBPD_OK != USBPD_DPM_UserInit())
  {
    while(1);
  }

  /* Initialize the Device Policy Manager */
  if( USBPD_ERROR == USBPD_DPM_InitOS())
  {
    /* error the OS init  */
    while(1);
  }

  USBPD_DPM_Run();
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI48)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 48000000
  *            PREDIV                         = 2
  *            PLLMUL                         = 2
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Set FLASH latency */ 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);


#if defined(STM32F072xB)
#if _HSE_ENABLE
  /* Enable HSE and wait for activation*/
  LL_RCC_HSE_EnableBypass(); 
  LL_RCC_HSE_Enable(); 
  while(LL_RCC_HSE_IsReady() != 1) 
  {
  };  
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLL_MUL_12, LL_RCC_PREDIV_DIV_2); 
#else
  /* Enable HSI48 and wait for activation*/
  LL_RCC_HSI48_Enable(); 
  while(LL_RCC_HSI48_IsReady() != 1) 
  {
  };
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI48, LL_RCC_PLL_MUL_2, LL_RCC_PREDIV_DIV_2);
#endif
#elif defined (STM32F051x8)
  
  /* Enable HSI48 and wait for activation*/
  LL_RCC_HSI_Enable(); 
  while(LL_RCC_HSI_IsReady() != 1) 
  {
  };
  
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
#endif


#if defined(_OPTIM_CONSO)
#elif !defined (STM32F051x8)
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

#endif /* _OPTIM_CONSO_ */

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
#if defined(_OPTIM_CONSO)
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI48);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI48)
#else
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
#endif /* _OPTIM_CONSO_ */
  {
  };
  
  /* Set APB1 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  
#if 0
  /* HSI48 used as USB clock source */
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_HSI48);
  
  /*Configure the clock recovery system (CRS)**********************************/
  /*Enable CRS Clock*/
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CRS);
  
  /* HSI48 Synchronization with synchronization frequency (source set to USB SOF)  */
  /* Set the TRIM[5:0] to the default value : HSI48CalibrationValue = 0x20 
     Default Synchro Signal division factor (not divided) : RCC_CRS_SYNC_DIV1
     Synchro Signal source set to USB SOF 
     HSI48 is synchronized with USB SOF at 1KHz rate
  */
  LL_CRS_ConfigSynchronization(0x20 << CRS_CR_TRIM_Pos,   /* Tmp patch waiting for STM32F0_HAL #419318 */
                               LL_CRS_ERRORLIMIT_DEFAULT,
                               __LL_CRS_CALC_CALCULATE_RELOADVALUE(HSI48_VALUE, 1000),
                               LL_CRS_SYNC_DIV_1 | LL_CRS_SYNC_SOURCE_USB | LL_CRS_SYNC_POLARITY_FALLING
                              );
  
  /* Start automatic synchronization */ 
  /* Enable Automatic trimming & Frequency error counter */
  LL_CRS_EnableFreqErrorCounter();
  LL_CRS_EnableAutoTrimming();
#endif
  
  /* Set systick to 1ms in using frequency set to 48MHz */
  /* This frequency can be calculated through LL RCC macro */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI48_VALUE, LL_RCC_PLL_MUL_2, LL_RCC_PREDIV_DIV_2) */
  LL_Init1msTick(48000000);
#if defined(_OPTIM_CONSO)
#else
  SysTick->CTRL|= SysTick_CTRL_TICKINT_Msk;
  HAL_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY ,0U);
#endif /* _OPTIM_CONSO */
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);
}

#if defined(_OPTIM_CONSO)
/**
  * Brief   This function configures RTC.
  * Param   None
  * Retval  None
  */
void Configure_RTC(void)
{
  /*##-1- Enables the PWR Clock and Enables access to the backup domain #######*/
  /* To change the source clock of the RTC feature (LSE, LSI), you have to:
     - Enable the power clock
     - Enable write access to configure the RTC clock source (to be done once after reset).
     - Reset the Back up Domain
     - Configure the needed RTC clock source */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_EnableBkUpAccess();
  
  /*##-2- Configure LSI as RTC clock source ###############################*/
  /* Enable LSI */
  LL_RCC_LSI_Enable();
  while (LL_RCC_LSI_IsReady() != 1)
  {
  }
  LL_RCC_ForceBackupDomainReset();
  LL_RCC_ReleaseBackupDomainReset();
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  
  /*##-3- Enable RTC peripheral Clocks #######################################*/
  /* Enable RTC Clock */ 
  LL_RCC_EnableRTC();

  /*##-4- Configure RTC ######################################################*/
  /* Disable RTC registers write protection */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Disable wake up timer to modify it */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* Wait until it is allow to modify wake up reload value */

  while (LL_RTC_IsActiveFlag_WUTW(RTC) != 1)
  {
  }
  
  /* RTCCLK = LSIFreq = 40kHz 
     - Step of 50us if 20Khz (RTCCLK_DIV2)
     - RTC_WUT_TIME = ((1000 (1ms) / 50) - 1) = 19*/
  LL_RTC_WAKEUP_SetAutoReload(RTC, 19);
  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_DIV_2);
  
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_20);
  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_20);
  
  LL_RTC_EnableIT_WUT(RTC);
  LL_RTC_WAKEUP_Enable(RTC);

  /* Enable RTC registers write protection */
  LL_RTC_EnableWriteProtection(RTC);
  
  HAL_NVIC_SetPriority(RTC_IRQn, USBPD_LOWEST_IRQ_PRIO, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
}

extern void vApplicationIdleHook( void );
extern volatile uint32_t FlagExplicitContract;

/* Function call in idle mode by FreeRTOS */
void vApplicationIdleHook( void )
{
  #if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */
  
  /* Allow to switch to 8 MHz only when there are no more AMS ongoing */
  if (FlagExplicitContract == 2)
  {
    /* System should be interupted during switch to 8MHz (possible 
       if PD message occurs (through TIM RX IT) */
    __disable_irq();
    SystemClock_Config_8Mhz();
    __enable_irq();
  }
    
  if (LL_RCC_HSI48_IsReady() == 0)
  {
    /* 
        System is at 8MHz, no PD negociation ongoing.
        Can enter in STOP mode
    */
    /* Set STOP_LPREGU mode when CPU enters deepsleep */
    LL_PWR_SetPowerMode(LL_PWR_MODE_STOP_LPREGU);

    /* Set SLEEPDEEP bit of Cortex System Control Register */
    LL_LPM_EnableDeepSleep();  

    /* Request Wait For Interrupt */
    __WFI();
    
    /* Need to restart ADC as automatically stopped in STOP mode*/
    if (LL_ADC_IsEnabled( HW_IF_ADC) == 0)
    {
      /* Enable ADC */
      LL_ADC_Enable(HW_IF_ADC);
      
      /* Poll for ADC ready to convert */
      #if (USE_TIMEOUT == 1)
      Timeout = ADC_ENABLE_TIMEOUT_MS;
      #endif /* USE_TIMEOUT */
      
      while (LL_ADC_IsActiveFlag_ADRDY(HW_IF_ADC) == 0)
      {
      #if (USE_TIMEOUT == 1)
        /* Check Systick counter flag to decrement the time-out value */
        if (LL_SYSTICK_IsActiveCounterFlag())
        {
          if(Timeout-- == 0)
          {
          /* Time-out occurred. Set LED to blinking mode */
          LED_Blinking(LED_BLINK_ERROR);
          }
        }
      #endif /* USE_TIMEOUT */
      }
    }
    
    /* Start ADC group regular conversion */
    LL_ADC_REG_StartConversion(HW_IF_ADC);
  }
  else
  {
    /* 
        System is at 48MHz, PD negociation ongoing.
        Can enter in SLEEP mode
    */
    LL_LPM_EnableSleep();
    
    /* Request Wait For Interrupt */
    __WFI();
  }
}

/* Change from 48 to 8 Mhz*/
void SystemClock_Config_8Mhz(void)
{
  /* HSI configuration and activation */
  /* Reset Value is HSI enabled */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1) 
  {
  };
  
  /* Sysclk activation on the HSI */
  /* Reset Value is Sysclk activated on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) 
  {
  };
  
  /* Set AHB & APB1 prescaler */
  /* Reset Value is AHB & APB1 prescaler DIV1 */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  /* Set systick to 1ms in using frequency set to 8MHz */
  LL_Init1msTick(8000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  /* Reset Value is SystemCoreClock at 8Mhz */
  LL_SetSystemCoreClock(8000000);

  /* Set FLASH latency */ 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  
  /* Disable HSI48 and wait for de-activation*/
  LL_RCC_HSI48_Disable(); 
  while(LL_RCC_HSI48_IsReady() != 0) 
  {
  };
}

/* Change from 8 to 48 Mhz*/
void SystemClock_Config_48Mhz(void)
{
  /* Set FLASH latency */ 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Enable HSI48 and wait for activation*/
  LL_RCC_HSI48_Enable(); 
  while(LL_RCC_HSI48_IsReady() != 1) 
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI48);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI48)
  {
  };
  
  /* Set APB1 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  
  /* Set systick to 1ms in using frequency set to 48MHz */
  /* This frequency can be calculated through LL RCC macro */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI48_VALUE, LL_RCC_PLL_MUL_2, LL_RCC_PREDIV_DIV_2) */
  LL_Init1msTick(48000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);

  /* Disable HSI and wait for de-activation*/
  LL_RCC_HSI_Disable(); 
  while(LL_RCC_HSI_IsReady() != 0) 
  {
  };
}
#endif /* _OPTIM_CONSO */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file pointer to the source file name
  * @param  line assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

#ifdef USBPD_TERMINAL_OUTPUT
/**
 * @brief  Send an async message to Terminal Output
 * @usage  
 */ 
void USBPD_TerminalOutput_Async_Notify(char *string)
{
  if (BSP_USART_GetHandle() != NULL)
  {
    uint8_t len = (uint8_t)strlen(string);
    if (len > 0)
    {
      HAL_UART_Transmit(BSP_USART_GetHandle(), (uint8_t *)string, len, 100);
    }
  }
}

void USBPD_TerminalOutput_Async_PrintBuffer(uint16_t Length, uint8_t * pData)
{
  uint16_t i=0;
  
  sprintf(TerminalOutputbuffer, "\r\n");
  USBPD_TerminalOutput_Async_Notify(TerminalOutputbuffer);
  for (i=0; i<Length ; i++)
  {
    sprintf(TerminalOutputbuffer, "0x%02x, ", pData[i]);
    USBPD_TerminalOutput_Async_Notify(TerminalOutputbuffer);
    
    if ((i+1)%16 == 0)
    {
      sprintf(TerminalOutputbuffer, "\r\n");
      USBPD_TerminalOutput_Async_Notify(TerminalOutputbuffer);
    }
  }
  sprintf(TerminalOutputbuffer, "\r\n");
  USBPD_TerminalOutput_Async_Notify(TerminalOutputbuffer);
}

#endif /* USBPD_TERMINAL_OUTPUT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
