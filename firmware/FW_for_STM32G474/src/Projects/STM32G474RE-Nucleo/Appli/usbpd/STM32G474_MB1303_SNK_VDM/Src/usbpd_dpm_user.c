/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file    usbpd_dpm_user.c
* @author  MCD Application Team
* @brief   USBPD DPM user code
******************************************************************************
*
* Copyright (c) 2018 STMicroelectronics. All rights reserved.
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/
/* USER CODE END Header */

#define USBPD_DPM_USER_C
/* Includes ------------------------------------------------------------------*/
#include "User_BSP.h"
#include "usbpd_core.h"
#include "usbpd_dpm_core.h"
#include "usbpd_dpm_conf.h"
#include "usbpd_dpm_user.h"
#if defined(_TRACE)
#include "usbpd_trace.h"
#endif /* _TRACE */
#if defined(_VDM ) || defined(_VCONN_SUPPORT)
#include "usbpd_vdm_user.h"
#endif /*defined( VDM ) || defined( VCONN_SUPPORT)*/
#include "usbpd_pwr_if.h"
#if defined(USBPD_LED_SERVER)
#include "led_server.h"
#endif /* USBPD_LED_SERVER */
#include "string.h"
#if defined(USBPD_USBDATA)
#include "usbd_core.h"
#include "usbd_desc.h"
#ifdef _CLASS_BB
#include "usbd_billboard.h"
#elif _CLASS_HID
#include "usbd_hid.h"
extern  void USBD_HID_Init(void);
#elif _CLASS_CDC
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
extern  void USBD_CDC_Init(void);
#endif
#endif /* USBPD_USBDATA */

/** @addtogroup STM32_USBPD_APPLICATION
* @{
*/

/** @addtogroup STM32_USBPD_APPLICATION_DPM_USER
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN Private_Typedef */
/** @brief  Sink Request characteritics Structure definition
*
*/
typedef struct
{
  uint32_t RequestedVoltageInmVunits;              /*!< Sink request operating voltage in mV units       */
  uint32_t MaxOperatingCurrentInmAunits;           /*!< Sink request Max operating current in mA units   */
  uint32_t OperatingCurrentInmAunits;              /*!< Sink request operating current in mA units       */
  uint32_t MaxOperatingPowerInmWunits;             /*!< Sink request Max operating power in mW units     */
  uint32_t OperatingPowerInmWunits;                /*!< Sink request operating power in mW units         */
} USBPD_DPM_SNKPowerRequestDetails_TypeDef;
/* USER CODE END Private_Typedef */

/* Private define ------------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_DEFINES USBPD USER Private Defines
* @{
*/
/* USER CODE BEGIN Private_Define */

#define DPM_TIMER_ENABLE_MSK      ((uint16_t)0x8000U)       /*!< Enable Timer Mask                                                        */
#define DPM_TIMER_READ_MSK        ((uint16_t)0x7FFFU)       /*!< Read Timer Mask                                                          */
#define DPM_TIMER_DISABLE_MSK      ((uint16_t)0x0000U)      /*!< Disable Timer Mask */
#define DPM_BOX_MESSAGES_MAX      30u
#if defined(_CLASS_BB)

/* Timer used to start billboard driver (based on tAMETimeout */
#define DPM_TIMER_AME             1000u  /*!< 1000ms: The time between a Sink attach until a */
                                        /*!< USB Billboard Device Class interface is exposed when an Alternate Mode is*/
                                        /*!< not successfully entered*/
#ifdef _CLASS_BB
extern uint8_t hUSBDBOSDesc[USB_SIZ_BOS_DESC];
#endif
#endif /* billboard */
#if _ALERT

/* Timer used to check if need to send an alert */
#define DPM_TIMER_ALERT             100u  /*!< 100ms */
#endif
#if _VDM
#define DPM_TIMER_DISCO             20u  /*!< 20ms */
#endif
#define DPM_TIMER_RETRY             100u /*!<100ms */
#if _ADC_MONITORING
#define DPM_TIMER_ADC               5u /*!< 5ms */
#endif
#if !defined (RTOS)
#define DPM_TIMER_EXECUTE            2 /*!<2ms */      
#endif
#define DPM_NO_SRC_PDO_FOUND      0xFFU        /*!< No match found between Received SRC PDO and SNK capabilities                             */

/* USER CODE END Private_Define */
/**
* @}
*/

/* Private macro -------------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_MACROS USBPD USER Private Macros
* @{
*/
/* USER CODE BEGIN Private_Macro */
#define DPM_START_TIMER(_PORT_,_TIMER_,_TIMEOUT_)   do{                                                               \
DPM_Ports[_PORT_]._TIMER_ = (_TIMEOUT_) |  DPM_TIMER_ENABLE_MSK;\
  osMessagePut(DPMMsgBox,DPM_USER_EVENT_TIMER, 0);                \
                                                    }while(0);
#define DPM_STOP_TIMER(_PORT_,_TIMER_,_TIMEOUT_)   do{                                                               \
DPM_Ports[_PORT_]._TIMER_ = (_TIMEOUT_) &  DPM_TIMER_DISABLE_MSK;\
  osMessagePut(DPMMsgBox,DPM_USER_EVENT_TIMER, 0);                \
                                                    }while(0);

#define IS_DPM_TIMER_RUNNING(_PORT_, _TIMER_)       ((DPM_Ports[_PORT_]._TIMER_ & DPM_TIMER_READ_MSK) > 0)
#define IS_DPM_TIMER_EXPIRED(_PORT_, _TIMER_)       (DPM_TIMER_ENABLE_MSK == DPM_Ports[_PORT_]._TIMER_)

#if defined(_TRACE)
#define __DEBUG_CALLBACK(_PORT_, __MESSAGE__)  \
USBPD_TRACE_Add(USBPD_TRACE_DEBUG, (_PORT_), 0u, (uint8_t *)(__MESSAGE__), sizeof(__MESSAGE__) - 1u);
#endif
#if defined(_PE_TRACE_CALLBACK)
#define __DPM_DEBUG_CALLBACK(_PORT_, __MESSAGE__)  USBPD_TRACE_Add(USBPD_TRACE_DEBUG,    (_PORT_), 0u,(__MESSAGE__), sizeof(__MESSAGE__) - 1u)
#else
#define __DPM_DEBUG_CALLBACK(_PORT_, __MESSAGE__)
#endif /* _PE_TRACE_CALLBACK */

/* USER CODE END Private_Macro */
/**
* @}
*/

/* Private variables ---------------------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_VARIABLES USBPD USER Private Variables
* @{
*/
/* USER CODE BEGIN Private_Variables */
osMessageQId  DPMMsgBox;
#if defined(USBPD_USBDATA)
USBD_HandleTypeDef hUsbDevice[1]; /* 1 needs to be replaced by USBPD_PORT_COUNT if multiple USB IPs are innitialized */
#endif
#if defined(USBPD_USBDATA) || defined (_VDM)
extern uint8_t VDM_Mode_On[USBPD_PORT_COUNT];
#endif /* USBPD_USBDATA */
#if defined(_OPTIM_CONSO)
volatile uint32_t FlagExplicitContract;
#endif /* _OPTIM_CONSO */
extern USBPD_ParamsTypeDef DPM_Params[USBPD_PORT_COUNT];

/*STUSB1602 specific */
uint8_t PE_is_explicit =0;
extern void CAD_Set_default_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue);


/**
* @}
*/

/* Private function prototypes -----------------------------------------------*/
/** @defgroup USBPD_USER_PRIVATE_FUNCTIONS USBPD USER Private Functions
* @{
*/
/* USER CODE BEGIN USBPD_USER_PRIVATE_FUNCTIONS_Prototypes */
static  void     DPM_SNK_BuildRDOfromSelectedPDO(uint8_t PortNum, uint8_t IndexSrcPDO, USBPD_DPM_SNKPowerRequestDetails_TypeDef* PtrRequestPowerDetails,
                                                 USBPD_SNKRDO_TypeDef* Rdo, USBPD_CORE_PDO_Type_TypeDef *PtrPowerObject);
static uint32_t  DPM_FindVoltageIndex(uint32_t PortNum, USBPD_DPM_SNKPowerRequestDetails_TypeDef* PtrRequestPowerDetails);
static USBPD_StatusTypeDef DPM_TurnOnPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role);
static USBPD_StatusTypeDef DPM_TurnOffPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role);
static void DPM_AssertRp(uint8_t PortNum);
static uint32_t CheckDPMTimers(void);

#if defined(USBPD_REV30_SUPPORT)
#if _ALERT
void DPM_ManageAlert(void);
#endif /* _ALERT */
#if defined (USBPD_USBDATA)
USBPD_StatusTypeDef DPM_USB_Init(void);
static void DPM_USB_Start(uint32_t PortNum);
static void DPM_USB_Stop(uint32_t PortNum);
#ifdef _CLASS_BB
static void DPM_USB_BB_Start(uint32_t PortNum);
#endif
#endif /* USBPD_USBDATA */
#if _ADC_MONITORING
void DPM_ManageADC(void);
#endif
#endif /* USBPD_REV30_SUPPORT */
/* USER CODE END USBPD_USER_PRIVATE_FUNCTIONS_Prototypes */

/**
* @}
*/

/* Exported functions ------- ------------------------------------------------*/
/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS USBPD USER Exported Functions
* @{
*/
/* USER CODE BEGIN USBPD_USER_EXPORTED_FUNCTIONS */

/* USER CODE END USBPD_USER_EXPORTED_FUNCTIONS */

/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP1 USBPD USER Exported Functions called by DPM CORE
* @{
*/
/* USER CODE BEGIN USBPD_USER_EXPORTED_FUNCTIONS_GROUP1 */

/* USER CODE END USBPD_USER_EXPORTED_FUNCTIONS_GROUP1 */

/**
* @brief  Initialize DPM (port power role, PWR_IF, CAD and PE Init procedures)
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_UserInit(void)
{
  /* USER CODE BEGIN USBPD_DPM_UserInit */
#ifdef USBPD_LED_SERVER
  /* Led management initialization */
  Led_Init();
  
    /* Set the power role */
    Led_Set(LED_PORT_ROLE(USBPD_PORT_0),((DPM_Settings[USBPD_PORT_0].PE_DefaultRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC),0);
#if  USBPD_PORT_COUNT == 2
    Led_Set(LED_PORT_ROLE(USBPD_PORT_1),((DPM_Settings[USBPD_PORT_1].PE_DefaultRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC),0);
#endif
#endif /* USBPD_LED_SERVER */
  
  /* PWR SET UP */
  if(USBPD_OK !=  USBPD_PWR_IF_Init())
  {
    return USBPD_ERROR;
  }
  
  /* VDM initialisation */
#if  defined(_VCONN_SUPPORT)
  
  
  if (USBPD_OK != USBPD_VDM_UserInit(USBPD_PORT_0))
  {
    return USBPD_ERROR;
  }
  
#if USBPD_PORT_COUNT == 2
  if (USBPD_OK != USBPD_VDM_UserInit(USBPD_PORT_1))
    {
      return USBPD_ERROR;
    }
#endif
  
#elif _VDM
  if(USBPD_TRUE == DPM_Settings[USBPD_PORT_0].PE_VDMSupport)
  {
    if (USBPD_OK != USBPD_VDM_UserInit(USBPD_PORT_0))
    {
      return USBPD_ERROR;
    }
  }
#if USBPD_PORT_COUNT == 2
  if(USBPD_TRUE == DPM_Settings[USBPD_PORT_1].PE_VDMSupport)
  {
    if (USBPD_OK != USBPD_VDM_UserInit(USBPD_PORT_1))
    {
      return USBPD_ERROR;
    }
  }
#endif /* USBPD_PORT_COUNT == 2 */
#endif /* _VDM */  
  
  if(USBPD_OK != USBPD_PWR_IF_PowerResetGlobal()) return USBPD_ERROR;
  
  
/*STUSB1602 specific */
  
  osMessageQDef(MsgBox, DPM_BOX_MESSAGES_MAX, uint32_t);
  DPMMsgBox = osMessageCreate(osMessageQ(MsgBox), NULL);
  osThreadDef(DPM, USBPD_DPM_UserExecute, osPriorityLow, 0, 120);
  
  if(NULL == osThreadCreate(osThread(DPM), &DPMMsgBox))
  {
    return USBPD_ERROR;
  }
  
  return USBPD_OK;
  /* USER CODE END USBPD_DPM_UserInit */
}
/**
* @brief  User delay implementation which is OS dependant
* @param  Time time in ms
* @retval None
*/
void USBPD_DPM_WaitForTime(uint32_t Time)
{
  /* USER CODE BEGIN USBPD_DPM_WaitForTime */
  osDelay(Time);
  /* USER CODE END USBPD_DPM_WaitForTime */
}

/**
* @brief  User processing time, it is recommended to avoid blocking task for long time
* @param  argument  DPM User event
* @retval None
*/
#if  _ADC_MONITORING
/*STUSB1602 specific */
uint16_t current ;
uint16_t current_meas[USBPD_PORT_COUNT][5];      
uint16_t i;

uint32_t meas_vsrc;
uint32_t meas_vbus;
uint32_t step = 0;
#endif /*STUSB1602 specific */

void USBPD_DPM_UserExecute(void const *argument)
{
  /* USER CODE BEGIN USBPD_DPM_UserExecute */
  /* User code implementation */
  uint32_t _timing = osWaitForever;
  osMessageQId  queue = *(osMessageQId *)argument;
  
  do
  {
    osEvent event = osMessageGet(queue, _timing);
    switch (((DPM_USER_EVENT)event.value.v & 0xF))
    {
    case DPM_USER_EVENT_TIMER:                      
      {
        DPM_Ports[USBPD_PORT_0].DPM_MeasuredVbus = APPLI_GetVBUS(USBPD_PORT_0);
#if  USBPD_PORT_COUNT == 2
        DPM_Ports[USBPD_PORT_1].DPM_MeasuredVbus = APPLI_GetVBUS(USBPD_PORT_1);
#endif
#ifdef _CLASS_BB
      if (1 == DPM_Ports[USBPD_PORT_0].DPM_IsConnected)
      {
        DPM_USB_BB_Start(USBPD_PORT_0);
      }
#endif
#if defined(USBPD_REV30_SUPPORT)
#if _ALERT
        DPM_ManageAlert();
#endif /* _ALERT*/
#if _VDM
        if (DPM_TIMER_ENABLE_MSK == DPM_Ports[USBPD_PORT_0].DPM_TimerDisco)
    {
            USBPD_PE_SVDM_RequestIdentity(USBPD_PORT_0, USBPD_SOPTYPE_SOP);
            DPM_Ports[USBPD_PORT_0].DPM_TimerDisco=0;
    }
#if  USBPD_PORT_COUNT == 2
     if (DPM_TIMER_ENABLE_MSK == DPM_Ports[USBPD_PORT_1].DPM_TimerDisco)
    {
            USBPD_PE_SVDM_RequestIdentity(USBPD_PORT_1, USBPD_SOPTYPE_SOP);
            DPM_Ports[USBPD_PORT_1].DPM_TimerDisco=0;
    }
#endif
#endif
#endif /* USBPD_REV30_SUPPORT */
     if (DPM_TIMER_ENABLE_MSK == DPM_Ports[USBPD_PORT_0].DPM_TimerRetry_DRswap)
     {
       DPM_Ports[USBPD_PORT_0].DPM_TimerRetry_DRswap=0;
       USBPD_Retry_DRSWAP(USBPD_PORT_0);
     }
         if (DPM_TIMER_ENABLE_MSK == DPM_Ports[USBPD_PORT_0].DPM_TimerRetry_PRswap)
     {
       DPM_Ports[USBPD_PORT_0].DPM_TimerRetry_PRswap=0;
       USBPD_Retry_PRSWAP(USBPD_PORT_0);
     }
#if  USBPD_PORT_COUNT == 2
     if (DPM_TIMER_ENABLE_MSK == DPM_Ports[USBPD_PORT_1].DPM_TimerRetry_DRswap)
     {
       DPM_Ports[USBPD_PORT_1].DPM_TimerRetry_DRswap=0;
       USBPD_Retry_DRSWAP(USBPD_PORT_1);
     }
     if (DPM_TIMER_ENABLE_MSK == DPM_Ports[USBPD_PORT_1].DPM_TimerRetry_PRswap)
     {
       DPM_Ports[USBPD_PORT_1].DPM_TimerRetry_PRswap=0;
       USBPD_Retry_PRSWAP(USBPD_PORT_1);
     }
#endif

 
#if _ADC_MONITORING
#if USBPD_PORT_COUNT == 1
        if (1 == DPM_Ports[USBPD_PORT_0].DPM_IsConnected)
        {
       DPM_ManageADC();   
        }
#elif USBPD_PORT_COUNT == 2
        if ((1 == DPM_Ports[USBPD_PORT_0].DPM_IsConnected) || (1 == DPM_Ports[USBPD_PORT_1].DPM_IsConnected))
        {
       DPM_ManageADC();   
        }
#endif
#endif /* _ALERT */   
        
      }
      break;
      
    default:
      break;
    }
    _timing = CheckDPMTimers();
  }
  while(1);
}

USBPD_StatusTypeDef USBPD_Retry_DRSWAP(uint8_t PortNum)
{
  USBPD_StatusTypeDef status = USBPD_OK;
#ifdef SOURCING_DEVICE
  if  ((DPM_USER_Settings[PortNum].PE_DR_Swap_To_UFP) && (USBPD_PORTDATAROLE_DFP ==  DPM_Params[PortNum].PE_DataRole))
  {
#endif
    if ((0==DPM_Ports[PortNum].DR_swap_rejected ) && (DPM_Ports[PortNum].DPM_DR_retry <10))
    {
      status= USBPD_DPM_RequestDataRoleSwap(PortNum);
 
        if ( USBPD_BUSY == status)
        {
          DPM_START_TIMER(PortNum, DPM_TimerRetry_DRswap, DPM_TIMER_RETRY);
        }
        else
        { 
          if (USBPD_OK == status)
          {
            DPM_Ports[PortNum].DPM_DR_retry ++;
          }
          else
          {
             DPM_Ports[PortNum].DPM_TimerRetry_DRswap    = 0;
             DPM_Ports[PortNum].DPM_DR_retry = 0;
          }
        }     
    }
  else
  {
    DPM_Ports[PortNum].DPM_TimerRetry_DRswap    = 0;
    status = USBPD_OK;
  }
#ifdef SOURCING_DEVICE
  }
#endif
  return status;
}

USBPD_StatusTypeDef USBPD_Retry_PRSWAP(uint8_t PortNum)
{
    USBPD_StatusTypeDef status = USBPD_OK;
#ifdef SOURCING_DEVICE
  if (USBPD_PORTPOWERROLE_SNK ==  DPM_Params[PortNum].PE_PowerRole)
  {   
#endif
  if ((0==DPM_Ports[PortNum].DPM_FlagSendPRSwap ) && (DPM_Ports[PortNum].DPM_PR_retry <10))
     {
       status =  USBPD_DPM_RequestPowerRoleSwap(PortNum) ;
        if ( USBPD_BUSY == status)
        {
          DPM_START_TIMER(PortNum, DPM_TimerRetry_PRswap, DPM_TIMER_RETRY);
        }
        else
        {
          if (USBPD_OK == status)
          {
            DPM_Ports[PortNum].DPM_PR_retry ++;
          }
          else
          {
             DPM_Ports[PortNum].DPM_TimerRetry_PRswap    = 0;
             DPM_Ports[PortNum].DPM_PR_retry = 0;
          }
        }  
  }
  else
  {
    DPM_Ports[PortNum].DPM_TimerRetry_PRswap    = 0;
    status = USBPD_OK;
  }
#ifdef SOURCING_DEVICE
  }
#endif
  return status;
}

/**
* @brief  UserCableDetection reporting events on a specified port from CAD layer.
* @param  PortNum The handle of the port
* @param  State CAD state
* @retval None
*/
void USBPD_DPM_UserCableDetection(uint8_t PortNum, USBPD_CAD_EVENT State)
{
  switch(State)
  {
  case USBPD_CAD_EVENT_ATTEMC:
#if defined(_OPTIM_CONSO)
    /* Switch to 48Mhz*/
    SystemClock_Config_48Mhz();
    FlagExplicitContract = 0;
#endif /* _OPTIM_CONSO */
    
#ifdef _VDM
    USBPD_VDM_UserReset(PortNum);
#endif /* _VDM */
    if(USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
    {
      if (USBPD_OK != USBPD_PWR_IF_VBUSEnable(PortNum))
      {
        /* Should not occurr */
        while(1);
      }
      /* Wait for that VBUS is stable */
      USBPD_HW_IF_CheckVbusValid(PortNum , 40 );
    }
#ifdef USBPD_USBDATA
    if(USBPD_PORTDATAROLE_UFP == DPM_Params[PortNum].PE_DataRole)
    {
#ifdef _CLASS_BB
      /* Start tAMETimeout */
      DPM_START_TIMER(PortNum, DPM_TimerAME, DPM_TIMER_AME);
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "timerusb start attemc", sizeof("timerusb start attemc"));
#endif
#endif
     if ((DPM_Ports[PortNum].DPM_USBState != 1) && (USBPD_PORT_0 == PortNum))
     {
       DPM_USB_Start(PortNum);
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "usb_hid start attemc", sizeof("usb start attemc"));
#endif   
     }

    }
#endif /* USBPD_USBDATA */
    
#ifdef USBPD_LED_SERVER
    /* Led feedback */
    Led_Set(LED_PORT_ROLE(PortNum) , (DPM_Params[PortNum].ActiveCCIs == CC1 ? LED_MODE_BLINK_CC1 : LED_MODE_BLINK_CC2), 0);
    Led_Set(LED_PORT_VBUS(PortNum) , LED_MODE_BLINK_VBUS, 0);
    Led_Set(LED_PORT_CC(PortNum) , ((DPM_Params[PortNum].PE_PowerRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC), 0);
#endif /* USBPD_LED_SERVER */
    DPM_Ports[PortNum].DPM_IsConnected = 1;
    break;
    
  case USBPD_CAD_EVENT_ATTACHED:
#if defined(_OPTIM_CONSO)
    /* Switch to 48Mhz*/
    SystemClock_Config_48Mhz();
    FlagExplicitContract = 0;
#endif /* _OPTIM_CONSO */
    DPM_Ports[PortNum].DPM_origine = 5000;
    if(USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
    {
     STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Discharge_Path_Enable);
      if (USBPD_OK != USBPD_PWR_IF_VBUSEnable(PortNum))
      {
        /* Should not occurr */
        while(1);
      }
/* STUSB1602 specific */
      /* Wait for that VBUS is stable */
      USBPD_HW_IF_CheckVbusValid(PortNum , 40 );
    }
    else /*PE is SINK */
    {
      STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Discharge_Path_Disable);
    }
#ifdef USBPD_USBDATA   
  if(USBPD_PORTDATAROLE_UFP == DPM_Params[PortNum].PE_DataRole)
    {
#ifdef _CLASS_BB     
      /* Start tAMETimeout */
      DPM_START_TIMER(PortNum, DPM_TimerAME, DPM_TIMER_AME);
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "timerusb start att", sizeof("timerusb start att"));
#endif
#endif
      if ((DPM_Ports[PortNum].DPM_USBState != 1)&& (USBPD_PORT_0 == PortNum))
     {
      DPM_USB_Start(PortNum);
 #ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "usb_hid start att", sizeof("usb_hid start att"));
#endif   
     }
    }
#endif /* USBPD_USBDATA */
#ifdef USBPD_LED_SERVER
    /* Led feedback */
    Led_Set(LED_PORT_ROLE(PortNum) , (DPM_Params[PortNum].ActiveCCIs == CC1 ? LED_MODE_BLINK_CC1 : LED_MODE_BLINK_CC2), 0);
    Led_Set(LED_PORT_VBUS(PortNum), LED_MODE_BLINK_VBUS, 0);
    Led_Set(LED_PORT_CC(PortNum) , ((DPM_Params[PortNum].PE_PowerRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC), 0);
#endif /* USBPD_LED_SERVER */
    DPM_Ports[PortNum].DPM_IsConnected = 1;
    DPM_Ports[PortNum].DPM_origine = 5000;
    break;
    
  case USBPD_CAD_EVENT_DETACHED :
  case USBPD_CAD_EVENT_EMC :
  default :
#ifdef USBPD_USBDATA
    /* Stop USB */
    if ((DPM_Ports[PortNum].DPM_USBState != 0)&& (USBPD_PORT_0 == PortNum))
      {    
    DPM_USB_Stop(PortNum);
    } 
#endif
    /* reset all values received from port partner */
    memset(&DPM_Ports[PortNum], 0, sizeof(DPM_Ports[PortNum]));
#ifdef _VDM
    USBPD_VDM_UserReset(PortNum);
#endif /* _VDM */
    DPM_Ports[PortNum].DPM_origine = 5000;
#if defined(_OPTIM_CONSO)
    /* Switch to 8Mhz*/
    SystemClock_Config_8Mhz();
    FlagExplicitContract = 0;
#endif /* _OPTIM_CONSO */
#if defined(_VCONN_SUPPORT)
    if(USBPD_TRUE == DPM_Params[PortNum].VconnStatus)
    {
      /* Switch Off Vconn */
      USBPD_DPM_PE_VconnPwr(PortNum, USBPD_DISABLE);
    }
#endif /* _VCONN_SUPPORT */
#ifdef USBPD_LED_SERVER
    /* Led feedback */
    Led_Set(LED_PORT_CC(PortNum), LED_MODE_OFF, 0);
    Led_Set(LED_PORT_VBUS(PortNum) , LED_MODE_OFF, 0);
      /* Set the power role */
      Led_Set(LED_PORT_ROLE(PortNum),((DPM_Settings[PortNum].PE_DefaultRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC),              0);
#endif /* USBPD_LED_SERVER */
#ifdef _ADC_MONITORING
    DPM_Ports[PortNum].DPM_TimerADC = 0;
#endif
    break;
  }
  /* USER CODE END USBPD_DPM_UserCableDetection */
}

/**
* @brief  function used to manage user timer.
* @param  PortNum Port number
* @retval None
*/
void USBPD_DPM_UserTimerCounter(uint8_t PortNum)
{
  /* USER CODE BEGIN USBPD_DPM_UserTimerCounter */
  
#ifdef _CLASS_BB
  if((DPM_Ports[PortNum].DPM_TimerAME & DPM_TIMER_READ_MSK) > 0)
  {
    DPM_Ports[PortNum].DPM_TimerAME--;
  }
#endif /* _CLASS_BB */
#ifdef USBPD_REV30_SUPPORT
#if _ALERT
  if((DPM_Ports[PortNum].DPM_TimerAlert & DPM_TIMER_READ_MSK) > 0)
  {
    DPM_Ports[PortNum].DPM_TimerAlert--;
  }
#endif /* _ALERT */
#if _VDM
  if((DPM_Ports[PortNum].DPM_TimerDisco & DPM_TIMER_READ_MSK) > 0)
  {
    DPM_Ports[PortNum].DPM_TimerDisco--;
  }
#endif /* VDM */
  if((DPM_Ports[PortNum].DPM_TimerRetry_DRswap & DPM_TIMER_READ_MSK) > 0)
  {
    DPM_Ports[PortNum].DPM_TimerRetry_DRswap--;
  }
    if((DPM_Ports[PortNum].DPM_TimerRetry_PRswap & DPM_TIMER_READ_MSK) > 0)
  {
    DPM_Ports[PortNum].DPM_TimerRetry_PRswap--;
  }
#if _ADC_MONITORING
  if((DPM_Ports[PortNum].DPM_TimerADC & DPM_TIMER_READ_MSK) > 0)
  {
    DPM_Ports[PortNum].DPM_TimerADC--;
  }
#endif /* _ALERT */
#endif /* USBPD_REV30_SUPPORT */
  /* USER CODE END USBPD_DPM_UserTimerCounter */
}

/**
* @}
*/

/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP2 USBPD USER Exported Callbacks functions called by PE
* @{
*/

/**
* @brief  Callback function called by PE layer when HardReset message received from PRL
* @param  PortNum     The current port number
* @param  CurrentRole the current role
* @param  Status      status on hard reset event
* @retval None
*/
void USBPD_DPM_HardReset(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HR_Status_TypeDef Status)
{
  /* USER CODE BEGIN USBPD_DPM_HardReset */
  USBPD_StatusTypeDef status;
#ifdef _VDM
  USBPD_VDM_UserReset(PortNum);
  DPM_Ports[PortNum].DPM_TimerDisco    = 0;
#endif /* _VDM */
  DPM_Ports[PortNum].DPM_TimerRetry_DRswap    = 0;
  DPM_Ports[PortNum].DPM_TimerRetry_PRswap    = 0;
#if defined(USBPD_REV30_SUPPORT) && _ALERT
  /* Stop Alert timer */
  DPM_Ports[PortNum].DPM_TimerAlert    = 0;
  DPM_Ports[PortNum].DPM_SendAlert.d32 = 0;
/* USBPD_REV30_SUPPORT && ALERT */  
#endif 

/*STUSB1602 specific */
  DPM_Ports[PortNum].DPM_RDOPositionPrevious   = 1;
  DPM_Ports[PortNum].DPM_RequestedVoltage = 5000;
  DPM_Ports[PortNum].DPM_MeasuredCurrent = 0;
  if (vRp_3_0A == DPM_Settings[PortNum].CAD_DefaultResistor )
  {
    DPM_Ports[PortNum].DPM_RequestedCurrent = 3000;
    STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_3_0_A);
  }
  else
  {
    if (vRp_1_5A == DPM_Settings[PortNum].CAD_DefaultResistor )
    {
      DPM_Ports[PortNum].DPM_RequestedCurrent = 1500;
      STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_1_5_A);
    }
    else
    {
      DPM_Ports[PortNum].DPM_RequestedCurrent = 500;   
      STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_Default);
    }
  }
     DPM_Ports[PortNum].DPM_RequestedPDP = (uint32_t)((DPM_Ports[PortNum].DPM_RequestedCurrent ) * (DPM_Ports[PortNum].DPM_RequestedVoltage ) /1000) ;

    APPLI_Set_Current_Limit(PortNum,DPM_Ports[PortNum].DPM_RequestedCurrent); 
    DPM_Ports[PortNum].DPM_VBUSCC = DPM_Ports[PortNum].DPM_RequestedVoltage;
    DPM_Ports[PortNum].DPM_origine = 5000;
    APPLI_SetVoltage(PortNum, 5000);
    switch(Status)
    {
    case USBPD_HR_STATUS_START_ACK:
      DPM_Ports[PortNum].DPM_FlagHardResetOngoing = 1;
#if defined(_TRACE)
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_START_ACK)");
#endif /* defined(_TRACE)*/
      STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
      STUSB1602_VBUS_Presence_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Presence_Enable);
 
      if (USBPD_PORTPOWERROLE_SRC == CurrentRole)
      {
        /* Restore default Role in case of Power Swap failing due to no PS_READY from Sink (TC PC.E2)  */
        DPM_AssertRp(PortNum);
      }
      APPLI_GetVBUS(PortNum);
      USBPD_HW_IF_HR_Start(PortNum, CurrentRole, ACKNOWLEDGE);
      DPM_TurnOffPower(PortNum, CurrentRole);
      break;
    case USBPD_HR_STATUS_START_REQ:
      DPM_Ports[PortNum].DPM_FlagHardResetOngoing = 1;
      STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
      STUSB1602_VBUS_Presence_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Presence_Enable);

#if defined(_TRACE)
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_START_REQ)");
#endif /* defined(_TRACE)*/      
      if (USBPD_PORTPOWERROLE_SRC == CurrentRole)
      {
        /* Restore default Role in case of Power   failing due to no PS_READY from Sink (TC PC.E2)  */
        DPM_AssertRp(PortNum);
      }
      USBPD_HW_IF_HR_Start(PortNum, CurrentRole, REQUEST);
      DPM_TurnOffPower(PortNum, CurrentRole);
      break;
    case USBPD_HR_STATUS_WAIT_VBUS_VSAFE0V:
#if defined(_TRACE)     
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_WAIT_VBUS_VSAFE0V)");
#endif /* defined(_TRACE)*/  

      break;
    case USBPD_HR_STATUS_WAIT_VBUS_VSAFE5V:
#if defined(_TRACE)      
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_WAIT_VBUS_VSAFE5V)");
#endif /* defined(_TRACE)*/      
      USBPD_HW_IF_HR_End(PortNum, CurrentRole);
      status = DPM_TurnOnPower(PortNum, CurrentRole);
      UNUSED(status);
      break;
    case USBPD_HR_STATUS_COMPLETED:
      DPM_Ports[PortNum].DPM_FlagHardResetOngoing = 0;
#if defined(_TRACE)      
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_COMPLETED)");
#endif /* defined(_TRACE)*/      
      status = DPM_TurnOnPower(PortNum, CurrentRole); 
#if USBPD_USBDATA
 /* USB peripheral needs to be restarted */
if (USBPD_PORTDATAROLE_UFP == DPM_Params[PortNum].PE_DataRole)
 {   
 #ifdef _CLASS_BB
    /* Start tAMETimeout */
      DPM_START_TIMER(PortNum, DPM_TimerAME, DPM_TIMER_AME); 
#endif
   if ((DPM_Ports[PortNum].DPM_USBState != 1)&& (USBPD_PORT_0 == PortNum))
     {
     DPM_USB_Start(PortNum);
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "USB start after HR", sizeof("USB start after HR"));
#endif
  }
 }
#endif
      break;
    case USBPD_HR_STATUS_FAILED:
#if defined(_TRACE)      
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_FAILED)");
#endif /*defined(_TRACE)      */
      USBPD_HW_IF_HR_End(PortNum, CurrentRole);
      DPM_Ports[PortNum].DPM_FlagHardResetOngoing = 0;
      break;
    default:
#if defined(_TRACE)      
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(Default)");
#endif /*defined(_TRACE)      */       
      break;
    }
    /* USER CODE END USBPD_DPM_HardReset */
}

/**
* @brief  Evaluate the swap request from PE.
* @param  PortNum The current port number
* @retval USBPD_ACCEPT, USBPD_WAIT, USBPD_REJECT
*/
USBPD_StatusTypeDef USBPD_DPM_EvaluatePowerRoleSwap(uint8_t PortNum)
{
  /* USER CODE BEGIN USBPD_DPM_EvaluatePowerRoleSwap */
#ifdef SOURCING_DEVICE
  /* only accepted if role is not already SINK*/
  if (USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
  { 
    return USBPD_REJECT;
  }
  else
#endif
  return USBPD_ACCEPT;
  /* USER CODE END USBPD_DPM_EvaluatePowerRoleSwap */
}

/**
* @brief  Callback function called by PE to inform DPM about PE event.
* @param  PortNum The current port number
* @param  EventVal @ref USBPD_NotifyEventValue_TypeDef
* @retval None
*/

void USBPD_DPM_Notification(uint8_t PortNum, USBPD_NotifyEventValue_TypeDef EventVal)
{
  /* USER CODE BEGIN USBPD_DPM_Notification */
  switch(EventVal)
  {
case USBPD_NOTIFY_PE_DISABLED:
  {
#if USBPD_USBDATA
    if (USBPD_PORTDATAROLE_UFP == DPM_Params[PortNum].PE_DataRole)
    {
      if ((DPM_Ports[PortNum].DPM_USBState != 1)&& (USBPD_PORT_0 == PortNum))
      {
        DPM_USB_Start(PortNum);
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "usb start pe_disabled", sizeof("usb start pe_disabled"));
#endif   

      }
    }
#endif
    }
  case  USBPD_NOTIFY_SOFTRESET_SENT:
    {
    if (0 ==  DPM_Ports[PortNum].DPM_IsConnected)
    {
      __NOP();
    }
    }
    /***************************************************************************
    Power Notification
    */
  case  USBPD_NOTIFY_POWER_STATE_CHANGE:
    {
    }
    break;
  case USBPD_NOTIFY_POWER_EXPLICIT_CONTRACT :
    /* Power ready means an explicit contract has been establish and Power is available */
#if defined(_VDM)
    if (USBPD_PORTDATAROLE_DFP == DPM_Params[PortNum].PE_DataRole)
    {
      if (USBPD_SPECIFICATION_REV3 == DPM_Params[PortNum].PE_SpecRevision)
      {
       DPM_START_TIMER(PortNum, DPM_TimerDisco, DPM_TIMER_DISCO);
      }
      else
      {
       USBPD_PE_SVDM_RequestIdentity(PortNum, USBPD_SOPTYPE_SOP);
      }
    }
#endif /*_VDM*/
#if defined(USBPD_REV30_SUPPORT)
#if _ALERT
#endif /* _ALERT */
#if USBPD_USBDATA    
        if ((USBPD_PORTDATAROLE_UFP == DPM_Params[PortNum].PE_DataRole) && (USBPD_PORT_0 == PortNum))
        {
#if defined(_CLASS_HID)||defined(_CLASS_CDC)  
          if (DPM_Ports[PortNum].DPM_USBState != 1)
           {
            DPM_USB_Start(PortNum);
           }
#endif
        }
#endif
#if defined(_ADC_MONITORING)
    DPM_START_TIMER(PortNum, DPM_TimerADC, DPM_TIMER_ADC);
#endif    
#endif /* USBPD_REV30_SUPPORT */
#ifdef USBPD_LED_SERVER
    /* Turn On VBUS LED when an explicit contract is established */
    Led_Set(LED_PORT_VBUS(PortNum) , LED_MODE_ON, 0);
#endif /* USBPD_LED_SERVER */

#if defined(_OPTIM_CONSO)
    FlagExplicitContract = 2;
#endif /* _OPTIM_CONSO */



    break;
    /*
    End Power Notification
    ***************************************************************************/
    /***************************************************************************
    REQUEST ANSWER NOTIFICATION
    */
  case USBPD_NOTIFY_REQUEST_ACCEPTED:
    /* Update DPM_RDOPosition only if current role is SNK */
    if (USBPD_PORTPOWERROLE_SNK == DPM_Params[PortNum].PE_PowerRole)
    {
      USBPD_SNKRDO_TypeDef rdo;
      rdo.d32                             = DPM_Ports[PortNum].DPM_RequestDOMsg;
      DPM_Ports[PortNum].DPM_RDOPosition  = rdo.GenericRDO.ObjectPosition;
      APPLI_SetVoltage(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage);
    }
    break;
  case USBPD_NOTIFY_REQUEST_REJECTED:
  case USBPD_NOTIFY_REQUEST_WAIT:
    DPM_Ports[PortNum].DPM_RDOPosition  = DPM_Ports[PortNum].DPM_RDOPositionPrevious;
    DPM_Ports[PortNum].DPM_RequestDOMsg = DPM_Ports[PortNum].DPM_RequestDOMsgPrevious;
    break;
  case USBPD_NOTIFY_DATAROLESWAP_REJECTED:
    DPM_Ports[PortNum].DR_swap_rejected =1;
    break;
    

#if _ALERT
  case USBPD_NOTIFY_ALERT_SENT:
    
    {      /* for PPS*/
      if ( USBPD_ADO_TYPE_ALERT_OPERATING_COND == DPM_Ports[PortNum].DPM_SendAlert.b.TypeAlert)
      {
        DPM_Ports[PortNum].DPM_SrcOutputVoltage= DPM_Ports[PortNum].DPM_SrcOutputVoltage - 20;
        APPLI_SetVoltage(PortNum, DPM_Ports[PortNum].DPM_SrcOutputVoltage);
#ifdef _TRACE
        uint8_t tab[32];
        uint8_t size;
        size = sprintf((char*)tab, "DPM_SrcOutputVoltage: %ld", DPM_Ports[PortNum].DPM_SrcOutputVoltage);
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size); 
#endif
        
      }
      /* case notify Alert sent */
      DPM_Ports[PortNum].DPM_Alertsent =1;
      if (DPM_Ports[PortNum].DPM_SendAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OCP)
      {
        USBPD_DPM_RequestHardReset(PortNum);
      }
      break;
    }
#endif
    /* end Laert notif*/
#if _ALERT
  case USBPD_NOTIFY_ALERT_RECEIVED:
    {
      uint32_t index = 0;
      /*!< Battery Status Change Event(Attach/Detach/charging/discharging/idle) */
      if (USBPD_ADO_TYPE_ALERT_BATTERY_STATUS == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_BATTERY_STATUS))
      {
        USBPD_GBSDB_TypeDef get_battery_status;
        uint32_t battery_found = 0;
        
        /* Set default value for BatteryStatusRef
        (useful when no battery selected Fixed Batteries or Hot Swappable Batteries fields in ALERT message
        as in MQP implementation of Test PROT-SRC3-STATUS */
        get_battery_status.BatteryStatusRef = 0;
        
        for (index = 0; index < 4; index++)
        {
          if ((1 << index) == (DPM_Ports[PortNum].DPM_RcvAlert.b.FixedBatteries & (1 << index)))
          {
            get_battery_status.BatteryStatusRef = index;
            battery_found = 1;
            break;
          }
        }
        if (!battery_found)
        {
          for (index = 0; index < 4; index++)
          {
            if ((1 << index) == (DPM_Ports[PortNum].DPM_RcvAlert.b.HotSwappableBatteries & (1 << index)))
            {
              get_battery_status.BatteryStatusRef = index + 4;
              battery_found = 1;
              break;
            }
          }
        }
        
        /* Post a GET_BATTERY_STATUS message to PE */
        USBPD_DPM_RequestGetBatteryStatus(PortNum, (uint8_t*)&get_battery_status);
        
        /* Reset Battery Status bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_BATTERY_STATUS;
        break;
      }
      
      USBPD_DPM_RequestGetStatus(PortNum);
    }
    break;
#endif /* _ALERT */
    /*
    End REQUEST ANSWER NOTIFICATION
    ***************************************************************************/
#ifdef _VDM
  case USBPD_NOTIFY_HARDRESET_RX:
  case USBPD_NOTIFY_HARDRESET_TX:
    if (USBPD_PORTPOWERROLE_SNK == DPM_Params[PortNum].PE_PowerRole)
    {
      USBPD_VDM_UserReset(PortNum);
    }
    break;
#endif /* _VDM */
  case USBPD_NOTIFY_STATE_SNK_READY:
    {
#if defined(USBPD_REV30_SUPPORT)
#endif /* USBPD_REV30_SUPPORT */
#ifdef SOURCING_DEVICE
      /* ask for a power role swap */
      
       if ((0==DPM_Ports[PortNum].DPM_FlagSendPRSwap ))
       {
         DPM_START_TIMER(PortNum, DPM_TimerRetry_PRswap, DPM_TIMER_RETRY);
       }
#endif
    }
    break;
    
  case USBPD_NOTIFY_STATE_SRC_READY:
    {
#ifdef SOURCING_DEVICE
     if ((USBPD_PORTDATAROLE_DFP ==  DPM_Params[PortNum].PE_DataRole) && (PortNum == USBPD_PORT_0)) /* no USB data available on port1 */
      {  
        if (0==DPM_Ports[PortNum].DR_swap_rejected ) 
        {
          DPM_START_TIMER(PortNum, DPM_TimerRetry_DRswap, DPM_TIMER_RETRY);
        }
      }
#endif
    }
    break;
    
  case USBPD_NOTIFY_STATE_SRC_DISABLED:
    {
      /* SINK Port Partner is not PD capable. Legacy cable may have been connected
      In this state, VBUS is set to 5V */
    }
    break;
  case USBPD_NOTIFY_POWER_SWAP_ACCEPTED:
    /* Sink needs to stop sinking current here */
    break;
  case USBPD_NOTIFY_POWER_SWAP_REJ:
    DPM_Ports[PortNum].DPM_FlagSendPRSwap = 1;
    break;
  case USBPD_NOTIFY_DATAROLESWAP_DFP:
    STUSB16xx_HW_IF_DataRoleSwap(PortNum);
    USBPD_HW_IF_DataRole(PortNum);
#if USBPD_USBDATA
    if (USBPD_FALSE == DPM_USER_Settings[PortNum].PE_DR_Swap_To_DFP)
    { 
      
       if ((DPM_Ports[PortNum].DPM_USBState != 0)&& (USBPD_PORT_0 == PortNum))
      {
    DPM_USB_Stop(PortNum);
     }
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "USB stop. no to dfp", sizeof("USB start swap dfp"));
#endif
    }
#endif
#if defined(_VDM) 
      if(1)
      {  
        USBPD_PE_SVDM_RequestIdentity(PortNum, USBPD_SOPTYPE_SOP);
      }    
#endif /*_VDM*/
    break;
  case USBPD_NOTIFY_DATAROLESWAP_UFP :
    STUSB16xx_HW_IF_DataRoleSwap(PortNum);
    USBPD_HW_IF_DataRole(PortNum);
#if USBPD_USBDATA
#ifdef _CLASS_BB
     DPM_START_TIMER(PortNum, DPM_TimerAME, DPM_TIMER_AME);
#endif
     if ((DPM_Ports[PortNum].DPM_USBState != 1)&& (USBPD_PORT_0 == PortNum))
     {
     DPM_USB_Start(PortNum);
     #ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "USB start DRswap", sizeof("USB start DRswap"));
     #endif
     }
     else
     {
     #ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "USB already ON", sizeof("USB already ON"));
     #endif
     }

#endif
        
    break;
    
  case USBPD_NOTIFY_REQUEST_ENTER_MODE:
    __NOP();
    break;
  default :
    break;
  }
  /* USER CODE END USBPD_DPM_Notification */
}

/**
* @brief  DPM callback to allow PE to retrieve information from DPM/PWR_IF.
* @param  PortNum Port number
* @param  DataId  Type of data to be updated in DPM based on @ref USBPD_CORE_DataInfoType_TypeDef
* @param  Ptr     Pointer on address where DPM data should be written (u8 pointer)
* @param  Size    Pointer on nb of u8 written by DPM
* @retval None
*/
void USBPD_DPM_GetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint8_t *Ptr, uint32_t *Size)
{
  /* USER CODE BEGIN USBPD_DPM_GetDataInfo */
  uint8_t index = 0;
  /* Check type of information targeted by request */
  switch (DataId)
  {
    /* Case Port Source PDO Data information :
    Case Port SINK PDO Data information :
    Call PWR_IF PDO reading request.
    */
  case USBPD_CORE_DATATYPE_SRC_PDO :
  case USBPD_CORE_DATATYPE_SNK_PDO :
    USBPD_PWR_IF_GetPortPDOs(PortNum, DataId, Ptr, Size);
    *Size *= 4;
    break;
    
    /* Case Port Received Source PDO Data information (from distant port) */
  case USBPD_CORE_DATATYPE_RCV_SRC_PDO :
    for(index = 0; index < DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO; index++)
    {
      *(uint32_t *)(Ptr + index*4) = DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO[index];
    }
    *Size = (DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO * 4);
    break;
    
    /* Case Port Received Sink PDO Data information (from distant port) */
  case USBPD_CORE_DATATYPE_RCV_SNK_PDO :
    for(index = 0; index < DPM_Ports[PortNum].DPM_NumberOfRcvSNKPDO; index++)
    {
      *(uint32_t*)(Ptr + index*4) = DPM_Ports[PortNum].DPM_ListOfRcvSNKPDO[index];
    }
    *Size = (DPM_Ports[PortNum].DPM_NumberOfRcvSNKPDO * 4);
    break;
    
    /* Case Requested voltage value Data information */
  case USBPD_CORE_DATATYPE_REQ_VOLTAGE :
    *Size = 4;
    (void)memcpy((uint8_t*)Ptr, (uint8_t *)&DPM_Ports[PortNum].DPM_RequestedVoltage, *Size);
    break;
    
#if defined(USBPD_REV30_SUPPORT)
#if _STATUS
  case USBPD_CORE_INFO_STATUS :
    {
      USBPD_SDB_TypeDef  infostatus = {
        .InternalTemp = 0,          /*!< Source or Sink internal temperature in degrees centigrade */
        .PresentInput = POWER_TYPE_NO_BATT,          /*!< Present Input                                             */
        .PresentBatteryInput = 0,   /*!< Present Battery Input                                     */
        .EventFlags = 0,            /*!< Event Flags                                               */
        .TemperatureStatus = 0,     /*!< Temperature                                               */
        .PowerStatus = 0,           /*!< Power Status based on combination of @ref USBPD_SDB_POWER_STATUS*/
      };
      
      *Size = sizeof(USBPD_SDB_TypeDef);
      memcpy((uint8_t *)Ptr, &infostatus, *Size);
    }
    break;
#endif /* _STATUS */
#if _MANU_INFO
  case USBPD_CORE_MANUFACTURER_INFO :
    {
      USBPD_MIDB_TypeDef* manu_info;
      /* Manufacturer Info Target must be a range 0..1 */
      /* Manufacturer Info Ref must be a range 0..7    */
      if((DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoTarget > USBPD_MANUFINFO_TARGET_BATTERY)
         || (DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoRef > USBPD_MANUFINFO_REF_MAX_VALUES))
      {
        /* If the Manufacturer Info Target field or Manufacturer Info Ref field in the Get_Manufacturer_Info Message is
        unrecognized the field Shall return a null terminated ascii text string “Not Supported”.*/
        char *_notsupported = "Not Supported\0";
        *Size = 4 + 14; /* VID (2) + .PID(2) + sizeof("Not Supported\0")*/
        /* Copy Manufacturer Info into data area for transmission */
        manu_info = (USBPD_MIDB_TypeDef*)&DPM_USER_Settings[PortNum].DPM_ManuInfoPort;
        /* If Manu info Ref is different from VID project 0xFFFF should be sent as VID*/
        memset((uint8_t*)Ptr,0xFF, 4);
        memcpy((uint8_t*)(Ptr + 4), (uint8_t *)_notsupported, 14);
      }
      else
      {
        if (USBPD_MANUFINFO_TARGET_PORT_CABLE_PLUG == DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoTarget)
        {
          /* Manufacturer info requested for the port */
          /* VID(2) + .PID(2) + .ManuString("STMicroelectronics") */
          *Size = 4 + strlen((char*)(DPM_USER_Settings[PortNum].DPM_ManuInfoPort.ManuString));
          /* Copy Manufacturer Info into data area for transmission */
          manu_info = (USBPD_MIDB_TypeDef*)&DPM_USER_Settings[PortNum].DPM_ManuInfoPort;
          memcpy((uint8_t*)Ptr, (uint8_t *)manu_info, *Size);
        }
        else
        {
          /* Manufacturer info requested for the battery (not available yet) */
          /* If the Manufacturer Info Target field or Manufacturer Info Ref field in the Get_Manufacturer_Info Message is
          unrecognized the field Shall return a null terminated ascii text string “Not Supported”.*/
          char *_notsupported = "Not Supported\0";
          *Size = 4 + 14; /* VID (2) + .PID(2) + sizeof("Not Supported\0")*/
          /* Copy Manufacturer Info into data area for transmission */
          manu_info = (USBPD_MIDB_TypeDef*)&DPM_USER_Settings[PortNum].DPM_ManuInfoPort;
          memcpy((uint8_t*)Ptr, (uint8_t *)manu_info, 4);
          memcpy((uint8_t*)(Ptr + 4), (uint8_t *)_notsupported, 14);
        }
      }
    }
    break;
#endif /* _MANU_INFO */
  case USBPD_CORE_BATTERY_STATUS:
    {
      USBPD_BSDO_TypeDef  battery_status;
      
      battery_status.d32 = 0;
      {
        battery_status.b.BatteryPC    = 0xFFFFu;
        battery_status.b.BatteryInfo  = USBPD_BSDO_BATT_INFO_INVALID_REF;
      }
      *Size = 4;
      memcpy((uint8_t *)Ptr, (uint8_t *)&battery_status.d32, *Size);
    }
    break;
    
  case USBPD_CORE_BATTERY_CAPABILITY:
    {
      {
        /* Set Size to 0 to send a not supported message */
        *Size = 0;
      }
    }
    break;
#if _ALERT
  case USBPD_CORE_ALERT:
    {
      uint32_t index = 0;
      
      /*!< Battery Status Change Event(Attach/Detach/charging/discharging/idle) */
      if (USBPD_ADO_TYPE_ALERT_BATTERY_STATUS == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_BATTERY_STATUS))
      {
        USBPD_GBSDB_TypeDef get_battery_status;
        uint32_t battery_found = 0;
        
        /* Set default value for BatteryStatusRef
        (useful when no battery selected Fixed Batteries or Hot Swappable Batteries fields in ALERT message
        as in MQP implementation of Test PROT-SRC3-STATUS */
        get_battery_status.BatteryStatusRef = 0;
        
        for (index = 0; index < 4; index++)
        {
          if ((1 << index) == (DPM_Ports[PortNum].DPM_RcvAlert.b.FixedBatteries & (1 << index)))
          {
            get_battery_status.BatteryStatusRef = index;
            battery_found = 1;
            break;
          }
        }
        if (!battery_found)
        {
          for (index = 0; index < 4; index++)
          {
            if ((1 << index) == (DPM_Ports[PortNum].DPM_RcvAlert.b.HotSwappableBatteries & (1 << index)))
            {
              get_battery_status.BatteryStatusRef = index + 4;
              battery_found = 1;
              break;
            }
          }
        }
        
        /* Post a GET_BATTERY_STATUS message to PE */
        USBPD_DPM_RequestGetBatteryStatus(PortNum, (uint8_t*)&get_battery_status);
        
        /* Reset Battery Status bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_BATTERY_STATUS;
        break;
      }
      /*!< Over-Current Protection event when set (Source only, for Sink Reserved and Shall be set to zero) */
      /* Bit reserved for a Sink */
      if ((USBPD_PORTPOWERROLE_SNK == DPM_Params[PortNum].PE_PowerRole)
          && (USBPD_ADO_TYPE_ALERT_OCP == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OCP)))
      {
        /* Reset OCP bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_OCP;
        /* Send a Get_Status */
        goto _ctrl_msg;
      }
      /*!< Over-Temperature Protection event when set. */
      else if (USBPD_ADO_TYPE_ALERT_OTP == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OTP))
      {
        /* Reset OTP bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_OTP;
        /* Send a Get_Status */
        goto _ctrl_msg;
      }
      /*!< Operating Condition Change when set */
      else if (USBPD_ADO_TYPE_ALERT_OPERATING_COND == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OPERATING_COND))
      {
        /* Reset OP COND bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_OPERATING_COND;
        /* Send a Get_Status */
        goto _ctrl_msg;
      }
      /*!< Source Input Change Event when set */
      else if (USBPD_ADO_TYPE_ALERT_SRC_INPUT == (DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_SRC_INPUT))
      {
        /* Reset SRC input bit */
        DPM_Ports[PortNum].DPM_RcvAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_SRC_INPUT;
        /* Send a Get_Status */
        goto _ctrl_msg;
      }
      /*!< Over-Voltage Protection event when set (Sink only, for Source Reserved and Shall be set to zero) */
      /* Bit reserved for a Source */
      else
      {
      }
      
    _ctrl_msg:
      /* Post GET_STATUS message */
      USBPD_DPM_RequestGetStatus(PortNum);
    }
    break;
#endif /* ALERT */
#endif /* USBPD_REV30_SUPPORT */
  default :
    *Size = 0;
    break;
  }
  /* USER CODE END USBPD_DPM_GetDataInfo */
}

/**
* @brief  DPM callback to allow PE to update information in DPM/PWR_IF.
* @param  PortNum Port number
* @param  DataId  Type of data to be updated in DPM based on @ref USBPD_CORE_DataInfoType_TypeDef
* @param  Ptr     Pointer on the data
* @param  Size    Nb of bytes to be updated in DPM
* @retval None
*/
void USBPD_DPM_SetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint8_t *Ptr, uint32_t Size)
{
  /* USER CODE BEGIN USBPD_DPM_SetDataInfo */
  uint32_t index;
  
  /* Check type of information targeted by request */
  switch (DataId)
  {
    /* Case requested DO position Data information :
    */
  case USBPD_CORE_DATATYPE_RDO_POSITION :
    if (Size == 4)
    {
      uint8_t* temp;
      temp = (uint8_t*)&DPM_Ports[PortNum].DPM_RDOPosition;
      (void)memcpy(temp, Ptr, Size);
      DPM_Ports[PortNum].DPM_RDOPositionPrevious = *Ptr;
      temp = (uint8_t*)&DPM_Ports[PortNum].DPM_RDOPositionPrevious;
      (void)memcpy(temp, Ptr, Size);
    }
    break;
    
    /* Case Received Source PDO values Data information :
    */
  case USBPD_CORE_DATATYPE_RCV_SRC_PDO :
    if (Size <= (USBPD_MAX_NB_PDO * 4))
    {
      uint8_t* rdo;
      DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO = (Size / 4);
      /* Copy PDO data in DPM Handle field */
      for (index = 0; index < (Size / 4); index++)
      {
        rdo = (uint8_t*)&DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO[index];
        (void)memcpy(rdo, (Ptr + (index * 4u)), (4u * sizeof(uint8_t)));
      }
    }
    break;
    
    /* Case Received Sink PDO values Data information :
    */
  case USBPD_CORE_DATATYPE_RCV_SNK_PDO :
    if (Size <= (USBPD_MAX_NB_PDO * 4))
    {
      uint8_t* rdo;
      DPM_Ports[PortNum].DPM_NumberOfRcvSNKPDO = (Size / 4);
      /* Copy PDO data in DPM Handle field */
      for (index = 0; index < (Size / 4); index++)
      {
        rdo = (uint8_t*)&DPM_Ports[PortNum].DPM_ListOfRcvSNKPDO[index];
        (void)memcpy(rdo, (Ptr + (index * 4u)), (4u * sizeof(uint8_t)));
      }
    }
    break;
    
    /* Case Received Request PDO Data information :
    */
  case USBPD_CORE_DATATYPE_RCV_REQ_PDO :
    if (Size == 4)
    {
      uint8_t* rdo;
      rdo = (uint8_t*)&DPM_Ports[PortNum].DPM_RcvRequestDOMsg;
      (void)memcpy(rdo, Ptr, Size);
    }
    break;
    
    /* Case Request message DO (from Sink to Source) Data information :
    */
//  case USBPD_CORE_DATATYPE_REQUEST_DO :
//    if (Size == 4)
//    {
//      uint8_t* rdo;
//      rdo = (uint8_t*)&DPM_Ports[PortNum].DPM_RcvRequestDOMsg;
//      (void)memcpy(rdo, Ptr, Size);
//    }
//    break;
    
#if defined(USBPD_REV30_SUPPORT)
#if _STATUS
  case USBPD_CORE_INFO_STATUS :
    {
      uint8_t* info_status;
      info_status = (uint8_t*)&DPM_Ports[PortNum].DPM_RcvStatus;
      memcpy(info_status, Ptr, Size);
    }
    break;
#endif /* _STATUS */
#if _MANU_INFO
  case USBPD_CORE_GET_MANUFACTURER_INFO:
    {
      uint8_t* temp = (uint8_t*)Ptr;
      DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoTarget = *temp;
      DPM_Ports[PortNum].DPM_GetManufacturerInfo.ManufacturerInfoRef    = *(temp + 1);
    }
    break;
#endif /* _MANU_INFO */
#if _BATTERY
  case USBPD_CORE_BATTERY_STATUS:
    DPM_Ports[PortNum].DPM_BatteryStatus.d32 = *Ptr;
    break;
    
  case USBPD_CORE_GET_BATTERY_STATUS:
    {
      DPM_Ports[PortNum].DPM_GetBatteryStatus.BatteryStatusRef = *(uint8_t*)Ptr;
    }
    break;
    
  case USBPD_CORE_GET_BATTERY_CAPABILITY:
    {
      uint8_t* temp = (uint8_t*)Ptr;
      DPM_Ports[PortNum].DPM_GetBatteryCapability.BatteryCapRef= *temp;
    }
    break;
#endif /* _BATTERY */
#if _ALERT
  case USBPD_CORE_ALERT:
    {
      uint8_t*  alert;
      alert = (uint8_t*)&DPM_Ports[PortNum].DPM_RcvAlert.d32;
      memcpy(alert, Ptr, Size);
      __NOP();
    }
    break;
#endif /* _ALERT */
#endif /* USBPD_REV30_SUPPORT */
    
    /* In case of unexpected data type (Set request could not be fulfilled) :
    */
  default :
    break;
  }
  /* USER CODE END USBPD_DPM_SetDataInfo */
}


/**
* @brief  Evaluate received Capabilities Message from Source port and prepare the request message
* @param  PortNum             Port number
* @param  PtrRequestData      Pointer on selected request data object
* @param  PtrPowerObjectType  Pointer on the power data object
* @retval None
*/
void USBPD_DPM_SNK_EvaluateCapabilities(uint8_t PortNum, uint32_t *PtrRequestData, USBPD_CORE_PDO_Type_TypeDef *PtrPowerObjectType)
{
  /* USER CODE BEGIN USBPD_DPM_SNK_EvaluateCapabilities */
  USBPD_PDO_TypeDef  fixed_pdo;
  USBPD_SNKRDO_TypeDef rdo;
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];
  USBPD_USER_SettingsTypeDef *puser = (USBPD_USER_SettingsTypeDef *)&DPM_USER_Settings[PortNum];
  USBPD_DPM_SNKPowerRequestDetails_TypeDef snkpowerrequestdetails;
  uint32_t pdoindex, size;
  uint32_t snkpdolist[USBPD_MAX_NB_PDO];
  USBPD_PDO_TypeDef snk_fixed_pdo;
  
  /* USBPD_DPM_EvaluateCapabilities: Port Partner Requests Max Voltage */

  /* Find the Pdo index for the requested voltage */
  pdoindex = DPM_FindVoltageIndex(PortNum, &snkpowerrequestdetails);
  /* Initialize RDO */
  rdo.d32 = 0;
  
    /* If no valid SNK PDO or if no SRC PDO match found (index>=nb of valid received SRC PDOs or function returned DPM_NO_SRC_PDO_FOUND*/
    if (pdoindex >= pdhandle->DPM_NumberOfRcvSRCPDO)
    {
#ifdef _TRACE
      USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "PE_EvaluateCapability: could not find desired voltage", sizeof("PE_EvaluateCapability: could not find desired voltage"));
#endif /* _TRACE */
      fixed_pdo.d32 = pdhandle->DPM_ListOfRcvSRCPDO[0];
      /* Read SNK PDO list for retrieving useful data to fill in RDO */
      USBPD_PWR_IF_GetPortPDOs(PortNum, USBPD_CORE_DATATYPE_SNK_PDO, (uint8_t*)&snkpdolist[0], &size);
      /* Store value of 1st SNK PDO (Fixed) in local variable */
      snk_fixed_pdo.d32 = snkpdolist[0];
      rdo.FixedVariableRDO.ObjectPosition = 1;
      rdo.FixedVariableRDO.OperatingCurrentIn10mAunits  =  fixed_pdo.SRCFixedPDO.MaxCurrentIn10mAunits;
      rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits / 10;
      rdo.FixedVariableRDO.CapabilityMismatch = 1;
      rdo.FixedVariableRDO.USBCommunicationsCapable = snk_fixed_pdo.SNKFixedPDO.USBCommunicationsCapable;
#if defined(USBPD_REV30_SUPPORT) && defined(_UNCHUNKED_SUPPORT)
      if (USBPD_SPECIFICATION_REV2 < DPM_Params[PortNum].PE_SpecRevision)
      {
        rdo.FixedVariableRDO.UnchunkedExtendedMessage = DPM_Settings[PortNum].PE_PD3_Support.d.PE_UnchunkSupport;
        DPM_Params[PortNum].PE_UnchunkSupport   = USBPD_FALSE;
        /* Set unchuncked bit if supported by port partner;*/
        if (USBPD_TRUE == fixed_pdo.SRCFixedPDO.UnchunkedExtendedMessage)
        {
          DPM_Params[PortNum].PE_UnchunkSupport   = USBPD_TRUE;
        }
      }
#endif /* USBPD_REV30_SUPPORT && UNCHUNKED_SUPPORT */
      DPM_Ports[PortNum].DPM_RequestedCurrent = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits;
      
      pdhandle->DPM_RequestDOMsg = rdo.d32;
      *PtrPowerObjectType = USBPD_CORE_PDO_TYPE_FIXED;
      *PtrRequestData     = rdo.d32;
      pdhandle->DPM_RequestedVoltage = 5000;
      return;
    }
  
  DPM_SNK_BuildRDOfromSelectedPDO(PortNum, pdoindex, &snkpowerrequestdetails,&rdo, PtrPowerObjectType);
  
  
  *PtrRequestData = pdhandle->DPM_RequestDOMsg;
  /* USER CODE END USBPD_DPM_SNK_EvaluateCapabilities */
}

#if defined(_VCONN_SUPPORT)
/**
* @brief  Callback to be used by PE to evaluate a Vconn swap
* @param  PortNum Port number
* @retval USBPD_ACCEPT, USBPD_REJECT, USBPD_WAIT
*/
USBPD_StatusTypeDef USBPD_DPM_EvaluateVconnSwap(uint8_t PortNum)
{
  /* USER CODE BEGIN USBPD_DPM_EvaluateVconnSwap */
  USBPD_StatusTypeDef status = USBPD_REJECT;
  if (USBPD_TRUE == DPM_USER_Settings[PortNum].PE_VconnSwap)
  {
    status = USBPD_ACCEPT;
  }
  
  return status;
  /* USER CODE END USBPD_DPM_EvaluateVconnSwap */
}

/**
* @brief  Callback to be used by PE to manage VConn
* @param  PortNum Port number
* @param  State  Enable or Disable VConn on CC lines
* @retval USBPD_ACCEPT, USBPD_REJECT
*/
USBPD_StatusTypeDef USBPD_DPM_PE_VconnPwr(uint8_t PortNum, USBPD_FunctionalState State)
{
  /* USER CODE BEGIN USBPD_DPM_PE_VconnPwr */
  USBPD_StatusTypeDef status = USBPD_ERROR;
  /* Vconn is required to be ON and was OFF */
  if((USBPD_ENABLE == State) && (CCNONE != DPM_Params[PortNum].VconnCCIs))
  {
#ifdef _TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "Enable Vconn", sizeof("Enable Vconn"));
#endif
    status = USBPD_PWR_IF_Enable_VConn(PortNum,DPM_Params[PortNum].VconnCCIs);
  }
  /* Vconn is required to be OFF and was ON */
  if((USBPD_DISABLE == State) && (CCNONE != DPM_Params[PortNum].VconnCCIs))
  {
#ifdef _TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "Disable Vconn", sizeof("Disable Vconn"));
#endif
    status = USBPD_PWR_IF_Disable_VConn(PortNum,DPM_Params[PortNum].VconnCCIs);
  }
  return status;
  /* USER CODE END USBPD_DPM_PE_VconnPwr */
}
#endif /* _VCONN_SUPPORT */

#ifdef USBPD_REV30_SUPPORT
/**
* @brief  DPM callback to allow PE to forward extended message information.
* @param  PortNum Port number
* @param  MsgType Type of message to be handled in DPM
*         This parameter can be one of the following values:
*           @arg @ref USBPD_EXT_SECURITY_REQUEST Security Request extended message
*           @arg @ref USBPD_EXT_SECURITY_RESPONSE Security Response extended message
* @param  ptrData   Pointer on address Extended Message data could be read (u8 pointer)
* @param  DataSize  Nb of u8 that compose Extended message
* @retval None
*/
void USBPD_DPM_ExtendedMessageReceived(uint8_t PortNum, USBPD_ExtendedMsg_TypeDef MsgType, uint8_t *ptrData, uint16_t DataSize)
{
  /* USER CODE BEGIN USBPD_DPM_ExtendedMessageReceived */
  if (DataSize == 0)
  {
    /* No data received. */
    return;
  }
  
  switch(MsgType)
  {
#ifdef _SECURITY_MSG
  case   USBPD_EXT_SECURITY_REQUEST :
  case   USBPD_EXT_SECURITY_RESPONSE :
    uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    USBPD_PE_SendExtendedMessage(PortNum, USBPD_SOPTYPE_SOP, USBPD_EXT_SECURITY_RESPONSE, data, 4);
    break;
  case   USBPD_EXT_SECURITY_RESPONSE :
    break;
#endif /* _SECURITY_MSG */
#ifdef _FWUPD
  case USBPD_EXT_FIRM_UPDATE_REQUEST :
    uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    USBPD_PE_SendExtendedMessage(PortNum, USBPD_SOPTYPE_SOP, USBPD_EXT_FIRM_UPDATE_RESPONSE, data, 4);
  case USBPD_EXT_FIRM_UPDATE_RESPONSE :
    break;
#endif /* _FWUPD */
#ifdef _COUNTRY_MSG
  case USBPD_EXT_COUNTRY_INFO :
  case USBPD_EXT_COUNTRY_CODES :
    break;
#endif /* _COUNTRY_MSG */
  default:
    break;
  }
  /* USER CODE END USBPD_DPM_ExtendedMessageReceived */
}

#endif /* USBPD_REV30_SUPPORT */
#ifdef _ERROR_RECOVERY
/**
* @brief  DPM callback to allow PE to enter ERROR_RECOVERY state.
* @param  PortNum Port number
* @retval None
*/
void USBPD_DPM_EnterErrorRecovery(uint8_t PortNum)
{
#ifdef _TRACE
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "Enter ErrorRecovery", sizeof("Enter ErrorRecovery"));
  uint8_t tab[32];
  uint8_t size;
  size = sprintf((char*)tab, "DPM_MeasuredCurrent: %d", DPM_Ports[PortNum].DPM_MeasuredCurrent);
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size); 
  size = sprintf((char*)tab, "DPM_MeasuredVbus: %d", DPM_Ports[PortNum].DPM_MeasuredVbus);
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size);
#endif
  /* Take care about VBUS, VCONN voltage */
  DPM_TurnOffPower(PortNum, DPM_Params[PortNum].PE_PowerRole);
  
#if defined(_VCONN_SUPPORT)
  /* VCONN switch OFF */
  if (USBPD_TRUE == DPM_Params[PortNum].VconnStatus)
  {
    /* Switch Off Vconn */
    USBPD_PWR_IF_Disable_VConn(PortNum,CC1);
    USBPD_PWR_IF_Disable_VConn(PortNum,CC2);
    DPM_Params[PortNum].VconnStatus = USBPD_FALSE;
  }
#endif /* _VCONN_SUPPORT */
 /* Inform CAD to enter recovery mode */
  USBPD_CAD_EnterErrorRecovery(PortNum);
#if USBPD_USBDATA
   /* Stop USB */

      if ((DPM_Ports[PortNum].DPM_USBState != 0)&& (USBPD_PORT_0 == PortNum))
      {
    DPM_USB_Stop(PortNum);
     }
#ifdef _CLASS_BB
      DPM_Ports[PortNum].DPM_TimerAME=0;
#endif      
#endif
#if _ADC_MONITORING
  DPM_Ports[PortNum].DPM_TimerADC =0;
#endif
 
  /* USBPD_HW_IF_ErrorRecovery(PortNum);*/
    DPM_Ports[PortNum].DPM_Alertsent =0;
}
#endif /* _ERROR_RECOVERY */

/**
* @brief  DPM callback used to know user choice about Data Role Swap.
* @param  PortNum Port number
* @retval USBPD_REJECT, UBPD_ACCEPT
*/

USBPD_StatusTypeDef USBPD_DPM_EvaluateDataRoleSwap(uint8_t PortNum)
{
  /* USER CODE BEGIN USBPD_DPM_EvaluateDataRoleSwap */
  USBPD_StatusTypeDef status = USBPD_REJECT;

  if ((USBPD_FALSE == DPM_USER_Settings[PortNum].PE_DR_Swap_To_DFP)
    && (USBPD_FALSE == DPM_USER_Settings[PortNum].PE_DR_Swap_To_UFP))
  {
    status = USBPD_NOTSUPPORTED;
  }
  else
  {
  if (USBPD_PORTDATAROLE_DFP == DPM_Params[PortNum].PE_DataRole)
  {
    if (USBPD_TRUE == DPM_USER_Settings[PortNum].PE_DR_Swap_To_UFP)
    {
      STUSB16xx_HW_IF_DataRoleSwap(PortNum);
      status = USBPD_ACCEPT;
    }
  }
  else
  { /* USBPD_PORTDATAROLE_UFP == DPM_Params[PortNum].PE_DataRole*/
    if (USBPD_TRUE == DPM_USER_Settings[PortNum].PE_DR_Swap_To_DFP)
    {
      STUSB16xx_HW_IF_DataRoleSwap(PortNum);
      status = USBPD_ACCEPT;
    }
  }
  }
  return status;
  /* USER CODE END USBPD_DPM_EvaluateDataRoleSwap */
}

/**
* @brief  Callback to be used by PE to check is VBUS is ready or present
* @param  PortNum Port number
* @param  Vsafe   Vsafe status based on @ref USBPD_VSAFE_StatusTypeDef
* @retval USBPD_DISABLE or USBPD_ENABLE
*/
USBPD_FunctionalState USBPD_DPM_IsPowerReady(uint8_t PortNum, USBPD_VSAFE_StatusTypeDef Vsafe)
{
  /* USER CODE BEGIN USBPD_DPM_IsPowerReady */
  USBPD_FunctionalState ret ;
  ret = ((USBPD_OK == USBPD_PWR_IF_SupplyReady(PortNum, Vsafe)) ? USBPD_ENABLE : USBPD_DISABLE);
#ifdef _TRACE
  if ((USBPD_VSAFE_0V == Vsafe) && (ret))
  {
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "vsafe0", sizeof("vsafe0"));    
  }
  else
    if (ret)
    {
      USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "vsafe5", sizeof("vsafe5"));    
    }
  
#endif
#if USBPD_USBDATA
if (USBPD_PORTDATAROLE_UFP == DPM_Params[PortNum].PE_DataRole)
 {  
  if  ((USBPD_VSAFE_0V == Vsafe)&& (ret))
  {
   /* Stop USB */
    if ((DPM_Ports[PortNum].DPM_USBState != 0)&& (USBPD_PORT_0 == PortNum))
      {    
        DPM_USB_Stop(PortNum);
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "USB stop HR", sizeof("USB stop HR"));
#endif 
     }
#ifdef _CLASS_BB
    DPM_Ports[PortNum].DPM_TimerAME = 0;
#endif   
  }
  if ((USBPD_VSAFE_5V == Vsafe) && (ret))
  {
    if ((DPM_Ports[PortNum].DPM_USBState == 0) && (USBPD_PORT_0 == PortNum))
      { 
   /* Start USB */
        DPM_USB_Start(PortNum);
#ifdef _CLASS_BB
      /* Start tAMETimeout */
      DPM_START_TIMER(PortNum, DPM_TimerAME, DPM_TIMER_AME);
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "timerusb start Vdafe5v", sizeof("timerusb start Vdafe5v"));
#endif
#endif
      }
  }
}
#endif 
  return ret;
  /* USER CODE END USBPD_DPM_IsPowerReady */
}

/**
* @}
*/

/** @defgroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP3 USBPD USER Functions PD messages requests
* @{
*/

/**
* @brief  Request the PE to send a hard reset
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestHardReset(uint8_t PortNum)
{
  DPM_Ports[PortNum].DPM_Alertsent =0;
  return USBPD_PE_Request_HardReset(PortNum);
}

/**
* @brief  Request the PE to send a cable reset.
* @note   Only a DFP Shall generate Cable Reset Signaling. A DFP Shall only generate Cable Reset Signaling within an Explicit Contract.
The DFP has to be supplying VCONN prior to a Cable Reset
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestCableReset(uint8_t PortNum)
{
  return USBPD_PE_Request_CableReset(PortNum);
}

/**
* @brief  Request the PE to send a GOTOMIN message
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGotoMin(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GOTOMIN, USBPD_SOPTYPE_SOP);
}

/**
* @brief  Request the PE to send a PING message
* @note   In USB-PD stack, only ping management for P3.0 is implemented.
*         If PD2.0 is used, PING timer needs to be implemented on user side.
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestPing(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_PING, USBPD_SOPTYPE_SOP);
}

/**
* @brief  Request the PE to send a request message.
* @param  PortNum     The current port number
* @param  IndexSrcPDO Index on the selected SRC PDO (value between 1 to 7)
* @param  RequestedVoltage Requested voltage (in MV and use mainly for APDO)
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestMessageRequest(uint8_t PortNum, uint8_t IndexSrcPDO, uint16_t RequestedVoltage)
{
  USBPD_StatusTypeDef status = USBPD_ERROR;
  uint32_t voltage, allowablepower;
  USBPD_SNKRDO_TypeDef rdo;
  USBPD_PDO_TypeDef  pdo;
  USBPD_CORE_PDO_Type_TypeDef pdo_object;
  USBPD_USER_SettingsTypeDef *puser = (USBPD_USER_SettingsTypeDef *)&DPM_USER_Settings[PortNum];
  USBPD_DPM_SNKPowerRequestDetails_TypeDef request_details;
  rdo.d32 = 0;
  
  /* selected SRC PDO */
  pdo.d32 = DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO[(IndexSrcPDO - 1)];
  voltage = RequestedVoltage;
  allowablepower = (puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits * RequestedVoltage) / 1000;
  
  if (USBPD_TRUE == USBPD_DPM_SNK_EvaluateMatchWithSRCPDO(PortNum, pdo.d32, &voltage, &allowablepower))
  {
    /* Check that voltage has been correctly selected */
    if (RequestedVoltage == voltage)
    {
      request_details.RequestedVoltageInmVunits    = RequestedVoltage;
      request_details.MaxOperatingCurrentInmAunits = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits;
      request_details.MaxOperatingPowerInmWunits   = puser->DPM_SNKRequestedPower.MaxOperatingPowerInmWunits;
      request_details.OperatingPowerInmWunits      = puser->DPM_SNKRequestedPower.OperatingPowerInmWunits;
      request_details.OperatingCurrentInmAunits    = (1000 * allowablepower)/RequestedVoltage;
      
      DPM_SNK_BuildRDOfromSelectedPDO(PortNum, (IndexSrcPDO - 1), &request_details, &rdo, &pdo_object);
      
      status = USBPD_PE_Send_Request(PortNum, rdo.d32, pdo_object);
    }
  }
  
  return status;
}

/**
* @brief  Request the PE to send a GET_SRC_CAPA message
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetSourceCapability(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_SRC_CAP, USBPD_SOPTYPE_SOP);
}

/**
* @brief  Request the PE to send a GET_SNK_CAPA message
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetSinkCapability(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_SNK_CAP, USBPD_SOPTYPE_SOP);
}

/**
* @brief  Request the PE to perform a Data Role Swap.
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestDataRoleSwap(uint8_t PortNum)
{
#ifdef _DEBUG_TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "Req DR_swap", sizeof("Req DR_swap"));
#endif
      
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_DR_SWAP, USBPD_SOPTYPE_SOP);
}

/**
* @brief  Request the PE to perform a Power Role Swap.
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestPowerRoleSwap(uint8_t PortNum)
{
  return USBPD_ERROR;
  
}

#if defined(_VCONN_SUPPORT)
/**
* @brief  Request the PE to perform a VCONN Swap.
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestVconnSwap(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_VCONN_SWAP, USBPD_SOPTYPE_SOP);
}
#endif
/**
* @brief  Request the PE to send a soft reset
* @param  PortNum The current port number
* @param  SOPType SOP Type based on @ref USBPD_SOPType_TypeDef
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestSoftReset(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_SOFT_RESET, SOPType);
}

/**
* @brief  Request the PE to send a Source Capability message.
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestSourceCapability(uint8_t PortNum)
{
  DPM_Ports[PortNum].DPM_FlagHardResetOngoing = 0;
  /* PE will directly get the PDO saved in structure @ref PWR_Port_PDO_Storage */
  return USBPD_PE_Request_DataMessage(PortNum, USBPD_DATAMSG_SRC_CAPABILITIES, NULL);
}

#if defined(_VDM ) || defined(_VCONN_SUPPORT)
/**
* @brief  Request the PE to send a VDM discovery identity
* @param  PortNum The current port number
* @param  SOPType SOP Type
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_DiscoveryIdentify(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType)
{
  __DPM_DEBUG_CALLBACK(PortNum, "USBPD_PE_SVDM_RequestIdentity");
  return USBPD_PE_SVDM_RequestIdentity(PortNum, SOPType);
}

/**
* @brief  Request the PE to send a VDM discovery SVID
* @param  PortNum The current port number
* @param  SOPType SOP Type
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_DiscoverySVID(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType)
{
  __DPM_DEBUG_CALLBACK(PortNum, "USBPD_PE_SVDM_RequestSVID");
  return USBPD_PE_SVDM_RequestSVID(PortNum, SOPType);
}

/**
* @brief  Request the PE to perform a VDM Discovery mode message on one SVID.
* @param  PortNum The current port number
* @param  SOPType SOP Type
* @param  SVID    SVID used for discovery mode message
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_DiscoveryMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID)
{
  __DPM_DEBUG_CALLBACK(PortNum, "USBPD_PE_SVDM_RequestMode");
  return USBPD_PE_SVDM_RequestMode(PortNum, SOPType, SVID);
}

/**
* @brief  Request the PE to perform a VDM mode enter.
* @param  PortNum   The current port number
* @param  SOPType   SOP Type
* @param  SVID      SVID used for discovery mode message
* @param  ModeIndex Index of the mode to be entered
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_EnterMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint8_t ModeIndex)
{
  __DPM_DEBUG_CALLBACK(PortNum, "USBPD_PE_SVDM_RequestModeEnter");
  return USBPD_PE_SVDM_RequestModeEnter(PortNum, SOPType, SVID, ModeIndex);
}

/**
* @brief  Request the PE to perform a VDM mode exit.
* @param  PortNum   The current port number
* @param  SOPType   SOP Type
* @param  SVID      SVID used for discovery mode message
* @param  ModeIndex Index of the mode to be exit
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_ExitMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint8_t ModeIndex)
{
  __DPM_DEBUG_CALLBACK(PortNum, "USBPD_PE_SVDM_RequestModeExit");
  return USBPD_PE_SVDM_RequestModeExit(PortNum, SOPType, SVID, ModeIndex);
}

/**
* @brief  Request the PE to send a Display Port status
* @param  PortNum The current port number
* @param  SOPType SOP Type
* @param  SVID    Used SVID
* @param  pDPStatus Pointer on DP Status data (32 bit)
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestDisplayPortStatus(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint32_t *pDPStatus)
{
  __DPM_DEBUG_CALLBACK(PortNum, "USBPD_PE_SVDM_RequestSpecific(DP_STATUS)");
#if defined(_VDM)
  USBPD_VDM_FillDPStatus(PortNum, (USBPD_DPStatus_TypeDef*)pDPStatus);
#endif
  return USBPD_PE_SVDM_RequestSpecific(PortNum, SOPType, SVDM_SPECIFIC_1, SVID);
}
/**
* @brief  Request the PE to send a Display Port Config
* @param  PortNum The current port number
* @param  SOPType SOP Type
* @param  SVID    Used SVID
* @param  pDPConfig Pointer on DP Config data (32 bit)
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestDisplayPortConfig(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint32_t *pDPConfig)
{
  __DPM_DEBUG_CALLBACK(PortNum, "USBPD_PE_SVDM_RequestSpecific(DP_CONFIG)");
#if defined(_VDM)
  USBPD_VDM_FillDPConfig(PortNum, (USBPD_DPConfig_TypeDef*)pDPConfig);
#endif 
  return USBPD_PE_SVDM_RequestSpecific(PortNum, SOPType, SVDM_SPECIFIC_2, SVID);
}
#endif  /*#if defined( VDM ) || defined( VCONN_SUPPORT)*/
#if defined(_VDM)

/**
* @brief  Request the PE to perform a VDM Attention.
* @param  PortNum The current port number
* @param  SOPType SOP Type
* @param  SVID    Used SVID
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestAttention(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID)
{
  __DPM_DEBUG_CALLBACK(PortNum, "USBPD_PE_SVDM_RequestAttention");
  return USBPD_PE_SVDM_RequestAttention(PortNum, SOPType, SVID);
}

#endif /* _VDM */
#ifdef USBPD_REV30_SUPPORT
/**
* @brief  Request the PE to send an ALERT to port partner
* @param  PortNum The current port number
* @param  Alert   Alert based on @ref USBPD_ADO_TypeDef
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestAlert(uint8_t PortNum, USBPD_ADO_TypeDef Alert)
{
#if _ALERT
  return USBPD_PE_Request_DataMessage(PortNum, USBPD_DATAMSG_ALERT, (uint32_t*)&Alert.d32);
#else
  return USBPD_ERROR; /* Not supported by stack */
#endif /* _ALERT */
}

/**
* @brief  Request the PE to get a source capability extended
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetSourceCapabilityExt(uint8_t PortNum)
{
  return USBPD_ERROR; /* Not supported by stack */
}


/**
* @brief  Request the PE to get a sink capability extended
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetSinkCapabilityExt(uint8_t PortNum)
{
  return USBPD_ERROR; /* Not supported by stack */
}

/**
* @brief  Request the PE to get a manufacturer infor
* @param  PortNum The current port number
* @param  SOPType SOP Type
* @param  pManuInfoData Pointer on manufacturer info based on @ref USBPD_GMIDB_TypeDef
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetManufacturerInfo(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint8_t* pManuInfoData)
{
#if _MANU_INFO
  return USBPD_PE_SendExtendedMessage(PortNum, SOPType, USBPD_EXT_GET_MANUFACTURER_INFO, (uint8_t*)pManuInfoData, sizeof(USBPD_GMIDB_TypeDef));
#else
  return USBPD_ERROR; /* Not supported by stack */
#endif /* _MANU_INFO */
}

/**
* @brief  Request the PE to request a GET_PPS_STATUS
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetPPS_Status(uint8_t PortNum)
{
  return USBPD_ERROR; /* Not supported by stack */
}

/**
* @brief  Request the PE to request a GET_STATUS
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetStatus(uint8_t PortNum)
{
#if _STATUS
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_STATUS, USBPD_SOPTYPE_SOP);
#else
  return USBPD_ERROR; /* Not supported by stack */
#endif /* _STATUS */
}

/**
* @brief  Request the PE to perform a Fast Role Swap.
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestFastRoleSwap(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_FR_SWAP, USBPD_SOPTYPE_SOP);
}

/**
* @brief  Request the PE to send a GET_COUNTRY_CODES message
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetCountryCodes(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_COUNTRY_CODES, USBPD_SOPTYPE_SOP);
}

/**
* @brief  Request the PE to send a GET_COUNTRY_INFO message
* @param  PortNum     The current port number
* @param  CountryCode Country code (1st character and 2nd of the Alpha-2 Country)
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetCountryInfo(uint8_t PortNum, uint16_t CountryCode)
{
  return USBPD_PE_Request_DataMessage(PortNum, USBPD_DATAMSG_GET_COUNTRY_INFO, (uint32_t*)&CountryCode);
}

/**
* @brief  Request the PE to send a GET_BATTERY_CAPA
* @param  PortNum         The current port number
* @param  pBatteryCapRef  Pointer on the Battery Capability reference
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryCapability(uint8_t PortNum, uint8_t *pBatteryCapRef)
{
#if _BATTERY
  return USBPD_PE_SendExtendedMessage(PortNum, USBPD_SOPTYPE_SOP, USBPD_EXT_GET_BATTERY_CAP, (uint8_t*)pBatteryCapRef, 1);
#else
  return USBPD_ERROR; /* Not supported by stack */
#endif /* _BATTERY */
}

/**
* @brief  Request the PE to send a GET_BATTERY_STATUS
* @param  PortNum           The current port number
* @param  pBatteryStatusRef Pointer on the Battery Status reference
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryStatus(uint8_t PortNum, uint8_t *pBatteryStatusRef)
{
#if _BATTERY
  return USBPD_PE_SendExtendedMessage(PortNum, USBPD_SOPTYPE_SOP, USBPD_EXT_GET_BATTERY_STATUS, (uint8_t*)pBatteryStatusRef, 1);
#else
  return USBPD_ERROR; /* Not supported by stack */
#endif /* _BATTERY */
}

/**
* @brief  Request the PE to send a SECURITY_REQUEST
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestSecurityRequest(uint8_t PortNum)
{
  return USBPD_OK;
}

/**
* @brief  Request the PE to send a FIRWMARE_UPDATE_REQUEST or FIRWMARE_UPDATE_RESPONSE
* @param  PortNum     The current port number
* @param  MessageType Type of the message (REQUEST or RESPONSE)
* @param  pPayload    Pointer of the Payload to send to port partner
* @param  Size        Size of the payload
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestFirwmwareUpdate(uint8_t PortNum, USBPD_ExtendedMsg_TypeDef MessageType, uint8_t *pPayload, uint16_t Size)
{
#if defined(_FWUPDATE_INITIATOR) || defined(_FWUPDATE_RESPONDER)
  return USBPD_PE_SendExtendedMessage(PortNum, USBPD_SOPTYPE_SOP, MessageType, pPayload, Size);
#else
  return USBPD_ERROR; /* Not supported by stack */
#endif /* _FWUPDATE_INITIATOR || _FWUPDATE_RESPONDER */
}
#endif /*USBPD_REV30_SUPPORT*/

/**
* @}
*/


/** @addtogroup USBPD_USER_PRIVATE_FUNCTIONS
* @{
*/

/* USER CODE BEGIN USBPD_USER_PRIVATE_FUNCTIONS */

uint32_t USBPD_DPM_Store_Source_PDOs(uint8_t PortNum,uint8_t index, uint32_t SrcPDO)
{
  USBPD_PDO_TypeDef srcpdo;
  srcpdo.d32 = SrcPDO;
      switch(srcpdo.GenericPDO.PowerObject)
  {
    /* SRC Fixed Supply PDO */
  case USBPD_CORE_PDO_TYPE_FIXED:
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_SourceVoltagemaxPDOs[index]=srcpdo.SRCFixedPDO.VoltageIn50mVunits * 50;
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_SourceVoltageminPDOs[index]=srcpdo.SRCFixedPDO.VoltageIn50mVunits * 50;
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_SourceCurrentPDOs[index]=srcpdo.SRCFixedPDO.MaxCurrentIn10mAunits*10;
    break;
  case USBPD_CORE_PDO_TYPE_VARIABLE:
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_SourceVoltagemaxPDOs[index]=srcpdo.SRCVariablePDO.MaxVoltageIn50mVunits * 50;
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_SourceVoltageminPDOs[index]=srcpdo.SRCVariablePDO.MinVoltageIn50mVunits * 50;
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_SourceCurrentPDOs[index]=srcpdo.SRCVariablePDO.MaxCurrentIn10mAunits*10;
    break;
  }
 return 0; 
}
/**
* @brief  Examinate a given SRC PDO to check if matching with SNK capabilities.
* @param  PortNum             Port number
* @param  SrcPDO              Selected SRC PDO (32 bits)
* @param  PtrRequestedVoltage Pointer on Voltage value that could be reached if SRC PDO is requested (only valid if USBPD_TRUE is returned) in mV
* @param  PtrRequestedPower   Pointer on Power value that could be reached if SRC PDO is requested (only valid if USBPD_TRUE is returned) in mW
* @retval USBPD_FALSE of USBPD_TRUE (USBPD_TRUE returned in SRC PDO is considered matching with SNK profile)
*/
uint32_t USBPD_DPM_SNK_EvaluateMatchWithSRCPDO(uint8_t PortNum, uint32_t SrcPDO, uint32_t* PtrRequestedVoltage, uint32_t* PtrRequestedPower)
{
  USBPD_PDO_TypeDef  srcpdo, snkpdo;
  uint32_t match = USBPD_FALSE;
  uint32_t nbsnkpdo;
  uint32_t snkpdo_array[USBPD_MAX_NB_PDO];
  uint16_t i =0;
  uint16_t srcvoltage50mv =0;
  uint16_t srcmaxvoltage50mv =0;
  uint16_t srcminvoltage50mv =0;
  uint16_t srcmaxcurrent10ma =0;
  uint16_t snkvoltage50mv =0;
  uint16_t snkmaxvoltage50mv =0;
  uint16_t snkminvoltage50mv =0;
  uint16_t snkopcurrent10ma =0;
  uint32_t maxrequestedpower =0;
  uint32_t currentrequestedpower =0;
  uint32_t srcmaxcurrent=0;
  uint32_t maxrequestedvoltage =0;
  uint32_t currentrequestedvoltage=0;
  uint32_t srcpdomaxpower=0;
  uint32_t snkoppower250mw=0;
  uint32_t srcmaxpower250mw=0;

  /* Retrieve SNK PDO list from PWR_IF storage : PDO values + nb of u32 written by PWR_IF (nb of PDOs) */
  USBPD_PWR_IF_GetPortPDOs(PortNum, USBPD_CORE_DATATYPE_SNK_PDO, (uint8_t*)snkpdo_array, &nbsnkpdo);
  
  if (0 == nbsnkpdo)
  {
    return(USBPD_FALSE);
  }

  /* Set default output values */
  maxrequestedpower    = 0;
  maxrequestedvoltage  = 0;
  
  /* Check SRC PDO value according to its type */
  srcpdo.d32 = SrcPDO;
  switch(srcpdo.GenericPDO.PowerObject)
  {
    /* SRC Fixed Supply PDO */
  case USBPD_CORE_PDO_TYPE_FIXED:
    srcvoltage50mv = srcpdo.SRCFixedPDO.VoltageIn50mVunits;
    srcmaxcurrent10ma = srcpdo.SRCFixedPDO.MaxCurrentIn10mAunits;
    srcpdomaxpower = srcvoltage50mv * srcmaxcurrent10ma /2; /* max power available for this PDO*/
 /*   srcminvoltage = srcvoltage50mv *50;*/
    srcmaxcurrent = srcmaxcurrent10ma*10;
    /* Loop through SNK PDO list */
//    currentrequestedpower = 0;
//    currentrequestedvoltage = 0;
    for (i=0; i<nbsnkpdo; i++)
    {
      currentrequestedpower = 0;
      currentrequestedvoltage = 0;
      
      /* Retrieve SNK PDO value according to its type */
      snkpdo.d32 = snkpdo_array[i];
      switch(snkpdo.GenericPDO.PowerObject)
      {
        /* SNK Fixed Supply PDO */
      case USBPD_CORE_PDO_TYPE_FIXED:
        {
          snkvoltage50mv = snkpdo.SNKFixedPDO.VoltageIn50mVunits;
          snkopcurrent10ma = snkpdo.SNKFixedPDO.OperationalCurrentIn10mAunits;
          
          /* Match if :
          SNK Voltage = SRC Voltage
          &&
          SNK Op Current <= SRC Max Current
          
          Requested Voltage : SNK Voltage
          Requested Op Current : SNK Op Current
          Requested Max Current : SNK Op Current
          */
          if (  (snkvoltage50mv == srcvoltage50mv)
              &&(snkopcurrent10ma <= srcmaxcurrent10ma))
          {
            currentrequestedpower = (snkvoltage50mv * snkopcurrent10ma) / 2; /* to get value in mw */
            currentrequestedvoltage = snkvoltage50mv *50;
            DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_CurrentPDOSink_OperatingPowerinmW = currentrequestedpower;
          }
          break;
        }
        /* SNK Variable Supply (non-battery) PDO */
      case USBPD_CORE_PDO_TYPE_VARIABLE:
        snkmaxvoltage50mv = snkpdo.SNKVariablePDO.MaxVoltageIn50mVunits;
        snkminvoltage50mv = snkpdo.SNKVariablePDO.MinVoltageIn50mVunits;
        snkopcurrent10ma  = snkpdo.SNKVariablePDO.OperationalCurrentIn10mAunits;
        
        /* Match if :
        SNK Max voltage >= SRC Voltage
        &&
        SNK Min voltage <= SRC Voltage
        &&
        SNK Op current <= SRC Max current
        
        Requested Voltage : SRC Voltage
        Requested Op Current : SNK Op Current
        Requested Max Current : SNK Op Current
        */
        if (  (snkmaxvoltage50mv >= srcvoltage50mv)
            &&(snkminvoltage50mv <= srcvoltage50mv)
              &&(snkopcurrent10ma  <= srcmaxcurrent10ma))
        {
          currentrequestedpower = (srcvoltage50mv * snkopcurrent10ma) / 2; /* to get value in mw */
          currentrequestedvoltage = srcvoltage50mv *50;
        }
        break;
        
        /* SNK Battery Supply PDO */
      case USBPD_CORE_PDO_TYPE_BATTERY:
        snkmaxvoltage50mv = snkpdo.SNKBatteryPDO.MaxVoltageIn50mVunits;
        snkminvoltage50mv = snkpdo.SNKBatteryPDO.MinVoltageIn50mVunits;
        snkoppower250mw   = snkpdo.SNKBatteryPDO.OperationalPowerIn250mWunits;
        
        /* Match if :
        SNK Max voltage >= SRC Voltage
        &&
        SNK Min voltage <= SRC Voltage
        &&
        SNK Op power <= SRC Max current * SRC Voltage
        
        Requested Voltage : SRC Voltage
        Requested Op Current : SNK Op Power/ SRC Voltage
        Requested Max Current : SNK Op Power/ SRC Voltage
        */
        if (  (snkmaxvoltage50mv >= srcvoltage50mv)
            &&(snkminvoltage50mv <= srcvoltage50mv)
              &&(snkoppower250mw <= ((srcvoltage50mv * srcmaxcurrent10ma)/500)))  /* to get value in 250 mw units */
        {
          currentrequestedpower = (srcvoltage50mv * snkopcurrent10ma) / 2; /* to get value in mw */
          currentrequestedvoltage = srcvoltage50mv *50;
        }
        break;
        
        
      default:
        break;
      }
      
      if ((currentrequestedpower <= srcpdomaxpower) && (0 != currentrequestedpower))
      {
        match = USBPD_TRUE;
        maxrequestedpower   = currentrequestedpower;
        maxrequestedvoltage = currentrequestedvoltage;
      }
    } /*End  Loop through SNK PDO list */
    break; /* end SRC_PDO= USBPD_CORE_PDO_TYPE_FIXED*/
    
    /* SRC Variable Supply (non-battery) PDO */
  case USBPD_CORE_PDO_TYPE_VARIABLE:
    srcmaxvoltage50mv = srcpdo.SRCVariablePDO.MaxVoltageIn50mVunits;
    srcminvoltage50mv = srcpdo.SRCVariablePDO.MinVoltageIn50mVunits;
    srcmaxcurrent10ma = srcpdo.SRCVariablePDO.MaxCurrentIn10mAunits;
    srcpdomaxpower = srcmaxvoltage50mv * srcmaxcurrent10ma /2; /* max power available for this PDO*/
  /*  srcminvoltage = srcminvoltage50mv *50;   */
    srcmaxcurrent = srcmaxcurrent10ma*10;

/* Loop through SNK PDO list */
    for (i=0; i<nbsnkpdo; i++)
    {
      currentrequestedpower = 0;
      currentrequestedvoltage = 0;
      
      /* Retrieve SNK PDO value according to its type */
      snkpdo.d32 = snkpdo_array[i];
      switch(snkpdo.GenericPDO.PowerObject)
      {
        /* SNK Fixed Supply PDO */
      case USBPD_CORE_PDO_TYPE_FIXED:
        /* No match */
        break;
        
        /* SNK Variable Supply (non-battery) PDO */
      case USBPD_CORE_PDO_TYPE_VARIABLE:
        snkmaxvoltage50mv = snkpdo.SNKVariablePDO.MaxVoltageIn50mVunits;
        snkminvoltage50mv = snkpdo.SNKVariablePDO.MinVoltageIn50mVunits;
        snkopcurrent10ma  = snkpdo.SNKVariablePDO.OperationalCurrentIn10mAunits;
        
        /* Match if :
        SNK Max voltage >= SRC Max Voltage
        &&
        SNK Min voltage <= SRC Min Voltage
        &&
        SNK Op current <= SRC Max current
        
        Requested Voltage : Any value between SRC Min Voltage and SRC Max Voltage
        Requested Op Current : SNK Op Current
        Requested Max Current : SNK Op Current
        */
        if (  (snkmaxvoltage50mv >= srcmaxvoltage50mv)
            &&(snkminvoltage50mv <= srcminvoltage50mv)
              &&(snkopcurrent10ma <= srcmaxcurrent10ma))
        {
          currentrequestedpower = (srcmaxvoltage50mv * snkopcurrent10ma) / 2; /* to get value in mw */
          currentrequestedvoltage = srcmaxvoltage50mv *50;
        }
        break;
        
        /* SNK Battery Supply PDO */
      case USBPD_CORE_PDO_TYPE_BATTERY:
        snkmaxvoltage50mv = snkpdo.SNKBatteryPDO.MaxVoltageIn50mVunits;
        snkminvoltage50mv = snkpdo.SNKBatteryPDO.MinVoltageIn50mVunits;
        snkoppower250mw   = snkpdo.SNKBatteryPDO.OperationalPowerIn250mWunits;
        
        /* Match if :
        SNK Max voltage >= SRC Max Voltage
        &&
        SNK Min voltage <= SRC Min Voltage
        &&
        SNK Op power <= SRC Max current * SRC Max Voltage
        
        Requested Voltage : Any value between SRC Min Voltage and SRC Max Voltage, that fulfill
        SNK Op power <= Voltage * SRC Max Current
        Requested Op Current : SNK Op Power/ SRC Voltage
        Requested Max Current : SNK Op Power/ SRC Voltage
        */
        if (  (snkmaxvoltage50mv >= srcmaxvoltage50mv)
            &&(snkminvoltage50mv <= srcminvoltage50mv)
              &&(snkoppower250mw <= ((srcmaxvoltage50mv * srcmaxcurrent10ma)/500)))  /* to get value in 250 mw units */
        {
          currentrequestedpower   = snkoppower250mw * 250; /* to get value in mw */
          currentrequestedvoltage = srcmaxvoltage50mv *50;
        }
        break;
        
        
      default:
        break;
      }
      
      if (currentrequestedpower <= srcpdomaxpower)
      {
        match = USBPD_TRUE;
        maxrequestedpower   = currentrequestedpower;
        maxrequestedvoltage = currentrequestedvoltage;
      }
    } /*End  Loop through SNK PDO list */
    break; /* end SRC_PDO=USBPD_CORE_PDO_TYPE_VARIABLE*/
    
    /* SRC Battery Supply PDO */
  case USBPD_CORE_PDO_TYPE_BATTERY:
    srcmaxvoltage50mv = srcpdo.SRCBatteryPDO.MaxVoltageIn50mVunits;
    srcminvoltage50mv = srcpdo.SRCBatteryPDO.MinVoltageIn50mVunits;
    srcmaxpower250mw  = srcpdo.SRCBatteryPDO.MaxAllowablePowerIn250mWunits;
 /*   srcminvoltage = srcminvoltage50mv *50;    */

    /* Loop through SNK PDO list */
    for (i=0; i<nbsnkpdo; i++)
    {
      currentrequestedpower = 0;
      currentrequestedvoltage = 0;
      
      /* Retrieve SNK PDO value according to its type */
      snkpdo.d32 = snkpdo_array[i];
      switch(snkpdo.GenericPDO.PowerObject)
      {
        /* SNK Fixed Supply PDO */
      case USBPD_CORE_PDO_TYPE_FIXED:
        /* No match */
        break;
        
        /* SNK Variable Supply (non-battery) PDO */
      case USBPD_CORE_PDO_TYPE_VARIABLE:
        snkmaxvoltage50mv = snkpdo.SNKVariablePDO.MaxVoltageIn50mVunits;
        snkminvoltage50mv = snkpdo.SNKVariablePDO.MinVoltageIn50mVunits;
        snkopcurrent10ma  = snkpdo.SNKVariablePDO.OperationalCurrentIn10mAunits;
        
        /* Match if :
        SNK Max voltage >= SRC Max Voltage
        &&
        SNK Min voltage <= SRC Min Voltage
        &&
        SNK Op current * SRC Max Voltage <= SRC Max Power
        
        Requested Voltage : Any value between SRC Min Voltage and SRC Max Voltage : SRC Max Voltage
        Requested Op Current : SNK Op Current
        Requested Max Current : SNK Op Current
        */
        if (  (snkmaxvoltage50mv >= srcmaxvoltage50mv)
            &&(snkminvoltage50mv <= srcminvoltage50mv)
              &&(srcmaxvoltage50mv * snkopcurrent10ma <= srcmaxpower250mw))
        {
          currentrequestedpower = (srcmaxvoltage50mv * snkopcurrent10ma) / 2; /* to get value in mw */
          currentrequestedvoltage = srcmaxvoltage50mv*50 ;
        }
        break;
        
        /* SNK Battery Supply PDO */
      case USBPD_CORE_PDO_TYPE_BATTERY:
        snkmaxvoltage50mv = snkpdo.SNKBatteryPDO.MaxVoltageIn50mVunits;
        snkminvoltage50mv = snkpdo.SNKBatteryPDO.MinVoltageIn50mVunits;
        snkoppower250mw   = snkpdo.SNKBatteryPDO.OperationalPowerIn250mWunits;
        
        /* Match if :
        SNK Max voltage >= SRC Max Voltage
        &&
        SNK Min voltage <= SRC Min Voltage
        &&
        SNK Op power <= SRC Max power
        
        Requested Voltage : Any value between SRC Min Voltage and SRC Max Voltage, that fulfill
        SNK Op power <= Voltage * SRC Max Current
        Requested Op Current : SNK Op Power/ SRC Voltage
        Requested Max Current : SNK Op Power/ SRC Voltage
        */
        if (  (snkmaxvoltage50mv >= srcmaxvoltage50mv)
            &&(snkminvoltage50mv <= srcminvoltage50mv)
              &&(snkoppower250mw <= srcmaxpower250mw))
        {
          currentrequestedpower   = snkoppower250mw * 250; /* to get value in mw */
          currentrequestedvoltage = srcmaxvoltage50mv *50;
        }
        break;
        
        
      default:
        break;
      }
      
      if (currentrequestedpower <= srcpdomaxpower)
      {
        match = USBPD_TRUE;
        maxrequestedpower   = currentrequestedpower;
        maxrequestedvoltage = currentrequestedvoltage;
      }
    }/*End  Loop through SNK PDO list */
    break;/* end SRC_PDO=USBPD_CORE_PDO_TYPE_BATTERY*/
    
    
  default:
    return(USBPD_FALSE);
  }
  
  if (maxrequestedpower > 0)
  {

    *PtrRequestedPower   = maxrequestedpower;
    *PtrRequestedVoltage = maxrequestedvoltage ;//* 50 ; /* value in mV */
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_CurrentPDOSource_Current = srcmaxcurrent;
  }
  return(match);
}

/**
* @brief  Find PDO index that offers the most amount of power and in accordance with SNK capabilities.
* @param  PortNum Port number
* @param  PtrRequestPowerDetails  Sink requested power details structure pointer
* @retval Index of PDO within source capabilities message (DPM_NO_SRC_PDO_FOUND indicating not found)
*/
static uint32_t DPM_FindVoltageIndex(uint32_t PortNum, USBPD_DPM_SNKPowerRequestDetails_TypeDef* PtrRequestPowerDetails)
{
  uint32_t *ptpdoarray;
  USBPD_PDO_TypeDef  pdo;
  uint32_t voltage, reqvoltage, nbpdo, allowablepower, maxpower, opcurrent;
  uint32_t curr_index = DPM_NO_SRC_PDO_FOUND, temp_index;
  USBPD_USER_SettingsTypeDef *puser = (USBPD_USER_SettingsTypeDef *)&DPM_USER_Settings[PortNum];
  
  allowablepower = 0;
  maxpower       = DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MaxOperatingPowerInmWunits; /*!<max power defined for the application in PDP */
  reqvoltage     = 0;
  voltage        = 0;
  opcurrent      = 0;
  
  /* Search PDO index among Source PDO of Port */
  nbpdo = DPM_Ports[PortNum].DPM_NumberOfRcvSRCPDO; 
  ptpdoarray = DPM_Ports[PortNum].DPM_ListOfRcvSRCPDO;
  
  /* search the better PDO in the list of source PDOs */
  for(temp_index = 0; temp_index < nbpdo; temp_index++)
  {
    pdo.d32 = ptpdoarray[temp_index];
    /* check if the received source PDO is matching any of the SNK PDO */
    allowablepower = 0;
    if (USBPD_TRUE == USBPD_DPM_SNK_EvaluateMatchWithSRCPDO(PortNum, pdo.d32, &voltage, &allowablepower))
    {
      /* choose the "better" PDO, in this case only the distance in absolute value from the target voltage */
      if (allowablepower <= DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MaxOperatingPowerInmWunits)
      {
        /* Add additional check for compatibility of this SRC PDO with port characteristics (defined in DPM_USER_Settings) */
        if (  (voltage >= puser->DPM_SNKRequestedPower.MinOperatingVoltageInmVunits)
            &&(voltage <= puser->DPM_SNKRequestedPower.MaxOperatingVoltageInmVunits))
        {
          /* consider the current PDO the better one until now */
           curr_index = temp_index;
          reqvoltage = voltage;
          if (0!=maxpower)
          {
            opcurrent = MIN(maxpower,DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_CurrentPDOSink_OperatingPowerinmW) *1000 / reqvoltage;
          }          
          else
          {opcurrent = DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_CurrentPDOSink_OperatingPowerinmW *1000 / reqvoltage;
          maxpower   = allowablepower;
          }
        }
      }
    }
  }
  
  if (curr_index != DPM_NO_SRC_PDO_FOUND)
  {
    PtrRequestPowerDetails->MaxOperatingCurrentInmAunits = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits;
    PtrRequestPowerDetails->OperatingCurrentInmAunits    = opcurrent;
    PtrRequestPowerDetails->MaxOperatingPowerInmWunits   = maxpower;
    PtrRequestPowerDetails->RequestedVoltageInmVunits    = reqvoltage;
  }
  
  return curr_index;
}

/**
* @brief  Build RDO to be used in Request message according to selected PDO from received SRC Capabilities
* @param  PortNum           Port number
* @param  IndexSrcPDO       Index on the selected SRC PDO (value between 0 to 6)
* @param  PtrRequestPowerDetails  Sink requested power details structure pointer
* @param  Rdo               Pointer on the RDO
* @param  PtrPowerObject    Pointer on the selected power object
* @retval None
*/
void DPM_SNK_BuildRDOfromSelectedPDO(uint8_t PortNum, uint8_t IndexSrcPDO, USBPD_DPM_SNKPowerRequestDetails_TypeDef* PtrRequestPowerDetails,
                                     USBPD_SNKRDO_TypeDef* Rdo, USBPD_CORE_PDO_Type_TypeDef *PtrPowerObject)
{
  uint32_t mv = 0, mw = 0, ma = 0, size;
  USBPD_PDO_TypeDef  pdo;
  USBPD_SNKRDO_TypeDef rdo;
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];
  USBPD_USER_SettingsTypeDef *puser = (USBPD_USER_SettingsTypeDef *)&DPM_USER_Settings[PortNum];
  uint32_t snkpdolist[USBPD_MAX_NB_PDO];
  USBPD_PDO_TypeDef snk_fixed_pdo;
  
  /* Initialize RDO */
  rdo.d32 = 0;
  /* Read SNK PDO list for retrieving useful data to fill in RDO */
  USBPD_PWR_IF_GetPortPDOs(PortNum, USBPD_CORE_DATATYPE_SNK_PDO, (uint8_t*)&snkpdolist[0], &size);
  /* Store value of 1st SNK PDO (Fixed) in local variable */
  snk_fixed_pdo.d32 = snkpdolist[0];
  
  /* Set common fields in RDO */
  pdo.d32 = pdhandle->DPM_ListOfRcvSRCPDO[0];
  rdo.GenericRDO.USBCommunicationsCapable     = snk_fixed_pdo.SNKFixedPDO.USBCommunicationsCapable;
#if defined(USBPD_REV30_SUPPORT) && defined(_UNCHUNKED_SUPPORT)
  /* USBPD_REV30_SUPPORT && UNCHUNKED_SUPPORT */
  if (USBPD_SPECIFICATION_REV2 < DPM_Params[PortNum].PE_SpecRevision)
  {
    rdo.FixedVariableRDO.UnchunkedExtendedMessage = DPM_Settings[PortNum].PE_PD3_Support.d.PE_UnchunkSupport;
    DPM_Params[PortNum].PE_UnchunkSupport   = USBPD_FALSE;
    /* Set unchuncked bit if supported by port partner;*/
    if (USBPD_TRUE == pdo.SRCFixedPDO.UnchunkedExtendedMessage)
    {
      DPM_Params[PortNum].PE_UnchunkSupport   = USBPD_TRUE;
    }
  }
 /* USBPD_REV30_SUPPORT && UNCHUNKED_SUPPORT */
#endif 
  /* If no valid SNK PDO or if no SRC PDO match found (index>=nb of valid received SRC PDOs */
  if ((size < 1) || (IndexSrcPDO >= pdhandle->DPM_NumberOfRcvSRCPDO))
  {
    /* USBPD_DPM_EvaluateCapabilities: Mismatch, could not find desired pdo index */
#ifdef _TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t*)"DPM_SNK_BuildRDOfromSelectedPDO: Pb in SRC PDO selection", sizeof("DPM_SNK_BuildRDOfromSelectedPDO: Pb in SRC PDO selection"));
#endif /* _TRACE */
    rdo.FixedVariableRDO.ObjectPosition = 1;
    rdo.FixedVariableRDO.OperatingCurrentIn10mAunits  = pdo.SRCFixedPDO.MaxCurrentIn10mAunits;
    rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits / 10;
    rdo.FixedVariableRDO.CapabilityMismatch           = 1;
    rdo.FixedVariableRDO.USBCommunicationsCapable     = snk_fixed_pdo.SNKFixedPDO.USBCommunicationsCapable;
    DPM_Ports[PortNum].DPM_RequestedCurrent           = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits;
    /* USBPD_DPM_EvaluateCapabilities: Mismatch, could not find desired pdo index */
    
    pdhandle->DPM_RequestDOMsg = rdo.d32;
    return;
  }
  /* Set the Object position */
  rdo.GenericRDO.ObjectPosition               = IndexSrcPDO + 1;
  rdo.GenericRDO.NoUSBSuspend                 = 1;
  
  /* Extract power information from Power Data Object */
  pdo.d32 = pdhandle->DPM_ListOfRcvSRCPDO[IndexSrcPDO];
  
  *PtrPowerObject = pdo.GenericPDO.PowerObject;
  
  /* Retrieve request details from SRC PDO selection */
  mv = PtrRequestPowerDetails->RequestedVoltageInmVunits;
  ma = PtrRequestPowerDetails->OperatingCurrentInmAunits;
  
  switch(pdo.GenericPDO.PowerObject)
  {
  case USBPD_CORE_PDO_TYPE_FIXED:
  case USBPD_CORE_PDO_TYPE_VARIABLE:
    {
      /* USBPD_DPM_EvaluateCapabilities: Mismatch, less power offered than the operating power */
      ma = USBPD_MIN(ma, puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits);
      mw = (ma * mv)/1000; /* mW */
      DPM_Ports[PortNum].DPM_RequestedCurrent           = ma;
      rdo.FixedVariableRDO.OperatingCurrentIn10mAunits  = ma / 10;
      rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = MIN(DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits , DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_CurrentPDOSource_Current) / 10; /* !< Min between MaxSnk current and Source PDO current */
      if(mw < puser->DPM_SNKRequestedPower.OperatingPowerInmWunits)
      {
        /* USBPD_DPM_EvaluateCapabilities: Mismatch, less power offered than the operating power */
        rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits = puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits / 10;
        rdo.FixedVariableRDO.CapabilityMismatch = 1;
      }
    }
    break;
    
  case USBPD_CORE_PDO_TYPE_BATTERY:
    {
      /* USBPD_DPM_EvaluateCapabilities: Battery Request Data Object */
      mw = USBPD_MIN(PtrRequestPowerDetails->OperatingPowerInmWunits, puser->DPM_SNKRequestedPower.MaxOperatingPowerInmWunits); /* mW */
      ma = (1000 * mw) / mv; /* mA */
      ma = USBPD_MIN(ma, puser->DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits);
      DPM_Ports[PortNum].DPM_RequestedCurrent       = ma;
      mw = (ma * mv)/1000; /* mW */
      rdo.BatteryRDO.OperatingPowerIn250mWunits     = mw / 250;
      rdo.BatteryRDO.MaxOperatingPowerIn250mWunits  = mw / 250;
      if(mw < puser->DPM_SNKRequestedPower.OperatingPowerInmWunits)
      {
        /* Mismatch, less power offered than the operating power */
        rdo.BatteryRDO.CapabilityMismatch = 1;
      }
    }
    break;
    
  default:
    break;
  }
  
  pdhandle->DPM_RequestDOMsg = rdo.d32;
  pdhandle->DPM_RDOPosition  = rdo.GenericRDO.ObjectPosition;
  
  Rdo->d32 = pdhandle->DPM_RequestDOMsg;
  /* Get the requested voltage */
  pdhandle->DPM_RequestedVoltage = mv;
}

/**
* @brief  Turn Off power supply.
* @param  PortNum The current port number
* @param  Role    Port power role
* @retval USBPD_OK, USBPD_ERROR
*/
static USBPD_StatusTypeDef DPM_TurnOffPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role)
{
  USBPD_StatusTypeDef status = USBPD_OK;
  
  status = USBPD_PWR_IF_VBUSDisable(PortNum);
#ifdef USBPD_LED_SERVER
  Led_Set(LED_PORT_VBUS(PortNum) , LED_MODE_OFF, 0);
#endif /* USBPD_LED_SERVER */
  return status;
}

/**
* @brief  Turn On power supply.
* @param  PortNum The current port number
* @param  Role    Port power role
* @retval USBPD_ACCEPT, USBPD_WAIT, USBPD_REJECT
*/
static USBPD_StatusTypeDef DPM_TurnOnPower(uint8_t PortNum, USBPD_PortPowerRole_TypeDef Role)
{
  USBPD_StatusTypeDef status = USBPD_OK;
  /* Enable the output */
  status = USBPD_PWR_IF_VBUSEnable(PortNum);
  {
    /* stop current sink */
  }
  
  
  
  return status;
}

/**
* @brief  Assert Rp resistor.
* @param  PortNum The current port number
* @retval None
*/
static void DPM_AssertRp(uint8_t PortNum)
{
  USBPD_CAD_AssertRp(PortNum);
  
#ifdef USBPD_LED_SERVER
  Led_Set(LED_PORT_ROLE(PortNum) , LED_MODE_BLINK_ROLE_SRC, 0);
#endif /* USBPD_LED_SERVER */
}

/**
* @brief  EXTI line detection callback.
* @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
* @retval None
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13)
  {
    USBPD_DPM_RequestPowerRoleSwap(USBPD_PORT_0);
  }
  else
  {                                       
    USBPD_HW_IF_EXTI_Callback(GPIO_Pin);
  }
#if defined(_VDM)
#endif /* VDM & SNK */
}
static uint32_t CheckDPMTimers(void)
{
  uint32_t _timing = osWaitForever;
#if defined(USBPD_REV30_SUPPORT) && (_SRC_CAPA_EXT || _ALERT) || defined(_CLASS_BB) || (_ADC_MONITORING)
  uint32_t _current_timing;
  
  /* Calculate the minimum timers to wake-up DPM tasks */
  for(uint8_t instance = 0; instance < USBPD_PORT_COUNT; instance++)
  {
#ifdef _CLASS_BB
    _current_timing = DPM_Ports[instance].DPM_TimerAME & DPM_TIMER_READ_MSK;
    if(_current_timing > 0)
    {
      if (_current_timing < _timing)
      {
        _timing = _current_timing;
      }
    }
#endif /* _CLASS_BB */
#if _ALERT
    _current_timing = DPM_Ports[instance].DPM_TimerAlert & DPM_TIMER_READ_MSK;
    if(_current_timing > 0)
    {
      if (_current_timing < _timing)
      {
        _timing = _current_timing;
      }
    }
#endif /* ALERT */
#if _VDM
    _current_timing = DPM_Ports[instance].DPM_TimerDisco & DPM_TIMER_READ_MSK;
    if(_current_timing > 0)
    {
      if (_current_timing < _timing)
      {
        _timing = _current_timing;
      }
    }
#endif /* Disco */
   _current_timing = DPM_Ports[instance].DPM_TimerRetry_DRswap & DPM_TIMER_READ_MSK;
    if(_current_timing > 0)
    {
      if (_current_timing < _timing)
      {
        _timing = _current_timing;
      }
    }
     _current_timing = DPM_Ports[instance].DPM_TimerRetry_PRswap & DPM_TIMER_READ_MSK;
    if(_current_timing > 0)
    {
      if (_current_timing < _timing)
      {
        _timing = _current_timing;
      }
    }
#if _ADC_MONITORING
    _current_timing = DPM_Ports[instance].DPM_TimerADC & DPM_TIMER_READ_MSK;
    if(_current_timing > 0)
    {
      if (_current_timing < _timing)
      {
        _timing = _current_timing;
      }
    }
#endif /* _ADC */
  }
#endif /* USBPD_REV30_SUPPORT */
  return _timing;
}


#if defined(USBPD_REV30_SUPPORT)

#if _ADC_MONITORING
void DPM_ManageADC(void)
{
  for(uint8_t _instance = 0; _instance < USBPD_PORT_COUNT; _instance++)
  {
    /* check if ADC timer is expired */
    if (DPM_TIMER_ENABLE_MSK == DPM_Ports[_instance].DPM_TimerADC)
    {
      DPM_START_TIMER(_instance, DPM_TimerADC, DPM_TIMER_ADC);
      DPM_Ports[_instance].DPM_MeasuredCurrent = APPLI_GetIvbus(_instance);
      if (DPM_Ports[_instance].DPM_MeasuredCurrent > (DPM_Ports[_instance].OCP_Limit + 300))
      {
        USBPD_DPM_RequestHardReset(_instance);
      }
    }
  }
}
#endif
#if _ALERT
/**
* @brief  Manage the ALERT.
* @retval none
*/
void DPM_ManageAlert(void)
{
  for(uint8_t _instance = 0; _instance < USBPD_PORT_COUNT; _instance++)
  {
    /* check if Alert timer is expired */
    if (DPM_TIMER_ENABLE_MSK == DPM_Ports[_instance].DPM_TimerAlert)
    {
      /* Restart alert timer */
      DPM_START_TIMER(_instance, DPM_TimerAlert, DPM_TIMER_ALERT);
      DPM_Ports[_instance].DPM_MeasuredCurrent = APPLI_GetIvbus(_instance);
      if (DPM_Ports[_instance].DPM_MeasuredCurrent > 3600)
      {
        USBPD_DPM_RequestHardReset(_instance);
      }
      else
      {
        if (DPM_Ports[_instance].DPM_MeasuredCurrent > 3400)
        {
          if (0 == (DPM_Ports[_instance].DPM_SendAlert.b.TypeAlert & USBPD_ADO_TYPE_ALERT_OCP))
          {
            USBPD_ADO_TypeDef alert = {0};
            alert.b.TypeAlert = USBPD_ADO_TYPE_ALERT_OCP;
            USBPD_DPM_RequestAlert(_instance, alert);
            DPM_Ports[_instance].DPM_SendAlert.b.TypeAlert |= alert.b.TypeAlert;
          }
        }
        else
        {
          /* Reset of the OCP bit */
          DPM_Ports[_instance].DPM_SendAlert.b.TypeAlert &= ~USBPD_ADO_TYPE_ALERT_OCP;
        }
      }
    }
  }
}

#endif /* ALERT */
#endif /* USBPD_REV30_SUPPORT */

#if defined (USBPD_USBDATA)
/**
  * @brief  Initialize billboard driver
  * @param  None
  * @retval status
  */
 USBPD_StatusTypeDef DPM_USB_Init(void)
{
#if defined(_CLASS_BB)
  USBD_BosDescTypedef *pUSBBosDesc = (USBD_BosDescTypedef *)hUSBDBOSDesc;

  /* Init Device Library,Add Supported Class and Start the library*/
  if(USBD_OK != USBD_Init(&hUsbDevice[USBPD_PORT_0], &BB_Desc, 0))
  {
    return USBPD_ERROR;
  }

  if(USBD_OK != USBD_RegisterClass(&hUsbDevice[USBPD_PORT_0], USBD_BB_CLASS))
  {
    return USBPD_ERROR;
  }

  pUSBBosDesc->wTotalLength = 0x19U;
  pUSBBosDesc->bNumDeviceCaps = 0x1U;

#elif (_CLASS_HID)
  /* Init HID Application */
  USBD_Init(&hUsbDevice[USBPD_PORT_0], &HID_Desc, 0);

  /* initialize the HID class */
  /* Add Supported Class */
  USBD_RegisterClass(&hUsbDevice[USBPD_PORT_0], USBD_HID_CLASS);

#if !defined(USBPDCORE_LIB_NO_PD) || defined(USBPD_TYPE_STATE_MACHINE)
#else
  USBD_ConfigDescTypedef *pConfDesc;
  pConfDesc = (USBD_ConfigDescTypedef *)hUsbDevice[USBPD_PORT_0].pConfDesc;
  pConfDesc->bMaxPower = 0x0U;

#endif /* !USBPDCORE_LIB_NO_PD  || USBPD_TYPE_STATE_MACHINE */
#endif
#if defined(_CLASS_CDC)
   /* Init Device Library */
  USBD_Init(&hUsbDevice[USBPD_PORT_0], &VCP_Desc, 0);
  
  /* Add Supported Class */
  USBD_RegisterClass(&hUsbDevice[USBPD_PORT_0], USBD_CDC_CLASS);
  
  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&hUsbDevice[USBPD_PORT_0], &USBD_CDC_fops);
#if !defined(USBPDCORE_LIB_NO_PD) || defined(USBPD_TYPE_STATE_MACHINE)
#else
  USBD_ConfigDescTypedef *pConfDesc;
  pConfDesc = (USBD_ConfigDescTypedef *)hUsbDevice[USBPD_PORT_0].pConfDesc;
  pConfDesc->bMaxPower = 0x0U;

#endif /* !USBPDCORE_LIB_NO_PD  || USBPD_TYPE_STATE_MACHINE */
#endif

  for(uint8_t index = 0; index < USBPD_PORT_COUNT; index++)
  {
    DPM_Ports[index].DPM_USBState = 0;
  }
  return USBPD_OK;
}
#if defined(_CLASS_BB)
void DPM_USB_BB_Start(uint32_t PortNum)
{
   USBD_BosBBCapDescTypedef *pCapDesc;
   USBD_BosDescTypedef *pUSBBosDesc = (USBD_BosDescTypedef *)hUSBDBOSDesc;
   USBD_ConfigDescTypedef *pConfDesc;
   pCapDesc = USBD_BB_GetCapDesc(&hUsbDevice[PortNum], (uint8_t*)pUSBBosDesc);
   
         /* check if tAMETimeout is expired */
        if (DPM_TIMER_ENABLE_MSK == DPM_Ports[PortNum].DPM_TimerAME)
        {
          DPM_Ports[PortNum].DPM_TimerAME = 0;
#if defined(_VDM)
          /* Check if VDM failed */
          if (0 == VDM_Mode_On[PortNum])
          {
            pCapDesc->bmConfigured[0] = CONFIGURATION_UNSUCCESSFUL;
          }
          else
#endif
          {
            pCapDesc->bmConfigured[0] = CONFIGURATION_SUCCESSFUL;
          }

          pConfDesc = (USBD_ConfigDescTypedef *)hUsbDevice[PortNum].pConfDesc;
          pConfDesc->bMaxPower = 0x0U;
          
          /* Expose the billboard driver */
          pUSBBosDesc->wTotalLength = USB_SIZ_BOS_DESC;
          pUSBBosDesc->bNumDeviceCaps = 0x4;
          DPM_Ports[PortNum].DPM_USBState = 1;
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "BB class exposed", sizeof("USB start  case ufp"));
#endif
        
        }  
}
#endif
/**
  * @brief  check if BILLBAORD usb driver must be started.
  * @retval none
  */

void DPM_USB_Start(uint32_t PortNum)
{
#if defined(_CLASS_HID)||defined(_CLASS_CDC)
  USBD_ConfigDescTypedef *pConfDesc;
#endif /* USB_HID */
#if defined(_VDM) && defined(_CLASS_BB)
  USBD_BosDescTypedef *pUSBBosDesc = (USBD_BosDescTypedef *)hUSBDBOSDesc;
  
  /* Billboard Cap desc and Alternate mode desc can be updated by user if needed */

#endif /* VDM && CLASS_BB */
#if defined(_CLASS_BB)  
  pUSBBosDesc->wTotalLength = 0x19U;
  pUSBBosDesc->bNumDeviceCaps = 0x1U;
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "dpm_usb_start", sizeof("dpm_usb_start"));
#endif
#endif  
    switch(DPM_Params[PortNum].PE_DataRole)
    {
    case USBPD_PORTDATAROLE_UFP :
      {
#if defined _CLASS_BB
         USBD_Start(&hUsbDevice[PortNum]);
         DPM_Ports[PortNum].DPM_USBState = 1;
#endif
#if defined(_CLASS_HID)||defined(_CLASS_CDC)
        /* Start HID driver */
#if defined(_TRACE)
#if _CLASS_HID
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "USB HID start", sizeof("USb HID start"));
#else
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "USB CDC start", sizeof("USb HID start"));
#endif
#endif /* _TRACE */
        pConfDesc = (USBD_ConfigDescTypedef *)hUsbDevice[PortNum].pConfDesc;
        pConfDesc->bMaxPower = 0x0U;
        DPM_Ports[PortNum].DPM_USBState = 1;
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "USB start case ufp", sizeof("USB start  case ufp"));
#endif
        USBD_Start(&hUsbDevice[PortNum]);
#endif /* CLASS_HID or CLASS_CDC*/

        break;
      }
    case USBPD_PORTDATAROLE_DFP :
      {
        /* start USB host here */
        break;
      }
    default :
      {
        break;
      }
    }
}

void DPM_USB_Stop(uint32_t PortNum)
{
  if(DPM_Ports[PortNum].DPM_USBState == 1)
  {
#if defined(_CLASS_BB) || defined(_CLASS_HID) ||defined(_CLASS_CDC)
#if defined(_TRACE)
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, 0, 0,(uint8_t *)"USB::STOP", 10);
#endif /* _TRACE */
     if (DPM_Ports[PortNum].DPM_USBState != 0)
      {
#if defined(_CLASS_BB)  
    DPM_Ports[PortNum].DPM_TimerAME =0;    
#endif
    USBD_Stop(&hUsbDevice[PortNum]);

    DPM_Ports[PortNum].DPM_USBState = 0;
     }
#endif
  }

  if(DPM_Ports[PortNum].DPM_USBState == 2)
  {
    /* Stop the host */
  }
}

#endif /* _USB */

/* USER CODE END USBPD_USER_PRIVATE_FUNCTIONS */

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
