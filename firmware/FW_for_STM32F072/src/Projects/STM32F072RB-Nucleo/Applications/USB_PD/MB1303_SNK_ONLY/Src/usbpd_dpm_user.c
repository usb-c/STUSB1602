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
#include "usbpd_pwr_if.h"
#include "led_server.h"
#include "string.h"

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

#if !defined (RTOS)
#define DPM_TIMER_EXECUTE            2 /*!<2ms */      
#endif
#define DPM_NO_SRC_PDO_FOUND      0xFFU        /*!< No match found between Received SRC PDO and SNK capabilities                             */

#define DPM_GUI_NOTIF_ISCONNECTED       (1 << 5)
#define DPM_GUI_NOTIF_POWER_EVENT       (1 << 15)

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

#if defined(_OPTIM_CONSO)
volatile uint32_t FlagExplicitContract;
#endif /* _OPTIM_CONSO */
extern USBPD_ParamsTypeDef DPM_Params[USBPD_PORT_COUNT];
/*GENERATOR_USBPD_STUSB1602 specific */
uint8_t PE_is_explicit =0;
extern void CAD_Set_default_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue);
GUI_NOTIFICATION_POST         DPM_GUI_PostNotificationMessage   = NULL;
GUI_NOTIFICATION_FORMAT_SEND  DPM_GUI_FormatAndSendNotification = NULL;
GUI_SAVE_INFO                 DPM_GUI_SaveInfo                  = NULL;



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
#endif /* USBPD_REV30_SUPPORT */
/* USER CODE END USBPD_USER_PRIVATE_FUNCTIONS_Prototypes */

/**
* @}
*/
#include "usbpd_cad_hw_if.h"

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
  /* Led management initialization */
  Led_Init();
  
    /* Set the power role */
    Led_Set(LED_PORT_ROLE(USBPD_PORT_0),((DPM_Settings[USBPD_PORT_0].PE_DefaultRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC),0);
#if  USBPD_PORT_COUNT == 2
    Led_Set(LED_PORT_ROLE(USBPD_PORT_1),((DPM_Settings[USBPD_PORT_1].PE_DefaultRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC),0);
#endif
  
  /* PWR SET UP */
  if(USBPD_OK !=  USBPD_PWR_IF_Init())
  {
    return USBPD_ERROR;
  }
  
  /* VDM initialisation */
  
  if(USBPD_OK != USBPD_PWR_IF_PowerResetGlobal()) return USBPD_ERROR;
  
  
/*GENERATOR_USBPD_STUSB1602 specific */
  
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
* @brief  Function to set the function ptr linked to GUI interface
* @param  PtrFormatSend Pointer on function to format and send GUI notifications
* @param  PtrPost       Pointer on function to send GUI notifications
* @param  PtrSaveInfo   Pointer on function to save information from Port Partner
* @retval None
*/
void USBPD_DPM_SetNotification_GUI(GUI_NOTIFICATION_FORMAT_SEND PtrFormatSend, GUI_NOTIFICATION_POST PtrPost, GUI_SAVE_INFO PtrSaveInfo)
{
  DPM_GUI_PostNotificationMessage   = PtrPost;
  DPM_GUI_FormatAndSendNotification = PtrFormatSend;
  DPM_GUI_SaveInfo                  = PtrSaveInfo;
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
#if defined(USBPD_REV30_SUPPORT)

#endif /* USBPD_REV30_SUPPORT */
        
      }
      break;
      
    default:
      break;
    }
    _timing = CheckDPMTimers();
  }
  while(1);
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
    /* Format and send a notification to GUI if enabled */
    if (NULL != DPM_GUI_FormatAndSendNotification)
    {
      DPM_GUI_FormatAndSendNotification(PortNum, DPM_GUI_NOTIF_ISCONNECTED, 0);
    }
#if defined(_OPTIM_CONSO)
    /* Switch to 48Mhz*/
    SystemClock_Config_48Mhz();
    FlagExplicitContract = 0;
#endif /* _OPTIM_CONSO */
    
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
    
    /* Led feedback */
    Led_Set(LED_PORT_ROLE(PortNum) , (DPM_Params[PortNum].ActiveCCIs == CC1 ? LED_MODE_BLINK_CC1 : LED_MODE_BLINK_CC2), 0);
    Led_Set(LED_PORT_VBUS(PortNum) , LED_MODE_BLINK_VBUS, 0);
    Led_Set(LED_PORT_CC(PortNum) , ((DPM_Params[PortNum].PE_PowerRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC), 0);
    DPM_Ports[PortNum].DPM_IsConnected = 1;
    break;
    
  case USBPD_CAD_EVENT_ATTACHED:
    /* Format and send a notification to GUI if enabled */
    if (NULL != DPM_GUI_FormatAndSendNotification)
    {
      DPM_GUI_FormatAndSendNotification(PortNum, DPM_GUI_NOTIF_ISCONNECTED | DPM_GUI_NOTIF_POWER_EVENT, 0);
    }
#if defined(_OPTIM_CONSO)
    /* Switch to 48Mhz*/
    SystemClock_Config_48Mhz();
    FlagExplicitContract = 0;
#endif /* _OPTIM_CONSO */
    DPM_Ports[PortNum].DPM_origine = 5000;
    if(USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
    {
      if (USBPD_OK != USBPD_PWR_IF_VBUSEnable(PortNum))
      {
        /* Should not occurr */
        while(1);
      }
/* GENERATOR_USBPD_STUSB1602 specific */
      /* Wait for that VBUS is stable */
      USBPD_HW_IF_CheckVbusValid(PortNum , 40 );
    }
    /* Led feedback */
    Led_Set(LED_PORT_ROLE(PortNum) , (DPM_Params[PortNum].ActiveCCIs == CC1 ? LED_MODE_BLINK_CC1 : LED_MODE_BLINK_CC2), 0);
    Led_Set(LED_PORT_VBUS(PortNum), LED_MODE_BLINK_VBUS, 0);
    Led_Set(LED_PORT_CC(PortNum) , ((DPM_Params[PortNum].PE_PowerRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC), 0);
    DPM_Ports[PortNum].DPM_IsConnected = 1;
    DPM_Ports[PortNum].DPM_origine = 5000;
    break;
    
  case USBPD_CAD_EVENT_DETACHED :
  case USBPD_CAD_EVENT_EMC :
  default :
    /* reset all values received from port partner */
    memset(&DPM_Ports[PortNum], 0, sizeof(DPM_Ports[PortNum]));
    /* Format and send a notification to GUI if enabled */
    if (NULL != DPM_GUI_FormatAndSendNotification)
    {
      DPM_GUI_FormatAndSendNotification(PortNum, DPM_GUI_NOTIF_ISCONNECTED | DPM_GUI_NOTIF_POWER_EVENT, 0);
    }
    DPM_Ports[PortNum].DPM_origine = 5000;
#if defined(_OPTIM_CONSO)
    /* Switch to 8Mhz*/
    SystemClock_Config_8Mhz();
    FlagExplicitContract = 0;
#endif /* _OPTIM_CONSO */
    /* Led feedback */
    Led_Set(LED_PORT_CC(PortNum), LED_MODE_OFF, 0);
    Led_Set(LED_PORT_VBUS(PortNum) , LED_MODE_OFF, 0);
      /* Set the power role */
      Led_Set(LED_PORT_ROLE(PortNum),((DPM_Settings[PortNum].PE_DefaultRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC),              0);
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
  
#ifdef USBPD_REV30_SUPPORT
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
/*GENERATOR_USBPD_STUSB1602 specific */
  DPM_Ports[PortNum].DPM_RDOPositionPrevious   = 1;
  DPM_Ports[PortNum].DPM_RequestedVoltage = 5000;
  DPM_Ports[PortNum].DPM_MeasuredCurrent = 0;
  if (vRp_3_0A == DPM_Settings[PortNum].CAD_DefaultResistor )
    DPM_Ports[PortNum].DPM_RequestedCurrent = 3000;
  
  else
    if (vRp_1_5A == DPM_Settings[PortNum].CAD_DefaultResistor )
      DPM_Ports[PortNum].DPM_RequestedCurrent = 1500;
    else
      DPM_Ports[PortNum].DPM_RequestedCurrent = 500;
    
    DPM_Ports[PortNum].DPM_RequestedPDP = (uint32_t)((DPM_Ports[PortNum].DPM_RequestedCurrent ) * (DPM_Ports[PortNum].DPM_RequestedVoltage )) ;
    APPLI_Set_Current_Limit(PortNum,DPM_Ports[PortNum].DPM_RequestedCurrent); 
    DPM_Ports[PortNum].DPM_VBUSCC = DPM_Ports[PortNum].DPM_RequestedVoltage;
    DPM_Ports[PortNum].DPM_origine = 5000;
    
    switch(Status)
    {
    case USBPD_HR_STATUS_START_ACK:
#if defined(_TRACE)
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_START_ACK)");
#endif /* defined(_TRACE)*/
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
#if defined(_TRACE)
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_START_REQ)");
#endif /* defined(_TRACE)*/      
      if (USBPD_PORTPOWERROLE_SRC == CurrentRole)
      {
        /* Restore default Role in case of Power Swap failing due to no PS_READY from Sink (TC PC.E2)  */
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
#if defined(_TRACE)      
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_COMPLETED)");
#endif /* defined(_TRACE)*/      
      status = DPM_TurnOnPower(PortNum, CurrentRole);
      
      break;
    case USBPD_HR_STATUS_FAILED:
#if defined(_TRACE)      
      __DEBUG_CALLBACK(PortNum, "USBPD_PE_HardReset(USBPD_HR_STATUS_FAILED)");
#endif /*defined(_TRACE)      */
      USBPD_HW_IF_HR_End(PortNum, CurrentRole);
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
  case  USBPD_NOTIFY_SOFTRESET_SENT:
    {
         extern CAD_HW_HandleTypeDef CAD_HW_Handles[USBPD_PORT_COUNT];
    if (CAD_HW_Handles[PortNum].state == USBPD_CAD_STATE_DETACHED)
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
#if defined(USBPD_REV30_SUPPORT)
#endif /* USBPD_REV30_SUPPORT */
    /* Turn On VBUS LED when an explicit contract is established */
    Led_Set(LED_PORT_VBUS(PortNum) , LED_MODE_ON, 0);
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
    /* end Laert notif*/
    /*
    End REQUEST ANSWER NOTIFICATION
    ***************************************************************************/
  case USBPD_NOTIFY_STATE_SNK_READY:
    {
#if defined(USBPD_REV30_SUPPORT)
#endif /* USBPD_REV30_SUPPORT */
    }
    break;
    
  case USBPD_NOTIFY_STATE_SRC_DISABLED:
    {
      /* SINK Port Partner is not PD capable. Legacy cable may have been connected
      In this state, VBUS is set to 5V */
    }
    break;
  case USBPD_NOTIFY_POWER_SWAP_REJ:
    DPM_Ports[PortNum].DPM_FlagSendPRSwap = 1;
    break;
  case USBPD_NOTIFY_DATAROLESWAP_DFP:
    STUSB16xx_HW_IF_DataRoleSwap(PortNum);
    USBPD_HW_IF_DataRole(PortNum);
    break;
  case USBPD_NOTIFY_DATAROLESWAP_UFP :
    STUSB16xx_HW_IF_DataRoleSwap(PortNum);
    USBPD_HW_IF_DataRole(PortNum);
    break;
    
  case USBPD_NOTIFY_REQUEST_ENTER_MODE:
    __NOP();
    break;
  default :
    break;
  }
  
  /* Forward PE notifications to GUI if enabled */
  if (NULL != DPM_GUI_PostNotificationMessage)
  {
    DPM_GUI_PostNotificationMessage(PortNum, EventVal);
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
#if defined USBPD_CLI
    DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MinOperatingVoltageInmVunits = CLI_APDO_params.CLI_APDO_Voltagemin;
    DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MaxOperatingVoltageInmVunits = CLI_APDO_params.CLI_APDO_Voltagemax;
#endif
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
  case USBPD_CORE_DATATYPE_REQUEST_DO :
    if (Size == 4)
    {
      uint8_t* rdo;
      rdo = (uint8_t*)&DPM_Ports[PortNum].DPM_RcvRequestDOMsg;
      (void)memcpy(rdo, Ptr, Size);
    }
    break;
    
#if defined(USBPD_REV30_SUPPORT)
#endif /* USBPD_REV30_SUPPORT */
    
    /* In case of unexpected data type (Set request could not be fulfilled) :
    */
  default :
    break;
  }
  /* Forward info to GUI if enabled */
  if (NULL != DPM_GUI_SaveInfo)
  {
    DPM_GUI_SaveInfo(PortNum, DataId, Ptr, Size);
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
  
  
  /* Inform CAD to enter recovery mode */
  USBPD_CAD_EnterErrorRecovery(PortNum);
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
  return status;
  /* USER CODE END USBPD_DPM_EvaluateDataRoleSwap */
}

//#endif /* not _gui_interface*/
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

/**
* @brief  Request the PE to perform a VCONN Swap.
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestVconnSwap(uint8_t PortNum)
{
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_VCONN_SWAP, USBPD_SOPTYPE_SOP);
}

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

#ifdef USBPD_REV30_SUPPORT
/**
* @brief  Request the PE to send an ALERT to port partner
* @param  PortNum The current port number
* @param  Alert   Alert based on @ref USBPD_ADO_TypeDef
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestAlert(uint8_t PortNum, USBPD_ADO_TypeDef Alert)
{
  return USBPD_ERROR;
}

/**
* @brief  Request the PE to get a source capability extended
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetSourceCapabilityExt(uint8_t PortNum)
{
  return USBPD_ERROR;
}

/**
* @brief  Request the PE to get a sink capability extended
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetSinkCapabilityExt(uint8_t PortNum)
{
#if _SNK_CAPA_EXT
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_SNK_CAPEXT, USBPD_SOPTYPE_SOP);
#else
  return USBPD_ERROR;
#endif /* _SNK_CAPA_EXT */
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
  return USBPD_ERROR;
}

/**
* @brief  Request the PE to request a GET_PPS_STATUS
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetPPS_Status(uint8_t PortNum)
{
  return USBPD_ERROR;
}

/**
* @brief  Request the PE to request a GET_STATUS
* @param  PortNum The current port number
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetStatus(uint8_t PortNum)
{
  return USBPD_ERROR;
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
  return USBPD_ERROR;
}

/**
* @brief  Request the PE to send a GET_BATTERY_STATUS
* @param  PortNum           The current port number
* @param  pBatteryStatusRef Pointer on the Battery Status reference
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryStatus(uint8_t PortNum, uint8_t *pBatteryStatusRef)
{
  return USBPD_ERROR;
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
  return USBPD_ERROR;
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
  case USBPD_CORE_PDO_TYPE_APDO:
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_SourceVoltageminPDOs[index]=srcpdo.SRCSNKAPDO.MinVoltageIn100mV*100;
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_SourceVoltagemaxPDOs[index]=srcpdo.SRCSNKAPDO.MaxVoltageIn100mV*100;
    DPM_Ports[PortNum].DPM_CurrentPDOInfo.DPM_SourceCurrentPDOs[index]=srcpdo.SRCSNKAPDO.MaxCurrentIn50mAunits*50;   
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
  uint16_t i, srcvoltage50mv, srcmaxvoltage50mv, srcminvoltage50mv, srcmaxcurrent10ma;
  uint16_t snkvoltage50mv, snkmaxvoltage50mv, snkminvoltage50mv, snkopcurrent10ma/*,snkmaxvoltage100mv,snkminvoltage100mv,snkopcurrent50ma*/;
  uint32_t maxrequestedpower, currentrequestedpower,srcmaxcurrent;
  uint32_t maxrequestedvoltage, currentrequestedvoltage/*,srcpdominpower*/, srcpdomaxpower;
  uint32_t snkoppower250mw, srcmaxpower250mw;

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
   //         DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.OperatingPowerInmWunits = currentrequestedpower ;//nat *1000 / currentrequestedvoltage;
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
 //     if (allowablepower >= maxpower)
      if (allowablepower <= DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MaxOperatingPowerInmWunits)
      {
        /* Add additional check for compatibility of this SRC PDO with port characteristics (defined in DPM_USER_Settings) */
        if (  (voltage >= puser->DPM_SNKRequestedPower.MinOperatingVoltageInmVunits)
            &&(voltage <= puser->DPM_SNKRequestedPower.MaxOperatingVoltageInmVunits))
         //     &&(allowablepower <= puser->DPM_SNKRequestedPower.MaxOperatingPowerInmWunits))
        {
          /* consider the current PDO the better one until now */
           curr_index = temp_index;
          reqvoltage = voltage;
      //    maxpower   = allowablepower;
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
 //   PtrRequestPowerDetails->OperatingCurrentInmAunits    = (1000 * maxpower)/voltage;
    PtrRequestPowerDetails->OperatingCurrentInmAunits    = opcurrent;
    PtrRequestPowerDetails->MaxOperatingPowerInmWunits   = maxpower;
 //nat   PtrRequestPowerDetails->OperatingPowerInmWunits      = puser->DPM_SNKRequestedPower.MaxOperatingPowerInmWunits;
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
  Led_Set(LED_PORT_VBUS(PortNum) , LED_MODE_OFF, 0);
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
  
  Led_Set(LED_PORT_ROLE(PortNum) , LED_MODE_BLINK_ROLE_SRC, 0);
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
}

static uint32_t CheckDPMTimers(void)
{
  uint32_t _timing = osWaitForever;
  return _timing;
}

#if defined(USBPD_REV30_SUPPORT)

#endif /* USBPD_REV30_SUPPORT */

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
