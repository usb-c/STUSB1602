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
#if  defined(_VCONN_SUPPORT)
#include "usbpd_vdm_user.h"
#endif /*defined( VDM ) || defined( VCONN_SUPPORT)*/
#include "usbpd_pwr_if.h"
#if defined(USBPD_LED_SERVER)
#include "led_server.h"
#endif /* USBPD_LED_SERVER */
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
#define DPM_START_TIMER(_PORT_, _TIMER_,_TIMEOUT_)  DPM_Ports[_PORT_]._TIMER_ = (_TIMEOUT_) |  DPM_TIMER_ENABLE_MSK;
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

#if defined(USBPD_REV30_SUPPORT)
#if _ADC_MONITORING
void DPM_ManageADC(void);
#endif
#endif /* USBPD_REV30_SUPPORT */
#if defined(USBPD_LED_SERVER)
#error /* LED_Server not available without RTOS */
#endif
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
  
  if (DPM_Settings[USBPD_PORT_0].CAD_RoleToggle == USBPD_TRUE)
  {
    Led_Set(LED_PORT_ROLE(USBPD_PORT_0),LED_MODE_BLINK_ROLE_DRP, 0);
  }
  else
    /* Set the power role */
    Led_Set(LED_PORT_ROLE(USBPD_PORT_0),((DPM_Settings[USBPD_PORT_0].PE_DefaultRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC),0);
#if  USBPD_PORT_COUNT == 2
  if (DPM_Settings[USBPD_PORT_1].CAD_RoleToggle == USBPD_TRUE)
  {
    Led_Set(LED_PORT_ROLE(USBPD_PORT_1),LED_MODE_BLINK_ROLE_DRP, 0);
  }
  else
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
  if (15 <= USBPD_PDP_SRC_IN_WATTS)
  {
    DPM_Settings[USBPD_PORT_0].CAD_DefaultResistor = vRp_3_0A;
#if USBPD_PORT_COUNT == 2
    DPM_Settings[USBPD_PORT_1].CAD_DefaultResistor = vRp_3_0A;
#endif /* USBPD_PORT_COUNT == 2 */
  }
  else
    if (7 == USBPD_PDP_SRC_IN_WATTS)
    {
      DPM_Settings[USBPD_PORT_0].CAD_DefaultResistor = vRp_1_5A;
#if USBPD_PORT_COUNT == 2
      DPM_Settings[USBPD_PORT_1].CAD_DefaultResistor = vRp_1_5A;
#endif /* USBPD_PORT_COUNT == 2 */
    }
    else
    {
      DPM_Settings[USBPD_PORT_0].CAD_DefaultResistor = vRp_Default;
#if USBPD_PORT_COUNT == 2
      DPM_Settings[USBPD_PORT_1].CAD_DefaultResistor = vRp_Default;
#endif /* USBPD_PORT_COUNT == 2 */
    }
  CAD_Set_default_ResistorRp(USBPD_PORT_0,DPM_Settings[USBPD_PORT_0].CAD_DefaultResistor);
#if USBPD_PORT_COUNT == 2
  CAD_Set_default_ResistorRp(USBPD_PORT_1,DPM_Settings[USBPD_PORT_1].CAD_DefaultResistor);
#endif /* USBPD_PORT_COUNT == 2 */
  
  
  
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
  HAL_Delay(Time);
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

uint16_t current_meas[USBPD_PORT_COUNT][5];      

void USBPD_DPM_UserExecute(void const *argument)
{
if (DPM_TIMER_ENABLE_MSK == DPM_Ports[USBPD_PORT_0].DPM_TimerExecute)
  {
    /* Restart alert timer */
    DPM_START_TIMER(USBPD_PORT_0, DPM_TimerExecute, DPM_TIMER_EXECUTE);
    
    /* check current sink by Snk **/   
    uint8_t PortNum = USBPD_PORT_0; 
    if  (  0 == DPM_Ports[USBPD_PORT_0].DPM_FlagHardResetOngoing)
    {
      if (0 == DPM_Ports[PortNum].DPM_FlagSetupNewPowerOngoing && (USBPD_POWER_TRANSITION != DPM_Params[PortNum].PE_Power))
      {
         /* average current calculation */
              uint8_t i;
            uint32_t average_current, sum_current=0;
            for (i=0;i<5 ;i++)
            {
              sum_current= current_meas[PortNum][i]+ sum_current;
            }
            average_current = sum_current / 5;
            
            DPM_Ports[PortNum].DPM_MeasuredCurrent = average_current; /* APPLI_GetIvbus(PortNum);*/
            DPM_Ports[PortNum].DPM_MeasuredVbus = APPLI_GetVBUS(PortNum);
      }
    }
    if (USBPD_POWER_EXPLICITCONTRACT == DPM_Params[PortNum].PE_Power)
    {
    if ( (DPM_USER_Settings[PortNum].CurrentMeas > DPM_USER_Settings[PortNum].OCP_Limit +300) && (DPM_Ports[PortNum].DPM_MeasuredVbus !=0) && (  0 == DPM_Ports[PortNum].DPM_FlagHardResetOngoing) && ( 1 == DPM_Ports[PortNum].DPM_RDOPosition))
      { 
#ifdef _TRACE
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, USBPD_PORT_0, 0, (uint8_t *) "ErrorReco voltage", sizeof("ErrorReco PDO current"));
#endif    
        USBPD_DPM_EnterErrorRecovery(PortNum);
        __NOP();
      }
    }
  }
}

USBPD_StatusTypeDef USBPD_Retry_DRSWAP(uint8_t PortNum)
{
  USBPD_StatusTypeDef status = USBPD_OK;
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
  return status;
}

USBPD_StatusTypeDef USBPD_Retry_PRSWAP(uint8_t PortNum)
{
    USBPD_StatusTypeDef status = USBPD_OK;
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
    /* reset all values received from port partner */
    memset(&DPM_Ports[PortNum], 0, sizeof(DPM_Ports[PortNum]));
    DPM_Ports[PortNum].DPM_origine = 5000;
#if defined(_OPTIM_CONSO)
    /* Switch to 8Mhz*/
    SystemClock_Config_8Mhz();
    FlagExplicitContract = 0;
#endif /* _OPTIM_CONSO */
    if(USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
    {
      if (USBPD_OK != USBPD_PWR_IF_VBUSDisable(PortNum))
      {
        /* Should not occurr */
        while(1);
      }
    }
    
    
/*STUSB1602 specific */
    if (vRp_1_5A == DPM_Settings[PortNum].CAD_DefaultResistor)
      
    {
      APPLI_Set_Current_Limit(PortNum, 1500);
      DPM_Ports[PortNum].OCP_Limit = 1500;
    }
    else
    {
      if (vRp_3_0A == DPM_Settings[PortNum].CAD_DefaultResistor)
      {
        APPLI_Set_Current_Limit(PortNum,3000);
        DPM_Ports[PortNum].OCP_Limit = 3000;

      }
      else
      {
        APPLI_Set_Current_Limit(PortNum,500);
        DPM_Ports[PortNum].OCP_Limit = 500;
      }
    }
    /* Set back default profile */             
    DPM_Ports[PortNum].DPM_RequestedVoltage = 5000;
#ifdef _TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "EE", sizeof("EE"));
#endif
    
    USBPD_DPM_SetupNewPower(PortNum); 
    STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
    STUSB1602_VBUS_Presence_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Presence_Enable);
#ifdef _DEBUG_TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "range enable", sizeof("range enable"));
#endif
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
    if (DPM_Settings[PortNum].CAD_RoleToggle == USBPD_TRUE)
    {
      Led_Set(LED_PORT_ROLE(PortNum), LED_MODE_BLINK_ROLE_DRP, 0);
    }
    else
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
  
#ifdef USBPD_REV30_SUPPORT
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
#endif /* GENERATOR_ALERT */
   if((DPM_Ports[PortNum].DPM_TimerExecute & DPM_TIMER_READ_MSK) > 0)
  {
    DPM_Ports[PortNum].DPM_TimerExecute--;
  }
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
  DPM_Ports[PortNum].DPM_TimerRetry_DRswap    = 0;
  DPM_Ports[PortNum].DPM_TimerRetry_PRswap    = 0;
  DPM_Ports[PortNum].DPM_TimerExecute    = 0;

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
* @brief  Request the DPM to setup the new power level.
* @param  PortNum The current port number
* @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_DPM_SetupNewPower(uint8_t PortNum)
{
  /* USER CODE BEGIN USBPD_DPM_SetupNewPower */
  USBPD_StatusTypeDef status;
  uint8_t rdoposition, previous_rdoposition;
  
  /* Retrieve Request DO position from DPM handle : RDO position in the table of PDO (possible value from 1 to 7) */
  rdoposition = DPM_Ports[PortNum].DPM_RDOPosition;
  previous_rdoposition = DPM_Ports[PortNum].DPM_RDOPositionPrevious;
  
#ifdef _DEBUG_TRACE
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "Enter setupnewpower", sizeof("Enter setupnewpower"));
  uint8_t tab[32];
  uint8_t size;
  size = sprintf((char*)tab, "rdoposition: %d", rdoposition);
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size); 
#endif
  /* Check if get the right pdo position */
  if (rdoposition > 0)
  {
    
    if ((DPM_Ports[PortNum].DPM_OldRequestedVoltage == DPM_Ports[PortNum].DPM_RequestedVoltage) && (DPM_Ports[PortNum].DPM_RequestedCurrent_new > (DPM_Ports[PortNum].DPM_MeasuredCurrent + 225) )) 
    {
      DPM_Ports[PortNum].DPM_FlagSetupNewPowerOngoing = 0;
      status = USBPD_OK;
    }
    else
    {
      status = USBPD_PWR_IF_SetProfile(PortNum, rdoposition-1, previous_rdoposition);
    }    
  }
  else
  {
    /* Put it to VSafe5V */   
    DPM_Ports[PortNum].DPM_FlagSetupNewPowerOngoing = 1;
    status = (USBPD_StatusTypeDef)APPLI_SetVoltage(PortNum, 5000);
    DPM_Ports[PortNum].DPM_FlagSetupNewPowerOngoing = 0;
  }
  
  return status;
  /* USER CODE END USBPD_DPM_SetupNewPower */
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
case USBPD_NOTIFY_PE_DISABLED:
  {
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
#if defined(USBPD_REV30_SUPPORT)
#if defined(_ADC_MONITORING)
    DPM_START_TIMER(PortNum, DPM_TimerADC, DPM_TIMER_ADC);
#endif    
#endif /* USBPD_REV30_SUPPORT */
     if (USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
    {
      DPM_START_TIMER(PortNum, DPM_TimerExecute, DPM_TIMER_EXECUTE);
    }
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
    
  case USBPD_NOTIFY_STATE_SRC_READY:
    {
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
//  case USBPD_CORE_DATATYPE_REQUEST_DO :
//    if (Size == 4)
//    {
//      uint8_t* rdo;
//      rdo = (uint8_t*)&DPM_Ports[PortNum].DPM_RcvRequestDOMsg;
//      (void)memcpy(rdo, Ptr, Size);
//    }
//    break;
    
#if defined(USBPD_REV30_SUPPORT)
#endif /* USBPD_REV30_SUPPORT */
    
    /* In case of unexpected data type (Set request could not be fulfilled) :
    */
  default :
    break;
  }
  /* USER CODE END USBPD_DPM_SetDataInfo */
}

/**
* @brief  Evaluate received Request Message from Sink port
* @param  PortNum Port number
* @param  PtrPowerObject  Pointer on the power data object
* @retval USBPD status : USBPD_ACCEPT, USBPD_REJECT, USBPD_WAIT, USBPD_GOTOMIN
*/
USBPD_StatusTypeDef USBPD_DPM_EvaluateRequest(uint8_t PortNum, USBPD_CORE_PDO_Type_TypeDef *PtrPowerObject)
{
  /* USER CODE BEGIN USBPD_DPM_EvaluateRequest */
  USBPD_SNKRDO_TypeDef rdo;
  USBPD_PDO_TypeDef pdo;
  uint32_t pdomaxcurrent = 0;
  uint32_t rdomaxcurrent = 0, rdoopcurrent = 0, rdoobjposition = 0;
  uint32_t rdovoltage;
  USBPD_HandleTypeDef *pdhandle = &DPM_Ports[PortNum];
  
  rdo.d32 = pdhandle->DPM_RcvRequestDOMsg;
  rdoobjposition  = rdo.GenericRDO.ObjectPosition;
  pdhandle->DPM_RDOPosition = 0;
  
  DPM_Ports[PortNum].DPM_FlagHardResetOngoing = 0;
  /* Check if RDP can be met within the supported PDOs by the Source port */
  /* USBPD_DPM_EvaluateRequest: Evaluate Sink Request\r */
  /* USBPD_DPM_EvaluateRequest: Check if RDP can be met within the supported PDOs by the Source port\r */
  
  /* Search PDO in Port Source PDO list, that corresponds to Position provided in Request RDO */
  if (USBPD_PWR_IF_SearchRequestedPDO(PortNum, rdoobjposition, &pdo.d32) != USBPD_OK)
  {
    /* Invalid PDO index */
    /* USBPD_DPM_EvaluateRequest: Invalid PDOs index */
    return USBPD_REJECT;
  }
  
  switch(pdo.GenericPDO.PowerObject)
  {
  case USBPD_CORE_PDO_TYPE_FIXED:
    {
      pdomaxcurrent = pdo.SRCFixedPDO.MaxCurrentIn10mAunits;
      rdomaxcurrent = rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits;
      rdoopcurrent  = rdo.FixedVariableRDO.OperatingCurrentIn10mAunits;
      DPM_Ports[PortNum].DPM_RequestedCurrent = rdoopcurrent * 10;
      rdovoltage    = pdo.SRCFixedPDO.VoltageIn50mVunits * 50;
      DPM_Ports[PortNum].APDO_State = CV_State ;
      DPM_Ports[PortNum].DPM_Alertsent =0;
      
      if (0 != DPM_Ports[PortNum].DPM_RequestedVoltage)
        DPM_Ports[PortNum].DPM_OldRequestedVoltage = DPM_Ports[PortNum].DPM_RequestedVoltage;
      else
      {
        DPM_Ports[PortNum].DPM_OldRequestedVoltage = 5000;
        DPM_Ports[PortNum].DPM_origine = 5000;
        APPLI_SetVoltage(PortNum,DPM_Ports[PortNum].DPM_origine);
      }
      DPM_Ports[PortNum].DPM_RequestedVoltage = rdovoltage;
      DPM_Ports[PortNum].DPM_RequestedPDP = (uint32_t)((DPM_Ports[PortNum].DPM_RequestedCurrent ) * (DPM_Ports[PortNum].DPM_RequestedVoltage )/1000);
      DPM_Ports[PortNum].DPM_VBUSCC = DPM_Ports[PortNum].DPM_RequestedVoltage;
      if(rdoopcurrent > pdomaxcurrent)
      {
        /* Sink requests too much operating current */
        /* USBPD_DPM_EvaluateRequest: Sink requests too much operating current*/
        return USBPD_REJECT;
      }
      
      if(rdomaxcurrent > pdomaxcurrent)
      {
        /* Sink requests too much maximum operating current */
        /* USBPD_DPM_EvaluateRequest: Sink requests too much maximum operating current */
        return USBPD_REJECT;
      }
      else
      {
#if defined (_VCONN_SUPPORT)
        uint32_t cable_current,current_limit;
        if (VBUS_5A == DPM_Ports[PortNum].VDM_DiscoCableIdentify.CableVDO.b.VBUS_CurrentHandCap)
        {
          cable_current = 500;
        }
        else
        {
          cable_current = 300;
        }
        current_limit = 10 * USBPD_MIN(USBPD_MAX(rdomaxcurrent,pdomaxcurrent),cable_current);
        /* Set Current limitation according to cable capability and PDO selected */
        APPLI_Set_Current_Limit(PortNum,current_limit);
#else  
        uint32_t cable_current=3000;
        uint32_t current_limit;
        current_limit = 10 * USBPD_MIN(USBPD_MAX(rdomaxcurrent,pdomaxcurrent),cable_current);
        APPLI_Set_Current_Limit(PortNum,current_limit);
        
#endif /*defined (_VCONN_SUPPORT)*/ 
        DPM_Ports[PortNum].OCP_Limit = current_limit;

      }
    }
    break;
  case USBPD_CORE_PDO_TYPE_BATTERY:
  case USBPD_CORE_PDO_TYPE_VARIABLE:
  default:
    {
      return USBPD_REJECT;
    }
  }
  
  /* Set RDO position and requested voltage in DPM port structure */
//  pdhandle->DPM_RequestedVoltage = pdo.SRCFixedPDO.VoltageIn50mVunits * 50;
  pdhandle->DPM_RDOPositionPrevious = pdhandle->DPM_RDOPosition;
  pdhandle->DPM_RDOPosition = rdoobjposition;
  
  /* Save the power object */
  *PtrPowerObject = pdo.GenericPDO.PowerObject;
  
  /* Accept the requested power */
  /* USBPD_DPM_EvaluateRequest: Sink requested %d mV %d mA for operating current from %d to %d mA\r",
  pdo.SRCFixedPDO.VoltageIn50mVunits * 50, pdo.SRCFixedPDO.MaxCurrentIn10mAunits * 10,
  rdo.FixedVariableRDO.MaxOperatingCurrent10mAunits * 10, rdo.FixedVariableRDO.OperatingCurrentIn10mAunits * 10 */
  /* USBPD_DPM_EvaluateRequest: Source accepts the requested power */
  return USBPD_ACCEPT;
  /* USER CODE END USBPD_DPM_EvaluateRequest */
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

/**
* @brief  Power role swap status update
* @param  PortNum Port number
* @param  CurrentRole the current role
* @param  Status status on power role swap event
*/
void USBPD_DPM_PowerRoleSwap(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_PRS_Status_TypeDef Status)
{
  /* USER CODE BEGIN USBPD_DPM_PowerRoleSwap */
  uint16_t tvbusOn_max = 0x113; /*113hex is 275decimal.*/
  switch (Status)
  {
  case USBPD_PRS_STATUS_START_ACK:
    break;
  case USBPD_PRS_STATUS_START_REQ:
    break;
  case USBPD_PRS_STATUS_ACCEPTED:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_ACCEPTED)");
#endif 
    break;
  case USBPD_PRS_STATUS_REJECTED:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_REJECTED)");
#endif 
    break;
  case USBPD_PRS_STATUS_WAIT:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_WAIT)");
#endif 
    break;
  case USBPD_PRS_STATUS_VBUS_OFF:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_VBUS_OFF)");
#endif 
    USBPD_HW_IF_PRS_Vbus_OFF(PortNum, CurrentRole);
    DPM_TurnOffPower(PortNum, CurrentRole);
    break;
  case USBPD_PRS_STATUS_SRC_RP2RD:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_SRC_RP2RD)");
#endif 
    USBPD_HW_IF_PRS_Assert_Rd(PortNum, CurrentRole);
    break;
  case USBPD_PRS_STATUS_SRC_PS_READY_SENT:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_SRC_PS_READY_SENT)");
#endif
    break;
  case USBPD_PRS_STATUS_SNK_PS_READY_RECEIVED:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_SNK_PS_READY_RECEIVED)");
#endif
    break;
  case USBPD_PRS_STATUS_SNK_RD2RP:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_SNK_RD2RP)");
#endif
    USBPD_HW_IF_PRS_Assert_Rp(PortNum, CurrentRole);
    if (USBPD_HW_IF_CheckVbusValid(PortNum , tvbusOn_max ) == USBPD_TIMEOUT )
    {
      
#ifdef _TRACE
      USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "tVBUSon timer expire", sizeof("tVBUSon timer expire"));   
#else 
      __NOP();
#endif
    }
    
    break;
  case USBPD_PRS_STATUS_VBUS_ON:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_VBUS_ON)");
#endif
    
    DPM_TurnOnPower(PortNum, CurrentRole);
    break;
  case USBPD_PRS_STATUS_SNK_PS_READY_SENT:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_SNK_PS_READY_SENT)");
#endif  
    break;
  case USBPD_PRS_STATUS_SRC_PS_READY_RECEIVED:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_SRC_PS_READY_RECEIVED)");
#endif
    break;
  case USBPD_PRS_STATUS_COMPLETED:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_COMPLETED)");
#endif
    USBPD_HW_IF_PRS_End(PortNum, CurrentRole);
    STUSB16xx_HW_IF_Set_VBus_Monitoring(PortNum, DPM_Ports[PortNum].DPM_RequestedVoltage, 10, 10);
#ifdef USBPD_LED_SERVER
    /* Led feedback */
    Led_Set(LED_PORT_ROLE(PortNum) , ((DPM_Params[PortNum].PE_PowerRole == USBPD_PORTPOWERROLE_SNK) ? LED_MODE_BLINK_ROLE_SNK : LED_MODE_BLINK_ROLE_SRC), 0);
#endif /* USBPD_LED_SERVER */
    break;
  case USBPD_PRS_STATUS_FAILED:
  case USBPD_PRS_STATUS_ABORTED:
#if defined(_TRACE)    
    __DEBUG_CALLBACK(PortNum, "USBPD_PE_PRS(USBPD_PRS_STATUS_FAILED\\ABORTED)");
#endif
    break;
  default:
    break;
  }
  /* USER CODE END USBPD_DPM_PowerRoleSwap */
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
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_PR_SWAP, USBPD_SOPTYPE_SOP);
  
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

#if  defined(_VCONN_SUPPORT)
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
  return USBPD_PE_SVDM_RequestSpecific(PortNum, SOPType, SVDM_SPECIFIC_2, SVID);
}
#endif  /*#if defined( VDM ) || defined( VCONN_SUPPORT)*/
#ifdef USBPD_REV30_SUPPORT
/**
* @brief  Request the PE to send an ALERT to port partner
* @param  PortNum The current port number
* @param  Alert   Alert based on @ref USBPD_ADO_TypeDef
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestAlert(uint8_t PortNum, USBPD_ADO_TypeDef Alert)
{
  return USBPD_ERROR; /* Not supported by stack */
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
  return USBPD_PE_Request_CtrlMessage(PortNum, USBPD_CONTROLMSG_GET_SNK_CAPEXT, USBPD_SOPTYPE_SOP);
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
  return USBPD_ERROR; /* Not supported by stack */
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
  return USBPD_ERROR; /* Not supported by stack */
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
  return USBPD_ERROR; /* Not supported by stack */
}

/**
* @brief  Request the PE to send a GET_BATTERY_STATUS
* @param  PortNum           The current port number
* @param  pBatteryStatusRef Pointer on the Battery Status reference
* @retval USBPD Status
*/
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryStatus(uint8_t PortNum, uint8_t *pBatteryStatusRef)
{
  return USBPD_ERROR; /* Not supported by stack */
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
  
#ifdef USBPD_LED_SERVER
  /* Led feedback */
  Led_Set(LED_PORT_VBUS(PortNum) , LED_MODE_BLINK_VBUS, 0);
#endif /* USBPD_LED_SERVER */
  
  if(USBPD_PORTPOWERROLE_SRC == Role)
  {
    /* Wait for that VBUS is stable */
    status =  USBPD_HW_IF_CheckVbusValid(PortNum , 275);
    return  status;
    
  }
  else
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
