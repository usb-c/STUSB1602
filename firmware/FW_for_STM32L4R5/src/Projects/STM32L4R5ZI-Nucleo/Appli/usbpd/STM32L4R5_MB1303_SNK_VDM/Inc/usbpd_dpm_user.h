/**
  ******************************************************************************
  * @file    usbpd_dpm_user.h
  * @author  MCD Application Team
  * @brief   Header file for usbpd_dpm_user.c file
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

#ifndef __USBPD_DPM_USER_H_
#define __USBPD_DPM_USER_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"

#include "cmsis_os.h"
#ifdef _VDM
#include "usbpd_vdm_user.h"
#endif /* _VDM */
/** @addtogroup STM32_USBPD_APPLICATION
  * @{
  */

/** @addtogroup STM32_USBPD_APPLICATION_DPM_USER
  * @{
  */
/* Exported typedef ----------------------------------------------------------*/
typedef struct
{
  USBPD_SNKPowerRequest_TypeDef DPM_SNKRequestedPower;          /*!< Requested Power by the sink board                     */
#if defined(USBPD_REV30_SUPPORT)
  uint32_t OCP_Limit;                                           /*!< SRC OCP limit                                         */   
  uint16_t CurrentMeas;
#if _MANU_INFO
  USBPD_MIDB_TypeDef  DPM_ManuInfoPort;                         /*!< Manufacturer information used for the port            */
  uint16_t            ReservedManu;                             /*!< Reserved bits to match with Manufacturer information            */
#endif /* _MANU_INFO */ 
#endif /* USBPD_REV30_SUPPORT */
  uint8_t DR_swap_rejected;
  uint32_t PE_DR_Swap_To_UFP                             : 1;  /*!< support data swap                                     */
  uint32_t PE_DR_Swap_To_DFP                             : 1;  /*!< support data swap                                     */
  uint32_t PE_VconnSwap                                   : 1;  /*!< support VCONN swap                                    */
  uint32_t Reserved1                                      :30;  /*!< Reserved bits */
} USBPD_USER_SettingsTypeDef;

typedef struct
{
  uint32_t XID;               /*!< Value provided by the USB-IF assigned to the product   */
  uint16_t VID;               /*!< Vendor ID (assigned by the USB-IF)                     */
  uint16_t PID;               /*!< Product ID (assigned by the manufacturer)              */
} USBPD_IdSettingsTypeDef;


typedef enum {
  DPM_USER_EVENT_TIMER,         /* TIMER EVENT */
  DPM_USER_EVENT_GUI,           /* GUI EVENT */
  DPM_USER_EVENT_CCFLASH,       /* CC FLASH FSM EVENT */
  DPM_USER_EVENT_NONE,          /* NO EVENT */
} DPM_USER_EVENT;

typedef enum
{
  CLP_State = 3 , /* APDO && not HWRESET && I > REQ && not I > OCP */
  CLM_State = 2 , /* APDO && not HWRESET && I > REQ && V > REQ */
  OCP_State = 1 , /* APDO && not HWRESET && I > REQ && I > OCP */
  CV_State = 0   /* APDO && not HWRESET && I > REQ && Not V > REQ */
}APDO_State_typedef;
uint32_t USBPD_DPM_Store_Source_PDOs(uint8_t PortNum,uint8_t index, uint32_t SrcPDO);

typedef struct
{
    uint32_t DPM_CurrentPDOSource_Voltage;
    uint32_t DPM_CurrentPDOSource_Current;
    uint32_t DPM_CurrentPDOSource_Power;
    uint32_t DPM_CurrentPDOSource_minVolt;
    uint32_t DPM_CurrentPDOSource_maxVolt;
    uint32_t DPM_CurrentPDOSource_MaxPowerFound; 
    uint32_t DPM_CurrentPDOSink_OperatingPowerinmW;
    uint32_t DPM_SourceVoltageminPDOs[7];
    uint32_t DPM_SourceVoltagemaxPDOs[7];
    uint32_t DPM_SourceCurrentPDOs[7];
  }USBPD_CurrentPDO_TypeDef;
/**
  * @brief  USBPD DPM handle Structure definition
  * @{
  */
typedef struct
{
  uint32_t                      DPM_ListOfRcvSRCPDO[USBPD_MAX_NB_PDO];   /*!< The list of received Source Power Data Objects from Port partner
                                                                              (when Port partner is a Source or a DRP port).                       */
  uint32_t                      DPM_NumberOfRcvSRCPDO;                   /*!< The number of received Source Power Data Objects from port Partner
                                                                              (when Port partner is a Source or a DRP port).
                                                                              This parameter must be set to a value lower than USBPD_MAX_NB_PDO    */
  uint32_t                      DPM_ListOfRcvSNKPDO[USBPD_MAX_NB_PDO];   /*!< The list of received Sink Power Data Objects from Port partner
                                                                              (when Port partner is a Sink or a DRP port).                         */
  uint32_t                      DPM_NumberOfRcvSNKPDO;                   /*!< The number of received Sink Power Data Objects from port Partner
                                                                              (when Port partner is a Sink or a DRP port).
                                                                              This parameter must be set to a value lower than USBPD_MAX_NB_PDO    */
  uint32_t                      DPM_RDOPosition;                         /*!< RDO Position of requested DO in Source list of capabilities          */
  uint32_t                      DPM_RequestedVoltage;                    /*!< Value of requested voltage                                           */
  uint32_t                      DPM_OldRequestedVoltage;                 /*!< Previous RDO voltage */
  uint32_t                      DPM_SrcOutputVoltage;                     /*! Source real output voltage setting */
  uint32_t                      DPM_RequestedCurrent;                    /*!< Value of requested current                                           */
  uint32_t                      DPM_RequestedCurrent_new;                    /*!< Value of requested current                                           */
  uint32_t                      DPM_RequestedPDP;                         /*!< Value of PDP requested>*/                        
  uint16_t                      DPM_MeasuredVbus;                        /*!< Value of measured Voltage on Vbus                                           */
  uint32_t                      DPM_VBUSCC;                               /*!<Vbus target because of CC */
  uint32_t                      DPM_origine;                             /*!<Vbus origine for transition calculation >*/
  uint32_t                      OCP_Limit;                                /*!<Vbus Overcurrent protection limit  >*/
  uint8_t                       DR_swap_rejected;                         /*!<device reject PR_swap>*/
#if defined(USBPD_REV30_SUPPORT)
  int16_t                       DPM_MeasuredCurrent;                     /*!< Value of measured current                                            */
#endif /* USBPD_REV30_SUPPORT */
  uint32_t                      DPM_RDOPositionPrevious;                 /*!< RDO Position of previous requested DO in Source list of capabilities */
  uint32_t                      DPM_RequestDOMsg;                        /*!< Request Power Data Object message to be sent                         */
  uint32_t                      DPM_RequestDOMsgPrevious;                /*!< Previous Request Power Data Object message to be sent                */
  uint32_t                      DPM_RcvRequestDOMsg;                     /*!< Received request Power Data Object message from the port Partner     */
  volatile uint32_t             DPM_ErrorCode;                           /*!< USB PD Error code                                                    */
  volatile uint8_t              DPM_IsConnected;                         /*!< USB PD connection state                                              */
  uint8_t                       DPM_FlagSendPRSwap;                      /*!< Flag used to detect if PR swap has been sent                         */
  uint8_t                       DPM_FlagHardResetOngoing;                 /*!< Flag used to inform that HardReset is on going                       */
  uint8_t                       DPM_FlagSetupNewPowerOngoing;            /*!< Flag used to inform power nego on going */
  uint8_t                       DPM_Alertsent;  
  APDO_State_typedef            APDO_State ;

#ifdef _CLASS_BB
  volatile uint16_t             DPM_TimerAME;                            /*!< tAMETimeout used to start USB Billboard class in SNK                 */
#endif /* USBPD_USBDATA */
#if defined(USBPD_REV30_SUPPORT)
#if _STATUS
  USBPD_SDB_TypeDef             DPM_RcvStatus;                           /*!< Status received by port partner                                      */
#endif /* _STATUS */
#if _MANU_INFO
  USBPD_GMIDB_TypeDef           DPM_GetManufacturerInfo;                 /*!< Get Manufacturer Info                                                */
#endif /* _MANU_INFO */
  USBPD_GBSDB_TypeDef           DPM_GetBatteryStatus;                    /*!< Get Battery status                                                   */
  USBPD_GBCDB_TypeDef           DPM_GetBatteryCapability;                /*!< Get Battery Capability                                               */
  USBPD_BSDO_TypeDef            DPM_BatteryStatus;                       /*!< Battery status                                                       */
#if _ALERT
  volatile uint16_t             DPM_TimerSend_Req;
  volatile uint16_t             DPM_TimerAlert;                          /*!< Timer used to monitor current and trig an ALERT                      */
  USBPD_ADO_TypeDef             DPM_SendAlert;                           /*!< Save the Alert sent to port partner                                  */
  USBPD_ADO_TypeDef             DPM_RcvAlert;                            /*!< Save the Alert received by port partner                              */
#endif /* _ALERT */
#if _VDM
  volatile uint16_t             DPM_TimerDisco;
#endif
    volatile uint16_t           DPM_TimerRetry_DRswap;
    volatile uint16_t           DPM_TimerRetry_PRswap;
    volatile uint8_t            DPM_DR_retry;
    volatile uint8_t            DPM_PR_retry;
#if _ADC_MONITORING
  volatile uint16_t             DPM_TimerADC;                           /*!< Timer to ask regular check vs ADC measurement*/
#endif
  USBPD_CurrentPDO_TypeDef             DPM_CurrentPDOInfo;                     /*!< Save the current PDO info for selection/sending best request          */
#endif /* USBPD_REV30_SUPPORT */
#ifdef _VCONN_SUPPORT
  USBPD_DiscoveryIdentity_TypeDef VDM_DiscoCableIdentify;                /*!< VDM Cable Discovery Identify                                         */
  uint16_t                      DPM_CablePDCapable:1;                    /*!< Flag to keep information that Cable may be PD capable                */
  uint16_t                      DPM_CableResetOnGoing:1;                 /*!< Flag to manage a cable reset on going                                */
  uint16_t                      DPM_Reserved:14;                         /*!< Reserved bytes                                                       */
#else
  uint16_t                      DPM_Reserved:16;                         /*!< Reserved bytes                                                       */
#endif /* _VCONN_SUPPORT */
#ifdef _VDM
  USBPD_DiscoveryIdentity_TypeDef   VDM_DiscoIdentify;                   /*!< VDM Discovery Identify                                               */
  USBPD_SVIDPortPartnerInfo_TypeDef VDM_SVIDPortPartner;                 /*!< VDM SVID list                                                        */
  USBPD_ModeInfo_TypeDef            VDM_ModesPortPartner;                /*!< VDM Modes list   */
#endif /* _VDM */
#ifdef USBPD_USBDATA
   uint8_t                          DPM_USBState;                           /*!< Value to keep the USB state : 0 stop, 1 device started, 2 host started */
#endif
} USBPD_HandleTypeDef;

typedef void     (*GUI_NOTIFICATION_POST)(uint8_t PortNum, uint16_t EventVal);
typedef uint32_t (*GUI_NOTIFICATION_FORMAT_SEND)(uint32_t PortNum, uint32_t TypeNotification, uint32_t Value);
typedef void     (*GUI_SAVE_INFO)(uint8_t PortNum, uint8_t DataId, uint8_t *Ptr, uint32_t Size);

/**
  * @}
  */

/* Exported define -----------------------------------------------------------*/
/*
 * USBPD FW version
 */
#define USBPD_FW_VERSION  0x05022019u

/*
 * USBPD Start Port Number
 */
#define USBPD_START_PORT_NUMBER  0u

/*
 * Number af thread defined by user to include in the low power control
 */
#define USBPD_USER_THREAD_COUNT    0
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

#if !defined(USBPD_DPM_USER_C)
extern USBPD_HandleTypeDef DPM_Ports[USBPD_PORT_COUNT];
#else
USBPD_HandleTypeDef DPM_Ports[USBPD_PORT_COUNT] =
{
  {
    .DPM_Reserved = 0,
#if defined(USBPD_REV30_SUPPORT)
#if _MANU_INFO
    .DPM_GetManufacturerInfo = {0},                 /*!< Get Manufacturer Info                                                */
#endif /* _MANU_INFO */
    .DPM_GetBatteryStatus = {0},                    /*!< Get Battery status                                                   */
    .DPM_GetBatteryCapability = {0},                /*!< Get Battery Capability                                               */
    .DPM_BatteryStatus = {0},                       /*!< Battery status                                                       */
#if _ALERT
    .DPM_TimerAlert = 0,                            /*!< Timer used to monitor current and trig an ALERT                      */
    .DPM_SendAlert = {0},                           /*!< Save the Alert sent to port partner                                  */
    .DPM_RcvAlert = {0},                            /*!< Save the Alert received by port partner                              */
#endif /* _ALERT */
#if _VDM
    .DPM_TimerDisco = 0,
#endif
    .DPM_TimerRetry_DRswap =0,
    .DPM_TimerRetry_PRswap =0,
#if _ADC_MONITORING
    .DPM_TimerADC = 0,
#endif
    .DPM_CurrentPDOInfo ={0},                       /*!< Store Current PDO info  from Source                                  */
#endif /* USBPD_REV30_SUPPORT */

#if USBPD_PORT_COUNT >= 2
  },
  {
    .DPM_Reserved = 0,
#if defined(USBPD_REV30_SUPPORT)
#if _MANU_INFO
    .DPM_GetManufacturerInfo = {0},                 /*!< Get Manufacturer Info                                                */
#endif /* _MANU_INFO */
#if _BATTERY
    .DPM_GetBatteryStatus = {0},                    /*!< Get Battery status                                                   */
    .DPM_GetBatteryCapability = {0},                /*!< Get Battery Capability                                               */
    .DPM_BatteryStatus = {0},                       /*!< Battery status                                                       */
#endif /* _BATTERY */
#if _ALERT
    .DPM_TimerAlert = 0,                            /*!< Timer used to monitor current and trig an ALERT                      */
    .DPM_SendAlert = {0},                           /*!< Save the Alert sent to port partner                                  */
    .DPM_RcvAlert = {0},                            /*!< Save the Alert received by port partner                              */
#endif /* _ALERT */
#if _VDM
    .DPM_TimerDisco = 0,
#endif
    .DPM_TimerRetry_PRswap =0,
    .DPM_TimerRetry_DRswap =0,
#if _ADC_MONITORING
    .DPM_TimerADC = 0,
#endif
    .DPM_CurrentPDOInfo ={0},                       /*!< Store Current PDO info  from Source                                  */
#endif /* USBPD_REV30_SUPPORT */

  }
#else
  }
#endif /*USBPD_PORT_COUNT >= 2*/
};
#endif /* !USBPD_DPM_USER_C */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup USBPD_USER_EXPORTED_FUNCTIONS
  * @{
  */
/** @addtogroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP1
  * @{
  */
USBPD_StatusTypeDef USBPD_DPM_UserInit(void);
void  USBPD_DPM_SetNotification_GUI(GUI_NOTIFICATION_FORMAT_SEND PtrFormatSend, GUI_NOTIFICATION_POST PtrPost, GUI_SAVE_INFO PtrSaveInfo);
void  USBPD_DPM_UserExecute(void const *argument);
void  USBPD_DPM_UserCableDetection(uint8_t PortNum, USBPD_CAD_EVENT State);
void  USBPD_DPM_WaitForTime(uint32_t Time);
void  USBPD_DPM_UserTimerCounter(uint8_t PortNum);

/**
  * @}
  */

/** @addtogroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP2
  * @{
  */
USBPD_StatusTypeDef USBPD_DPM_SetupNewPower(uint8_t PortNum);
void                USBPD_DPM_HardReset(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HR_Status_TypeDef Status);
USBPD_StatusTypeDef USBPD_DPM_EvaluatePowerRoleSwap(uint8_t PortNum);
void                USBPD_DPM_Notification(uint8_t PortNum, USBPD_NotifyEventValue_TypeDef EventVal);
#if 0
USBPD_StatusTypeDef USBPD_DPM_IsContractStillValid(uint8_t PortNum);
#endif
#ifdef USBPD_REV30_SUPPORT
void                USBPD_DPM_ExtendedMessageReceived(uint8_t PortNum, USBPD_ExtendedMsg_TypeDef MsgType, uint8_t *ptrData, uint16_t DataSize);
#endif /*USBPD_REV30_SUPPORT*/
void                USBPD_DPM_GetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId , uint8_t *Ptr, uint32_t *Size);
void                USBPD_DPM_SetDataInfo(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId , uint8_t *Ptr, uint32_t Size);
void                USBPD_DPM_SNK_EvaluateCapabilities(uint8_t PortNum, uint32_t *PtrRequestData, USBPD_CORE_PDO_Type_TypeDef *PtrPowerObjectType);
uint32_t            USBPD_DPM_SNK_EvaluateMatchWithSRCPDO(uint8_t PortNum, uint32_t SrcPDO, uint32_t* PtrRequestedVoltage, uint32_t* PtrRequestedPower);

#ifdef _VCONN_SUPPORT
USBPD_StatusTypeDef USBPD_DPM_EvaluateVconnSwap(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_PE_VconnPwr(uint8_t PortNum, USBPD_FunctionalState State);
#endif /* _VCONN_SUPPORT */
#ifdef _ERROR_RECOVERY
void                USBPD_DPM_EnterErrorRecovery(uint8_t PortNum);
#endif /* _ERROR_RECOVERY */
USBPD_StatusTypeDef USBPD_DPM_EvaluateDataRoleSwap(uint8_t PortNum);
USBPD_FunctionalState USBPD_DPM_IsPowerReady(uint8_t PortNum, USBPD_VSAFE_StatusTypeDef Vsafe);

/**
  * @}
  */



/** @addtogroup USBPD_USER_EXPORTED_FUNCTIONS_GROUP3
  * @{
  */
USBPD_StatusTypeDef USBPD_DPM_RequestHardReset(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestCableReset(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGotoMin(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestPing(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestMessageRequest(uint8_t PortNum, uint8_t IndexSrcPDO, uint16_t RequestedVoltage);
USBPD_StatusTypeDef USBPD_DPM_RequestGetSourceCapability(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetSinkCapability(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestDataRoleSwap(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestPowerRoleSwap(uint8_t PortNum);
#if  defined(_VCONN_SUPPORT)
USBPD_StatusTypeDef USBPD_DPM_RequestVconnSwap(uint8_t PortNum);
#endif
USBPD_StatusTypeDef USBPD_DPM_RequestSoftReset(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef USBPD_DPM_RequestSourceCapability(uint8_t PortNum);
#if defined(_VDM) || defined(_VCONN_SUPPORT)
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_DiscoveryIdentify(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_DiscoverySVID(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType);
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_DiscoveryMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID);
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_EnterMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint8_t ModeIndex);
USBPD_StatusTypeDef USBPD_DPM_RequestVDM_ExitMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint8_t ModeIndex);
USBPD_StatusTypeDef USBPD_DPM_RequestDisplayPortStatus(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint32_t *pDPStatus);
USBPD_StatusTypeDef USBPD_DPM_RequestDisplayPortConfig(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID, uint32_t *pDPConfig);
#endif
#if defined(_VDM)
USBPD_StatusTypeDef USBPD_DPM_RequestAttention(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint16_t SVID);
#endif /* _VDM */
#ifdef USBPD_REV30_SUPPORT
USBPD_StatusTypeDef USBPD_DPM_RequestAlert(uint8_t PortNum, USBPD_ADO_TypeDef Alert);
USBPD_StatusTypeDef USBPD_DPM_RequestGetSourceCapabilityExt(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetSinkCapabilityExt(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetManufacturerInfo(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, uint8_t* pManuInfoData);
USBPD_StatusTypeDef USBPD_DPM_RequestGetStatus(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestFastRoleSwap(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetPPS_Status(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetCountryCodes(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestGetCountryInfo(uint8_t PortNum, uint16_t CountryCode);
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryCapability(uint8_t PortNum, uint8_t *pBatteryCapRef);
USBPD_StatusTypeDef USBPD_DPM_RequestGetBatteryStatus(uint8_t PortNum, uint8_t *pBatteryStatusRef);
USBPD_StatusTypeDef USBPD_DPM_RequestSecurityRequest(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_DPM_RequestFirwmwareUpdate(uint8_t PortNum, USBPD_ExtendedMsg_TypeDef MessageType, uint8_t *pPayload, uint16_t Size);
#endif /*USBPD_REV30_SUPPORT*/
USBPD_StatusTypeDef USBPD_Retry_DRSWAP(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_Retry_PRSWAP(uint8_t PortNum);
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

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_DPM_USER_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
