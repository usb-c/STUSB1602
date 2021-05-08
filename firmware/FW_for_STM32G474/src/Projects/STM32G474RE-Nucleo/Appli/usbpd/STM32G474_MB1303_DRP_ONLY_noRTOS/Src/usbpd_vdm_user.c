/**
  ******************************************************************************
  * @file    usbpd_vdm_user.c
  * @author  MCD Application Team
  * @brief   USBPD provider demo file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
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
#if  defined(_VCONN_SUPPORT)
#include "usbpd_core.h"
#include "usbpd_vdm_user.h"
#include "usbpd_dpm_user.h"
#include "usbpd_dpm_conf.h"
#endif 
#include "usbpd_dpm_conf.h"    
#include "string.h"
 

/* Private define ------------------------------------------------------------*/
#if defined(USBPDCORE_SNK_CAPA_EXT)
#define VDM_CABLE_INFO      USBPD_CORE_SNK_EXTENDED_CAPA + 1U
#else
#define VDM_CABLE_INFO      USBPD_CORE_UNSTRUCTURED_VDM + 1U
#endif /* USBPDCORE_SNK_CAPA_EXT */
/* USER CODE END Private_define */

/* Private typedef -----------------------------------------------------------*/
/* GNU Compiler */
#if defined   (__GNUC__)
/* ARM Compiler */
#elif defined   (__CC_ARM)
#pragma anon_unions
/* IAR Compiler */
#elif defined (__ICCARM__)
#endif

/* Private macro -------------------------------------------------------------*/

#if defined(_DEBUG_TRACE)
#define VDM_USER_DEBUG_TRACE(_PORT_, __MESSAGE__) 
#else
#define VDM_USER_DEBUG_TRACE(_PORT_, __MESSAGE__)
#endif /* _DEBUG_TRACE */


/* Private function prototypes -----------------------------------------------*/
/* List of callbacks for VDM layer */

#if  defined(_VCONN_SUPPORT)
static void                USBPD_VDM_UserCallback(uint8_t PortNum/*, MUX_TypeCMuxIdTypeDef TypeCMuxId, MUX_HPDStateTypeDef   HPDState*/);
static void                USBPD_VDM_InformIdentity(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef status, USBPD_DiscoveryIdentity_TypeDef *pIdentity);
static void                USBPD_VDM_InformSVID(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_SVIDInfo_TypeDef *pListSVID);
static void                USBPD_VDM_InformMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_ModeInfo_TypeDef *pModesInfo);
static void                USBPD_VDM_InformModeEnter(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, uint16_t SVID, uint32_t ModeIndex);
static void                USBPD_VDM_InformModeExit(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, uint16_t SVID, uint32_t ModeIndex);
static void                USBPD_VDM_InformSpecific(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *NbData, uint32_t *VDO);
static void                USBPD_VDM_SendSpecific(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *NbData, uint32_t *VDO);
#endif /* VDM || VCONN_SUPPORT */
/* Private variables ---------------------------------------------------------*/


const USBPD_VDM_Callbacks vdmCallbacks =
{
#if  defined(_VCONN_SUPPORT)
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  USBPD_VDM_InformIdentity,
  USBPD_VDM_InformSVID,
  USBPD_VDM_InformMode,
  USBPD_VDM_InformModeEnter,
  USBPD_VDM_InformModeExit,
  NULL,
  NULL,
  USBPD_VDM_SendSpecific,
  NULL,
  USBPD_VDM_InformSpecific,
#else
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
#endif /* _VDM || _VCONN_SUPPORT */
};

extern USBPD_ParamsTypeDef DPM_Params[USBPD_PORT_COUNT];

/* USER CODE END Private_variables */

/* Private functions ---------------------------------------------------------*/


#if  defined(_VCONN_SUPPORT)
/**
  * @brief  Inform identity callback
  * @note   Function is called to save Identity information received in Discovery identity from port partner
            (answer to SVDM discovery identity sent by device)
  * @param  PortNum       current port number
  * @param  SOPType       SOP type 
  * @param  CommandStatus Command status based on @ref USBPD_VDM_CommandType_Typedef
  * @param  pIdentity     Pointer on the discovery identity information based on @ref USBPD_DiscoveryIdentity_TypeDef
  * @retval None
*/
static void USBPD_VDM_InformIdentity(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_DiscoveryIdentity_TypeDef *pIdentity)
{
  switch(CommandStatus)
  {
  case SVDM_RESPONDER_ACK :
#if defined(_VCONN_SUPPORT)
    if (USBPD_SOPTYPE_SOP1 == SOPType)
    {
      uint8_t*  disco_ident;
      disco_ident = (uint8_t*)&DPM_Ports[PortNum].VDM_DiscoCableIdentify;
      memcpy(disco_ident, (uint8_t*)pIdentity, sizeof(USBPD_DiscoveryIdentity_TypeDef));
    }
    else
#endif /* _VCONN_SUPPORT */ 
    break;
  case SVDM_RESPONDER_NAK :
    /* Nothing to do */
    break;
  case SVDM_RESPONDER_BUSY :
    /* retry in 50ms */
    break;
#if defined(_VCONN_SUPPORT)
  case SVDM_CABLE_NO_PD_CAPABLE :
    if (USBPD_SOPTYPE_SOP1 == SOPType)
    {
      USBPD_DPM_PE_VconnPwr(PortNum, USBPD_DISABLE);
      /* No more need to request VCONN ON as CABLE is not PD capable. */
      DPM_Ports[PortNum].DPM_CablePDCapable = USBPD_FALSE;
    }
    break;
  case SVDM_CABLE_TIMEOUT :
    if (USBPD_SOPTYPE_SOP1 == SOPType)
    {
    }
    break;
#endif /*  _VCONN_SUPPORT */
  default :
    break;
}
}

/**
  * @brief  Inform SVID callback
  * @note   Function is called to save list of SVID received in Discovery SVID from port partner
            (answer to SVDM discovery SVID sent by device)
  * @param  PortNum       current port number
  * @param  SOPType       SOP type 
  * @param  CommandStatus Command status based on @ref USBPD_VDM_CommandType_Typedef
  * @param  pListSVID     Pointer of list of SVID based on @ref USBPD_SVIDInfo_TypeDef
  * @retval None
  */
static void USBPD_VDM_InformSVID(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_SVIDInfo_TypeDef *pListSVID)
{
/* USER CODE BEGIN USBPD_VDM_InformSVID */
/* USER CODE END USBPD_VDM_InformSVID */
}

/**
  * @brief  Inform Mode callback ( received in Discovery Modes ACK)
  * @note   Function is called to save list of modes linked to SVID received in Discovery mode from port partner
            (answer to SVDM discovery mode sent by device)
  * @param  PortNum         current port number
  * @param  SOPType         SOP type 
  * @param  CommandStatus   Command status based on @ref USBPD_VDM_CommandType_Typedef
  * @param  pModesInfo      Pointer of Modes info based on @ref USBPD_ModeInfo_TypeDef
  * @retval None
  */
static void USBPD_VDM_InformMode(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, USBPD_ModeInfo_TypeDef *pModesInfo)
{
}

/**
  * @brief  Inform Mode enter callback
  * @note   Function is called to inform if port partner accepted or not to enter in the mode
  *         specified in the SVDM enter mode sent by the device
  * @param  PortNum       current port number
  * @param  SOPType       SOP type
  * @param  CommandStatus Command status based on @ref USBPD_VDM_CommandType_Typedef
  * @param  SVID      SVID ID
  * @param  ModeIndex Index of the mode to be entered
  * @retval None
  */
static void USBPD_VDM_InformModeEnter(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, uint16_t SVID, uint32_t ModeIndex)
{
}

/**
  * @brief  Inform Exit Mode callback
  * @param  PortNum   current port number
  * @param  SVID      SVID ID
  * @param  ModeIndex Index of the mode to be entered
  * @retval None
  */
static void USBPD_VDM_InformModeExit(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_CommandType_Typedef CommandStatus, uint16_t SVID, uint32_t ModeIndex)
{
/* USER CODE BEGIN USBPD_VDM_InformModeExit */
}

/**
  * @brief  VDM Fill DP Status 
  * @param  PortNum    current port number
  * @param  pDP_Status Pointer on @ref USBPD_DPStatus_TypeDef
  * @retval None
  */
void USBPD_VDM_FillDPStatus(uint8_t PortNum, USBPD_DPStatus_TypeDef *pDP_Status)
{
}

/**
  * @brief  VDM Fill DP Config 
  * @param  PortNum    current port number
  * @param  pDP_Config Pointer on @ref USBPD_DPConfig_TypeDef
  * @retval None
  */
void USBPD_VDM_FillDPConfig(uint8_t PortNum, USBPD_DPConfig_TypeDef *pDP_Config)
{
}


/**
  * @brief  VDM Send Specific message callback
  * @note   Function is called when device wants to send a SVDM specific init message to port partner
  *         (for instance DP status or DP configure can be filled through this function)
  * @param  PortNum    current port number
  * @param  SOPType    SOP type
  * @param  VDMCommand VDM command based on @ref USBPD_VDM_Command_Typedef
  * @param  pNbData    Pointer of number of VDO to send
  * @param  pVDO       Pointer of VDO to send
  * @retval None
  */
static void USBPD_VDM_SendSpecific(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *pNbData, uint32_t *pVDO)
{
/* USER CODE BEGIN USBPD_VDM_SendSpecific */
#if defined(_VCONN_SUPPORT)
  /* Manage Specific message sent to EMC cable */
  if (USBPD_SOPTYPE_SOP != SOPType)
  {
    VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_SendSpecific(CABLE)");
  }
  else
  {
#endif /* _VCONN_SUPPORT */
#if defined(_VCONN_SUPPORT)
  }
#endif /* _VCONN_SUPPORT */
/* USER CODE END USBPD_VDM_SendSpecific */
}

/**
  * @brief  VDM Receive Specific message callback
  * @note   Function is called to answer to a SVDM specific init message received by port partner.
  *         (for instance, retrieve DP status or DP configure data through this function)
  * @param  PortNum     Current port number
  * @param  VDMCommand  VDM command based on @ref USBPD_VDM_Command_Typedef
  * @param  pNbData     Pointer of number of received VDO and used for the answer
  * @param  pVDO        Pointer of received VDO and use for the answer
  * @retval USBPD status
  * @retval None
  */
static void USBPD_VDM_InformSpecific(uint8_t PortNum, USBPD_SOPType_TypeDef SOPType, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *pNbData, uint32_t *pVDO)
{
/* USER CODE BEGIN USBPD_VDM_InformSpecific */
#if defined(_VCONN_SUPPORT)
  /* Manage Specific message received by EMC cable */
  if (USBPD_SOPTYPE_SOP != SOPType)
  {
    VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_SendSpecific(CABLE)");
  }
  else
  {
#endif /* _VCONN_SUPPORT */
#if defined(_VCONN_SUPPORT)
  }
#endif /* VCONN_SUPPORT */
/* USER CODE END USBPD_VDM_InformSpecific */
}
/* _VDM || VCONN_SUPPORT */
#endif 


#if defined _VDM
/**
  * @brief  Initialize HPD Alert pin
  * @param  PortNum    current port number
  * @retval USBPD status
  */
static USBPD_StatusTypeDef HPD_Init(uint8_t PortNum)
{
  /* check the status of USB3 orientation + USB3 vs DP muxes*/
  /* example:
  MUX_HPDStateTypeDef hpd_state;
  if (MUX_OK != BSP_MUX_Init(TYPE_C_MUX_2))
  {
    return USBPD_ERROR;
  }

  if (MUX_OK != BSP_MUX_Init(TYPE_C_MUX_1))
  {
    return USBPD_ERROR;
  }
*/

  /* in DP to TypeC application, BSP including muxes and GPIO settings/status needs to be defined */
  /* Register HPD callback function in case of DP connector to trig an event */
  /*  */
  /* example 
  if (BSP_MUX_RegisterHPDCallbackFunc(TYPE_C_MUX_2, USBPD_VDM_UserCallback) != MUX_OK)
  {
    return USBPD_ERROR;
  }
  */

  /* in DP to TypeC application, BSP including muxes and GPIO settings/status needs to be defined */
  /* get HPD state from DP connector */
  /*  */
  /* example 
  if (MUX_OK != BSP_MUX_GetHPDState(TYPE_C_MUX_2, &hpd_state))
  {
    return USBPD_ERROR;
  }
  */
  
  /*if DP already connected, update state accordingly */
  /* example
  if (HPD_STATE_HIGH == hpd_state)
  {
    sDP_Status[PortNum].d.HPDState = 1;
    sDP_Status[PortNum].d.Enable = 1;
  }
  else
  {
    sDP_Status[PortNum].d.HPDState = 0;
    sDP_Status[PortNum].d.Enable = 0;
  }
  */
  return USBPD_OK;
}

/* VDM_UserCallback is example of callback to be used in case of DP application*/
static void USBPD_VDM_UserCallback(uint8_t PortNum/*,MUX_TypeCMuxIdTypeDef TypeCMuxId, MUX_HPDStateTypeDef   HPDState*/ )
{
  /* update DP_status according to HPDState*/
  /* example
  if (HPD_STATE_HIGH == HPDState)
  {
    sDP_Status[USBPD_PORT_0].d.HPDState = 1;
    sDP_Status[USBPD_PORT_0].d.Enable = 1;
  }
  else
  {
    sDP_Status[USBPD_PORT_0].d.HPDState = 0;
    sDP_Status[USBPD_PORT_0].d.Enable = 1;
  }
 */

  /* Send attention message only if USBPD_PORT_0 is connected for example*/
  if (USBPD_POWER_EXPLICITCONTRACT == DPM_Params[USBPD_PORT_0].PE_Power)
  {
    USBPD_PE_SVDM_RequestAttention(USBPD_PORT_0, USBPD_SOPTYPE_SOP, DISPLAY_PORT_SVID);
  }
}
#endif
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief  VDM Initialization function
  * @param  PortNum     Index of current used port
  * @retval status
  */

#if  defined(_VCONN_SUPPORT)
USBPD_StatusTypeDef USBPD_VDM_UserInit(uint8_t PortNum)
{

  USBPD_PE_InitVDM_Callback(PortNum, (USBPD_VDM_Callbacks *)&vdmCallbacks);
   return USBPD_OK;
}
/* VCONN_SUPPORT */
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

