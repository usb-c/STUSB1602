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
#if defined(_VDM) || defined(_VCONN_SUPPORT)
#include "usbpd_core.h"
#include "usbpd_vdm_user.h"
#include "usbpd_dpm_user.h"
#include "usbpd_dpm_conf.h"
#endif 
#include "usbpd_dpm_conf.h"    
#include "string.h"
#ifdef _CLASS_BB
#include "usbd_billboard.h"
#endif /* CLASS_BB */
 
#ifdef _CLASS_BB
extern uint8_t hUSBDBOSDesc[USB_SIZ_BOS_DESC];
#endif /* CLASS_BB */

/* Private define ------------------------------------------------------------*/
#if defined(_VDM)
#define SVDM_DP_STATUS SVDM_SPECIFIC_1
#define SVDM_DP_CONFIG SVDM_SPECIFIC_2
#if defined SOURCING_DEVICE
#define MAX_SVID_USER   0
#else
#define MAX_SVID_USER   1
#endif
/*
 * DP Pin assignement
 */
#define  DP_PIN_ASSIGNMENT_NONE 0x00            /*!< De-select pin assignment.  */
#define  DP_PIN_ASSIGNMENT_A    0x01            /*!< Select Pin Assignment A    */
#define  DP_PIN_ASSIGNMENT_B    0x02            /*!< Select Pin Assignment B    */
#define  DP_PIN_ASSIGNMENT_C    0x04            /*!< Select Pin Assignment C    */
#define  DP_PIN_ASSIGNMENT_D    0x08            /*!< Select Pin Assignment D    */
#define  DP_PIN_ASSIGNMENT_E    0x10            /*!< Select Pin Assignment E    */
#define  DP_PIN_ASSIGNMENT_F    0x20            /*!< Select Pin Assignment F    */

/* Pin configs B/D/F support multi-function */
#define MODE_DP_PIN_MF_MASK 0x2a
/* Pin configs A/B support BR2 signaling levels */
#define MODE_DP_PIN_BR2_MASK 0x3
/* Pin configs C/D/E/F support DP signaling levels */
#define MODE_DP_PIN_DP_MASK 0x3c

#define MODE_DP_V13  0x1
#define MODE_DP_GEN2 0x2

#define MODE_DP_NUMBER    1u  /* 2u: settings for 2 modes vailable also*/
#define MODE_DP_MODE_SNK  0x1
#define MODE_DP_MODE_SRC  0x2
#define MODE_DP_MODE_BOTH 0x3

#define MODE_DP_STATUS_CONNECT_NO     0x0    /*!< no (DFP|UFP)_D is connected or disabled */
#define MODE_DP_STATUS_CONNECT_DFP_D  0x1    /*!< DFP_D connected                         */
#define MODE_DP_STATUS_CONNECT_UFP_D  0x2    /*!< UFP_D connected                         */
#define MODE_DP_STATUS_CONNECT_BOTH   0x3    /*!< DFP_D & UFP_D connected                 */

#define DISPLAY_PORT_SVID       0xFF01U          /*!< SVID For Display Port              */
#define PRODUCT_VDO             0xAAAAAAAAU      /*!< Device version + USB Product ID    */

/** 
  * @brief Type-C to Plug/Receptacle
  * @{
  */
typedef enum {
  CABLE_TO_PLUG        = 0, /*0b0*/
  CABLE_TO_RECEPTACLE  = 1, /*0b1*/
} USBPD_CableToPR;

/** 
  * @}
  */
#endif /* _VDM */
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
#if defined(_VDM)
/*
 * DisplayPort modes capabilities
 */

typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t   SignalDirection  : 2;    /*!< signal direction ( 00b=rsv, 01b=sink, 10b=src 11b=both ) */
    uint32_t   Supports         : 4;    /*!< xxx1: Supports DPv1.3, xx1x Supports USB Gen 2 signaling
                                             Other bits are reserved.                                 */
    uint32_t   PlugOrRecept     : 1;    /*!< Plug | Receptacle (0b == plug, 1b == receptacle)         */
    uint32_t   USB20            : 1;    /*!< USB 2.0 signaling (0b=yes, 1b=no)                        */
    uint32_t   DFP_D_Pin        : 8;    /*!< DFP_D pin assignment supported                           */
    uint32_t   UFP_D_Pin        : 8;    /*!< UFP_D pin assignment supported                           */
    uint32_t   Reserved         : 8;    /*!< Reserved                                                 */
  }d;
}USBPD_DPModeTypeDef;

/*
 * Structure to SVID supported by the devices
 */
typedef struct {
  uint32_t  NumSVIDs;
  uint16_t  SVIDs[MAX_SVID_USER];
}USBPD_SVIDUSerInfo_TypeDef;
#endif /* _VDM */

/* Private macro -------------------------------------------------------------*/

#if defined(_DEBUG_TRACE)
#define VDM_USER_DEBUG_TRACE(_PORT_, __MESSAGE__) 
#else
#define VDM_USER_DEBUG_TRACE(_PORT_, __MESSAGE__)
#endif /* _DEBUG_TRACE */


/* Private function prototypes -----------------------------------------------*/
/* List of callbacks for VDM layer */

#if defined(_VDM)
static USBPD_StatusTypeDef USBPD_VDM_DiscoverIdentity(uint8_t PortNum, USBPD_DiscoveryIdentity_TypeDef *pIdentity);
static USBPD_StatusTypeDef USBPD_VDM_DiscoverSVIDs(uint8_t PortNum, uint16_t **p_SVID_Info, uint8_t *nb);
static USBPD_StatusTypeDef USBPD_VDM_DiscoverModes(uint8_t PortNum, uint16_t SVID, uint32_t **p_ModeInfo, uint8_t *nbMode);
static USBPD_StatusTypeDef USBPD_VDM_ModeEnter(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex);
static USBPD_StatusTypeDef USBPD_VDM_ModeExit(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex);
static void                USBPD_VDM_SendAttention(uint8_t PortNum, uint8_t *NbData, uint32_t *VDO);
static void                USBPD_VDM_ReceiveAttention(uint8_t PortNum, uint8_t NbData, uint32_t VDO);
static USBPD_StatusTypeDef USBPD_VDM_ReceiveSpecific(uint8_t PortNum, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *NbData, uint32_t *VDO);
#endif /* _VDM */
#if defined(_VDM) || defined(_VCONN_SUPPORT)
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

#ifdef _VDM
USBPD_VDM_SettingsTypeDef       DPM_VDM_Settings[USBPD_PORT_COUNT]=
{
  {
    .VDM_XID_SOP                = USBPD_XID,    /*!< A decimal number assigned by USB-IF before certification */
    .VDM_USB_VID_SOP            = USBPD_VID,      /*!< A decimal number assigned by USB-IF before certification */
    .VDM_PID_SOP                = USBPD_PID,        /*!< A unique number assigned by the Vendor ID holder identifying the product. */
#ifdef _CLASS_BB
    .VDM_ModalOperation         = MODAL_OPERATION_SUPPORTED, /*!< Product support Modes based on @ref USBPD_ModalOp_TypeDef */
#else
    .VDM_ModalOperation         = MODAL_OPERATION_NONSUPP, /*!< Product support Modes based on @ref USBPD_ModalOp_TypeDef */
#endif
    .VDM_bcdDevice_SOP          = 0xAAAAu,        /*!< A unique number assigned by the Vendor ID holder containing identity information relevant to the release version of the product. */
    .VDM_USBHostSupport         = USB_NOTCAPABLE, /*!< Indicates whether the UUT is capable of enumerating USB Host */
    .VDM_USBDeviceSupport       = USB_NOTCAPABLE, /*!< Indicates whether the UUT is capable of enumerating USB Devices */
  #if defined(_CLASS_BB)
    .VDM_ProductTypeUFPorCP     = PRODUCT_TYPE_AMA,
  #elif defined (_CLASS_HID)
    .VDM_ProductTypeUFPorCP     = PRODUCT_TYPE_PERIPHERAL,
  #elif defined(USBPD_REV30_SUPPORT) &&  defined (DRP)
    .VDM_ProductTypeUFPorCP     = PRODUCT_TYPE_PSD,
  #else
    .VDM_ProductTypeUFPorCP     = PRODUCT_TYPE_UNDEFINED,
  #endif
#if defined(USBPD_REV30_SUPPORT)
    .VDM_ProductTypeDFP         = PRODUCT_TYPE_POWER_BRICK, /*!< Product type DFP of the UUT based on @ref USBPD_ProductType_TypeDef */
#endif /* USBPD_REV30_SUPPORT */
#if USBPD_PORT_COUNT >= 2
  },
  {
    .VDM_XID_SOP                = USBPD_XID,    /* A decimal number assigned by USB-IF before certification */
    .VDM_USB_VID_SOP            = USBPD_VID,        /* A decimal number assigned by USB-IF before certification */
    .VDM_PID_SOP                = USBPD_PID,        /*!< A unique number assigned by the Vendor ID holder identifying the product. */
    .VDM_ModalOperation         = MODAL_OPERATION_NONSUPP, /*!< Product support Modes based on @ref USBPD_ModalOp_TypeDef */
    .VDM_bcdDevice_SOP          = 0xAAAAu,        /*!< A unique number assigned by the Vendor ID holder containing identity information relevant to the release version of the product. */
    .VDM_USBHostSupport         = USB_NOTCAPABLE, /*!< Indicates whether the UUT is capable of enumerating USB Host */
    .VDM_USBDeviceSupport       = USB_NOTCAPABLE, /*!< Indicates whether the UUT is capable of enumerating USB Devices */
  #if defined(USBPD_REV30_SUPPORT)
    .VDM_ProductTypeUFPorCP     = PRODUCT_TYPE_PSD,
  #else
    .VDM_ProductTypeUFPorCP     = PRODUCT_TYPE_UNDEFINED,
  #endif /* SNK || DRP */
#if defined(USBPD_REV30_SUPPORT)
    .VDM_ProductTypeDFP         = PRODUCT_TYPE_POWER_BRICK, /*!< Product type DFP of the UUT based on @ref USBPD_ProductType_TypeDef */
#endif /* USBPD_REV30_SUPPORT */
#endif /* USBPD_PORT_COUNT >= 2 */
  }
};
#endif /* _VDM */

const USBPD_VDM_Callbacks vdmCallbacks =
{
#if defined(_VDM) || defined(_VCONN_SUPPORT)
#if defined(_VDM)
  USBPD_VDM_DiscoverIdentity,
  USBPD_VDM_DiscoverSVIDs,
  USBPD_VDM_DiscoverModes,
  USBPD_VDM_ModeEnter,
  USBPD_VDM_ModeExit,
#else
  NULL,
  NULL,
  NULL,
  NULL,
  NULL,
#endif /* _VDM */
  USBPD_VDM_InformIdentity,
  USBPD_VDM_InformSVID,
  USBPD_VDM_InformMode,
  USBPD_VDM_InformModeEnter,
  USBPD_VDM_InformModeExit,
#if defined(_VDM)
  USBPD_VDM_SendAttention,
  USBPD_VDM_ReceiveAttention,
#else
  NULL,
  NULL,
#endif /* _VDM */
  USBPD_VDM_SendSpecific,
#if defined(_VDM)
  USBPD_VDM_ReceiveSpecific,
#else
  NULL,
#endif /* _VDM */
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

#if defined(_VDM)
uint8_t VDM_Mode_On[USBPD_PORT_COUNT];

USBPD_IDHeaderVDO_TypeDef IDHeaderVDO[USBPD_PORT_COUNT];

USBPD_DiscoveryIdentity_TypeDef sIdentity[USBPD_PORT_COUNT];

uint16_t Remote_CurrentSVID[USBPD_PORT_COUNT];
uint16_t Remote_SVID_Mode[USBPD_PORT_COUNT];

USBPD_SVIDUSerInfo_TypeDef SVIDInfo[USBPD_PORT_COUNT];

const USBPD_DPModeTypeDef vdo_dp_modes[USBPD_PORT_COUNT][MODE_DP_NUMBER] =
{
  {
    {
      .d.UFP_D_Pin        = DP_PIN_ASSIGNMENT_C | DP_PIN_ASSIGNMENT_D,              /* UFP pin cfg supported : none         */
      .d.DFP_D_Pin        = DP_PIN_ASSIGNMENT_NONE, /* DFP pin cfg supported */
      .d.USB20            = 1,              /* USB2.0 signalling even in AMode      */
      .d.PlugOrRecept     = CABLE_TO_RECEPTACLE,  /* its a receptacle                           */
      .d.Supports         = MODE_DP_V13,    /* DPv1.3 Support, no Gen2              */
      .d.SignalDirection  = MODE_DP_MODE_SNK  /* Its a sink only                      */
    },
#if MODE_DP_NUMBER==2u
    {
      .d.UFP_D_Pin        = DP_PIN_ASSIGNMENT_C | DP_PIN_ASSIGNMENT_D,                  /* UFP pin cfg supported : none     */
      .d.DFP_D_Pin        = DP_PIN_ASSIGNMENT_NONE, /* DFP pin cfg supported */
      .d.USB20            = 0,                  /* USB2.0 signalling even in AMode  */
      .d.PlugOrRecept     = CABLE_TO_RECEPTACLE,/* its a receptacle                 */
      .d.Supports         = MODE_DP_V13,        /* DPv1.3 Support, no Gen2          */
      .d.SignalDirection  = MODE_DP_MODE_BOTH   /* Its a sink / src                 */
    }
#endif /* MODE_DP_NUMBER==2u */
  },
#if USBPD_PORT_COUNT == 2
  {
    {
      .d.UFP_D_Pin        = DP_PIN_ASSIGNMENT_C | DP_PIN_ASSIGNMENT_D,                  /* UFP pin cfg supported : none     */
      .d.DFP_D_Pin        = DP_PIN_ASSIGNMENT_NONE, /* DFP pin cfg supported */
      .d.USB20            = 1,              /* USB2.0 signalling even in AMode      */
      .d.PlugOrRecept     = CABLE_TO_RECEPTACLE,  /* its a receptacle                           */
      .d.Supports         = MODE_DP_V13,    /* DPv1.3 Support, no Gen2              */
      .d.SignalDirection  = MODE_DP_MODE_SNK  /* Its a sink only                      */
    },
#if MODE_DP_NUMBER==2u
    {
      .d.UFP_D_Pin        = DP_PIN_ASSIGNMENT_C | DP_PIN_ASSIGNMENT_D,                  /* UFP pin cfg supported : none     */
      .d.DFP_D_Pin        = DP_PIN_ASSIGNMENT_NONE, /* DFP pin cfg supported */
      .d.USB20            = 0,                  /* USB2.0 signalling even in AMode  */
      .d.PlugOrRecept     = CABLE_TO_RECEPTACLE,/* its a receptacle                 */
      .d.Supports         = MODE_DP_V13,        /* DPv1.3 Support, no Gen2          */
      .d.SignalDirection  = MODE_DP_MODE_BOTH   /* Its a sink / src                 */
    }
#endif /* MODE_DP_NUMBER==2u */
  },
#endif /*USBPD_PORT_COUNT == 2*/
};

USBPD_DPStatus_TypeDef sDP_Status[USBPD_PORT_COUNT] = 
{
  {
    .d.Reserved       = 0,
    .d.IRQ_HPD        = 0,
    .d.HPDState       = 0,
    .d.ExitDPMode     = 0,
    .d.USBConfig      = 0,
    .d.MultiFunction  = 1,
    .d.Enable         = 1,
    .d.PowerLow       = 0,
    .d.ConnectStatus  = MODE_DP_STATUS_CONNECT_DFP_D,
  },
#if USBPD_PORT_COUNT == 2
  {
    .d.Reserved       = 0,
    .d.IRQ_HPD        = 0,
    .d.HPDState       = 0,
    .d.ExitDPMode     = 0,
    .d.USBConfig      = 0,
    .d.MultiFunction  = 0,
    .d.Enable         = 1,
    .d.PowerLow       = 0,
    .d.ConnectStatus  = MODE_DP_STATUS_CONNECT_UFP_D,
  },
#endif /*USBPD_PORT_COUNT == 2*/
};

#endif /* _VDM */
/* USER CODE END Private_variables */

/* Private functions ---------------------------------------------------------*/

#if defined(_VDM)
/**
  * @brief  VDM Discovery identity callback
  * @note   Function is called to get Discovery identity information linked to the device and answer
  *         to SVDM Discovery identity init message sent by port partner
  * @param  PortNum   current port number
  * @param  pIdentity Pointer on @ref USBPD_DiscoveryIdentity_TypeDef structure
  * @retval USBPD status: @ref USBPD_ACK or @ref USBPD_BUSY
  */
static USBPD_StatusTypeDef USBPD_VDM_DiscoverIdentity(uint8_t PortNum, USBPD_DiscoveryIdentity_TypeDef *pIdentity)
{
  
  /* Fill header VDO to describe the product 
    * in DRD case, 'UFP' VDOs needs to be followed by 'DFP one'
    * AMA has single VDO - UFP has 2 VDOs - VPD has single VDO 
    * DFP VDO is not always present
  */
  IDHeaderVDO[PortNum].d32                      = 0;
  if ((DPM_Params[PortNum].PE_SpecRevision) > USBPD_SPECIFICATION_REV2)
  {
    IDHeaderVDO[PortNum].b30.VID                  = DPM_VDM_Settings[PortNum].VDM_USB_VID_SOP;
    IDHeaderVDO[PortNum].b30.ConnectorType        = CONNECTOR_TYPE_C_RECEPTACLE;
    IDHeaderVDO[PortNum].b30.ModalOperation       = DPM_VDM_Settings[PortNum].VDM_ModalOperation;
    IDHeaderVDO[PortNum].b30.USBHostCapability    = DPM_VDM_Settings[PortNum].VDM_USBHostSupport;
    IDHeaderVDO[PortNum].b30.USBDevCapability     = DPM_VDM_Settings[PortNum].VDM_USBDeviceSupport;
    if (PRODUCT_TYPE_AMA==DPM_VDM_Settings[PortNum].VDM_ProductTypeUFPorCP)
    {
          IDHeaderVDO[PortNum].b30.ProductTypeUFPorCP   = PRODUCT_TYPE_PERIPHERAL;
    }
    else
    {
          IDHeaderVDO[PortNum].b30.ProductTypeUFPorCP   = DPM_VDM_Settings[PortNum].VDM_ProductTypeUFPorCP;
    }
    if (PRODUCT_TYPE_AMC==DPM_VDM_Settings[PortNum].VDM_ProductTypeDFP)
    {
      IDHeaderVDO[PortNum].b30.ProductTypeDFP       = PRODUCT_TYPE_UNDEFINED;
    }
    else
    {
      IDHeaderVDO[PortNum].b30.ProductTypeDFP       = DPM_VDM_Settings[PortNum].VDM_ProductTypeDFP;
    }
  }
  else
  {
    IDHeaderVDO[PortNum].b20.VID                    = DPM_VDM_Settings[PortNum].VDM_USB_VID_SOP;
    IDHeaderVDO[PortNum].b20.ModalOperation         = DPM_VDM_Settings[PortNum].VDM_ModalOperation;
    IDHeaderVDO[PortNum].b20.USBHostCapability      = DPM_VDM_Settings[PortNum].VDM_USBHostSupport;
    IDHeaderVDO[PortNum].b20.USBDevCapability       = DPM_VDM_Settings[PortNum].VDM_USBDeviceSupport;
     if ((PRODUCT_TYPE_PSD == DPM_VDM_Settings[PortNum].VDM_ProductTypeUFPorCP)
#if defined(USBPDCORE_VPD)
     || (PRODUCT_TYPE_VPD == DPM_VDM_Settings[PortNum].VDM_ProductTypeUFPorCP)
#endif /* USBPDCORE_VPD */
       )
    {
      IDHeaderVDO[PortNum].b20.ProductTypeUFPorCP     = PRODUCT_TYPE_UNDEFINED;
    }
    else
    {
      IDHeaderVDO[PortNum].b20.ProductTypeUFPorCP     = DPM_VDM_Settings[PortNum].VDM_ProductTypeUFPorCP;
    }
  }
  sIdentity[PortNum].IDHeader.d32               = IDHeaderVDO[PortNum].d32;
  sIdentity[PortNum].CertStatVDO.b.XID          = DPM_VDM_Settings[PortNum].VDM_XID_SOP;
  sIdentity[PortNum].ProductVDO.b.USBProductId  = DPM_VDM_Settings[PortNum].VDM_PID_SOP;
  sIdentity[PortNum].ProductVDO.b.bcdDevice     = DPM_VDM_Settings[PortNum].VDM_bcdDevice_SOP;

  sIdentity[PortNum].CableVDO_Presence    = 0;
  sIdentity[PortNum].AMA_VDO_Presence     = 0;
  sIdentity[PortNum].UFP_VDO_Presence     = 0;
  sIdentity[PortNum].DFP_VDO_Presence     = 0;
#warning "/* add other VDO presence fields*/"

  /************************************************************/
  /* VDO for 'UFP' */
  /************************************************************/

  /* Fill the AMA VDO if needed (for UFP or DFP depending on the role) */


  if ((DPM_Params[PortNum].PE_SpecRevision) > USBPD_SPECIFICATION_REV2)
  {
    if (( USB_CAPABLE== DPM_VDM_Settings[PortNum].VDM_USBHostSupport) && (USB_CAPABLE==DPM_VDM_Settings[PortNum].VDM_USBDeviceSupport))
    {
      switch (DPM_VDM_Settings[PortNum].VDM_ProductTypeUFPorCP)

  {
  case PRODUCT_TYPE_HUB:        /*!< PDUSB Hub (UFP)                        */
  case PRODUCT_TYPE_PERIPHERAL: /*!< PDUSB Host (UFP)                       */
    {
      /* 2 UFP VDO need to be filled if PD rev3 */
      USBPD_UFPVdo1_TypeDef      ufp1_vdo =
      {
        .b.USB_HighestSpeed = USB2P0_ONLY,              /*!< USB Highest Speed Support           */
        .b.AlternateModes   = ALTERNATE_MODES_RECONFIG_TYPEC_2P0,     /*!< Alternate Modes based */
        .b.ConnectorType    = CONNECTOR_TYPE_RESERVED,  /*!< Connector Type */
#if defined (_CLASS_BB)
        .b.DeviceCapability = DEVICE_CAPABILITY_USB2P0_BILLBOARD,/*!< Device Capability   */
#elif defined (_CLASS_HID)
        .b.DeviceCapability = DEVICE_CAPABILITY_USB2P0, /*!< Device Capability   */
#endif /* CLASS_BB */
        .b.UFPVDOVersion    = USBPD_VDM_VDO_UFP_VERSION_REV1P1, /*!< Version Number of the VDO (should be set to Version1.1)   */
      };
      /* Fill the UFP2 VDO */
      USBPD_UFPVdo2_TypeDef      ufp2_vdo =
      {
        .b.USB3_MaxPower = 0,   /*!< Power in watts required for full functionality excluding any power required for battery charging or for redistribution in [USB 3.2] operation.   */
        .b.USB3_MinPower = 0,   /*!< Minimum power in watts required to function in [USB 3.2] operation.   */
        .b.USB4_MaxPower = 0,   /*!< Power in watts required for full functionality excluding any power required for battery charging or for redistribution in [USB4] operation   */
        .b.USB4_MinPower = 0,   /*!< Minimum power in watts required to function in [USB4] operation.   */
      };

      sIdentity[PortNum].UFP_VDO_Presence     = 1;
      sIdentity[PortNum].UFP_VDO1.d32          = ufp1_vdo.d32;
      sIdentity[PortNum].UFP_VDO2.d32          = ufp2_vdo.d32;
      break;
    }
#if defined(USBPDCORE_VPD)
  case PRODUCT_TYPE_VPD:        /*!< VCONN-Powered USB Device (VPD) (UFP)   */
    {
      /* VPD VDO to be filled */
      USBPD_VPDVdo_TypeDef      vpd_vdo =
      {
        .b.ChargeThroughSupport   = VPD_CHARGE_THROUGH_NOT_SUPPORTED,  /*!< Charge Through Support           */
        .b.GroundImpedance        = 0,  /*!< Charge Through Support bit = 1b: Ground impedance through the VPD in 1 mOhm increments.
                                                                  Values less than 10 mOhm are Reserved and Shall Not be used.
                                                                  Charge Through Support bit = 0b: Reserved, Shall be set to zero */
        .b.VBUS_Impedance         = 0,  /*!< Charge Through Support bit = 1b: Vbus impedance through the VPD in 2 mOhm increments.
                                                                  Values less than 10 mOhm are Reserved and Shall Not be used.
                                                                  Charge Through Support bit = 0b: Reserved, Shall be set to zero   */
        .b.ChargeThroughCurrent   = VPD_CHARGE_THROUGH_NOT_SUPPORTED,  /*!< Charge Through Current Support            */
        .b.CableMaxVoltage        = VBUS_MAX_20V,  /*!< Cable maximun voltage                         */
        .b.VDO_Version            = USBPD_VDM_VDO_VPD_VERSION_REV1P0,  /*!< Version number  of the VPD VDO                */
        .b.FWVersion              = 0x1,  /*!< Cable FW version number (vendor defined)           */
        .b.HWVersion              = 0x1,  /*!< Cable HW version number (vendor defined)           */
      };
      sIdentity[PortNum].VPD_VDO_Presence     = 1;
      sIdentity[PortNum].VPD_VDO.d32          = vpd_vdo.d32;
      break;
    }
#endif /* USBPDCORE_VPD */
  case PRODUCT_TYPE_UNDEFINED: /*!< Undefined             */
  case PRODUCT_TYPE_PSD: /*!< PSD, e.g. power bank (UFP)             */
  default:
    break;
    }
  
  /************************************************************/
 if ((PRODUCT_TYPE_AMA != DPM_VDM_Settings[PortNum].VDM_ProductTypeUFPorCP) || ((PRODUCT_TYPE_AMA == DPM_VDM_Settings[PortNum].VDM_ProductTypeUFPorCP)&& (USBPD_SPECIFICATION_REV3 == DPM_Params[PortNum].PE_SpecRevision)))
 { /* do not send other VDO when AMA VDO is sent */
  switch (DPM_VDM_Settings[PortNum].VDM_ProductTypeDFP)
  {
  case PRODUCT_TYPE_HUB:          /*!< PDUSB Hub (DFP)                        */
  case PRODUCT_TYPE_HOST:         /*!< PDUSB Host  (DFP)                      */
  case PRODUCT_TYPE_POWER_BRICK:  /*!< Power Brick (DFP)                      */
    {
      /* Fill the UFP1 VDO */
      USBPD_DFPVdo_TypeDef      dfp_vdo =
      {
        .b.PortNumber        = PortNum,   /*!< Unique port number to identify a specific port on a multi-port device  */
        .b.ConnectorType     = CONNECTOR_TYPE_C_RECEPTACLE,   /*!< Connector Type                                                         */
        .b.HostCapability    = HOST_CAPABILITY_USB2P0,   /*!< Host Capability                                                        */
        .b.DFPVDOVersion     = USBPD_VDM_VDO_DFP_VERSION_REV1P1,   /*!< Version Number of the VDO (should be set to Version1.1)                */
      };
      sIdentity[PortNum].DFP_VDO_Presence     = 1;
      sIdentity[PortNum].DFP_VDO.d32          = dfp_vdo.d32;
      break;
    }
  case PRODUCT_TYPE_UNDEFINED:    /*!< Undefined             */
  case PRODUCT_TYPE_AMC:          /*!<  Alternate Mode Controller (AMC) (DFP) */
  default:
    break;
  }
  }
  }
  }
  *pIdentity = sIdentity[PortNum];

  return USBPD_ACK;
}

/**
  * @brief  VDM Discover SVID callback
  * @note   Function is called to retrieve SVID supported by device and answer
  *         to SVDM Discovery SVID init message sent by port partner
  * @param  PortNum     current port number
  * @param  p_SVID_Info Pointer on @ref USBPD_SVIDInfo_TypeDef structure
  * @param  pNbSVID     Pointer on number of SVID
  * @retval USBPD status  @ref USBPD_BUSY or @ref USBPD_ACK or @ref USBPD_NAK
  */
static USBPD_StatusTypeDef USBPD_VDM_DiscoverSVIDs(uint8_t PortNum, uint16_t **p_SVID_Info, uint8_t *pNbSVID)
{
  USBPD_StatusTypeDef _status = USBPD_NAK;
  if (MODAL_OPERATION_SUPPORTED == DPM_VDM_Settings[PortNum].VDM_ModalOperation )
  {
    *pNbSVID = SVIDInfo[PortNum].NumSVIDs;
    *p_SVID_Info = SVIDInfo[PortNum].SVIDs;
    _status = USBPD_ACK;  
  }
  return _status; 
}

/**
  * @brief  VDM Discover Mode callback (report all the modes supported by SVID)
  * @note   Function is called to report all the modes supported by selected SVID and answer
  *         to SVDM Discovery Mode init message sent by port partner
  * @param  PortNum      current port number
  * @param  SVID         SVID value
  * @param  p_ModeTab    Pointer on the mode value
  * @param  NumberOfMode Number of mode available
  * @retval USBPD status
  */
static USBPD_StatusTypeDef USBPD_VDM_DiscoverModes(uint8_t PortNum, uint16_t SVID, uint32_t **p_ModeTab, uint8_t *NumberOfMode)
{
  USBPD_StatusTypeDef _status = USBPD_NAK;
  
  if (MODAL_OPERATION_SUPPORTED == DPM_VDM_Settings[PortNum].VDM_ModalOperation )
  {
    switch(SVID)
    {
    case DISPLAY_PORT_SVID :
      *NumberOfMode  = MODE_DP_NUMBER;
      *p_ModeTab = (uint32_t *)&vdo_dp_modes[PortNum];
      _status = USBPD_ACK;
      break;
    default :
      *NumberOfMode = 0;
      *p_ModeTab = NULL;
      break;
    }
  }
  return _status;
}
/**
  * @brief  VDM Mode enter callback
  * @note   Function is called to check if device can enter in the mode received for the selected SVID in the
  *         SVDM enter mode init message sent by port partner
  * @param  PortNum   current port number
  * @param  SVID      SVID value
  * @param  ModeIndex Index of the mode to be entered
  * @retval USBPD status @ref USBPD_ACK/@ref USBPD_NAK
  */
static USBPD_StatusTypeDef USBPD_VDM_ModeEnter(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex)
{
  USBPD_StatusTypeDef status = USBPD_NAK;
#ifdef _CLASS_BB
  USBD_BosDescTypedef *pUSBBosDesc = (USBD_BosDescTypedef *)hUSBDBOSDesc;
  if(VDM_Mode_On[PortNum] != 0) return status;

  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 1))
  {
    status = USBPD_ACK;
    VDM_Mode_On[PortNum] = 1;

    /* Do not expose billboard anymore */
    pUSBBosDesc->wTotalLength = 0x19;
    pUSBBosDesc->bNumDeviceCaps = 0x1;
  }
 #endif /* CLASS_BB */

#ifdef _CLASS_BB
#if MODE_DP_NUMBER==2u
  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 2))
  {
    status = USBPD_ACK;
    VDM_Mode_On[PortNum] = 2;
  }
#endif /* MODE_DP_NUMBER==2u */
#endif
  return status;
/* USER CODE END USBPD_VDM_ModeEnter */
}

/**
  * @brief  VDM Mode exit callback
  * @note   Function is called to check if device can exit from the mode received for the selected SVID in the
  *         SVDM exit mode init message sent by port partner
  * @param  PortNum   current port number
  * @param  SVID      SVID value
  * @param  ModeIndex Index of the mode to be exited
  * @retval USBPD status @ref USBPD_ACK/@ref USBPD_NAK
  */
static USBPD_StatusTypeDef USBPD_VDM_ModeExit(uint8_t PortNum, uint16_t SVID, uint32_t ModeIndex)
{
  USBPD_StatusTypeDef status = USBPD_NAK;

  if(VDM_Mode_On[PortNum] == 0) return status; 
  
  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 1) && (VDM_Mode_On[PortNum] == 1))
  {
    status = USBPD_ACK;
    VDM_Mode_On[PortNum] = 0;
  }
#if MODE_DP_NUMBER==2u

  if ((DISPLAY_PORT_SVID == SVID) && (ModeIndex == 2) && (VDM_Mode_On[PortNum] == 2))
  {
    status = USBPD_ACK;
    VDM_Mode_On[PortNum] = 0;
  }
#endif /* MODE_DP_NUMBER==2u */

  return status;
}

/**
  * @brief  VDM Send Specific message callback
  * @param  PortNum    current port number
  * @param  NbData     Pointer of number of VDO to send
  * @param  VDO        Pointer of VDO to send
  * @retval status
  */
static void USBPD_VDM_SendAttention(uint8_t PortNum, uint8_t *pNbData, uint32_t *pVDO)
{
  if (USBPD_PORTDATAROLE_DFP == DPM_Params[PortNum].PE_DataRole)
  {
    USBPD_DPStatus_TypeDef dp_status = {0};
    dp_status.d.ConnectStatus = MODE_DP_STATUS_CONNECT_DFP_D;
    
    *pVDO = dp_status.d32;
    *pNbData = 1;
  }
  else
  {
    sDP_Status[PortNum].d.ConnectStatus = MODE_DP_STATUS_CONNECT_UFP_D;
    
    *pVDO = sDP_Status[PortNum].d32;
    *pNbData = 1;
  }
}

/**
  * @brief  VDM Attention callback to forward information from PE stack
  * @param  PortNum   current port number
  * @param  NbData    Number of received VDO
  * @param  VDO       Received VDO
  * @retval None
  */
static void USBPD_VDM_ReceiveAttention(uint8_t PortNum, uint8_t NbData, uint32_t VDO)
{
  if (USBPD_PORTDATAROLE_DFP == DPM_Params[PortNum].PE_DataRole)
  {
    if (NbData == 1)
    {
      USBPD_DPStatus_TypeDef vdo;
      vdo.d32 = VDO;
      
      /* Check state of HPD pin in attention message */
      if (0 == vdo.d.HPDState)
      {
        /* Screen has been disconnected... Disconnect laptop */
           /* action to be taken by user application*/
      }
      else
      {
        /* Screen has been connected... Sent a config message to screen for connection*/
        USBPD_PE_SVDM_RequestSpecific(PortNum, USBPD_SOPTYPE_SOP, SVDM_DP_CONFIG, DISPLAY_PORT_SVID);
      }
    }
  }
  else
  {
    if (NbData == 1)
    {
      /* Check if DP Status has been received in the attention message*/
      USBPD_DPStatus_TypeDef dp_status;
      dp_status.d32 = VDO;
    
      if (dp_status.d.ConnectStatus == MODE_DP_STATUS_CONNECT_UFP_D)
      {
        USBPD_PE_SVDM_RequestSpecific(PortNum, USBPD_SOPTYPE_SOP, SVDM_DP_CONFIG, DISPLAY_PORT_SVID);
      }
    }
  }
/* USER CODE END USBPD_VDM_ReceiveAttention */
}

/**
  * @brief  VDM Receive Specific message callback
  * @note   Function is called to answer to a SVDM specific init message received by port partner.
  *         (for instance, retrieve DP status or DP configure data through this function)
  * @param  PortNum         Current port number
  * @param  VDMCommand      VDM command based on @ref USBPD_VDM_Command_Typedef
  * @param  pNbData     Pointer of number of received VDO and used for the answer
  * @param  pVDO        Pointer of received VDO and use for the answer
  * @retval USBPD Status
  */
static USBPD_StatusTypeDef USBPD_VDM_ReceiveSpecific(uint8_t PortNum, USBPD_VDM_Command_Typedef VDMCommand, uint8_t *pNbData, uint32_t *pVDO)
{
/* USER CODE BEGIN USBPD_VDM_ReceiveSpecific */
  USBPD_StatusTypeDef status = USBPD_ACK;
  if (USBPD_PORTDATAROLE_DFP == DPM_Params[PortNum].PE_DataRole)
  {
    /* Data role is DFP */
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_ReceiveSpecific(DP_STATUS)");
        /* Port0 is configured as SRC */
        sDP_Status[PortNum].d.ConnectStatus = MODE_DP_STATUS_CONNECT_DFP_D;

        *pVDO = sDP_Status[PortNum].d32;
        *pNbData = 1;
      }
      break;
    case SVDM_DP_CONFIG :
      VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_ReceiveSpecific(DP_CONFIG)");
      break;

    default :
      status = USBPD_NAK;
      break;
    }
  }
  else
  {
    /* Data role is UFP */
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        USBPD_DPStatus_TypeDef vdo;
        VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_ReceiveSpecific(DP_STATUS)");
        if (*pNbData == 1)
        {
          vdo.d32 = pVDO[0];
          if ((MODE_DP_STATUS_CONNECT_DFP_D == vdo.d.ConnectStatus)
           || (MODE_DP_STATUS_CONNECT_BOTH == vdo.d.ConnectStatus)
           || (MODE_DP_STATUS_CONNECT_NO == vdo.d.ConnectStatus)) /* Connect none set by Chromebook only */
          {
            sDP_Status[PortNum].d.ConnectStatus = MODE_DP_STATUS_CONNECT_UFP_D;

            *pVDO = sDP_Status[PortNum].d32;
            *pNbData = 1;
          }
          else
          {
            status = USBPD_NAK;
          }
        }
        else
        {
          status = USBPD_NAK;
        }
      }
      break;
    case SVDM_DP_CONFIG :
      {
        /* Handle DP configuration according to Plug orientation : to be completed */
        VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_ReceiveSpecific(DP_CONFIG)");
                USBPD_DPConfig_TypeDef vdo;
        vdo.d32 = pVDO[0];
 /* Only Port0 can be DP in this example */       
        if (USBPD_PORT_0 == PortNum)
        {
          switch(vdo.d.UFP_U_Pin)
          {
          case DP_PIN_ASSIGNMENT_NONE:         /*!< De-select pin assignment.  */
            /* make USBSS-DP assignement = USB_ONLY_PIN_ASSIGNMMENT;*/
            break;
          case DP_PIN_ASSIGNMENT_A:            /*!< Select Pin Assignment A    */
            /* make USBSS-DP assignement = UFP_PIN_ASSIGNMMENT_A;*/
            break;
          case DP_PIN_ASSIGNMENT_B:            /*!< Select Pin Assignment B    */
            /* make USBSS-DP assignement = UFP_PIN_ASSIGNMMENT_B;*/
            break;
          case DP_PIN_ASSIGNMENT_C:            /*!< Select Pin Assignment C    */
            /* make USBSS-DP assignement = UFP_PIN_ASSIGNMMENT_C;*/
            break;
          case DP_PIN_ASSIGNMENT_D:            /*!< Select Pin Assignment D    */
            /* make USBSS-DP assignement = UFP_PIN_ASSIGNMMENT_D;*/
            break;
          case DP_PIN_ASSIGNMENT_E:            /*!< Select Pin Assignment E    */
            /* make USBSS-DP assignement = UFP_PIN_ASSIGNMMENT_E;*/
            break;
          case DP_PIN_ASSIGNMENT_F:            /*!< Select Pin Assignment F    */
            /* make USBSS-DP  assignement = UFP_PIN_ASSIGNMMENT_F; */
            break;
          default :
            status = USBPD_NAK;
            break;
          }
          if (CC1 == DPM_Params[PortNum].ActiveCCIs)
          {
            /* toggle external USB3 switch to CC1 raw line */
                        __NOP();
          }
          else
          {
            /* toggle external USB3 switch to CC2 raw line */
                __NOP();
          }
        }
        *pVDO = 0;
        *pNbData = 0;
      }
      break;

    default :
      status = USBPD_NAK;
      break;
    }
  }
  return status;
/* USER CODE END USBPD_VDM_ReceiveSpecific */
}

#endif /* _VDM */

#if defined(_VDM) || defined(_VCONN_SUPPORT)
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
#if defined(_VDM)
    {
      uint8_t*  disco_ident;
      disco_ident = (uint8_t*)&DPM_Ports[PortNum].VDM_DiscoIdentify;
      memcpy(disco_ident, (uint8_t*)pIdentity, sizeof(USBPD_DiscoveryIdentity_TypeDef));

      {
        /* Alternate mode presence */
        if (pIdentity->AMA_VDO_Presence == 1)
        {
          /* Request to get SVID */
          USBPD_PE_SVDM_RequestSVID(PortNum, USBPD_SOPTYPE_SOP);
        }
      }
      memset(&DPM_Ports[PortNum].VDM_SVIDPortPartner, 0, sizeof(USBPD_SVIDPortPartnerInfo_TypeDef));
      Remote_CurrentSVID[PortNum]                   = 0;
    }
#endif /* _VDM */
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
#if defined(_VDM)
  uint32_t index = 0;

  /* Manage only reception of SVID coming from SOP (SVID on cable to be implemented) */
  if (USBPD_SOPTYPE_SOP == SOPType)
  {
  switch(CommandStatus)
  {
  case SVDM_RESPONDER_ACK :
    if (1 == DPM_Ports[PortNum].VDM_SVIDPortPartner.FlagAllSVIDReceived)
    {
      /* New list of SVIDs is being received. */
      memset(&DPM_Ports[PortNum].VDM_SVIDPortPartner, 0, sizeof(USBPD_SVIDPortPartnerInfo_TypeDef));
    }
    /* Save the received SVIDs */
    for (index = 0; index < pListSVID->NumSVIDs; index++)
    {
      DPM_Ports[PortNum].VDM_SVIDPortPartner.SVIDs[DPM_Ports[PortNum].VDM_SVIDPortPartner.NumSVIDs++] = pListSVID->SVIDs[index];
    }

    /* Check if all the SVIDs have been received */
    if ((pListSVID->AllSVID_Received == 0) && (0 == (DPM_Ports[PortNum].VDM_SVIDPortPartner.NumSVIDs %12)))
    {
      /* Request a new SVID message */
      USBPD_PE_SVDM_RequestSVID(PortNum, USBPD_SOPTYPE_SOP);
    }
    else
    {
      /* All the SVIDs have been received, request discovery mode on 1st SVID available
      in the list */
      DPM_Ports[PortNum].VDM_SVIDPortPartner.FlagAllSVIDReceived = 1;
      /* Request a discovery mode */
      Remote_SVID_Mode[PortNum] = 0;
      {
        USBPD_PE_SVDM_RequestMode(PortNum, USBPD_SOPTYPE_SOP, DPM_Ports[PortNum].VDM_SVIDPortPartner.SVIDs[0]);
      }
    }
    break;
  case SVDM_RESPONDER_NAK :
    /* Nothing to do */
    break;
  case SVDM_RESPONDER_BUSY :
    break;
  default :
    break;
  }
}
#endif /* _VDM */
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
#if defined(_VDM)
  switch(CommandStatus)
  {
  case SVDM_RESPONDER_ACK :
    {
      USBPD_DPModeTypeDef dp_mode;
      DPM_Ports[PortNum].VDM_ModesPortPartner.NumModes = pModesInfo->NumModes;
      for (uint32_t index = 0; index < DPM_Ports[PortNum].VDM_ModesPortPartner.NumModes; index++)
      {
        DPM_Ports[PortNum].VDM_ModesPortPartner.Modes[index] = pModesInfo->Modes[index];
      }

      DPM_Ports[PortNum].VDM_ModesPortPartner.SVID = pModesInfo->SVID;

      dp_mode.d32 = (DPM_Ports[PortNum].VDM_ModesPortPartner.Modes[0]);
#if _CLASS_BB
      {
        /* Enter in the mode only if DFP_D and UFP_D are supported */
        if ((dp_mode.d.SignalDirection == MODE_DP_MODE_BOTH) || (MODE_DP_MODE_SNK == dp_mode.d.SignalDirection))
        {
          /* Request to enter in the 1st mode */
          USBPD_PE_SVDM_RequestModeEnter(PortNum, SOPType, pModesInfo->SVID, 1);
        }
      }
#endif
    }
    break;
  case SVDM_RESPONDER_NAK :
    {
#if _CLASS_BB
      /* All the SVIDs have been received, request discovery mode on next SVID available
      in the list */
      Remote_SVID_Mode[PortNum]++;
      /* Request a discovery mode */
      USBPD_PE_SVDM_RequestMode(PortNum, USBPD_SOPTYPE_SOP, DPM_Ports[PortNum].VDM_SVIDPortPartner.SVIDs[Remote_SVID_Mode[PortNum]]);
#endif
    }
    break;
  case SVDM_RESPONDER_BUSY :
    break;
  default :
    break;
  }
#endif /* _VDM */
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
#if defined(_VDM)
  switch(CommandStatus)
  {
  case SVDM_RESPONDER_ACK :
    {
      if (DPM_Ports[PortNum].VDM_SVIDPortPartner.SVIDs[(ModeIndex - 1)] == SVID)
      {
        VDM_Mode_On[PortNum] = 1;
        
        USBPD_PE_SVDM_RequestSpecific(PortNum, USBPD_SOPTYPE_SOP, SVDM_DP_STATUS, DISPLAY_PORT_SVID);
      }
    }
    break;
  case SVDM_RESPONDER_NAK :
    /* Nothing to do */
    break;
  case SVDM_RESPONDER_BUSY :
    /* retry in 50ms */
    break;
  default :
    break;
  }
#endif /* _VDM */
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
#if defined(_VDM)
  /* Manage only reception of exit Mode coming from SOP (Enter Mode on cable to be implemented) */
  switch(CommandStatus)
  {
  case SVDM_RESPONDER_ACK :
    {
      if (DPM_Ports[PortNum].VDM_SVIDPortPartner.SVIDs[(ModeIndex - 1)] == SVID)
      {
        VDM_Mode_On[PortNum] = 0;
      }
    }
    break;
  case SVDM_RESPONDER_NAK :
    /* Nothing to do */
    break;
  case SVDM_RESPONDER_BUSY :
    /* retry in 50ms */
    break;
  default :
    break;
  }
  
 
#endif /* _VDM */
}

/**
  * @brief  VDM Fill DP Status 
  * @param  PortNum    current port number
  * @param  pDP_Status Pointer on @ref USBPD_DPStatus_TypeDef
  * @retval None
  */
void USBPD_VDM_FillDPStatus(uint8_t PortNum, USBPD_DPStatus_TypeDef *pDP_Status)
{
#if defined(_VDM)
#endif /* _VDM */
}

/**
  * @brief  VDM Fill DP Config 
  * @param  PortNum    current port number
  * @param  pDP_Config Pointer on @ref USBPD_DPConfig_TypeDef
  * @retval None
  */
void USBPD_VDM_FillDPConfig(uint8_t PortNum, USBPD_DPConfig_TypeDef *pDP_Config)
{
#if defined(_VDM)
#endif /* _VDM */
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
#if defined(_VDM)
    /* Manage Specific message sent to port partner */
  if (USBPD_PORTDATAROLE_DFP == DPM_Params[PortNum].PE_DataRole)
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_SendSpecific(DP_STATUS)");
        USBPD_DPStatus_TypeDef dp_status = {0};
        dp_status.d.ConnectStatus = MODE_DP_STATUS_CONNECT_DFP_D;

        *pVDO = dp_status.d32;
        *pNbData = 1;
      }
      break;
    case SVDM_DP_CONFIG :
      {
        USBPD_DPModeTypeDef dp_mode;
        dp_mode.d32 = (DPM_Ports[PortNum].VDM_ModesPortPartner.Modes[0]);

        VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_SendSpecific(DP_CONFIG)");
        if (DP_PIN_ASSIGNMENT_C == (dp_mode.d.DFP_D_Pin & DP_PIN_ASSIGNMENT_C))
        {
          USBPD_DPConfig_TypeDef vdo_config = {0};
          vdo_config.d.SelectConfiguration = MODE_DP_CONFIG_SELECT_UFP_U_AS_DFP_D;
          /* Support of DPv1.3 */
          vdo_config.d.Signaling = 0x1;
          vdo_config.d.UFP_U_Pin = DP_PIN_ASSIGNMENT_C;
          *pVDO = vdo_config.d32;
          *pNbData = 1;
        }
      }
      break;
    default :
      break;
    }
  }
  else
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_SendSpecific(DP_STATUS)");
        sDP_Status[PortNum].d.ConnectStatus = MODE_DP_STATUS_CONNECT_UFP_D;

        *pVDO = sDP_Status[PortNum].d32;
        *pNbData = 1;
      }
      break;
    case SVDM_DP_CONFIG :
      VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_SendSpecific(DP_CONFIG)");
      *pVDO = 0;
      *pNbData = 0;
      break;

    default :
      break;
    }
  }
#endif /* _VDM */
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
#if defined(_VDM)
    /* Manage Specific message received by port partner */
  if (USBPD_PORTDATAROLE_DFP == DPM_Params[PortNum].PE_DataRole)
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        USBPD_DPStatus_TypeDef vdo_received;
        vdo_received.d32 = pVDO[0];
        VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_InformSpecific(DP_STATUS)");
        /* SEND SVDM_DP_CONFIG only if Port is connected */
        if (MODE_DP_STATUS_CONNECT_NO != vdo_received.d.ConnectStatus)
        {
          USBPD_PE_SVDM_RequestSpecific(PortNum, USBPD_SOPTYPE_SOP, SVDM_DP_CONFIG, DISPLAY_PORT_SVID);
        }
     }
      break;
    case SVDM_DP_CONFIG :
      VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_InformSpecific(DP_CONFIG)");
        /* Handle DP configuration information sending : to be completed */
     
      /* When DP connector is used, HDP needs to be properly set according to PD message */
          __NOP();
          
      /* Mux needs to be toggled according to CC orientation*/
        if (CC1 == DPM_Params[PortNum].ActiveCCIs)
          {
            /* toggle external USB3 switch to CC1 raw line */
                        __NOP();
          }
          else
          {
            /* toggle external USB3 switch to CC2 raw line */
                __NOP();
          }
 
      break;

    default :
      return;
    }
  }
  else
  {
    switch(VDMCommand)
    {
    case SVDM_DP_STATUS :
      {
        USBPD_DPStatus_TypeDef vdo_received;

        vdo_received.d32 = pVDO[0];

        VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_InformSpecific(DP_STATUS)");
        if (vdo_received.d.ConnectStatus != MODE_DP_STATUS_CONNECT_NO)
        {
          sDP_Status[PortNum].d.ConnectStatus = MODE_DP_STATUS_CONNECT_UFP_D;
        }
        else
        {
          /* DP not connected */
          sDP_Status[PortNum].d.ConnectStatus = MODE_DP_STATUS_CONNECT_NO;
        }
      }
      break;
    case SVDM_DP_CONFIG :
      VDM_USER_DEBUG_TRACE(PortNum, "USBPD_VDM_InformSpecific(DP_CONFIG)");
      break;

    default :
      break;
    }
  }
#endif /* _VDM */
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

#if defined(_VDM ) || defined(_VCONN_SUPPORT)
USBPD_StatusTypeDef USBPD_VDM_UserInit(uint8_t PortNum)
{
#if defined(_VDM)
  uint32_t index = 0;
  

   /* Initialize SVID supported by consumer */
  SVIDInfo[PortNum].NumSVIDs = MAX_SVID_USER;
  
  for (index = 0; index < MAX_SVID_USER; index++)
  {
    SVIDInfo[PortNum].SVIDs[index] = DISPLAY_PORT_SVID + index;
  }
 /* _VDM */ 
#endif 

  USBPD_PE_InitVDM_Callback(PortNum, (USBPD_VDM_Callbacks *)&vdmCallbacks);
#if defined(_VDM)
  if (USBPD_PORT_0 == PortNum)
    return HPD_Init(PortNum);
  else
#endif
   return USBPD_OK;
}
/* VCONN_SUPPORT */
#endif
#if defined(_VDM)
/**
  * @brief  VDM Reset function
  * @param  PortNum     Index of current used port
  * @retval status
  */
void USBPD_VDM_UserReset(uint8_t PortNum)
{
  /* Reset Port Partner variables*/
  Remote_CurrentSVID[PortNum]       = 0;
   VDM_Mode_On[PortNum] = 0;
}

#endif /*defined(_VDM ) */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

