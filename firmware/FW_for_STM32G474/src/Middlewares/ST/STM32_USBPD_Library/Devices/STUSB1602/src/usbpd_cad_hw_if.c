/**
  ******************************************************************************
  * @file    usbpd_cad_hw_if.c
  * @author  MCD Application Team
  * @brief   This file contains CAD interfaces functions.
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

/* Includes ------------------------------------------------------------------*/
#include "User_BSP.h"

#include "usbpd_cad_hw_if.h"
#include "usbpd_dpm_conf.h"
void CAD_Set_default_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue);

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_CAD_HW_IF
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
extern  USBPD_StatusTypeDef USBPD_HW_IF_ErrorRecovery(uint8_t PortNum);


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Handle for the ports inside @ref USBPD_DEVICE_HW_IF */
extern STUSB16xx_PORT_HandleTypeDef Ports[];

/**
  * @brief handle to manage the detection state machine 
  */
CAD_HW_HandleTypeDef CAD_HW_Handles[USBPD_PORT_COUNT];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_CAD_HW_IF_Exported_Functions USBPD DEVICE CAD HW IF Exported Functions
  * @{
  */

/**
  * @}
  */
/** Function to set TypeC Rp resistor depending on customer application requirement **/
/** RpValue is defined in dpm_conf.h: CAD_DefaultResistor **/
    
void CAD_Set_default_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue)
{
  switch (RpValue)
  {  
  case vRp_Default:
     STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_Default); /* USB_C_Current_default */
     break;
  case vRp_1_5A:
     STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_1_5_A); /* USB_C_Current_1.5A */
     break;
  case vRp_3_0A: 
    STUSB1602_Current_Advertised_Set(STUSB1602_I2C_Add(PortNum), USB_C_Current_3_0_A); /* USB_C_Current_3.0A */
    break;
  }
}

/** Publique Function not used in 1602 context **/
#if 1
uint32_t CAD_Set_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue)
{
  return 1; /* Dynamique update of the resistor  */
}
#endif

/**
  * @brief  This function performs an initialization of the CAD 
  * @param  PortNum        Number of the port
  * @param  Settings       Settings for the system based on @ref USBPD_SettingsTypeDef
  * @param  Params         Parameters definition based on @ref USBPD_ParamsTypeDef
  * @param  WakeUp         Callback for the wakeup
  * @retval CC pin line based on @ref CCxPin_TypeDef
  */
void CAD_Init(uint8_t PortNum, USBPD_SettingsTypeDef *Settings, USBPD_ParamsTypeDef *Params,  void (*WakeUp)(void))
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  
  /* store the settings and parameters */
  _handle->params = Params;
  _handle->settings = Settings;
  _handle->state = USBPD_CAD_STATE_RESET;
  _handle->cc = CCNONE;
  _handle->SNK_Source_Current_Adv = vRd_Undefined;
  Ports[PortNum].USBPD_CAD_WakeUp = WakeUp;
  CAD_Set_default_ResistorRp(PortNum,DPM_Settings[PortNum].CAD_DefaultResistor);
}


/**
  * @brief  CAD enters in error recovery mode 
  * @param  PortNum  Number of the port
  * @retval None
  */ 
void CAD_Enter_ErrorRecovery(uint8_t PortNum)
{
  /* remove resistor terminaison 
     switch CAD_StateMachine to Error Recovery state
     wakeup CAD task */
  USBPD_HW_IF_ErrorRecovery(PortNum);
}

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

