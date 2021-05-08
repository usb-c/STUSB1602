/**
  ******************************************************************************
  * @file    usbpd_Stusb_Dpm_if.h
  * @author  AMG Application Team
  * @brief   This file contains the headers of usbpd_Stusb_Dpm_if.h.
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
#include "usbpd_phy_hw_if.h"
#include "usbpd_porthandle.h"

#define TIMER_VCONNSTABLE 40u;
#define TIMER_DELAY_PE 15u;
#define TIMER_DELAY_ATTEMC 25u; 

/* Port management functions */
void HW_IF_Port_SetInitialRole(uint8_t PortNum,USBPD_PortPowerRole_TypeDef role);
void HW_IF_Port_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc);

/* IOs management functions */
void HW_IF_RESET_Assert(uint8_t PortNum);
void HW_IF_RESET_Deassert(uint8_t PortNum);
void HW_IF_STUSB16xx_Reset(uint8_t PortNum);



USBPD_StatusTypeDef USBPD_HW_IF_PortHwInit(uint8_t PortNum, USBPD_HW_IF_Callbacks cbs, USBPD_PortPowerRole_TypeDef role);
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rd(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rp(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Vbus_OFF(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);
USBPD_StatusTypeDef USBPD_HW_IF_PRS_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);

USBPD_StatusTypeDef USBPD_HW_IF_HR_Start(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HRPRS_Mode_TypeDef Mode);
USBPD_StatusTypeDef USBPD_HW_IF_HR_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole);

USBPD_StatusTypeDef USBPD_HW_IF_DataRole(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_HW_IF_ResetDataRole(uint8_t PortNum);
USBPD_StatusTypeDef USBPD_HW_IF_ErrorRecovery(uint8_t PortNum);
void USBPD_HW_IF_EnterErrorRecovery(uint8_t PortNum);
void USBPD_HW_IF_ExitErrorRecovery(uint8_t PortNum);

/* Public Monitoring functions */ 
USBPD_StatusTypeDef USBPD_HW_IF_CheckVBusPresence(uint8_t PortNum, uint32_t Timeout);
USBPD_StatusTypeDef USBPD_HW_IF_CheckVbusValid(uint8_t PortNum, uint32_t Timeout);
USBPD_StatusTypeDef USBPD_HW_IF_CheckVbusVSafe0V(uint8_t PortNum, uint32_t Timeout);
USBPD_StatusTypeDef USBPD_HW_IF_CheckVconnPresence(uint8_t PortNum, uint32_t Timeout);

STUSB1602_StatusTypeDef USBPD_Type_C_Command(uint8_t PortNum,Type_C_CTRL_TypeDef Ctrl );

