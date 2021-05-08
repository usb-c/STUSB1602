/**
******************************************************************************
* @file    usbpd_Stusb_Dpm_if.c
* @author  AMG
* @brief   This file contains DPM to Device interfaces functions.
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
#include "string.h" 

#include "usbpd_cad_hw_if.h"

#include "usbpd_stusb_dpm_if.h"
#include "usbpd_dpm_conf.h"

/** @addtogroup STM32_USBPD_LIBRARY
* @{
*/

/** @addtogroup USBPD_DEVICE
* @{
*/

/** @addtogroup USBPD_DEVICE_HW_IF
* @{
*/

/** @addtogroup USBPD_DEVICE_STUSB16XX_HW_IF
* @{
*/


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
* @def STUSB16xx_STATUS_AL_MASK
* @brief   The following definition is used to mask event interrupt and to prevent the assertion of the alert bit in the ALERT_STATUS register
* @details STUSB16xx_CC_DETECTION_STATUS_AL_MASK masks CC detection alerts
* @details STUSB16xx_MONITORING_STATUS_AL_MASK maks monitoring alerts
* @details STUSB16xx_FAULT_STATUS_AL_MASK masks fault alerts
*/
#define STUSB16xx_STATUS_AL_MASK = STUSB16xx_MONITORING_STATUS_AL_MASK | STUSB16xx_FAULT_STATUS_AL_MASK;


/* Private macro -------------------------------------------------------------*/

/**
@def CCXHANDLE(__CC__)
@brief It matches CC handle and CC line number
*/
#define CCXHANDLE(__CC__) ((CCxPin_TypeDef) (__CC__ +1))

/* Private variables ---------------------------------------------------------*/
uint8_t nvm_read = 0;                   /*!< Variable used to check if NVM has been loaded correctly */

extern uint8_t          TXRXBuffer0[];    /* Buffer for raw data received on port 0 */
#if (USBPD_PORT_COUNT == 2)
extern uint8_t          TXRXBuffer1[];    /* Buffer for raw data received on port 1 */
#endif

extern CAD_HW_HandleTypeDef CAD_HW_Handles[USBPD_PORT_COUNT]; /*!CAD state handle Structure */
extern USBPD_ParamsTypeDef DPM_Params[USBPD_PORT_COUNT];
extern void CAD_Set_default_ResistorRp(uint8_t PortNum, CAD_RP_Source_Current_Adv_Typedef RpValue);
/**
*  \warning   Position of USBPD_HW_IF_ErrorRecovery function has to be reviewed. If it is public it have be moved outside this file
*/



/**
* @brief  Handle for the ports inside @ref USBPD_DEVICE_HW_IF
*/
STUSB16xx_PORT_HandleTypeDef Ports[USBPD_PORT_COUNT] =
{
  { 0,                          /* USBPD PORT number 0 */
  (uint8_t *)TXRXBuffer0,        /* Pointer to Tx Buffer of port 0 */
  CCNONE,                     /* CC pin used for communication on port 0 */
  RESET,                      /* CC event change flag of port 0 */
  HAL_UNLOCKED,               /* Locking object of port 0 */
  HAL_USBPD_PORT_STATE_RESET, /* Communication state of port 0 */
  0,                          /* Error code of port 0 */
#if defined(_SRC)
  USBPD_PORTPOWERROLE_SRC,    /* As default the Port 0 plays the Provider power role */
#else
  USBPD_PORTPOWERROLE_SNK,    /* As default the Port 0 plays the Consumer power role */
#endif
  0,                          /* Index for monitoring BIST Msg bits on port 0 */
  ENABLE,                     /* VConn status flag of port 0 */
#if defined(_SRC)
  USBPD_PORTDATAROLE_DFP,     /* As default the Port 0 plays the DFP data role */
#else
  USBPD_PORTDATAROLE_UFP,     /* As default the Port 0 plays the UFP data role */
#endif
  0,                          /* Tx spare bits on port 0 */
  },
#if (USBPD_PORT_COUNT == 2)
  { 1,                         /* USBPD PORT number 1 */
  (uint8_t *)TXRXBuffer1,        /* Pointer to Tx Buffer of port 1 */
  CCNONE,                     /* CC pin used for communication on port 1 */
  RESET,                      /* CC event change flag of port 1 */
  HAL_UNLOCKED,               /* Locking object of port 1 */
  HAL_USBPD_PORT_STATE_RESET, /* Communication state of port 1 */
  0,                          /* Error code of port 1 */
#if defined(_SRC)
  USBPD_PORTPOWERROLE_SRC,    /* As default the Port 1 plays the Provider power role */
#else
  USBPD_PORTPOWERROLE_SNK,    /* As default the Port 1 plays the Consumer power role */
#endif
  0,                          /* Index for monitoring BIST Msg bits on port 1 */
  ENABLE,                     /* VConn status flag of port 1 */
#if defined(_SRC)
  USBPD_PORTDATAROLE_DFP,     /* As default the Port 1 plays the DFP data role */
#else
  USBPD_PORTDATAROLE_UFP,     /* As default the Port 1 plays the UFP data role */
#endif
  0,                          /* Tx spare bits on port 1 */
  },
#endif
};

/* Inner function prototypes -----------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_STUSB16XX_HW_IF_Public_Functions USBPD DEVICE STUSB16XX HW IF Public functions
* @details Public functions can be used at stack level
* @{
*/

/**
* @brief  It Initializes port harware interface
* @param  PortNum The port index
* @param  cbs The callbacks exposed by the HW_IF to the PHY
* @param  role The port power role
* @retval USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_PortHwInit(uint8_t PortNum, USBPD_HW_IF_Callbacks cbs, USBPD_PortPowerRole_TypeDef role)
{
  USBPD_StatusTypeDef res = USBPD_OK;
/* Control TypeC state */  
  switch (STUSB1602_TypeC_FSM_State_Get(STUSB1602_I2C_Add(PortNum)))
  {
  case DebugAccessory_SNK :
  case Attached_SNK:  
  case AttachWait_Accessory:
  case Powered_Accessory:  
    break;
  default :
    HW_IF_STUSB1602_Interrupt_CC_Detection(PortNum, DISABLE);
    HW_IF_STUSB1602_Interrupt_Monitoring(PortNum, DISABLE);
    STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), SW_RST); /* to avoid STUSB under SW reset after a reboot */ 
    __NOP();
    __NOP();
    STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), No_SW_RST); /* to avoid STUSB under SW reset after a reboot */ 
    HW_IF_RESET_CTRL(PortNum);
    break;
  }
          
  
  /* Set the power role of the port */
  HW_IF_Port_SetInitialRole(PortNum,role);
  /* Switch the port in RX mode */
  HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_RX);
  
  /* Alert interrupt init*/
  HW_IF_STUSB1602_Interrupt_CC_Detection(PortNum, ENABLE);
  
  
#ifdef __STAT  
  Ports[PortNum].unwrapdata.preamble_counter=0;;  
#endif   
  /* Initialize State and callbacks */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  Ports[PortNum].cbs = cbs;
  Ports[PortNum].role = role;
  Ports[PortNum].Error_Recovery_Flag = 0;
  
  switch (STUSB1602_TypeC_FSM_State_Get(STUSB1602_I2C_Add(PortNum)))
  {
  case DebugAccessory_SNK :
  case Attached_SNK: 
    CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_SWITCH_TO_SNK;
    break;
  case Attached_SRC:
    CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_SWITCH_TO_SRC;
  default :
    break;
  }  
  
  
  return res;
}


/**
* @brief   It connects Rp resitor on the CC lines
* @details This function is left empty because Rp is asserted by STUSB16xx when it acts the Provider power role
* @param   PortNum The port index
* @retval none
*/
void USBPDM1_AssertRp(uint8_t PortNum)
{
}


/**
* @brief   It disconnects Rp resitor on the CC lines
* @details This function is left empty because Rp is denied by STUSB16xx when it acts the Provider power role
* @param   PortNum The port index
* @retval none
*/
void USBPDM1_DeAssertRp(uint8_t PortNum)
{
}


/**
* @brief   It connects Rd resitor on the CC lines
* @details This function is left empty because Rd is asserted by STUSB16xx when it acts the Consumer power role
* @param   PortNum The port index
* @retval none
*/
void USBPDM1_AssertRd(uint8_t PortNum)
{
}


/**
* @brief   It disconnects Rd resitor on the CC lines
* @details This function is left empty because Rd is denied by STUSB16xx when it acts the Consumer power role
* @param   PortNum The port index
* @retval none
*/
void USBPDM1_DeAssertRd(uint8_t PortNum)
{
}

/**
* @brief   Power role swap: Rp resitor on the CC line
* @details It requests Rp assertion on CC line as step of power role swap USB PD transaction
* @param   PortNum The port index
* @param   CurrentRole    The port power role
* @retval  USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rp(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{

  /* i2c_pr_swap_rp_assert_req command */
  return (USBPD_StatusTypeDef)USBPD_Type_C_Command(PortNum, PD_PR_SWAP_RP_ASSERT_REQ);
}


/**
* @brief   Power role swap: Rd resitor on the CC line
* @details It requests Rd assertion on CC line as step of power role swap USB PD transaction
* @param   PortNum The port index
* @param   CurrentRole    The port power role
* @retval  USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Assert_Rd(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  /* i2c_pr_swap_rd_assert_req command */
  return (USBPD_StatusTypeDef)USBPD_Type_C_Command(PortNum, PD_PR_SWAP_RD_ASSERT_REQ);
}


/**
* @brief   Power role swap: VBUS OFF in sink role
* @details It requests VBUS OFF as step of power role swap USB PD transaction
* @param   PortNum The port index
* @param   CurrentRole  The port power role
* @retval  USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_PRS_Vbus_OFF(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  if (USBPD_FALSE == _handle->settings->CAD_RoleToggle)
  {
    return USBPD_ERROR;
  }
  
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  /* try to acquire the communication resource to avoid the conflict */
  
  if (CurrentRole == USBPD_PORTPOWERROLE_SRC)
  {
    
    /* i2c_pr_swap_src_vbus_off_req command */
    ret = (USBPD_StatusTypeDef)USBPD_Type_C_Command(PortNum, PD_PR_SWAP_SRC_VBUS_OFF_REQ);
    
    /* Turn off the power */
    HW_IF_STUSB1602_Interrupt_Monitoring(PortNum, ENABLE);
    /* enabling discharge */
    STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Discharge_Path_Enable);
    
    /* waiting for VSafe0V */
    if (USBPD_HW_IF_CheckVbusVSafe0V(PortNum, 300) == USBPD_TIMEOUT )
      return USBPD_ERROR;
    
    /* disabling discharge */
    STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Discharge_Path_Disable);
  }
  else if (CurrentRole == USBPD_PORTPOWERROLE_SNK)
  {
    /* i2c_pr_swap_snk_vbus_off_req command */
    ret = (USBPD_StatusTypeDef)USBPD_Type_C_Command(PortNum, PD_PR_SWAP_SNK_VBUS_OFF_REQ);
  }
  return ret;
}


/**
* @brief  Power role swap: end of procedure
* @details It notifies to the STUSB16xx device that PRS transaction is ending
* @param   PortNum The port index
* @param   CurrentRole    The port power role
* @retval USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_PRS_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  if (CurrentRole == USBPD_PORTPOWERROLE_SNK) /* initially it was SRC */
  {
    ret = (USBPD_StatusTypeDef)USBPD_Type_C_Command(PortNum, PD_PR_SWAP_PS_RDY_REQ);
  }
  else 
  {
    ret = USBPD_OK; /* initially it was SNK */
  }
  /* Set the exit from Attached.SNK to UnAttached.SNK on VBUS removed */
  ret = (USBPD_StatusTypeDef)STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);
  
  return ret;
}


/**
* @brief   Hard Reset: start of procedure
* @details It notifies to the STUSB16xx device that PRS transaction is starting
* @param   PortNum The port index
* @param   CurrentRole    The port power role
* @param   Mode           Two allowed values: ACKNOWLEDGE or REQUEST
* @retval  USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_HR_Start(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole, USBPD_HRPRS_Mode_TypeDef Mode)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  HW_IF_STUSB1602_Interrupt_Monitoring(PortNum, ENABLE);
  
  /* pd_hard_reset_received_req or pd_hard_reset_send_req command */
  ret = (USBPD_StatusTypeDef)USBPD_Type_C_Command(PortNum, Mode == ACKNOWLEDGE ? PD_HARD_RESET_RECEIVED_REQ : PD_HARD_RESET_SEND_REQ);
  
  
  if(Ports[PortNum].Monitoring_Status.b.VCONN_PRESENCE == VCONN_above_UVLO_threshold)
  {
    /* i2c_hard_reset_turn_off_vconn_req command */
    ret = (USBPD_StatusTypeDef)USBPD_Type_C_Command(PortNum, PD_HARD_RESET_TURN_OFF_VCONN_REQ);
  }
  
  
  /* if SRC role but Vconn was provided by SNK, need to turn ON Vconn after HardReset */
  if (Ports[PortNum].role == USBPD_PORTPOWERROLE_SRC)
  {
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_VCONN_SWAP_TURN_ON_VCONN_REQ);
  }
  
  
  return ret;
}


/* update Ports according to data role */
USBPD_StatusTypeDef USBPD_HW_IF_DataRole(uint8_t PortNum)
{
  Ports[PortNum].DataRole = (USBPD_PortDataRole_TypeDef)STUSB1602_Data_Role_Get(PortNum);
  
  return USBPD_ERROR;
}
USBPD_StatusTypeDef USBPD_HW_IF_ResetDataRole(uint8_t PortNum)
{
#if defined(_SRC)
  Ports[PortNum].DataRole = USBPD_PORTDATAROLE_DFP;
#else
  Ports[PortNum].DataRole = USBPD_PORTDATAROLE_UFP;
#endif
  return USBPD_ERROR;
} 

/* Public Monitoring functions */ 
/**
* @brief   It checks if Vbus is below the safe voltage threshold
* @param   PortNum The port index
* @param   @param   Timeout 0 no timeout checked > 0 timeout reported
* @retval  USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_CheckVbusVSafe0V(uint8_t PortNum, uint32_t Timeout)
{
#if _ADC_MONITORING
  if ( Timeout > 0 )
  {
    uint32_t tickstart = HAL_GetTick();
    do 
    {
      DPM_Ports[PortNum].DPM_MeasuredVbus = APPLI_GetVBUS(PortNum); /* this read is performe by Alert event that occurs each time monitored value has change */
    }  
    while ((DPM_Ports[PortNum].DPM_MeasuredVbus >= 800) && ((HAL_GetTick() - tickstart) <= Timeout));
    if ((HAL_GetTick() - tickstart) >= Timeout)
      return USBPD_TIMEOUT;
  }
    if (800 >= DPM_Ports[PortNum].DPM_MeasuredVbus)
    {
      Ports[PortNum].Monitoring_Status.b.VBUS_VSAFE0V = VBUS_below_VSAFE0V_threshold;
      return USBPD_OK;
    }
   return USBPD_ERROR;
#else
  if ( Timeout > 0 )
  {
    uint32_t tickstart = HAL_GetTick();
    do 
    {
#ifndef _RTOS
      Ports[PortNum].Monitoring_Trans = STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));
      if ( Ports[PortNum].Monitoring_Trans.d8 != 0)
        Ports[PortNum].Monitoring_Status = STUSB1602_Monitoring_Status_Reg_Get(STUSB1602_I2C_Add(PortNum));
#endif
      __NOP(); /* this read is performe by Alert event that occurs each time monitored value has change */
    }  
    while ((Ports[PortNum].Monitoring_Status.b.VBUS_VSAFE0V != VBUS_below_VSAFE0V_threshold && (HAL_GetTick() - tickstart) <= Timeout));
    if ((HAL_GetTick() - tickstart) >= Timeout)
      return USBPD_TIMEOUT;
  }
  if (Ports[PortNum].Monitoring_Status.b.VBUS_VSAFE0V == VBUS_below_VSAFE0V_threshold)
    return USBPD_OK;
  return USBPD_ERROR;  
#endif
}

/**
* @brief   It checks if Vbus is below the safe5V voltage threshold
* @param   PortNum The port index
* @param   Timeout 0 no timeout checked > 0 timeout reported 
* @retval  USBPD_StatusTypeDef :USBPD_OK ,USBPD_ERROR,USBPD_TIMEOUT
*/

USBPD_StatusTypeDef USBPD_HW_IF_CheckVbusValid(uint8_t PortNum, uint32_t Timeout)
{
  if ( Timeout > 0 )
  {
    uint32_t tickstart = HAL_GetTick();
    do 
    {
#ifndef _RTOS
    Ports[PortNum].Monitoring_Trans = STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));
      if ( Ports[PortNum].Monitoring_Trans.d8 != 0)
        Ports[PortNum].Monitoring_Status = STUSB1602_Monitoring_Status_Reg_Get(STUSB1602_I2C_Add(PortNum));
#endif
      __NOP(); /* this read is performe by Alert event that occurs each time monitored value has change */
    }
    while ((Ports[PortNum].Monitoring_Status.b.VBUS_VALID != VBUS_within_VALID_vrange && (HAL_GetTick() - tickstart) <= Timeout));
    if ((HAL_GetTick() - tickstart) >= Timeout)
      return USBPD_TIMEOUT;
  }
  if (Ports[PortNum].Monitoring_Status.b.VBUS_VALID == VBUS_within_VALID_vrange)
    return USBPD_OK;
  return USBPD_ERROR;  
  
}
/**
* @brief  It checks if VBus is present or not
* @param  PortNum The port index
* @retval USBPD_FunctionalState :USBPD_OK ,USBPD_ERROR,USBPD_TIMEOUT
*/
USBPD_StatusTypeDef USBPD_HW_IF_CheckVBusPresence(uint8_t PortNum, uint32_t Timeout)
{
  if ( Timeout > 0 )
  {
    uint32_t tickstart = HAL_GetTick();
    do 
    {
#ifndef _RTOS
    Ports[PortNum].Monitoring_Trans = STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));
      if ( Ports[PortNum].Monitoring_Trans.d8 != 0)
        Ports[PortNum].Monitoring_Status = STUSB1602_Monitoring_Status_Reg_Get(STUSB1602_I2C_Add(PortNum));
#endif
      __NOP(); /* this read is performe by Alert event that occurs each time monitored value has change */
    }
    while ((Ports[PortNum].Monitoring_Status.b.VBUS_PRESENCE != VBUS_above_UVLO_threshold && (HAL_GetTick() - tickstart) <= Timeout));
    if ((HAL_GetTick() - tickstart) >= Timeout)
      return USBPD_TIMEOUT;
  }
  if (Ports[PortNum].Monitoring_Status.b.VBUS_PRESENCE == VBUS_above_UVLO_threshold)
    return USBPD_OK;
  return USBPD_ERROR;    
}

/**
* @brief  It checks if Vconn is present or not
* @param  PortNum The port index
* @retval USBPD_FunctionalState :USBPD_OK ,USBPD_ERROR,USBPD_TIMEOUT
*/
USBPD_StatusTypeDef USBPD_HW_IF_CheckVconnPresence(uint8_t PortNum, uint32_t Timeout)
{
  if ( Timeout > 0 )
  {
    uint32_t tickstart = HAL_GetTick();
    do 
    {
#ifndef _RTOS
    Ports[PortNum].Monitoring_Trans = STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));
      if ( Ports[PortNum].Monitoring_Trans.d8 != 0)
        Ports[PortNum].Monitoring_Status = STUSB1602_Monitoring_Status_Reg_Get(STUSB1602_I2C_Add(PortNum));
#endif
      __NOP(); /* this read is performe by Alert event that occurs each time monitored value has change */
    }
    while ((Ports[PortNum].Monitoring_Status.b.VCONN_PRESENCE != VCONN_above_UVLO_threshold && (HAL_GetTick() - tickstart) <= Timeout));
    if ((HAL_GetTick() - tickstart) >= Timeout)
      return USBPD_TIMEOUT;
  }
  if (Ports[PortNum].Monitoring_Status.b.VCONN_PRESENCE == VCONN_above_UVLO_threshold)
    return USBPD_OK;
  return USBPD_ERROR;    
}

/**
* @brief   Hard Reset: end of procedure
* @details It notifies to the STUSB16xx device that PRS transaction is ending
* @param   PortNum The port index
* @param   CurrentRole The current port power role
* @retval  USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef USBPD_HW_IF_HR_End(uint8_t PortNum, USBPD_PortPowerRole_TypeDef CurrentRole)
{
  /* i2c_hard_reset_complete_req command */
  return (USBPD_StatusTypeDef)USBPD_Type_C_Command(PortNum, PD_HARD_RESET_COMPLETE_REQ);    
}




/**
* @brief   Error recovery function
* @param   PortNum The port index
* @retval  USBPD status 
*/
USBPD_StatusTypeDef USBPD_HW_IF_ErrorRecovery(uint8_t PortNum)
{
  USBPD_StatusTypeDef ret = USBPD_OK;
  Ports[PortNum].Error_Recovery_Flag = 1;
  /* Wakeup CAD to manage the error recovery exit */
  Ports[PortNum].USBPD_CAD_WakeUp();
  return ret;
}


void USBPD_HW_IF_EnterErrorRecovery(uint8_t PortNum)
{
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), SW_RST);
  STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum),VBUS_Discharge_Path_Enable);
  Ports[PortNum].Error_Recovery_Flag = 2;
  Ports[PortNum].USBPD_CAD_WakeUp();
}

void USBPD_HW_IF_ExitErrorRecovery(uint8_t PortNum)
{
  STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum),VBUS_Discharge_Path_Disable);
  HAL_GPIO_WritePin(TX_EN_GPIO_PORT(PortNum),TX_EN_GPIO_PIN(PortNum),GPIO_PIN_RESET); 
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), No_SW_RST);
  (void)STUSB1602_Attach_State_Trans_Get(STUSB1602_I2C_Add(PortNum));
  (void)STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));
#ifdef _TRACE
  USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "Exit ErrorRecovery", sizeof("Exit ErrorRecovery"));
#endif
}
/**
* @}
*/

/* Inner functions -----------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_STUSB16XX_HW_IF_Inner_Functions USBPD DEVICE STUSB16XX HW IF Inner functions
* @details Inner functions can be used at file level
* @{
*/



/**
* @brief  STUSB16xx software reset
* @param  PortNum The port index
* @retval None
*/ 
void HW_IF_RESET_CTRL(uint8_t PortNum)
{
  Attach_State_Trans_TypeDef state  ;
  STUSB1602_MONITORING_STATUS_TRANS_RegTypeDef Monit;      
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_RESET);  
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), SW_RST);
  STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum),VBUS_Discharge_Path_Enable);
  state = STUSB1602_Attach_State_Trans_Get(STUSB1602_I2C_Add(PortNum));
  UNUSED(state);
  Monit = STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));
  UNUSED(Monit); 
  STUSB1602_VBUS_Discharge_State_Set(STUSB1602_I2C_Add(PortNum),VBUS_Discharge_Path_Disable);
  STUSB1602_SW_RESET_Set(STUSB1602_I2C_Add(PortNum), No_SW_RST);  
  
}

/**
* @brief  Initialization of STUSB1602 GPIO pins status after reset
* @param  PortNum The port index
* @retval None
*/ 
void HW_IF_STUSB16xx_Reset(uint8_t PortNum)
{
   uint8_t j; 

  HW_IF_RESET_Assert(PortNum);
  for ( j = 0 ;j <=4 ;j++) /*4 time 4 ms timer */
  {  
   USBPD_TIM_Start(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC), 3000); /* 16ms reset */
  do
  {
    __NOP();
  }
  while (!USBPD_TIM_IsExpired(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC)) );
  }
  HW_IF_RESET_Deassert(PortNum);
  
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_RESET);
}

/**
* @brief  Assert STUSB16xx hardware reset
* @param  PortNum The port index
* @retval None
*/ 
void HW_IF_RESET_Assert(uint8_t PortNum)
{
  HAL_GPIO_WritePin(RESET_GPIO_PORT(PortNum), RESET_GPIO_PIN(PortNum), GPIO_PIN_SET);
}


/**
* @brief  Desert STUSB16xx hardware reset
* @param  PortNum The port index
* @retval None
*/ 
void HW_IF_RESET_Deassert(uint8_t PortNum)
{
  HAL_GPIO_WritePin(RESET_GPIO_PORT(PortNum), RESET_GPIO_PIN(PortNum), GPIO_PIN_RESET);
}

/**
* @brief  Initialization of the STUSB16xx registers according to the initial role
* @param  PortNum The port index
* @param  role
* @retval None
*/ 
void HW_IF_Port_SetInitialRole(uint8_t PortNum,USBPD_PortPowerRole_TypeDef role)
{
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  
  CAD_Set_default_ResistorRp(PortNum,DPM_Settings[PortNum].CAD_DefaultResistor);
  STUSB1602_VCONN_Discharge_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Discharge_Enable_250ms_on_CC_pin);
  
#if defined(_VDM) || defined(_VCONN_SUPPORT)
  STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);   
#else
  STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);
#endif
  STUSB1602_Data_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Data_Role_Swap_Enable);    
  STUSB1602_Power_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), Power_Role_Swap_Enable);    
  STUSB1602_VCONN_Role_Swap_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Role_Swap_Enable);
  STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);
  
  
  /*0x1E*/
  STUSB1602_VCONN_Switch_Current_Limit_Set(STUSB1602_I2C_Add(PortNum), ILIM_350_ma);
  
  
  /*0x20*/
  STUSB1602_VCONN_Monitor_Status_Set(STUSB1602_I2C_Add(PortNum), Enable_UVLO_thr_detect_on_VCONN); 
  STUSB1602_VCONN_UVLO_Thresh_Status_Set(STUSB1602_I2C_Add(PortNum), Lo_UVLO_thr_of_2_65_V);
  
  /*0x22*/
  STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), 20); 
  STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), -20); 
  
  STUSB1602_WriteRegSingle(0x99,STUSB1602_I2C_Add(PortNum), STUSB1602_VBUS_DISCHARGE_TIME_CTRL_REG);
  
  /*0x2E*/
  STUSB1602_VDD_OVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_OVLO_Enable);
  
  STUSB1602_VBUS_VSAFE0V_Threshold_Set(STUSB1602_I2C_Add(PortNum), VBUS_vSafe0V_Thr_0_6V); /* default value is VBUS_vSafe0V_Thr_0_6V, VBUS_vSafe0V_Thr_1_8V */
  STUSB1602_VDD_UVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_UVLO_Disable);
  
#if _PPS
     
  {
    STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5100); 
    STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000); 
    STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);
    STUSB1602_VDD_UVLO_Threshold_Set(STUSB1602_I2C_Add(PortNum), VDD_UVLO_Disable);
    STUSB1602_WriteRegSingle(0x91, STUSB1602_I2C_Add(PortNum), STUSB1602_VBUS_MONITORING_CTRL_REG);
    
  }  
#else 
  STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5100); 
  STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), 5000);
  STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
#endif
  
  
  if (USBPD_TRUE == _handle->settings->CAD_RoleToggle)
  {
    
    /*0x1F*/
   // STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), DRP_w_accessory_TrySRC_supp);//DRP_w_accessory_supp);  
    STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), DRP_w_accessory_supp);  
    
  }
  else
  {
    switch (role)
    {
      /* Consumer power role */
    case USBPD_PORTPOWERROLE_SNK:
      /*0x1F*/
      STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), SNK_without_accessory_supp);  
      /* Set the exit from Attached.SNK to UnAttached.SNK on VBUS removed */
      STUSB1602_SNK_Disconnect_Mode_Status_Set(STUSB1602_I2C_Add(PortNum), VBUS_or_SRC_removed);
      
      break;
      
      /* Provider power role */
    case USBPD_PORTPOWERROLE_SRC:
      /*0x1F*/
      STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), SRC_with_accessory_supp);  
      
      DPM_Ports[PortNum].DPM_origine = 5000;
#ifdef _BUCKCV
      APPLI_SetVoltage(PortNum,DPM_Ports[PortNum].DPM_origine);
#endif
      break;
      
    default:
      
      /*0x1F*/
   //   STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), DRP_w_accessory_TrySRC_supp);//DRP_w_accessory_supp);  
      STUSB1602_Power_Mode_Set(STUSB1602_I2C_Add(PortNum), DRP_w_accessory_supp);  
      
      break;
    }
  }
  
}


/**
* @brief  It assigns CC line number to Port Handle
* @param  PortNum The port index
* @param  cc CC line number
* @retval None
*/
void HW_IF_Port_Set_CC(uint8_t PortNum, CCxPin_TypeDef cc)
{
  Ports[PortNum].CCx = cc;
  Ports[PortNum].CCxChange = SET;
}

/**
* @brief  CAD state machine
* @details It returns details on CAD event referred to the CC line
* @param  PortNum The port index
* @param  Event Pointer to USBPD_CAD_EVENT 
* @param  CCXX Pointer to CCxPin_TypeDef
* @retval uint32_t
*/
uint32_t CAD_StateMachine(uint8_t PortNum, USBPD_CAD_EVENT *Event, CCxPin_TypeDef *CCXX)
{
  STUSB1602_CC_DETECTION_STATUS_RegTypeDef STUSB1602_CC_DETECTION_STATUS_Value;
  static USBPD_CAD_EVENT previous_event[USBPD_PORT_COUNT] = 
  {
    USBPD_CAD_EVENT_NONE
#if USBPD_PORT_COUNT == 2
      , USBPD_CAD_EVENT_NONE
#endif /*USBPD_PORT_COUNT == 2*/
  };
  CAD_HW_HandleTypeDef *_handle = &CAD_HW_Handles[PortNum];
  
  *Event = USBPD_CAD_EVENT_NONE;
  uint32_t timeout_end;
  uint32_t timeout;
  static uint32_t timeout_errorrecovery;
  /* Alert management */
  STUSB16xx_PORT_HandleTypeDef * hhw_handle = &Ports[PortNum];
  if (HAL_GPIO_ReadPin(ALERT_GPIO_PORT(PortNum), ALERT_GPIO_PIN(PortNum)) == GPIO_PIN_RESET)
  {
    hhw_handle->AlertEventCount = 1;
  }

  if( 1 == Ports[PortNum].Error_Recovery_Flag)
  {
	  /* An error recovery has been required by PE */
	  _handle->state = USBPD_CAD_STATE_ENTER_ERRORRECOVERY;
  }
  else
      if( 2 == Ports[PortNum].Error_Recovery_Flag)
  {
	  /* An error recovery has been required by PE */
	  _handle->state = USBPD_CAD_STATE_EXIT_ERRORRECOVERY;
  } 
  else {
  if (hhw_handle->AlertEventCount > 0)
  {
    /* try to acquire the communication resource to avoid the conflict */
    if (1)
    {
      if (STUSB16xx_HW_IF_Alert_Manager(PortNum) == USBPD_OK)
      {
        /* The alert was correctly served */
        hhw_handle->AlertEventCount = 0;
      }
    }
  }
  }
  
  /*Check CAD STATE*/
  switch(_handle->state)
  {
    
  case USBPD_CAD_STATE_ENTER_ERRORRECOVERY :
    /* Enter the error recovery ie disable the resistor */
    USBPD_HW_IF_EnterErrorRecovery(PortNum);
    *Event = USBPD_CAD_EVENT_DETACHED;
    /* Start the error recovery timer */
    timeout_errorrecovery = HAL_GetTick();
    _handle->state = USBPD_CAD_STATE_EXIT_ERRORRECOVERY;
    Ports[PortNum].Error_Recovery_Flag = 2;
    Ports[PortNum].USBPD_CAD_WakeUp();
    break;
    
  case USBPD_CAD_STATE_EXIT_ERRORRECOVERY :
    if( HAL_GetTick() - timeout_errorrecovery > 275)
    {
      /* Exit the error recovery ie enable the resistor */
      USBPD_HW_IF_ExitErrorRecovery(PortNum);
      Ports[PortNum].Error_Recovery_Flag = 0;
      _handle->state = USBPD_CAD_STATE_DETACHED;
    }
    break;

  case USBPD_CAD_STATE_DETACHED :
    {
      if ( (hhw_handle->Error_Recovery_Flag) == 0 &&  (hhw_handle->NbDetach == 0) )
      {
        PHY_HW_IF_TX_ABORT(PortNum);
        CAD_Set_default_ResistorRp(PortNum,DPM_Settings[PortNum].CAD_DefaultResistor);
        STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);
        hhw_handle->CableCapa5A = 0;
        if (hhw_handle -> Monitoring_Status.b.VBUS_VSAFE0V == 1)
        {
        hhw_handle->NbDetach = 1;
        }
        else
        {
          hhw_handle->NbDetach = 2;
        }
        *Event = USBPD_CAD_EVENT_DETACHED; 
      }
      else
      {
        if (hhw_handle->NbDetach == 1)
        {
          /* cover the case where stusb1602 becomes unattached to SRC and VBUS at strange level kept by other device */
          if  (STUSB1602_TypeC_FSM_State_Get(STUSB1602_I2C_Add(PortNum)) == Unattached_SNK)
          {
            if (Ports[PortNum].Monitoring_Status.b.VBUS_VSAFE0V != VBUS_below_VSAFE0V_threshold)
            {
              if ((STUSB1602_TypeC_FSM_State_Get(STUSB1602_I2C_Add(PortNum)) == AttachWait_SRC) && 
                  (Ports[PortNum].Monitoring_Status.b.VBUS_VSAFE0V != VBUS_below_VSAFE0V_threshold))
              {
                HW_IF_RESET_CTRL(PortNum);
                 *Event = USBPD_CAD_EVENT_DETACHED;
              }
            }
          }/* end if STUSB1602_TypeC_FSM_State_Get */
        }
        else 
        {
          if (hhw_handle->NbDetach == 2)
          {
            STUSB1602_CC_DETECTION_STATUS_RegTypeDef    STUSB1602_Cc_Detection_Value;
            STUSB1602_Cc_Detection_Value = STUSB1602_CC_Detection_Status_Get(STUSB1602_I2C_Add(PortNum));
            if (STUSB1602_Cc_Detection_Value.b.CC_ATTACH_STATE)
            {
//              hhw_handle->NbDetach = 0;
              /* USBPD_CAD_STATE_ATTACHED */
              CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_SWITCH_TO_SNK;
              HW_IF_Port_Set_CC(PortNum,CAD_HW_Handles[PortNum].cc);
              /* enable VBUS monitioring to get detach event */
              HW_IF_STUSB1602_Interrupt_Monitoring(PortNum, ENABLE);
            }
              hhw_handle->NbDetach = 3;
            *Event = USBPD_CAD_EVENT_DETACHED;
          }
          else
          {
            if (hhw_handle->NbDetach == 5)
            {
              CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_DETACHED;
              *Event = USBPD_CAD_EVENT_DETACHED;
              hhw_handle->NbDetach = 1;
              HW_IF_RESET_CTRL(PortNum);
 #ifdef _DEBUG_TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "***HW_IF_RESET_CTRL***", sizeof("***HW_IF_RESET_CTRL***"));
#endif
            }
          }
        }
      }
    } 
    break;
    /* end case */
    
  case USBPD_CAD_STATE_RESET :
    
    *Event = USBPD_CAD_EVENT_DETACHED;
    break;
  case USBPD_CAD_STATE_EMC :
  case USBPD_CAD_STATE_ACCESSORY :
  case USBPD_CAD_STATE_ATTEMC :
  case USBPD_CAD_STATE_ATTACHED :
    {
      Ports[PortNum].NbDetach = 0;
    }
    
    break;
    
  case USBPD_CAD_STATE_SWITCH_TO_SRC :
    {
    
    STUSB1602_CC_DETECTION_STATUS_Value = STUSB1602_CC_Detection_Status_Get(STUSB1602_I2C_Add(PortNum));
    _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SRC;
    _handle->params->PE_DataRole = USBPD_PORTDATAROLE_DFP;
    hhw_handle->role = USBPD_PORTPOWERROLE_SRC;
    timeout = HAL_GetTick();
    timeout_end = TIMER_DELAY_PE;
    while ((HAL_GetTick() - timeout) < timeout_end) ;
    
    if (STUSB1602_CC_DETECTION_STATUS_Value.b.CC_VCONN_SUPPLY_STATE)
    {
      _handle->state = USBPD_CAD_STATE_ATTEMC;
      *Event = USBPD_CAD_EVENT_ATTEMC;
      timeout = HAL_GetTick();
      timeout_end = TIMER_DELAY_ATTEMC;
      while ((HAL_GetTick() - timeout) < timeout_end);
    }
    else                                                     
    {
      _handle->state = USBPD_CAD_STATE_ATTACHED;
      *Event = USBPD_CAD_EVENT_ATTACHED;
    }
    
    hhw_handle->NbDetach =0;
    }
    break;
#ifdef _STUSB4761    
  case USBPD_CAD_STATE_DEBUG :
#endif 
  case USBPD_CAD_STATE_SWITCH_TO_SNK :
    {
    _handle->params->PE_PowerRole = USBPD_PORTPOWERROLE_SNK;
    _handle->params->PE_DataRole = USBPD_PORTDATAROLE_UFP;
    hhw_handle->role = USBPD_PORTPOWERROLE_SNK;
    _handle->state = USBPD_CAD_STATE_ATTACHED;
    *Event = USBPD_CAD_EVENT_ATTACHED;
    hhw_handle->NbDetach =0;
    }
    break;
    
  default:
    /* nothing to do */
    hhw_handle->NbDetach =0;
    break;
  }
  
  /* change of CAD state machine*/
  if ((hhw_handle->CCxChange == SET) && (_handle->state != USBPD_CAD_STATE_SWITCH_TO_SRC) && (_handle->state != USBPD_CAD_STATE_SWITCH_TO_SNK))
  {
    *CCXX  = _handle->cc;
    
    /* reset the flag */
    hhw_handle->CCxChange = RESET;
  }
  /* Report Detach event only if no already reported */
  if ((*Event != USBPD_CAD_EVENT_DETACHED) || (previous_event[PortNum] != USBPD_CAD_EVENT_DETACHED))
  {
    if (*Event == USBPD_CAD_EVENT_DETACHED)
    {
      CAD_Set_default_ResistorRp(PortNum,DPM_Settings[PortNum].CAD_DefaultResistor);
#if defined(_VDM) || defined(_VCONN_SUPPORT)
      STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Enable_on_CC_pin);
#else
      STUSB1602_VCONN_Supply_Status_Set(STUSB1602_I2C_Add(PortNum), VCONN_Supply_Capability_Disable_on_CC_pin);
#endif
    }
    /* report Event */
    previous_event[PortNum] = *Event;
  }
  else
  {
    /* report USBPD_CAD_EVENT_NONE */
    *Event = USBPD_CAD_EVENT_NONE;
  }
  
  return 2;
}

extern void USBPD_CAD_Task(void *argument);
/**
* @brief  It notifies that a new alert event occurred
* @param  PortNum The port index
* @retval None
*/ 
void STUSB16xx_HW_IF_Alert_Check(uint8_t PortNum)
{
 Ports[PortNum].AlertEventCount++;
 Ports[PortNum].USBPD_CAD_WakeUp();
}


/**
* @brief  It manages registers related to STUSB1602 ALERT interrupts
* @param  PortNum The port index
* @retval USBPD_StatusTypeDef
*/ 
uint8_t AlertAttempts = 10;

USBPD_StatusTypeDef STUSB16xx_HW_IF_Alert_Manager(uint8_t PortNum)
{
  STUSB1602_ALERT_STATUS_RegTypeDef           STUSB1602_Alert_Raise_Value;
  STUSB1602_CC_DETECTION_STATUS_RegTypeDef    STUSB1602_Cc_Detection_Value;
  
  STUSB1602_HW_FAULT_STATUS_TRANS_RegTypeDef    STUSB1602_HW_Fault_Status_Trans_Value;
  AlertAttempts = 10;
  
  do 
  {
    AlertAttempts --;
    STUSB1602_Alert_Raise_Value = STUSB1602_Alert_Raise_Get(STUSB1602_I2C_Add(PortNum));
    /* Connection status analysis*/    
    if ( STUSB1602_Alert_Raise_Value.b.CC_DETECTION_STATUS_AL && STUSB1602_Attach_State_Trans_Get(STUSB1602_I2C_Add(PortNum) )) 
    {
      STUSB1602_Cc_Detection_Value = STUSB1602_CC_Detection_Status_Get(STUSB1602_I2C_Add(PortNum));
      
      if ( STUSB1602_Cc_Detection_Value.b.CC_ATTACH_STATE )
      {
        switch (STUSB1602_Cc_Detection_Value.b.CC_ATTACH_MODE )
        {
        case Sink_Attached:
          /* USBPD_CAD_STATE_ATTACHED */
          CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_SWITCH_TO_SRC;
          /* enable VBUS monitioring to get detach event */
          HW_IF_STUSB1602_Interrupt_Monitoring(PortNum, ENABLE);
          
          break;
          
        case Source_Attached:
          /* USBPD_CAD_STATE_ATTACHED */
          CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_SWITCH_TO_SNK;
          HW_IF_STUSB1602_Interrupt_Monitoring(PortNum, ENABLE);
          break;
          
        case Audio_Acc_Attached:
          /* USBPD_CAD_STATE_ACCESSORY */
          CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_ACCESSORY;
          break;
          
        case Debug_Acc_Attached:
          /* USBPD_CAD_STATE_DEBUG */
          CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_DEBUG;
          break;
          
        case Powered_Acc_Attached:
          /*  */
          break;
          
        default:
          /* USPPD_CAD_STATE_UNKNOWN */
          CAD_HW_Handles[PortNum].state = USPPD_CAD_STATE_UNKNOW;
          break;
        }
        
        /* CAD handle is updated */
        
        CAD_HW_Handles[PortNum].cc = CCXHANDLE(STUSB1602_CCx_Pin_Attach_Get(STUSB1602_I2C_Add(PortNum)));
        
        /* Port handle is updated */
        HW_IF_Port_Set_CC(PortNum, CAD_HW_Handles[PortNum].cc);
        
        /* RX mode is enabled */
        HW_IF_RX_Enable(PortNum);
      }
      else  /* CC line is DETACHED */
      {
        HW_IF_STUSB1602_Interrupt_Monitoring(PortNum,DISABLE);
        /* CAD handle is updated */
        CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_DETACHED;
        CAD_HW_Handles[PortNum].cc = CCNONE;
        
        /* Port handle is updated */
        HW_IF_Port_Set_CC(PortNum,CAD_HW_Handles[PortNum].cc);
        
        /* TX mode is enabled */
        HW_IF_RX_Disable(PortNum);
      }
    }
    /* Monitoring analysis Vbus , Vconn */ /* registers 0x0F and 0x10 */
    if (STUSB1602_Alert_Raise_Value.b.MONITORING_STATUS_AL )
    {
      Ports[PortNum].Monitoring_Trans = STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));
      if ( Ports[PortNum].Monitoring_Trans.d8 != 0)
      {
        Ports[PortNum].Monitoring_Status = STUSB1602_Monitoring_Status_Reg_Get(STUSB1602_I2C_Add(PortNum));
        if ((Ports[PortNum].Monitoring_Status.b.VBUS_VSAFE0V == VBUS_below_VSAFE0V_threshold) && ( Attached_SNK == STUSB1602_TypeC_FSM_State_Get(STUSB1602_I2C_Add(PortNum))) && (0 == DPM_Ports[PortNum].DPM_FlagHardResetOngoing))
        {
          CAD_HW_Handles[PortNum].state = USBPD_CAD_STATE_DETACHED;
          Ports[PortNum].NbDetach = 5;
#ifdef _DEBUG_TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "NbDetach=5", sizeof("NbDetach=5"));
#endif
        }
      }
    }
    if ( STUSB1602_Alert_Raise_Value.b.HW_FAULT_STATUS_AL )
    {
      STUSB1602_HW_Fault_Status_Trans_Value = STUSB1602_Hard_Fault_Trans_Status_Get(STUSB1602_I2C_Add(PortNum));
      if (STUSB1602_HW_Fault_Status_Trans_Value.d8 != 0)
        Ports[PortNum].Hw_Fault.d8 =  STUSB1602_ReadRegSingle(STUSB1602_I2C_Add(PortNum), STUSB1602_HW_FAULT_STATUS_REG);
    }
  }
  while ((HAL_GPIO_ReadPin(ALERT_GPIO_PORT(PortNum), ALERT_GPIO_PIN(PortNum)) == GPIO_PIN_RESET)&& (AlertAttempts > 0));
  
  
  if (HAL_GPIO_ReadPin(ALERT_GPIO_PORT(PortNum), ALERT_GPIO_PIN(PortNum)) == GPIO_PIN_RESET)
  {
    /* I2C broken */
    
    volatile uint8_t loop=100; 
    while(loop != 0)
      
    {
      loop --;
    }
    return USBPD_ERROR; 
  }
  return USBPD_OK;
} 
/* Mapping table of ack messages */
const PD_TypeC_Handshake_TypeDef C_CTRL_Ack_Map[] = {
  TypeC_NoAck,                         /* NO_REQ (default)                    = 0 */
  PD_Hard_Reset_Complete_Ack,          /* PD_HARD_RESET_COMPLETE_REQ          = 1 */
  PD_Hard_Reset_Turn_Off_Vconn_Ack,    /* PD_HARD_RESET_TURN_OFF_VCONN_REQ    = 2 */
  PD_Hard_Reset_Port_Change_2_DFP_Ack, /* PD_HARD_RESET_PORT_CHANGE_2_DFP_REQ = 3 */
  PD_Hard_Reset_Port_Change_2_UFP_Ack, /* PD_HARD_RESET_PORT_CHANGE_2_UFP_REQ = 4 */
  PD_PR_Swap_Snk_Vbus_Off_Ack,         /* PD_PR_SWAP_SNK_VBUS_OFF_REQ         = 5 */
  PD_PR_Swap_Src_Vbus_Off_Ack,         /* PD_PR_SWAP_SRC_VBUS_OFF_REQ         = 6 */
  PD_PR_SWAP_Rp_Assert_Ack,            /* PD_PR_SWAP_RP_ASSERT_REQ            = 7 */
  PD_PR_SWAP_Rd_Assert_Ack,            /* PD_PR_SWAP_RD_ASSERT_REQ            = 8 */
  PD_DR_SWAP_Port_Change_2_DFP_Ack,    /* PD_DR_SWAP_PORT_CHANGE_2_DFP_REQ    = 9 */
  PD_DR_SWAP_Port_Change_2_UFP_Ack,    /* PD_DR_SWAP_PORT_CHANGE_2_UFP_REQ    = 10 */
  PD_VCONN_SWAP_Turn_On_VCONN_Ack,     /* PD_VCONN_SWAP_TURN_ON_VCONN_REQ     = 11 */
  PD_VCONN_SWAP_Turn_Off_VCONN_Ack,    /* PD_VCONN_SWAP_TURN_OFF_VCONN_REQ    = 12 */
  PD_PR_Swap_Ps_Rdy_Ack,               /* PD_PR_SWAP_PS_RDY_REQ               = 13 */
  PD_Hard_Reset_Received_Ack,          /* PD_HARD_RESET_RECEIVED_REQ          = 14 */
  PD_Hard_Reset_Send_Ack,              /* PD_HARD_RESET_SEND_REQ              = 15 */
}; /*!< Mapping table of ack commands */
/**
* @brief STUSB1602_Type_C_Command
* @param Addr I2C address of port controller device
* @param Ctrl Control to be set
* @retval STUSB1602_StatusTypeDef
*/
STUSB1602_StatusTypeDef USBPD_Type_C_Command(uint8_t PortNum,Type_C_CTRL_TypeDef Ctrl )
{
  STUSB1602_StatusTypeDef ret = STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), Ctrl); /* register */
  uint32_t timeout = 100 ;
  PD_TypeC_Handshake_TypeDef ackValue = C_CTRL_Ack_Map[(uint8_t)Ctrl];
  uint32_t tickstart = HAL_GetTick(); 
  UNUSED(ret); /* warning removal */
#ifndef _RTOS
    do 
  {
    Ports[PortNum].Monitoring_Trans = STUSB1602_Monitoring_Status_Trans_Reg_Get(STUSB1602_I2C_Add(PortNum));
      if ( Ports[PortNum].Monitoring_Trans.d8 != 0)
        Ports[PortNum].Monitoring_Status = STUSB1602_Monitoring_Status_Reg_Get(STUSB1602_I2C_Add(PortNum));
  }
#endif
  while(( Ports[PortNum].Monitoring_Trans.b.PD_TYPEC_HAND_SHAKE != ackValue )&& ((HAL_GetTick() - tickstart) <= timeout));
  if ((HAL_GetTick() - tickstart) >= timeout)
  {
#ifdef _DEBUG_TRACE
    uint8_t tab[32];
    uint8_t size;
    size = sprintf((char*)tab, "Type C Timeout %d", Ctrl);
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size);    
#endif    
    return STUSB1602_TIMEOUT;
  }
  else
  {
#ifdef _DEBUG_TRACE
    uint8_t tab[32];
    uint8_t size;
    size = sprintf((char*)tab, "Type C %d", Ctrl);
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size); 
#endif   
    return STUSB1602_OK;
  }
}


/**
* @brief  VConn swap management
* @param  PortNum The port index
* @retval USBPD_StatusTypeDef 
*/
USBPD_StatusTypeDef STUSB16xx_HW_IF_VConnSwap(uint8_t PortNum)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  if (DPM_Params[PortNum].VconnStatus == USBPD_TRUE)
  {
    Ports[PortNum].VConn = ENABLE;
  }
  else
  {
    Ports[PortNum].VConn = DISABLE;
  }
  if (Ports[PortNum].VConn == DISABLE)
  {                                                                                                 
    /* i2c_vconn_swap_turn_on_vconn_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_VCONN_SWAP_TURN_ON_VCONN_REQ); 
    ret = USBPD_HW_IF_CheckVconnPresence(PortNum, 1) ;
  }
  else
  {
    /* i2c_vconn_swap_turn_off_vconn_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_VCONN_SWAP_TURN_OFF_VCONN_REQ);    
    ( USBPD_HW_IF_CheckVconnPresence(PortNum, 1) ==  USBPD_ERROR ? USBPD_OK : USBPD_ERROR );
    
  }
  
  return ret;
}


/**
* @brief  Data Role swap management
* @param  PortNum The port index
* @retval USBPD_StatusTypeDef 
*/
USBPD_StatusTypeDef STUSB16xx_HW_IF_DataRoleSwap(uint8_t PortNum)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  if (Ports[PortNum].DataRole == USBPD_PORTDATAROLE_DFP)
  {
    /* i2c_dr_swap_port_change_2_ufp_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_DR_SWAP_PORT_CHANGE_2_UFP_REQ);    
    /* update Ports variable for datarole */
    Ports[PortNum].DataRole = USBPD_PORTDATAROLE_UFP;
  }
  else if (Ports[PortNum].DataRole == USBPD_PORTDATAROLE_UFP)
  {
    /* i2c_dr_swap_port_change_2_dfp_req command */
    ret = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_DR_SWAP_PORT_CHANGE_2_DFP_REQ);    
    /* update Ports variable for datarole */
    Ports[PortNum].DataRole = USBPD_PORTDATAROLE_DFP;
  }
  else
    ret = USBPD_ERROR;
  
  return ret;
}


/**
* @brief  It sets the VBus monitoring reference voltage as well as the upper and lower tolerance threshold
* @param  PortNum The port index
* @param  VBus It is the VBus monitoring reference voltage (expressed in mV)
* @param  Hset It is the upper tolerance (expressed in % and >= 5%)
* @param  Lset It is the lower tolerance (expressed in % and >= 5%)
* @retval USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef STUSB16xx_HW_IF_Set_VBus_Monitoring(uint8_t PortNum, uint16_t VBus, uint8_t Hset, uint8_t Lset)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  /* check for parameters*/
  if (Hset > 15) Hset = 15;
  if (Lset > 15) Lset = 15;
  
  /* Sets the VBUS_SELECT DAC reference for VBUS sensing (bit7:0 0x21) */
  ret = (USBPD_StatusTypeDef)STUSB1602_VBUS_Select_Status_Set(STUSB1602_I2C_Add(PortNum), VBus);
  
  /* Sets the VBUS_VShift_High and VBUS_VShift_Low */
   ret = (USBPD_StatusTypeDef)STUSB1602_VBUS_VShift_Set(STUSB1602_I2C_Add(PortNum), Hset, Lset);

  /* Sets the VBUS_VShift_High (bit7:4 0x22) */
 // ret = (USBPD_StatusTypeDef)STUSB1602_VBUS_VShift_High_Set(STUSB1602_I2C_Add(PortNum), Hset);
  
  /* Sets the VBUS_VShift_Low (bit3:0 0x22) */
//  ret = (USBPD_StatusTypeDef)STUSB1602_VBUS_VShift_Low_Set(STUSB1602_I2C_Add(PortNum), Lset);
  
  return ret;
}

/**
* @brief  Get Sink Resistors State
* @param  PortNum The port index
* @retval USBPD_FALSE of USBPD_TRUE
*/
uint8_t USBPD_16xx_IsResistor_SinkTxOk(uint8_t PortNum)
{
  uint8_t TX_OK;
  
  TX_OK = STUSB1602_Sink_Power_State_Get(STUSB1602_I2C_Add(PortNum));
  
  if ( TX_OK == Pwr_3_0_SNK)
  {
#ifdef _TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "TX ok", sizeof("TX ok"));
#endif
    return USBPD_TRUE;
  }
  else
  {
#ifdef _TRACE
    USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "TX NG", sizeof("TX NG"));
#endif
    return USBPD_FALSE;
  }
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

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
