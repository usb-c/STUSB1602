/**
******************************************************************************
* @file    usbpd_pwr_if.c
* @author  MCD Application Team
* @brief   This file contains power interface control functions.
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

#define __USBPD_PWR_IF_C

/* Includes ------------------------------------------------------------------*/
#include "User_BSP.h"
#include "usbpd_pwr_if.h"
#include "usbpd_dpm_user.h"
#include "usbpd_dpm_core.h"
#include "usbpd_dpm_conf.h"
#include "usbpd_pdo_defs.h"
#if defined(_TRACE)
#include "usbpd_core.h"
#include "usbpd_trace.h"
#endif /* _TRACE */
#include "string.h"
/** @addtogroup STM32_USBPD_APPLICATION
* @{
*/

/** @addtogroup STM32_USBPD_APPLICATION_POWER_IF
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup STM32_USBPD_APPLICATION_POWER_IF_Private_Defines
* @{
*/

#if ((PORT0_NB_SOURCEPDO) > USBPD_MAX_NB_PDO)
#error "Nb of Source PDO/APDO is exceeding stack capabilities"
#endif
#if ((PORT0_NB_SINKPDO) > USBPD_MAX_NB_PDO)
#error "Nb of Sink PDO/APDO is exceeding stack capabilities"
#endif
#if USBPD_PORT_COUNT == 2
#if ((PORT1_NB_SOURCEPDO) > USBPD_MAX_NB_PDO)
#error "Nb of Source PDO/APDO is exceeding stack capabilities"
#endif
#if ((PORT1_NB_SINKPDO) > USBPD_MAX_NB_PDO)
#error "Nb of Sink PDO/APDO is exceeding stack capabilities"
#endif
#endif /* USBPD_PORT_COUNT == 2 */

/**
* @}
*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup STM32_USBPD_APPLICATION_POWER_IF_Private_Macros
* @{
*/

#define _PWR_UPDATE_VOLTAGE_MIN(_PDO_VOLT_, _SNK_VOLT_) \
if ((_PDO_VOLT_) < (_SNK_VOLT_)) \
  { \
    /* Update min voltage */ \
      (_SNK_VOLT_) = (_PDO_VOLT_); \
  }
#define _PWR_UPDATE_VOLTAGE_MAX(_PDO_VOLT_, _SNK_VOLT_) \
if ((_PDO_VOLT_) > (_SNK_VOLT_)) \
  { \
    /* Update min voltage */ \
      (_SNK_VOLT_) = (_PDO_VOLT_); \
  }
#define _PWR_UPDATE_CURRENT_MAX(_PDO_CURRENT_, _SNK_CURRENT_) \
if ((_PDO_CURRENT_) > (_SNK_CURRENT_)) \
  { \
    /* Update min current */ \
      (_SNK_CURRENT_) = (_PDO_CURRENT_); \
  }
#define _PWR_UPDATE_POWER_MAX(_PDO_POWER_, _SNK_POWER_) \
if ((_PDO_POWER_) > (_SNK_POWER_)) \
  { \
    /* Update min POWER */ \
      (_SNK_POWER_) = (_PDO_POWER_); \
  }
#define _PWR_CHECK_VOLTAGE_MIN(_PDO_VOLT_, _SNK_VOLT_) \
      if ((_PDO_VOLT_) < (_SNK_VOLT_)) \
        { \
          /* Disalignment between PDO and DPM_SNKRequestedPower structure */ \
            _status = USBPD_ERROR; \
        }
#define _PWR_CHECK_VOLTAGE_MAX(_PDO_VOLT_, _SNK_VOLT_) \
      if ((_PDO_VOLT_) > (_SNK_VOLT_)) \
        { \
          /* Disalignment between PDO and DPM_SNKRequestedPower structure */ \
            _status = USBPD_ERROR; \
        }
#define _PWR_CHECK_CURRENT_MAX(_PDO_CURRENT_, _SNK_CURRENT_) \
      if ((_PDO_CURRENT_) > (_SNK_CURRENT_)) \
        { \
          /* Disalignment between PDO and DPM_SNKRequestedPower structure */ \
            _status = USBPD_ERROR; \
        }
#define _PWR_CHECK_POWER_MAX(_PDO_POWER_, _SNK_POWER_) \
      if ((_PDO_POWER_) > (_SNK_POWER_)) \
        { \
          /* Disalignment between PDO and DPM_SNKRequestedPower structure */ \
            _status = USBPD_ERROR; \
        }
      
      /**
      * @}
      */
      /* Private variables ---------------------------------------------------------*/
      /** @addtogroup STM32_USBPD_APPLICATION_POWER_IF_Private_Variables
      * @{
      */
      /**
      * @brief  USBPD Port PDO Storage array declaration
      */
      
      /**** PDO ****/
      USBPD_PWR_Port_PDO_Storage_TypeDef PWR_Port_PDO_Storage[USBPD_PORT_COUNT];
      
      /**
      * @}
      */
      /* Private function prototypes -----------------------------------------------*/
      /** @addtogroup STM32_USBPD_APPLICATION_POWER_IF_Private_Functions
      * @{
      */
      
      /* Functions to initialize Source PDOs */
      uint32_t _PWR_SRCFixedPDO(float  _C_, float _V_,
                                USBPD_CORE_PDO_PeakCurr_TypeDef _PK_,
                                USBPD_CORE_PDO_DRDataSupport_TypeDef DRDSupport,
                                USBPD_CORE_PDO_USBCommCapable_TypeDef UsbCommCapable,
                                USBPD_CORE_PDO_ExtPowered_TypeDef ExtPower,
                                USBPD_CORE_PDO_USBSuspendSupport_TypeDef UsbSuspendSupport,
                                USBPD_CORE_PDO_DRPowerSupport_TypeDef DRPSupport);
      
      uint32_t _PWR_SRCVariablePDO(float _MAXV_, float _MINV_, float _C_);
      
      uint32_t _PWR_SRCBatteryPDO(float _MAXV_,float _MINV_,float _PWR_);
      /* Functions to initialize Sink PDOs */
      
      uint32_t _PWR_SNKFixedPDO(float  _C_, float _V_,
                                USBPD_CORE_PDO_DRDataSupport_TypeDef DRDSupport,
                                USBPD_CORE_PDO_USBCommCapable_TypeDef UsbCommCapable,
                                USBPD_CORE_PDO_ExtPowered_TypeDef ExtPower,
                                USBPD_CORE_PDO_HigherCapability_TypeDef HigherCapab,
                                USBPD_CORE_PDO_DRPowerSupport_TypeDef DRPSupport);
      
      uint32_t _PWR_SNKVariablePDO(float  _MAXV_,float _MINV_,float _C_);
      
      uint32_t _PWR_SNKBatteryPDO(float _MAXV_,float _MINV_,float _PWR_);
      
#if defined(USBPD_REV30_SUPPORT)
#endif /* USBPD_REV30_SUPPORT */
      
      /**
      * @}
      */
      
      /** @addtogroup STM32_USBPD_APPLICATION_POWER_IF_Exported_Functions
      * @{
      */
      
      /**
      * @brief  Initialize structures and variables related to power board profiles
      *         used by Sink and Source, for all available ports.
      * @retval USBPD status
      */
      /* GNU Compiler */
#if defined(__GNUC__)
      /* ARM Compiler */
#elif defined(__CC_ARM)
      /* IAR Compiler */
#elif defined(__ICCARM__)
#pragma optimize=none
#endif
      USBPD_StatusTypeDef USBPD_PWR_IF_Init(void)
      {
        USBPD_StatusTypeDef _status = USBPD_OK;
        
        /* Set links to PDO values and number for Port 0 (defined in PDO arrays in H file).
        */
        PWR_Port_PDO_Storage[USBPD_PORT_0].SourcePDO.ListOfPDO = (uint32_t *) PORT0_PDO_ListSRC;
        PWR_Port_PDO_Storage[USBPD_PORT_0].SourcePDO.NumberOfPDO = &USBPD_NbPDO[1];
        
        
        PWR_Port_PDO_Storage[USBPD_PORT_0].SinkPDO.ListOfPDO = (uint32_t *)PORT0_PDO_ListSNK;
        
        PWR_Port_PDO_Storage[USBPD_PORT_0].SinkPDO.NumberOfPDO = &USBPD_NbPDO[0];
        _status |= USBPD_PWR_IF_CheckUpdateSNKPower(USBPD_PORT_0);
        
#if USBPD_PORT_COUNT == 2
        /* Set links to PDO values and number for Port 1 (defined in PDO arrays in H file).
        */
#if (PORT1_NB_SOURCEPDO > 0) || (PORT1_NB_SOURCEAPDO > 0)
        
        PWR_Port_PDO_Storage[USBPD_PORT_1].SourcePDO.ListOfPDO = (uint32_t *) PORT1_PDO_ListSRC;
        
        PWR_Port_PDO_Storage[USBPD_PORT_1].SourcePDO.NumberOfPDO = &USBPD_NbPDO[3];
#endif
        
#if (PORT1_NB_SINKPDO > 0) || (PORT1_NB_SINKAPDO > 0)
        
        PWR_Port_PDO_Storage[USBPD_PORT_1].SinkPDO.ListOfPDO = (uint32_t *)PORT1_PDO_ListSNK;
        
        PWR_Port_PDO_Storage[USBPD_PORT_1].SinkPDO.NumberOfPDO = &USBPD_NbPDO[2];
        _status |= USBPD_PWR_IF_CheckUpdateSNKPower(USBPD_PORT_1);
#endif
        
#endif /* USBPD_PORT_COUNT == 2 */
        
        return _status;
      }
      
      /**
      * @brief  Sets the required power profile, now it works only with Fixed ones  
      * @param  PortNum               Port number
      * @param  Profile          Number of the required Power Data Objects
      * @param  PreviousPowerProfile  Number of the previous required Power Data Objects
      * @retval USBPD status
      */
USBPD_StatusTypeDef USBPD_PWR_IF_SetProfile(uint8_t PortNum, uint8_t Profile, uint8_t PreviousPowerProfile)
      {
        uint32_t vbusTargetInmv;
        USBPD_PDO_TypeDef pdo;
#if (PORT0_NB_SOURCEPDO >= 2) || (PORT0_NB_SOURCEAPDO > 0)
        uint32_t delta_val;
        uint32_t step;
        uint32_t new_val;
        uint32_t min;
        uint32_t max;
        uint8_t pos;
        uint32_t origine_new;
        uint32_t transition;
        uint32_t vbusmin;
        uint32_t vbusmax;
        uint32_t origine;
        uint32_t timeout = BSP_PWR_TIMEOUT_PDO;
        uint32_t timeout_end;  
        USBPD_SNKRDO_TypeDef rdo;
#endif /* (PORT0_NB_SOURCEPDO >= 2)|| (PORT0_NB_SOURCEAPDO > 0) */
        USBPD_StatusTypeDef      ret = USBPD_OK;
        
        /* Check if valid port */
        if ( !USBPD_PORT_IsValid(PortNum) )
        {
          return USBPD_ERROR;
        }
        
#if (PORT0_NB_SOURCEPDO >= 2)|| (PORT0_NB_SOURCEAPDO > 0)
        /* Check if profile nb is valid for this port */
        /* SRC Mode */
        if (USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
        {
#ifdef _GPIO_FOR_SRC
          /* disable internal monitoing during PDO transition */
          STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Disable);
#endif
          if (Profile >= *PWR_Port_PDO_Storage[PortNum].SourcePDO.NumberOfPDO)
          {
            return USBPD_ERROR;
          }
          if (0 == *PWR_Port_PDO_Storage[PortNum].SourcePDO.NumberOfPDO)
          {
            return USBPD_ERROR;
          }
          pdo.d32 = PWR_Port_PDO_Storage[PortNum].SourcePDO.ListOfPDO[Profile];
          
          if(pdo.GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_APDO)
          {
            rdo.d32        = DPM_Ports[PortNum].DPM_RcvRequestDOMsg;
            vbusTargetInmv = rdo.ProgRDO.OutputVoltageIn20mV * 20;
            
          } 
          /**end of adjust target value vs current drawn from sink */
          else
            
          {
            /* Case PDO is a fixed type */
            if (pdo.GenericPDO.PowerObject == USBPD_PDO_TYPE_FIXED)
            {
              vbusTargetInmv = (pdo.SRCFixedPDO.VoltageIn50mVunits * 50);
            }
            else
            {
              return USBPD_ERROR;
            }
          }
          
          if (USBPD_ENABLE == USBPD_PWR_IF_VBUSIsEnabled(PortNum))
          {
            /* We ask for 100mV precision */
            /* store setting of previous step*/
            origine = DPM_Ports[PortNum].DPM_origine;
            origine_new =DPM_Ports[PortNum].DPM_origine;
            vbusTargetInmv = vbusTargetInmv;
#ifdef _TRACE
            uint8_t tab[32];
            uint8_t size;
            size = sprintf((char*)tab, "origine: %d", origine);
            USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size);
            size = sprintf((char*)tab, "vbusTargetInmv: %d", vbusTargetInmv);
            USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size); 
#endif
            
#ifdef _GPIO_FOR_SRC
            APPLI_SetVoltage(PortNum,DPM_Ports[PortNum].DPM_RequestedVoltage);
#endif
#ifndef _GPIO_FOR_SRC
            if (origine != vbusTargetInmv)
            {
              if (vbusTargetInmv > origine)
              {
                delta_val = vbusTargetInmv - origine;
                new_val=origine;
                step = 20;
                pos = 1;
              }
              else
              { 
                delta_val = origine - vbusTargetInmv;
                new_val=origine;
                step = 20;
                pos = 0;
              }
              
              if (delta_val >500)
                transition = 1;
              else 
                transition = 1;
              
              if (delta_val >=160)
                step = 100;
              else 
                step = 20;
              if ( (delta_val > step) && (pos == 1) )
              {
                do
                {
                  new_val = new_val + step;
                  max = (110 * new_val)/100 ; /*110*/
                  min = (90 * new_val)/100 ;  /*90*/
                  APPLI_SetVoltage(PortNum, new_val);
                  /* Get Start Tick*/
                  timeout_end = timeout;
                  timeout = HAL_GetTick();
                  HAL_Delay(transition);
                  while  ((((  APPLI_GetVBUS(PortNum)) > max ) || ((  APPLI_GetVBUS(PortNum)) < min ) )  )           
                  {
                    /* Check if timeout to get voltage expired or not */
                    if ((HAL_GetTick() - timeout) > timeout_end)
                    {
                      return USBPD_ERROR;
                    }
                  }
                } while ( new_val < vbusTargetInmv);
              }
              else
              {
                if ( (delta_val > step) && (pos == 0))
                {
                  do
                  {
                    new_val = new_val - step;
                    max = (110 * new_val)/100 ;
                    min = (90 * new_val)/100 ;
                    /* Get Start Tick*/
                    APPLI_SetVoltage(PortNum, new_val);
                    {
#ifdef _TRACE
                      uint8_t tab[32];
                      uint8_t size;
                      size = sprintf((char*)tab, "new_val: %d", new_val);
                      USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size);
#endif
                    }              
                    timeout_end = timeout;
                    timeout = HAL_GetTick();
                    
                    osDelay(transition);
                    while  (((  APPLI_GetVBUS(PortNum)  ) > max ) || ((  APPLI_GetVBUS(PortNum)) < min )) ;
                    {
                      {
#ifdef _TRACE
                        uint8_t tab[32];
                        uint8_t size;
                        size = sprintf((char*)tab, "new_val done: %d", new_val);
                        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size); 
#endif
                      }
                      /* Check if timeout to get voltage expired or not */
                      if ((HAL_GetTick() - timeout) > timeout_end)
                      {
#ifdef _TRACE
                        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0,(uint8_t *)"Timeout", sizeof("Timeout"));
#endif
                        return USBPD_ERROR;
                      }
                    }
                  } while ( new_val > vbusTargetInmv);
                  
                }
              }
              if (delta_val <= step)
              {
                APPLI_SetVoltage(PortNum, vbusTargetInmv);
                if(pdo.GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_APDO)
                {
                  APPLI_GetVBUS(PortNum);
                  vbusmin = 90* vbusTargetInmv /100;
                  vbusmax = 110* vbusTargetInmv / 100;
                  HAL_Delay(transition);
                  /* Get Start Tick*/
                  timeout_end = timeout;
                  timeout = HAL_GetTick();
                  /* Initialize timeout value depending on the PDO */
                  while  (((  APPLI_GetVBUS(PortNum)  ) > vbusmax) || ((  APPLI_GetVBUS(PortNum) ) < vbusmin))
                  {
                    /* OS delay used to allow task preemption */
                    /* Check if timeout to get voltage expired or not */
                    if ((HAL_GetTick() - timeout) > timeout_end)
                    {
                      return USBPD_ERROR;
                    }
                  }
                }
              } /* delta <=500 */
              origine_new = new_val;
            }
            /* now delta < 160 */
            if (origine_new != vbusTargetInmv)
            {
              if (vbusTargetInmv > origine_new)
              {
                delta_val = vbusTargetInmv - origine_new;
                new_val=origine_new;
                step = 20;
                pos = 1;
              }
              else
              { 
                delta_val = origine_new - vbusTargetInmv;
                new_val=origine_new;
                step = 20;
                pos = 0;
              }
              
              if (delta_val >500)
                transition = 10;
              else 
                transition = 1;
              
              if (delta_val >=160)
                step = 100;
              else 
                step = 20;
              
              if ( (delta_val > step) && (pos == 1) )
              {
                do
                {
                  new_val = new_val + step;
                  max = (110 * new_val)/100 ; /*110*/
                  min = (90 * new_val)/100 ;  /*90*/
                  APPLI_SetVoltage(PortNum, new_val);
                  /* Get Start Tick*/
                  timeout_end = timeout;
                  timeout = HAL_GetTick();
                  while  ((((  APPLI_GetVBUS(PortNum)) > max ) || ((  APPLI_GetVBUS(PortNum)) < min ) )  )           
                  {
                    /* Check if timeout to get voltage expired or not */
                    if ((HAL_GetTick() - timeout) > timeout_end)
                    {
                      return USBPD_ERROR;
                    }
                  }
                } while ( new_val < vbusTargetInmv);
              }
              else
              {
                if ( (delta_val > step) && (pos == 0))
                {
                  do
                  {
                    new_val = new_val - step;
                    max = (110 * new_val)/100 ;
                    min = (90 * new_val)/100 ;
                    /* Get Start Tick*/
                    timeout_end = timeout;
                    timeout = HAL_GetTick();
                    APPLI_SetVoltage(PortNum, new_val);
                    while  (((  APPLI_GetVBUS(PortNum)  ) > max ) || ((  APPLI_GetVBUS(PortNum)) < min )) ;
                    {
                      /* Check if timeout to get voltage expired or not */
                      if ((HAL_GetTick() - timeout) > timeout_end)
                      {
                        return USBPD_ERROR;
                      }
                    }
                  } while ( new_val > vbusTargetInmv);
                }
              }
              if (delta_val <= step)
              {
                APPLI_SetVoltage(PortNum, vbusTargetInmv);
                if(pdo.GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_APDO)
                {
                  APPLI_GetVBUS(PortNum);
                  vbusmin = 90* vbusTargetInmv /100;
                  vbusmax = 110* vbusTargetInmv / 100;
                  /* Get Start Tick*/
                  timeout_end = timeout;
                  timeout = HAL_GetTick();
                  /* Initialize timeout value depending on the PDO */
                  while  (((  APPLI_GetVBUS(PortNum)  ) > vbusmax) || ((  APPLI_GetVBUS(PortNum) ) < vbusmin))
                  {
                    /* Check if timeout to get voltage expired or not */
                    if ((HAL_GetTick() - timeout) > timeout_end)
                    {
                      return USBPD_ERROR;
                    }
                  }
                }
              } /* delta <=500 */
            }
            
            /* end delta < 160 */
#endif /*  _GPIO_FOR_SRC */            
          }
          else  /* USBPD_DISABLE */
          {
            
            APPLI_SetVoltage(PortNum, 5000);
            APPLI_GetVBUS(PortNum);
            DPM_Ports[PortNum].DPM_origine = 5000;
            APPLI_SetVoltage(PortNum,DPM_Ports[PortNum].DPM_origine);
          }
          
#ifdef _GPIO_FOR_SRC
          /* end of voltage transition: re enable voltage monitoring */
          STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
          /* delay added to wait for power stabilization - Vs standard, VBUS has up to 275ms to stabilize between transition */	
          USBPD_DPM_WaitForTime(100);     
#endif
          
#ifdef _TRACE
          uint8_t tab[32];
          uint8_t size;
          USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "end of setprofile", sizeof("end of setprofile"));
          size = sprintf((char*)tab, "vbus set: %d", vbusTargetInmv);
          USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size); 
          size = sprintf((char*)tab, "CurrentMeas: %d", DPM_Ports[PortNum].DPM_MeasuredCurrent);
          USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size); 
#endif   
          DPM_Ports[USBPD_PORT_0].DPM_SrcOutputVoltage = vbusTargetInmv;
          DPM_Ports[PortNum].DPM_origine = vbusTargetInmv;
          DPM_Ports[PortNum].DPM_FlagSetupNewPowerOngoing = 0;
        }
#else /* not GENERATOR_MB1303 */
        /* SRC Mode */
        if (USBPD_PORTPOWERROLE_SRC == DPM_Params[PortNum].PE_PowerRole)
        {
          /* Check if profile nb is valid for this port */
          if (Profile >= *PWR_Port_PDO_Storage[PortNum].SourcePDO.NumberOfPDO)
          {
            return USBPD_ERROR;
          }
          if (0 == *PWR_Port_PDO_Storage[PortNum].SourcePDO.NumberOfPDO)
          {
            return USBPD_ERROR;
          }
          pdo.d32 = PWR_Port_PDO_Storage[PortNum].SourcePDO.ListOfPDO[Profile];
          
          /* Case PDO is a fixed type */
          if (pdo.GenericPDO.PowerObject == USBPD_PDO_TYPE_FIXED)
          {
            /* Get the voltage in MV and set it */
            vbusTargetInmv = (pdo.SRCFixedPDO.VoltageIn50mVunits * 50);
            APPLI_SetVoltage(PortNum, vbusTargetInmv);
#ifdef _DEBUG_TRACE
            USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *) "last part", sizeof("last part"));
            uint8_t tab[32];
            uint8_t size;
            size = sprintf((char*)tab, "vbusTargetInmv: %d", vbusTargetInmv);
            USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, tab, size);       
#endif      
          }
        }
#endif
        
        
        
        return ret;
      }
      
      
      /**
      * @brief  Resets the Power Board
      * @retval USBPD status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_PowerResetGlobal(void)
      {
        int i = 0;
        
        /* Resets all the ports */
        for(i = 0; i < USBPD_PORT_COUNT; i++)
        {
          USBPD_PWR_IF_PowerReset(i);
        }
        return USBPD_OK;
      }
      
      /**
      * @brief  Resets the Power on a specified port
      * @param  PortNum Port number
      * @retval USBPD status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_PowerReset(uint8_t PortNum)
      {
        /* check for valid port */
        if (!USBPD_PORT_IsValid(PortNum))
        {
          return USBPD_ERROR;
        }
        /* Put the usbpd port into ready to start the application */
        return USBPD_PWR_IF_InitPower(PortNum);
      }
      
      
      /**
      * @brief  Checks if the power on a specified port is ready
      * @param  PortNum Port number
      * @param  Vsafe   Vsafe status based on @ref USBPD_VSAFE_StatusTypeDef
      * @retval USBPD status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_SupplyReady(uint8_t PortNum, USBPD_VSAFE_StatusTypeDef Vsafe)
      {
        USBPD_StatusTypeDef status = USBPD_ERROR;
        uint32_t Vbus_board;
        /* check for valid port */
        if (!USBPD_PORT_IsValid(PortNum))
        {
          return status;
        }
        
        
        if (USBPD_VSAFE_0V == Vsafe)
        {
       STUSB1602_VBUS_Range_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Range_Enable);
       STUSB1602_VBUS_Presence_State_Set(STUSB1602_I2C_Add(PortNum), VBUS_Presence_Enable);
          /* Vsafe0V */
          Vbus_board = APPLI_GetVBUS(PortNum);
          status = (Vbus_board < 800 ? USBPD_OK: USBPD_ERROR);
        }
        else
        {
          /* Vsafe5V */
          Vbus_board = APPLI_GetVBUS(PortNum);
          status = (Vbus_board > 4750? USBPD_OK: USBPD_ERROR);
        }
        
        return status;
      }
      
      /**
      * @brief  Enables VBUS power on a specified port
      * @param  PortNum Port number
      * @retval USBPD status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_VBUSEnable(uint8_t PortNum)
      {
        USBPD_StatusTypeDef _status = USBPD_OK;
        
        /* check for valid port */
        if (USBPD_PORT_IsValid(PortNum))
        {
#if defined(_DEBUG_TRACE)
          USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0,(uint8_t *) "EN_VBUS", 7);
#if defined(_VCONN_SUPPORT)
          char str[20];
          sprintf(str, "CC:%d VCONN:%d", DPM_Params[PortNum].VconnCCIs, DPM_Params[PortNum].VconnStatus);
          USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t*)str, strlen(str));
#endif /* _VCONN_SUPPORT */
#endif /* _TRACE */
          /* Set the new state */
          _status = (USBPD_StatusTypeDef)APPLI_SetVoltage(PortNum,5000);
          /* Power Enable is handle feature directly by the device */
        }
        return _status;
      }
      
      /**
      * @brief  Disbale VBUS/VCONN the power on a specified port
      * @param  PortNum Port number
      * @retval USBPD status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_VBUSDisable(uint8_t PortNum)
      {
        USBPD_StatusTypeDef _status = USBPD_OK;
        
        /* check for valid port */
        if (USBPD_PORT_IsValid(PortNum))
        {
#if defined(_DEBUG_TRACE)
          USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 0, (uint8_t *)"DIS VBUS", 8);
#endif /* _TRACE */
          /* Set the new state */
          _status = USBPD_OK;
        }
        return _status;
      }
      
      /**
      * @brief  Disbale the SNK to stop current consumption 
      * @param  PortNum Port number
      * @retval USBPD status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_SNKDisable(uint8_t PortNum)
      {
        USBPD_StatusTypeDef _status = USBPD_ERROR;
        
        /* check for valid port */
        if (USBPD_PORT_IsValid(PortNum))
        {
        }
        return _status;
      }
      
      /**
      * @brief  Initialize power on a specified port
      * @param  PortNum Port number
      * @retval USBPD status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_InitPower(uint8_t PortNum)
      {
        return USBPD_OK;
      }
      
      /**
      * @brief  Checks if the power on a specified port is enabled
      * @param  PortNum Port number
      * @retval USBPD_ENABLE or USBPD_DISABLE
      */
      USBPD_FunctionalState USBPD_PWR_IF_VBUSIsEnabled(uint8_t PortNum)
      {
        /* Get the Status of the port */
        // return USBPD_PORT_IsValid(PortNum) ? (USBPD_FunctionalState)HW_IF_PWR_VBUSIsEnabled(PortNum) : USBPD_DISABLE;
        return USBPD_ENABLE;
      }
      
      /**
      * @brief  Reads the voltage and the current on a specified port
      * @param  PortNum Port number
      * @param  pVoltage: The Voltage in mV
      * @param  pCurrent: The Current in mA
      * @retval USBPD_ERROR or USBPD_OK
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_ReadVA(uint8_t PortNum, uint16_t *pVoltage, uint16_t *pCurrent)
      {
        /* check for valid port */
        if (!USBPD_PORT_IsValid(PortNum))
        {
          return USBPD_ERROR;
        }
        
        /* USBPD_OK if at least one pointer is not null, otherwise USBPD_ERROR */
        USBPD_StatusTypeDef ret = USBPD_ERROR;
        
        /* Get values from HW_IF */
        if (pVoltage != NULL)
        {
          *pVoltage = APPLI_GetVBUS(PortNum);
          ret = USBPD_OK;
        }
        if (pCurrent != NULL)
        {
          *pCurrent = APPLI_GetIvbus(PortNum);
          ret = USBPD_OK;
        }
        
        return ret;
      }
      
#if defined(_VCONN_SUPPORT)
      /**
      * @brief  Enables the VConn on the port.
      * @param  PortNum Port number
      * @param  CC      Specifies the CCx to be selected based on @ref CCxPin_TypeDef structure
      * @retval USBPD status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_Enable_VConn(uint8_t PortNum, CCxPin_TypeDef CC)
      {
        USBPD_StatusTypeDef _status = USBPD_OK;
#if defined(_TRACE)
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 1,(uint8_t *)"VCONN ON", 8);
#endif /* _TRACE */
        
        DPM_Params[PortNum].VconnStatus = USBPD_TRUE;
        _status = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_VCONN_SWAP_TURN_ON_VCONN_REQ); 
        _status = USBPD_HW_IF_CheckVconnPresence(PortNum, 1) ;
        return _status;
      }
      
      /**
      * @brief  Disable the VConn on the port.
      * @param  PortNum Port number
      * @param  CC      Specifies the CCx to be selected based on @ref CCxPin_TypeDef structure
      * @retval USBPD status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_Disable_VConn(uint8_t PortNum, CCxPin_TypeDef CC)
      {
        USBPD_StatusTypeDef _status = USBPD_OK;
#if defined(_TRACE)
        USBPD_TRACE_Add(USBPD_TRACE_DEBUG, PortNum, 1,(uint8_t *)"VCONN OFF", 9);
#endif /* _TRACE */
        DPM_Params[PortNum].VconnStatus = USBPD_FALSE;
        _status = (USBPD_StatusTypeDef)STUSB1602_Type_C_Control_Set(STUSB1602_I2C_Add(PortNum), PD_VCONN_SWAP_TURN_OFF_VCONN_REQ);    
        ( USBPD_HW_IF_CheckVconnPresence(PortNum, 1) ==  USBPD_ERROR ? USBPD_OK : USBPD_ERROR );
        return _status;
      }
#endif /* _VCONN_SUPPORT */
      
      /**
      * @brief  Allow PDO data reading from PWR_IF storage.
      * @param  PortNum Port number
      * @param  DataId Type of data to be read from PWR_IF
      *         This parameter can be one of the following values:
      *           @arg @ref USBPD_CORE_DATATYPE_SRC_PDO Source PDO reading requested
      *           @arg @ref USBPD_CORE_DATATYPE_SNK_PDO Sink PDO reading requested
      * @param  Ptr Pointer on address where PDO values should be written (u8 pointer)
      * @param  Size Pointer on nb of u32 written by PWR_IF (nb of PDOs)
      * @retval None
      */
      void USBPD_PWR_IF_GetPortPDOs(uint8_t PortNum, USBPD_CORE_DataInfoType_TypeDef DataId, uint8_t *Ptr, uint32_t *Size)
      {
        uint32_t   nbpdo, index = 0, nb_valid_pdo = 0;
        uint32_t   *ptpdoarray = NULL;
#if defined(USBPD_REV30_SUPPORT)
        USBPD_PDO_TypeDef pdo_first;
#endif /* USBPD_REV30_SUPPORT */
        
        /* Check if valid port */
        if (USBPD_PORT_IsValid(PortNum))
        {
          /* According to type of PDO to be read, set pointer on values and nb of elements */
          switch (DataId)
          {
          case USBPD_CORE_DATATYPE_SRC_PDO:
            nbpdo       = *PWR_Port_PDO_Storage[PortNum].SourcePDO.NumberOfPDO;
            ptpdoarray  = PWR_Port_PDO_Storage[PortNum].SourcePDO.ListOfPDO;
#if defined(USBPD_REV30_SUPPORT)
            /* Save the 1st PDO */
            pdo_first.d32 = *ptpdoarray;
            /* Reset unchunked bit if current revision is PD2.0*/
            if (USBPD_SPECIFICATION_REV2 == DPM_Params[PortNum].PE_SpecRevision)
            {
              pdo_first.SRCFixedPDO.UnchunkedExtendedMessage  = USBPD_PDO_SRC_FIXED_UNCHUNK_NOT_SUPPORTED;
            }
#endif /* USBPD_REV30_SUPPORT */
            break;
          case USBPD_CORE_DATATYPE_SNK_PDO:
            nbpdo       = *PWR_Port_PDO_Storage[PortNum].SinkPDO.NumberOfPDO;
            ptpdoarray  = PWR_Port_PDO_Storage[PortNum].SinkPDO.ListOfPDO;
#if defined(USBPD_REV30_SUPPORT)
            /* Save the 1st PDO */
            pdo_first.d32 = *ptpdoarray;
            /* Reset FRS bit if current revision is PD2.0*/
            if (USBPD_SPECIFICATION_REV2 == DPM_Params[PortNum].PE_SpecRevision)
            {
              pdo_first.SNKFixedPDO.FastRoleSwapRequiredCurrent = USBPD_PDO_SNK_FIXED_FRS_NOT_SUPPORTED;
            }
#endif /* USBPD_REV30_SUPPORT */
            break;
          default:
            nbpdo = 0;
            break;
          }
          uint16_t cable_amps = USBPD_CORE_CABLE_3A;
          /* Copy PDO data in output buffer */
          for (index = 0; index < nbpdo; index++)
          {
            /* Copy 1st PDO as potentially FRS or UNCHUNKED bits have been reset */
            if (0 == index)
            {
              (void)memcpy(Ptr, (uint8_t*)&pdo_first.d32, 4u);
              nb_valid_pdo++;
            }
            else
            {
              USBPD_PDO_TypeDef localpdo = {0};
              /* Copy the next PDO */
              localpdo.d32 = *ptpdoarray;
#if  defined(_VCONN_SUPPORT)
                if ((0 == DPM_Ports[PortNum].VDM_DiscoCableIdentify.CableVDO_Presence)
                    || (VBUS_3A == DPM_Ports[PortNum].VDM_DiscoCableIdentify.CableVDO.b.VBUS_CurrentHandCap))
                {
                  cable_amps = USBPD_CORE_CABLE_3A;
                }
                else
                {
                  if (USBPD_PORTPOWERROLE_SRC ==DPM_Params[PortNum].PE_PowerRole)
                  {
                    if (VBUS_5A == DPM_Ports[PortNum].VDM_DiscoCableIdentify.CableVDO.b.VBUS_CurrentHandCap)
                    {
                      cable_amps = USBPD_CORE_CABLE_5A;
                    }
                  }
                }
#endif /* (SRC) || DRP) && VCONN_SUPPORT */
#if defined(USBPD_REV30_SUPPORT)
                {
                  localpdo.SRCFixedPDO.MaxCurrentIn10mAunits = USBPD_MIN(localpdo.SRCFixedPDO.MaxCurrentIn10mAunits , cable_amps / 10);
                }
#endif /*USBPD_REV30_SUPPORT */
                (void)memcpy((Ptr + (nb_valid_pdo * 4u)), (uint8_t*)&localpdo.d32, 4u);
                nb_valid_pdo++;
              }
            ptpdoarray++;
          }
        }
        /* Set nb of read PDO (nb of u32 elements); */
        *Size = nb_valid_pdo;
      }
      
      /**
      * @brief  Find out SRC PDO pointed out by a position provided in a Request DO (from Sink).
      * @param  PortNum Port number
      * @param  RdoPosition RDO Position in list of provided PDO
      * @param  Pdo Pointer on PDO value pointed out by RDO position (u32 pointer)
      * @retval Status of search
      *         USBPD_OK : Src PDO found for requested DO position (output Pdo parameter is set)
      *         USBPD_FAIL : Position is not compliant with current Src PDO for this port (no corresponding PDO value)
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_SearchRequestedPDO(uint8_t PortNum, uint32_t RdoPosition, uint32_t *Pdo)
      {
        if((RdoPosition == 0) || (RdoPosition > *PWR_Port_PDO_Storage[PortNum].SourcePDO.NumberOfPDO))
        {
          /* Invalid PDO index */
          return USBPD_FAIL;
        }
        
        *Pdo = PWR_Port_PDO_Storage[PortNum].SourcePDO.ListOfPDO[RdoPosition - 1];
        return USBPD_OK;
      }
      
      /**
      * @brief  Function to check validity between SNK PDO and power user settings
      * @param  PortNum Port number
      * @retval USBPD Status
      */
      USBPD_StatusTypeDef USBPD_PWR_IF_CheckUpdateSNKPower(uint8_t PortNum)
      {
        USBPD_StatusTypeDef _status = USBPD_OK;
        USBPD_PDO_TypeDef pdo;
        uint32_t _max_power = 0;
        uint16_t _voltage = 0, _current = 0, _power = 0;
        uint16_t _min_voltage = 0xFFFF, _max_voltage = 0, _max_current = 0;
        
        for (uint32_t _index = 0; _index < *PWR_Port_PDO_Storage[PortNum].SinkPDO.NumberOfPDO; _index++)
        {
          pdo.d32 = PWR_Port_PDO_Storage[PortNum].SinkPDO.ListOfPDO[_index];
          switch (pdo.GenericPDO.PowerObject)
          {
          case USBPD_CORE_PDO_TYPE_FIXED:    /*!< Fixed Supply PDO                             */
            _voltage = PWR_DECODE_50MV(pdo.SNKFixedPDO.VoltageIn50mVunits);
            _PWR_UPDATE_VOLTAGE_MIN(_voltage, _min_voltage);
            _PWR_UPDATE_VOLTAGE_MAX(_voltage, _max_voltage);
            _current = PWR_DECODE_10MA(pdo.SNKFixedPDO.OperationalCurrentIn10mAunits);
            _PWR_UPDATE_CURRENT_MAX(_current, _max_current);
            break;
          case USBPD_CORE_PDO_TYPE_BATTERY:  /*!< Battery Supply PDO                           */
            _voltage = PWR_DECODE_50MV(pdo.SNKBatteryPDO.MinVoltageIn50mVunits);
            _PWR_UPDATE_VOLTAGE_MIN(_voltage, _min_voltage);
            _voltage = PWR_DECODE_50MV(pdo.SNKBatteryPDO.MaxVoltageIn50mVunits);
            _PWR_UPDATE_VOLTAGE_MAX(_voltage, _max_voltage);
            _power = PWR_DECODE_MW(pdo.SNKBatteryPDO.OperationalPowerIn250mWunits);
            _PWR_UPDATE_POWER_MAX(_power, _max_power);
            break;
          case USBPD_CORE_PDO_TYPE_VARIABLE: /*!< Variable Supply (non-battery) PDO            */
            _voltage = PWR_DECODE_50MV(pdo.SNKVariablePDO.MinVoltageIn50mVunits);
            _PWR_UPDATE_VOLTAGE_MIN(_voltage, _min_voltage);
            _voltage = PWR_DECODE_50MV(pdo.SNKVariablePDO.MaxVoltageIn50mVunits);
            _PWR_UPDATE_VOLTAGE_MAX(_voltage, _max_voltage);
            _current = PWR_DECODE_10MA(pdo.SNKVariablePDO.OperationalCurrentIn10mAunits);
            _PWR_UPDATE_CURRENT_MAX(_current, _max_current);
            break;
          default:
            break;
          }
        }
        
        _PWR_CHECK_VOLTAGE_MIN(_min_voltage, DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MinOperatingVoltageInmVunits);
        _PWR_CHECK_VOLTAGE_MAX(_max_voltage, DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MaxOperatingVoltageInmVunits);
        _PWR_CHECK_CURRENT_MAX(_max_current, DPM_USER_Settings[PortNum].DPM_SNKRequestedPower.MaxOperatingCurrentInmAunits);
        return _status;
      }
      
      /**
      * @}
      */
      
      /** @addtogroup STM32_USBPD_APPLICATION_POWER_IF_Private_Functions
      * @{
      */
      
      /**
      * @brief  Create SRC Fixed PDO object
      * @param  _C_: Current in A
      * @param  _V_: voltage in V
      * @param  _PK_: The peak of current
      * @param  DRDSupport: Data Role Swap support indication
      * @param  UsbCommCapable: USB communications capability indication
      * @param  ExtPower: Port externally powered indication
      * @param  UsbSuspendSupport: USB Suspend support indication
      * @param  DRPSupport: Dual Role Power support indication
      * @retval PDO object value (expressed on u32)
      */
      uint32_t _PWR_SRCFixedPDO(float  _C_, float _V_,
                                USBPD_CORE_PDO_PeakCurr_TypeDef _PK_,
                                USBPD_CORE_PDO_DRDataSupport_TypeDef DRDSupport,
                                USBPD_CORE_PDO_USBCommCapable_TypeDef UsbCommCapable,
                                USBPD_CORE_PDO_ExtPowered_TypeDef ExtPower,
                                USBPD_CORE_PDO_USBSuspendSupport_TypeDef UsbSuspendSupport,
                                USBPD_CORE_PDO_DRPowerSupport_TypeDef DRPSupport)
      {
        USBPD_PDO_TypeDef pdo;
        
        pdo.d32 = 0;
        pdo.SRCFixedPDO.MaxCurrentIn10mAunits       = PWR_A_10MA(_C_);
        pdo.SRCFixedPDO.VoltageIn50mVunits          = PWR_V_50MV(_V_);
        pdo.SRCFixedPDO.PeakCurrent                 = _PK_;
        pdo.SRCFixedPDO.DataRoleSwap                = DRDSupport;
        pdo.SRCFixedPDO.USBCommunicationsCapable    = UsbCommCapable;
        pdo.SRCFixedPDO.ExternallyPowered           = ExtPower;
        pdo.SRCFixedPDO.USBSuspendSupported         = UsbSuspendSupport;
        pdo.SRCFixedPDO.DualRolePower               = DRPSupport;
        pdo.SRCFixedPDO.FixedSupply                 = USBPD_CORE_PDO_TYPE_FIXED;
        return pdo.d32;
      }
      
      /**
      * @brief  Create SRC Variable PDO object
      * @param  _MAXV_ Max voltage in V
      * @param  _MINV_ Min voltage in V
      * @param  _C_: Max current in A
      * @retval PDO object value (expressed on u32)
      */
      uint32_t _PWR_SRCVariablePDO(float _MAXV_, float _MINV_, float _C_)
      {
        USBPD_PDO_TypeDef pdo;
        
        pdo.d32 = 0;
        pdo.SRCVariablePDO.MaxCurrentIn10mAunits = PWR_A_10MA(_C_);
        pdo.SRCVariablePDO.MaxVoltageIn50mVunits = PWR_V_50MV(_MAXV_);
        pdo.SRCVariablePDO.MinVoltageIn50mVunits = PWR_V_50MV(_MINV_);
        pdo.SRCVariablePDO.VariableSupply        = USBPD_CORE_PDO_TYPE_VARIABLE;
        return pdo.d32;
      }
      
      /**
      * @brief  Create SRC Battery PDO object
      * @param  _MAXV_ Max voltage in V
      * @param  _MINV_ Min voltage in V
      * @param  _PWR_ Max allowable power in W
      * @retval PDO object value (expressed on u32)
      */
      uint32_t _PWR_SRCBatteryPDO(float _MAXV_,float _MINV_,float _PWR_)
      {
        USBPD_PDO_TypeDef pdo;
        
        pdo.d32 = 0;
        pdo.SRCBatteryPDO.MaxAllowablePowerIn250mWunits = PWR_W(_PWR_);
        pdo.SRCBatteryPDO.MinVoltageIn50mVunits         = PWR_V_50MV(_MINV_);
        pdo.SRCBatteryPDO.MaxVoltageIn50mVunits         = PWR_V_50MV(_MAXV_);
        pdo.SRCBatteryPDO.Battery                       = USBPD_CORE_PDO_TYPE_BATTERY;
        return pdo.d32;
      }
      
      /**
      * @brief  Create SNK Fixed PDO object
      * @param  _C_ Current in A
      * @param  _V_ voltage in V
      * @param  DRDSupport: Data Role Swap support indication
      * @param  UsbCommCapable: USB communications capability indication
      * @param  ExtPower: Port externally powered indication
      * @param  HigherCapab: Sink needs more than vSafe5V to provide full functionality indication
      * @param  DRPSupport: Dual Role Power support indication
      * @retval PDO object value (expressed on u32)
      */
      uint32_t _PWR_SNKFixedPDO(float  _C_, float _V_,
                                USBPD_CORE_PDO_DRDataSupport_TypeDef DRDSupport,
                                USBPD_CORE_PDO_USBCommCapable_TypeDef UsbCommCapable,
                                USBPD_CORE_PDO_ExtPowered_TypeDef ExtPower,
                                USBPD_CORE_PDO_HigherCapability_TypeDef HigherCapab,
                                USBPD_CORE_PDO_DRPowerSupport_TypeDef DRPSupport)
      {
        USBPD_PDO_TypeDef pdo;
        
        pdo.d32 = 0;
        pdo.SNKFixedPDO.OperationalCurrentIn10mAunits = PWR_A_10MA(_C_);
        pdo.SNKFixedPDO.VoltageIn50mVunits            = PWR_V_50MV(_V_);
        pdo.SNKFixedPDO.DataRoleSwap                  = DRDSupport;
        pdo.SNKFixedPDO.USBCommunicationsCapable      = UsbCommCapable;
        pdo.SNKFixedPDO.ExternallyPowered             = ExtPower;
        pdo.SNKFixedPDO.HigherCapability              = HigherCapab;
        pdo.SNKFixedPDO.DualRolePower                 = DRPSupport;
        pdo.SNKFixedPDO.FixedSupply                   = USBPD_CORE_PDO_TYPE_FIXED;
        
        return pdo.d32;
      }
      
      /**
      * @brief  Create SNK Variable PDO object
      * @param  _MAXV_ Max voltage in V
      * @param  _MINV_ Min voltage in V
      * @param  _C_: Max current in A
      * @retval PDO object value (expressed on u32)
      */
      uint32_t _PWR_SNKVariablePDO(float  _MAXV_,float _MINV_,float _C_)
      {
        USBPD_PDO_TypeDef pdo;
        
        pdo.d32 = 0;
        pdo.SRCVariablePDO.MaxCurrentIn10mAunits = PWR_A_10MA(_C_);
        pdo.SRCVariablePDO.MaxVoltageIn50mVunits = PWR_V_50MV(_MAXV_);
        pdo.SRCVariablePDO.MinVoltageIn50mVunits = PWR_V_50MV(_MINV_);
        pdo.SRCVariablePDO.VariableSupply        = USBPD_CORE_PDO_TYPE_VARIABLE;
        return pdo.d32;
      }
      
      /**
      * @brief  Create SNK Battery PDO object
      * @param  _MAXV_ Max voltage in V
      * @param  _MINV_ Min voltage in V
      * @param  _PWR_ Max allowable power in W
      * @retval PDO object value (expressed on u32)
      */
      uint32_t _PWR_SNKBatteryPDO(float _MAXV_,float _MINV_,float _PWR_)
      {
        USBPD_PDO_TypeDef pdo;
        
        pdo.d32 = 0;
        pdo.SRCBatteryPDO.MaxAllowablePowerIn250mWunits = PWR_W(_PWR_);
        pdo.SRCBatteryPDO.MinVoltageIn50mVunits         = PWR_V_50MV(_MINV_);
        pdo.SRCBatteryPDO.MaxVoltageIn50mVunits         = PWR_V_50MV(_MAXV_);
        pdo.SRCBatteryPDO.Battery                       = USBPD_CORE_PDO_TYPE_BATTERY;
        return pdo.d32;
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
      
      /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
      