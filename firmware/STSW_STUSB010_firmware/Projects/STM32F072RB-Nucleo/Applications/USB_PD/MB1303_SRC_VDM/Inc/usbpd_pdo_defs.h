/**
  ******************************************************************************
  * @file    usbpd_pdo_defs.h
  * @author  MCD Application Team
  * @brief   Header file for definition of PDO/APDO values for 1 ports(SRC) configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#ifndef __USBPD_PDO_DEF_H_
#define __USBPD_PDO_DEF_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/* Define   ------------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/**
  * @brief  USBPD Port PDO Structure definition
  *
  */
typedef struct
{

  uint32_t *ListOfPDO;                          /*!< Pointer on Power Data Objects list, defining
                                                     port capabilities */
  uint8_t  NumberOfPDO;                         /*!< Number of Power Data Objects defined in ListOfPDO
                                                     This parameter must be set at max to @ref USBPD_MAX_NB_PDO value */
} USBPD_PortPDO_TypeDef;

/**
  * @brief  USBPD Port PDO Storage Structure definition
  */
typedef struct
{
  USBPD_PortPDO_TypeDef    SourcePDO;        /*!< SRC Power Data Objects */
}USBPD_PWR_Port_PDO_Storage_TypeDef;

/* Exported define -----------------------------------------------------------*/
#define USBPD_PDP_SRC_IN_WATTS          15  /* SRC PD Power in Watts */


/* Max current */
#define USBPD_CORE_CABLE_3A 3000
#define USBPD_CORE_CABLE_5A 5000
#define USBPD_CORE_PDO_SRC_FIXED_MAX_CURRENT_5A 5000

#define USBPD_CORE_PDO_SRC_FIXED_2000   2000

/* 3000mA in source mode */
#define USBPD_CORE_PDO_SRC_FIXED_MAX_CURRENT   3000

/* Definitions of nb of PDO and APDO for each port */
#define PORT0_NB_SOURCEPDO         1   /* Number of Source PDOs (applicable for port 0)   */
#define PORT0_NB_SINKPDO           0   /* Number of Sink PDOs (applicable for port 0)     */
#if  (USBPD_PORT_COUNT == 2)
#define PORT1_NB_SOURCEPDO         1   /* Number of Source PDOs (applicable for port 1)   */
#define PORT1_NB_SINKPDO           0   /* Number of Sink PDOs (applicable for port 1)     */
#else
#define PORT1_NB_SOURCEPDO         0   /* Number of Source PDOs (applicable for port 1)   */
#define PORT1_NB_SINKPDO           0   /* Number of Sink PDOs (applicable for port 1)     */
#endif /*USBPD_PORT_COUNT == 2*/


#define PORT0_NB_SOURCEAPDO        0   /* Number of Source APDOs (applicable for port 0)  */
#define PORT0_NB_SINKAPDO          0   /* Number of Sink APDOs (applicable for port 0)    */
#define PORT1_NB_SOURCEAPDO        0   /* Number of Source APDOs (applicable for port 1)  */
#define PORT1_NB_SINKAPDO          0   /* Number of Sink APDOs (applicable for port 1)    */

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
#ifndef __USBPD_PWR_IF_C

extern uint8_t USBPD_NbPDO[4];

extern uint32_t PORT0_PDO_ListSRC[USBPD_MAX_NB_PDO];
#if  (USBPD_PORT_COUNT == 2)
extern uint32_t PORT1_PDO_ListSRC[USBPD_MAX_NB_PDO];
#endif /*USBPD_PORT_COUNT == 2*/
#else
uint8_t USBPD_NbPDO[4] = {(PORT0_NB_SINKPDO + PORT0_NB_SINKAPDO), 
                          ((PORT0_NB_SOURCEPDO + PORT0_NB_SOURCEAPDO)),
                          ((PORT1_NB_SINKPDO + PORT1_NB_SINKAPDO)),
                          ((PORT1_NB_SOURCEPDO + PORT1_NB_SOURCEAPDO))};
/* Definition of Source PDO for Port 0 */
uint32_t PORT0_PDO_ListSRC[USBPD_MAX_NB_PDO] =
{
  /* PDO 1 */  
        ( ((PWR_A_10MA(USBPD_CORE_PDO_SRC_FIXED_MAX_CURRENT/1000.0)) << USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos) |
          ((PWR_V_50MV(5)) << USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)                                        |
          USBPD_PDO_SRC_FIXED_PEAKCURRENT_EQUAL                                                       |
#if defined(USBPD_REV30_SUPPORT)
          USBPD_PDO_SRC_FIXED_UNCHUNK_SUPPORTED                                                      |
#endif /*USBPD_REV30_SUPPORT*/
          USBPD_PDO_SRC_FIXED_DRD_NOT_SUPPORTED                                                       |
          USBPD_PDO_SRC_FIXED_USBCOMM_NOT_SUPPORTED                                                   |
          USBPD_PDO_SRC_FIXED_EXT_POWER_AVAILABLE                                                     |
          USBPD_PDO_SRC_FIXED_USBSUSPEND_NOT_SUPPORTED                                                |
          USBPD_PDO_SRC_FIXED_DRP_NOT_SUPPORTED                                                       |
          USBPD_PDO_TYPE_FIXED
        ),
  /* PDO 2 */ (0x00000000U),

  /* PDO 3 */ (0x00000000U),
  /* PDO 4 */ (0x00000000U),
  /* PDO 5 */ (0x00000000U),
  /* PDO 6 */ (0x00000000U),
  /* PDO 7 */ (0x00000000U)
};


#if  (USBPD_PORT_COUNT == 2)
/* Definition of Source PDO for Port 1 */
uint32_t PORT1_PDO_ListSRC[USBPD_MAX_NB_PDO] = 
{
  /* PDO 1 */
        ( ((PWR_A_10MA(USBPD_CORE_PDO_SRC_FIXED_MAX_CURRENT/1000.0)) << USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos) |
          ((PWR_V_50MV(5)) << USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)                                        |
          USBPD_PDO_SRC_FIXED_PEAKCURRENT_EQUAL                                                       |
#if defined(USBPD_REV30_SUPPORT)
          USBPD_PDO_SRC_FIXED_UNCHUNK_SUPPORTED                                                       |
#endif /*USBPD_REV30_SUPPORT*/
          USBPD_PDO_SRC_FIXED_DRD_NOT_SUPPORTED                                                       |
          USBPD_PDO_SRC_FIXED_USBCOMM_NOT_SUPPORTED                                                   |
          USBPD_PDO_SRC_FIXED_EXT_POWER_AVAILABLE                                                     |
          USBPD_PDO_SRC_FIXED_USBSUSPEND_NOT_SUPPORTED                                                |
          USBPD_PDO_SRC_FIXED_DRP_NOT_SUPPORTED                                                       |
          USBPD_PDO_TYPE_FIXED
        ),
  /* PDO 2 */ (0x00000000U),
  /* PDO 3 */ (0x00000000U),
  /* PDO 4 */ (0x00000000U),
  /* PDO 5 */ (0x00000000U),
  /* PDO 6 */ (0x00000000U),
  /* PDO 7 */ (0x00000000U)
};

/* Definition of Sink PDO for Port 1 */
#endif /*USBPD_PORT_COUNT == 2*/

#endif /* __USBPD_PWR_IF_C */

/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_PDO_DEF_DRP_1PORT_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
