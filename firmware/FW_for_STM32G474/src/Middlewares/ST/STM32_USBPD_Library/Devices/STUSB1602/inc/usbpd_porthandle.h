/**
  ******************************************************************************
  * @file    usbpd_porthandle.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of usbpd_porthandle.h.
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
/*#warning "Porthandle header sof"*/
#ifndef __USBPD_PORTHANDLE_H_
#define __USBPD_PORTHANDLE_H_
/*#warning "Porthandle header sof 2"*/
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "User_BSP.h"  
#include "STUSB1602_Registers_if.h"
#include "STUSB1602_Peripherals_if.h"
  
  
  
/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_HW_IF
  * @{
  */

/* Exported typedef ----------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_PORTHANDLE_Exported_Types USBPD DEVICE PORTHANDLE Exported Types
  * @{
  */
#define   TX_PREAMBLE             0xAA    /*!< Preamble byte 0xAA = 10101010b */
#define   TX_PREAMBLE_SIZE        8u       /*!< Amount of repeated preamble bytes */
#define   SOP_AND_HEADER_SIZE     5u       /*!< Amount addition of Header and SOP in bytes */  
#define   CRC_ENCODED             5u       /* CRC in 5b code in byte */  
  
#define __RX_DATA_LEN (USBPD_MAX_RX_BUFFER_SIZE)      /* header + 28 bytes */ /* this length is defined in usbpd_def.h*/  

#define MAX_DATA_LEN (uint16_t)((((uint16_t ) __RX_DATA_LEN * 10 ) /8 )+ SOP_AND_HEADER_SIZE + CRC_ENCODED + 1)
#define   TXRX_BUFFER_SIZE     (MAX_DATA_LEN + TX_PREAMBLE_SIZE )  /*!< Amount of bytes constituting the packet 349 bytes for unchunck 54 for chunck*/ 
/**
  * @brief Default current limit of the power switches supplying VCONN on the CC pins
  */
typedef enum                       
{
  VConn_Ilim_350mA = 0x00,  /*!< VCONN current limit set to 350 mA */
  VConn_Ilim_300mA = 0x01,  /*!< VCONN current limit set to 300 mA */
  VConn_Ilim_250mA = 0x02,  /*!< VCONN current limit set to 250 mA */
  VConn_Ilim_200mA = 0x03,  /*!< VCONN current limit set to 200 mA */
  VConn_Ilim_150mA = 0x04,  /*!< VCONN current limit set to 150 mA */
  VConn_Ilim_100mA = 0x05,  /*!< VCONN current limit set to 100 mA */
  VConn_Ilim_400mA = 0x06,  /*!< VCONN current limit set to 400 mA */
  VConn_Ilim_450mA = 0x07,  /*!< VCONN current limit set to 450 mA */
  VConn_Ilim_500mA = 0x08,  /*!< VCONN current limit set to 500 mA */
  VConn_Ilim_550mA = 0x09,  /*!< VCONN current limit set to 550 mA */
  VConn_Ilim_600mA = 0x0A,  /*!< VCONN current limit set to 600 mA */
} USBPD_VConnIlim_TypeDef;



/**
  * @brief Struct with parameter used in the decoding phase
  */
typedef struct
{
  uint8_t     exed_flag;              /*!< Exit cases flag              */
  uint8_t     preamble;               /*!< Flag set if preamble ended   */
  uint32_t    dataindex;              /*!< Index inside RxDataBuf       */
  uint32_t    dataoffset;             /*!< Offset inside RxDataBuf      */
  uint32_t    index;                  /*!< Word index inside RXBuf      */
  uint32_t    offset;                 /*!< Bit index inside RXBuf       */
#ifdef __STAT
  uint8_t preamble_offset;                        /*!< Offset identifies the byte of data after preamble */
  uint8_t preamble_index;                         /*!< Index identifies the bit of data after preamble */
  uint32_t preamble_counter;                   /*!< Preamble counter */
#endif  
} UnwrapData_TypeDef;

/**
  * @brief Status of decoding phase
  */
typedef enum
{
  USBPD_PHY_RX_STATUS_NONE=0,                   /*!< Unknown Status                   */
  USBPD_PHY_RX_STATUS_OK=1,                     /*!< Status OK                        */
  USBPD_PHY_RX_STATUS_SOP_DETECTING=2,          /*!< Detecting SOP                    */
  USBPD_PHY_RX_STATUS_DATA=3,                   /*!< Decoding Data                    */
  USBPD_PHY_RX_STATUS_MESSAGE_READY=4,          /*!< Message Ready                    */
  USBPD_PHY_RX_STATUS_ERROR=5,                  /*!< Generic error during decoding    */
  USBPD_PHY_RX_STATUS_ERROR_UNSUPPORTED_SOP=6,  /*!< Error; Unsupported SOP detected  */
  USBPD_PHY_RX_STATUS_ERROR_INVALID_SOP=7,      /*!< Error: Invalid SOP detected      */
  USBPD_PHY_RX_STATUS_ERROR_INVALID_SYMBOL=8,   /*!< Error: Invalid symbol found      */
  USBPD_PHY_RX_STATUS_ERROR_EOP_NOT_FOUND=9,    /*!< Error: No EOP found              */
  USBPD_PHY_RX_STATUS_ERROR_CRC_FAILED=10,       /*!< Error: CRC failed                */
}
USBPD_PHY_RX_Status_TypeDef;

/**
  * @brief RX PHY Status
  */
typedef enum
{
  RX_OK,                    /*!< RX OK                        */
  RX_ERROR,                 /*!< RX ERROR                     */
  RX_ERROR_WRONG_ORDERSET,  /*!< RX ERROR: Wrong Ordered Set  */
  RX_ERROR_WRONG_SYMBOL,    /*!< RX ERROR: Wrong Symbol       */
  RX_ERROR_MISSING_EOP,     /*!< RX ERROR: Wrong EOP          */
  RX_ERROR_CRC_FAILED       /*!< RX ERROR: CRC Failed         */
} USBPD_PHY_RX_StatusTypeDef;

/**
  * @brief CallBacks exposed by the HW_IF to the PHY
  */
typedef struct
{
  /**
    * @brief  A message has been received on a specified port.
    * @param  PortNum   The current port number
    * @param  pPayload  pointer to the received data buffer in 5bit representation
    * @param  Bitsize   number of bit received
    * @retval RX PHY Status
    */
  USBPD_PHY_RX_StatusTypeDef (*USBPD_HW_IF_ReceiveMessage)(uint8_t PortNum, uint32_t *pPayload, uint16_t Bitsize);
  /**
    * @brief  The message transfer has been completed
    * @param  PortNum The current port number
    * @retval None
    */
  void (*USBPD_HW_IF_TxCompleted)(uint8_t PortNum, uint32_t Status);
  /**
    * @brief  Bist data sent callback from PHY_HW_IF
    * @param  PortNum   Index of current used port
    * @param  BistMode  Bist mode
    *         The supported mode are:
    *           @arg @ref USBPD_BIST_CARRIER_MODE0
    *           @arg @ref USBPD_BIST_CARRIER_MODE1
    *           @arg @ref USBPD_BIST_CARRIER_MODE2
    *           @arg @ref USBPD_BIST_CARRIER_MODE3
    *           @arg @ref USBPD_BIST_EYE_PATTERN
    * @retval None
    */
  void (*USBPD_HW_IF_BistCompleted)(uint8_t PortNum, USBPD_BISTMsg_TypeDef BistMode);
  /**
    * @brief  A new message is incoming, need to reset the status.
    * @param  PortNum The current port number
    * @retval The status of the decoding process
    */
  USBPD_PHY_RX_Status_TypeDef (*USBPD_HW_IF_RX_Reset)(uint8_t PortNum);
  /**
    * @brief  Some data are avaiable for current message, performe a decoding step.
    * @param  PortNum The current port number
    * @param  data    The last data received
    * @retval The status of the decoding process
    */
  USBPD_PHY_RX_Status_TypeDef (*USBPD_HW_IF_RX_Accumulate)(uint8_t PortNum, uint32_t data);
  /**
    * @brief  The reception phase of the current message is completed, now to complete the decoding, check the message(CRC) and notify it.
    * @param  PortNum The current port number
    * @retval The status of the decoding process
    */
  USBPD_PHY_RX_Status_TypeDef (*USBPD_HW_IF_RX_Completed)(uint8_t PortNum);
} USBPD_HW_IF_Callbacks;

/**
  * @brief USBPD Port State
  */
typedef enum
{
  HAL_USBPD_PORT_STATE_RESET             = 0x00,    /*!< Peripheral is not initialized                      */
  HAL_USBPD_PORT_STATE_READY             = 0x01,    /*!< Peripheral Initialized and ready for use           */
  HAL_USBPD_PORT_STATE_BUSY              = 0x02,    /*!< An internal process is ongoing                     */
  HAL_USBPD_PORT_STATE_BUSY_TX           = 0x03,    /*!< Data Transmission process is ongoing               */
  HAL_USBPD_PORT_STATE_BUSY_RX           = 0x04,    /*!< Data Reception process is ongoing                  */
  HAL_USBPD_PORT_STATE_WAITING           = 0x05,    /*!< Waiting for Data Reception process                 */
  HAL_USBPD_PORT_STATE_TIMEOUT           = 0x06,    /*!< Timeout state                                      */
  HAL_USBPD_PORT_STATE_ERROR             = 0x07,    /*!< Error                                              */
  HAL_USBPD_PORT_STATE_BIST              = 0x08     /*!< BIST Transmission process is ongoing               */
}HAL_USBPD_PORT_StateTypeDef;


/**
  * @brief  USBPD Port Handle
  */
typedef struct
{
  uint8_t                     Instance;        /*!< USBPD_PORT number                              */
  uint8_t                     *pTxRxBuffPtr;     /*!< Pointer to Tx Buffer                           */
  CCxPin_TypeDef              CCx;             /*!< CC pin used for communication                  */
  FlagStatus                  CCxChange;       /*!< CC event change                                */
  HAL_LockTypeDef             Lock;            /*!< Locking object                                 */
  HAL_USBPD_PORT_StateTypeDef State;           /*!< Communication state                            */
  __IO uint32_t               ErrorCode;       /*!< Error code                                     */
  USBPD_PortPowerRole_TypeDef role;            /*!< The Role of the port Provider or Consumer      */
  uint32_t                    BIST_index;      /*!< Index for monitoring BIST Msg bits             */
  FunctionalState             VConn;           /*!< VConn status flag                              */
  USBPD_PortDataRole_TypeDef  DataRole;        /*!< Data role of the port                          */
  uint8_t                     TxSpareBits;     /*!< TxSpareBits                                    */
  UnwrapData_TypeDef          unwrapdata;      /*!< Fields used for decoding                       */
  SPI_HandleTypeDef           hspi;            /*!< SPI Handle parameters                          */
  DMA_HandleTypeDef           hdmatx;          /*!< Tx DMA Handle parameters                       */
#ifdef RX_DMACH
  DMA_HandleTypeDef           hdmarx;          /*!< Rx DMA Handle parameters                       */
#endif  
  I2C_HandleTypeDef  *         hi2c;            /*!< I2C Handle pointer                            */
  USBPD_HW_IF_Callbacks       cbs;             /*!< Port callbacks, see @ref USBPD_HW_IF_Callbacks */
  uint8_t                     AlertEventCount; /*!< Alert event counter                            */
  uint8_t                     Error_Recovery_Flag; /*!< Error recovery on going flag               */
  uint8_t                     NbDetach;        /*!< Number of CC detach                            */
  uint8_t(*IsSwapOngoing)(uint8_t);            /*!< Function to check if a swap is ongoing used by @ref HW_IF_check_bus_idle */
  void (*USBPD_CAD_WakeUp)(void);               /*!< function used to wakeup cad task   */  
  STUSB1602_MONITORING_STATUS_RegTypeDef Monitoring_Status ; /*!< Monitoring Parameters on Vbus and Vconn                    */ 
  STUSB1602_MONITORING_STATUS_TRANS_RegTypeDef Monitoring_Trans ; /*!< Monitoring Parameters on Vbus and Vconn                    */   
  STUSB1602_HW_FAULT_STATUS_RegTypeDef Hw_Fault; /*!< Hardware fault on analog system              */
  DEVICE_CUT_TypeDef          Device_cut;      /*!< Device Cut identifier                          */
  uint8_t                     CableCapa5A;
}STUSB16xx_PORT_HandleTypeDef;

/**
  * @brief  USBPD Initialization Structure
  */
typedef struct
{
  uint8_t                                     Instance;                 /*!< PORT number */
  USBPD_PortPowerRole_TypeDef                 RolePower;                /*!< The Port power role @ref USBPD_PortPowerRole_TypeDef */
  USBPD_PortDataRole_TypeDef                  RoleData;                 /*!< Port default data role @ref USBPD_PortDataRole_TypeDef */
  FunctionalState                             VendorMessages;           /*!< If enabled, It allows sending vendor messages */
  FunctionalState                             Ping;                     /*!< If enabled it allows sending Ping message when an Explicit Contract is established */
  FunctionalState                             ExtendedMessages;         /*!< If enabled it supports extended messages */
  uint16_t                                    PE_SCAP_HR;               /*!< Number of source capabilities requests before hard reset */
  Current_Capability_Advertised_TypeDef       CCCurrentAdvertised;      /*!< It advertises the current capability @ref Current_Capability_Advertised_TypeDef*/
  FunctionalState                             DataRoleSwap;             /*!< It enables or disables the data role swap capability */
  FunctionalState                             PowerRoleSwap;            /*!< It enables or disables the power role swap capability */
  FunctionalState                             VConnSwap;                /*!< It enables or disables the VCONN swap capability */
  FunctionalState                             VConnSupply;              /*!< It enables or disables the VCONN supply capability on CC pin */
  FunctionalState                             VConnDischarge;           /*!< It enables or disables the VCONN discharge on CC pin */
  FunctionalState                             VBusDischarge;            /*!< It enables or disables the Vbus discharge */
  USBPD_VConnIlim_TypeDef                     VConnIlim;                /*!< It allows changing the default current limit supplying VCONN on the CC pins */
  FunctionalState                             VConnMonitoring;          /*!< It enables or disables UVLO threshold detection on VCONN pin */
  VCONN_UVLO_Threshold_TypeDef                VConnThresholdUVLO;       /*!< High UVLO threshold of 4.65 V; 1b: Low UVLO threshold of 2.65 V (case of VCONN-powered accessories operating down to 2.7 V) @ref VCONN_UVLO_Threshold_TypeDef*/
  uint16_t                                    VBusSelect;               /*!< Value (mV) related to targeted VBUS voltage used for VBUS range monitoring by 100mV step */
  uint8_t                                     VbusVShiftHigh;           /*!< Shift coefficient used for computing the high threshold value (5% + VBUS_VSHIFT_HIGH) of the monitoring voltage range */
  uint8_t                                     VbusVShiftLow;            /*!< Shift coefficient used for computing the low threshold value (-5% - VBUS_VSHIFT_LOW) of the monitoring voltage range */
  FunctionalState                             VBusRange;                /*!< It enables or disables VBUS voltage range detection */
  VBUS_VSAFE0V_Threshold_TypeDef              VBusThresholdVSafe0V;     /*!< VBus vSafe0V threshold @ref VBUS_VSAFE0V_Threshold_TypeDef*/
  FunctionalState                             VddOVLO;                  /*!< It enables or disables OVLO threshold detection on Vdd voltage */
  FunctionalState                             VddUVLO;                  /*!< It enables or disables UVLO threshold detection on Vdd voltage */
  uint8_t                                     VBusDischargeTimeTo0V;    /*!< Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to 0 V: 84 ms*TDISPARAM (840ms is default discharge time) */
  uint8_t                                     VBusDischargeTimeToPDO;   /*!< Binary coded TDISPARAM coefficient used to compute the VBUS discharge time to PDO: 20 ms*TDISPARAM (200 ms is default discharge time) */
  FunctionalState                             PowerAccessoryDetection;  /*!< It enables or disables powered accessory detection */
  FunctionalState                             PowerAccessoryTransition; /*!< It enables or disables powered accessory transition from Powered.Accessory state to Try.SNK */
}USBPD_Init_TypeDef;

/**
  * @}
  */

/* Exported define -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_PORTHANDLE_Exported_Macros USBPD DEVICE PORTHANDLE Exported Macros
  *   @note @param __PORT__ identifies the Port number
  * @{
  */



/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

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
/*#warning "Porthandle header eof 2"*/
#endif /* __USBPD_PORTHANDLE_H_ */
/*#warning "Porthandle header eof"*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

