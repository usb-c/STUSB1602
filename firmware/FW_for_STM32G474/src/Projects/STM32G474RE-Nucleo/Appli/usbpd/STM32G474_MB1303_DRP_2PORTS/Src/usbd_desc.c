/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @author  MCD Application Team
  * @brief   This file provides the USBD descriptors and string formating method.
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
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"


/* Private define ------------------------------------------------------------*/
#define USBD_VID                  0x0483
#if !defined (_CLASS_CDC)
#define USBD_PID                  0x572B /* to be modified vs controler PID*/
#else
#define USBD_PID                      0x5740
#endif

#define USBD_LANGID_STRING         0x409
#define USBD_MANUFACTURER_STRING   "STMicroelectronics"
    
#ifdef _CLASS_BB
#define USBD_PRODUCT_HS_STRING        "USB BillBoard Device"
#define USBD_SERIALNUMBER_HS_STRING   "00000000002A"
#define USBD_CONFIGURATION_HS_STRING  "BillBoard Configuration"
#define USBD_INTERFACE_HS_STRING      "BillBoard Interface"
#define USBD_PRODUCT_FS_STRING        "USB BillBoard Device"
#define USBD_SERIALNUMBER_FS_STRING   "00000000002B"
#define USBD_CONFIGURATION_FS_STRING  "BillBoard Configuration"
#define USBD_INTERFACE_FS_STRING      "BillBoard Interface"
    
#elif _CLASS_HID
#define USBD_PRODUCT_HS_STRING        "HID Joystick in HS Mode"
#define USBD_CONFIGURATION_HS_STRING  "HID Config"
#define USBD_INTERFACE_HS_STRING      "HID Interface"
#define USBD_PRODUCT_FS_STRING        "HID Joystick in FS Mode"
#define USBD_CONFIGURATION_FS_STRING  "HID Config"
#define USBD_INTERFACE_FS_STRING      "HID Interface"

#elif _CLASS_CDC
#define USBD_LANGID_STRING            0x409
#define USBD_MANUFACTURER_STRING      "STMicroelectronics"
#define USBD_PRODUCT_HS_STRING        "STM32 Virtual ComPort in HS Mode"
#define USBD_PRODUCT_FS_STRING        "STM32 Virtual ComPort in FS Mode"
#define USBD_CONFIGURATION_HS_STRING  "VCP Config"
#define USBD_INTERFACE_HS_STRING      "VCP Interface"
#define USBD_CONFIGURATION_FS_STRING  "VCP Config"
#define USBD_INTERFACE_FS_STRING      "VCP Interface"
#endif
 
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#if defined(_CLASS_HID)||defined(_CLASS_CDC)
static void Get_SerialNum(void);
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len);
#endif
/* Private variables ---------------------------------------------------------*/

/** @defgroup USBD_DESC_Private_Variables
  * @{
  */ 
#ifdef _CLASS_BB
uint8_t *     USBD_BB_DeviceDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_BB_LangIDStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_BB_ManufacturerStrDescriptor ( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_BB_ProductStrDescriptor ( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_BB_SerialStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_BB_ConfigStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
uint8_t *     USBD_BB_InterfaceStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
#if (USBD_CLASS_USER_STRING_DESC == 1)
uint8_t *USBD_BB_UserStrDescriptor(USBD_SpeedTypeDef speed, uint8_t idx, uint16_t *length);
#endif /* USB_SUPPORT_USER_STRING_DESC */

#if (USBD_CLASS_BOS_ENABLED == 1)
uint8_t *     USBD_BB_BOSDescriptor( USBD_SpeedTypeDef speed , uint16_t *length);
#endif
#endif /* _CLASS_BB */
#if _CLASS_HID
uint8_t *USBD_HID_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_HID_LangIDStrDescriptor(USBD_SpeedTypeDef speed,
                                      uint16_t *length);
uint8_t *USBD_HID_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed,
                                            uint16_t *length);
uint8_t *USBD_HID_ProductStrDescriptor(USBD_SpeedTypeDef speed,
                                       uint16_t *length);
uint8_t *USBD_HID_SerialStrDescriptor(USBD_SpeedTypeDef speed,
                                      uint16_t *length);
uint8_t *USBD_HID_ConfigStrDescriptor(USBD_SpeedTypeDef speed,
                                      uint16_t *length);
uint8_t *USBD_HID_InterfaceStrDescriptor(USBD_SpeedTypeDef speed,
                                         uint16_t *length);
#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *USBD_HID_USRStringDesc(USBD_SpeedTypeDef speed, uint8_t idx,
                                uint16_t * length);
#endif                          /* USB_SUPPORT_USER_STRING_DESC */

/* USB Standard Device Descriptor */
#if defined ( __ICCARM__ )   
/* !< IAR Compiler */
#pragma data_alignment=4
#endif
#endif
#if defined (_CLASS_HID)||(_CLASS_CDC)

__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12,                         /* bLength */
  USB_DESC_TYPE_DEVICE,         /* bDescriptorType */
  0x00,                         /* bcdUSB */
  0x02,
#if defined (_CLASS_HID)
  0x00,                         /* bDeviceClass */
  0x00,                         /* bDeviceSubClass */
#elif defined (_CLASS_CDC)
  0x02,                       /* bDeviceClass */
  0x02,                       /* bDeviceSubClass */
#endif
  0x00,                         /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,             /* bMaxPacketSize */
  LOBYTE(USBD_VID),             /* idVendor */
  HIBYTE(USBD_VID),             /* idVendor */
  LOBYTE(USBD_PID),             /* idVendor */
  HIBYTE(USBD_PID),             /* idVendor */
  0x00,                         /* bcdDevice rel. 2.00 */
  0x02,
  USBD_IDX_MFC_STR,             /* Index of manufacturer string */
  USBD_IDX_PRODUCT_STR,         /* Index of product string */
  USBD_IDX_SERIAL_STR,          /* Index of serial number string */
  USBD_MAX_NUM_CONFIGURATION    /* bNumConfigurations */
}; 
#endif
#if _CLASS_CDC
uint8_t *USBD_VCP_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_VCP_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_VCP_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_VCP_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_VCP_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_VCP_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_VCP_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#endif

#ifdef _CLASS_BB
USBD_DescriptorsTypeDef BB_Desc =
{
  USBD_BB_DeviceDescriptor,  
  USBD_BB_LangIDStrDescriptor, 
  USBD_BB_ManufacturerStrDescriptor,
  USBD_BB_ProductStrDescriptor,
  USBD_BB_SerialStrDescriptor,
  USBD_BB_ConfigStrDescriptor,
  USBD_BB_InterfaceStrDescriptor,
#if (USBD_CLASS_USER_STRING_DESC == 1)
  USBD_BB_UserStrDescriptor,
#endif
#if (USBD_CLASS_BOS_ENABLED == 1)
  USBD_BB_BOSDescriptor,
#endif
};
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */
#if defined ( __ICCARM__ ) 
/*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t hUSBDDeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
  {
    0x12,                       /*bLength */
    USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
#if (USBD_CLASS_BOS_ENABLED == 1)
    0x01,                       /*bcdUSB */     /* changed to USB version 2.01
                                              in order to support BOS Desc */
#else
     0x00,                       /* bcdUSB */
#endif
    0x02,
    0x11,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
    LOBYTE(USBD_VID),           /*idVendor*/
    HIBYTE(USBD_VID),           /*idVendor*/
    LOBYTE(USBD_PID),           /*idProduct*/
    HIBYTE(USBD_PID),           /*idProduct*/
    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,       /*Index of product string*/
    USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
    USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
  }; 
 /* USB_DeviceDescriptor BB*/
/* BOS descriptor */
#if (USBD_CLASS_BOS_ENABLED == 1)
#if defined ( __ICCARM__ ) 
/*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN  uint8_t hUSBDBOSDesc[USB_SIZ_BOS_DESC] __ALIGN_END =
{
0x05,                                /* bLength */
USB_DESC_TYPE_BOS,                   /* Device Descriptor Type */
USB_SIZ_BOS_DESC,                    /* Total length of BOS descriptor and all of its sub descs */
0x00,
0x04,                                /* The number of separate device capability descriptors in the BOS */

    /* ----------- Device Capability Descriptor: CONTAINER_ID ---------- */
0x14,                                /* bLength */
USB_DEVICE_CAPABITY_TYPE,            /* bDescriptorType: DEVICE CAPABILITY Type */
0x04,                                /* bDevCapabilityType: CONTAINER_ID */
0x00,                                /* bReserved */
0xa7, 0xd6, 0x1b, 0xfa,              /* ContainerID: This is a Unique 128-bit number GUID */
0x91, 0xa6, 0xa8, 0x4e,
0xa8, 0x21, 0x9f, 0x2b,
0xaf, 0xf7, 0x94, 0xd4,

    /* ----------- Device Capability Descriptor: BillBoard ---------- */
0x34,                                /* bLength */
0x10,                                /* bDescriptorType: DEVICE CAPABILITY Type */
0x0D,                                /* bDevCapabilityType: BILLBOARD_CAPABILITY */
USBD_BB_URL_STRING_INDEX,            /* iAddtionalInfoURL: Index of string descriptor providing a URL where the user can go to get more
                                        detailed information about the product and the various Alternate Modes it supports */

0x02,                                /* bNumberOfAlternateModes: Number of Alternate modes supported. The
                                        maximum value that this field can be set to is MAX_NUM_ALT_MODE. */

0x00,                                /* bPreferredAlternateMode: Index of the preferred Alternate Mode. System
                                        software may use this information to provide the user with a better user experience. */

0x00, 0x00,                          /* VCONN Power needed by the adapter for full functionality 000b = 1W */

0x01,0x00,0x00,0x00,                 /* bmConfigured. 01b: Alternate Mode configuration not attempted or exited */
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,
0x21, 0x01,                          /* bcdVersion = 0x0121 */
0x00,                                /* bAdditionalFailureInfo */
0x00,                                /* bReserved */
LOBYTE(USBD_VID),
HIBYTE(USBD_VID),                    /* wSVID[0]: Standard or Vendor ID. This shall match one of the SVIDs
                                        returned in response to a USB PD Discover SVIDs command */

0x00,                                /* bAlternateMode[0] Index of the Alternate Mode within the SVID as
                                        returned in response to a Discover Modes command. Example:
                                        0 ? first Mode entry
                                        1 ? second mode entry */

USBD_BB_ALTMODE0_STRING_INDEX,       /* iAlternateModeString[0]: Index of string descriptor describing protocol.
                                        It is optional to support this string. */
LOBYTE(USBD_VID),
HIBYTE(USBD_VID),                    /* wSVID[1]: Standard or Vendor ID. This shall match one of the SVIDs
                                        returned in response to a USB PD Discover SVIDs command */

0x01,                                /* bAlternateMode[1] Index of the Alternate Mode within the SVID as
                                        returned in response to a Discover Modes command. Example:
                                        0 ? first Mode entry
                                        1 ? second Mode entry */

USBD_BB_ALTMODE1_STRING_INDEX,           /* iAlternateModeString[1]: Index of string descriptor describing protocol.
                                        It is optional to support this string. */
 /* Alternate Mode Desc */
    /* ----------- Device Capability Descriptor: BillBoard Alternate Mode Desc ---------- */
0x08,                                /* bLength */
0x10,                                /* bDescriptorType: Device Descriptor Type */
0x0F,                                /* bDevCapabilityType: BILLBOARD ALTERNATE MODE CAPABILITY */
0x00,                                /* bIndex: Index of Alternate Mode described in the Billboard Capability Desc */
0x10, 0x00, 0x00, 0x00,              /* dwAlternateModeVdo: contents of the Mode VDO for the alternate mode identified by bIndex */

0x08,                                /* bLength */
0x10,                                /* bDescriptorType: Device Descriptor Type */
0x0F,                                /* bDevCapabilityType: BILLBOARD ALTERNATE MODE CAPABILITY */
0x01,                                /* bIndex: Index of Alternate Mode described in the Billboard Capability Desc */
0x20, 0x00, 0x00, 0x00,              /* dwAlternateModeVdo: contents of the Mode VDO for the alternate mode identified by bIndex */
};
#endif
#elif _CLASS_HID

USBD_DescriptorsTypeDef HID_Desc =
{
  USBD_HID_DeviceDescriptor,
  USBD_HID_LangIDStrDescriptor,
  USBD_HID_ManufacturerStrDescriptor,
  USBD_HID_ProductStrDescriptor,
  USBD_HID_SerialStrDescriptor,
  USBD_HID_ConfigStrDescriptor,
  USBD_HID_InterfaceStrDescriptor
};

#if defined ( __ICCARM__ ) 
/* IAR Compiler */
  #pragma data_alignment=4
#endif 
/** USB standard device descriptor. */
__ALIGN_BEGIN uint8_t USBD_HID_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB */
  0x02,
  0x00,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID),           /*idProduct*/
  HIBYTE(USBD_PID),           /*idProduct*/
  0x00,                       /*bcdDevice rel. 2.00*/
  0x02,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};
#endif

#ifdef _CLASS_CDC
USBD_DescriptorsTypeDef VCP_Desc = {
  USBD_VCP_DeviceDescriptor,
  USBD_VCP_LangIDStrDescriptor, 
  USBD_VCP_ManufacturerStrDescriptor,
  USBD_VCP_ProductStrDescriptor,
  USBD_VCP_SerialStrDescriptor,
  USBD_VCP_ConfigStrDescriptor,
  USBD_VCP_InterfaceStrDescriptor,  
};
#endif
#if defined ( __ICCARM__ ) 
/* IAR Compiler */
  #pragma data_alignment=4
#endif 

/** USB lang indentifier descriptor. */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

#if defined ( __ICCARM__ ) 
/* IAR Compiler */
  #pragma data_alignment=4
#endif 
/* Internal string descriptor. */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

#if defined ( __ICCARM__ ) 
/*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};
#if defined ( __ICCARM__ ) 
/*!< IAR Compiler */
  #pragma data_alignment=4   
#endif
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

#if defined(_CLASS_CDC)
/**
  * @brief  Returns the device descriptor. 
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t *USBD_VCP_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(speed);

  *length = sizeof(USBD_DeviceDesc);
  return (uint8_t*)USBD_DeviceDesc;
}
/**
  * @brief  Returns the LangID string descriptor.        
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t *USBD_VCP_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(speed);

  *length = sizeof(USBD_LangIDDesc);  
  return (uint8_t*)USBD_LangIDDesc;
}

/**
  * @brief  Returns the product string descriptor. 
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t *USBD_VCP_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {   
    USBD_GetString((uint8_t *)USBD_PRODUCT_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);    
  }
  return USBD_StrDesc;
}

/**
  * @brief  Returns the manufacturer string descriptor. 
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t *USBD_VCP_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(speed);

  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}
/**
  * @brief  Returns the serial number string descriptor.        
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t *USBD_VCP_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(speed);

  *length = USB_SIZ_STRING_SERIAL;
  
  /* Update the serial number string descriptor with the data from the unique ID*/
  Get_SerialNum();
  
  return (uint8_t*)USBD_StringSerial;
}

/**
  * @brief  Returns the configuration string descriptor.    
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t *USBD_VCP_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {  
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length); 
  }
  return USBD_StrDesc;  
}

/**
  * @brief  Returns the interface string descriptor.        
  * @param  speed: Current device speed
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t *USBD_VCP_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;  
}
#endif


/**
  * @}
  */

/** @defgroup USBD_DESC_Private_Functions USBD_DESC_Private_Functions
  * @brief Private functions.
  * @{
  */

#if defined(_CLASS_BB)
/**
* @brief  USBD_BB_DeviceDescriptor
*         return the device descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_BB_DeviceDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  *length = sizeof(hUSBDDeviceDesc);
  return hUSBDDeviceDesc;
}

/**
* @brief  USBD_BB_LangIDStrDescriptor
*         return the LangID string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_BB_LangIDStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  *length =  sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

/**
* @brief  USBD_BB_ProductStrDescriptor
*         return the product string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_BB_ProductStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
  USBD_GetString ((uint8_t *)USBD_PRODUCT_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
      USBD_GetString ((uint8_t *)USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
* @brief  USBD_BB_ManufacturerStrDescriptor
*         return the manufacturer string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_BB_ManufacturerStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  USBD_GetString ((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
* @brief  USBD_BB_SerialStrDescriptor
*         return the serial number string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_BB_SerialStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
  USBD_GetString ((uint8_t *)USBD_SERIALNUMBER_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
      USBD_GetString ((uint8_t *)USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
* @brief  USBD_BB_ConfigStrDescriptor
*         return the configuration string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_BB_ConfigStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
   if(speed == USBD_SPEED_HIGH)
  {
  USBD_GetString ((uint8_t *)USBD_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
  USBD_GetString ((uint8_t *)USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
* @brief  USBD_BB_InterfaceStrDescriptor
*         return the interface string descriptor
* @param  speed : current device speed
* @param  length : pointer to data length variable
* @retval pointer to descriptor buffer
*/
uint8_t *  USBD_BB_InterfaceStrDescriptor( USBD_SpeedTypeDef speed , uint16_t *length)
{
   if(speed == USBD_SPEED_HIGH)
  {
  USBD_GetString ((uint8_t *)USBD_INTERFACE_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
      USBD_GetString ((uint8_t *)USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}


#if (USBD_CLASS_BOS_ENABLED == 1)

/**
  * @brief  USBD_BB_BOSDescriptor
  *         return the BOS descriptor
  * @param  speed : current device speed
  * @param  length : pointer to data length variable
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_BB_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  *length = hUSBDBOSDesc[2];
  return (uint8_t*)hUSBDBOSDesc;
}
#endif


#if (USBD_CLASS_USER_STRING_DESC == 1)
/**
  * @brief  Returns the Class User string descriptor.
  * @param  speed: Current device speed
  * @param  idx: index of string descriptor
  * @param  length: Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t *USBD_BB_UserStrDescriptor(USBD_SpeedTypeDef speed, uint8_t idx, uint16_t *length)
{
  static uint8_t USBD_StrDesc[255];

  switch (idx)
  {
  case USBD_BB_IF_STRING_INDEX:
    USBD_GetString(USBD_BB_IF_STR_DESC, USBD_StrDesc, length);
    break;

  case USBD_BB_URL_STRING_INDEX:
    USBD_GetString(USBD_BB_URL_STR_DESC, USBD_StrDesc, length);
    break;

  case USBD_BB_ALTMODE0_STRING_INDEX:
    USBD_GetString(USBD_BB_ALTMODE0_STR_DESC, USBD_StrDesc, length);
    break;

  case USBD_BB_ALTMODE1_STRING_INDEX:
    USBD_GetString(USBD_BB_ALTMODE1_STR_DESC, USBD_StrDesc, length);
    break;

  default:
    USBD_GetString(USBD_BB_URL_STR_DESC, USBD_StrDesc, length);
    break;
  }

  return USBD_StrDesc;
}
#endif

/* CLASS_BILLBOARD */
#elif _CLASS_HID 
/**
  * @brief  Return the device descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_HID_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  *length = sizeof(USBD_HID_DeviceDesc);
  return USBD_HID_DeviceDesc;
}

/**
  * @brief  Return the LangID string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_HID_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

/**
  * @brief  Return the product string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_HID_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the manufacturer string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_HID_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

/**
  * @brief  Return the serial number string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_HID_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  *length = USB_SIZ_STRING_SERIAL;

  /* Update the serial number string descriptor with the data from the unique
   * ID */
  Get_SerialNum();

  return (uint8_t *) USBD_StringSerial;
}

/**
  * @brief  Return the configuration string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_HID_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

/**
  * @brief  Return the interface string descriptor
  * @param  speed : Current device speed
  * @param  length : Pointer to data length variable
  * @retval Pointer to descriptor buffer
  */
uint8_t * USBD_HID_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_HS_STRING, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}
#endif
#if defined(_CLASS_HID)||defined(_CLASS_CDC)

/**
  * @brief  Create the serial number string descriptor
  * @param  None
  * @retval None
  */
static void Get_SerialNum(void)
{
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = *(uint32_t *) DEVICE_ID1;
  deviceserial1 = *(uint32_t *) DEVICE_ID2;
  deviceserial2 = *(uint32_t *) DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0)
  {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

/**
  * @brief  Convert Hex 32Bits value into char
  * @param  value: value to convert
  * @param  pbuf: pointer to the buffer
  * @param  len: buffer length
  * @retval None
  */
static void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len)
{
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++)
  {
    if (((value >> 28)) < 0xA)
    {
      pbuf[2 * idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
}
#endif /* USB_HID */
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
