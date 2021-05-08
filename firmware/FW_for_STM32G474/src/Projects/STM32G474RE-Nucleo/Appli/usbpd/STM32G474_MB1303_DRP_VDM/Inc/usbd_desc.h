/**
  ******************************************************************************
  * @file    usbd_desc.h
  * @author  MCD Application Team
  * @brief   Header for usbd_desc.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_DESC_H
#define __USBD_DESC_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define         DEVICE_ID1          (0x1FFF7A10)
#define         DEVICE_ID2          (0x1FFF7A14)
#define         DEVICE_ID3          (0x1FFF7A18)

#define  USB_SIZ_STRING_SERIAL       0x1A
#if defined(_CLASS_BB)
#if (USBD_LPM_ENABLED == 1)
#define  USB_SIZ_BOS_DESC            0x0CU
#elif (USBD_CLASS_BOS_ENABLED == 1)
#define  USB_SIZ_BOS_DESC            0x5DU    
#endif
extern uint8_t USBD_BOSDesc[USB_SIZ_BOS_DESC];
#endif

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#ifdef _CLASS_BB
extern USBD_DescriptorsTypeDef BB_Desc;
#define USBD_BB_IF_STRING_INDEX         0x10U
#define USBD_BB_URL_STRING_INDEX        0x11U
#define USBD_BB_ALTMODE0_STRING_INDEX   0x12U
#define USBD_BB_ALTMODE1_STRING_INDEX   0x13U
/* Add Specific USER string Desc */
#define USBD_BB_IF_STR_DESC           (uint8_t *)"STM32 BillBoard Interface"
#define USBD_BB_URL_STR_DESC          (uint8_t *)"www.st.com"
#define USBD_BB_ALTMODE0_STR_DESC     (uint8_t *)"STM32 Alternate0 Mode"
#define USBD_BB_ALTMODE1_STR_DESC     (uint8_t *)"STM32 Alternate1 Mode"


#if (USBD_LPM_ENABLED == 1)
#define  USB_SIZ_BOS_DESC            0x0CU
#elif (USBD_CLASS_BOS_ENABLED == 1)
#define  USB_SIZ_BOS_DESC            0x5DU
#endif
#endif
#if _CLASS_HID
extern USBD_DescriptorsTypeDef HID_Desc;
#endif
#if _CLASS_CDC
extern USBD_DescriptorsTypeDef VCP_Desc;
#endif


#endif /* __USBD_DESC_H */
