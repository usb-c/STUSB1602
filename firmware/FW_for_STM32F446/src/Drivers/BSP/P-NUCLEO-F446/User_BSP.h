/**
  ******************************************************************************
  * @file    User_BSP.h
  * @author  AMG Application Team
  * @brief   This file contains bsp interface control functions.
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
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
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
/*#warning "User_BSP header bof"*/
#ifndef USER_BSP_H
#define USER_BSP_H
/* Includes ------------------------------------------------------------------*/
/*#warning "User_BSP header bof 2"*/
#include "p-nucleo-F446.h"  /* to port the device to other Cortex family we could change it Here */
//#include "usbpd_def.h"
#include "usbpd_stusb_dpm_if.h"
#include "usbpd_timersserver.h"
#include "usbpd_porthandle.h"
//#include "usbpd_hw_if.h"
#if defined (_RTOS)
#include "cmsis_os.h"   
#endif
#include "usbpd_core.h"
//#include "usbd_desc.h"
#if defined(SPI_ONE_LINE)
#include "stm32f4xx_ll_spi.h"
#endif
#if _CLASS_HID
#include "usbd_hid.h" 
#endif
#include "string.h"    
#if defined(_TRACE)
#include "usbpd_trace.h"
#include <stdio.h>
#endif /* _TRACE */
#ifdef _ADC_MONITORING
#include "usbpd_dpm_user.h"
#endif

/*#warning "User_BSP header eof 2"*/
#endif
/*#warning "User_BSP header eof"*/
