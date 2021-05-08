/**
  @page STM32L4R5_MB1303_DRP_VDM add here description
  
  @verbatim
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    usbpd/STM32L4R5_MB1303_DRP_VDM/readme.txt 
  * @author  MCD Application Team
  * @brief   add here the very short description of the example (the name ?).
  ******************************************************************************
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Example Description 

Add here a detailled but simple description of the example. this description will be
reused inside the application note which list all the example. This paragraph end when
a "." is at the end of a line like the following one ==> .

Add here the behavior of the example.
It is free text.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Keywords

Add here all the importants words of the example separated by "," + usbpd STM32L4R5_MB1303_DRP_VDM
usbpd, STM32L4R5_MB1303_DRP_VDM, Security, IEC 60870-5, hardware CRC, 

@par Directory contents 
  
  - usbpd/STM32L4R5_MB1303_DRP_VDM/Inc/stm32l4xx_hal_conf.h    HAL configuration file
  - usbpd/STM32L4R5_MB1303_DRP_VDM/Inc/stm32l4xx_it.h          Interrupt handlers header file
  - usbpd/STM32L4R5_MB1303_DRP_VDM/Inc/main.h                  Header for main.c module
  - usbpd/STM32L4R5_MB1303_DRP_VDM/Src/stm32l4xx_it.c          Interrupt handlers
  - usbpd/STM32L4R5_MB1303_DRP_VDM/Src/main.c                  Main program
  - usbpd/STM32L4R5_MB1303_DRP_VDM/Src/stm32l4xx_hal_msp.c     HAL MSP module 
  - usbpd/STM32L4R5_MB1303_DRP_VDM/Src/system_stm32l4xx.c      STM32L4xx system source file

     
@par Hardware and Software environment

  - This example runs on STUSB1602 devices.
  
  - This example has been tested with an STMicroelectronics STM32L4R5ZI-Nucleo (144 pins)
    board and can be easily tailored to any other supported device 
    and development board.

@par How to use it ? 

In order to make the program work, you must do the following:
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 