/**
******************************************************************************
* @file    usbpd_phy_hw_if.c
* @author  AMG
* @brief   This file contains phy hardware interface control functions.
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
#include "STUSB1602_Peripherals_if.h"
#include "usbpd_phy_hw_if.h"

/** @addtogroup STM32_USBPD_LIBRARY
* @{
*/

/** @addtogroup USBPD_DEVICE
* @{
*/

/** @addtogroup USBPD_DEVICE_HW_IF
* @{
*/

/** @addtogroup USBPD_DEVICE_PHY_HW_IF
* @{
*/

extern STUSB16xx_PORT_HandleTypeDef Ports[];
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
__ALIGN_BEGIN uint8_t TXRXBuffer0[TXRX_BUFFER_SIZE] __ALIGN_END;      /*!< Buffer storing raw received data on port 0 */
#if (USBPD_PORT_COUNT == 2)
__ALIGN_BEGIN uint8_t TXRXBuffer1[TXRX_BUFFER_SIZE] __ALIGN_END;      /*!< Buffer storing raw received data on port 1 */
#endif

USBPD_StatusTypeDef HW_IF_check_bus_idle(uint8_t PortNum);

/* Private function prototypes -----------------------------------------------*/
/* Unwrap data init function */
void HW_IF_UnwrapData_Init(uint8_t PortNum);

/* Inner functions prototypes ------------------------------------------------*/
static inline void HW_IF_RX_CompleteParsingData(uint8_t PortNum);
/* private functions ---------------------------------------------*/


/* Public functions ----------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_PHY_HW_IF_Public_Functions USBPD DEVICE PHY HW IF Public functions
* @details Public functions can be used at stack level
* @{
*/

/* Pre-computed values for polynom 0x04C11DB7 */
const uint32_t crc32_tab[] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

/* CRC is calculated during frame encoding 5b for Tx , and frame 5b decoding for Rx */
/**
* @brief  It attaches the preamble at the beginning of the packet and moves it towards the SPI  
* @param  PortNum       The port index
* @param  *pBuffer      Pointer to the TX data buffer
* @param  Bitsize:      Amount of bits to be transmitted
* @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_HW_IF_SendBuffer(uint8_t PortNum, uint8_t *pBuffer, uint32_t Bitsize)
{
  /* Check if the port is still receiving */
  if (Ports[PortNum].State == HAL_USBPD_PORT_STATE_BUSY_RX)
    return USBPD_BUSY;
  
  uint16_t size = DIV_ROUND_UP(Bitsize, 8) + TX_PREAMBLE_SIZE;
  
  
  /* Spare clock cycles at the end of transmission are calculated */
  Ports[PortNum].TxSpareBits = (Bitsize % 8);
    
  /* Packet is ready to be sent to SPI */
 
  return STUSB16xx_HW_IF_Send_Packet(PortNum, pBuffer, size);;
}


/**
* @brief  It sends BIST pattern  
* @param  PortNum       The port index
* @retval USBPD status
*/
USBPD_StatusTypeDef USBPD_HW_IF_Send_BIST_Pattern(uint8_t PortNum)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  /* BIST Carrier mode flag set */
  Ports[PortNum].State=HAL_USBPD_PORT_STATE_BIST;
  
  /* Fill the buffer with the pattern to be sent */
  memset(Ports[PortNum].pTxRxBuffPtr, 0xAA, TX_BUFFER_LEN);
  
  /* start a circular DMA transfer */
  STUSB16xx_HW_IF_Set_DMA_Circular_Mode(PortNum);
  
  /* Set the SPI in TX mode */
  HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_TX);
  
  HAL_SPI_DMAStop(&Ports[PortNum].hspi);

#if defined (SPI_SR_FTLVL)
    __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_GI_FLAG_INDEX(&Ports[PortNum].hdmatx));
#else
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[PortNum].hdmatx));
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(&Ports[PortNum].hdmatx));
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(&Ports[PortNum].hdmatx));
#endif
  
  
  
  /* Send TX Buffer by SPI DMA */
  HAL_SPI_Transmit_DMA(&Ports[PortNum].hspi, (uint8_t*)(Ports[PortNum].pTxRxBuffPtr), TX_BUFFER_LEN);
  
  /* Start transmission */
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_SET);
  
  ret = USBPD_OK;
  return ret;
}


/**
* @brief  It checks if the bus is idle
* @param  PortNum The port index
* @retval USBPD_StatusTypeDef
*/
USBPD_StatusTypeDef HW_IF_check_bus_idle(uint8_t PortNum)
{
  return (((Ports[PortNum].CCx == CCNONE) || ((HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum)) == GPIO_PIN_RESET))) ? USBPD_BUSY : USBPD_OK);
}


/**
* @brief Rx Transfer completed callback.
* @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
*               the configuration information for SPI module.
* @retval None
*/
void RX_ByteReceiverHandler(uint8_t PortNum)
{
  
  
  uint8_t r, prev_bit, curr_bit;
  uint32_t temp_data = 0;
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  uint8_t* pbuff_in = Ports[PortNum].pTxRxBuffPtr;
  SPI_HandleTypeDef* rx_spi = &(Ports[PortNum].hspi); 
  if (__HAL_SPI_GET_IT_SOURCE(rx_spi, SPI_IT_RXNE)!= RESET) 
  {
    
    if (HAL_GPIO_ReadPin(SPI_NSS_PORT(PortNum), SPI_NSS_PIN(PortNum)) == GPIO_PIN_RESET)    /* stopping the decoding in case of NSS is high */
    {
      
      if ( (USBPD_TIM_IsExpired(((PortNum == 0 )? TIM_PORT0_CA:TIM_PORT1_CA))) && (ud->preamble) )
      {
        /* CRC timeout */ /* too much time between 2 Bytes)*/ 
        ud->exed_flag = 9; 
        __HAL_SPI_DISABLE_IT(rx_spi, (SPI_IT_RXNE | SPI_IT_ERR  ));  
        
      }
      else
      {
        USBPD_TIM_Start(((PortNum == 0 )? TIM_PORT0_CA:TIM_PORT1_CA), PHY_TXRX_BYTE_TIMEOUT);  
#ifdef DMA_PFCTRL            
          if( (ud->exed_flag == 0) && ((TXRX_BUFFER_SIZE - Ports[PortNum].hdmarx.Instance->NDTR)>(ud->index + 2)) ) 
#else
            if( (ud->exed_flag == 0) && ((TXRX_BUFFER_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR)>(ud->index + 2)) )  
#endif            
            {
              
              if (!ud->preamble)                  /* The end of preamble hasn't identified yet */ 
              {
                /* Search end of preamble */
                r = pbuff_in[ud->index]^0xAA;
                if (r == 0x00 || r == 0xFF)       /* The end of preamble is not part of the received data */ 
                {
                  /* Preamble */
                  ud->index++;
                }
                else                              /* Received data contain the end of preamble */
                {
                  prev_bit = (pbuff_in[ud->index-1]>>7) & 0x01;
                  while (ud->offset < 8)
                  {
                    curr_bit = (pbuff_in[ud->index]>>ud->offset) & 0x01;
                    if (prev_bit == curr_bit)
                    {
                      /* Preamble identified. Index and offset identify the byte and the bit position of data after preamble */
                      if (curr_bit == 0)
                      {
                        if (ud->offset == 0)
                        {
                          ud->offset = 7;
                          ud->index--;
                        }
                        else
                          ud->offset--;
                      }
                      ud->preamble = 1;
#ifdef __STAT                      
                      ud->preamble_offset = ud->offset;
                      ud->preamble_index = ud->index;
#endif                      
                      break;
                    }
                    prev_bit = curr_bit;
                    ud->offset++;
                  }
                }             
              }
              else
              {
                /* Extract 10-bits from data flow (structured in bytes) */
                memcpy(&temp_data, &pbuff_in[ud->index], 3);
                temp_data = (temp_data >> ud->offset) & 0x3FF;
                ud->index += ud->offset <= 5 ? 1 : 2;
                ud->offset = (ud->offset + 2) & 7;
                
                /* Callback core phy accumulate */
                if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate != NULL)
                {
                  Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate(PortNum, temp_data);
                }
                
                /* EOP detecting */
                if ((temp_data & 0x1F) == 0x0D) /* EOP */
                {
                  /* EOP to be managed */
                  ud->exed_flag = 2;
                  SWCALL_RX_STOP(PortNum);
                  
                }
              }
            }
            else
            {
              USBPD_TIM_Start(((PortNum == 0 )? TIM_PORT0_CA:TIM_PORT1_CA), PHY_TXRX_BYTE_TIMEOUT ); 
              if (ud->exed_flag == 2)
              { 
                ud->exed_flag = 3 ;
                __HAL_SPI_DISABLE_IT(rx_spi, (SPI_IT_RXNE | SPI_IT_ERR  ));  
                
              }  
            }
      } 
    } 
  }
  if (__HAL_SPI_GET_IT_SOURCE(rx_spi, SPI_IT_ERR)!= RESET)
  {
    __HAL_SPI_CLEAR_OVRFLAG(rx_spi) ;
  }
}


/**
* @brief   It performs actions at hardware level when a RESET event occurs
* @details Not implemented
* @param   PortNum The port index
* @param   Mode Allowed values are ACKNOWLEDGE or REQUEST
* @retval  None
*/
void USBPD_HW_IF_Reset(uint8_t PortNum, USBPD_HRPRS_Mode_TypeDef Mode)
{
  __NOP();
}



/** @} End of PUBLIC_FUNCTIONS group   */


/* Private functions ---------------------------------------------------------*/


/**
* @brief  PHY is prepared for data receiving phase
* @param  PortNum The port index
* @retval None
*/
void PHY_HW_IF_RX_Start(uint8_t PortNum)
{
  SPI_HandleTypeDef* rx_spi = &(Ports[PortNum].hspi);
  /* Callback core phy reset */
  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Reset != NULL)
  {
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Reset(PortNum); 
  }
  
  /* Set the state of the port */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_BUSY_RX;
  
  /* Variables for decoding stage are initalized */
  HW_IF_UnwrapData_Init(PortNum);
  
  HAL_SPI_DMAStop(rx_spi);
#if defined (SPI_SR_FTLVL)
    __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_GI_FLAG_INDEX(&Ports[PortNum].hdmarx));
#else
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[PortNum].hdmarx));
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(&Ports[PortNum].hdmarx));
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(&Ports[PortNum].hdmarx));
#endif
  
  
#if defined(HAL_SPIEx_FlushRxFifo)
  HAL_SPIEx_FlushRxFifo(rx_spi);
#endif  
  HAL_NVIC_EnableIRQ(SPI_IRQn(PortNum));
  
  /* Clean-up of buffer storing raw received data */
  memset(Ports[PortNum].pTxRxBuffPtr, 0x00, TXRX_BUFFER_SIZE);/* Preamble + 5b buffer reset */
  
  USBPD_TIM_Start(((PortNum == 0 )? TIM_PORT0_CA:TIM_PORT1_CA), PHY_TXRX_BYTE_TIMEOUT);
  
  /* Start DMA receiving */
  HAL_SPI_Receive_DMA(rx_spi, (uint8_t*)Ports[PortNum].pTxRxBuffPtr, TXRX_BUFFER_SIZE);
  __HAL_SPI_ENABLE_IT(rx_spi, SPI_IT_RXNE| SPI_IT_ERR );

}

/**
* @brief  PHY performe the data receiving phase
* @param  PortNum The port index
* @retval None
*/
void PHY_HW_IF_RX_Stop(uint8_t PortNum)
{
  /* Within this function a check of the exed_flag variable is made.
  * The values associated to the exit cases are:
  * 0 - Reset value
  * 1 - 
  * 2 - EOP detected
  * 3 - PHY_HW_IF_RX_Stop function is ongoing
  */
  SPI_HandleTypeDef* rx_spi = &(Ports[PortNum].hspi);
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  /* Stop DMA */
  HAL_DMA_Abort(&Ports[PortNum].hdmarx);
  /* Disable the selected SPI peripheral */
#ifdef DMA_PFCTRL  
  __HAL_SPI_DISABLE(&Ports[PortNum].hspi);
#endif  
  __HAL_SPI_DISABLE_IT(rx_spi, (SPI_IT_RXNE | SPI_IT_ERR | SPI_IT_TXE ));    
  HAL_NVIC_DisableIRQ(SPI_IRQn(PortNum)) ; 
  /* Set the state of the port */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  if (USBPD_TIM_IsExpired(((PortNum == 0 )? TIM_PORT0_CA:TIM_PORT1_CA)) )  /* Colision avoidance */
  { 
    ud->exed_flag = 6; 
    return ;
  }
  if (ud->exed_flag == 3)  
  {
    
   __NOP(); 
  }
  /* Complete the parsing process */ /* EOF detected during Reception IT */
  if (ud->exed_flag == 2)  
  { 
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed(PortNum);
    return;
  }
  if (ud->exed_flag == 0)  /*reception ended by NSS rising edge */
  {
    HW_IF_RX_CompleteParsingData(PortNum);
  }
  
  /* Callback core phy completed */
  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed != NULL)
  {    
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Completed(PortNum);
  }
  
}


/**
* @brief  Packet transmission has been accomplished
* @param  PortNum The port index
* @retval none
*/
void PHY_HW_IF_TX_Done(uint8_t PortNum)
{
  
  uint8_t i,j = 0;
  USBPD_TIM_Start(((PortNum == 0 )? TIM_PORT0_CA:TIM_PORT1_CA), PHY_TXRX_BYTE_TIMEOUT);
  /* Wait until FIFO is empty */
  
#if defined (SPI_SR_FTLVL) 
  if (Ports[PortNum].TxSpareBits == 0)
  {
    while ( (Ports[PortNum].hspi.Instance->SR & SPI_SR_FTLVL) != 0);  /* != 0x0800 */
  }
  else
  {
    while (((((Ports[PortNum].hspi.Instance->SR & SPI_SR_FTLVL)) >> SPI_SR_FTLVL_Pos) & 0x03) > 1)
    {
      __NOP();
    }
  }
  /* Wait for BUSY flag */
  do
  {
    j = USBPD_TIM_IsExpired(((PortNum == 0 )? TIM_PORT0_CA:TIM_PORT1_CA));
  }
  while (( (Ports[PortNum].hspi.Instance->SR & SPI_SR_BSY) > 0) || (j==0) );  
#else
  do
  {
    j = USBPD_TIM_IsExpired(((PortNum == 0 )? TIM_PORT0_CA:TIM_PORT1_CA));
  }
  while (( (Ports[PortNum].hspi.Instance->SR & SPI_SR_TXE) != 0) && (j==0) );  
  /* Act on TX spare bits */
  /* Cut_1_A */
  if (Ports[PortNum].State!=HAL_USBPD_PORT_STATE_BIST)
#endif 
    
    for(i=0; i<(Ports[PortNum].TxSpareBits - SPARE_BIT_SUB_CORRECTION ); i++)    
    {
      
      /* Wait for SPI CLK GPIO flag RESET */
      while (!HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)) && HAL_GPIO_ReadPin(TX_EN_GPIO_PORT(PortNum), TX_EN_GPIO_PIN(PortNum)) ); 
      
      /* Wait for SPI CLK GPIO flag SET */
      while (HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)) && HAL_GPIO_ReadPin(TX_EN_GPIO_PORT(PortNum), TX_EN_GPIO_PIN(PortNum)));
      /* if TXEN is already disable --> Tx ABORT occurs*/
    }
  
  /* Reset TX_EN GPIO */
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_RESET);
  
  /* Here the SPI has completed the transmission*/
#if !defined (SPI_SR_FTLVL)  
  Ports[PortNum].hspi.Instance->CR2 = 0; 
#endif
  /* Disable the selected SPI peripheral */
  __HAL_SPI_DISABLE(&Ports[PortNum].hspi);
  
  /* RX FIFO is cleaned */
#if defined (SPI_SR_FTLVL)
  uint8_t dummyDR;
  while ((Ports[PortNum].hspi.Instance->SR & SPI_SR_FRLVL) != 0)
  {
    dummyDR= *(__IO uint8_t *)&Ports[PortNum].hspi.Instance->DR;
    UNUSED(dummyDR);
  }
#endif  
  /* SPI DMA is stopped */
  HAL_SPI_DMAStop(&Ports[PortNum].hspi);
  /* Check if BIST TX Done */
  if(Ports[PortNum].State==HAL_USBPD_PORT_STATE_BIST)
  {
    Ports[PortNum].State=HAL_USBPD_PORT_STATE_RESET;
    /* Evaluate callback*/
    if ((Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted != NULL) )
    {
      Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted(PortNum,USBPD_BIST_CARRIER_MODE2);
    }
  }
  else
  {
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_WAITING;
  }
  SPI_FORCE_RESET(PortNum);
  __NOP();
  SPI_RELEASE_RESET(PortNum);
    HW_IF_DMA_Init(PortNum);
    HW_IF_SPI_Init(PortNum);

  /* Set the RX mode */
  HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_RX);
  
  /* TX completed callback */
  if (Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted != NULL)
  {
    Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum, 0);
  }
}
/**
* @brief  Packet transmission must be aborted due to disconnection 
* @param  PortNum The port index
* @retval none
*/
void PHY_HW_IF_TX_ABORT(uint8_t PortNum)
{
#if defined (SPI_SR_FTLVL)  
  uint8_t dummyDR;
#endif  
  
  /* Reset TX_EN GPIO */
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_RESET);
  if (__HAL_SPI_GET_IT_SOURCE(&(Ports[PortNum].hspi), SPI_IT_ERR)!= RESET)
  {
    __HAL_SPI_CLEAR_OVRFLAG(&(Ports[PortNum].hspi)) ;
  }
  
  /* Here the SPI must be stopped due to error recovery or disconnection */
  
  /* Disable the selected SPI peripheral */
  __HAL_SPI_DISABLE(&Ports[PortNum].hspi);
#if defined (SPI_SR_FTLVL)
  /* RX FIFO is cleaned */
  while ((Ports[PortNum].hspi.Instance->SR & SPI_SR_FRLVL) != 0)
  {
    dummyDR= *(__IO uint8_t *)&Ports[PortNum].hspi.Instance->DR;
    UNUSED(dummyDR);
  }
  
  /* SPI DMA is stopped */
  HAL_SPI_DMAStop(&Ports[PortNum].hspi);
#else
  Ports[PortNum].hspi.Instance->CR2 = 0; 
#endif
  /* Check if BIST TX Done */
  if(Ports[PortNum].State==HAL_USBPD_PORT_STATE_BIST)
  {
    Ports[PortNum].State=HAL_USBPD_PORT_STATE_RESET;
    /* Evaluate callback*/
    if ((Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted != NULL) )
    {
      Ports[PortNum].cbs.USBPD_HW_IF_BistCompleted(PortNum,USBPD_BIST_CARRIER_MODE2);
    }
  }
  else
  {
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_WAITING;
  }
  
  /* Set the RX mode */
  HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_RX);
  
}

/**
* @brief  It enables the data trasmission from microcontroller to STUSB1602 device
* @param  PortNum The port index
* @param  pData The pointer to data buffer
* @param  Size The amount of data to be sent
* @retval USBPD status
*/
USBPD_StatusTypeDef STUSB16xx_HW_IF_Send_Packet(uint8_t PortNum, uint8_t *pData, uint16_t Size)
{
  USBPD_StatusTypeDef ret = USBPD_ERROR;
  
  /* Check if the bus is idle */
  ret = HW_IF_check_bus_idle(PortNum);
  if (ret == USBPD_OK /* tx ready */)
  {
    /* Set the state to busy*/
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_BUSY_TX;
    
    HAL_SPI_DMAStop(&Ports[PortNum].hspi);
    
#if defined (SPI_SR_FTLVL)
    __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_GI_FLAG_INDEX(&Ports[PortNum].hdmatx));
#else
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(&Ports[PortNum].hdmatx));
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(&Ports[PortNum].hdmatx));
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(&Ports[PortNum].hdmatx));
#endif    
    /* Set the SPI in TX mode */
    HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_TX);
    /* Send TX Buffer by SPI DMA */
#if defined (SPI_SR_FTLVL)
    HAL_SPI_Transmit_DMA(&Ports[PortNum].hspi, pData, Size);
#else  
    HAL_SPI_Transmit_DMA(&Ports[PortNum].hspi, pData, Size+1);
#endif   
    __HAL_SPI_DISABLE_IT(&Ports[PortNum].hspi, SPI_IT_RXNE || SPI_IT_TXE || SPI_IT_ERR  );
    /* Set TX_EN GPIO */
    STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_SET);
  }
  else 
    /* TX completed callback */
    if (Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted != NULL)
    {
      Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum, 1);
    }
  
  return ret;
}
/**
* @brief  TX_EN GPIO control function
* @details It sets or reset the TX_EN pin
* @param  PortNum The port index
* @param  status Two allowed values: GPIO_PIN_SET or GPIO_PIN_RESET
* @retval None
*/ 
void STUSB16xx_HW_IF_TX_EN_Status(uint8_t PortNum, GPIO_PinState status)
{
  HAL_GPIO_WritePin(TX_EN_GPIO_PORT(PortNum), TX_EN_GPIO_PIN(PortNum), status);
}

/**
* @brief  It updates the Ports handle stating the beginning of RX phase
* @param  PortNum The port index
* @retval None
*/
void HW_IF_RX_Enable(uint8_t PortNum)
{
  /* Set the port state to waiting */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_WAITING;
  
  if (Ports[PortNum].cbs.USBPD_HW_IF_ReceiveMessage != NULL)
  {
    __NOP();
  }
}


/**
* @brief  It updates the Ports handle stating the ending of RX phase
* @param  PortNum The port index
* @retval None
*/
void HW_IF_RX_Disable(uint8_t PortNum)
{
  /* The port is ready to transmit */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  
  if (Ports[PortNum].cbs.USBPD_HW_IF_ReceiveMessage != NULL)
  {
    __NOP();
  }
}


/** @}*/ /* End of PRIVATE_FUNCTIONS group*/


/* Inner functions -----------------------------------------------------------*/

/** @addtogroup USBPD_DEVICE_PHY_HW_IF_Inner_Functions USBPD DEVICE PHY HW IF Inner functions
* @details Inner functions can be used at file level
* @{
*/




/**
* @brief  UnwrapData structure init function
* @param  PortNum The port index
* @retval None
*/
void HW_IF_UnwrapData_Init(uint8_t PortNum)
{
  /* Decoding variables */
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  
  /* Init the decoding variables */
  ud->exed_flag =       0;
  ud->preamble =        0;
  ud->dataindex =       0;
  ud->dataoffset =      0;
  ud->index =           2;      /* It discards first two bytes */
  ud->offset =          0;

#ifdef __STAT  
  ud->preamble_offset = 0;
  ud->preamble_index = 0;
  ud->preamble_counter++;
#endif
}


/**
* @brief  It completes data parsing
* @param  PortNum The port index
* @retval None
*/
void HW_IF_RX_CompleteParsingData(uint8_t PortNum)
{
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  uint8_t* pbuff_in = Ports[PortNum].pTxRxBuffPtr;
  uint32_t temp_data = 0;

#ifdef DMA_PFCTRL
  uint16_t lastindex = (TXRX_BUFFER_SIZE - Ports[PortNum].hdmarx.Instance->NDTR);
#else  
  uint16_t lastindex = (TXRX_BUFFER_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR);
#endif 
  /* If callback is not available skip the accumulation phase */
  if (Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate == NULL)
  {
    __NOP();
    return;
  }
  while(ud->index <= lastindex)
  {
    memcpy(&temp_data, &pbuff_in[ud->index], 3);
    temp_data = (temp_data >> ud->offset) & 0x3FF;
    ud->index += ud->offset <= 5 ? 1 : 2;
    ud->offset = (ud->offset + 2) & 7;
    
    /* Callback core phy accumulate */
    Ports[PortNum].cbs.USBPD_HW_IF_RX_Accumulate(PortNum, temp_data);
    
    /* EOP detecting */
    if ((temp_data & 0x1F) == 0x0D)
    {
      break;
    }
  }
}

/** @}*/ // End of INNER_FUNCTIONS group


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
