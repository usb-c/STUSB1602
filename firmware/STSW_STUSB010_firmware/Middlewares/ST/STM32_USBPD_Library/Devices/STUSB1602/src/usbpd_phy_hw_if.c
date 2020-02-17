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
uint8_t         TXRXBuffer0[RX_BUFFER_SIZE];      /*!< Buffer storing raw received data on port 0 */
//uint8_t         TXBuffer0[TX_BUFFER_SIZE];      /*!< Buffer that stores the packet to be transmitted on port 0 */
#if (USBPD_PORT_COUNT == 2)
uint8_t         TXRXBuffer1[RX_BUFFER_SIZE];      /*!< Buffer storing raw received data on port 1 */
//uint8_t         TXBuffer1[TX_BUFFER_SIZE];      /*!< Buffer that stores the packet to be transmitted on port 1 */
#endif

uint8_t preamble_offset;                        /*!< Offset identifies the byte of data after preamble */
uint8_t preamble_index;                         /*!< Index identifies the bit of data after preamble */
uint32_t preamble_counter = 0;                  /*!< Preamble counter */



// extern STUSB16xx_PORT_HandleTypeDef Ports[]; /* Handle for the ports inside @ref USBPD_DEVICE_HW_IF */
//uint8_t ud_index_current[3] = {0, 0,0};
//uint8_t modulo=0;

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


/**
* @brief  CRC calculation
* @param  *pBuffer    Pointer to the input data buffer
* @param  len         Input data buffer length
* @retval uint32_t    CRC value
*/
uint32_t USBPD_HW_IF_CRC_Calculate(uint8_t *pBuffer, uint16_t len)
{
  uint32_t crc=0;
  uint32_t index = 0U; /* CRC input data buffer index */
  /* Processing time optimization: 4 bytes are entered in a row with a single word write,
  * last bytes must be carefully fed to the CRC calculator to ensure a correct type
  * handling by the IP */
  
  LL_CRC_ResetCRCCalculationUnit(CRC);
  
  for(index = 0U; index < (len/4U); index++)
  {
    crc = ((uint32_t)pBuffer[4U*index]<<24U) | ((uint32_t)pBuffer[4U*index+1]<<16U) | ((uint32_t)pBuffer[4U*index+2]<<8U) | (uint32_t)pBuffer[4U*index+3];
    LL_CRC_FeedData32(CRC,crc);      
  }
  /* last bytes specific handling */  
  if ((len%4U) != 0U)
  {
    if  (len%4U == 1U)
    {
      LL_CRC_FeedData8(CRC,pBuffer[4*index]);
    }
    if  (len%4U == 2U)
    {
      LL_CRC_FeedData16(CRC,((uint32_t)pBuffer[4*index]<<8) | (uint32_t)pBuffer[4*index+1]);
    }
    if  (len%4U == 3U)
    {
      LL_CRC_FeedData16(CRC,((uint32_t)pBuffer[4*index]<<8) | (uint32_t)pBuffer[4*index+1]);
      LL_CRC_FeedData8(CRC,pBuffer[4*index]);     
    }
  }
  
  crc = LL_CRC_ReadData32(CRC);
  crc ^= 0xFFFFFFFF;
  return crc;
}


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
  
  uint8_t *pTxBuffer = (uint8_t *)Ports[PortNum].pTxRxBuffPtr;
  uint16_t size = DIV_ROUND_UP(Bitsize, 8)+TX_PREAMBLE_SIZE;
  
  memset((uint8_t *)pTxBuffer, 0x00, TX_BUFFER_SIZE);
  memset((uint8_t *)pTxBuffer, TX_PREAMBLE, TX_PREAMBLE_SIZE);                          /* preamble is added */
  memcpy((uint8_t *)(pTxBuffer+TX_PREAMBLE_SIZE), pBuffer, (size-TX_PREAMBLE_SIZE));    /* data are added */
  
  /* Spare clock cycles at the end of transmission are calculated */
  Ports[PortNum].TxSpareBits = (Bitsize % 8);
  
  /* Packet is ready to be sent to SPI */
  USBPD_StatusTypeDef ret = USBPD_OK;
  ret = STUSB16xx_HW_IF_Send_Packet(PortNum, pTxBuffer, size);
  return ret;
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
  __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx,__HAL_DMA_GET_HT_FLAG_INDEX(&Ports[PortNum].hdmatx));
  
  
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
      if (USBPD_TIM_IsExpired(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC)))
      {
        /* CRC timeout */ /* too much time between 2 Bytes)*/ 
        ud->exed_flag = 9; 
        __HAL_SPI_DISABLE_IT(rx_spi, (SPI_IT_RXNE | SPI_IT_ERR  ));    
      }
      else
      {  
        USBPD_TIM_Start(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC), DMA_TIME_ELAPSED);
                                        
        if( (ud->exed_flag == 0) && ((RX_BUFFER_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR)>(ud->index + 2)) )       /* checking if in the buffer there are enough data */
        {
          if (pbuff_in[ud->index]==0xFF)
          {
            __NOP();
          }
          
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
                  preamble_offset = ud->offset;
                  preamble_index = ud->index;
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
              PHY_HW_IF_RX_Stop(PortNum);
             
            }
          }
          
        }
        else
        {
          USBPD_TIM_Start(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC), DMA_TIME_ELAPSED ); 
          
          if (ud->exed_flag == 2)
          { 
            ud->exed_flag = 3 ;
            __HAL_SPI_DISABLE_IT(rx_spi, (SPI_IT_RXNE | SPI_IT_ERR  ));  
            
          }  
        }
      } 
    }
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
  
  HAL_SPIEx_FlushRxFifo(rx_spi);
  
  HAL_NVIC_EnableIRQ(SPI_IRQn(PortNum));
  
  /* Clean-up of buffer storing raw received data */
  memset(Ports[PortNum].pTxRxBuffPtr, 0x00, RX_BUFFER_SIZE);

  USBPD_TIM_Start(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC), DMA_TIME_ELAPSED);
  /* Start DMA receiving */
  HAL_SPI_Receive_DMA(rx_spi, (uint8_t*)Ports[PortNum].pTxRxBuffPtr, RX_BUFFER_SIZE);
  __HAL_SPI_ENABLE_IT(rx_spi, SPI_IT_RXNE);  
  
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
  HAL_DMA_Abort(&Ports[PortNum].hdmarx);
  /* Stop DMA */
  
  __HAL_SPI_DISABLE_IT(rx_spi, (SPI_IT_RXNE | SPI_IT_ERR | SPI_IT_TXE ));    
  HAL_NVIC_DisableIRQ(SPI_IRQn(PortNum)) ; 
  
  /* Set the state of the port */
  Ports[PortNum].State = HAL_USBPD_PORT_STATE_READY;
  if (USBPD_TIM_IsExpired(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC)) )  /* Colision ovoidance */
  {  
    ud->exed_flag = 6; 
    return ;
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
  uint8_t dummyDR;
  uint8_t i ,j;
  USBPD_TIM_Start(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC), DMA_TIME_ELAPSED);
  /* Wait until FIFO is empty */
  if (Ports[PortNum].TxSpareBits == 0)
  {
    while (( (Ports[PortNum].hspi.Instance->SR & SPI_SR_FTLVL) != 0) || !USBPD_TIM_IsExpired(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC)));  /* != 0x0800 */
  }
  else
  {
    while (((((Ports[PortNum].hspi.Instance->SR & SPI_SR_FTLVL) >> SPI_SR_FTLVL_Pos) & 0x03) > 1) || !USBPD_TIM_IsExpired((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC));
  }
  
  /* Wait for BUSY flag */
  do
  {
   j = USBPD_TIM_IsExpired(((PortNum == 0 )? TIM_PORT0_CRC:TIM_PORT1_CRC));
  }
  while (( (Ports[PortNum].hspi.Instance->SR & SPI_SR_BSY) > 0) || (j==0) );
  /* Act on TX spare bits */
  
  /* Cut_1_A */
  for(i=0; i<(Ports[PortNum].TxSpareBits); i++)
  {
    
    /* Wait for SPI CLK GPIO flag RESET */
    while (!HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum)) || (j==0));
    /* Wait for SPI CLK GPIO flag SET */
    while (HAL_GPIO_ReadPin(SPI_CLK_PORT(PortNum), SPI_CLK_PIN(PortNum))|| (j==0));
  }
  
  /* Reset TX_EN GPIO */
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_RESET);
  
  /* Here the SPI has accomplished the transmission*/
  
  /* Disable the selected SPI peripheral */
  __HAL_SPI_DISABLE(&Ports[PortNum].hspi);
  
  /* RX FIFO is cleaned */
  while ((Ports[PortNum].hspi.Instance->SR & SPI_SR_FRLVL) != 0)
  {
    dummyDR= *(__IO uint8_t *)&Ports[PortNum].hspi.Instance->DR;
    UNUSED(dummyDR);
  }
  
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
  
  /* Set the RX mode */
  HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_RX);
  
  /* TX completed callback */
  if (Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted != NULL)
  {
    Ports[PortNum].cbs.USBPD_HW_IF_TxCompleted(PortNum);
  }
}
/**
* @brief  Packet transmission must be aborted due to disconnection 
* @param  PortNum The port index
* @retval none
*/
void PHY_HW_IF_TX_ABORT(uint8_t PortNum)
{
  uint8_t dummyDR;
  
 
  /* Reset TX_EN GPIO */
  STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_RESET);
  
  /* Here the SPI has accomplished the transmission*/
  
  /* Disable the selected SPI peripheral */
  __HAL_SPI_DISABLE(&Ports[PortNum].hspi);
  
  /* RX FIFO is cleaned */
  while ((Ports[PortNum].hspi.Instance->SR & SPI_SR_FRLVL) != 0)
  {
    dummyDR= *(__IO uint8_t *)&Ports[PortNum].hspi.Instance->DR;
    UNUSED(dummyDR);
  }
  
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
    ret = HW_IF_check_bus_idle(PortNum);
  /* Check if the bus is idle */
  
  
  if (ret == USBPD_OK )
  {
    /* Set the state to busy*/
    Ports[PortNum].State = HAL_USBPD_PORT_STATE_BUSY_TX;
    
    HAL_SPI_DMAStop(&Ports[PortNum].hspi);
    __HAL_DMA_CLEAR_FLAG(&Ports[PortNum].hdmatx, __HAL_DMA_GET_GI_FLAG_INDEX(&Ports[PortNum].hdmatx));
    
    /* Set the SPI in TX mode */
    HW_IF_Switch_Mode(PortNum, STUSB16xx_SPI_Mode_TX);
    
    /* Send TX Buffer by SPI DMA */
    HAL_SPI_Transmit_DMA(&Ports[PortNum].hspi, pData, Size);
    
    /* Set TX_EN GPIO */
    STUSB16xx_HW_IF_TX_EN_Status(PortNum, GPIO_PIN_SET);
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


/** @}*/ // End of PRIVATE_FUNCTIONS group


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
  
  preamble_offset = 0;
  preamble_index = 0;
  preamble_counter++;
}


/**
* @brief  It completes data parsing
* @param  PortNum The port index
* @retval None
*/
static inline void HW_IF_RX_CompleteParsingData(uint8_t PortNum)
{
  UnwrapData_TypeDef* ud = &(Ports[PortNum].unwrapdata);
  uint8_t* pbuff_in = Ports[PortNum].pTxRxBuffPtr;
  uint32_t temp_data = 0;
  
  uint16_t lastindex = (RX_BUFFER_SIZE - Ports[PortNum].hdmarx.Instance->CNDTR);
  
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
