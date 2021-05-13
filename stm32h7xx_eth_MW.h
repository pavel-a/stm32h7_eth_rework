/** Based on stm32h7xx_hal_eth.h from STM32H7 HAL lib v.1.10 (Cube H7 package v 1.9.0)
  ******************************************************************************
  * @file    stm32h7xx_eth_MW.h
  * @author
  * @brief   Header file of ETH "middleware" layer on top of the ETH HAL driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32H7xx_YYY_ETH_H
#define STM32H7xx_YYY_ETH_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_eth.h"

#ifndef ETH_TX_DESC_CNT
 #define ETH_TX_DESC_CNT         4U
#endif

#ifndef ETH_RX_DESC_CNT
 #define ETH_RX_DESC_CNT         4U
#endif

/**
  * @brief  Extended ETH DMA Descriptor structure definition
  * NOTE: size of extension is same for RX and TX descriptors
  */

#if (ETH_DESC_EXTRA_SIZE != 8)
#error This software needs 2 extra words in descriptors
#endif

typedef struct
{
  uint32_t DESC0;
  uint32_t DESC1;
  uint32_t DESC2;
  uint32_t DESC3;
  uint32_t BackupAddr0; /* used to store rx buffer 1 address */
  uint32_t BackupAddr1; /* used to store rx buffer 2 address */
} ETHx_DMADescTypeDef;


/**
  * @brief  ETH Buffers List structure definition
  */
typedef struct __ETH_BufferTypeDef
{
  uint8_t *buffer;                /*<! buffer address */

  uint32_t len;                   /*<! buffer length */

  struct __ETH_BufferTypeDef *next; /*<! Pointer to the next buffer in the list */
}ETH_BufferTypeDef;


/**
  * @brief  DMA Transmit Descriptors Wrapper structure definition
  */
typedef struct
{
  ETHx_DMADescTypeDef * TxDesc[ETH_TX_DESC_CNT];        /*<! Tx DMA descriptors addresses */

  uint32_t  CurTxDesc;                      /*<! Current Tx descriptor index for packet transmission */

  uint32_t* PacketAddress[ETH_TX_DESC_CNT];  /*<! Ethernet packet addresses array */

  uint32_t* CurrentPacketAddress;           /*<! Current transmit NX_PACKET addresses */

  uint32_t BuffersInUse;                   /*<! Buffers in Use */
}ETH_TxDescListTypeDef;


/**
 * @brief  DMA Receive Descriptors Wrapper structure definition
 */
typedef struct
{
  ETHx_DMADescTypeDef * RxDesc[ETH_RX_DESC_CNT];     /*<! Rx DMA descriptors addresses. */

  uint32_t CurRxDesc;                   /*<! Current Rx descriptor, ready for next reception. */

  uint32_t FirstAppDesc;                /*<! First descriptor of last received packet. */

  uint32_t AppDescNbr;                  /*<! Number of descriptors of last received packet. */

  uint32_t AppContextDesc;              /*<! If 1 a context descriptor is present in last received packet.
                                             If 0 no context descriptor is present in last received packet. */

  uint32_t ItMode;                      /*<! If 1, DMA will generate the Rx complete interrupt.
                                             If 0, DMA will not generate the Rx complete interrupt. */
}ETH_RxDescListTypeDef;


/* Exported functions --------------------------------------------------------*/

HAL_StatusTypeDef ETHx_DescAssignMemory(ETH_HandleTypeDef *heth, uint32_t Index, uint8_t *pBuffer1,uint8_t *pBuffer2);
uint8_t           ETHx_IsRxDataAvailable(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef ETHx_GetRxDataBuffer(ETH_HandleTypeDef *heth, ETH_BufferTypeDef *RxBuffer);
HAL_StatusTypeDef ETHx_GetRxDataLength(ETH_HandleTypeDef *heth, uint32_t *Length);
HAL_StatusTypeDef ETHx_GetRxDataInfo(ETH_HandleTypeDef *heth, ETH_RxPacketInfo *RxPacketInfo);
HAL_StatusTypeDef ETHx_BuildRxDescriptors(ETH_HandleTypeDef *heth);

HAL_StatusTypeDef ETHx_Transmit(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, ETH_BufferTypeDef *txbuffer, uint32_t Timeout);
HAL_StatusTypeDef ETHx_Transmit_IT(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, ETH_BufferTypeDef *txbuffer);

// Initialize "middleware layer" ++ pa03
HAL_StatusTypeDef ETHx_init(ETH_HandleTypeDef *heth);

#ifdef __cplusplus
}
#endif

#endif /* STM32H7xx_???_ETH_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
