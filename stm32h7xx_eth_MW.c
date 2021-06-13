/** Based on stm32h7xx_hal_eth.c from STM32H7 HAL lib v.1.10 (Cube H7 package v 1.9.0)
  ******************************************************************************
  * @file    stm32h7xx_hal_eth.c
  * @author
  * @brief    ETH "middleware" layer on top of the ETH HAL driver
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
     [..]
      (#) Ethernet data reception is asynchronous, so call the following API
          to start the listening mode:
          (##) HAL_ETH_Start():
               This API starts the MAC and DMA transmission and reception process,
               without enabling end of transfer interrupts, in this mode user
               has to poll for data availability by calling HAL_ETH_IsRxDataAvailable()
          (##) HAL_ETH_Start_IT():
               This API starts the MAC and DMA transmission and reception process,
               end of transfer interrupts are enabled in this mode,
               HAL_ETH_RxCpltCallback() will be executed when an Ethernet packet is received

      (#) When data is received (HAL_ETH_IsRxDataAvailable() returns 1 or Rx interrupt
          occurred), user can call the following APIs to get received data:
          (##) HAL_ETH_GetRxDataBuffer(): Get buffer address of received frame
          (##) HAL_ETH_GetRxDataLength(): Get received frame length
          (##) HAL_ETH_GetRxDataInfo(): Get received frame additional info,
               please refer to ETH_RxPacketInfo typedef structure

      (#) For transmission path, two APIs are available:
         (##) HAL_ETH_Transmit(): Transmit an ETH frame in blocking mode
         (##) HAL_ETH_Transmit_IT(): Transmit an ETH frame in interrupt mode,
              HAL_ETH_TxCpltCallback() will be executed when end of transfer occur
  @endverbatim
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#ifdef HAL_ETH_MODULE_ENABLED
#error Cannot use both this and ST HAL library eth driver. Choose one.
#endif

#include "stm32h7xx_eth_MW.h"

#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup ETH_Private_Macros ETH Private Macros
  * @{
  */
/* Helper macros for TX descriptor handling */
#define INCR_TX_DESC_INDEX(inx, offset) do {\
	(inx) += (offset);\
          if ((inx) >= (uint32_t)ETH_TX_DESC_CNT){\
            (inx) = ((inx) - (uint32_t)ETH_TX_DESC_CNT);}\
} while (0)

/* Helper macros for RX descriptor handling */
#define INCR_RX_DESC_INDEX(inx, offset) do {\
	(inx) += (offset);\
          if ((inx) >= (uint32_t)ETH_RX_DESC_CNT){\
            (inx) = ((inx) - (uint32_t)ETH_RX_DESC_CNT);}\
} while (0)
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/

static uint32_t ETHx_Prepare_Tx_Descriptors(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, ETH_BufferTypeDef *txbuffer, uint32_t ItMode);

// Static data - pa03 TODO put in some context struct -> link to heth
ETH_TxDescListTypeDef g_TxDescList;
ETH_RxDescListTypeDef g_RxDescList;


/**
  * @brief  Assign memory buffers to a DMA Rx descriptor
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  Index : index of the DMA Rx descriptor
  *                  this parameter can be a value from 0x0 to (ETH_RX_DESC_CNT -1)
  * @param  pBuffer1: address of buffer 1
  * @param  pBuffer2: address of buffer 2 if available
  * @retval HAL status
  */
HAL_StatusTypeDef ETHx_DescAssignMemory(ETH_HandleTypeDef *heth, uint32_t Index, uint8_t *pBuffer1, uint8_t *pBuffer2)
{
  ETHx_DMADescTypeDef *dmarxdesc = (ETHx_DMADescTypeDef *)g_RxDescList.RxDesc[Index];

  if((pBuffer1 == NULL) || (Index >= (uint32_t)ETH_RX_DESC_CNT))
  {
    /* Set Error Code */
    heth->ErrorCode = HAL_ETH_ERROR_PARAM;
    /* Return Error */
    return HAL_ERROR;
  }

  /* write buffer address to RDES0 */
  WRITE_REG(dmarxdesc->DESC0, (uint32_t)pBuffer1);
  /* store buffer address */
  WRITE_REG(dmarxdesc->BackupAddr0, (uint32_t)pBuffer1);
  /* set buffer address valid bit to RDES3 */
  SET_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_BUF1V);

  if(pBuffer2 != NULL)
  {
    /* write buffer 2 address to RDES1 */
    WRITE_REG(dmarxdesc->DESC2, (uint32_t)pBuffer2);
     /* store buffer 2 address */
    WRITE_REG(dmarxdesc->BackupAddr1, (uint32_t)pBuffer2);
    /* set buffer 2 address valid bit to RDES3 */
    SET_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_BUF2V);
  }
  /* set OWN bit to RDES3 */
  SET_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_OWN);

  return HAL_OK;
}


/**
  * @brief  Stop Ethernet MAC and DMA reception/transmission in Interrupt mode
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval HAL status
  */
HAL_StatusTypeDef ETHx_Stop_IT(ETH_HandleTypeDef *heth)
{
  ETHx_DMADescTypeDef *dmarxdesc;
  uint32_t descindex;

  HAL_ETH_Stop_IT(heth);

  /* Clear IOC bit to all Rx descriptors */
  for (descindex = 0; descindex < (uint32_t)ETH_RX_DESC_CNT; descindex++)
  {
      dmarxdesc = g_RxDescList.RxDesc[descindex];
      CLEAR_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_IOC);
  }

  g_RxDescList.ItMode = 0U;

  return HAL_OK;
}

/**
  * @brief  Sends an Ethernet Packet in polling mode.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pTxConfig: Hold the configuration of packet to be transmitted
  * @param  txbuffer : TX buffers chain to be transmitted
  * @param  Timeout: timeout value
  * @retval HAL status
  */
HAL_StatusTypeDef ETHx_Transmit(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, ETH_BufferTypeDef *txbuffer, uint32_t Timeout)
{
  uint32_t tickstart;
  const ETHx_DMADescTypeDef *dmatxdesc;

  if(pTxConfig == NULL || txbuffer == NULL)
  {
    heth->ErrorCode |= HAL_ETH_ERROR_PARAM;
    return HAL_ERROR;
  }

  if(heth->gState == HAL_ETH_STATE_READY)
  {
    /* Config DMA Tx descriptor by Tx Packet info */
    /* Assume TX DMA is idle else race can occur! pa01 */
    if (ETHx_Prepare_Tx_Descriptors(heth, pTxConfig, txbuffer, 0) != HAL_ETH_ERROR_NONE)
    {
      /* Set the ETH error code */
      heth->ErrorCode |= HAL_ETH_ERROR_BUSY;
      return HAL_ERROR;
    }

    dmatxdesc = g_TxDescList.TxDesc[g_TxDescList.CurTxDesc];

    /* Incr current tx desc index */
    INCR_TX_DESC_INDEX(g_TxDescList.CurTxDesc, 1U);

    /* Start transmission */
    /* issue a poll command to Tx DMA by writing address of next immediate free descriptor */
    WRITE_REG(heth->Instance->DMACTDTPR, (uint32_t)(g_TxDescList.TxDesc[g_TxDescList.CurTxDesc]));

    tickstart = HAL_GetTick();

    /* Wait for data to be transmitted or timeout occurred */
    while((dmatxdesc->DESC3 & ETH_DMATXNDESCWBF_OWN) != (uint32_t)RESET)
    {
      if((heth->Instance->DMACSR & ETH_DMACSR_FBE) != (uint32_t)RESET)
      {
        heth->ErrorCode |= HAL_ETH_ERROR_DMA;
        heth->DMAErrorCode = heth->Instance->DMACSR;
        /* Set ETH HAL State to Ready */
        heth->gState = HAL_ETH_STATE_ERROR;
        /* Return function status */
        return HAL_ERROR;
      }

      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if(((HAL_GetTick() - tickstart ) > Timeout) || (Timeout == 0U))
        {
          heth->ErrorCode |= HAL_ETH_ERROR_TIMEOUT;
          heth->gState = HAL_ETH_STATE_ERROR;
          return HAL_ERROR;
        }
      }
    }

    /* Return function status */
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Sends an Ethernet Packet in interrupt mode.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pTxConfig: Hold the configuration of packet to be transmitted
  * @retval HAL status
  */
HAL_StatusTypeDef ETHx_Transmit_IT(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, ETH_BufferTypeDef *txbuffer)
{
  if(pTxConfig == NULL)
  {
    heth->ErrorCode |= HAL_ETH_ERROR_PARAM;
    return HAL_ERROR;
  }

  if(heth->gState == HAL_ETH_STATE_READY)
  {
    /* Config DMA Tx descriptor by Tx Packet info */
    /* Assume TX DMA is idle else race can occur! pa01 */
    if (ETHx_Prepare_Tx_Descriptors(heth, pTxConfig, txbuffer, 1) != HAL_ETH_ERROR_NONE)
    {
      heth->ErrorCode |= HAL_ETH_ERROR_BUSY;
      return HAL_ERROR;
    }

    /* Incr current tx desc index */
    INCR_TX_DESC_INDEX(g_TxDescList.CurTxDesc, 1U);

    /* Start transmission */
    /* issue a poll command to Tx DMA by writing address of next immediate free descriptor */
    WRITE_REG(heth->Instance->DMACTDTPR, (uint32_t)(g_TxDescList.TxDesc[g_TxDescList.CurTxDesc]));

    return HAL_OK;

  }
  else
  {
    return HAL_ERROR;
  }
}

/**
  * @brief  Checks for received Packets.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval  1: A Packet is received
  *          0: no Packet received
  */
uint8_t ETHx_IsRxDataAvailable(ETH_HandleTypeDef *heth)
{
  ETH_RxDescListTypeDef *dmarxdesclist = &g_RxDescList;
  uint32_t descidx = dmarxdesclist->CurRxDesc;
  ETHx_DMADescTypeDef *dmarxdesc = dmarxdesclist->RxDesc[descidx];
  uint32_t descscancnt = 0;
  uint32_t appdesccnt = 0, firstappdescidx = 0;

  if(dmarxdesclist->AppDescNbr != 0U)
  {
    /* data already received by not yet processed*/
    return 0;
  }

  /* Check if descriptor is not owned by DMA */
  while((READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_OWN) == (uint32_t)RESET) && (descscancnt < (uint32_t)ETH_RX_DESC_CNT))
  {
    descscancnt++;

    /* Check if last descriptor */
    if(READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_LD) != (uint32_t)RESET)
    {
      /* Increment the number of descriptors to be passed to the application */
      appdesccnt += 1U;

      if(appdesccnt == 1U)
      {
        WRITE_REG(firstappdescidx, descidx);
      }

      /* Increment current rx descriptor index */
      INCR_RX_DESC_INDEX(descidx, 1U);

      /* Check for Context descriptor */
      /* Get current descriptor address */
      dmarxdesc = dmarxdesclist->RxDesc[descidx];

      if(READ_BIT(dmarxdesc->DESC3,  ETH_DMARXNDESCWBF_OWN)  == (uint32_t)RESET)
      {
        if(READ_BIT(dmarxdesc->DESC3,  ETH_DMARXNDESCWBF_CTXT)  != (uint32_t)RESET)
        {
          /* Increment the number of descriptors to be passed to the application */
          dmarxdesclist->AppContextDesc = 1;
          /* Increment current rx descriptor index */
          INCR_RX_DESC_INDEX(descidx, 1U);
        }
      }
      /* Fill information to Rx descriptors list */
      dmarxdesclist->CurRxDesc = descidx;
      dmarxdesclist->FirstAppDesc = firstappdescidx;
      dmarxdesclist->AppDescNbr = appdesccnt;

      /* Return function status */
      return 1;
    }
    /* Check if first descriptor */
    else if(READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_FD) != (uint32_t)RESET)
    {
      WRITE_REG(firstappdescidx, descidx);
      /* Increment the number of descriptors to be passed to the application */
      appdesccnt = 1U;

      /* Increment current rx descriptor index */
      INCR_RX_DESC_INDEX(descidx, 1U);
      /* Get current descriptor address */
      dmarxdesc = dmarxdesclist->RxDesc[descidx];
    }
    /* It should be an intermediate descriptor */
    else
    {
      /* Increment the number of descriptors to be passed to the application */
      appdesccnt += 1U;

      /* Increment current rx descriptor index */
      INCR_RX_DESC_INDEX(descidx, 1U);
      /* Get current descriptor address */
      dmarxdesc = dmarxdesclist->RxDesc[descidx];
    }
  }

  /* Build Descriptors if an incomplete Packet is received */
  if(appdesccnt > 0U)
  {
    dmarxdesclist->CurRxDesc = descidx;
    dmarxdesclist->FirstAppDesc = firstappdescidx;
    descidx = firstappdescidx;
    dmarxdesc = dmarxdesclist->RxDesc[descidx];

    for(descscancnt = 0; descscancnt < appdesccnt; descscancnt++)
    {
      WRITE_REG(dmarxdesc->DESC0, dmarxdesc->BackupAddr0);
      WRITE_REG(dmarxdesc->DESC3, ETH_DMARXNDESCRF_BUF1V);

      if (READ_REG(dmarxdesc->BackupAddr1) != ((uint32_t)RESET))
      {
        WRITE_REG(dmarxdesc->DESC2, dmarxdesc->BackupAddr1);
        SET_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_BUF2V);
      }

      SET_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_OWN);

      if(dmarxdesclist->ItMode != ((uint32_t)RESET))
      {
        SET_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_IOC);
      }
      if(descscancnt < (appdesccnt - 1U))
      {
        /* Increment rx descriptor index */
        INCR_RX_DESC_INDEX(descidx, 1U);
        /* Get descriptor address */
        dmarxdesc = dmarxdesclist->RxDesc[descidx];
      }
    }

    /* Set the Tail pointer address to the last rx descriptor hold by the app */
    WRITE_REG(heth->Instance->DMACRDTPR, (uint32_t)dmarxdesc);
  }

  /* Fill information to Rx descriptors list: No received Packet */
  dmarxdesclist->AppDescNbr = 0U;

  return 0;
}

/**
  * @brief  This function gets the buffer address of last received Packet.
  * @note   Please insure to allocate the RxBuffer structure before calling this function
  *         how to use example:
  *           HAL_ETH_GetRxDataLength(heth, &Length);
  *           BuffersNbr = (Length / heth->Init.RxBuffLen) + 1;
  *           RxBuffer = (ETH_BufferTypeDef *)malloc(BuffersNbr * sizeof(ETH_BufferTypeDef));
  *           HAL_ETH_GetRxDataBuffer(heth, RxBuffer);
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  RxBuffer: Pointer to a ETH_BufferTypeDef structure
  * @retval HAL status
  */
HAL_StatusTypeDef ETHx_GetRxDataBuffer(ETH_HandleTypeDef *heth, ETH_BufferTypeDef *RxBuffer)
{
  ETH_RxDescListTypeDef *dmarxdesclist = &g_RxDescList;
  uint32_t descidx = dmarxdesclist->FirstAppDesc;
  uint32_t index, accumulatedlen = 0, lastdesclen;
  __IO const ETHx_DMADescTypeDef *dmarxdesc = dmarxdesclist->RxDesc[descidx];
  ETH_BufferTypeDef *rxbuff = RxBuffer;

  if(rxbuff == NULL)
  {
    heth->ErrorCode = HAL_ETH_ERROR_PARAM;
    return HAL_ERROR;
  }

  if(dmarxdesclist->AppDescNbr == 0U)
  {
    if(ETHx_IsRxDataAvailable(heth) == 0U)
    {
      /* No data to be transferred to the application */
      return HAL_ERROR;
    }
    else
    {
      descidx = dmarxdesclist->FirstAppDesc;
      dmarxdesc = dmarxdesclist->RxDesc[descidx];
    }
  }

  /* Get intermediate descriptors buffers: in case of the Packet is split into multi descriptors */
  for(index = 0; index < (dmarxdesclist->AppDescNbr - 1U); index++)
  {
    /* Get Address and length of the first buffer address */
    rxbuff->buffer = (uint8_t *) dmarxdesc->BackupAddr0;
    rxbuff->len =  heth->Init.RxBuffLen;

    /* Check if the second buffer address of this descriptor is valid */
    if(dmarxdesc->BackupAddr1 != 0U)
    {
      /* Point to next buffer */
      rxbuff = rxbuff->next;
      /* Get Address and length of the second buffer address */
      rxbuff->buffer = (uint8_t *) dmarxdesc->BackupAddr1;
      rxbuff->len =  heth->Init.RxBuffLen;
    }
    else
    {
      /* Nothing to do here */
    }

    /* get total length until this descriptor */
    accumulatedlen = READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_PL);

    /* Increment to next descriptor */
    INCR_RX_DESC_INDEX(descidx, 1U);
    dmarxdesc = dmarxdesclist->RxDesc[descidx];

    /* Point to next buffer */
    rxbuff = rxbuff->next;
  }

  /* last descriptor data length */
  lastdesclen = READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_PL) - accumulatedlen;

  /* Get Address of the first buffer address */
  rxbuff->buffer = (uint8_t *) dmarxdesc->BackupAddr0;

  /* data is in only one buffer */
  if(lastdesclen <= heth->Init.RxBuffLen)
  {
    rxbuff->len = lastdesclen;
  }
  /* data is in two buffers */
  else if(dmarxdesc->BackupAddr1 != 0U)
  {
    /* Get the Length of the first buffer address */
    rxbuff->len = heth->Init.RxBuffLen;
    /* Point to next buffer */
    rxbuff = rxbuff->next;
    /* Get the Address the Length of the second buffer address */
    rxbuff->buffer = (uint8_t *) dmarxdesc->BackupAddr1;
    rxbuff->len =  lastdesclen - (heth->Init.RxBuffLen);
  }
  else /* Buffer 2 not valid*/
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  This function gets the length of last received Packet.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  Length: parameter to hold Rx packet length
  * @retval HAL Status
  */
HAL_StatusTypeDef ETHx_GetRxDataLength(ETH_HandleTypeDef *heth, uint32_t *Length)
{
  ETH_RxDescListTypeDef *dmarxdesclist = &g_RxDescList;
  uint32_t descidx = dmarxdesclist->FirstAppDesc;
  __IO const ETH_DMADescTypeDef *dmarxdesc;

  if(dmarxdesclist->AppDescNbr == 0U)
  {
    if(ETHx_IsRxDataAvailable(heth) == 0U)
    {
      /* No data to be transferred to the application */
      return HAL_ERROR;
    }
  }

  /* Get index of last descriptor */
  INCR_RX_DESC_INDEX(descidx, (dmarxdesclist->AppDescNbr - 1U));
  /* Point to last descriptor */
  dmarxdesc = (ETH_DMADescTypeDef *)dmarxdesclist->RxDesc[descidx];

  *Length = READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_PL);

  return HAL_OK;
}

/**
  * @brief  Get the Rx data info (Packet type, VLAN tag, Filters status, ...)
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  RxPacketInfo: parameter to hold info of received buffer
  * @retval HAL status
  */
HAL_StatusTypeDef ETHx_GetRxDataInfo(ETH_HandleTypeDef *heth, ETH_RxPacketInfo *RxPacketInfo)
{
  ETH_RxDescListTypeDef *dmarxdesclist = &g_RxDescList;
  uint32_t descidx = dmarxdesclist->FirstAppDesc;
  __IO const ETHx_DMADescTypeDef *dmarxdesc;

  if(dmarxdesclist->AppDescNbr == 0U)
  {
    if(ETHx_IsRxDataAvailable(heth) == 0U)
    {
      /* No data to be transferred to the application */
      return HAL_ERROR;
    }
  }

  /* Get index of last descriptor */
  INCR_RX_DESC_INDEX(descidx, ((dmarxdesclist->AppDescNbr) - 1U));
  /* Point to last descriptor */
  dmarxdesc = dmarxdesclist->RxDesc[descidx];

  if((dmarxdesc->DESC3 & ETH_DMARXNDESCWBF_ES) != (uint32_t)RESET)
  {
    RxPacketInfo->ErrorCode = READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_ERRORS_MASK);
  }
  else
  {
    if(READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_RS0V) != 0U)
    {

      if(READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_LT) == ETH_DMARXNDESCWBF_LT_DVLAN)
      {
        RxPacketInfo->VlanTag = READ_BIT(dmarxdesc->DESC0, ETH_DMARXNDESCWBF_OVT);
        RxPacketInfo->InnerVlanTag = READ_BIT(dmarxdesc->DESC0, ETH_DMARXNDESCWBF_IVT) >> 16;
      }
      else
      {
        RxPacketInfo->VlanTag = READ_BIT(dmarxdesc->DESC0, ETH_DMARXNDESCWBF_OVT);
      }
    }

    if(READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_RS1V) != 0U)
    {
      /* Get Payload type */
      RxPacketInfo->PayloadType =READ_BIT( dmarxdesc->DESC1, ETH_DMARXNDESCWBF_PT);
      /* Get Header type */
      RxPacketInfo->HeaderType = READ_BIT(dmarxdesc->DESC1, (ETH_DMARXNDESCWBF_IPV4 | ETH_DMARXNDESCWBF_IPV6));
      /* Get Checksum status */
      RxPacketInfo->Checksum = READ_BIT(dmarxdesc->DESC1, (ETH_DMARXNDESCWBF_IPCE | ETH_DMARXNDESCWBF_IPCB | ETH_DMARXNDESCWBF_IPHE));
    }

    if(READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_RS2V) != 0U)
    {
      RxPacketInfo->MacFilterStatus = READ_BIT(dmarxdesc->DESC2, (ETH_DMARXNDESCWBF_HF | ETH_DMARXNDESCWBF_DAF | ETH_DMARXNDESCWBF_SAF | ETH_DMARXNDESCWBF_VF));
      RxPacketInfo->L3FilterStatus = READ_BIT(dmarxdesc->DESC2,  (ETH_DMARXNDESCWBF_L3FM | ETH_DMARXNDESCWBF_L3L4FM));
      RxPacketInfo->L4FilterStatus = READ_BIT(dmarxdesc->DESC2, (ETH_DMARXNDESCWBF_L4FM | ETH_DMARXNDESCWBF_L3L4FM));
    }
  }

  /* Get the segment count */
  WRITE_REG(RxPacketInfo->SegmentCnt, dmarxdesclist->AppDescNbr);

  return HAL_OK;
}

/**
* @brief  This function gives back Rx Desc of the last received Packet
*         to the DMA, so ETH DMA will be able to use these descriptors
*         to receive next Packets.
*         It should be called after processing the received Packet.
* @param  heth: pointer to a ETH_HandleTypeDef structure that contains
*         the configuration information for ETHERNET module
* @retval HAL status.
*/
HAL_StatusTypeDef ETHx_BuildRxDescriptors(ETH_HandleTypeDef *heth)
{
  ETH_RxDescListTypeDef *dmarxdesclist = &g_RxDescList;
  uint32_t descindex = dmarxdesclist->FirstAppDesc;
  __IO ETHx_DMADescTypeDef *dmarxdesc = dmarxdesclist->RxDesc[descindex];
  uint32_t totalappdescnbr = dmarxdesclist->AppDescNbr;
  uint32_t descscan;

  if(dmarxdesclist->AppDescNbr == 0U)
  {
    /* No Rx descriptors to build */
    return HAL_ERROR;
  }

  if(dmarxdesclist->AppContextDesc != 0U)
  {
    /* A context descriptor is available */
    totalappdescnbr += 1U;
  }

  for(descscan =0; descscan < totalappdescnbr; descscan++)
  {
    WRITE_REG(dmarxdesc->DESC0, dmarxdesc->BackupAddr0);
    WRITE_REG(dmarxdesc->DESC3, ETH_DMARXNDESCRF_BUF1V);

    if (READ_REG(dmarxdesc->BackupAddr1) != 0U)
    {
      WRITE_REG(dmarxdesc->DESC2, dmarxdesc->BackupAddr1);
      SET_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_BUF2V);
    }

    SET_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_OWN);

    if(dmarxdesclist->ItMode != 0U)
    {
      SET_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCRF_IOC);
    }

    if(descscan < (totalappdescnbr - 1U))
    {
      /* Increment rx descriptor index */
      INCR_RX_DESC_INDEX(descindex, 1U);
      /* Get descriptor address */
      dmarxdesc = dmarxdesclist->RxDesc[descindex];
    }
  }

  /* Set the Tail pointer address to the last rx descriptor hold by the app */
  WRITE_REG(heth->Instance->DMACRDTPR, (uint32_t)dmarxdesc);

  /* reset the Application desc number */
  WRITE_REG(dmarxdesclist->AppDescNbr, 0);

  /*  reset the application context descriptor */
  WRITE_REG(g_RxDescList.AppContextDesc, 0);

  return HAL_OK;
}

/**
  * @brief  Prepare Tx DMA descriptor before transmission.
  *         called by HAL_ETH_Transmit_IT and HAL_ETH_Transmit_IT() API.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @param  pTxConfig: Tx packet configuration
  * @param  ItMode: Enable or disable Tx EOT interrupt
  * @retval Status
  * NOTE: assuming TX DMA is not running, else race! - pa01
  * NOTE: the idea after conversion is that pTxConfig is immutable; all per-TX params passed separately - pa01
  */
static uint32_t ETHx_Prepare_Tx_Descriptors(ETH_HandleTypeDef *heth, ETH_TxPacketConfig *pTxConfig, ETH_BufferTypeDef *txbuffer, uint32_t ItMode)
{
  ETH_TxDescListTypeDef *dmatxdesclist = &g_TxDescList;
  uint32_t descidx = dmatxdesclist->CurTxDesc;
  uint32_t firstdescidx = dmatxdesclist->CurTxDesc;
  uint32_t descnbr = 0, idx;
  ETH_DMADescTypeDef *dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[descidx];
  uint32_t           bd_count = 0;
  uint32_t framelen = 0; // total frame len +pa01

  /* Current Tx Descriptor Owned by DMA: cannot be used by the application  */
  if((READ_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCWBF_OWN) == ETH_DMATXNDESCWBF_OWN) || (dmatxdesclist->PacketAddress[descidx] != NULL))
  {
    return HAL_ETH_ERROR_BUSY;
  }

  /***************************************************************************/
  /*****************    Context descriptor configuration (Optional) **********/
  /***************************************************************************/
  /* If VLAN tag is enabled for this packet */
  if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_VLANTAG) != 0U)
  {
    /* Set vlan tag value */
    MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXCDESC_VT, pTxConfig->VlanTag);
    /* Set vlan tag valid bit */
    SET_BIT(dmatxdesc->DESC3, ETH_DMATXCDESC_VLTV);
    /* Set the descriptor as the vlan input source */
    SET_BIT(heth->Instance->MACVIR, ETH_MACVIR_VLTI);

    /* if inner VLAN is enabled */
    if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_INNERVLANTAG) != 0U)
    {
      /* Set inner vlan tag value */
      MODIFY_REG(dmatxdesc->DESC2, ETH_DMATXCDESC_IVT, (pTxConfig->InnerVlanTag << 16));
      /* Set inner vlan tag valid bit */
      SET_BIT(dmatxdesc->DESC3, ETH_DMATXCDESC_IVLTV);

      /* Set Vlan Tag control */
      MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXCDESC_IVTIR, pTxConfig->InnerVlanCtrl);

      /* Set the descriptor as the inner vlan input source */
      SET_BIT(heth->Instance->MACIVIR, ETH_MACIVIR_VLTI);
      /* Enable double VLAN processing */
      SET_BIT(heth->Instance->MACVTR, ETH_MACVTR_EDVLP);
    }
  }

  /* if tcp segmentation is enabled for this packet */
  if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_TSO) != 0U)
  {
    /* Set MSS value */
    MODIFY_REG(dmatxdesc->DESC2, ETH_DMATXCDESC_MSS, pTxConfig->MaxSegmentSize);
    /* Set MSS valid bit */
    SET_BIT(dmatxdesc->DESC3, ETH_DMATXCDESC_TCMSSV);
  }

  if((READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_VLANTAG) != 0U)|| (READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_TSO) != 0U))
  {
    /* Set as context descriptor */
    SET_BIT(dmatxdesc->DESC3, ETH_DMATXCDESC_CTXT);
    /* Set own bit */
    SET_BIT(dmatxdesc->DESC3, ETH_DMATXCDESC_OWN);
    /* Increment current tx descriptor index */
    INCR_TX_DESC_INDEX(descidx, 1U);
    /* Get current descriptor address */
    dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[descidx];

    descnbr += 1U;

    /* Current Tx Descriptor Owned by DMA: cannot be used by the application  */
    if(READ_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCWBF_OWN) == ETH_DMATXNDESCWBF_OWN)
    {
      dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[firstdescidx];
      /* Clear own bit */
      CLEAR_BIT(dmatxdesc->DESC3, ETH_DMATXCDESC_OWN);

      return HAL_ETH_ERROR_BUSY;
    }
  }

  /***************************************************************************/
  /*****************    Normal descriptors configuration     *****************/
  /***************************************************************************/

  descnbr += 1U;

  /* Set header or buffer 1 address */
  WRITE_REG(dmatxdesc->DESC0, (uint32_t)txbuffer->buffer);
  /* Set header or buffer 1 Length */
  MODIFY_REG(dmatxdesc->DESC2, ETH_DMATXNDESCRF_B1L, txbuffer->len);
  framelen += txbuffer->len;

  if(txbuffer->next != NULL)
  {
    txbuffer = txbuffer->next;
    /* Set buffer 2 address */
    WRITE_REG(dmatxdesc->DESC1, (uint32_t)txbuffer->buffer);
    /* Set buffer 2 Length */
    MODIFY_REG(dmatxdesc->DESC2, ETH_DMATXNDESCRF_B2L, (txbuffer->len << 16));
    framelen += txbuffer->len;
    if (txbuffer->next != NULL) {
        // More than 2 buffers not supported yet see below! -pa01
        __BKPT(0x40);
        return HAL_ETH_ERROR_PARAM;
    }
  }
  else
  {
    WRITE_REG(dmatxdesc->DESC1, 0x0);
    /* Set buffer 2 Length */
    MODIFY_REG(dmatxdesc->DESC2, ETH_DMATXNDESCRF_B2L, 0x0U);
  }

  if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_TSO) != 0U)
  {
    /* Set TCP Header length */
    MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXNDESCRF_THL, (pTxConfig->TCPHeaderLen << 19));
    /* Set TCP payload length */
    MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXNDESCRF_TPL, pTxConfig->PayloadLen); //pa01 ??PayloadLen
    /* Set TCP Segmentation Enabled bit */
    SET_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_TSE);
  }
  else
  {
    MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXNDESCRF_FL, framelen /*pTxConfig->Length*/);

    if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_CSUM) != 0U)
    {
      MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXNDESCRF_CIC, pTxConfig->ChecksumCtrl);
    }

    if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_CRCPAD) != 0U)
    {
      MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXNDESCRF_CPC, pTxConfig->CRCPadCtrl);
    }
  }

  if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_VLANTAG) != 0U)
  {
    /* Set Vlan Tag control */
    MODIFY_REG(dmatxdesc->DESC2, ETH_DMATXNDESCRF_VTIR, pTxConfig->VlanCtrl);
  }

  /* Mark it as First Descriptor */
  SET_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_FD);
  /* Mark it as NORMAL descriptor */
  CLEAR_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_CTXT);
  /* set OWN bit of FIRST descriptor */
  SET_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_OWN);

  /* If source address insertion/replacement is enabled for this packet */
  //$$$$ BUGBUG revise! OWN bit is already set here! pa01
  if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_SAIC) != 0U)
  {
    MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXNDESCRF_SAIC, pTxConfig->SrcAddrCtrl);
  }

  /* only if the packet is split into more than one descriptors > 1 */
  //$$$$ BUGBUG revise! OWN bit is already set here! pa01
  while (txbuffer->next != NULL)
  {
    /* Clear the LD bit of previous descriptor */
    CLEAR_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_LD);
    /* Increment current tx descriptor index */
    INCR_TX_DESC_INDEX(descidx, 1U);
    /* Get current descriptor address */
    dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[descidx];

    /* Clear the FD bit of new Descriptor */
    CLEAR_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_FD);

    /* Current Tx Descriptor Owned by DMA: cannot be used by the application  */
    if((READ_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_OWN) == ETH_DMATXNDESCRF_OWN) || (dmatxdesclist->PacketAddress[descidx] != NULL))
    {
      descidx = firstdescidx;
      dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[descidx];

      /* clear previous desc own bit */
      for(idx = 0; idx < descnbr; idx ++)
      {
        CLEAR_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_OWN);

        /* Increment current tx descriptor index */
        INCR_TX_DESC_INDEX(descidx, 1U);
        /* Get current descriptor address */
        dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[descidx];
      }

      return HAL_ETH_ERROR_BUSY;
    }

    descnbr += 1U;

    /* Get the next Tx buffer in the list */
    txbuffer = txbuffer->next;

    /* Set header or buffer 1 address */
    WRITE_REG(dmatxdesc->DESC0, (uint32_t)txbuffer->buffer);
    /* Set header or buffer 1 Length */
    MODIFY_REG(dmatxdesc->DESC2, ETH_DMATXNDESCRF_B1L, txbuffer->len);
    framelen += txbuffer->len;

    if (txbuffer->next != NULL)
    {
      /* Get the next Tx buffer in the list */
      txbuffer = txbuffer->next;
      /* Set buffer 2 address */
      WRITE_REG(dmatxdesc->DESC1, (uint32_t)txbuffer->buffer);
      /* Set buffer 2 Length */
      MODIFY_REG(dmatxdesc->DESC2, ETH_DMATXNDESCRF_B2L, (txbuffer->len << 16));
      framelen += txbuffer->len;
    }
    else
    {
      WRITE_REG(dmatxdesc->DESC1, 0x0);
      /* Set buffer 2 Length */
      MODIFY_REG(dmatxdesc->DESC2, ETH_DMATXNDESCRF_B2L, 0x0U);
    }

    if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_TSO) != 0U)
    {
      /* Set TCP payload length */
      MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXNDESCRF_TPL, pTxConfig->PayloadLen); //pa01 ??PayloadLen
      /* Set TCP Segmentation Enabled bit */
      SET_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_TSE);
    }
    else
    {
      /* Set the packet length */
      MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXNDESCRF_FL, framelen /*pTxConfig->Length*/);

      if(READ_BIT(pTxConfig->Attributes, ETH_TX_PACKETS_FEATURES_CSUM) != 0U)
      {
        /* Checksum Insertion Control */
        MODIFY_REG(dmatxdesc->DESC3, ETH_DMATXNDESCRF_CIC, pTxConfig->ChecksumCtrl);
      }
    }

    bd_count += 1U;
    /* Set Own bit */
    SET_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_OWN);
    /* Mark it as NORMAL descriptor */
    CLEAR_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_CTXT);
  }

  if(ItMode != ((uint32_t)RESET))
  {
    /* Set Interrupt on completion bit */
    SET_BIT(dmatxdesc->DESC2, ETH_DMATXNDESCRF_IOC);
  }
  else
  {
    /* Clear Interrupt on completion bit */
    CLEAR_BIT(dmatxdesc->DESC2, ETH_DMATXNDESCRF_IOC);
  }

  /* Mark it as LAST descriptor */
  SET_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_LD);
  /* Save the current packet address to expose it to the application */
  dmatxdesclist->PacketAddress[descidx] = dmatxdesclist->CurrentPacketAddress;

  dmatxdesclist->CurTxDesc = descidx;

  /* disable the interrupt */
  __disable_irq();

  dmatxdesclist->BuffersInUse += bd_count + 1U;

  /* Enable interrupts back */
  __enable_irq();


  /* Return function status */
  return HAL_ETH_ERROR_NONE;
}


static void RxDescListInit_(ETH_DMADescTypeDef *a_dmarxdesc, unsigned cnt)
{
    // Factored out from ETH_DMARxDescListInit - pa01
    for (unsigned i = 0; i < cnt; i++)
    {
      ETHx_DMADescTypeDef *dmarxdesc = (ETHx_DMADescTypeDef*)&a_dmarxdesc[i];
      memset(dmarxdesc, 0, sizeof(ETHx_DMADescTypeDef));
      g_RxDescList.RxDesc[i] = dmarxdesc;
    }

    g_RxDescList.CurRxDesc = 0;
    g_RxDescList.FirstAppDesc = 0;
    g_RxDescList.AppDescNbr = 0;
    g_RxDescList.ItMode = 0;
    g_RxDescList.AppContextDesc = 0;
}


static void TxDescListInit_(ETH_DMADescTypeDef *a_dmatxdesc, unsigned cnt)
{
    // Factored out from ETH_DMATxDescListInit - pa01
    for (unsigned i = 0; i < cnt; i++)
    {
      ETHx_DMADescTypeDef *dmatxdesc = (ETHx_DMADescTypeDef*)&a_dmatxdesc[i];
      memset(dmatxdesc, 0, sizeof(ETHx_DMADescTypeDef));
      g_TxDescList.TxDesc[i] = dmatxdesc;
    }

    g_TxDescList.CurTxDesc = 0;
}

// pa03 Initialize "middleware" layer and call HAL_ETH_Init
// Caller should fill other heth->Init fields! TODO revise
HAL_StatusTypeDef ETHx_init(ETH_HandleTypeDef *heth)
{
    heth->Init.RxDescCnt = ETH_RX_DESC_CNT;
    heth->Init.TxDescCnt = ETH_TX_DESC_CNT;
    heth->Init.RxBuffLen = ETH_RX_BUFFER_SIZE;

    // Init g_RxDescList, g_TxDescList
    TxDescListInit_(heth->Init.TxDesc, heth->Init.TxDescCnt);
    RxDescListInit_(heth->Init.RxDesc, heth->Init.RxDescCnt);

    /* Configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
    return HAL_ETH_Init(heth);
}
