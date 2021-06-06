/**
  ******************************************************************************
  * @file    ethernetif.c
  * @brief   This file implements Ethernet network interface for lwIP
  *
  * Variant for polling with no RTOS. Not using interrupts.
  * PHY: LAN8742 as on STM32 Nucleo and other eval boards
  * pa02 TODO: adapt to actual PHY
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_eth_MW.h" //+pa03
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include <string.h>
#include "debug.h"
#include "BSP_MPU_defs.h"
#include "../Components/lan8742/lan8742.h" /* PHY on EVB! */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

#ifndef ETH_RX_BUFFER_SIZE
#define ETH_RX_BUFFER_SIZE                     (1536UL)
#endif

#define ETH_DMA_TRANSMIT_TIMEOUT                (20U)

#if 0
uint8_t macaddress[6]= {ETH_MAC_ADDR0, ETH_MAC_ADDR1, ETH_MAC_ADDR2, ETH_MAC_ADDR3, ETH_MAC_ADDR4, ETH_MAC_ADDR5};
#endif
extern uint8_t g_MAC_addr[6];

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

unsigned g_RX_drop_cnt; // count of RX dropped packets, dbg

/* 
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx buffers are allocated statically and passed directly to the LwIP stack
          they will return back to DMA after been processed by the stack.
        - Tx Buffers will be allocated from LwIP stack memory heap, 
          then passed to ETH HAL driver.

@Notes: 
  1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4, 
       to customize it please redefine ETH_RX_DESC_CNT in stm32xxxx_hal_conf.h
  1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4, 
       to customize it please redefine ETH_TX_DESC_CNT in stm32xxxx_hal_conf.h

  2.a. Rx Buffers number must be between ETH_RX_DESC_CNT and 2*ETH_RX_DESC_CNT
  2.b. Rx Buffers must have the same size: ETH_RX_BUFFER_SIZE, this value must
       passed to ETH DMA in the init field (EthHandle.Init.RxBuffLen)
*/

#if defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_RX_BUFFER_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */
#else
#error "Revise for other compilers"
#endif

ETH_HandleTypeDef EthHandle;
ETH_TxPacketConfig TxConfig; 

lan8742_Object_t LAN8742; // PHY on EVB ***

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config_for_ETH(void);
u32_t sys_now(void);
void pbuf_free_custom(struct pbuf *p);

// PHY interface:
int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit (void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);


lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_IO_Init,
                               ETH_PHY_IO_DeInit,
                               ETH_PHY_IO_WriteReg,
                               ETH_PHY_IO_ReadReg,
                               ETH_PHY_IO_GetTick};

LWIP_MEMPOOL_DECLARE(RX_POOL, 10, sizeof(struct pbuf_custom), "Zero-copy RX PBUF pool");

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
                       Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
  * @brief In this function, the hardware should be initialized.
  * Called from ethernetif_init().
  *
  * @param netif the already initialized lwip network interface structure
  *        for this ethernetif
  */
static err_enum_t low_level_init(struct netif *netif, const uint8_t *macaddress)
{ 

#if defined(STM32H7_ETH_TREGO_PATCH)
#  if STM32H7_ETH_TREGO_PATCH == 1
    dbgprintf("ETH: Trego patch 1\n");
#  else
#    error "unsupported"
#  endif
#else
    dbgprintf("ETH: ST driver unmodified\n");
#endif

  static bool init_done = false;
  if (init_done) {
      dbgprintf("Repeated init not supported yet, restart!\n");
      return ERR_ALREADY; //TODO restart lwip? take the netif down/up?
  }
  init_done = true;

  uint32_t idx = 0;
  STATIC_ASSERT(6==ETH_HWADDR_LEN, "sanity...");
  STATIC_ASSERT(0 == (ETH_RX_BUFFER_SIZE % 32), "buffers size must be multiple of cache line");
  
  // Configure memory areas used by ETH DMA...
  MPU_Config_for_ETH();

  EthHandle.Instance = ETH;
  EthHandle.Init.MACAddr = (void*)macaddress;
  EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
  EthHandle.Init.RxDesc = DMARxDscrTab;
  EthHandle.Init.TxDesc = DMATxDscrTab;
  EthHandle.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;
  
  //pa03 Initialize "middleware" layer:
  HAL_StatusTypeDef st = ETHx_init(&EthHandle);
  if (st != HAL_OK)
  {
      dbgprintf("ETHx: Failed init!\n");
      return ERR_IF;
  }
  
  /* set MAC hardware address */
  STATIC_ASSERT(sizeof(netif->hwaddr) >= ETH_HWADDR_LEN, "sanity");
  netif->hwaddr_len = ETH_HWADDR_LEN;
  printf("ETHERNET: MAC Address ");
  for (idx = 0; idx < ETH_HWADDR_LEN; idx++)
  {
      netif->hwaddr[idx] = macaddress[idx];
      printf("%02x, ", macaddress[idx]);
  }
  printf("\n");

  /* maximum transfer unit */
  netif->mtu = ETH_MAX_PAYLOAD;  // use standard eth packet size, no jumbo
  STATIC_ASSERT((ETH_MAX_PAYLOAD + 2*ETH_HWADDR_LEN + 4) <= ETH_RX_BUFFER_SIZE, "sanity");

  /* NETIF_FLAG_ETHARP tells LwIP to use ARP */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
  
  /* Initialize RX buffers, descriptors */
  for (idx = 0; idx < ETH_RX_DESC_CNT; idx++)
  {
      ETHx_DescAssignMemory(&EthHandle, idx, Rx_Buff[idx], NULL);
  }
  
  /* Initialize LwIP RX POOL */
  LWIP_MEMPOOL_INIT(RX_POOL);
  
  /* Set Tx parameters */
  /* Enable L2 CRC & padding offload. */
  memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  // L3 cksm offload - must correspond to LwIP config
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  
  /* Set PHY IO functions */
  LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
  
  /* Initialize the LAN8742 ETH PHY */
  int32_t phy_st =
  LAN8742_Init(&LAN8742);
  if (phy_st < 0)
  {
      dbgprintf("ETH: Failed PHY init! err=%"PRId32"\n", phy_st);
      return ERR_IF;
  }
  
  ethernet_link_check_state(netif);

  return ERR_OK;
}



/**
  * @brief  Configure the MPU
  * @param  None
  * @retval None
  * @note Physical addresses for regions below must be defined in the link script!
  */
static void MPU_Config_for_ETH(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU before config*/
  HAL_MPU_Disable();

  dbgprintf("ETH: #TX desc=%u, total size=%u\n", ETH_TX_DESC_CNT, ETH_TX_DESC_CNT*sizeof(ETH_DMADescTypeDef));
  dbgprintf("ETH: #RX desc=%u, total size=%u\n", ETH_RX_DESC_CNT, ETH_RX_DESC_CNT*sizeof(ETH_DMADescTypeDef));
  //! TODO STATIC_ASSERT(0 == (sizeof(ETH_DMADescTypeDef) % 32), "size of descriptor must be multiple of cacheline size");
  STATIC_ASSERT(0 == (ETH_RX_BUFFER_SIZE % 32), "size of RX buf must be multiple of cacheline size");
  // TX buffers are owned by LwIP; also must be multiple of cacheline size!

  //NOTE: 2 MPU regions modified per this KB article:
  // https://community.st.com/s/article/How-to-create-project-for-STM32H7-with-Ethernet-and-LwIP-stack-working
  // Region N = 32KB at 0x30040000 (~16KB for RX, ~16 KB 0x30044000 for TX, LwIP pool )
  // Region N+1 overlapping over N : 256 bytes for DMA descriptors
  static const uint8_t reg_start_n = MPU_REGION_NUMBER0;

  MPU_InitStruct.Number = reg_start_n;
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  //MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  //MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  //MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  //MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_set_Normal_Noncached(&MPU_InitStruct);
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Configure the MPU attributes as Device not cacheable
     for ETH DMA descriptors */
  MPU_InitStruct.Number = reg_start_n + 1;
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30040000;  // <<< TODO check the address, size against ^^^
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  STATIC_ASSERT(256 >= (ETH_TX_DESC_CNT+ETH_RX_DESC_CNT)*sizeof(ETH_DMADescTypeDef), "increase MPU region size");
  //MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  //MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  //MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  //MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_set_SharedDevice(&MPU_InitStruct);
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}


/**
 * Flush data cache in specified range
 * @param p    - address, may be not cache aligned
 * @param len  - length in bytes
 * For TX: flush cache. For RX: invalidate cache
 */
static inline
void flush_cache_(bool is_TX, void *ptr, unsigned len)
{
    if (!ptr || (len == 0))
        __BKPT(0);
#if 0 //pa05 -- cache NOT enabled. SCB_invalidate causes fault!
    // Align the pointer down and size up.
    // Cache line size for M7 is 32, not defined in ST CMSIS header yet!
    uint8_t a = (uintptr_t)ptr & (32-1);
    if (a) {
        ptr = (void*)((uintptr_t)ptr - a);
        len += a; // CMSIS v.5 does this but v4 not!
    }

    if (is_TX) {
        // ptr, len may be not cache line aligned: see impl. of SCB_CleanDCache_by_Addr
        SCB_CleanDCache_by_Addr(ptr, len);
    } else {
        // ptr, len may be not cache line aligned: see impl. of SCB_InvalidateDCache_by_Addr
        SCB_InvalidateDCache_by_Addr(ptr, len);
    }
#endif
}


/**
  * @brief This function should do the actual transmission of the packet. The packet is
  * contained in the pbuf that is passed to the function. This pbuf
  * might be chained.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
  * @return ERR_OK if the packet could be sent
  *         an err_t value if the packet couldn't be sent
  *
  * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
  *       strange results. You might consider waiting for space in the DMA queue
  *       to become available since the stack doesn't retry to send a packet
  *       dropped because of memory failure (except for the TCP timers).
  */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  uint32_t i=0, framelen = 0;
  struct pbuf *q;
  err_t errval = ERR_OK;
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];
  
  memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));
  
  for(q = p; q != NULL; q = q->next)
  {
    if (i >= ETH_TX_DESC_CNT)
    {
      return ERR_IF;
    }
    
    Txbuffer[i].buffer = q->payload;
    Txbuffer[i].len = q->len;
    framelen += q->len;
    
    if(i>0)
    {
      Txbuffer[i-1].next = &Txbuffer[i];
    }
    
    if(q->next == NULL)
    {
      Txbuffer[i].next = NULL;
    }
    
    i++;
  }

  TxConfig.Length = framelen;
  
  HAL_StatusTypeDef st =
  ETHx_Transmit(&EthHandle, &TxConfig, Txbuffer, ETH_DMA_TRANSMIT_TIMEOUT);
  if (st != HAL_OK) {
      dbgprintf("ETH: TX error!\n");
  }
  
  return errval;
}

/**
  * @brief Should allocate a pbuf and transfer the bytes of the incoming
  * packet from the interface into the pbuf.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return a pbuf filled with the received packet (including MAC header)
  *         NULL on memory error
  */
static struct pbuf * low_level_input(struct netif *netif)
{
#if defined(STM32H7_ETH_TREGO_PATCH)
#  if STM32H7_ETH_TREGO_PATCH == 1
    struct pbuf *p = NULL;
    ETH_BufferTypeDef RxBuff;
    uint32_t framelength = 0;
    struct pbuf_custom* custom_pbuf;

    HAL_StatusTypeDef st = HAL_ETH_GetRxDataBuffer(&EthHandle, &RxBuff);
    if (st != HAL_OK)
    {
        return NULL;
    }

    framelength = RxBuff.len;
    if (framelength == 0) {
        dbgprintf("ETH: RX len=0 ??\n");
        return NULL;
    }

    /* Invalidate data cache for ETH Rx Buffers. Assume buffers are DCACHE_LINE_SIZE aligned  */
    flush_cache_(false, RxBuff.buffer, framelength);

    custom_pbuf  = (struct pbuf_custom*)LWIP_MEMPOOL_ALLOC(RX_POOL);
    if (!custom_pbuf) {
        //__BKPT(1);
        ++g_RX_drop_cnt;
        dbgprintf("ETH RX drop %u, len %u\n", g_RX_drop_cnt, (unsigned)framelength);
        return NULL;
    }
    custom_pbuf->custom_free_function = pbuf_free_custom;

    p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_REF, custom_pbuf, RxBuff.buffer, ETH_RX_BUFFER_SIZE);

    return p;
#  else
#    error
#  endif
#else /* no patch */
  struct pbuf *p = NULL;
  ETH_BufferTypeDef RxBuff;
  uint32_t framelength = 0;
  struct pbuf_custom* custom_pbuf;
  
  if (ETHx_IsRxDataAvailable(&EthHandle))
  {
    ETHx_GetRxDataBuffer(&EthHandle, &RxBuff);
    ETHx_GetRxDataLength(&EthHandle, &framelength);
    
    /* Build Rx descriptor to be ready for next data reception */
    ETHx_BuildRxDescriptors(&EthHandle);

    /* Invalidate data cache for ETH Rx Buffers */
    flush_cache_(false, RxBuff.buffer, framelength);

    custom_pbuf  = (struct pbuf_custom*)LWIP_MEMPOOL_ALLOC(RX_POOL);
    if (!custom_pbuf) {
        //Packet dropped because no memory
        //__BKPT(1);
        ++g_RX_drop_cnt;
        dbgprintf("ETH RX drop %u, len %u\n", g_RX_drop_cnt, (unsigned)framelength);
        return NULL;
    }
    custom_pbuf->custom_free_function = pbuf_free_custom;

    p = pbuf_alloced_custom(PBUF_RAW, framelength, PBUF_REF, custom_pbuf, RxBuff.buffer, ETH_RX_BUFFER_SIZE);
    return p;
  }
  return NULL;
#endif
}

/**
  * @brief This function is the ethernetif_input task, it is processed when a packet 
  * is ready to be read from the interface. It uses the function low_level_input() 
  * that should handle the actual reception of bytes from the network
  * interface. Then the type of the received packet is determined and
  * the appropriate input function is called.
  *
  * @param netif the lwip network interface structure for this ethernetif
  */
void ethernetif_input(struct netif *netif)
{
  err_t err;
  struct pbuf *p;
  
  /* Try to get received packet into a new pbuf */
  p = low_level_input(netif);
    
  /* If no input packet is available return */
  if (p == NULL) return;
    
  /* Pass it to the LwIP stack */
  err = netif->input(p, netif);
    
  if (err != ERR_OK)
  {
    dbgprintf("ETH: IP input error %d\n", err);
    pbuf_free(p);
    p = NULL;
  }
}

/**
  * @brief Should be called at the beginning of the program to set up the
  * network interface. It calls the function low_level_init() to do the
  * actual setup of the hardware.
  *
  * This function should be passed as a parameter to netif_add().
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return ERR_OK if the loopif is initialized
  *         ERR_MEM if private data couldn't be allocated
  *         any other err_t on error
  */
err_t ethernetif_init(struct netif *netif)
{
  ASSERT(netif != NULL);
  
#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */
  STATIC_ASSERT(sizeof(netif->name)/sizeof(netif->name[0]) == 2, "compat");
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function and call etharp_output()
   * from it if you have to do some checks before sending */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  return low_level_init(netif, g_MAC_addr);
}

void ethernetif_shutdown(struct netif *netif)
{
    HAL_ETH_Stop(&EthHandle);
    LAN8742_DeInit(&LAN8742);
}

/**
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
void pbuf_free_custom(struct pbuf *p)
{
  struct pbuf_custom* custom_pbuf = (struct pbuf_custom*)p;
  /* Invalidate data cache: lwIP and/or application may have written into buffer */
  //SCB_InvalidateDCache_by_Addr((uint32_t *)p->payload, p->tot_len);
  flush_cache_(false, p->payload, p->tot_len);
  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
}

/**
  * @brief  Returns the current system time in milliseconds
  *         when LWIP_TIMERS == 1 and NO_SYS == 1
  * @param  None
  * @retval Current Time value
  */
u32_t sys_now(void)
{
  // Assume the HAL tick is 1 ms
  return HAL_GetTick();
}

/*******************************************************************************
                       Ethernet MSP Routines
*******************************************************************************/
/**
  * @brief  Initializes the ETH MSP.
  * @param  heth: ETH handle
  * @retval None
  */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  /* Ethernet MSP init: RMII Mode */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Alternate = GPIO_AF11_ETH;
  
  /* Enable GPIOs clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

#if defined(EVB_ST_EVAL2) && (EVB_ST_EVAL2)

  /* Ethernet RMII pins configuration for EVAL2 Rev. E; checked per UM2198 *******************
        RMII_REF_CLK ----------------------> PA1
        RMII_MDIO -------------------------> PA2
        RMII_CRS_DV -----------------------> PA7
        RMII_MDC --------------------------> PC1
        RMII_RXD0 -------------------------> PC4
        RMII_RXD1 -------------------------> PC5
        RMII_TX_EN ------------------------> PG11
        RMII_TXD1 -------------------------> PG12
        RMII_TXD0 -------------------------> PG13

       ETH_PPS_OUT - not connected; can be PB5|PG8
       PA8 (MCO1 @ 25 Mhz) can be used as RMII_REF_CLK; selected by jumper (default: NO)
  */

  /* Configure PA1, PA2 and PA7 */
  GPIO_InitStructure.Pin =  GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure PG11, PG12 and PG13 */
  GPIO_InitStructure.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);
  
  /* Configure PC1, PC4 and PC5 */
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5; 
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);	

#elif  defined(EVB_NUCLEO743) && (EVB_NUCLEO743)

  /* Ethernet RMII pins configuration for NUCLEO-H7; checked per UM2407 *******************
        RMII_REF_CLK ----------------------> PA1
        RMII_MDIO -------------------------> PA2
        RMII_CRS_DV -----------------------> PA7
        RMII_TXD1 -------------------------> PB13
        RMII_MDC --------------------------> PC1
        RMII_RXD0 -------------------------> PC4
        RMII_RXD1 -------------------------> PC5
        RMII_TX_EN ------------------------> PG11
        RMII_TXD0 -------------------------> PG13

       ETH_PPS_OUT - not connected; can be PB5|PG8
  */

  /* Configure PA1, PA2 and PA7 */
  GPIO_InitStructure.Pin =  GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PB13 */
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Configure PC1, PC4 and PC5 */
  GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PG11, PG13 ??? was also PG2, why ?? */
  GPIO_InitStructure.Pin =  GPIO_PIN_11 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);	

#else
#error "ETH: undefined board"
#endif
  /* Enable Ethernet clocks */
  __HAL_RCC_ETH1MAC_CLK_ENABLE();
  __HAL_RCC_ETH1TX_CLK_ENABLE();
  __HAL_RCC_ETH1RX_CLK_ENABLE();
}

/*******************************************************************************
                       PHY IO Functions
*******************************************************************************/
/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_Init(void)
{  
  /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here 
  */
  
  /* Configure the MDIO Clock */
  HAL_ETH_SetMDIOClockRange(&EthHandle);
  
  return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_DeInit (void)
{
  return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  if(HAL_ETH_ReadPHYRegister(&EthHandle, DevAddr, RegAddr, pRegVal) != HAL_OK)
  {
    return -1;
  }
  
  return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written 
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  if(HAL_ETH_WritePHYRegister(&EthHandle, DevAddr, RegAddr, RegVal) != HAL_OK)
  {
    return -1;
  }
  
  return 0;
}

static inline int32_t PHY_GetLinkState(void) {return LAN8742_GetLinkState(&LAN8742); }

// Map PHY link status to generic - pa02
// Negative values are errors
#define  ETH_PHY_STATUS_OK                    LAN8742_STATUS_OK
_Static_assert(0 == ETH_PHY_STATUS_OK, "sanity");
#define  ETH_PHY_STATUS_LINK_DOWN             LAN8742_STATUS_LINK_DOWN
#define  ETH_PHY_STATUS_100MBITS_FULLDUPLEX   LAN8742_STATUS_100MBITS_FULLDUPLEX
#define  ETH_PHY_STATUS_100MBITS_HALFDUPLEX   LAN8742_STATUS_100MBITS_HALFDUPLEX
#define  ETH_PHY_STATUS_10MBITS_FULLDUPLEX    LAN8742_STATUS_10MBITS_FULLDUPLEX
#define  ETH_PHY_STATUS_10MBITS_HALFDUPLEX    LAN8742_STATUS_10MBITS_HALFDUPLEX
#define  ETH_PHY_STATUS_AUTONEGO_NOTDONE      LAN8742_STATUS_AUTONEGO_NOTDONE

/**
  * @brief  Get the system time in milliseconds used for internal PHY driver process.
  * @retval Time value
  */
int32_t ETH_PHY_IO_GetTick(void)
{
  // Assume the HAL tick is 1 ms
  return HAL_GetTick();
}

/**
  * @brief Checks PHY link state, update netif logical state
  * @retval None
  */
void ethernet_link_check_state(struct netif *netif)
{
  ETH_MACConfigTypeDef MACConf;
  uint32_t PHYLinkState;
  uint32_t linkchanged = 0, speed = 0, duplex =0;
  
  PHYLinkState = PHY_GetLinkState();
  
  if (PHYLinkState < 0 && netif_is_link_up(netif))
  {
      dbgprintf("ETH: PHY state error %"PRId32"\n", PHYLinkState);
  }

  if (netif_is_link_up(netif) && (PHYLinkState <= ETH_PHY_STATUS_LINK_DOWN))
  {
    // Link goes DOWN
    dbgprintf("ETH: cable disconnect\n");
    HAL_ETH_Stop(&EthHandle);
    netif_set_down(netif);
    netif_set_link_down(netif);
  }
  else if(!netif_is_link_up(netif) && (PHYLinkState > ETH_PHY_STATUS_LINK_DOWN))
  {
    // Link goes UP
    dbgprintf("ETH: cable connected\n");
    switch (PHYLinkState)
    {
    case ETH_PHY_STATUS_100MBITS_FULLDUPLEX:
      dbgprintf("ETH: link 100/FDX\n");
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case ETH_PHY_STATUS_100MBITS_HALFDUPLEX:
      dbgprintf("ETH: link 100/HDX\n");
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case ETH_PHY_STATUS_10MBITS_FULLDUPLEX:
      dbgprintf("ETH: link 10/FDX\n");
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    case ETH_PHY_STATUS_10MBITS_HALFDUPLEX:
      dbgprintf("ETH: link 10/HDX\n");
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    case ETH_PHY_STATUS_AUTONEGO_NOTDONE:
      dbgprintf("ETH: link not ready yet ...\n");
      break;
    default:
      dbgprintf("ETH: link UNDEFINED??\n");
      break;      
    }
    
    if (linkchanged)
    {
      /* Update MAC Config and restart ETH */
      HAL_ETH_GetMACConfig(&EthHandle, &MACConf); 
      MACConf.DuplexMode = duplex;
      MACConf.Speed = speed;
      HAL_StatusTypeDef st =
      HAL_ETH_SetMACConfig(&EthHandle, &MACConf);
      if (st != HAL_OK) {
          dbgprintf("ETH: error SetMACConfig!\n");
      }
      st = HAL_ETH_Start(&EthHandle);
      if (st != HAL_OK) {
          dbgprintf("ETH: error ETH_Start!\n");
      }
      netif_set_up(netif);
      netif_set_link_up(netif);
    }
  }
}

/*
******************************************************************************
* @ file    LwIP/LwIP_UDP_Echo_Server/Src/ethernetif.c
* @ author  MCD Application Team
* @ brief   This file implements Ethernet network interface drivers for lwIP
* <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
******************************************************************************
*/
