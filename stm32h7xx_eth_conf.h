#pragma once

/* Number of descriptors */
#ifdef ETH_TX_DESC_CNT
#undef ETH_TX_DESC_CNT
#endif

#define ETH_TX_DESC_CNT         4U

#ifdef ETH_RX_DESC_CNT
#undef ETH_RX_DESC_CNT
#endif

#define ETH_RX_DESC_CNT         4U


#define ETH_RX_BUFFER_SIZE     (1536UL)


/* Extra size reserved in descriptors; both RX and TX */
#define ETH_DESC_EXTRA_SIZE     8 /* 2 words */

/* MAC address */
#define ETH_MAC_ADDR0    0x02
#define ETH_MAC_ADDR1    0x00
#define ETH_MAC_ADDR2    0x00
#define ETH_MAC_ADDR3    0x00
#define ETH_MAC_ADDR4    0x00
#define ETH_MAC_ADDR5    0x00

/* Definitions for PHY - TODO ... */
