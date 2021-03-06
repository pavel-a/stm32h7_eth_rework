STM32H7 Ethernet driver split POC
Example how the H7 ETH driver separation to "core" and "middleware" may look

Based on ST Cube library examples with LwIP

WIP


*************
* stm32h7xx_eth_conf.h is a new config file that replaces ethernet section in stm32h7xx_hal_conf.h
* stm32h7xx_hal_eth.c renamed to stm32h7xx_eth.c , all descriptor related code moved out
* stm32h7xx_hal_eth.h renamed to stm32h7xx_eth.h, all descriptor related code moved out
* stm32h7xx_eth_MW.c - "middleware" layer extracted from ^^
* stm32h7xx_eth_MW.h - "middleware" layer extracted from ^^
    This #includes stm32h7xx_hal_eth.h and stm32h7xx_eth_conf.h

The "middleware" layers should be different and specific for each network stack such as LwIP, FreeRTOS+ etc.

Size of DMA descriptors (extra fields in the descritors) is defined by the "middleware" layer.
This is defined as ETH_DESC_EXTRA_SIZE in stm32h7xx_eth_conf.h (fixed at compile time for simplicity;
cannot move this definition to stm32h7xx_eth_MW.h as it is needed in stm32h7xx_hal_eth.c.
This extra size could be made variable and defined in runtime, may consider for later.)

Number of the DMA descriptors is variable, specified in runtime.
(this is to ease configuration for our project where all boards have same  "middleware" layer, but variable amount of descriptors)
stm32h7xx_eth_MW.c uses the 'legacy' definitions ETH_TX_DESC_CNT, ETH_RX_DESC_CNT and passes them to stm32h7xx_hal_eth.c
stm32h7xx_hal_eth.c no longer depends on these definitions.

TODO For LwIP based examples: rework ethernetif.c to use stm32h7xx_eth_MW.c|h
   Factor out remaining few register-level deps from stm32h7xx_eth_MW.c to stm32h7xx_hal_eth.c (TX start...)
=====

When using this driver, either do NOT define HAL_ETH_MODULE_ENABLED in stm32h7xx_hal_conf.h
(but HAL_ETH_MspInit is still requred, bring your own) or remove/exclude the original stm32h7xx_hal_eth.c from build,
and do not include original stm32h7xx_hal_eth.h.

Original from STM32H7 HAL drivers v 10.0.0, included in Cube package H7_V1.9.0


To distinguish variants and patches, use following defines in stm32h7xx_eth.h (and/or stm32h7xx_eth_MW.h , and/or stm32h7xx_eth_conf.h)

  Nothing of following defined  -- ST HAL driver with minimal changes only (renamed, hal_conf dependency decoupled...)
  #define STM32H7_ETH_PATCH_x 1    etc

