#ifndef LPC_H
#define LPC_H

#define LPC_AHBRAM0_BASE (0x2007C000)
#define LPC_AHBRAM1_BASE (0x20080000)
#define LPC_GPIO_BASE (0x2009C000)
#define LPC_APB0_BASE (0x40000000)
#define LPC_APB1_BASE (0x40080000)
#define LPC_AHB_BASE (0x50000000)
#define LPC_CM3_BASE (0xE0000000)
#define LPC_WDT_BASE (LPC_APB0_BASE + 0x00000)
#define LPC_TIM0_BASE (LPC_APB0_BASE + 0x04000)
#define LPC_TIM1_BASE (LPC_APB0_BASE + 0x08000)
#define LPC_UART0_BASE (LPC_APB0_BASE + 0x0C000)
#define LPC_UART1_BASE (LPC_APB0_BASE + 0x10000)
#define LPC_PWM1_BASE (LPC_APB0_BASE + 0x18000)
#define LPC_I2C0_BASE (LPC_APB0_BASE + 0x1C000)
#define LPC_SPI_BASE (LPC_APB0_BASE + 0x20000)
#define LPC_RTC_BASE (LPC_APB0_BASE + 0x24000)
#define LPC_GPIOINT_BASE (LPC_APB0_BASE + 0x28080)
#define LPC_PINCON_BASE (LPC_APB0_BASE + 0x2C000)
#define LPC_SSP1_BASE (LPC_APB0_BASE + 0x30000)
#define LPC_ADC_BASE (LPC_APB0_BASE + 0x34000)
#define LPC_CANAF_RAM_BASE (LPC_APB0_BASE + 0x38000)
#define LPC_CANAF_BASE (LPC_APB0_BASE + 0x3C000)
#define LPC_CANCR_BASE (LPC_APB0_BASE + 0x40000)
#define LPC_CAN1_BASE (LPC_APB0_BASE + 0x44000)
#define LPC_CAN2_BASE (LPC_APB0_BASE + 0x48000)
#define LPC_I2C1_BASE (LPC_APB0_BASE + 0x5C000)
/*LPC_SSP0_BASE = (LPC_APB1_BASE + 0x08000)
LPC_DAC_BASE = (LPC_APB1_BASE + 0x0C000)
LPC_TIM2_BASE = (LPC_APB1_BASE + 0x10000)
LPC_TIM3_BASE = (LPC_APB1_BASE + 0x14000)
LPC_UART2_BASE = (LPC_APB1_BASE + 0x18000)
LPC_UART3_BASE = (LPC_APB1_BASE + 0x1C000)*/
#define LPC_I2C2_BASE (LPC_APB1_BASE + 0x20000)
/*LPC_I2S_BASE = (LPC_APB1_BASE + 0x28000)
LPC_RIT_BASE = (LPC_APB1_BASE + 0x30000)*/
#define LPC_MCPWM_BASE (LPC_APB1_BASE + 0x38000)
/*LPC_QEI_BASE = (LPC_APB1_BASE + 0x3C000)
LPC_SC_BASE = (LPC_APB1_BASE + 0x7C000)
LPC_EMAC_BASE = (LPC_AHB_BASE  + 0x00000)
LPC_GPDMA_BASE = (LPC_AHB_BASE  + 0x04000)
LPC_GPDMACH0_BASE = (LPC_AHB_BASE  + 0x04100)
LPC_GPDMACH1_BASE = (LPC_AHB_BASE  + 0x04120)
LPC_GPDMACH2_BASE = (LPC_AHB_BASE  + 0x04140)
LPC_GPDMACH3_BASE = (LPC_AHB_BASE  + 0x04160)
LPC_GPDMACH4_BASE = (LPC_AHB_BASE  + 0x04180)
LPC_GPDMACH5_BASE = (LPC_AHB_BASE  + 0x041A0)
LPC_GPDMACH6_BASE = (LPC_AHB_BASE  + 0x041C0)
LPC_GPDMACH7_BASE = (LPC_AHB_BASE  + 0x041E0)
LPC_USB_BASE = (LPC_AHB_BASE  + 0x0C000)
LPC_GPIO0_BASE = (LPC_GPIO_BASE + 0x00000)
LPC_GPIO1_BASE = (LPC_GPIO_BASE + 0x00020)
LPC_GPIO2_BASE = (LPC_GPIO_BASE + 0x00040)
LPC_GPIO3_BASE = (LPC_GPIO_BASE + 0x00060)
LPC_GPIO4_BASE = (LPC_GPIO_BASE + 0x00080)
LPC_SC = LPC_SC_BASE
LPC_GPIO0 = LPC_GPIO0_BASE
LPC_GPIO1 = LPC_GPIO1_BASE
LPC_GPIO2 = LPC_GPIO2_BASE
LPC_GPIO3 = LPC_GPIO3_BASE
LPC_GPIO4 = LPC_GPIO4_BASE
LPC_WDT = LPC_WDT_BASE
LPC_TIM0 = LPC_TIM0_BASE
LPC_TIM1 = LPC_TIM1_BASE
LPC_TIM2 = LPC_TIM2_BASE
LPC_TIM3 = LPC_TIM3_BASE
LPC_RIT = LPC_RIT_BASE
LPC_UART0 = LPC_UART0_BASE
LPC_UART1 = LPC_UART1_BASE
LPC_UART2 = LPC_UART2_BASE
LPC_UART3 = LPC_UART3_BASE*/
#define LPC_PWM1 LPC_PWM1_BASE
#define LPC_I2C0 LPC_I2C0_BASE
#define LPC_I2C1 LPC_I2C1_BASE
#define LPC_I2C2 LPC_I2C2_BASE
/*LPC_I2S = LPC_I2S_BASE
LPC_SPI = LPC_SPI_BASE
LPC_RTC = LPC_RTC_BASE
LPC_GPIOINT = LPC_GPIOINT_BASE
LPC_PINCON = LPC_PINCON_BASE
LPC_SSP0 = LPC_SSP0_BASE
LPC_SSP1 = LPC_SSP1_BASE*/
#define LPC_ADC LPC_ADC_BASE
#define LPC_DAC LPC_DAC_BASE
/*LPC_CANAF_RAM = LPC_CANAF_RAM_BASE
LPC_CANAF = LPC_CANAF_BASE
LPC_CANCR = LPC_CANCR_BASE
LPC_CAN1 = LPC_CAN1_BASE
LPC_CAN2 = LPC_CAN2_BASE
LPC_MCPWM = LPC_MCPWM_BASE
LPC_QEI = LPC_QEI_BASE
LPC_EMAC = LPC_EMAC_BASE
LPC_GPDMA = LPC_GPDMA_BASE
DMAREQSEL = 0x4000C1C4
LPC_GPDMACH0 =  LPC_GPDMACH0_BASE
LPC_GPDMACH1 =  LPC_GPDMACH1_BASE
LPC_GPDMACH2 = LPC_GPDMACH2_BASE
LPC_GPDMACH3 =  LPC_GPDMACH3_BASE
LPC_GPDMACH4 =  LPC_GPDMACH4_BASE
LPC_GPDMACH5 =  LPC_GPDMACH5_BASE
LPC_GPDMACH6 =  LPC_GPDMACH6_BASE
LPC_GPDMACH7 =  LPC_GPDMACH7_BASE
LPC_USB =  LPC_USB_BASE*/

  // Channel 0
#define ADC_CHANNEL_0 0
  // Channel 1
#define ADC_CHANNEL_1 1
  // Channel 2
#define ADC_CHANNEL_2 2
  // Channel 3
#define ADC_CHANNEL_3 3
  // Channel 4
#define ADC_CHANNEL_4 4
  // Channel 5
#define ADC_CHANNEL_5 5
  // Channel 6
#define ADC_CHANNEL_6 6
  // Channel 7
#define ADC_CHANNEL_7 7
  // Continuous mode
#define ADC_START_CONTINUOUS 0
  // Start conversion now
#define ADC_START_NOW 1
  // Start conversion when the edge selected by bit 27 occurs on P2.10/EINT0
#define ADC_START_ON_EINT0 2
  // Start conversion when the edge selected by bit 27 occurs on P1.27/CAP0.1
#define ADC_START_ON_CAP01 3
  // Start conversion when the edge selected by bit 27 occurs on MAT0.1
#define ADC_START_ON_MAT01 4
  // Start conversion when the edge selected by bit 27 occurs on MAT0.3
#define ADC_START_ON_MAT03 5
  // Start conversion when the edge selected by bit 27 occurs on MAT1.0
#define ADC_START_ON_MAT10 6
  // Start conversion when the edge selected by bit 27 occurs on MAT1.1
#define ADC_START_ON_MAT11 7
#define ADC_DATA_BURST 0
  // Done bit
#define ADC_DATA_DONE 1

#endif