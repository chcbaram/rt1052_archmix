/*
 * bsp.c
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */




#include "bsp.h"
#include "uart.h"




static volatile uint32_t systick_counter = 0;
extern void swtimerISR(void);


void SysTick_Handler(void)
{
  systick_counter++;
  swtimerISR();
}


extern void qspiInit(void);

void bspInit(void)
{
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
  BOARD_InitBootPeripherals();


  SysTick_Config(SystemCoreClock / 1000U);


  DisableIRQ(LPUART1_SERIAL_RX_TX_IRQN);


#if 0
  /* Disable I cache and D cache */
  if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR))
  {
      SCB_DisableICache();
  }
  if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR))
  {
      SCB_DisableDCache();
  }

  /* Disable MPU */
  ARM_MPU_Disable();

  MPU->RBAR = ARM_MPU_RBAR(0, 0x80000000U);
  MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_512MB);

  /* Enable MPU */
  ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
#endif

  SCB_EnableDCache();
  SCB_EnableICache();
}

void bspDeInit(void)
{
  // Disable Interrupts
  //
  for (int i=0; i<8; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    __DSB();
    __ISB();
  }
  SysTick->CTRL = 0;

  SCB_DisableDCache();
  SCB_DisableICache();
}

int __io_putchar(int ch)
{
  uartWrite(_DEF_UART1, (uint8_t *)&ch, 1);
  return 1;
}

void delay(uint32_t ms)
{
  uint32_t pre_time = systick_counter;

  while(systick_counter-pre_time < ms);
}

uint32_t millis(void)
{
  return systick_counter;
}

