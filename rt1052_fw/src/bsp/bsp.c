/*
 * bsp.c
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */




#include "bsp.h"
#include "uart.h"
#include "rtos.h"



static volatile uint32_t systick_counter = 0;
extern void swtimerISR(void);


void SysTick_Handler(void)
{
  systick_counter++;
  swtimerISR();
  osSystickHandler();
}




void bspInit(void)
{
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
  BOARD_InitBootPeripherals();


  SysTick_Config(SystemCoreClock / 1000U);


  DisableIRQ(LPUART1_SERIAL_RX_TX_IRQN);


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

#ifdef _USE_HW_RTOS
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    osDelay(ms);
  }
  else
  {
    while(systick_counter-pre_time < ms);
  }
#else
  while(systick_counter-pre_time < ms);
#endif
}

uint32_t millis(void)
{
  return systick_counter;
}

