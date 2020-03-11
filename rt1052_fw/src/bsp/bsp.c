/*
 * bsp.c
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */




#include "bsp.h"





static volatile uint32_t systick_counter = 0;
extern void swtimerISR(void);


void SysTick_Handler(void)
{
  systick_counter++;
  swtimerISR();
}



void bspInit(void)
{
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
  BOARD_InitBootPeripherals();

  SysTick_Config(SystemCoreClock / 1000U);



  SCB_EnableDCache();
  SCB_EnableICache();
}

void bspDeInit(void)
{
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

