/*
 * hw.c
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */




#include "hw.h"





void hwInit(void)
{
  bspInit();


  ledInit();
  uartInit();
  uartOpen(_DEF_UART1, 57600);
}
