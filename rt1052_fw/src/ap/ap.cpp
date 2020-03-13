/*
 * ap.cpp
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */




#include "ap.h"





void apInit(void)
{
  hwInit();

  cmdifOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);
}

void apMain(void)
{
  uint32_t pre_time;


  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 500)
    {
      pre_time = millis();
      ledToggle(_DEF_LED2);
    }

    cmdifMain();

    if (uartAvailable(_DEF_UART2) > 0)
    {
      uartPrintf(_DEF_UART2, "rx : 0x%X \n", uartRead(_DEF_UART2));
    }
  }
}


