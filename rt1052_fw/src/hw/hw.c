/*
 * hw.c
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */




#include "hw.h"



extern uint32_t __vectors_start__;


__attribute__((section(".tag"))) const flash_tag_t fw_tag =
   {
    // fw info
    //
    0xAAAA5555,        // magic_number
    "V200313R1",       // version_str
    "RT1052_B/D",      // board_str
    "Firmware",        // name
    __DATE__,
    __TIME__,
    (uint32_t)&fw_tag,
    (uint32_t)&__vectors_start__,


    // tag info
    //
   };


void hwInit(void)
{
  bspInit();

  microsInit();
  swtimerInit();
  cmdifInit();

  ledInit();
  buttonInit();
  vcpInit();
  uartInit();
  uartOpen(_DEF_UART1, 57600);

  logPrintf("\n\n[ Firmware Begin... ]\r\n");
  logPrintf("Tag Addr   \t\t: 0x%X\r\n", (int)&fw_tag);

  clocksInit();
  //flashInit();
  //sdramInit();
  gpioInit();

  if (sdInit() == true)
  {
    fatfsInit();
  }
}
