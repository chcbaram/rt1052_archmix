/*
 * hw.c
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */




#include "hw.h"

extern uint32_t __vectors_start__;
extern uint32_t _image_start;
extern uint32_t _image_end;
extern uint32_t _image_size;


__attribute__((aligned(2048))) __attribute__((used, section(".tag"))) const boot_tag_t boot_tag =
    {
        .boot_name    = "RT1052_B/D",
        .boot_ver     = "B2000312R1",
        .magic_number = 0x5555AAAA,
        .addr_fw      = (uint32_t)&__vectors_start__,
        .image_start  = (uint32_t)&_image_start,
        .image_end    = (uint32_t)&_image_end,
        .image_size   = (uint32_t)&_image_size,
    };


void hwInit(void)
{
  bspInit();

  swtimerInit();
  cmdifInit();

  ledInit();
  buttonInit();
  vcpInit();
  uartInit();
  uartOpen(_DEF_UART1, 57600);

  clocksInit();
  flashInit();
  sdramInit();
}
