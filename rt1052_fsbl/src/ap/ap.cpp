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

}

void apMain(void)
{
  uint32_t pre_time;
  void (**jump_func)(void);
  boot_tag_t *p_tag;


  p_tag = (boot_tag_t *)FW_ADDR_TAG;

  if (p_tag->magic_number == 0x5555AAAA)
  {

    jump_func = (void (**)(void))(p_tag->addr_fw + 4);

    memcpy((void *)p_tag->addr_fw, (const void *)p_tag->image_start, p_tag->image_size);

    bspDeInit();

    __set_MSP(*(uint32_t *)p_tag->addr_fw);
    (*jump_func)();
  }

  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 100)
    {
      pre_time = millis();

      ledToggle(_DEF_LED3);
    }
  }
}

