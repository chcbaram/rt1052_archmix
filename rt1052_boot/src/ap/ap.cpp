/*
 * ap.cpp
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */




#include "ap.h"



void bootCmdif(void);


void apInit(void)
{
  hwInit();

  cmdifOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);

  cmdifAdd("boot", bootCmdif);
}

void apMain(void)
{
  uint32_t pre_time;

  pre_time = millis();
  while(1)
  {
    if (millis()-pre_time >= 100)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
    }

    cmdifMain();

    if (uartAvailable(_DEF_UART2) > 0)
    {
      uartPrintf(_DEF_UART2, "rx : 0x%X \n", uartRead(_DEF_UART2));
    }
  }
}

void bootCmdif(void)
{
  bool ret = true;
  flash_tag_t *p_tag = (flash_tag_t *)FLASH_ADDR_TAG;



  if (cmdifGetParamCnt() == 1 && cmdifHasString("jump", 0) == true)
  {
    void (**jump_func)(void) = (void (**)(void))(p_tag->addr_fw + 4);

    if (p_tag->magic_number == 0x5555AAAA || p_tag->magic_number == 0xAAAA5555)
    {
      cmdifPrintf("Board     : %s \n", p_tag->board_str);
      cmdifPrintf("Name      : %s \n", p_tag->name_str);
      cmdifPrintf("Version   : %s \n", p_tag->version_str);
      cmdifPrintf("Addr Tag  : 0x%X \n", p_tag->addr_tag);
      cmdifPrintf("Addr Fw   : 0x%X \n", p_tag->addr_fw);
      delay(100);
      bspDeInit();

      __set_MSP(*(uint32_t *)p_tag->addr_fw);
      (*jump_func)();
    }
    else
    {
      cmdifPrintf("firmware empty \n");
    }
  }
  else if (cmdifGetParamCnt() == 1 && cmdifHasString("info", 0) == true)
  {
    if (p_tag->magic_number == 0x5555AAAA || p_tag->magic_number == 0xAAAA5555)
    {
      cmdifPrintf("Board     : %s \n", p_tag->board_str);
      cmdifPrintf("Name      : %s \n", p_tag->name_str);
      cmdifPrintf("Version   : %s \n", p_tag->version_str);
      cmdifPrintf("Date      : %s \n", p_tag->date_str);
      cmdifPrintf("Time      : %s \n", p_tag->time_str);
      cmdifPrintf("Addr Tag  : 0x%X \n", p_tag->addr_tag);
      cmdifPrintf("Addr Fw   : 0x%X \n", p_tag->addr_fw);
    }
    else
    {
      cmdifPrintf("firmware empty \n");
    }
  }
  else
  {
    ret = false;
  }

  if (ret == false)
  {
    cmdifPrintf( "boot jump \n");
    cmdifPrintf( "boot info \n");
  }
}
