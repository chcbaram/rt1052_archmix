/*
 * ap.cpp
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */




#include "ap.h"
#include "boot/boot.h"


static cmd_t cmd_boot;


void bootCmdif(void);


void apInit(void)
{
  hwInit();

  cmdifOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);

  cmdInit(&cmd_boot);
  cmdBegin(&cmd_boot, _DEF_UART2, 57600);
  cmdifAdd("boot", bootCmdif);


  if (buttonGetPressed(0) != true && (resetGetBootMode() & (1<<0)) == 0)
  {
    resetSetBootMode(0);

    if (bootVerifyCrc(FLASH_ADDR_TAG) != true)
    {
      logPrintf("fw crc    \t\t: Fail\r\n");
      logPrintf("boot begin...\r\n");
    }
    else
    {
      bootJumpToFw(FLASH_ADDR_TAG);
    }
  }

  usbdInit();
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

    if (cmdReceivePacket(&cmd_boot) == true)
    {
      bootProcessCmd(&cmd_boot);
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
      cmdifPrintf("Load Start: 0x%X \n", p_tag->load_start);
      cmdifPrintf("Load Size : 0x%X \n", p_tag->load_size);

      if (p_tag->addr_fw != p_tag->load_start)
      {
        uint32_t pre_time;
        pre_time = micros();
        memcpy((void *)p_tag->addr_fw, (const void *)p_tag->load_start, p_tag->load_size);
        SCB_InvalidateDCache_by_Addr((void *)(p_tag->addr_fw), p_tag->load_size);
        cmdifPrintf("Load Fw   : %d ms \n", (micros()-pre_time)/1000);
      }

      cmdifPrintf("Jump Addr : 0x%X \n", (int)(*jump_func));

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
      cmdifPrintf("Load Start: 0x%X \n", p_tag->load_start);
      cmdifPrintf("Load Size : %d KB \n", p_tag->load_size/1024);
    }
    else
    {
      cmdifPrintf("firmware empty \n");
    }
  }
  else if (cmdifGetParamCnt() == 2 && cmdifHasString("file", 0) == true)
  {
    char *file_name;

    p_tag = (flash_tag_t *)SDRAM_ADDR_TAG;
    file_name = (char *)cmdifGetParamStr(1);


    cmdifPrintf("File Name    : %s \n", file_name);

    FILE *f;
    int  file_len;
    int  read_len;
    f = fopen(file_name, "rb");
    if (f != NULL)
    {
      cmdifPrintf("fopen        : OK \n");

      fseek(f, 0, SEEK_END);
      file_len = ftell(f);
      fseek(f, 0, SEEK_SET);

      cmdifPrintf("file size    : %d KB\n", file_len/1024);

      read_len = fread((void *)SDRAM_ADDR_TAG, 1, file_len, f);
      if (read_len == file_len)
      {
        void (**jump_func)(void) = (void (**)(void))(p_tag->addr_fw + 4);

        SCB_InvalidateDCache_by_Addr((void *)(p_tag->addr_fw), p_tag->load_size);

        cmdifPrintf("fread        : OK\n");
        cmdifPrintf("Jump Addr    : 0x%X \n", (int)(*jump_func));

        delay(100);
        bspDeInit();

        __set_MSP(*(uint32_t *)p_tag->addr_fw);
        (*jump_func)();
      }
      else
      {
        cmdifPrintf("fread        : fail\n");
      }
      fclose(f);
    }
    else
    {
      cmdifPrintf("fopen fail \n");
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

