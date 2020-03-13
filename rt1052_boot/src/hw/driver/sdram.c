/*
 * sdram.c
 *
 *  Created on: 2020. 2. 21.
 *      Author: HanCheol Cho
 */




#include "sdram.h"
#include "cmdif.h"
#include "fsl_semc.h"





static bool is_init = false;


#ifdef _USE_HW_CMDIF
void sdramCmdifInit(void);
void sdramCmdif(void);
#endif



bool sdramInit(void)
{
  bool ret = true;;

  semc_config_t config;
  semc_sdram_config_t sdramconfig;
  uint32_t clockFrq = CLOCK_GetFreq(kCLOCK_SemcClk);

  /* Initializes the MAC configure structure to zero. */
  memset(&config, 0, sizeof(semc_config_t));
  memset(&sdramconfig, 0, sizeof(semc_sdram_config_t));

  /* Initialize SEMC. */
  SEMC_GetDefaultConfig(&config);
  config.dqsMode = kSEMC_Loopbackdqspad; /* For more accurate timing. */
  SEMC_Init(SEMC, &config);

  /* Configure SDRAM. */
  sdramconfig.csxPinMux           = kSEMC_MUXCSX0;
  sdramconfig.address             = 0x80000000;
  sdramconfig.memsize_kbytes      = 32 * 1024; /* 32MB = 32*1024*1KBytes*/
  sdramconfig.portSize            = kSEMC_PortSize16Bit;
  sdramconfig.burstLen            = kSEMC_Sdram_BurstLen8;
  sdramconfig.columnAddrBitNum    = kSEMC_SdramColunm_9bit;
  sdramconfig.casLatency          = kSEMC_LatencyThree;
  sdramconfig.tPrecharge2Act_Ns   = 18; /* Trp 18ns */
  sdramconfig.tAct2ReadWrite_Ns   = 18; /* Trcd 18ns */
  sdramconfig.tRefreshRecovery_Ns = 67; /* Use the maximum of the (Trfc , Txsr). */
  sdramconfig.tWriteRecovery_Ns   = 12; /* 12ns */
  sdramconfig.tCkeOff_Ns =
      42; /* The minimum cycle of SDRAM CLK off state. CKE is off in self refresh at a minimum period tRAS.*/
  sdramconfig.tAct2Prechage_Ns       = 42; /* Tras 42ns */
  sdramconfig.tSelfRefRecovery_Ns    = 67;
  sdramconfig.tRefresh2Refresh_Ns    = 60;
  sdramconfig.tAct2Act_Ns            = 60;
  sdramconfig.tPrescalePeriod_Ns     = 160 * (1000000000 / clockFrq);
  sdramconfig.refreshPeriod_nsPerRow = 64 * 1000000 / 8192; /* 64ms/8192 */
  sdramconfig.refreshUrgThreshold    = sdramconfig.refreshPeriod_nsPerRow;
  sdramconfig.refreshBurstLen        = 1;

  if (SEMC_ConfigureSDRAM(SEMC, kSEMC_SDRAM_CS0, &sdramconfig, clockFrq) == kStatus_Success)
  {
    ret = true;
  }
  else
  {
    ret = false;
  }


  if (ret == true)
  {
    logPrintf("SDRAM %dMB \t\t: OK\r\n", (int)(SDRAM_MEM_SIZE/1024/1024));
  }
  else
  {
    logPrintf("SDRAM  \t\t: Fail\r\n");
  }

#ifdef _USE_HW_CMDIF
  sdramCmdifInit();
#endif

  is_init = ret;

  return ret;
}

bool sdramIsInit(void)
{
  return is_init;
}

uint32_t sdramGetAddr(void)
{
  return SDRAM_MEM_ADDR;
}

uint32_t sdramGetLength(void)
{
  return SDRAM_MEM_SIZE;
}

bool sdramTest(void)
{
  uint32_t *p_data = (uint32_t *)SDRAM_MEM_ADDR;
  uint32_t i;


  for (i=0; i<SDRAM_MEM_SIZE/4; i++)
  {
    p_data[i] = i;
  }

  for (i=0; i<SDRAM_MEM_SIZE/4; i++)
  {
    if (p_data[i] != i)
    {
      return false;
    }
  }

  for (i=0; i<SDRAM_MEM_SIZE/4; i++)
  {
    p_data[i] = 0x5555AAAA;
  }
  for (i=0; i<SDRAM_MEM_SIZE/4; i++)
  {
    if (p_data[i] != 0x5555AAAA)
    {
      return false;
    }
  }

  for (i=0; i<SDRAM_MEM_SIZE/4; i++)
  {
    p_data[i] = 0xAAAA5555;
  }
  for (i=0; i<SDRAM_MEM_SIZE/4; i++)
  {
    if (p_data[i] != 0xAAAA5555)
    {
      return false;
    }
  }

  return true;
}





#ifdef _USE_HW_CMDIF
void sdramCmdifInit(void)
{
  cmdifAdd("sdram", sdramCmdif);
}

void sdramCmdif(void)
{
  bool ret = true;
  uint8_t number;
  uint32_t i;
  uint32_t pre_time;


  if (cmdifGetParamCnt() == 2)
  {
    if(cmdifHasString("test", 0) == true)
    {
      uint32_t *p_data = (uint32_t *)SDRAM_MEM_ADDR;

      number = (uint8_t)cmdifGetParam(1);

      while(number > 0)
      {
        pre_time = millis();
        for (i=0; i<SDRAM_MEM_SIZE/4; i++)
        {
          p_data[i] = i;
        }
        cmdifPrintf( "Write : %d MB/s\n", SDRAM_MEM_SIZE / 1000 / (millis()-pre_time) );

        volatile uint32_t data_sum = 0;
        pre_time = millis();
        for (i=0; i<SDRAM_MEM_SIZE/4; i++)
        {
          data_sum += p_data[i];
        }
        cmdifPrintf( "Read : %d MB/s\n", SDRAM_MEM_SIZE / 1000 / (millis()-pre_time) );


        for (i=0; i<SDRAM_MEM_SIZE/4; i++)
        {
          if (p_data[i] != i)
          {
            cmdifPrintf( "%d : 0x%X fail\n", i, p_data[i]);
            break;
          }
        }

        if (i == SDRAM_MEM_SIZE/4)
        {
          cmdifPrintf( "Count %d\n", number);
          cmdifPrintf( "Sdram %d MB OK\n\n", SDRAM_MEM_SIZE/1024/1024);
          for (i=0; i<SDRAM_MEM_SIZE/4; i++)
          {
            p_data[i] = 0x5555AAAA;
          }
        }

        number--;

        if (cmdifRxAvailable() > 0)
        {
          cmdifPrintf( "Stop test...\n");
          break;
        }
      }
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    ret = false;
  }


  if (ret == false)
  {
    cmdifPrintf( "sdram test 1~100 \n");
  }
}
#endif
