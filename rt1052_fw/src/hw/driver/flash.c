/*
 * flash.c
 *
 *  Created on: 2020. 3. 13.
 *      Author: Baram
 */






#include "flash.h"
#include "qspi.h"
#include "cmdif.h"
#include "fsl_cache.h"
#include "fsl_flexspi.h"



#define FLASH_ADDR_OFFSET         0x60000000
#define FLASH_MAX_SIZE            (8*1024*1024)
#define FLASH_SECTOR_SIZE         (4*1024)
#define FLASH_PAGE_SIZE           (256)
#define FLASH_MAX_SECTOR          (FLASH_MAX_SIZE / FLASH_SECTOR_SIZE)


static bool is_init = false;


static bool _flashInit(void);
static bool _flashEraseSector(uint32_t start_sector,  uint32_t sector_cnt);
static bool _flashWritePage(uint32_t addr, uint32_t buf_addr);



#ifdef _USE_HW_CMDIF
void flashCmdifInit(void);
void flashCmdif(void);
#endif




bool flashInit(void)
{

  if (_flashInit() == false)
  {
    logPrintf("False                \t : Fail\r\n");
  }


#ifdef _USE_HW_CMDIF
  flashCmdifInit();
#endif

  is_init = true;

  return true;
}

bool flashErase(uint32_t addr, uint32_t length)
{
  bool ret = false;

  int32_t start_sector = -1;
  int32_t end_sector = -1;


#ifdef _USE_HW_QSPI
  if (addr >= qspiGetAddr() && addr < (qspiGetAddr() + qspiGetLength()))
  {
    ret = qspiErase(addr - qspiGetAddr(), length);
    return ret;
  }
#endif


  if (addr < FLASH_ADDR_OFFSET) return false;
  if (addr >= (FLASH_ADDR_OFFSET + FLASH_MAX_SIZE)) return false;
  if ((addr+length) > (FLASH_ADDR_OFFSET + FLASH_MAX_SIZE)) return false;

  start_sector = -1;
  end_sector = -1;


  for (int i=0; i<FLASH_MAX_SECTOR; i++)
  {
    bool update = false;
    uint32_t start_addr;
    uint32_t end_addr;


    start_addr = FLASH_ADDR_OFFSET + i * FLASH_SECTOR_SIZE;
    end_addr   = start_addr + FLASH_SECTOR_SIZE - 1;

    if (start_addr >= addr && start_addr < (addr+length))
    {
      update = true;
    }
    if (end_addr >= addr && end_addr < (addr+length))
    {
      update = true;
    }

    if (addr >= start_addr && addr <= end_addr)
    {
      update = true;
    }
    if ((addr+length-1) >= start_addr && (addr+length-1) <= end_addr)
    {
      update = true;
    }


    if (update == true)
    {
      if (start_sector < 0)
      {
        start_sector = i;
      }
      end_sector = i;
    }
  }

  if (start_sector >= 0)
  {
    ret = _flashEraseSector(start_sector,  (end_sector - start_sector) + 1);
  }


  return ret;
}

bool flashWrite(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  uint32_t index;
  uint32_t write_length;
  uint32_t write_addr;
  uint8_t buf[FLASH_PAGE_SIZE];
  uint32_t offset;



#ifdef _USE_HW_QSPI
  if (addr >= qspiGetAddr() && addr < (qspiGetAddr() + qspiGetLength()))
  {
    ret = qspiWrite(addr - qspiGetAddr(), p_data, length);
    return ret;
  }
#endif


  index = 0;
  offset = addr%FLASH_PAGE_SIZE;

  if (offset != 0 || length < FLASH_PAGE_SIZE)
  {
    write_addr = addr - offset;
    memcpy(&buf[0], (void *)write_addr, FLASH_PAGE_SIZE);
    memcpy(&buf[offset], &p_data[0], constrain(FLASH_PAGE_SIZE-offset, 0, length));

    ret = _flashWritePage(write_addr, (uint32_t)buf);
    if (ret != true)
    {
      return false;
    }

    if (length < FLASH_PAGE_SIZE)
    {
      index += length;
    }
    else
    {
      index += (FLASH_PAGE_SIZE - offset);
    }
  }


  while(index < length)
  {
    write_length = constrain(length - index, 0, FLASH_PAGE_SIZE);

    ret = _flashWritePage(addr + index, (uint32_t)&p_data[index]);
    if (ret != true)
    {
      ret = false;
      break;
    }

    index += write_length;

    if ((length - index) > 0 && (length - index) < FLASH_PAGE_SIZE)
    {
      offset = length - index;
      write_addr = addr + index;
      memcpy(&buf[0], (void *)write_addr, FLASH_PAGE_SIZE);
      memcpy(&buf[0], &p_data[index], offset);

      ret = _flashWritePage(write_addr, (uint32_t)buf);
      if (ret != true)
      {
        return false;
      }
      break;
    }
  }

  return ret;
}

bool flashRead(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  uint8_t *p_byte = (uint8_t *)addr;



  DCACHE_InvalidateByRange(addr, length);


  for (int i=0; i<length; i++)
  {
    p_data[i] = p_byte[i];
  }

  return ret;
}







#ifdef _USE_HW_CMDIF
void flashCmdifInit(void)
{
  cmdifAdd("flash", flashCmdif);
}

void flashCmdif(void)
{
  bool ret = true;
  uint32_t i;
  uint32_t addr;
  uint32_t length;
  uint8_t  data;
  uint32_t pre_time;
  bool flash_ret;


  if (cmdifGetParamCnt() == 1)
  {
    if(cmdifHasString("info", 0) == true)
    {
      cmdifPrintf("flash init  : %d\n", is_init);
      cmdifPrintf("flash addr  : 0x%X\n", 0x60000000);
    }
    else
    {
      ret = false;
    }
  }
  else if (cmdifGetParamCnt() == 3)
  {
    if(cmdifHasString("read", 0) == true)
    {
      addr   = (uint32_t)cmdifGetParam(1);
      length = (uint32_t)cmdifGetParam(2);

      for (i=0; i<length; i++)
      {
        flash_ret = flashRead(addr+i, &data, 1);

        if (flash_ret == true)
        {
          cmdifPrintf( "addr : 0x%X\t 0x%02X\n", addr+i, data);
        }
        else
        {
          cmdifPrintf( "addr : 0x%X\t Fail\n", addr+i);
        }
      }
    }
    else if(cmdifHasString("erase", 0) == true)
    {
      addr   = (uint32_t)cmdifGetParam(1);
      length = (uint32_t)cmdifGetParam(2);

      pre_time = millis();
      flash_ret = flashErase(addr, length);

      cmdifPrintf( "addr : 0x%X\t len : %d %d ms\n", addr, length, (millis()-pre_time));
      if (flash_ret)
      {
        cmdifPrintf("OK\n");
      }
      else
      {
        cmdifPrintf("FAIL\n");
      }
    }
    else if(cmdifHasString("write", 0) == true)
    {
      addr = (uint32_t)cmdifGetParam(1);
      data = (uint8_t )cmdifGetParam(2);

      pre_time = millis();
      flash_ret = flashWrite(addr, &data, 1);

      cmdifPrintf( "addr : 0x%X\t 0x%02X %d ms\n", addr, data, millis()-pre_time);
      if (flash_ret)
      {
        cmdifPrintf("OK\n");
      }
      else
      {
        cmdifPrintf("FAIL\n");
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
    cmdifPrintf( "flash info\n");
    cmdifPrintf( "flash read  [addr] [length]\n");
    cmdifPrintf( "flash erase [addr] [length]\n");
    cmdifPrintf( "flash write [addr] [data]\n");
  }

}
#endif












void     flexspi_nor_flash_init(FLEXSPI_Type *base);
status_t flexspi_nor_enable_quad_mode(FLEXSPI_Type *base);
status_t flexspi_nor_get_vendor_id(FLEXSPI_Type *base, uint8_t *vendorId);
status_t flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address);
status_t flexspi_nor_flash_page_program(FLEXSPI_Type *base, uint32_t dstAddr, const uint32_t *src);


bool _flashInit(void)
{
  status_t status;
  uint8_t device_id[3];

  flexspi_nor_flash_init(FLEXSPI);


  /* Enter quad mode. */
  status = flexspi_nor_enable_quad_mode(FLEXSPI);
  if (status != kStatus_Success)
  {
    return false;
  }

  status = flexspi_nor_get_vendor_id(FLEXSPI, device_id);
  if (status != kStatus_Success)
  {
    return false;
  }

  if (device_id[0] == 0x9D && device_id[1] == 0x70 && device_id[2] == 0x17)
  {
  }
  else
  {
    return false;
  }

  return true;
}

bool _flashEraseSector(uint32_t start_sector,  uint32_t sector_cnt)
{
  bool ret = true;
  status_t status;
  uint32_t start_addr;


  start_addr = start_sector * FLASH_SECTOR_SIZE;


  for (int i=0; i<sector_cnt; i++)
  {
    status = flexspi_nor_flash_erase_sector(FLEXSPI, start_addr + (i*FLASH_SECTOR_SIZE));

    if (status != kStatus_Success)
    {
      ret = false;
      break;
    }
  }

  DCACHE_InvalidateByRange(FLASH_ADDR_OFFSET + start_addr, sector_cnt * FLASH_SECTOR_SIZE);

  return ret;
}

bool _flashWritePage(uint32_t addr, uint32_t buf_addr)
{
  uint32_t buf[FLASH_PAGE_SIZE/4];
  uint8_t *p_dst;
  uint8_t *p_src;
  bool ret = true;
  status_t status;


  p_dst = (uint8_t *)buf;
  p_src = (uint8_t *)buf_addr;

  for (int i=0; i<FLASH_PAGE_SIZE; i++)
  {
    p_dst[i] = p_src[i];
  }

  status = flexspi_nor_flash_page_program(FLEXSPI, addr-FLASH_ADDR_OFFSET, buf);
  if (status != kStatus_Success)
  {
    ret = false;
  }

  DCACHE_InvalidateByRange(addr, FLASH_PAGE_SIZE);


  return ret;
}





#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL         7
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST           13
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD      0
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS          1
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE         2
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR         3
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE  6
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD    4
#define NOR_CMD_LUT_SEQ_IDX_READID              8
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG      9
#define NOR_CMD_LUT_SEQ_IDX_ENTERQPI            10
#define NOR_CMD_LUT_SEQ_IDX_EXITQPI             11
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG       12
#define NOR_CMD_LUT_SEQ_IDX_ERASECHIP           5

#define CUSTOM_LUT_LENGTH         60
#define FLASH_QUAD_ENABLE         0x40
#define FLASH_BUSY_STATUS_POL     1
#define FLASH_BUSY_STATUS_OFFSET  0



flexspi_device_config_t deviceconfig = {
    .flexspiRootClk       = 120000000,
    .flashSize            = FLASH_MAX_SIZE/1024,
    .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval           = 2,
    .CSHoldTime           = 3,
    .CSSetupTime          = 3,
    .dataValidTime        = 0,
    .columnspace          = 0,
    .enableWordAddress    = 0,
    .AWRSeqIndex          = 0,
    .AWRSeqNumber         = 0,
    .ARDSeqIndex          = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD,
    .ARDSeqNumber         = 1,
    .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};

const uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
    /* Normal read mode -SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x03, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Fast read mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0B, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Fast read quad mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x06, kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x04),

    /* Read extend parameters */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x81, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Write Enable */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Erase Sector  */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xD7, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

    /* Page Program - single mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x02, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Page Program - quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read ID */
    [4 * NOR_CMD_LUT_SEQ_IDX_READID] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enable Quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x01, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enter QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_ENTERQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x35, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Exit QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_EXITQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_4PAD, 0xF5, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read status register */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Erase whole chip */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASECHIP] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xC7, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),
};




status_t flexspi_nor_write_enable(FLEXSPI_Type *base, uint32_t baseAddr)
{
  flexspi_transfer_t flashXfer;
  status_t status;

  /* Write enable */
  flashXfer.deviceAddress = baseAddr;
  flashXfer.port          = kFLEXSPI_PortA1;
  flashXfer.cmdType       = kFLEXSPI_Command;
  flashXfer.SeqNumber     = 1;
  flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;

  status = FLEXSPI_TransferBlocking(base, &flashXfer);

  return status;
}

status_t flexspi_nor_wait_bus_busy(FLEXSPI_Type *base)
{
  /* Wait status ready. */
  bool isBusy;
  uint32_t readValue;
  status_t status;
  flexspi_transfer_t flashXfer;

  flashXfer.deviceAddress = 0;
  flashXfer.port          = kFLEXSPI_PortA1;
  flashXfer.cmdType       = kFLEXSPI_Read;
  flashXfer.SeqNumber     = 1;
  flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READSTATUSREG;
  flashXfer.data          = &readValue;
  flashXfer.dataSize      = 1;

  do
  {
      status = FLEXSPI_TransferBlocking(base, &flashXfer);

      if (status != kStatus_Success)
      {
          return status;
      }
      if (FLASH_BUSY_STATUS_POL)
      {
          if (readValue & (1U << FLASH_BUSY_STATUS_OFFSET))
          {
              isBusy = true;
          }
          else
          {
              isBusy = false;
          }
      }
      else
      {
          if (readValue & (1U << FLASH_BUSY_STATUS_OFFSET))
          {
              isBusy = false;
          }
          else
          {
              isBusy = true;
          }
      }

  } while (isBusy);

  return status;
}

status_t flexspi_nor_enable_quad_mode(FLEXSPI_Type *base)
{
  flexspi_transfer_t flashXfer;
  status_t status;
  uint32_t writeValue = FLASH_QUAD_ENABLE;

  /* Write enable */
  status = flexspi_nor_write_enable(base, 0);

  if (status != kStatus_Success)
  {
      return status;
  }

  /* Enable quad mode. */
  flashXfer.deviceAddress = 0;
  flashXfer.port          = kFLEXSPI_PortA1;
  flashXfer.cmdType       = kFLEXSPI_Write;
  flashXfer.SeqNumber     = 1;
  flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG;
  flashXfer.data          = &writeValue;
  flashXfer.dataSize      = 1;

  status = FLEXSPI_TransferBlocking(base, &flashXfer);
  if (status != kStatus_Success)
  {
      return status;
  }

  status = flexspi_nor_wait_bus_busy(base);

  /* Do software reset. */
  FLEXSPI_SoftwareReset(base);

  return status;
}

status_t flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address)
{
  status_t status;
  flexspi_transfer_t flashXfer;

  /* Write enable */
  flashXfer.deviceAddress = address;
  flashXfer.port          = kFLEXSPI_PortA1;
  flashXfer.cmdType       = kFLEXSPI_Command;
  flashXfer.SeqNumber     = 1;
  flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;

  status = FLEXSPI_TransferBlocking(base, &flashXfer);

  if (status != kStatus_Success)
  {
      return status;
  }

  flashXfer.deviceAddress = address;
  flashXfer.port          = kFLEXSPI_PortA1;
  flashXfer.cmdType       = kFLEXSPI_Command;
  flashXfer.SeqNumber     = 1;
  flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;
  status                  = FLEXSPI_TransferBlocking(base, &flashXfer);

  if (status != kStatus_Success)
  {
      return status;
  }

  status = flexspi_nor_wait_bus_busy(base);

  /* Do software reset. */
  FLEXSPI_SoftwareReset(base);

  return status;
}

status_t flexspi_nor_flash_program(FLEXSPI_Type *base, uint32_t dstAddr, const uint32_t *src, uint32_t length)
{
  status_t status;
  flexspi_transfer_t flashXfer;

  /* Write enable */
  status = flexspi_nor_write_enable(base, dstAddr);

  if (status != kStatus_Success)
  {
      return status;
  }

  /* Prepare page program command */
  flashXfer.deviceAddress = dstAddr;
  flashXfer.port          = kFLEXSPI_PortA1;
  flashXfer.cmdType       = kFLEXSPI_Write;
  flashXfer.SeqNumber     = 1;
  flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD;
  flashXfer.data          = (uint32_t *)src;
  flashXfer.dataSize      = length;
  status                  = FLEXSPI_TransferBlocking(base, &flashXfer);

  if (status != kStatus_Success)
  {
      return status;
  }

  status = flexspi_nor_wait_bus_busy(base);

  /* Do software reset. */
#if defined(FSL_FEATURE_SOC_OTFAD_COUNT)
  base->AHBCR |= FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK;
  base->AHBCR &= ~(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK);
#else
  FLEXSPI_SoftwareReset(base);
#endif

  return status;
}

status_t flexspi_nor_flash_page_program(FLEXSPI_Type *base, uint32_t dstAddr, const uint32_t *src)
{
  status_t status;
  flexspi_transfer_t flashXfer;

  /* Write enable */
  status = flexspi_nor_write_enable(base, dstAddr);

  if (status != kStatus_Success)
  {
      return status;
  }

  /* Prepare page program command */
  flashXfer.deviceAddress = dstAddr;
  flashXfer.port          = kFLEXSPI_PortA1;
  flashXfer.cmdType       = kFLEXSPI_Write;
  flashXfer.SeqNumber     = 1;
  flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD;
  flashXfer.data          = (uint32_t *)src;
  flashXfer.dataSize      = FLASH_PAGE_SIZE;
  status                  = FLEXSPI_TransferBlocking(base, &flashXfer);

  if (status != kStatus_Success)
  {
      return status;
  }

  status = flexspi_nor_wait_bus_busy(base);

  /* Do software reset. */
#if defined(FSL_FEATURE_SOC_OTFAD_COUNT)
  base->AHBCR |= FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK;
  base->AHBCR &= ~(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK);
#else
  FLEXSPI_SoftwareReset(base);
#endif

  return status;
}

status_t flexspi_nor_get_vendor_id(FLEXSPI_Type *base, uint8_t *vendorId)
{
  uint32_t temp;
  flexspi_transfer_t flashXfer;
  flashXfer.deviceAddress = 0;
  flashXfer.port          = kFLEXSPI_PortA1;
  flashXfer.cmdType       = kFLEXSPI_Read;
  flashXfer.SeqNumber     = 1;
  flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READID;
  flashXfer.data          = &temp;
  flashXfer.dataSize      = 3;

  status_t status = FLEXSPI_TransferBlocking(base, &flashXfer);

  vendorId[0] = temp;
  vendorId[1] = temp>>8;
  vendorId[2] = temp>>16;

  /* Do software reset. */
#if defined(FSL_FEATURE_SOC_OTFAD_COUNT)
  base->AHBCR |= FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK;
  base->AHBCR &= ~(FLEXSPI_AHBCR_CLRAHBRXBUF_MASK | FLEXSPI_AHBCR_CLRAHBTXBUF_MASK);
#else
  FLEXSPI_SoftwareReset(base);
#endif

  return status;
}

status_t flexspi_nor_erase_chip(FLEXSPI_Type *base)
{
  status_t status;
  flexspi_transfer_t flashXfer;

  /* Write enable */
  status = flexspi_nor_write_enable(base, 0);

  if (status != kStatus_Success)
  {
      return status;
  }

  flashXfer.deviceAddress = 0;
  flashXfer.port          = kFLEXSPI_PortA1;
  flashXfer.cmdType       = kFLEXSPI_Command;
  flashXfer.SeqNumber     = 1;
  flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASECHIP;

  status = FLEXSPI_TransferBlocking(base, &flashXfer);

  if (status != kStatus_Success)
  {
      return status;
  }

  status = flexspi_nor_wait_bus_busy(base);

  return status;
}

void flexspi_nor_flash_init(FLEXSPI_Type *base)
{
  flexspi_config_t config;

  /*Get FLEXSPI default settings and configure the flexspi. */
  FLEXSPI_GetDefaultConfig(&config);

  /*Set AHB buffer size for reading data through AHB bus. */
  config.ahbConfig.enableAHBPrefetch    = true;
  config.ahbConfig.enableAHBBufferable  = true;
  config.ahbConfig.enableReadAddressOpt = true;
  config.ahbConfig.enableAHBCachable    = true;
  config.rxSampleClock                  = kFLEXSPI_ReadSampleClkLoopbackFromDqsPad;
  FLEXSPI_Init(base, &config);

  /* Configure flash settings according to serial flash feature. */
  FLEXSPI_SetFlashConfig(base, &deviceconfig, kFLEXSPI_PortA1);

  /* Update LUT table. */
  FLEXSPI_UpdateLUT(base, 0, customLUT, CUSTOM_LUT_LENGTH);

  /* Do software reset. */
  FLEXSPI_SoftwareReset(base);
}

