/*
 * hw_def.h
 *
 *  Created on: 2020. 3. 11.
 *      Author: Baram
 */

#ifndef SRC_HW_HW_DEF_H_
#define SRC_HW_HW_DEF_H_


#include "def.h"
#include "bsp.h"


#define _USE_HW_RESET
#define _USE_HW_MICROS
#define _USE_HW_CLOCKS
#define _USE_HW_FLASH


#define _USE_HW_LED
#define      HW_LED_MAX_CH          3

#define _USE_HW_VCP

#define _USE_HW_UART
#define      HW_UART_MAX_CH         2

#define _USE_HW_SWTIMER
#define      HW_SWTIMER_MAX_CH      8

#define _USE_HW_BUTTON
#define      HW_BUTTON_MAX_CH       1

#define _USE_HW_GPIO
#define      HW_GPIO_MAX_CH         1

#define _USE_HW_FILES
#define _USE_HW_FATFS
#define _USE_HW_SD
#define      HW_SD_PIN_DETECTED     0
#define      HW_SD_PIN_PWREN       -1


#define _USE_HW_CMDIF
#define      HW_CMDIF_LIST_MAX              32
#define      HW_CMDIF_CMD_STR_MAX           16
#define      HW_CMDIF_CMD_BUF_LENGTH        128


#define _USE_HW_CMD
#define      HW_CMD_MAX_DATA_LENGTH         2048



#define _USE_HW_SDRAM
#define      HW_SDRAM_MEM_ADDR      0x80000000
#define      HW_SDRAM_MEM_SIZE      (32*1024*1024)



#define FLASH_ADDR_TAG                0x60080000
#define FLASH_ADDR_FW                 0x60080400

#define FLASH_ADDR_START              0x60080000
#define FLASH_ADDR_END                (FLASH_ADDR_START + 2*1024*1024)


#endif /* SRC_HW_HW_DEF_H_ */
