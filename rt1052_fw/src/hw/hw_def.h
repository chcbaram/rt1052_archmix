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




#define _USE_HW_LED
#define      HW_LED_MAX_CH          3

#define _USE_HW_UART
#define      HW_UART_MAX_CH         1

#define _USE_HW_SWTIMER
#define      HW_SWTIMER_MAX_CH      8

#define _USE_HW_BUTTON
#define      HW_BUTTON_MAX_CH       1


#define _USE_HW_CMDIF
#define      HW_CMDIF_LIST_MAX              32
#define      HW_CMDIF_CMD_STR_MAX           16
#define      HW_CMDIF_CMD_BUF_LENGTH        128



#endif /* SRC_HW_HW_DEF_H_ */
