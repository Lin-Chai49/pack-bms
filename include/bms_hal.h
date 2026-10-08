#ifndef BMS_HAL_H
#define BMS_HAL_H

#include <stdint.h>
#include "bms.h"

/* Fill these on the MCU. The host test provides a fake AFE. */

void bms_hal_sample(bms_sample_t *out);
void bms_hal_set_chg_mos(int on);
void bms_hal_set_dsg_mos(int on);
void bms_hal_set_balance(uint16_t mask); /* bit i bleeds cell i. Zero for one tick after each 1.0 s */
uint32_t bms_hal_millis(void);           /* not used by the core; your loop may use it */

#endif
