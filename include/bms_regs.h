#ifndef BMS_REGS_H
#define BMS_REGS_H

#include <stdint.h>
#include "bms.h"

/* 16-bit holding registers for Modbus or a CAN PDO unpacker.
   Inverter / PCS reads these. Do not put consumer UI here. */

#define BMS_REG_N  48

#define REG_PROTO       0
#define REG_NCELL       1
#define REG_STATUS      2
#define REG_FAULT       3
#define REG_MODE        4
#define REG_SOC_X10     5
#define REG_PACK_V_X10  6   /* 0.1 V */
#define REG_I_HI        7   /* signed mA; (int32_t)(((uint32_t)hi << 16) | lo) */
#define REG_I_LO        8
#define REG_TMAX_DC     9   /* signed 0.1 C */
#define REG_TMIN_DC     10
#define REG_VMAX_MV     11
#define REG_VMIN_MV     12
#define REG_I_HI_CELL   13
#define REG_I_LO_CELL   14
#define REG_ALLOW_CHG_A 15
#define REG_ALLOW_DSG_A 16
#define REG_BAL_MASK    17
#define REG_WARN        18
#define REG_FLAGS       19  /* bit0 full, 1 empty, 2 heat */
#define REG_CYCLES      20
#define REG_REMAIN_AH   21  /* 0.1 Ah */
#define REG_MAX_CHG_V   22  /* 0.1 V, inverter CV */
#define REG_MIN_DSG_V   23  /* 0.1 V, inverter cutoff */
#define REG_CELL0       32  /* 32..47 cell mV */

/* Fill from the same context as bms_tick(). Do not read `reg` from an ISR
   while this runs — pack_mA is two words and would tear. */
void bms_regs_fill(const bms_t *b, uint16_t *reg);

#endif
