#include "bms_regs.h"

static uint16_t u16_mv(int16_t v)
{
    return v < 0 ? 0 : (uint16_t)v;
}

void bms_regs_fill(const bms_t *b, uint16_t *reg)
{
    int i;
    int32_t pv;
    if (!b || !reg) return;
    for (i = 0; i < BMS_REG_N; i++) reg[i] = 0;
    reg[REG_PROTO] = 1;
    reg[REG_NCELL] = BMS_N_CELL;
    reg[REG_STATUS] = b->status;
    reg[REG_FAULT] = b->fault;
    reg[REG_MODE] = (uint16_t)b->mode;
    reg[REG_SOC_X10] = b->soc_x10;
    pv = b->pack_mv / 100; /* mV -> 0.1 V */
    if (pv < 0) pv = 0;
    if (pv > 65535) pv = 65535;
    reg[REG_PACK_V_X10] = (uint16_t)pv;
    {
        /* Two's complement 32-bit, high word then low. Charge current is negative. */
        uint32_t u = (uint32_t)b->pack_ma;
        reg[REG_I_HI] = (uint16_t)(u >> 16);
        reg[REG_I_LO] = (uint16_t)(u & 0xFFFFu);
    }
    /* Temperatures are signed 0.1 C in two's complement. */
    reg[REG_TMAX_DC] = (uint16_t)b->t_max_dC;
    reg[REG_TMIN_DC] = (uint16_t)b->t_min_dC;
    reg[REG_VMAX_MV] = u16_mv(b->v_max_mv);
    reg[REG_VMIN_MV] = u16_mv(b->v_min_mv);
    reg[REG_I_HI_CELL] = b->i_hi;
    reg[REG_I_LO_CELL] = b->i_lo;
    reg[REG_ALLOW_CHG_A] = b->allow_chg_a;
    reg[REG_ALLOW_DSG_A] = b->allow_dsg_a;
    reg[REG_BAL_MASK] = b->bal_mask;
    reg[REG_WARN] = b->warn;
    reg[REG_FLAGS] = b->flags;
    reg[REG_CYCLES] = b->cycles;
    reg[REG_REMAIN_AH] = b->remain_ah_x10;
    reg[REG_MAX_CHG_V] = b->max_chg_v_x10;
    reg[REG_MIN_DSG_V] = b->min_dsg_v_x10;
    for (i = 0; i < BMS_N_CELL; i++)
        reg[REG_CELL0 + i] = u16_mv(b->cell_mv[i]);
}
