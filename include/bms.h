#ifndef BMS_H
#define BMS_H

#include <stdint.h>

#define BMS_N_CELL          16
#define BMS_TICK_MS         10

#define BMS_CELL_OV_MV      3650
#define BMS_CELL_OV_REL_MV  3500
#define BMS_CELL_UV_MV      2500
#define BMS_CELL_UV_REL_MV  2800
#define BMS_CV_MV           3450    /* inverter charge-voltage target per cell */

/* Charge is stricter than discharge (LFP plating / electrolyte). */
#define BMS_CHG_OT_DC       500     /* 50.0 C */
#define BMS_CHG_OT_REL_DC   450
#define BMS_OT_DC           600     /* 60.0 C — pack too hot to discharge */
#define BMS_OT_REL_DC       500
#define BMS_CHG_UT_DC       0       /*  0.0 C — no charge (plating) */
#define BMS_CHG_UT_REL_DC   50
#define BMS_UT_DC           -200    /* -20.0 C — pack too cold to discharge */
#define BMS_UT_REL_DC       -100
/* Soaked pack. 40 A below 0 C is too much at -10 C. Hold 20 A until -5 C.
   The same -10 C point is where a rest voltage must not rewrite SOC. */
#define BMS_DSG_COLD_DC     -100    /* -10.0 C */
#define BMS_DSG_COLD_REL_DC -50     /* -5.0 C */

#define BMS_OCC_MA          120000L
#define BMS_OCD_MA          150000L
#define BMS_OC_REL_MA       20000L
#define BMS_DIFF_MV         400
#define BMS_DIFF_REL_MV     200
#define BMS_OW_LO_MV        500     /* open / missing sense wire */
#define BMS_OW_HI_MV        4800
#define BMS_BAL_DV_MV       25
#define BMS_BAL_MIN_MV      3400
#define BMS_BAL_MAX         4       /* bleed resistors: at most 4 cells at once */
#define BMS_REST_MA         500
#define BMS_LEAK_MA         2000    /* 2 A, in mA, through a FET commanded open */
#define BMS_REST_MS         30000
#define BMS_DEB_TICKS       20      /* 200 ms */
/* Capacity is learned only from rest snaps at a mild temperature.
   Cold capacity is not fade. 15.0 °C .. 40.0 °C. */
#define BMS_LEARN_TMIN_DC   150
#define BMS_LEARN_TMAX_DC   400

#define BMS_FLT_OV     (1u << 0)
#define BMS_FLT_UV     (1u << 1)
#define BMS_FLT_OCC    (1u << 2)
#define BMS_FLT_OCD    (1u << 3)
#define BMS_FLT_OT     (1u << 4)
#define BMS_FLT_UT     (1u << 5)
#define BMS_FLT_DIFF   (1u << 6)
#define BMS_FLT_OW     (1u << 7)
#define BMS_FLT_LEAK   (1u << 8)    /* current through a commanded-open FET */

#define BMS_ST_CHG_MOS (1u << 0)
#define BMS_ST_DSG_MOS (1u << 1)
#define BMS_ST_BAL     (1u << 2)
#define BMS_ST_REST    (1u << 3)

#define BMS_WRN_OV     (1u << 0)    /* vmax >= 3.50 V */
#define BMS_WRN_UV     (1u << 1)    /* vmin <= 2.80 V */
#define BMS_WRN_OT     (1u << 2)    /* tmax >= 45 C */
#define BMS_WRN_UT     (1u << 3)    /* tmin <= 5 C */
#define BMS_WRN_OCC    (1u << 4)
#define BMS_WRN_OCD    (1u << 5)
#define BMS_WRN_DIFF   (1u << 6)
#define BMS_WRN_LOW    (1u << 7)    /* SOC <= 10 % */
#define BMS_WRN_HIGH   (1u << 8)    /* SOC >= 95 % */

#define BMS_FLG_FULL   (1u << 0)    /* charge complete, latched through a current spike */
#define BMS_FLG_EMPTY  (1u << 1)
#define BMS_FLG_HEAT   (1u << 2)    /* heater: coldest <= 5 C and hottest < 45 C */

typedef enum {
    BMS_IDLE = 0,
    BMS_CHARGE,
    BMS_DISCHARGE,
    BMS_PROTECT
} bms_mode_t;

typedef struct {
    int16_t  cell_mv[BMS_N_CELL];
    int32_t  pack_ma;             /* discharge positive */
    int16_t  t_dC[4];             /* 0.1 C. Unused / open = INT16_MIN. 0 means 0.0 C. */
} bms_sample_t;

typedef struct {
    uint16_t fault;
    uint16_t warn;
    uint16_t flags;
    uint16_t status;
    bms_mode_t mode;
    uint16_t soc_x10;             /* 0..1000 = 0.0% .. 100.0% */
    uint16_t remain_ah_x10;       /* 1000 = 100.0 Ah */
    uint16_t cycles;
    int32_t  pack_mv;
    int32_t  pack_ma;
    int16_t  t_max_dC;
    int16_t  t_min_dC;
    int16_t  v_max_mv;
    int16_t  v_min_mv;
    uint8_t  i_hi;                /* 1..N */
    uint8_t  i_lo;
    uint16_t bal_mask;
    uint16_t allow_chg_a;         /* 0 = do not charge */
    uint16_t allow_dsg_a;
    uint16_t max_chg_v_x10;       /* inverter CV, 0.1 V */
    uint16_t min_dsg_v_x10;       /* inverter cutoff, 0.1 V */
    uint32_t cap_mah;             /* usable mAh. Coulomb count and remain Ah */
    uint32_t cap_nom_mah;         /* nameplate, frozen at init. Cycles use this */
    int16_t  cell_mv[BMS_N_CELL];
    uint8_t  db_ov, db_uv, db_occ, db_ocd, db_ot, db_ut, db_diff, db_ow, db_leak;
    /* Temperature lockouts. Set only by that limit, cleared only at its
       release. Another fault opening the FET must not start the wait. */
    uint8_t  hold_chg_ot, hold_chg_ut, hold_dsg_ot, hold_dsg_ut;
    uint8_t  hold_dsg_cold;       /* 20 A discharge from -10 C until -5 C */
    uint8_t  rest_snap;           /* 1 after this rest has applied an OCV snap */
    uint32_t rest_ms;
    int64_t  soc_resid;
    int64_t  cyc_resid;
    int64_t  learn_resid;         /* mA·ms since the last end-of-curve anchor */
    uint16_t learn_soc;           /* soc_x10 at that anchor */
    uint8_t  learn_end;           /* 1 below 3.20 V, 2 above 3.40 V, 0 none */
} bms_t;

void bms_init(bms_t *b, uint32_t cap_mah, uint16_t soc_x10);
void bms_tick(bms_t *b);          /* call every BMS_TICK_MS */
void bms_clear_faults(bms_t *b);  /* re-sample; clear bits past release. Does not trip. */

#endif
