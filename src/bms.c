#include "bms.h"
#include "bms_hal.h"
#include <limits.h>

static int16_t smax(int16_t a, int16_t b) { return a > b ? a : b; }
static int16_t smin(int16_t a, int16_t b) { return a < b ? a : b; }
static uint16_t umin(uint16_t a, uint16_t b) { return a < b ? a : b; }

static int32_t iabs_sat(int32_t x)
{
    if (x == INT32_MIN) return INT32_MAX;
    return x < 0 ? -x : x;
}

/* LFP rest voltage vs SOC 0..100. Only used after 30 s idle at the ends. */
static const int16_t OCV_SOC[] = {0, 2, 5, 10, 20, 40, 60, 80, 90, 95, 98, 100};
static const int16_t OCV_MV[]  = {2500, 2900, 3100, 3200, 3240, 3280, 3300, 3330, 3345, 3400, 3500, 3650};

static uint16_t soc_from_mv(int16_t mv)
{
    if (mv <= OCV_MV[0]) return 0;
    if (mv >= OCV_MV[11]) return 1000;
    int i;
    for (i = 0; i < 11; i++) {
        if (mv >= OCV_MV[i] && mv <= OCV_MV[i + 1]) {
            int d = OCV_MV[i + 1] - OCV_MV[i];
            int t = mv - OCV_MV[i];
            int soc = OCV_SOC[i] + (OCV_SOC[i + 1] - OCV_SOC[i]) * t / (d ? d : 1);
            return (uint16_t)(soc * 10);
        }
    }
    return 500;
}

static void pack_stats(bms_t *b, const bms_sample_t *s)
{
    int32_t sum = 0;
    int16_t vmax = s->cell_mv[0], vmin = s->cell_mv[0];
    uint8_t ihi = 0, ilo = 0;
    int i;
    for (i = 0; i < BMS_N_CELL; i++) {
        int16_t v = s->cell_mv[i];
        b->cell_mv[i] = v;
        sum += v;
        if (v > vmax) { vmax = v; ihi = (uint8_t)i; }
        if (v < vmin) { vmin = v; ilo = (uint8_t)i; }
    }
    b->pack_mv = sum;
    b->v_max_mv = vmax;
    b->v_min_mv = vmin;
    b->i_hi = (uint8_t)(ihi + 1);
    b->i_lo = (uint8_t)(ilo + 1);
    b->pack_ma = s->pack_ma;

    int16_t tmax = INT16_MIN, tmin = INT16_MAX;
    int n = 0;
    for (i = 0; i < 4; i++) {
        if (s->t_dC[i] == INT16_MIN) continue;
        tmax = smax(tmax, s->t_dC[i]);
        tmin = smin(tmin, s->t_dC[i]);
        n++;
    }
    if (n) {
        b->t_max_dC = tmax;
        b->t_min_dC = tmin;
    } else {
        /* No thermistor: fail-safe as over-temp so both FETs open. */
        b->t_max_dC = BMS_OT_DC;
        b->t_min_dC = BMS_OT_DC;
    }
}

/* Trip after DEB ticks in the set region. Clear as soon as analog is past release.
   Do not count 0..255 on the way up: that delayed recovery by seconds. */
static void trip_rel(uint8_t *db, int in_set, int in_rel, uint16_t *f, uint16_t bit)
{
    if (in_set) {
        if (*db < BMS_DEB_TICKS) (*db)++;
        if (*db >= BMS_DEB_TICKS) *f |= bit;
    } else {
        *db = 0;
        if (in_rel) *f &= (uint16_t)~bit;
    }
}

static int open_wire(const bms_t *b)
{
    int i;
    for (i = 0; i < BMS_N_CELL; i++) {
        if (b->cell_mv[i] < BMS_OW_LO_MV || b->cell_mv[i] > BMS_OW_HI_MV)
            return 1;
    }
    return 0;
}

static void protect(bms_t *b)
{
    uint16_t f = b->fault;
    int32_t dv = (int32_t)b->v_max_mv - (int32_t)b->v_min_mv;
    int ow = open_wire(b);

    trip_rel(&b->db_ov, b->v_max_mv >= BMS_CELL_OV_MV, b->v_max_mv <= BMS_CELL_OV_REL_MV, &f, BMS_FLT_OV);
    trip_rel(&b->db_uv, b->v_min_mv <= BMS_CELL_UV_MV, b->v_min_mv >= BMS_CELL_UV_REL_MV, &f, BMS_FLT_UV);
    trip_rel(&b->db_occ, b->pack_ma <= -BMS_OCC_MA, b->pack_ma > -BMS_OC_REL_MA, &f, BMS_FLT_OCC);
    trip_rel(&b->db_ocd, b->pack_ma >= BMS_OCD_MA, b->pack_ma < BMS_OC_REL_MA, &f, BMS_FLT_OCD);
    trip_rel(&b->db_ot, b->t_max_dC >= BMS_OT_DC, b->t_max_dC <= BMS_OT_REL_DC, &f, BMS_FLT_OT);
    trip_rel(&b->db_ut, b->t_min_dC <= BMS_UT_DC, b->t_min_dC >= BMS_UT_REL_DC, &f, BMS_FLT_UT);
    trip_rel(&b->db_diff, dv >= BMS_DIFF_MV, dv < BMS_DIFF_REL_MV, &f, BMS_FLT_DIFF);
    trip_rel(&b->db_ow, ow, !ow, &f, BMS_FLT_OW);

    b->fault = f;
}

static void sop(bms_t *b, int chg, int dsg)
{
    uint16_t ca = (uint16_t)(BMS_OCC_MA / 1000);
    uint16_t da = (uint16_t)(BMS_OCD_MA / 1000);
    int32_t ia = iabs_sat(b->pack_ma);
    int32_t cv, dv;

    if (!chg) ca = 0;
    else {
        if (b->v_max_mv >= BMS_CELL_OV_MV - 50) ca = umin(ca, 5);          /* 3.60 V */
        else if (b->v_max_mv >= BMS_CELL_OV_REL_MV) ca = umin(ca, 20);     /* 3.50 V */
        else if (b->v_max_mv >= BMS_CV_MV) ca = umin(ca, 60);              /* 3.45 V */

        if (b->t_min_dC <= BMS_CHG_UT_DC) ca = 0;
        else if (b->t_min_dC < 100) {                                     /* 0–10 C */
            uint16_t c02 = (uint16_t)(b->cap_mah / 5000u);               /* 0.2 C */
            if (!c02) c02 = 1;
            ca = umin(ca, c02);
        }

        if (b->t_max_dC >= BMS_CHG_OT_DC) ca = 0;
        else if (b->t_max_dC >= BMS_CHG_OT_REL_DC) ca = umin(ca, 20);

        if (b->soc_x10 >= 980) ca = umin(ca, 10);
        else if (b->soc_x10 >= 950) ca = umin(ca, 40);

        /* Charge complete: hold voltage, no more current. */
        if (b->v_max_mv >= BMS_CELL_OV_REL_MV && b->soc_x10 >= 950 && ia < 3000) ca = 0;
    }

    if (!dsg) da = 0;
    else {
        if (b->v_min_mv <= 2700) da = umin(da, 20);
        else if (b->v_min_mv <= 2900) da = umin(da, 60);

        if (b->t_min_dC <= BMS_UT_DC) da = 0;
        else if (b->t_min_dC < 0) da = umin(da, 40);

        if (b->t_max_dC >= BMS_OT_DC) da = 0;
        else if (b->t_max_dC >= BMS_OT_DC - 50) da = umin(da, 40);         /* 55 C */

        if (b->soc_x10 <= 30) da = umin(da, 20);
        else if (b->soc_x10 <= 100) da = umin(da, 60);
    }

    b->allow_chg_a = ca;
    b->allow_dsg_a = da;

    /* Closed-loop inverter CV / cutoff, 0.1 V. If one cell is already high, hold pack V. */
    cv = (int32_t)BMS_N_CELL * BMS_CV_MV / 100; /* 16 × 3.45 V → 552 */
    if (b->v_max_mv > BMS_CV_MV) {
        int32_t hold = b->pack_mv / 100;
        if (hold < cv) cv = hold;
    }
    if (cv < 0) cv = 0;
    if (cv > 65535) cv = 65535;
    b->max_chg_v_x10 = (uint16_t)cv;

    dv = (int32_t)BMS_N_CELL * BMS_CELL_UV_REL_MV / 100; /* 16 × 2.80 V → 448 */
    if (b->v_min_mv < BMS_CELL_UV_REL_MV) {
        int32_t hold = b->pack_mv / 100;
        if (hold > dv) dv = hold;
    }
    if (dv < 0) dv = 0;
    if (dv > 65535) dv = 65535;
    b->min_dsg_v_x10 = (uint16_t)dv;
}

static void mosfets(bms_t *b)
{
    int chg = 1, dsg = 1;
    int prev_chg = (b->status & BMS_ST_CHG_MOS) != 0;
    int prev_dsg = (b->status & BMS_ST_DSG_MOS) != 0;

    if (b->fault & (BMS_FLT_OV | BMS_FLT_OCC | BMS_FLT_OT | BMS_FLT_UT | BMS_FLT_DIFF | BMS_FLT_OW))
        chg = 0;
    if (b->fault & (BMS_FLT_UV | BMS_FLT_OCD | BMS_FLT_OT | BMS_FLT_UT | BMS_FLT_DIFF | BMS_FLT_OW))
        dsg = 0;

    /* Charge-only temperature: open the charge FET without a pack UT/OT fault. */
    if (b->t_max_dC >= BMS_CHG_OT_DC) chg = 0;
    else if (b->t_max_dC > BMS_CHG_OT_REL_DC && !prev_chg) chg = 0;
    if (b->t_min_dC <= BMS_CHG_UT_DC) chg = 0;
    else if (b->t_min_dC < BMS_CHG_UT_REL_DC && !prev_chg) chg = 0;

    if (b->t_max_dC >= BMS_OT_DC) dsg = 0;
    else if (b->t_max_dC > BMS_OT_REL_DC && !prev_dsg) dsg = 0;
    if (b->t_min_dC <= BMS_UT_DC) dsg = 0;
    else if (b->t_min_dC < BMS_UT_REL_DC && !prev_dsg) dsg = 0;

    if (chg) b->status |= BMS_ST_CHG_MOS; else b->status &= (uint16_t)~BMS_ST_CHG_MOS;
    if (dsg) b->status |= BMS_ST_DSG_MOS; else b->status &= (uint16_t)~BMS_ST_DSG_MOS;

    bms_hal_set_chg_mos(chg);
    bms_hal_set_dsg_mos(dsg);
    sop(b, chg, dsg);
}

static void soc_step(bms_t *b)
{
    int64_t step = (int64_t)b->cap_mah * 3600LL; /* mA*ms per 0.1% */
    if (step <= 0) return;
    b->soc_resid += (int64_t)b->pack_ma * BMS_TICK_MS;
    while (b->soc_resid >= step && b->soc_x10 > 0) {
        b->soc_x10--;
        b->soc_resid -= step;
    }
    while (b->soc_resid <= -step && b->soc_x10 < 1000) {
        b->soc_x10++;
        b->soc_resid += step;
    }
    /* At the ends, drop leftover. A current-sensor offset at 0 % / 100 %
       would otherwise need hours of opposite current before SOC moved. */
    if (b->soc_x10 == 0 && b->soc_resid > 0) b->soc_resid = 0;
    if (b->soc_x10 == 1000 && b->soc_resid < 0) b->soc_resid = 0;

    /* One cycle = one full capacity actually leaving the pack. RAM only. */
    if (b->pack_ma > BMS_REST_MA && (b->status & BMS_ST_DSG_MOS)) {
        int64_t one = step * 1000;
        b->cyc_resid += (int64_t)b->pack_ma * BMS_TICK_MS;
        while (one > 0 && b->cyc_resid >= one) {
            b->cyc_resid -= one;
            if (b->cycles < 65535) b->cycles++;
        }
    }
}

static void soc_rest(bms_t *b)
{
    int32_t ia = iabs_sat(b->pack_ma);
    if (ia < BMS_REST_MA) {
        if (b->rest_ms < 1000000u) b->rest_ms += BMS_TICK_MS;
        b->status |= BMS_ST_REST;
    } else {
        b->rest_ms = 0;
        b->status &= (uint16_t)~BMS_ST_REST;
    }
    if (b->rest_ms < BMS_REST_MS) return;
    /* LFP mid-flat: only snap at the ends. Use the lowest cell (conservative). */
    if (b->v_min_mv < 3200 || b->v_min_mv > 3400)
        b->soc_x10 = soc_from_mv(b->v_min_mv);
}

static int wants_bal(const bms_sample_t *s, const bms_t *b, int i)
{
    return s->cell_mv[i] >= BMS_BAL_MIN_MV
        && (int32_t)s->cell_mv[i] >= (int32_t)b->v_min_mv + BMS_BAL_DV_MV;
}

static void balance(bms_t *b, const bms_sample_t *s)
{
    uint16_t mask = 0;
    int32_t spread = (int32_t)b->v_max_mv - (int32_t)b->v_min_mv;
    int idle_or_chg = b->pack_ma < 1000;
    int k;

    /* Bleed resistors dump heat into the pack — stop if already warm, and
       only the highest few cells, not every cell above the floor. */
    if (idle_or_chg && spread >= BMS_BAL_DV_MV && b->v_max_mv >= BMS_BAL_MIN_MV
        && b->t_max_dC < BMS_CHG_OT_DC) {
        for (k = 0; k < BMS_BAL_MAX; k++) {
            int best = -1, i;
            int16_t bestv = INT16_MIN;
            for (i = 0; i < BMS_N_CELL; i++) {
                if (mask & (uint16_t)(1u << i)) continue;
                if (!wants_bal(s, b, i)) continue;
                if (s->cell_mv[i] > bestv) {
                    bestv = s->cell_mv[i];
                    best = i;
                }
            }
            if (best < 0) break;
            mask |= (uint16_t)(1u << best);
        }
    }
    b->bal_mask = mask;
    if (mask) b->status |= BMS_ST_BAL; else b->status &= (uint16_t)~BMS_ST_BAL;
    bms_hal_set_balance(mask);
}

static void publish(bms_t *b)
{
    uint16_t w = 0, g = 0;
    int32_t dv = (int32_t)b->v_max_mv - (int32_t)b->v_min_mv;
    int32_t ia = iabs_sat(b->pack_ma);
    uint64_t rem;

    if (b->v_max_mv >= BMS_CELL_OV_REL_MV) w |= BMS_WRN_OV;
    if (b->v_min_mv <= BMS_CELL_UV_REL_MV) w |= BMS_WRN_UV;
    if (b->t_max_dC >= BMS_CHG_OT_REL_DC) w |= BMS_WRN_OT;
    if (b->t_min_dC <= BMS_CHG_UT_REL_DC) w |= BMS_WRN_UT;
    if (b->pack_ma <= -(BMS_OCC_MA * 8 / 10)) w |= BMS_WRN_OCC;
    if (b->pack_ma >= (BMS_OCD_MA * 8 / 10)) w |= BMS_WRN_OCD;
    if (dv >= BMS_DIFF_REL_MV) w |= BMS_WRN_DIFF;
    if (b->soc_x10 <= 100) w |= BMS_WRN_LOW;
    if (b->soc_x10 >= 950) w |= BMS_WRN_HIGH;
    b->warn = w;

    if (b->v_max_mv >= BMS_CELL_OV_REL_MV && b->soc_x10 >= 950 && ia < 3000) g |= BMS_FLG_FULL;
    if (b->v_min_mv <= BMS_CELL_UV_REL_MV || b->soc_x10 <= 50) g |= BMS_FLG_EMPTY;
    if (b->t_min_dC <= BMS_CHG_UT_REL_DC) g |= BMS_FLG_HEAT;
    b->flags = g;

    rem = (uint64_t)b->cap_mah * (uint64_t)b->soc_x10 / 100000ull;
    if (rem > 65535ull) rem = 65535ull;
    b->remain_ah_x10 = (uint16_t)rem;
}

static void mode_of(bms_t *b)
{
    if (b->fault) b->mode = BMS_PROTECT;
    else if (b->pack_ma < -500) b->mode = BMS_CHARGE;
    else if (b->pack_ma > 500) b->mode = BMS_DISCHARGE;
    else b->mode = BMS_IDLE;
}

void bms_init(bms_t *b, uint32_t cap_mah, uint16_t soc_x10)
{
    int i;
    if (!b) return;
    for (i = 0; i < (int)sizeof(*b); i++) ((uint8_t *)b)[i] = 0;
    b->cap_mah = cap_mah ? cap_mah : 200000;
    b->soc_x10 = soc_x10 > 1000 ? 1000 : soc_x10;
    b->status = BMS_ST_CHG_MOS | BMS_ST_DSG_MOS;
    bms_hal_set_chg_mos(1);
    bms_hal_set_dsg_mos(1);
    bms_hal_set_balance(0);
}

void bms_tick(bms_t *b)
{
    bms_sample_t s;
    if (!b) return;
    bms_hal_sample(&s);
    pack_stats(b, &s);
    protect(b);
    mosfets(b);
    soc_step(b);
    soc_rest(b);
    balance(b, &s);
    publish(b);
    mode_of(b);
}

void bms_clear_faults(bms_t *b)
{
    bms_sample_t s;
    uint16_t f;
    int32_t dv;
    int ow;
    if (!b) return;
    /* Re-sample and drop bits that are past release. Do not advance debounce
       — calling this in a tight loop must not trip OV in microseconds. */
    bms_hal_sample(&s);
    pack_stats(b, &s);
    f = b->fault;
    dv = (int32_t)b->v_max_mv - (int32_t)b->v_min_mv;
    ow = open_wire(b);
    if (b->v_max_mv <= BMS_CELL_OV_REL_MV) { b->db_ov = 0; f &= (uint16_t)~BMS_FLT_OV; }
    if (b->v_min_mv >= BMS_CELL_UV_REL_MV) { b->db_uv = 0; f &= (uint16_t)~BMS_FLT_UV; }
    if (b->pack_ma > -BMS_OC_REL_MA) { b->db_occ = 0; f &= (uint16_t)~BMS_FLT_OCC; }
    if (b->pack_ma < BMS_OC_REL_MA) { b->db_ocd = 0; f &= (uint16_t)~BMS_FLT_OCD; }
    if (b->t_max_dC <= BMS_OT_REL_DC) { b->db_ot = 0; f &= (uint16_t)~BMS_FLT_OT; }
    if (b->t_min_dC >= BMS_UT_REL_DC) { b->db_ut = 0; f &= (uint16_t)~BMS_FLT_UT; }
    if (dv < BMS_DIFF_REL_MV) { b->db_diff = 0; f &= (uint16_t)~BMS_FLT_DIFF; }
    if (!ow) { b->db_ow = 0; f &= (uint16_t)~BMS_FLT_OW; }
    b->fault = f;
    mosfets(b);
    publish(b);
    mode_of(b);
}
