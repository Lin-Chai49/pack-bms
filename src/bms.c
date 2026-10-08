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

/* Trip after DEB ticks in the set region. A tick in the hysteresis band sheds
   one count; it does not erase the wait. Past release, clear at once and
   start the wait over. Stop at DEB so the counter cannot wrap. */
static void trip_rel(uint8_t *db, int in_set, int in_rel, uint16_t *f, uint16_t bit)
{
    if (in_set) {
        if (*db < BMS_DEB_TICKS) (*db)++;
        if (*db >= BMS_DEB_TICKS) *f |= bit;
    } else if (in_rel) {
        *db = 0;
        *f &= (uint16_t)~bit;
    } else if (*db > 0) {
        (*db)--;
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

/* 2 A or more in a direction whose FET is commanded open. Milliamp earth
   leakage is below this shunt. A 0.5–2 A band sheds the wait instead of
   erasing it. Status is 0 until the first command; that gap is not a weld. */
static int leak_set(const bms_t *b)
{
    if (!b->leak_live) return 0;
    if (!(b->status & BMS_ST_CHG_MOS) && b->pack_ma <= -BMS_LEAK_MA) return 1;
    if (!(b->status & BMS_ST_DSG_MOS) && b->pack_ma >= BMS_LEAK_MA) return 1;
    return 0;
}

static int leak_rel(const bms_t *b)
{
    int chg_ok = (b->status & BMS_ST_CHG_MOS) || b->pack_ma > -BMS_REST_MA;
    int dsg_ok = (b->status & BMS_ST_DSG_MOS) || b->pack_ma < BMS_REST_MA;
    return chg_ok && dsg_ok;
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
    trip_rel(&b->db_diff, dv >= BMS_DIFF_MV, dv <= BMS_DIFF_REL_MV, &f, BMS_FLT_DIFF);
    trip_rel(&b->db_leak, leak_set(b), leak_rel(b), &f, BMS_FLT_LEAK);
    /* Open wire has no band between set and release. Count the bad samples
       up and the good ones back down, and only drop the bit at zero, so one
       in-range sample cannot close the FETs. */
    if (ow) {
        if (b->db_ow < BMS_DEB_TICKS) b->db_ow++;
        if (b->db_ow >= BMS_DEB_TICKS) f |= BMS_FLT_OW;
    } else if (b->db_ow > 0) {
        b->db_ow--;
        if (b->db_ow == 0) f &= (uint16_t)~BMS_FLT_OW;
    }

    b->fault = f;
}

/* True once the pack has finished charging, until the cell or SOC leaves.
   Current is only the way in: a spike above 3 A must not turn charge back on. */
static int charge_done(const bms_t *b)
{
    int32_t ia = iabs_sat(b->pack_ma);
    if (b->v_max_mv < BMS_CELL_OV_REL_MV || b->soc_x10 < 950) return 0;
    if (ia < 3000) return 1;
    return (b->flags & BMS_FLG_FULL) != 0;
}

static void sop(bms_t *b, int chg, int dsg)
{
    uint16_t ca = (uint16_t)(BMS_OCC_MA / 1000);
    uint16_t da = (uint16_t)(BMS_OCD_MA / 1000);
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

        if (charge_done(b)) ca = 0;
    }

    if (!dsg) da = 0;
    else {
        if (b->v_min_mv <= 2700) da = umin(da, 20);
        else if (b->v_min_mv <= 2900) da = umin(da, 60);

        if (b->t_min_dC <= BMS_UT_DC) da = 0;
        else if (b->hold_dsg_cold) da = umin(da, 20);
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

    /* 44.8 V until a cell sags through 2.80 V. Then hold the present pack
       voltage until that cell is back to 2.90 V, so a small rebound does
       not drop the cutoff and let the inverter start into the same cell. */
    dv = (int32_t)BMS_N_CELL * BMS_CELL_UV_REL_MV / 100; /* 16 × 2.80 V → 448 */
    {
        int floor = (int)dv;
        int sag = b->v_min_mv < BMS_CELL_UV_REL_MV
            || (b->min_dsg_v_x10 > (uint16_t)floor && b->v_min_mv < 2900);
        if (sag) {
            int32_t hold = b->pack_mv / 100;
            if (hold > dv) dv = hold;
        }
    }
    if (dv < 0) dv = 0;
    if (dv > 65535) dv = 65535;
    b->min_dsg_v_x10 = (uint16_t)dv;
}

/* Latch on `trip`, drop on `release`, hold through the band between them. */
static void hold_temp(uint8_t *hold, int trip, int release)
{
    if (trip) *hold = 1;
    else if (release) *hold = 0;
}

static void mosfets(bms_t *b)
{
    int chg = 1, dsg = 1;

    hold_temp(&b->hold_chg_ot, b->t_max_dC >= BMS_CHG_OT_DC, b->t_max_dC <= BMS_CHG_OT_REL_DC);
    hold_temp(&b->hold_chg_ut, b->t_min_dC <= BMS_CHG_UT_DC, b->t_min_dC >= BMS_CHG_UT_REL_DC);
    hold_temp(&b->hold_dsg_ot, b->t_max_dC >= BMS_OT_DC, b->t_max_dC <= BMS_OT_REL_DC);
    hold_temp(&b->hold_dsg_ut, b->t_min_dC <= BMS_UT_DC, b->t_min_dC >= BMS_UT_REL_DC);
    /* Current cap only. Does not open the discharge FET. */
    hold_temp(&b->hold_dsg_cold, b->t_min_dC <= BMS_DSG_COLD_DC,
              b->t_min_dC >= BMS_DSG_COLD_REL_DC);

    if (b->fault & (BMS_FLT_OV | BMS_FLT_OCC | BMS_FLT_OT | BMS_FLT_UT | BMS_FLT_DIFF | BMS_FLT_OW | BMS_FLT_LEAK))
        chg = 0;
    if (b->fault & (BMS_FLT_UV | BMS_FLT_OCD | BMS_FLT_OT | BMS_FLT_UT | BMS_FLT_DIFF | BMS_FLT_OW | BMS_FLT_LEAK))
        dsg = 0;

    /* Charge-only limits have no fault bit. Pack OT/UT also open immediately,
       before the 200 ms fault debounce, and stay open until their release. */
    if (b->hold_chg_ot || b->hold_chg_ut) chg = 0;
    if (b->hold_dsg_ot || b->hold_dsg_ut) dsg = 0;

    /* Over-current releases below 20 A. A welded FET can still be passing
       15 A. Reclosing here would zero the leak wait and hide that current.
       Keep the FET that was already open until the wait finishes or the
       current is back under 0.5 A. */
    if (b->leak_live && (b->db_leak || leak_set(b))) {
        if (!(b->status & BMS_ST_CHG_MOS)) chg = 0;
        if (!(b->status & BMS_ST_DSG_MOS)) dsg = 0;
    }

    if (chg) b->status |= BMS_ST_CHG_MOS; else b->status &= (uint16_t)~BMS_ST_CHG_MOS;
    if (dsg) b->status |= BMS_ST_DSG_MOS; else b->status &= (uint16_t)~BMS_ST_DSG_MOS;
    b->leak_live = 1;

    bms_hal_set_chg_mos(chg);
    bms_hal_set_dsg_mos(dsg);
    sop(b, chg, dsg);
}

/* Usable capacity stays inside half..full nameplate. A restored value
   outside that is a bad save, not a measurement. */
static void cap_hold(bms_t *b)
{
    uint32_t nom = b->cap_nom_mah;
    uint32_t floor;
    if (!nom) return;
    floor = nom / 2;
    if (!floor) floor = 1;
    if (b->cap_mah > nom) b->cap_mah = nom;
    else if (b->cap_mah < floor) b->cap_mah = floor;
}

static void soc_step(bms_t *b)
{
    int64_t step;
    int32_t ma = b->pack_ma;
    cap_hold(b);
    step = (int64_t)b->cap_mah * 3600LL; /* mA*ms per 0.1% */
    if (step <= 0) return;
    /* An open FET is not a path. A shunt offset during a latched fault
       must not walk SOC for hours. */
    if (ma < 0 && !(b->status & BMS_ST_CHG_MOS)) ma = 0;
    if (ma > 0 && !(b->status & BMS_ST_DSG_MOS)) ma = 0;
    b->soc_resid += (int64_t)ma * BMS_TICK_MS;
    b->learn_resid += (int64_t)ma * BMS_TICK_MS;
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

    /* One cycle = one nameplate discharged. Fade must not speed this up. */
    if (ma > BMS_REST_MA) {
        int64_t one = (int64_t)b->cap_nom_mah * 3600LL * 1000;
        b->cyc_resid += (int64_t)ma * BMS_TICK_MS;
        while (one > 0 && b->cyc_resid >= one) {
            b->cyc_resid -= one;
            if (b->cycles < 65535) b->cycles++;
        }
    }
}

static int learn_temp_ok(const bms_t *b)
{
    return b->t_min_dC >= BMS_LEARN_TMIN_DC && b->t_max_dC <= BMS_LEARN_TMAX_DC;
}

/* Scale the coulombs between two opposite rest snaps up to a full capacity
   and take one quarter of the error. One noisy stroke must not collapse
   the pack. Below half the nameplate is a bad measurement, not fade. */
static void learn_apply(bms_t *b, uint16_t soc)
{
    int32_t span = (int32_t)soc - (int32_t)b->learn_soc;
    uint64_t ah, meas;
    uint32_t nom;
    int64_t next;

    /* SOC up means charge went in (resid negative). The opposite pair is
       discharge. Absolute value would invent capacity from a voltage glitch. */
    if ((span > 0 && b->learn_resid > 0) || (span < 0 && b->learn_resid < 0))
        return;
    if (span < 0) span = -span;
    if (span < 800) return;
    nom = b->cap_nom_mah;
    if (nom < 2) return;
    if (b->learn_resid == INT64_MIN) return;
    ah = b->learn_resid < 0 ? (uint64_t)(-b->learn_resid) : (uint64_t)b->learn_resid;
    meas = ah / (3600ull * (uint64_t)span);
    if (meas > nom) meas = nom;
    if (meas < nom / 2) return;
    next = (int64_t)b->cap_mah + ((int64_t)meas - (int64_t)b->cap_mah) / 4;
    if (next < (int64_t)(nom / 2)) next = (int64_t)(nom / 2);
    if (next > (int64_t)nom) next = (int64_t)nom;
    b->cap_mah = (uint32_t)next;
}

static void learn_on_snap(bms_t *b, uint16_t soc)
{
    int end = b->v_min_mv < 3200 ? 1 : 2;
    if (learn_temp_ok(b) && b->learn_end && (int)b->learn_end != end)
        learn_apply(b, soc);
    if (learn_temp_ok(b)) {
        b->learn_end = (uint8_t)end;
        b->learn_soc = soc;
    } else {
        b->learn_end = 0;
    }
    b->learn_resid = 0;
}

static void soc_rest(bms_t *b)
{
    int32_t ia = iabs_sat(b->pack_ma);
    if (ia < BMS_REST_MA) {
        if (b->rest_ms < 1000000u) b->rest_ms += BMS_TICK_MS;
        b->status |= BMS_ST_REST;
    } else {
        b->rest_ms = 0;
        b->rest_snap = 0;
        b->status &= (uint16_t)~BMS_ST_REST;
        return;
    }
    if (b->rest_ms < BMS_REST_MS || b->rest_snap) return;
    /* An open sense wire is not a cell voltage. Do not wipe the coulomb count.
       Leave rest_snap clear so a later in-range sample during this rest can. */
    if (open_wire(b)) return;
    /* Cold rest voltage sits low. Do not treat that as an empty pack.
       Leave rest_snap clear so this same rest can snap after the cells warm. */
    if (b->t_min_dC <= BMS_DSG_COLD_DC) return;
    /* LFP mid-flat: only snap at the ends. Use the lowest cell (conservative).
       Once per rest. Drop the remainder so it cannot undo the snap, then keep
       counting. A later load under 0.5 A is still real charge. */
    if (b->v_min_mv < 3200 || b->v_min_mv > 3400) {
        uint16_t soc = soc_from_mv(b->v_min_mv);
        learn_on_snap(b, soc);
        b->soc_x10 = soc;
        b->soc_resid = 0;
        b->rest_snap = 1;
    }
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
    /* Start under 1 A of discharge. A session already on, including the one
       tick the resistors are forced off, stays while current is under 2 A. */
    int session = (b->status & BMS_ST_BAL) || b->bal_gap;
    int current_ok = session ? (b->pack_ma < 2000) : (b->pack_ma < 1000);
    int top_ok = spread >= BMS_BAL_DV_MV && b->v_max_mv >= BMS_BAL_MIN_MV;
    int k;

    /* Bleed resistors dump heat into the pack — stop if already warm.
       An open wire makes vmin look like 0 V, so every real cell would "win".
       A sample taken with the resistors on reads low. Do not recruit from it.
       Hold the mask for 1.0 s, then force one off tick and decide again. */
    if (!current_ok || open_wire(b) || b->t_max_dC >= BMS_CHG_OT_DC) {
        b->bal_left = 0;
        b->bal_gap = 0;
    } else if (b->bal_mask != 0) {
        if (b->bal_left > 1 && top_ok) {
            b->bal_left--;
            mask = b->bal_mask;
            b->bal_gap = 0;
        } else {
            /* Window done, or the top fell out of range. A still-valid top
               is only a measurement gap, so the 2 A hold continues. */
            b->bal_left = 0;
            b->bal_gap = top_ok ? 1 : 0;
        }
    } else if (top_ok) {
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
        b->bal_left = mask ? (uint8_t)BMS_BAL_ON_TICKS : 0;
        b->bal_gap = 0;
    } else {
        b->bal_left = 0;
        b->bal_gap = 0;
    }
    b->bal_mask = mask;
    if (mask) b->status |= BMS_ST_BAL; else b->status &= (uint16_t)~BMS_ST_BAL;
    bms_hal_set_balance(mask);
}

static void publish(bms_t *b)
{
    uint16_t w = 0, g = 0;
    int32_t dv = (int32_t)b->v_max_mv - (int32_t)b->v_min_mv;
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

    if (charge_done(b)) g |= BMS_FLG_FULL;
    if (b->v_min_mv <= BMS_CELL_UV_REL_MV || b->soc_x10 <= 50) g |= BMS_FLG_EMPTY;
    /* Coldest cell wants heat, but not if another sensor is already hot. */
    if (b->t_min_dC <= BMS_CHG_UT_REL_DC && b->t_max_dC < BMS_CHG_OT_REL_DC)
        g |= BMS_FLG_HEAT;
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
    b->cap_nom_mah = b->cap_mah;
    b->soc_x10 = soc_x10 > 1000 ? 1000 : soc_x10;
    /* Stay open until the first tick has sampled. A full or hot pack must
       not conduct during the gap between init and that tick. */
    b->status = 0;
    bms_hal_set_chg_mos(0);
    bms_hal_set_dsg_mos(0);
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
    if (dv <= BMS_DIFF_REL_MV) { b->db_diff = 0; f &= (uint16_t)~BMS_FLT_DIFF; }
    if (!ow) { b->db_ow = 0; f &= (uint16_t)~BMS_FLT_OW; }
    if (leak_rel(b)) { b->db_leak = 0; f &= (uint16_t)~BMS_FLT_LEAK; }
    b->fault = f;
    mosfets(b);
    /* Same hardware outputs as a tick. A hot sample must drop the bleed
       even when this call does not latch a new fault bit. */
    balance(b, &s);
    publish(b);
    mode_of(b);
}
