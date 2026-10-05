#include "bms.h"
#include "bms_hal.h"
#include "bms_regs.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

static bms_sample_t g_s;
static int g_chg = 1, g_dsg = 1;
static uint16_t g_bal;
static uint32_t g_ms;

void bms_hal_sample(bms_sample_t *out) { *out = g_s; }
void bms_hal_set_chg_mos(int on) { g_chg = on; }
void bms_hal_set_dsg_mos(int on) { g_dsg = on; }
void bms_hal_set_balance(uint16_t mask) { g_bal = mask; }
uint32_t bms_hal_millis(void) { return g_ms; }

static void set_cells(int16_t mv)
{
    int i;
    for (i = 0; i < BMS_N_CELL; i++) g_s.cell_mv[i] = mv;
    g_s.t_dC[0] = 250;
    g_s.t_dC[1] = g_s.t_dC[2] = g_s.t_dC[3] = INT16_MIN;
}

static void ticks(bms_t *b, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        g_ms += BMS_TICK_MS;
        bms_tick(b);
    }
}

static int fail;

static void expect(const char *name, int cond)
{
    if (!cond) {
        printf("FAIL %s\n", name);
        fail++;
    } else {
        printf(" ok  %s\n", name);
    }
}

static void idle_pack(void)
{
    set_cells(3300);
    g_s.pack_ma = 0;
}

int main(void)
{
    bms_t b;
    uint16_t reg[BMS_REG_N];
    int32_t ireg;

    memset(&g_s, 0, sizeof g_s);
    idle_pack();
    bms_init(&b, 200000, 500);
    expect("init fets open", g_chg == 0 && g_dsg == 0);
    expect("init balance off", g_bal == 0);
    ticks(&b, 5);
    expect("idle mos on", g_chg && g_dsg);
    expect("soc mid", b.soc_x10 >= 490 && b.soc_x10 <= 510);
    expect("sop idle chg 120", b.allow_chg_a == 120);
    expect("sop idle dsg 150", b.allow_dsg_a == 150);
    expect("remain 100.0 Ah", b.remain_ah_x10 == 1000);
    expect("cv 55.2 V", b.max_chg_v_x10 == 552);
    expect("cutoff 44.8 V", b.min_dsg_v_x10 == 448);

    /* 100 ms OV pulse must not latch (200 ms debounce). */
    set_cells(3450);
    g_s.cell_mv[6] = 3700;
    g_s.pack_ma = -20000;
    ticks(&b, 10);
    expect("OV pulse ignored", !(b.fault & BMS_FLT_OV));
    expect("chg still on during pulse", g_chg == 1);
    {
        int i;
        for (i = 0; i < 50; i++) bms_clear_faults(&b);
    }
    expect("clear_faults does not speed debounce", !(b.fault & BMS_FLT_OV));

    /* Over-voltage: one cell to 3.70 V. Keep spread under the 400 mV pack-diff trip. */
    ticks(&b, 15);
    expect("OV latches", b.fault & BMS_FLT_OV);
    expect("charge mos off", g_chg == 0);
    expect("discharge still on", g_dsg == 1);
    expect("allow chg 0", b.allow_chg_a == 0);

    /* Hysteresis: 3.60 V is below set, above release — stay latched. */
    g_s.cell_mv[6] = 3600;
    ticks(&b, 25);
    expect("OV holds in hysteresis", b.fault & BMS_FLT_OV);
    expect("chg still off in hyst", g_chg == 0);

    /* clear_faults must not force-clear while analog is still high. */
    bms_clear_faults(&b);
    expect("clear refused while OV", b.fault & BMS_FLT_OV);

    /* Analog recovered: clear_faults re-samples and must close the FET without a tick. */
    g_s.cell_mv[6] = 3480;
    bms_clear_faults(&b);
    expect("OV released", !(b.fault & BMS_FLT_OV));
    expect("charge mos on again", g_chg == 1);

    /* Long hold used to wrap the debounce counter and delay recovery by seconds. */
    set_cells(3450);
    g_s.cell_mv[6] = 3700;
    g_s.pack_ma = -20000;
    ticks(&b, 100);
    expect("OV still latched after 1s", b.fault & BMS_FLT_OV);
    g_s.cell_mv[6] = 3480;
    ticks(&b, 5);
    expect("OV released after long hold", !(b.fault & BMS_FLT_OV));

    /* A dip into the release region starts the 200 ms wait over.
       A dip that only reaches the hysteresis band must not. */
    idle_pack();
    set_cells(3450);
    g_s.cell_mv[6] = 3700; /* spread 250 mV, under the 400 mV trip */
    g_s.pack_ma = -5000;
    ticks(&b, 2);
    expect("baseline no OV", !(b.fault & BMS_FLT_OV) && g_chg == 1);
    ticks(&b, 8);
    expect("100 ms OV does not latch", !(b.fault & BMS_FLT_OV));
    g_s.cell_mv[6] = 3480;
    ticks(&b, 1);
    g_s.cell_mv[6] = 3700;
    ticks(&b, 10);
    expect("recovered OV wait starts over", !(b.fault & BMS_FLT_OV));
    expect("chg on after recovered wait", g_chg == 1);
    ticks(&b, 9); /* 19 ticks in the set region so far on this wait */
    expect("OV wait not done at 190 ms", !(b.fault & BMS_FLT_OV));
    g_s.cell_mv[6] = 3600; /* 3.60 V: not set, not release */
    ticks(&b, 1);
    expect("hysteresis dip keeps the wait", !(b.fault & BMS_FLT_OV));
    expect("chg on through the dip", g_chg == 1);
    g_s.cell_mv[6] = 3700;
    ticks(&b, 1);
    expect("OV still waiting after one tick back", !(b.fault & BMS_FLT_OV));
    ticks(&b, 1);
    expect("OV latches once the wait finishes", b.fault & BMS_FLT_OV);
    expect("chg off when that OV latches", g_chg == 0);
    g_s.cell_mv[6] = 3480;
    ticks(&b, 2);

    /* Under-voltage. Keep spread under 400 mV or DIFF also opens the charge FET. */
    idle_pack();
    set_cells(2480);
    g_s.cell_mv[2] = 2400;
    g_s.pack_ma = 10000;
    ticks(&b, 25);
    expect("UV latches", b.fault & BMS_FLT_UV);
    expect("dsg mos off", g_dsg == 0);
    expect("chg on during UV", g_chg == 1);

    set_cells(3300);
    g_s.cell_mv[2] = 2850;
    ticks(&b, 5);
    expect("UV released", !(b.fault & BMS_FLT_UV));

    /* Discharge over-current */
    idle_pack();
    g_s.pack_ma = 160000;
    ticks(&b, 25);
    expect("OCD", b.fault & BMS_FLT_OCD);
    expect("dsg off on OCD", g_dsg == 0);
    expect("chg on during OCD", g_chg == 1);
    g_s.pack_ma = 0;
    ticks(&b, 5);
    expect("OCD released", !(b.fault & BMS_FLT_OCD));

    /* Charge over-current */
    idle_pack();
    g_s.pack_ma = -130000;
    ticks(&b, 25);
    expect("OCC", b.fault & BMS_FLT_OCC);
    expect("chg off on OCC", g_chg == 0);
    expect("dsg on during OCC", g_dsg == 1);
    g_s.pack_ma = 0;
    ticks(&b, 5);
    expect("OCC released", !(b.fault & BMS_FLT_OCC));

    /* 55 C: charge inhibited (LFP), discharge still allowed. */
    idle_pack();
    g_s.t_dC[0] = 550;
    ticks(&b, 5);
    expect("55C no pack OT fault", !(b.fault & BMS_FLT_OT));
    expect("55C chg off dsg on", g_chg == 0 && g_dsg == 1);
    expect("55C allow chg 0", b.allow_chg_a == 0);

    /* 61 C: pack OT, both FETs */
    g_s.t_dC[0] = 610;
    ticks(&b, 25);
    expect("OT", b.fault & BMS_FLT_OT);
    expect("both mos off on OT", g_chg == 0 && g_dsg == 0);
    /* 55 C is still inside the pack-OT band. Release is 50 C. */
    g_s.t_dC[0] = 550;
    ticks(&b, 5);
    expect("OT holds through 55C", b.fault & BMS_FLT_OT);
    expect("both mos stay off through 55C", g_chg == 0 && g_dsg == 0);
    g_s.t_dC[0] = 400;
    ticks(&b, 5);
    expect("OT released", !(b.fault & BMS_FLT_OT));
    expect("both mos on after OT", g_chg == 1 && g_dsg == 1);

    /* 3 C: still charge, but 0.2C and heater request. */
    idle_pack();
    g_s.t_dC[0] = 30;
    ticks(&b, 5);
    expect("3C chg still on", g_chg == 1);
    expect("3C chg derated 40A", b.allow_chg_a == 40);
    expect("3C heat flag", b.flags & BMS_FLG_HEAT);

    /* Heater follows the coldest cell, unless another sensor is already hot. */
    g_s.t_dC[1] = 470;
    ticks(&b, 2);
    expect("47C beside 3C blocks heater", (b.flags & BMS_FLG_HEAT) == 0);
    expect("47C beside 3C derates to 20A", b.allow_chg_a == 20);
    expect("47C beside 3C is not pack OT", !(b.fault & BMS_FLT_OT));
    expect("47C beside 3C chg stays on", g_chg == 1);
    g_s.t_dC[1] = INT16_MIN;

    /* -1 C: no charge (plating). Discharge stays on. Not a pack UT fault. */
    g_s.t_dC[0] = -10;
    ticks(&b, 5);
    expect("cold not pack UT", !(b.fault & BMS_FLT_UT));
    expect("-1C chg off dsg on", g_chg == 0 && g_dsg == 1);
    expect("-1C allow chg 0", b.allow_chg_a == 0);
    g_s.t_dC[0] = 30;
    ticks(&b, 5);
    expect("chg off until 5C", g_chg == 0);
    g_s.t_dC[0] = 50;
    ticks(&b, 5);
    expect("chg on at 5C", g_chg == 1);

    /* -21 C: pack too cold to discharge */
    g_s.t_dC[0] = -210;
    ticks(&b, 25);
    expect("UT", b.fault & BMS_FLT_UT);
    expect("both mos off on UT", g_chg == 0 && g_dsg == 0);
    g_s.t_dC[0] = 250;
    ticks(&b, 5);
    expect("UT released", !(b.fault & BMS_FLT_UT));

    /* 47 C never reached the 50 C charge cut. OV must not invent that wait.
       set_cells() rewrites the thermistor, so set temperature after it. */
    idle_pack();
    set_cells(3480);
    g_s.t_dC[0] = 470;
    ticks(&b, 5);
    expect("47C chg on", g_chg == 1 && g_dsg == 1);
    expect("47C allow chg 20A", b.allow_chg_a == 20);
    g_s.cell_mv[6] = 3700; /* spread 220 mV, under the 400 mV trip */
    ticks(&b, 25);
    expect("47C OV latches", b.fault & BMS_FLT_OV);
    expect("47C chg off on OV", g_chg == 0);
    expect("47C dsg stays on", g_dsg == 1);
    g_s.cell_mv[6] = 3480;
    ticks(&b, 5);
    expect("47C OV clear", !(b.fault & BMS_FLT_OV));
    expect("47C chg back after OV", g_chg == 1);
    expect("47C allow still 20A", b.allow_chg_a == 20);

    /* A real charge-over-temp still waits until 45.0 C. */
    g_s.t_dC[0] = 520;
    ticks(&b, 5);
    expect("52C chg off", g_chg == 0);
    g_s.t_dC[0] = 470;
    ticks(&b, 5);
    expect("47C chg stays off after 52C", g_chg == 0);
    g_s.t_dC[0] = 450;
    ticks(&b, 5);
    expect("chg on at 45C", g_chg == 1);

    /* 55 C derates discharge. It is not pack OT, so OCD must not hold DSG. */
    idle_pack();
    g_s.t_dC[0] = 550;
    ticks(&b, 5);
    expect("55C dsg on before OCD", g_dsg == 1);
    expect("55C chg held", g_chg == 0);
    g_s.pack_ma = 160000;
    ticks(&b, 25);
    expect("55C OCD", b.fault & BMS_FLT_OCD);
    expect("55C dsg off on OCD", g_dsg == 0);
    g_s.pack_ma = 0;
    ticks(&b, 5);
    expect("55C OCD clear", !(b.fault & BMS_FLT_OCD));
    expect("55C dsg back", g_dsg == 1);
    expect("55C chg still off", g_chg == 0);
    expect("55C allow dsg 40A", b.allow_dsg_a == 40);
    expect("55C no pack OT", !(b.fault & BMS_FLT_OT));

    /* -15 C: charge stays off. UV must not hold discharge until -10 C. */
    idle_pack();
    g_s.t_dC[0] = -150;
    ticks(&b, 5);
    expect("-15C chg off dsg on", g_chg == 0 && g_dsg == 1);
    expect("-15C not pack UT", !(b.fault & BMS_FLT_UT));
    set_cells(2480);
    g_s.cell_mv[2] = 2400; /* spread 80 mV */
    g_s.t_dC[0] = -150;
    g_s.pack_ma = 10000;
    ticks(&b, 25);
    expect("-15C UV", b.fault & BMS_FLT_UV);
    expect("-15C dsg off on UV", g_dsg == 0);
    expect("-15C chg off during UV", g_chg == 0);
    set_cells(3300);
    g_s.t_dC[0] = -150;
    g_s.pack_ma = 0;
    ticks(&b, 5);
    expect("-15C UV clear", !(b.fault & BMS_FLT_UV));
    expect("-15C no UT fault", !(b.fault & BMS_FLT_UT));
    expect("-15C dsg back", g_dsg == 1);
    expect("-15C chg still off", g_chg == 0);

    /* Missing NTC is fail-safe OT, not "assume 25 C". */
    idle_pack();
    g_s.t_dC[0] = g_s.t_dC[1] = g_s.t_dC[2] = g_s.t_dC[3] = INT16_MIN;
    ticks(&b, 25);
    expect("open NTC trips OT", b.fault & BMS_FLT_OT);
    expect("both mos off on open NTC", g_chg == 0 && g_dsg == 0);
    g_s.t_dC[0] = 250;
    ticks(&b, 5);
    expect("NTC restored", !(b.fault & BMS_FLT_OT));

    /* Pack spread without OV/UV: both FETs. 3500 vs 3090 = 410 mV. */
    idle_pack();
    g_s.cell_mv[0] = 3500;
    g_s.cell_mv[1] = 3090;
    ticks(&b, 10);
    expect("DIFF pulse ignored", !(b.fault & BMS_FLT_DIFF));
    ticks(&b, 15);
    expect("DIFF latches", b.fault & BMS_FLT_DIFF);
    expect("both mos off on DIFF", g_chg == 0 && g_dsg == 0);
    /* 3500 vs 3299 = 201 mV, still above the inclusive 200 mV release. */
    g_s.cell_mv[1] = 3299;
    ticks(&b, 5);
    expect("DIFF holds at 201 mV", b.fault & BMS_FLT_DIFF);
    expect("both mos stay off at 201 mV", g_chg == 0 && g_dsg == 0);
    g_s.cell_mv[1] = 3300; /* 3500 vs 3300 = 200 mV */
    ticks(&b, 5);
    expect("DIFF released at 200 mV", !(b.fault & BMS_FLT_DIFF));
    expect("both mos on after DIFF", g_chg == 1 && g_dsg == 1);

    /* Balance near top */
    idle_pack();
    set_cells(3450);
    g_s.cell_mv[0] = 3520;
    g_s.pack_ma = -2000;
    ticks(&b, 5);
    expect("balance mask on high cell", g_bal & 1u);
    expect("cell 7 not bleeding if low", (g_bal & (1u << 6)) == 0 || g_s.cell_mv[6] >= 3450 + 25);

    /* Mid-band: no balance at 25 mV (LFP flat, we require >= 3.40 V) */
    idle_pack();
    g_s.cell_mv[0] = 3330;
    ticks(&b, 5);
    expect("no balance in mid band", g_bal == 0);

    /* Near top: inverter current limit tapers before the OV trip. */
    idle_pack();
    set_cells(3520);
    ticks(&b, 5);
    expect("sop 3.52 V is 20 A", b.allow_chg_a == 20);
    expect("ov warn at 3.50", b.warn & BMS_WRN_OV);

    /* Charge complete: hold, no more current, MOS still closed. */
    g_s.pack_ma = -1000;
    b.soc_x10 = 980;
    ticks(&b, 5);
    expect("full flag", b.flags & BMS_FLG_FULL);
    expect("full allow chg 0", b.allow_chg_a == 0);
    expect("full chg mos still on", g_chg == 1);

    /* At most 4 bleed resistors at once. */
    idle_pack();
    set_cells(3450);
    {
        int i, n = 0;
        for (i = 0; i < 8; i++) g_s.cell_mv[i] = 3520;
        ticks(&b, 5);
        for (i = 0; i < BMS_N_CELL; i++) if (g_bal & (1u << i)) n++;
        expect("balance at most 4", n == 4);
        expect("balance the high group", (g_bal & 0xFF00u) == 0);
    }

    /* clear_faults re-samples. Bleed must follow that sample, and a hot
       sample opens the FETs without latching a fault bit by itself. */
    idle_pack();
    set_cells(3450);
    g_s.cell_mv[0] = 3520;
    g_s.pack_ma = -2000;
    ticks(&b, 5);
    expect("bleed on before hot clear", g_bal != 0);
    g_s.t_dC[0] = 600;
    bms_clear_faults(&b);
    expect("hot clear turns bleed off", g_bal == 0);
    expect("hot clear opens both fets", g_chg == 0 && g_dsg == 0);
    expect("hot clear does not latch OT", !(b.fault & BMS_FLT_OT));

    /* Open sense wire */
    idle_pack();
    g_s.cell_mv[4] = 0;
    ticks(&b, 25);
    expect("open wire", b.fault & BMS_FLT_OW);
    expect("both mos off on OW", g_chg == 0 && g_dsg == 0);
    g_s.cell_mv[4] = 3300;
    ticks(&b, 10);
    expect("open wire holds through 100 ms", b.fault & BMS_FLT_OW);
    expect("fets stay open through that 100 ms", g_chg == 0 && g_dsg == 0);
    ticks(&b, 10);
    expect("open wire released at 200 ms", !(b.fault & BMS_FLT_OW));

    /* A dead sense wire is not the lowest cell and not an empty pack. */
    bms_init(&b, 200000, 500);
    idle_pack();
    ticks(&b, 2);
    set_cells(3450);
    g_s.cell_mv[4] = 0;
    g_s.pack_ma = 0;
    ticks(&b, 5);
    expect("open wire no balance", g_bal == 0);
    ticks(&b, 3100);
    expect("open wire still latched", b.fault & BMS_FLT_OW);
    expect("open wire keeps soc", b.soc_x10 == 500);
    expect("open wire balance stays off", g_bal == 0);
    g_s.cell_mv[4] = 3450;
    bms_clear_faults(&b);
    expect("clear_faults releases open wire now", !(b.fault & BMS_FLT_OW));
    expect("clear_faults closes fets on a fixed wire", g_chg == 1 && g_dsg == 1);
    expect("clear_faults keeps soc", b.soc_x10 == 500);

    /* 100 A discharge, 36 s on 200 Ah -> 1 Ah -> 0.5% -> 5 counts of soc_x10 */
    idle_pack();
    g_s.pack_ma = 100000;
    b.soc_x10 = 500;
    b.soc_resid = 0;
    ticks(&b, 3600);
    expect("soc coulomb 0.5%", b.soc_x10 == 495);
    expect("200 Ah pack not a full cycle", b.cycles == 0);

    /* 1 Ah pack, 100 A × 36 s = 1 Ah → one cycle */
    bms_init(&b, 1000, 500);
    idle_pack();
    ticks(&b, 2);
    g_s.pack_ma = 100000;
    ticks(&b, 3600);
    expect("one cycle on 1 Ah pack", b.cycles == 1);

    /* Coulomb leftover at 0 % must not block the next charge. */
    bms_init(&b, 200000, 0);
    idle_pack();
    ticks(&b, 2);
    g_s.pack_ma = 100000;
    ticks(&b, 3600); /* 1 Ah past empty */
    expect("soc stuck at 0", b.soc_x10 == 0);
    expect("empty resid clamped", b.soc_resid == 0);
    g_s.pack_ma = -100000;
    ticks(&b, 3600); /* 1 Ah in → 0.5 % */
    expect("charge after empty overshoot", b.soc_x10 == 5);

    bms_init(&b, 200000, 1000);
    idle_pack();
    ticks(&b, 2);
    g_s.pack_ma = -100000;
    ticks(&b, 3600);
    expect("soc stuck at 100", b.soc_x10 == 1000);
    expect("full resid clamped", b.soc_resid == 0);
    g_s.pack_ma = 100000;
    ticks(&b, 3600);
    expect("discharge after full overshoot", b.soc_x10 == 995);

    /* Rest OCV snap only at the LFP ends, after 30 s. */
    bms_init(&b, 200000, 500);
    idle_pack();
    set_cells(3100);
    g_s.pack_ma = 0;
    /* One count short of a 0.1 % step. A snap that leaves this remainder
       undoes itself on the next tick. */
    b.soc_resid = (int64_t)b.cap_mah * 3600LL - 1;
    ticks(&b, 2900);
    expect("no ocv snap before 30s", b.soc_x10 == 500);
    ticks(&b, 150);
    expect("ocv snap at 3.10 V is 5%", b.soc_x10 == 50);
    expect("ocv snap clears resid", b.soc_resid == 0);
    set_cells(3300);
    b.soc_x10 = 500;
    b.soc_resid = (int64_t)b.cap_mah * 3600LL - 1;
    b.rest_ms = 0;
    ticks(&b, 3100);
    expect("no ocv snap in mid band", b.soc_x10 == 500);
    expect("mid band keeps resid", b.soc_resid == (int64_t)b.cap_mah * 3600LL - 1);

    /* One snap per rest. 0.4 A is under the rest threshold and must still count.
       900 ticks × 0.4 A × 10 ms = 3.6 A·s = 0.1 % of this 1 Ah pack. */
    bms_init(&b, 1000, 500);
    idle_pack();
    set_cells(3100);
    g_s.pack_ma = 0;
    ticks(&b, 3000);
    expect("snap once at 3.10 V", b.soc_x10 == 50);
    expect("snap once clears resid", b.soc_resid == 0);
    g_s.pack_ma = 400;
    ticks(&b, 900);
    expect("rest current still counts", b.soc_x10 == 49);
    g_s.pack_ma = 1000;
    ticks(&b, 5);
    g_s.pack_ma = 0;
    set_cells(2900);
    ticks(&b, 2900);
    expect("second rest not early", b.soc_x10 == 49);
    ticks(&b, 200);
    expect("second rest snaps again", b.soc_x10 == 20);

    /* One high cell: CV holds current pack V (53.1 V), not 55.2 V. */
    idle_pack();
    g_s.cell_mv[0] = 3600;
    ticks(&b, 5);
    expect("cv hold one high cell", b.max_chg_v_x10 == 531);

    bms_init(&b, 200000, 500);
    idle_pack();
    ticks(&b, 2);

    idle_pack();
    g_s.pack_ma = -20000;
    ticks(&b, 2);
    bms_regs_fill(&b, reg);
    expect("reg ncell 16", reg[REG_NCELL] == 16);
    expect("reg cell0", reg[REG_CELL0] == (uint16_t)b.cell_mv[0]);
    ireg = (int32_t)(((uint32_t)reg[REG_I_HI] << 16) | reg[REG_I_LO]);
    expect("signed current roundtrip", ireg == -20000);
    expect("reg cv", reg[REG_MAX_CHG_V] == b.max_chg_v_x10);
    expect("reg remain", reg[REG_REMAIN_AH] == b.remain_ah_x10);

    /* INT32_MIN must not abort in abs(); it is charge-direction so OCC. */
    g_s.pack_ma = INT32_MIN;
    ticks(&b, 25);
    expect("INT32_MIN current survived", b.fault & BMS_FLT_OCC);
    g_s.pack_ma = 0;
    ticks(&b, 5);

    bms_tick(NULL);
    bms_clear_faults(NULL);
    bms_init(NULL, 0, 0);
    bms_regs_fill(NULL, reg);
    bms_regs_fill(&b, NULL);
    expect("null pointers do not crash", 1);

    printf("%s\n", fail ? "FAILED" : "ok");
    return fail ? 1 : 0;
}
