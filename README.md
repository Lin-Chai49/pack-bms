# pack-bms

Firmware core for the MCU that sits on a **16-series LFP pack**.

It samples cells, opens charge and discharge MOSFETs, bleeds high cells near the top of charge, counts SOC, and publishes a 16-bit register image that an inverter (or a Modbus/CAN shim) can read.

It is not an app, not an energy-management dashboard, and not a binary for a named BMS board. You still write the AFE driver, the MOSFET gate drive, and the fieldbus.

Call `bms_tick()` every 10 ms. C99, no RTOS, no vendor HAL, no heap.

Default pack this was sized for: **16s LFP, 200 Ah** (`bms_init(..., 200000, ...)`). At 3.2 V/cell that is about 10 kWh. Charge/discharge current limits default to 120 A / 150 A (about 6–7 kW on a 48 V bus). Change the macros in `include/bms.h` for a different pack. Do not change them from the inverter side.

---

## What runs on the pack

```
                    +-- charge MOSFET (CHG) --+
   pack +  ---------+                         +---- inverter P+
                    +-- discharge MOSFET (DSG)+

   pack −  --------------------------------------- inverter P−

   16 cells in series. Bit i of the balance mask turns on the bleed
   resistor across cell i. Up to four cells at once.
```

Charge and discharge FETs are **independent**:

| Event | CHG | DSG |
|---|---|---|
| Cell over-voltage, charge over-current | open | stays on |
| Cell under-voltage, discharge over-current | stays on | open |
| Pack too hot (≥ 60 °C) or too cold (≤ −20 °C) | open | open |
| Cell spread ≥ 400 mV, or open sense wire | open | open |
| Charge-only cold (≤ 0 °C) or charge-only hot (≥ 50 °C) | open **this tick** | stays on |

That last row is not a latched pack fault. LFP should not be charged below 0 °C (plating) and charge is cut at 50 °C, but the pack can still discharge.

Current sign matches a shunt that reads **load as positive**:

- `pack_ma > 0` — discharging
- `pack_ma < 0` — charging

---

## Hook it up

Implement `include/bms_hal.h`. The core never talks to I²C or a MOSFET driver itself.

```c
void bms_hal_sample(bms_sample_t *out);   /* cell mV, pack mA, up to 4 temps */
void bms_hal_set_chg_mos(int on);
void bms_hal_set_dsg_mos(int on);
void bms_hal_set_balance(uint16_t mask);  /* bit i bleeds cell i */
uint32_t bms_hal_millis(void);            /* unused by the core */
```

Temperatures are 0.1 °C (`250` = 25.0 °C). Unused sensor slots **must** be `INT16_MIN`. `0` means 0.0 °C and will look like “too cold to charge”. If every slot is `INT16_MIN`, the core treats the pack as over-temperature and opens both FETs. It does not assume 25 °C.

Main loop:

```c
bms_t bms;
uint16_t holding[BMS_REG_N];

bms_init(&bms, 200000, 500);     /* 200 Ah, 50.0 % */

for (;;) {
    bms_tick(&bms);              /* every 10 ms */
    bms_regs_fill(&bms, holding);
    /* write `holding` to Modbus or unpack into a CAN PDO */
    delay_ms(10);
}
```

Compile `src/bms.c` and `src/bms_regs.c` with your HAL `.c`. Do not link `host/main_host.c` on the MCU — that file is a fake AFE for the desktop test.

Fill `holding` from the **same context** as `bms_tick()`. Current is two 16-bit words (registers 7 and 8). A Modbus read from an ISR in the middle of `bms_regs_fill()` can tear them.

The inverter must obey `allow_chg_a`, `allow_dsg_a`, and `max_chg_v_x10`. If it keeps charging into an open charge FET, that is the inverter’s bug (or a stuck FET), not this core.

AFE chips this was sized for: anything that can report 16 cell voltages (BQ76952, ADBMS6830, …). This repo does not contain an I²C driver for them.

---

## Protection

Fault **bits** wait 20 ticks (200 ms) in the set region, then latch until the analog value is past the **release** point. A 100 ms spike does not latch. Hysteresis is not optional: 3.60 V after an over-voltage trip is still a trip (release is 3.50 V).

`bms_clear_faults()` re-samples and drops bits that are already past release. It does not set new faults, and it does not advance debounce — calling it in a tight loop cannot trip over-voltage faster than 200 ms.

| | Set | Release |
|---|---|---|
| Cell over-voltage | 3.65 V | 3.50 V |
| Cell under-voltage | 2.50 V | 2.80 V |
| Charge over-current | 120 A | charge current &lt; 20 A |
| Discharge over-current | 150 A | discharge current &lt; 20 A |
| Charge over-temp (FET only, no 200 ms wait) | 50.0 °C | 45.0 °C |
| Discharge over-temp (fault bit) | 60.0 °C | 50.0 °C |
| Charge under-temp (FET only, no 200 ms wait) | 0.0 °C | 5.0 °C |
| Discharge under-temp (fault bit) | −20.0 °C | −10.0 °C |
| Cell spread | 400 mV | 200 mV |
| Open sense wire | &lt; 0.50 V or &gt; 4.80 V | voltage in range |

Open-wire at 0 V also looks like under-voltage and a huge spread. The extra bit is so the inverter can show “sense wire”, not a second path to the FETs.

---

## What the inverter is told to do

Registers 15–16 and 22–23 are the closed-loop interface. Current limits **taper**; they are not only 0 or full.

Charge current (`allow_chg_a`), starting from 120 A, then the **lowest** of:

| Condition | Cap |
|---|---|
| Highest cell ≥ 3.60 V | 5 A |
| Highest cell ≥ 3.50 V | 20 A |
| Highest cell ≥ 3.45 V | 60 A |
| Coldest cell ≤ 0 °C, or hottest ≥ 50 °C | 0 A (FET already open) |
| Coldest cell &lt; 10 °C | 0.2 C of `cap_mah` (40 A on 200 Ah) |
| Hottest cell ≥ 45 °C | 20 A |
| SOC ≥ 98 % | 10 A |
| SOC ≥ 95 % | 40 A |
| Charge complete: cell ≥ 3.50 V, SOC ≥ 95 %, \|I\| &lt; 3 A | 0 A, FET stays closed |

Discharge current (`allow_dsg_a`), starting from 150 A:

| Condition | Cap |
|---|---|
| Lowest cell ≤ 2.70 V | 20 A |
| Lowest cell ≤ 2.90 V | 60 A |
| Coldest cell ≤ −20 °C, or hottest ≥ 60 °C | 0 A |
| Coldest cell &lt; 0 °C | 40 A (fixed, not a C-rate) |
| Hottest cell ≥ 55 °C | 40 A |
| SOC ≤ 3 % | 20 A |
| SOC ≤ 10 % | 60 A |

Charge voltage (`max_chg_v_x10`): **55.2 V** (16 × 3.45 V). If one cell is already above 3.45 V, this becomes the **current pack voltage** so the inverter holds instead of pushing the high cell further.

Discharge cutoff (`min_dsg_v_x10`): **44.8 V** (16 × 2.80 V).

Warnings (register 18) fire **before** a trip: 3.50 V, 2.80 V, 45 °C, 5 °C, 80 % of the over-current limits, 200 mV spread, SOC ≤ 10 % or ≥ 95 %.

Flags (register 19):

- `FULL` — charge complete (same rule as the 0 A row above)
- `EMPTY` — lowest cell ≤ 2.80 V, or SOC ≤ 5 %
- `HEAT` — coldest cell ≤ 5.0 °C (a pack heater, if you have one, may turn on). This core does not drive a heater pin.

---

## SOC, cycles, balance

**SOC** is coulomb count on a remainder, so a 10 ms tick at household current does not round to zero. Units: `soc_x10 = 500` means 50.0 %. Remainder is discarded at 0 % and 100 %, otherwise a current-sensor offset at empty can sit in the remainder and block the next charge for a long time.

After 30 s with \|I\| &lt; 0.5 A, SOC is snapped from the **lowest** cell’s rest voltage, and only outside 3.20–3.40 V. LFP is too flat in the middle for voltage to mean SOC.

**Cycles** increment once per full `cap_mah` actually discharged (DSG FET on, current above rest). Stored in RAM. Persist it from your HAL if you care across power loss.

**Balance** is passive, and only near the top: highest cell ≥ 3.40 V and spread ≥ 25 mV. At most four cells, the highest ones. Off when pack ≥ 50 °C (bleed resistors heat the cells). Mid-band 25 mV is ignored on purpose.

---

## Register map

16-bit holding registers. Addresses 24–31 are unused (left 0). Cells start at 32.

| Addr | Name | Meaning |
|---|---|---|
| 0 | proto | 1 |
| 1 | n_cell | 16 |
| 2 | status | bit0 CHG FET, bit1 DSG FET, bit2 balancing, bit3 rest |
| 3 | fault | bit0 OV, 1 UV, 2 OCC, 3 OCD, 4 OT, 5 UT, 6 spread, 7 open-wire |
| 4 | mode | 0 idle, 1 charge, 2 discharge, 3 protect |
| 5 | soc_x10 | 500 = 50.0 % |
| 6 | pack_V_x10 | 0.1 V |
| 7–8 | pack_mA | signed 32-bit, high word then low. Charge is negative |
| 9–10 | t_max / t_min | signed 0.1 °C, two’s complement |
| 11–12 | v_max / v_min | mV (negative voltages are published as 0) |
| 13–14 | high / low cell | 1..16 |
| 15–16 | allow charge / discharge | amperes. 0 = do not use that direction |
| 17 | balance mask | bit i = cell i bleeding |
| 18 | warn | bit0 OV, 1 UV, 2 OT, 3 UT, 4 OCC, 5 OCD, 6 spread, 7 low SOC, 8 high SOC |
| 19 | flags | bit0 charge-complete, 1 empty, 2 heater requested |
| 20 | cycles | full-capacity discharges (RAM) |
| 21 | remain_ah_x10 | 1000 = 100.0 Ah |
| 22 | max_chg_v_x10 | inverter CV, 0.1 V |
| 23 | min_dsg_v_x10 | inverter cutoff, 0.1 V |
| 32–47 | cell mV | cell 1..16 |

Reconstruct current on any MCU, including 16-bit `int`:

```c
int32_t ma = (int32_t)(((uint32_t)reg[7] << 16) | reg[8]);
```

The `uint32_t` cast is required. `(reg[7] << 16)` is undefined or zero if `int` is 16 bits.

---

## Files

```
include/bms.h         trip points, fault bits, bms_t
include/bms_hal.h     the five functions you implement
include/bms_regs.h    holding-register addresses
src/bms.c             tick: protect, FETs, SOC, balance, SOP
src/bms_regs.c        pack bms_t into uint16_t[48]
host/main_host.c      fake AFE + tests (desktop only)
Makefile              `make test` — AddressSanitizer + UBSan
```

---

## Desktop test

```bash
make test
```

No MCU. Uses the fake AFE in `host/main_host.c`. The binary is gitignored.

Checks, among other things: 100 ms over-voltage pulse ignored; `clear_faults` cannot speed debounce; over-voltage opens charge only; under-voltage opens discharge only (spread kept under 400 mV so a spread trip does not hide that); 55 °C stops charge, −1 °C stops charge, −21 °C opens both; missing NTC is over-temp, not 25 °C; 0.2 C at 3 °C on the 200 Ah pack is 40 A; charge-complete; at most four bleed resistors; coulomb 100 A × 36 s on 200 Ah → −0.5 %; remainder clamp at 0 % / 100 %; rest snap at 3.10 V after 30 s, none at 3.30 V; one high cell holds CV at pack voltage; signed current round-trip; `NULL` pointers.

On an MCU, compile `src/bms.c` and `src/bms_regs.c` against your HAL.

---

## What this is not

- A customer dashboard or household EMS ([home-solar-ess](https://github.com/Lin-Chai49/home-solar-ess) is a separate simulator)
- SOH, thermal-runaway percentage, or an LLM
- A drop-in `.hex` for a specific BMS PCB
- Microsecond short-circuit protection, stuck-FET detection, precharge, or a CAN/Modbus stack
- A substitute for analog hardware protection (AFE comparators, fuse)

This firmware opens MOSFETs and publishes limits on a 10 ms tick. The analog front-end and the fuse stay in the circuit.

---

MIT. See [LICENSE](LICENSE).
