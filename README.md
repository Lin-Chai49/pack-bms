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
| ≥ 2 A through a FET commanded open, for 200 ms | open | open |

Charge-only cold and charge-only hot are not a latched pack fault. LFP should not be charged below 0 °C (plating) and charge is cut at 50 °C, but the pack can still discharge. Open-FET current is a latched fault and opens both FETs.

Each temperature limit remembers only itself. It opens on its own set point and closes only at its own release. Over-voltage, over-current, or under-voltage opening a FET does not start that wait. At 47 °C a recovered over-voltage may charge again, derated to 20 A. At 55 °C a recovered over-current may discharge again, derated to 40 A. At −15 °C a recovered under-voltage may discharge again, at 20 A; charge stays off until 5.0 °C. There is no outdoor thermometer. “Below −10 °C” means the coldest cell thermistor.

Pack over-temp and under-temp open on the tick the temperature crosses the set point, before the 200 ms fault bit, and stay open through the band down to the release point.

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

`bms_init` commands both FETs open and balance off. They close on the first healthy tick. A full or hot pack does not conduct in the gap before that tick. Calling init again mid-run opens the FETs until the next tick.

Compile `src/bms.c` and `src/bms_regs.c` with your HAL `.c`. Do not link `host/main_host.c` on the MCU — that file is a fake AFE for the desktop test.

Fill `holding` from the **same context** as `bms_tick()`. Current is two 16-bit words (registers 7 and 8). A Modbus read from an ISR in the middle of `bms_regs_fill()` can tear them.

The inverter must obey `allow_chg_a`, `allow_dsg_a`, `max_chg_v_x10`, and `min_dsg_v_x10`. Current at or above 2 A through a FET that was commanded open latches a leak fault after 200 ms and opens the other FET too.

AFE chips this was sized for: anything that can report 16 cell voltages (BQ76952, ADBMS6830, …). This repo does not contain an I²C driver for them.

---

## Protection

Fault **bits** wait 20 ticks (200 ms) in the set region, then latch until the analog value is past the **release** point. A 100 ms spike does not latch. A one-tick dip into the hysteresis band sheds one tick of that wait; it does not erase it. The wait starts over only when the value is past release. Hysteresis is not optional: 3.60 V after an over-voltage trip is still a trip (release is 3.50 V).

Open wire has no band between set and release. The bit stays until the wire has been in range for 200 ms, so one good sample cannot close the FETs.

`bms_clear_faults()` re-samples and drops bits that are already past release, including an open wire that is in range on that sample. It does not set new faults, and it does not advance debounce — calling it in a tight loop cannot trip over-voltage faster than 200 ms. It also does not clear a temperature hold that is still inside its band. It does refresh the balance mask, so bleed resistors follow that sample.

Spread releases at 200 mV inclusive. 201 mV stays latched; 200 mV clears.

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
| Open sense wire | &lt; 0.50 V or &gt; 4.80 V | in range for 200 ms |
| Open-FET current | ≥ 2 A in the open direction | back under 0.5 A |

Open-wire at 0 V also looks like under-voltage and a huge spread. The extra bit is so the inverter can show “sense wire”, not a second path to the FETs.

Open-FET current is the pack shunt. A welded FET or an inverter that keeps pushing into an open direction shows up here, and both FETs open. Between 0.5 A and 2 A the wait sheds one tick. This is not a chassis insulation check: milliamp earth leakage is below the shunt, and a residual-current device still belongs in the installation if that is the hazard.

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
| Coldest cell &lt; 10 °C | 0.2 C of usable `cap_mah` (40 A at the 200 Ah nameplate) |
| Hottest cell ≥ 45 °C | 20 A |
| SOC ≥ 98 % | 10 A |
| SOC ≥ 95 % | 40 A |
| Charge complete: cell ≥ 3.50 V, SOC ≥ 95 %, and current has been under 3 A | 0 A, FET stays closed |

Discharge current (`allow_dsg_a`), starting from 150 A:

| Condition | Cap |
|---|---|
| Lowest cell ≤ 2.70 V | 20 A |
| Lowest cell ≤ 2.90 V | 60 A |
| Coldest cell ≤ −20 °C, or hottest ≥ 60 °C | 0 A |
| Coldest cell ≤ −10 °C, held until −5 °C | 20 A |
| Coldest cell &lt; 0 °C | 40 A (fixed, not a C-rate) |
| Hottest cell ≥ 55 °C | 40 A |
| SOC ≤ 3 % | 20 A |
| SOC ≤ 10 % | 60 A |

Charge voltage (`max_chg_v_x10`): **55.2 V** (16 × 3.45 V). If one cell is already above 3.45 V, this becomes the **current pack voltage** so the inverter holds instead of pushing the high cell further.

Discharge cutoff (`min_dsg_v_x10`): **44.8 V** (16 × 2.80 V). If the lowest cell sags below 2.80 V, the cutoff rises to the present pack voltage and stays there until that cell is back to 2.90 V. A rebound of a few tens of millivolts does not let the inverter start into the same cell.

Charge complete, once entered, stays until the highest cell drops below 3.50 V or SOC drops below 95 %. A current spike above 3 A does not turn charge back on.

Warnings (register 18) fire **before** a trip: 3.50 V, 2.80 V, 45 °C, 5 °C, 80 % of the over-current limits, 200 mV spread, SOC ≤ 10 % or ≥ 95 %.

Flags (register 19):

- `FULL` — charge complete (same rule as the 0 A row above). It stays set until the cell or the SOC leaves that window
- `EMPTY` — lowest cell ≤ 2.80 V, or SOC ≤ 5 %
- `HEAT` — coldest cell ≤ 5.0 °C and hottest cell &lt; 45.0 °C (a pack heater, if you have one, may turn on). A cell that is already at the charge over-temp warning does not get a heater request. This core does not drive a heater pin.

---

## SOC, cycles, balance

**SOC** is coulomb count on a remainder, so a 10 ms tick at household current does not round to zero. Units: `soc_x10 = 500` means 50.0 %. Count only the direction whose FET is closed. Current reported while that FET is open does not move SOC: a shunt offset during a latched fault is not charge leaving or entering the pack. Remainder is discarded at 0 % and 100 %, otherwise a current-sensor offset at empty can sit in the remainder and block the next charge for a long time.

After 30 s with \|I\| &lt; 0.5 A, SOC is snapped once from the **lowest** cell’s rest voltage, and only outside 3.20–3.40 V. The snap does not run while the coldest cell is at or below −10 °C: that voltage is the cold pack, not an empty one. The snap stays pending, so a warmer sample in the same rest can still apply it. LFP is too flat in the middle for voltage to mean SOC. The snap is skipped while any sense wire is open (under 0.50 V or over 4.80 V): a disconnected wire is not an empty cell, and it must not zero the coulomb count. If the wire is still open at 30 s, a later in-range sample during that same rest may snap. When a snap does run, the remainder is cleared, so a leftover just under 0.1 % cannot undo it. Further current under 0.5 A still counts. The pack snaps again only after current rises above 0.5 A and then rests for another 30 s. In the flat band the remainder is left alone; if the lowest cell later leaves that band during the same rest, the snap runs then.

**Usable capacity** (`cap_mah`) starts equal to the nameplate passed to `bms_init`. Coulomb counting and `remain_ah_x10` use it, so a faded pack does not keep advertising nameplate amp-hours. A rest snap below 3.20 V and a later rest snap above 3.40 V, in either order, measure the coulombs that passed while the matching FET was closed. The SOC gap between those snaps scales that throughput up to a full capacity, and only when the net current has the same direction as the SOC change. A discharge that ends on a higher rest voltage is ignored. Both snaps must see every present sensor from 15.0 °C through 40.0 °C. If every thermistor slot is open, the pack is reported as 60 °C and does not learn. A rest in the 3.20–3.40 V band does not start or finish a measurement. Usable capacity moves one quarter of the way toward the measurement, never above the nameplate, and a measurement below half the nameplate is ignored. One short or cold stroke cannot collapse the pack.

**Cycles** increment once per full nameplate (`cap_nom_mah`) actually discharged (DSG FET on, current above rest). Fade does not make that counter run faster. Both values are RAM. Persist them from your HAL if you care across power loss. `bms_init` sets usable and nameplate to the same number; write a saved `cap_mah` back after init. A value outside half..full nameplate is pulled back onto that range on the next tick.

**Balance** is passive, and only near the top: highest cell ≥ 3.40 V and spread ≥ 25 mV. At most four cells, the highest ones. It starts only while discharge current is under 1 A, and once it is on it stays while discharge current is under 2 A, so a current hovering at 1 A does not chatter the bleed resistors. Off when pack ≥ 50 °C (bleed resistors heat the cells), and off while a sense wire is open. An open wire would otherwise be the lowest cell, and every cell near the top would bleed. Mid-band 25 mV is ignored on purpose.

---

## Register map

16-bit holding registers. Addresses 24–31 are unused (left 0). Cells start at 32.

| Addr | Name | Meaning |
|---|---|---|
| 0 | proto | 1 |
| 1 | n_cell | 16 |
| 2 | status | bit0 CHG FET, bit1 DSG FET, bit2 balancing, bit3 rest |
| 3 | fault | bit0 OV, 1 UV, 2 OCC, 3 OCD, 4 OT, 5 UT, 6 spread, 7 open-wire, 8 open-FET current |
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
| 20 | cycles | nameplate discharges (RAM) |
| 21 | remain_ah_x10 | usable Ah × SOC. 1000 = 100.0 Ah |
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
src/bms.c             tick: protect, FET limits, SOC, balance
src/bms_regs.c        pack bms_t into uint16_t[48]
host/main_host.c      fake AFE + tests (desktop only)
Makefile              `make test` — AddressSanitizer + UBSan
.github/workflows/check.yml   `make test` on push and pull request
```

---

## Desktop test

```bash
make test
```

No MCU. Uses the fake AFE in `host/main_host.c`. The binary is gitignored. The same command runs on GitHub Actions for every push and pull request.

Checks, among other things: 9.3 Ah from 3.10 V to 3.50 V on a 20 Ah pack moves usable capacity from 20.000 Ah to 17.500 Ah, and the return stroke continues to 15.625 Ah; a flat-band rest, a 1 Ah stroke, a 10 °C arrival, charge into an open FET, and a 9.3 Ah discharge that ends at 3.50 V do not move it; a stroke that measures above nameplate stays at nameplate; usable capacity written below half nameplate is pulled back to half, and one nameplate discharged is still one cycle; charge-complete holds through an 8 A spike and clears below 3.50 V; an open charge FET ignores charge current and an open discharge FET ignores discharge current; a cell below 2.80 V holds the discharge cutoff until 2.90 V; balance that is already on stays on at 1.5 A and drops at 2.5 A; FETs stay open until the first tick; 100 ms over-voltage pulse ignored; a one-tick dip to 3.60 V does not erase that wait, and a release to 3.48 V starts it over; `clear_faults` cannot speed debounce; over-voltage opens charge only; under-voltage opens discharge only (spread kept under 400 mV so a spread trip does not hide that); 55 °C stops charge, −1 °C stops charge, −10 °C holds discharge at 20 A until −5 °C and does not snap SOC, −21 °C opens both; 2 A through an open charge FET latches after 200 ms and opens discharge too, a 100 ms pulse does not, and one tick at 1 A does not erase that wait; a recovered over-voltage at 47 °C may charge again, a recovered over-current at 55 °C may discharge again, a recovered under-voltage at −15 °C may discharge again; a real 50 °C charge cut still waits for 45 °C, and a real 60 °C pack cut still holds at 55 °C; missing NTC is over-temp, not 25 °C; 0.2 C at 3 °C on the 200 Ah pack is 40 A; a 47 °C cell beside a 3 °C cell does not request heat; charge-complete; at most four bleed resistors; a hot `clear_faults` sample turns bleed off without latching over-temp; spread holds at 201 mV and releases at 200 mV; an open wire does not bleed and does not wipe SOC; the wire stays open through 100 ms back in range and releases at 200 ms, while `clear_faults` releases it on that sample; coulomb 100 A × 36 s on 200 Ah → −0.5 %; remainder clamp at 0 % / 100 %; rest snap at 3.10 V after 30 s clears the remainder, none at 3.30 V; after that snap a 0.4 A load still moves SOC, and a later rest can snap again; one high cell holds CV at pack voltage; signed current round-trip; `NULL` pointers.

On an MCU, compile `src/bms.c` and `src/bms_regs.c` against your HAL.

---

## What this is not

- A customer dashboard or household EMS ([home-solar-ess](https://github.com/Lin-Chai49/home-solar-ess) is a separate simulator)
- An SOH percentage, a thermal-runaway percentage, or an LLM
- A drop-in `.hex` for a specific BMS PCB
- Microsecond short-circuit protection, a hardware desat detector, insulation monitoring, precharge, or a CAN/Modbus stack
- A substitute for analog hardware protection (AFE comparators, fuse)

This firmware opens MOSFETs and publishes limits on a 10 ms tick. The analog front-end and the fuse stay in the circuit.

---

MIT. See [LICENSE](LICENSE).
