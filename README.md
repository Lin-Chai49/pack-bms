# pack-bms

[![check](https://github.com/Lin-Chai49/pack-bms/actions/workflows/check.yml/badge.svg)](https://github.com/Lin-Chai49/pack-bms/actions/workflows/check.yml)

Firmware core for the MCU on a **16-series LFP pack**. Each tick it samples the cells, commands the charge and discharge MOSFETs, bleeds the high cells near the top of charge, counts state of charge, and publishes a 48-word register image an inverter can read.

It is the pack controller, not a dashboard and not a `.hex` for a named board. You still write the analog-front-end driver, the MOSFET gate drive, and the fieldbus. The core is C99: no RTOS, no heap, no vendor HAL.

| | |
|---|---|
| Default pack | 16s LFP, 200 Ah, about 10 kWh at 3.2 V per cell |
| Call | `bms_tick()` every 10 ms |
| Charge / discharge caps | 120 A / 150 A, about 6–7 kW on a 48 V bus |
| Inverter CV / cutoff | 55.2 V / 44.8 V, then tapered as cells approach the ends |
| Sizing | Macros in `include/bms.h`. The inverter does not change them |

The household simulator is a separate repository: [home-solar-ess](https://github.com/Lin-Chai49/home-solar-ess).

## Contents

- [Pack wiring](#pack-wiring)
- [Bring-up](#bring-up)
- [Protection](#protection)
- [Inverter limits](#inverter-limits)
- [SOC, capacity, cycles, balance](#soc-capacity-cycles-balance)
- [Register map](#register-map)
- [Files and desktop test](#files-and-desktop-test)
- [Outside this repository](#outside-this-repository)

---

## Pack wiring

Charge and discharge are separate MOSFETs. Opening one direction leaves the other available.

```
                         CHG MOSFET
              +-----------/ ------------+
   pack + ----+                         +---- inverter P+
              +-----------/ ------------+
                         DSG MOSFET

   pack − ----------------------------------- inverter P−

   Sixteen cells in series.
   Balance mask bit i turns on the bleed resistor across cell i.
   At most four cells bleed at once.
```

| Event | CHG | DSG |
|---|---|---|
| Cell over-voltage, or charge over-current | open | stays on |
| Cell under-voltage, or discharge over-current | stays on | open |
| Pack ≥ 60 °C, or pack ≤ −20 °C | open | open |
| Cell spread ≥ 400 mV, or an open sense wire | open | open |
| Charge-only cold (≤ 0 °C) or charge-only hot (≥ 50 °C) | open on this tick | stays on |
| ≥ 2 A through a FET already commanded open, for 200 ms | open | open |

The charge-only cold and hot rows are FET holds. They do not set a fault bit. LFP is not charged at or below 0 °C, because of plating, and charge current is cut at 50 °C. Discharge can continue. The open-FET row is a latched fault and opens both FETs.

Each temperature limit remembers only itself. It opens on its own set point and closes only at its own release. An over-voltage, over-current, or under-voltage that opens a FET does not start that temperature wait.

| Recovered fault | What comes back |
|---|---|
| Over-voltage clears at 47 °C | Charge returns, capped at 20 A |
| Over-current clears at 55 °C | Discharge returns, capped at 40 A |
| Under-voltage clears at −15 °C | Discharge returns at 20 A. Charge stays off until 5.0 °C |

There is no outdoor thermometer. “Below −10 °C” means the coldest cell thermistor.

Pack over-temperature and under-temperature open on the tick the temperature crosses the set point, before the 200 ms fault bit, and they stay open through the band down to the release point.

The shunt sign is **load positive**:

- `pack_ma > 0` — discharging
- `pack_ma < 0` — charging

One tick, in order: sample, cell statistics, protection, MOSFET limits, coulomb count, rest snap, balance, publish, mode. Protection judges current against the FET command from the previous tick. That is how an already-open FET is caught still carrying current.

---

## Bring-up

Implement `include/bms_hal.h`. The core never talks to I²C or a gate driver.

```c
void bms_hal_sample(bms_sample_t *out);   /* cell mV, pack mA, up to 4 temps */
void bms_hal_set_chg_mos(int on);
void bms_hal_set_dsg_mos(int on);
void bms_hal_set_balance(uint16_t mask);  /* bit i bleeds cell i */
uint32_t bms_hal_millis(void);            /* unused by the core; your loop may use it */
```

Temperatures are 0.1 °C (`250` is 25.0 °C). An unused sensor slot must be `INT16_MIN`. Zero means 0.0 °C and will look too cold to charge. If every slot is `INT16_MIN`, both reported temperatures become 60 °C, both FETs open, and capacity is not learned. The core does not invent 25 °C.

```c
bms_t bms;
uint16_t holding[BMS_REG_N];

bms_init(&bms, 200000, 500);     /* 200 Ah, 50.0 % */

for (;;) {
    bms_tick(&bms);              /* every 10 ms */
    bms_regs_fill(&bms, holding);
    /* write `holding` to Modbus, or unpack it into a CAN PDO */
    delay_ms(10);
}
```

`bms_init` commands both FETs open and balance off. They close on the first healthy tick. A full or hot pack does not conduct in the gap before that tick. Calling init again mid-run opens the FETs until the next tick. Current already flowing on that first sample is the load the FETs are about to connect. It is not treated as a welded FET.

Compile `src/bms.c` and `src/bms_regs.c` with your HAL `.c`. Do not link `host/main_host.c` on the MCU. That file is a fake front end for the desktop test.

Fill `holding` from the **same context** as `bms_tick()`. Current is two 16-bit words, registers 7 and 8. A Modbus read from an ISR in the middle of `bms_regs_fill()` can tear them.

The inverter follows `allow_chg_a`, `allow_dsg_a`, `max_chg_v_x10`, and `min_dsg_v_x10`. Current at or above 2 A through a FET that was commanded open latches a leak fault after 200 ms and opens the other FET as well.

Any front end that can report 16 cell voltages is in range (BQ76952, ADBMS6830, and others). This repository has no I²C driver for them.

---

## Protection

A fault bit waits 20 ticks (200 ms) inside its set region, then latches until the measurement is past the release point. A 100 ms spike does not latch. One tick inside the hysteresis band sheds one tick of that wait. The wait starts over only after a true release. A cell sitting at 3.60 V after an over-voltage trip is still a trip: release is 3.50 V.

Open wire has no band between set and release. The bit stays until the wire has been in range for 200 ms, so one good sample cannot close the FETs.

`bms_clear_faults()` samples again and drops bits that are already past release, including an open wire that is in range on that sample. It does not set new faults, and it does not advance debounce, so a tight loop cannot trip over-voltage faster than 200 ms. It also leaves a temperature hold that is still inside its band. It does refresh the balance mask, so bleed resistors follow that sample.

Spread releases at 200 mV inclusive. 201 mV stays latched. 200 mV clears.

| | Set | Release |
|---|---|---|
| Cell over-voltage | 3.65 V | 3.50 V |
| Cell under-voltage | 2.50 V | 2.80 V |
| Charge over-current | 120 A | charge current &lt; 20 A |
| Discharge over-current | 150 A | discharge current &lt; 20 A |
| Charge over-temp (FET only, immediate) | 50.0 °C | 45.0 °C |
| Discharge over-temp (fault bit) | 60.0 °C | 50.0 °C |
| Charge under-temp (FET only, immediate) | 0.0 °C | 5.0 °C |
| Discharge under-temp (fault bit) | −20.0 °C | −10.0 °C |
| Cell spread | 400 mV | 200 mV |
| Open sense wire | &lt; 0.50 V or &gt; 4.80 V | in range for 200 ms |
| Open-FET current | ≥ 2 A in the open direction | back under 0.5 A |

An open wire at 0 V also looks like under-voltage and a large spread. The extra bit lets the inverter say “sense wire”. It is not a second path to the FETs.

Open-FET current is the pack shunt. A welded FET, or an inverter that keeps pushing into an open direction, shows up here, and both FETs open. From 0.5 A to 2 A the wait sheds one tick. If the fault that opened the FET releases while that current is still at 2 A or more, the FET stays commanded open until the wait finishes or the current falls under 0.5 A. Over-current itself releases below 20 A, so a fall from 150 A to 15 A does not reclose the FET and erase the wait.

Milliamp earth leakage is below this shunt. A residual-current device still belongs in the installation when the hazard is earth leakage. This core has no insulation measurement.

---

## Inverter limits

Registers 15–16 and 22–23 are the closed-loop interface. Current limits taper. They are not only off or full.

Charge current (`allow_chg_a`) starts at 120 A. Voltage picks one tier. Every later row can only lower that cap. The FET is already open when the cap is 0 A because of temperature.

| Condition | Cap |
|---|---|
| Highest cell ≥ 3.60 V | 5 A |
| Highest cell ≥ 3.50 V | 20 A |
| Highest cell ≥ 3.45 V | 60 A |
| Coldest cell ≤ 0 °C, or hottest ≥ 50 °C | 0 A |
| Coldest cell &lt; 10 °C, charge FET closed | 0.2 C of usable `cap_mah` (40 A at 200 Ah) |
| Hottest cell ≥ 45 °C | 20 A |
| SOC ≥ 98 % | 10 A |
| SOC ≥ 95 % | 40 A |
| Charge complete | 0 A, and the charge FET stays closed |

At 3 °C, with the charge FET still closed, that 0.2 C cap is 40 A on the 200 Ah pack. If the coldest cell has already been at or below 0 °C, the charge FET stays open until 5.0 °C, and the 0.2 C cap applies once the FET closes again. At 10.0 °C the cold taper is gone. The cap uses usable capacity, `cap_mah / 5000` amperes, not a second copy of the nameplate.

Discharge current (`allow_dsg_a`) starts at 150 A, with the same “one voltage tier, then only lower” rule.

| Condition | Cap |
|---|---|
| Lowest cell ≤ 2.70 V | 20 A |
| Lowest cell ≤ 2.90 V | 60 A |
| Coldest cell ≤ −20 °C, or hottest ≥ 60 °C | 0 A |
| Coldest cell ≤ −10.0 °C, held until −5.0 °C | 20 A |
| Coldest cell &lt; 0 °C, and the −10 °C hold is off | 40 A, fixed |
| Hottest cell ≥ 55 °C | 40 A |
| SOC ≤ 3 % | 20 A |
| SOC ≤ 10 % | 60 A |

The −10 °C discharge hold is a current cap. It does not open the discharge FET. The FET still opens at −20 °C and closes at −10 °C, and that close comes back at 20 A. It returns to 40 A only at −5.0 °C. A cell at −9 °C with the hold clear is still on the 40 A row.

**Charge voltage** (`max_chg_v_x10`) is **55.2 V** (16 × 3.45 V). If one cell is already above 3.45 V, the limit becomes the present pack voltage when that is lower, so the inverter holds instead of pushing the high cell further.

**Discharge cutoff** (`min_dsg_v_x10`) is **44.8 V** (16 × 2.80 V). If the lowest cell sags below 2.80 V, the cutoff rises to the present pack voltage and stays there until that cell is back to 2.90 V. A cell that is under 2.90 V but never went through 2.80 V does not raise it. A rebound of a few tens of millivolts does not invite the inverter back into the same cell.

**Charge complete** requires the highest cell at or above 3.50 V, SOC at or above 95 %, and current under 3 A. Once entered, it stays until the highest cell drops below 3.50 V or SOC drops below 95 %. A later spike above 3 A does not turn charge back on. The charge FET stays closed. `allow_chg_a` is 0.

**Warnings** (register 18) fire before a trip: 3.50 V, 2.80 V, 45 °C, 5 °C, 80 % of the over-current limits (96 A charge, 120 A discharge), 200 mV spread, SOC ≤ 10 % or ≥ 95 %.

**Flags** (register 19). There is no heater output pin. `HEAT` is a request only.

| Flag | Bit | Rule |
|---|---|---|
| `FULL` | 0 | Charge complete. Same window as the 0 A row above |
| `EMPTY` | 1 | Lowest cell ≤ 2.80 V, or SOC ≤ 5 % |
| `HEAT` | 2 | Coldest cell ≤ 5.0 °C and hottest cell &lt; 45.0 °C |

A single thermistor at 3 °C still requests heat. A 47 °C cell beside a 3 °C cell does not.

Any latched fault bit forces mode 3 (protect), including a one-sided fault whose other FET is still on. Read the FET bits and the current limits to see which direction is usable. Charge-only cold and hot do not set a fault bit, so mode can still say discharge while the charge FET is held open.

---

## SOC, capacity, cycles, balance

### Coulomb count

SOC is a coulomb count on a remainder, so a 10 ms tick at household current does not round away. `soc_x10 = 500` means 50.0 %. Only the direction whose FET is closed is counted. Current reported while that FET is open does not move SOC: a shunt offset during a latched fault is not charge leaving or entering the pack. The remainder is discarded at 0 % and at 100 %. Otherwise a current-sensor offset at empty can sit in the remainder and block the next charge for a long time.

### Rest snap

After 30 s with `|I| < 0.5 A`, SOC is snapped once from the **lowest** cell, and only outside 3.20–3.40 V. LFP is too flat in the middle for voltage to mean SOC. The table below is a room-temperature curve. A snap at −9 °C still uses it. A snap does not run at or below −10.0 °C: that voltage is the cold pack, not an empty one. `rest_snap` stays clear, so a warmer sample in the same rest can still snap.

| Lowest cell | 2.50 | 2.90 | 3.10 | 3.20 | 3.24 | 3.28 | 3.30 | 3.33 | 3.345 | 3.40 | 3.50 | 3.65 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| SOC | 0 % | 2 % | 5 % | 10 % | 20 % | 40 % | 60 % | 80 % | 90 % | 95 % | 98 % | 100 % |

The snap is skipped while any sense wire is open (under 0.50 V or over 4.80 V). A disconnected wire is not an empty cell and must not wipe the coulomb count. If the wire is still open at 30 s, a later in-range sample during that same rest may snap. When a snap does run, the remainder is cleared, so a leftover just under 0.1 % cannot undo it. Further current under 0.5 A still counts. Another snap needs current above 0.5 A, then another 30 s of rest. In the flat band the remainder is left alone. If the lowest cell later leaves that band during the same rest, the snap runs then.

### Usable capacity

`cap_mah` starts equal to the nameplate passed to `bms_init`. Coulomb counting and `remain_ah_x10` use it, so a faded pack does not keep advertising nameplate amp-hours.

A rest snap below 3.20 V and a later rest snap above 3.40 V, in either order, measure the coulombs that passed while the matching FET was closed. The SOC gap scales that throughput up to a full-pack capacity, and only when the net current has the same direction as the SOC change. A discharge that ends on a higher rest voltage is ignored. Both snaps must see every present sensor from 15.0 °C through 40.0 °C. Open thermistor slots are reported as 60 °C and do not learn. A rest in the 3.20–3.40 V band does not start or finish a measurement.

Usable capacity moves one quarter of the way toward the measurement. It never rises above the nameplate. A measurement below half the nameplate is ignored. One short or cold stroke cannot collapse the pack. There is no SOH percentage and no extra register for one.

Worked example, 20.000 Ah nameplate, 9.300 Ah from 3.10 V (5 %) to 3.50 V (98 %), which scales to 10.000 Ah: the first update lands on 17.500 Ah, and the return stroke lands on 15.625 Ah.

### Cycles

`cycles` increments once per full nameplate (`cap_nom_mah`) actually discharged, with the discharge FET closed and current above 0.5 A. Fade does not make that counter run faster. Both capacity values and the cycle count are RAM. Persist them from your HAL if they should survive power loss. `bms_init` sets usable and nameplate to the same number. Write a saved `cap_mah` back after init. A value outside half to full nameplate is pulled back onto that range on the next tick.

### Balance

Passive balance runs only near the top: highest cell ≥ 3.40 V and spread ≥ 25 mV. At most four cells, the highest ones. It starts only while pack current is under 1 A of discharge. Charge current does not block it. Once balance is on, it stays while discharge current is under 2 A, so a current hovering at 1 A does not chatter the bleed resistors. It turns off at 2 A and does not restart until current is under 1 A again.

The resistors stay on for 1.0 s, then off for one 10 ms tick. Bleed current sags the cell, so a sample taken while the resistors are on is not used to add a cell. The tick after that off command is the unloaded sample, and it may add a cell. A session already running at 1.5 A continues across the gap. A highest cell below 3.40 V, a spread under 25 mV, or current of 2 A or more turns the resistors off on that tick and ends the session.

Balance is off at or above 50 °C, because the bleed resistors heat the cells, and off while a sense wire is open. An open wire would otherwise look like the lowest cell, and every cell near the top would bleed. A 25 mV spread in the middle of the curve is ignored on purpose.

The open controllers that talk to Tesla Model S module boards stop balance before they read cell voltage, and they time out a bleed command so a silent master cannot leave the resistors on. The 1.0 s window is that idea, written for this LFP pack. Their cell voltages, precharge contactor, module bus, and the setting that ignores a dead thermistor are not used here. A missing thermistor is still over-temperature.

---

## Register map

Sixteen-bit holding registers. Addresses 24–31 are unused and read as 0. Cells start at 32.

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
| 11–12 | v_max / v_min | mV. A negative voltage is published as 0 |
| 13–14 | high / low cell | 1..16 |
| 15–16 | allow charge / discharge | amperes. 0 = do not use that direction |
| 17 | balance mask | bit i = cell i bleeding |
| 18 | warn | bit0 OV, 1 UV, 2 OT, 3 UT, 4 OCC, 5 OCD, 6 spread, 7 low SOC, 8 high SOC |
| 19 | flags | bit0 charge-complete, 1 empty, 2 heater requested |
| 20 | cycles | nameplate discharges, RAM only |
| 21 | remain_ah_x10 | usable Ah × SOC. 1000 = 100.0 Ah |
| 22 | max_chg_v_x10 | inverter CV, 0.1 V |
| 23 | min_dsg_v_x10 | inverter cutoff, 0.1 V |
| 32–47 | cell mV | cell 1..16 |

Rebuild current on any MCU, including a 16-bit `int`:

```c
int32_t ma = (int32_t)(((uint32_t)reg[7] << 16) | reg[8]);
```

The `uint32_t` cast is required. `(reg[7] << 16)` is undefined, or zero, when `int` is 16 bits.

---

## Files and desktop test

```
include/bms.h                 trip points, fault bits, bms_t
include/bms_hal.h             the five functions you implement
include/bms_regs.h            holding-register addresses
src/bms.c                     protect, FET limits, SOC, balance
src/bms_regs.c                pack bms_t into uint16_t[48]
host/main_host.c              fake front end and the host suite
Makefile                      make test — AddressSanitizer and UBSan
.github/workflows/check.yml   make test on every push and pull request
```

```bash
make test
```

No MCU. The binary `host/bms_test` is gitignored. The same command is the GitHub Actions job `check`.

The suite locks the behavior above, including:

- **Capacity.** 9.3 Ah from 3.10 V to 3.50 V on a 20 Ah pack moves usable capacity from 20.000 Ah to 17.500 Ah, then the return stroke to 15.625 Ah. A flat-band rest, a 1 Ah stroke, a 10 °C arrival, charge into an open FET, and a 9.3 Ah discharge that ends at 3.50 V do not move it. A stroke above nameplate stays at nameplate. A saved value below half nameplate is pulled back to half. One nameplate discharged is still one cycle.
- **Charge complete and SOC.** An 8 A spike does not clear charge-complete. It clears below 3.50 V. An open charge FET ignores charge current, and an open discharge FET ignores discharge current. 100 A for 36 s on 200 Ah moves SOC by −0.5 %. The remainder clamps at 0 % and 100 %.
- **Cutoff and balance.** A cell below 2.80 V holds the discharge cutoff until 2.90 V. Balance that is already on stays on at 1.5 A and drops at 2.5 A. It bleeds for 1.0 s, turns off for one tick, and can resume on the next tick, including at 1.5 A. A cell that rises while the resistors are on is not added until that off tick. At most four bleed resistors. One high cell holds CV at the present pack voltage.
- **Debounce.** FETs stay open until the first tick. A 100 ms over-voltage pulse is ignored. One tick at 3.60 V does not erase that wait, and a release to 3.48 V starts it over. `clear_faults` cannot speed debounce. Over-voltage opens charge only. Under-voltage opens discharge only, with spread kept under 400 mV so a spread trip cannot hide that.
- **Temperature.** 55 °C stops charge. −1 °C stops charge. −10 °C holds discharge at 20 A until −5 °C and does not snap SOC. −21 °C opens both FETs. A recovered over-voltage at 47 °C may charge again. A recovered over-current at 55 °C may discharge again. A recovered under-voltage at −15 °C may discharge again. A real 50 °C charge cut still waits for 45 °C, and a real 60 °C pack cut still holds at 55 °C. A missing thermistor is over-temperature. 0.2 C at 3 °C on the 200 Ah pack is 40 A. A 47 °C cell beside a 3 °C cell does not request heat. A hot `clear_faults` sample turns bleed off without latching over-temperature.
- **Open wire and open FET.** Spread holds at 201 mV and releases at 200 mV. An open wire does not bleed and does not wipe SOC. It stays open through 100 ms back in range and releases at 200 ms. `clear_faults` releases an in-range wire on that sample. Two amps through an open charge FET latches after 200 ms and opens discharge too. A 100 ms pulse does not, and one tick at 1 A does not erase that wait. Fifteen amps left after over-current releases keeps that FET open and then latches the leak fault. Current present on the first tick does not.
- **Rest and edges.** The 3.10 V snap after 30 s clears the remainder. 3.30 V does not snap. After the snap, a 0.4 A load still moves SOC, and a later rest can snap again. Signed current survives the register round-trip. `INT32_MIN` current and `NULL` pointers do not crash.

On an MCU, compile `src/bms.c` and `src/bms_regs.c` against your HAL.

---

## Outside this repository

- A customer dashboard or a household energy manager
- An SOH percentage, a thermal-runaway percentage, or a language model
- A drop-in `.hex` for a specific BMS board
- Microsecond short-circuit protection, a hardware desat detector, insulation monitoring, precharge, or a CAN/Modbus stack
- A substitute for the analog front end’s comparators, or for the fuse

This firmware opens MOSFETs and publishes limits on a 10 ms tick. The front end and the fuse stay in the circuit.

---

MIT. Copyright (c) 2026 Lin-Chai49. See [LICENSE](LICENSE).
