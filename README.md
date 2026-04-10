# BMS Step 1 — INA226 Sensor Reading on DE1-SoC

## Overview
FPGA-based (Cyclone V) EV Battery Management System — Step 1.  
Continuously reads the INA226 current/voltage/power sensor over I²C (400 kHz)
and streams measurement data to a PC via UART at 115200 baud.

---

## File Structure

| File | Description |
|------|-------------|
| `defines.v` | Global constants (clock dividers, I²C address, register map) |
| `i2c_master.v` | I²C master controller (400 kHz, open-drain) |
| `ina226_controller.v` | FSM: sensor init → read shunt/bus/power/current |
| `ina226_registers.v` | 16-bit register file (stores raw readings) |
| `data_converter.v` | Raw ADC → physical units (mV, µV, mA, mW) |
| `uart_baud_gen.v` | 115200 baud tick from 50 MHz clock |
| `uart_tx.v` | 8-N-1 UART transmitter |
| `bms_top.v` | Top-level module, formatter FSM, 1-second timer |
| `bms_step1_tb.v` | Testbench with I²C slave model and UART capture |
| `de1_soc_pin_assignments.csv` | Quartus pin assignments |

---

## Hardware Connections

### I²C Bus (open-drain — **mandatory 4.7 kΩ pull-ups**)

```
DE1-SoC GPIO Header (JP1)         INA226 Module
─────────────────────────         ──────────────
PIN_AG28  (GPIO[0])  ──── SCL ────  SCL
PIN_AF28  (GPIO[1])  ──── SDA ────  SDA
3.3 V header pin     ─────────────  VCC
GND header pin       ─────────────  GND

Pull-up resistors (required!):
  3.3 V ──[4.7 kΩ]── SCL line
  3.3 V ──[4.7 kΩ]── SDA line
```

### INA226 I²C Address

| A1 pin | A0 pin | Address |
|--------|--------|---------|
| GND    | GND    | **0x40** (default) |
| GND    | VS     | 0x41 |
| VS     | GND    | 0x44 |
| VS     | VS     | 0x45 |

Connect both A0 and A1 to **GND** to use the default address 0x40.

### Battery / Shunt Connections

```
Battery (+) ──────────────────── +IN (INA226)
                                    │
                              [0.1 Ω shunt]   ← use a wire-wound power resistor,
                                    │             rated for your max current
Battery (−) ──────────────────── −IN (INA226)
```

> ⚠️ **Safety:** Size the shunt resistor for your maximum load current.
> Power dissipation = I² × R.  At 10 A through 0.1 Ω → 10 W of heat.
> Use an appropriately rated resistor with heatsinking.
> For currents above 10 A, consider a **4-wire (Kelvin) shunt**.

### UART Connection

```
DE1-SoC PIN_AB21 ── TX ──► USB-UART adapter RX
GND              ── GND ── USB-UART adapter GND
```

---

## UART Output Format

Serial settings: **115200-8-N-1** (no flow control)

### Header (sent once at startup):
```
Timestamp(ms),Voltage(mV),Current(mA),Power(mW),Status
```

### Data lines (every 1 second):
```
00000000,30D4,+0028,00001E,OK
000003E8,30D1,+002A,00001F,OK
```

Fields are **hexadecimal** for compact, synthesisable formatting:

| Field | Width | Description |
|-------|-------|-------------|
| Timestamp | 8 hex digits | Milliseconds since reset |
| Voltage(mV) | 5 hex digits | Bus voltage in mV (1.25 mV/LSB) |
| Current(mA) | `+`/`-` + 5 hex | Signed current in mA (1 mA/LSB with Cal=512) |
| Power(mW) | 6 hex digits | Power in mW (25 mW/LSB) |
| Status | `OK`/`ERR` | `ERR` if any I²C NACK was detected |

---

## Data Conversion Formulas

| Measurement | Formula | Unit |
|-------------|---------|------|
| Bus Voltage | `reg[15:3] × 1.25` | mV |
| Shunt Voltage | `signed(reg) × 2.5` | µV |
| Current | `signed(reg) × 1` (Cal=512, R=0.1 Ω) | mA |
| Power | `reg × 25` | mW |

---

## Building in Quartus

1. Create a new Quartus project targeting **5CSEMA5F31C6** (DE1-SoC).
2. Add all `.v` files to the project.
3. Set **`bms_top`** as the top-level entity.
4. Import `de1_soc_pin_assignments.csv` via  
   *Assignments → Import Assignments*.
5. Compile and program the board.

---

## Simulation with ModelSim / QuestaSim / Icarus

```bash
# Icarus Verilog (free)
iverilog -g2005 -o bms_tb \
    defines.v uart_baud_gen.v uart_tx.v i2c_master.v \
    ina226_registers.v ina226_controller.v data_converter.v \
    bms_top.v bms_step1_tb.v
vvp bms_tb

# View waveforms
gtkwave bms_step1.vcd
```

---

## Integration Guide — Step 2 (Temperature Sensor)

Step 2 will add an LM75 / DS18B20 temperature sensor reading.

Recommended additions:
- Add a second I²C address in `defines.v` (e.g., `LM75_ADDR = 7'h48`).
- Extend `ina226_controller.v` FSM with LM75 read states, or create a
  dedicated `lm75_controller.v` and arbitrate via a shared `i2c_master.v`
  (add a `req`/`grant` handshake at the top level).
- Add `temp_raw [15:0]` to `ina226_registers.v` (or a new `temp_registers.v`).
- Extend `data_converter.v` with temperature conversion:
  `temp_°C = signed(reg[15:5]) × 0.125`.
- Add a `Temp(°C)` column to the UART output in `bms_top.v`.

---

## Wiring Checklist (Step-by-Step)

```
[ ] 1. Power INA226 from DE1-SoC 3.3 V header pin
[ ] 2. Connect INA226 GND to DE1-SoC GND header pin
[ ] 3. Install 4.7 kΩ pull-up from 3.3 V to SCL line
[ ] 4. Install 4.7 kΩ pull-up from 3.3 V to SDA line
[ ] 5. Connect PIN_AG28 (GPIO[0]) to INA226 SCL
[ ] 6. Connect PIN_AF28 (GPIO[1]) to INA226 SDA
[ ] 7. Set INA226 A0 = GND, A1 = GND (address 0x40)
[ ] 8. Connect battery (+) → INA226 +IN
[ ] 9. Connect battery (−) → [0.1 Ω shunt] → INA226 −IN
[ ] 10. Connect PIN_AB21 (UART TX) to USB-UART adapter RX
[ ] 11. Connect GND to USB-UART adapter GND
[ ] 12. Open terminal at 115200-8-N-1 and program the FPGA
```
