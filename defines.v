// ============================================================
// defines.v — Global constants for BMS Step 1
// FPGA: DE1-SoC (Cyclone V), Clock: 50 MHz
// ============================================================
`ifndef BMS_DEFINES_V
`define BMS_DEFINES_V

// ---- Clock & timing ----------------------------------------
`define CLK_FREQ_HZ     50_000_000
`define I2C_FREQ_HZ     400_000
// I2C half-period in clock cycles (50 MHz / 400 kHz / 2 = 62)
`define I2C_HALF_PER    6'd62

// UART 115200 baud: 50 MHz / 115200 ≈ 434
`define UART_DIV        16'd434

// 1-second measurement interval (50 MHz cycles)
`define MEAS_INTERVAL   26'd50_000_000

// ---- INA226 I2C address (A0=GND, A1=GND) ------------------
`define INA226_ADDR     7'h40

// ---- INA226 register addresses ----------------------------
`define REG_CONFIG      8'h00
`define REG_SHUNT_V     8'h01
`define REG_BUS_V       8'h02
`define REG_POWER       8'h03
`define REG_CURRENT     8'h04
`define REG_CALIB       8'h05

// ---- INA226 configuration word ----------------------------
// AVG=16, VBUS_CT=1.1ms, VSH_CT=1.1ms, mode=continuous shunt+bus
`define INA226_CFG      16'h4527

// ---- Calibration register value ---------------------------
// Cal = 0.00512 / (Current_LSB * Rshunt)
// Current_LSB = 1 mA, Rshunt = 0.1 Ω → Cal = 512
`define INA226_CAL      16'd512

`endif // BMS_DEFINES_V
