// =============================================================================
// File        : defines.v
// Project     : INA226 I2C Current Sensor Interface
// Target      : DE1-SoC (Cyclone V FPGA)
// Description : Global defines, INA226 register addresses, and constants
// =============================================================================

`ifndef INA226_DEFINES_V
`define INA226_DEFINES_V

// ---------------------------------------------------------------------------
// FPGA / Clock Parameters
// ---------------------------------------------------------------------------
`define FPGA_CLK_HZ       50_000_000   // 50 MHz system clock
`define I2C_CLK_HZ           400_000   // 400 kHz Fast-Mode I2C

// Clock divider: periods of FPGA_CLK per I2C quarter-period
// SCL period = FPGA_CLK / I2C_CLK = 125 cycles
// Quarter-period = 125 / 4 = 31 (rounded)
`define I2C_CLK_DIV       7'd31        // quarter-period counter limit

// ---------------------------------------------------------------------------
// INA226 I2C Address
// ---------------------------------------------------------------------------
// A1=GND, A0=GND -> 0x40 (default)
`define INA226_ADDR       7'h40

// ---------------------------------------------------------------------------
// INA226 Register Addresses (8-bit pointer)
// ---------------------------------------------------------------------------
`define REG_CONFIG        8'h00   // Configuration Register
`define REG_SHUNT_V       8'h01   // Shunt Voltage Register
`define REG_BUS_V         8'h02   // Bus Voltage Register
`define REG_POWER         8'h03   // Power Register
`define REG_CURRENT       8'h04   // Current Register
`define REG_CALIB         8'h05   // Calibration Register
`define REG_MASK_ENABLE   8'h06   // Mask/Enable (Alert) Register
`define REG_ALERT_LIMIT   8'h07   // Alert Limit Register
`define REG_MFR_ID        8'hFE   // Manufacturer ID (0x5449)
`define REG_DIE_ID        8'hFF   // Die ID (0x2260)

// ---------------------------------------------------------------------------
// INA226 Configuration Register Bit Fields (default 0x4127)
// Bit 15    : RST  - Software reset
// Bits 14:12: reserved
// Bits 11:9 : AVG  - Averaging mode
// Bits 8:6  : VBUSCT - Bus voltage conversion time
// Bits 5:3  : VSHCT - Shunt voltage conversion time
// Bits 2:0  : MODE - Operating mode
// ---------------------------------------------------------------------------
// AVG selections
`define AVG_1             3'b000   // 1 sample average
`define AVG_4             3'b001
`define AVG_16            3'b010
`define AVG_64            3'b011
`define AVG_128           3'b100
`define AVG_256           3'b101
`define AVG_512           3'b110
`define AVG_1024          3'b111

// Conversion time selections
`define CT_140US          3'b000
`define CT_204US          3'b001
`define CT_332US          3'b010
`define CT_588US          3'b011
`define CT_1100US         3'b100
`define CT_2116US         3'b101
`define CT_4156US         3'b110
`define CT_8244US         3'b111

// Operating mode
`define MODE_POWER_DOWN   3'b000
`define MODE_SHUNT_TRIG   3'b001
`define MODE_BUS_TRIG     3'b010
`define MODE_SHUNT_BUS_T  3'b011
`define MODE_POWER_DOWN2  3'b100
`define MODE_SHUNT_CONT   3'b101
`define MODE_BUS_CONT     3'b110
`define MODE_SHUNT_BUS_C  3'b111   // Continuous shunt + bus

// Target configuration: AVG=16, VBUSCT=1100us, VSHCT=1100us, MODE=shunt+bus cont
// Config = {1'b0, 3'b000, AVG_16, CT_1100US, CT_1100US, MODE_SHUNT_BUS_C}
//        = 16'h4527
`define CONFIG_VALUE      16'h4527

// ---------------------------------------------------------------------------
// Calibration Register
// CAL = 0.00512 / (Current_LSB * R_shunt)
// Current_LSB = 100uA (0.0001 A)
//   CAL = 0.00512 / (0.0001 * 0.1) = 512 = 0x0200
// Results in Current_LSB = 100 uA, Power_LSB = 2.5 mW
// ---------------------------------------------------------------------------
`define CALIB_VALUE       16'h0200

// ---------------------------------------------------------------------------
// Mask/Enable Register - enable conversion-ready flag (bit 10 = CVRF)
// ---------------------------------------------------------------------------
`define MASK_EN_VALUE     16'h0400

// ---------------------------------------------------------------------------
// I2C Master internal state encoding
// ---------------------------------------------------------------------------
`define I2C_IDLE          4'd0
`define I2C_START         4'd1
`define I2C_ADDR          4'd2
`define I2C_ADDR_ACK      4'd3
`define I2C_REG_PTR       4'd4
`define I2C_REG_PTR_ACK   4'd5
`define I2C_DATA_WR       4'd6
`define I2C_DATA_WR_ACK   4'd7
`define I2C_RSTART        4'd8
`define I2C_DATA_RD       4'd9
`define I2C_DATA_RD_ACK   4'd10
`define I2C_STOP          4'd11
`define I2C_ERROR         4'd12

// ---------------------------------------------------------------------------
// INA226 Controller FSM state encoding
// ---------------------------------------------------------------------------
`define CTRL_RESET        4'd0
`define CTRL_INIT_CFG     4'd1
`define CTRL_INIT_CAL     4'd2
`define CTRL_INIT_ALERT   4'd3
`define CTRL_WAIT_RDY     4'd4
`define CTRL_RD_SHUNT     4'd5
`define CTRL_RD_BUS       4'd6
`define CTRL_RD_CURRENT   4'd7
`define CTRL_RD_POWER     4'd8
`define CTRL_DONE         4'd9
`define CTRL_ERROR        4'd15

`endif // INA226_DEFINES_V
