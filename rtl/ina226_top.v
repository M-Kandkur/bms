// =============================================================================
// File        : ina226_top.v
// Project     : INA226 I2C Current Sensor Interface
// Target      : DE1-SoC (Cyclone V FPGA)
// Description : Top-Level Module
//               Instantiates I2C Master, INA226 Controller, and Register File.
//               Provides open-drain I2C I/O pad interface and presents
//               measurement results to the BMS top-level logic.
//
// Pin connections (DE1-SoC GPIO header):
//   SCL -> GPIO_0[0]  (with 4.7kΩ pull-up to 3.3V)
//   SDA -> GPIO_0[1]  (with 4.7kΩ pull-up to 3.3V)
//   (See Quartus pin assignment section in integration guide)
//
// Open-Drain I/O model:
//   scl_oe=1 -> tristate buffer OE=1 (drives 0); scl_oe=0 -> OE=0 (releases)
//   The FPGA I/O cell drives '0' when enabled; the pull-up provides '1'.
//   Assign the I/O pin:
//     assign GPIO_0[0] = scl_oe ? 1'b0 : 1'bz;
//     assign GPIO_0[1] = sda_oe ? 1'b0 : 1'bz;
//
// Outputs (to BMS logic):
//   shunt_uv   - shunt voltage in microvolts (signed 32-bit)
//   bus_mv_x10 - bus voltage in 0.1 mV steps (unsigned 16-bit)
//   current_ua - current in microamperes (signed 32-bit, +ve = into battery)
//   power_uw   - power in microwatts (unsigned 32-bit)
//   data_valid - asserted after first measurement data is available
//   init_done  - asserted after INA226 initialisation completes
//   meas_valid - pulses once per complete measurement round-trip
//   sensor_err - sticky error flag (requires reset to clear)
// =============================================================================
`timescale 1ns/1ps
`include "defines.v"

module ina226_top (
    // System
    input  wire        clk,       // 50 MHz from DE1-SoC PLL / CLOCK_50
    input  wire        rst_n,     // Active-low synchronous reset

    // I2C interface (connect to FPGA I/O pads via open-drain assignment)
    output wire        scl,       // I2C clock (open-drain, active-low drive)
    inout  wire        sda,       // I2C data  (open-drain, bidirectional)

    // Measurement outputs (to BMS logic)
    output wire signed [31:0] shunt_uv,    // shunt voltage (µV)
    output wire        [15:0] bus_mv_x10,  // bus voltage   (0.1 mV steps)
    output wire signed [31:0] current_ua,  // current (µA)
    output wire        [31:0] power_uw,    // power   (µW)

    // Raw register exports (optional, for debug / extended BMS logic)
    output wire        [15:0] raw_shunt_v,
    output wire        [15:0] raw_bus_v,
    output wire        [15:0] raw_current,
    output wire        [15:0] raw_power,
    output wire        [15:0] raw_config,
    output wire        [15:0] raw_calib,

    // Status
    output wire        data_valid,
    output wire        init_done,
    output wire        meas_valid,
    output wire        sensor_err
);

// ---------------------------------------------------------------------------
// Internal wires
// ---------------------------------------------------------------------------
// I2C Master <-> Controller
wire [6:0]  i2c_addr;
wire [7:0]  i2c_reg;
wire [15:0] i2c_wr_data;
wire        i2c_cmd_write;
wire        i2c_cmd_read;
wire [15:0] i2c_rd_data;
wire        i2c_busy;
wire        i2c_done;
wire        i2c_err;

// Controller <-> Register File
wire [7:0]  reg_wr_addr;
wire [15:0] reg_wr_data;
wire        reg_wr_en;
wire [15:0] cfg_init;
wire [15:0] cal_init;
wire [15:0] mask_init;

// I2C open-drain I/O
wire        scl_oe;
wire        sda_oe;
wire        sda_in;

// ---------------------------------------------------------------------------
// Open-drain I/O pad assignments
// scl_oe=1: drive SCL to 0; scl_oe=0: release (pulled high externally)
// sda_oe=1: drive SDA to 0; sda_oe=0: release (pulled high externally)
// ---------------------------------------------------------------------------
assign scl   = scl_oe ? 1'b0 : 1'bz;
assign sda   = sda_oe ? 1'b0 : 1'bz;
assign sda_in = sda;   // read back the actual SDA line state

// ---------------------------------------------------------------------------
// I2C Master
// ---------------------------------------------------------------------------
i2c_master u_i2c_master (
    .clk        (clk),
    .rst_n      (rst_n),
    .i2c_addr   (i2c_addr),
    .reg_addr   (i2c_reg),
    .wr_data    (i2c_wr_data),
    .cmd_write  (i2c_cmd_write),
    .cmd_read   (i2c_cmd_read),
    .rd_data    (i2c_rd_data),
    .busy       (i2c_busy),
    .done       (i2c_done),
    .err        (i2c_err),
    .scl_oe     (scl_oe),
    .sda_oe     (sda_oe),
    .sda_in     (sda_in)
);

// ---------------------------------------------------------------------------
// INA226 Controller FSM
// ---------------------------------------------------------------------------
ina226_controller u_ina226_ctrl (
    .clk          (clk),
    .rst_n        (rst_n),
    .i2c_addr     (i2c_addr),
    .i2c_reg      (i2c_reg),
    .i2c_wr_data  (i2c_wr_data),
    .i2c_cmd_write(i2c_cmd_write),
    .i2c_cmd_read (i2c_cmd_read),
    .i2c_busy     (i2c_busy),
    .i2c_done     (i2c_done),
    .i2c_err      (i2c_err),
    .i2c_rd_data  (i2c_rd_data),
    .reg_wr_addr  (reg_wr_addr),
    .reg_wr_data  (reg_wr_data),
    .reg_wr_en    (reg_wr_en),
    .cfg_init     (cfg_init),
    .cal_init     (cal_init),
    .mask_init    (mask_init),
    .init_done    (init_done),
    .meas_valid   (meas_valid),
    .ctrl_err     (sensor_err)
);

// ---------------------------------------------------------------------------
// Register File
// ---------------------------------------------------------------------------
ina226_registers u_ina226_regs (
    .clk          (clk),
    .rst_n        (rst_n),
    .wr_reg_addr  (reg_wr_addr),
    .wr_reg_data  (reg_wr_data),
    .wr_en        (reg_wr_en),
    .rd_reg_addr  (8'h00),         // not used in this integration
    .rd_reg_data  (),
    .cfg_init     (cfg_init),
    .cal_init     (cal_init),
    .mask_init    (mask_init),
    .shunt_uv     (shunt_uv),
    .bus_mv       (bus_mv_x10),
    .current_ua   (current_ua),
    .power_uw     (power_uw),
    .reg_config   (raw_config),
    .reg_shunt_v  (raw_shunt_v),
    .reg_bus_v    (raw_bus_v),
    .reg_power    (raw_power),
    .reg_current  (raw_current),
    .reg_calib    (raw_calib),
    .reg_mask_en  (),
    .reg_alert_lim(),
    .data_valid   (data_valid)
);

endmodule
