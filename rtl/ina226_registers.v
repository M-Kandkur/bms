// =============================================================================
// File        : ina226_registers.v
// Project     : INA226 I2C Current Sensor Interface
// Target      : DE1-SoC (Cyclone V FPGA)
// Description : Register file for INA226 measurement results and configuration.
//               Stores all INA226 register values received via I2C and provides
//               formatted output values to the rest of the BMS design.
//
// INA226 Register Map (all 16-bit, big-endian on I2C):
//   0x00  Configuration   (R/W)
//   0x01  Shunt Voltage   (R)   - 2's complement, LSB = 2.5 uV
//   0x02  Bus Voltage     (R)   - unsigned,       LSB = 1.25 mV
//   0x03  Power           (R)   - unsigned,       LSB = Power_LSB (2.5 mW here)
//   0x04  Current         (R)   - 2's complement, LSB = Current_LSB (100 uA here)
//   0x05  Calibration     (R/W)
//   0x06  Mask/Enable     (R/W)
//   0x07  Alert Limit     (R/W)
//
// Scaled outputs (all integer, no floating-point):
//   shunt_uv   : shunt voltage in microvolts (signed 32-bit)
//   bus_mv     : bus voltage in millivolts (unsigned 16-bit, 0..36000)
//   current_ua : current in microamperes   (signed 32-bit)
//   power_uw   : power in microwatts       (unsigned 32-bit)
// =============================================================================
`timescale 1ns/1ps
`include "defines.v"

module ina226_registers (
    input  wire        clk,
    input  wire        rst_n,

    // Write interface from controller
    input  wire [7:0]  wr_reg_addr,   // register address to write
    input  wire [15:0] wr_reg_data,   // data to write
    input  wire        wr_en,         // write enable pulse

    // Read interface from controller (for I2C read-back)
    input  wire [7:0]  rd_reg_addr,   // register to read out to I2C master
    output reg  [15:0] rd_reg_data,   // data to send over I2C

    // Initialisation values (loaded from defines)
    output wire [15:0] cfg_init,      // CONFIG register init value
    output wire [15:0] cal_init,      // CALIBRATION register init value
    output wire [15:0] mask_init,     // MASK/ENABLE register init value

    // Measurement outputs (scaled integer values)
    output reg  signed [31:0] shunt_uv,    // shunt voltage (µV)
    output reg         [15:0] bus_mv,      // bus voltage   (mV * 10 = 0.1mV steps)
    output reg  signed [31:0] current_ua,  // current (µA)
    output reg         [31:0] power_uw,    // power   (µW)

    // Raw register outputs (for debug / downstream use)
    output reg  [15:0] reg_config,
    output reg  [15:0] reg_shunt_v,
    output reg  [15:0] reg_bus_v,
    output reg  [15:0] reg_power,
    output reg  [15:0] reg_current,
    output reg  [15:0] reg_calib,
    output reg  [15:0] reg_mask_en,
    output reg  [15:0] reg_alert_lim,

    // Data valid flag (set after first complete measurement cycle)
    output reg         data_valid
);

// ---------------------------------------------------------------------------
// Constant initialisation outputs
// ---------------------------------------------------------------------------
assign cfg_init  = `CONFIG_VALUE;
assign cal_init  = `CALIB_VALUE;
assign mask_init = `MASK_EN_VALUE;

// ---------------------------------------------------------------------------
// Register file (initialised to INA226 power-on defaults where applicable)
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        reg_config    <= 16'h4127;  // INA226 power-on default
        reg_shunt_v   <= 16'h0000;
        reg_bus_v     <= 16'h0000;
        reg_power     <= 16'h0000;
        reg_current   <= 16'h0000;
        reg_calib     <= 16'h0000;
        reg_mask_en   <= 16'h0000;
        reg_alert_lim <= 16'h0000;
        data_valid    <= 1'b0;
    end else if (wr_en) begin
        case (wr_reg_addr)
            `REG_CONFIG:      reg_config    <= wr_reg_data;
            `REG_SHUNT_V:     begin
                                reg_shunt_v <= wr_reg_data;
                                data_valid  <= 1'b1;  // first measurement arrived
                              end
            `REG_BUS_V:       reg_bus_v     <= wr_reg_data;
            `REG_POWER:       reg_power     <= wr_reg_data;
            `REG_CURRENT:     reg_current   <= wr_reg_data;
            `REG_CALIB:       reg_calib     <= wr_reg_data;
            `REG_MASK_ENABLE: reg_mask_en   <= wr_reg_data;
            `REG_ALERT_LIMIT: reg_alert_lim <= wr_reg_data;
            default: ;
        endcase
    end
end

// ---------------------------------------------------------------------------
// Read-back multiplexer (used when controller needs to read a register to
// supply to the I2C master for comparison / verification)
// ---------------------------------------------------------------------------
always @(*) begin
    case (rd_reg_addr)
        `REG_CONFIG:      rd_reg_data = reg_config;
        `REG_SHUNT_V:     rd_reg_data = reg_shunt_v;
        `REG_BUS_V:       rd_reg_data = reg_bus_v;
        `REG_POWER:       rd_reg_data = reg_power;
        `REG_CURRENT:     rd_reg_data = reg_current;
        `REG_CALIB:       rd_reg_data = reg_calib;
        `REG_MASK_ENABLE: rd_reg_data = reg_mask_en;
        `REG_ALERT_LIMIT: rd_reg_data = reg_alert_lim;
        default:          rd_reg_data = 16'hxxxx;
    endcase
end

// ---------------------------------------------------------------------------
// Scaled value calculations
// ---------------------------------------------------------------------------
// Shunt Voltage (REG 0x01):
//   Raw value is 2's complement 16-bit, LSB = 2.5 µV
//   shunt_uv = raw * 5 / 2  (avoid 2.5 fraction -> *5 >> 1)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        shunt_uv   <= 32'sd0;
        bus_mv     <= 16'd0;
        current_ua <= 32'sd0;
        power_uw   <= 32'd0;
    end else if (wr_en) begin
        case (wr_reg_addr)
            `REG_SHUNT_V: begin
                // 2's complement, LSB = 2.5 uV
                // multiply by 5 then right-shift 1 to get uV
                shunt_uv <= ($signed(wr_reg_data) * $signed(32'sd5)) >>> 1;
            end
            `REG_BUS_V: begin
                // unsigned, LSB = 1.25 mV
                // bus_mv in units of 0.1 mV: raw * 125 / 100 = raw * 5 / 4
                // Store as mV * 10 to keep integer. bus_mv = raw * 5 >> 2
                bus_mv <= (wr_reg_data * 16'd5) >> 2;
            end
            `REG_CURRENT: begin
                // 2's complement, LSB = 100 uA (Current_LSB = 100 uA)
                current_ua <= $signed(wr_reg_data) * $signed(32'sd100);
            end
            `REG_POWER: begin
                // unsigned, LSB = 2500 uW (Power_LSB = 2.5 mW = 2500 uW)
                power_uw <= wr_reg_data * 32'd2500;
            end
            default: ;
        endcase
    end
end

endmodule
