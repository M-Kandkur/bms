// =============================================================================
// File        : ina226_controller.v
// Project     : INA226 I2C Current Sensor Interface
// Target      : DE1-SoC (Cyclone V FPGA)
// Description : INA226 Controller FSM
//               Orchestrates the I2C Master to:
//               1. Initialise INA226 (write Config, Calibration, Mask/Enable)
//               2. Continuously read measurement registers in round-robin:
//                  Shunt Voltage -> Bus Voltage -> Current -> Power -> repeat
//               Forwards read data to the register file.
//
// FSM State Sequence:
//   RESET -> INIT_CFG -> INIT_CAL -> INIT_ALERT ->
//   WAIT_RDY -> RD_SHUNT -> RD_BUS -> RD_CURRENT -> RD_POWER -> (loop)
//
// The controller waits for the I2C Master 'done' pulse between each
// transaction to ensure the bus is free.
// =============================================================================
`timescale 1ns/1ps
`include "defines.v"

module ina226_controller (
    input  wire        clk,
    input  wire        rst_n,

    // I2C Master command interface
    output reg  [6:0]  i2c_addr,
    output reg  [7:0]  i2c_reg,
    output reg  [15:0] i2c_wr_data,
    output reg         i2c_cmd_write,
    output reg         i2c_cmd_read,

    // I2C Master status
    input  wire        i2c_busy,
    input  wire        i2c_done,
    input  wire        i2c_err,
    input  wire [15:0] i2c_rd_data,

    // Register file write interface
    output reg  [7:0]  reg_wr_addr,
    output reg  [15:0] reg_wr_data,
    output reg         reg_wr_en,

    // Initialisation values from register file
    input  wire [15:0] cfg_init,
    input  wire [15:0] cal_init,
    input  wire [15:0] mask_init,

    // Status outputs
    output reg         init_done,     // high after initialisation completes
    output reg         meas_valid,    // pulses once per complete measurement set
    output reg         ctrl_err       // sticky error flag
);

// ---------------------------------------------------------------------------
// Controller FSM states
// ---------------------------------------------------------------------------
localparam ST_RESET      = 4'd0;
localparam ST_INIT_CFG   = 4'd1;
localparam ST_WAIT_CFG   = 4'd2;
localparam ST_INIT_CAL   = 4'd3;
localparam ST_WAIT_CAL   = 4'd4;
localparam ST_INIT_ALERT = 4'd5;
localparam ST_WAIT_ALERT = 4'd6;
localparam ST_RD_SHUNT   = 4'd7;
localparam ST_WAIT_SHUNT = 4'd8;
localparam ST_RD_BUS     = 4'd9;
localparam ST_WAIT_BUS   = 4'd10;
localparam ST_RD_CURRENT = 4'd11;
localparam ST_WAIT_CUR   = 4'd12;
localparam ST_RD_POWER   = 4'd13;
localparam ST_WAIT_PWR   = 4'd14;
localparam ST_ERROR      = 4'd15;

reg [3:0] state;

// ---------------------------------------------------------------------------
// Reset hold counter: wait 200 us after reset before starting I2C
// 200 us * 50 MHz = 10,000 cycles
// ---------------------------------------------------------------------------
reg [13:0] rst_cnt;
localparam RST_HOLD = 14'd10_000;

// ---------------------------------------------------------------------------
// Error retry counter
// ---------------------------------------------------------------------------
reg [3:0] err_cnt;
localparam MAX_RETRIES = 4'd3;

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state         <= ST_RESET;
        i2c_addr      <= `INA226_ADDR;
        i2c_reg       <= 8'd0;
        i2c_wr_data   <= 16'd0;
        i2c_cmd_write <= 1'b0;
        i2c_cmd_read  <= 1'b0;
        reg_wr_addr   <= 8'd0;
        reg_wr_data   <= 16'd0;
        reg_wr_en     <= 1'b0;
        init_done     <= 1'b0;
        meas_valid    <= 1'b0;
        ctrl_err      <= 1'b0;
        rst_cnt       <= 14'd0;
        err_cnt       <= 4'd0;
    end else begin
        // Default pulse clears
        i2c_cmd_write <= 1'b0;
        i2c_cmd_read  <= 1'b0;
        reg_wr_en     <= 1'b0;
        meas_valid    <= 1'b0;

        case (state)

            // ----------------------------------------------------------------
            // Hold after reset - wait for INA226 power-up (200 us min)
            // ----------------------------------------------------------------
            ST_RESET: begin
                init_done <= 1'b0;
                if (rst_cnt >= RST_HOLD) begin
                    rst_cnt <= 14'd0;
                    state   <= ST_INIT_CFG;
                end else begin
                    rst_cnt <= rst_cnt + 14'd1;
                end
            end

            // ----------------------------------------------------------------
            // Write Configuration Register
            // ----------------------------------------------------------------
            ST_INIT_CFG: begin
                if (!i2c_busy) begin
                    i2c_addr      <= `INA226_ADDR;
                    i2c_reg       <= `REG_CONFIG;
                    i2c_wr_data   <= cfg_init;
                    i2c_cmd_write <= 1'b1;
                    state         <= ST_WAIT_CFG;
                end
            end

            ST_WAIT_CFG: begin
                if (i2c_err) begin
                    state <= ST_ERROR;
                end else if (i2c_done) begin
                    // Mirror write to register file
                    reg_wr_addr <= `REG_CONFIG;
                    reg_wr_data <= cfg_init;
                    reg_wr_en   <= 1'b1;
                    state       <= ST_INIT_CAL;
                end
            end

            // ----------------------------------------------------------------
            // Write Calibration Register
            // ----------------------------------------------------------------
            ST_INIT_CAL: begin
                if (!i2c_busy) begin
                    i2c_addr      <= `INA226_ADDR;
                    i2c_reg       <= `REG_CALIB;
                    i2c_wr_data   <= cal_init;
                    i2c_cmd_write <= 1'b1;
                    state         <= ST_WAIT_CAL;
                end
            end

            ST_WAIT_CAL: begin
                if (i2c_err) begin
                    state <= ST_ERROR;
                end else if (i2c_done) begin
                    reg_wr_addr <= `REG_CALIB;
                    reg_wr_data <= cal_init;
                    reg_wr_en   <= 1'b1;
                    state       <= ST_INIT_ALERT;
                end
            end

            // ----------------------------------------------------------------
            // Write Mask/Enable Register (enable CVRF alert)
            // ----------------------------------------------------------------
            ST_INIT_ALERT: begin
                if (!i2c_busy) begin
                    i2c_addr      <= `INA226_ADDR;
                    i2c_reg       <= `REG_MASK_ENABLE;
                    i2c_wr_data   <= mask_init;
                    i2c_cmd_write <= 1'b1;
                    state         <= ST_WAIT_ALERT;
                end
            end

            ST_WAIT_ALERT: begin
                if (i2c_err) begin
                    state <= ST_ERROR;
                end else if (i2c_done) begin
                    reg_wr_addr <= `REG_MASK_ENABLE;
                    reg_wr_data <= mask_init;
                    reg_wr_en   <= 1'b1;
                    init_done   <= 1'b1;
                    err_cnt     <= 4'd0;
                    state       <= ST_RD_SHUNT;
                end
            end

            // ----------------------------------------------------------------
            // Read Shunt Voltage Register (0x01)
            // ----------------------------------------------------------------
            ST_RD_SHUNT: begin
                if (!i2c_busy) begin
                    i2c_addr     <= `INA226_ADDR;
                    i2c_reg      <= `REG_SHUNT_V;
                    i2c_cmd_read <= 1'b1;
                    state        <= ST_WAIT_SHUNT;
                end
            end

            ST_WAIT_SHUNT: begin
                if (i2c_err) begin
                    state <= ST_ERROR;
                end else if (i2c_done) begin
                    reg_wr_addr <= `REG_SHUNT_V;
                    reg_wr_data <= i2c_rd_data;
                    reg_wr_en   <= 1'b1;
                    state       <= ST_RD_BUS;
                end
            end

            // ----------------------------------------------------------------
            // Read Bus Voltage Register (0x02)
            // ----------------------------------------------------------------
            ST_RD_BUS: begin
                if (!i2c_busy) begin
                    i2c_addr     <= `INA226_ADDR;
                    i2c_reg      <= `REG_BUS_V;
                    i2c_cmd_read <= 1'b1;
                    state        <= ST_WAIT_BUS;
                end
            end

            ST_WAIT_BUS: begin
                if (i2c_err) begin
                    state <= ST_ERROR;
                end else if (i2c_done) begin
                    reg_wr_addr <= `REG_BUS_V;
                    reg_wr_data <= i2c_rd_data;
                    reg_wr_en   <= 1'b1;
                    state       <= ST_RD_CURRENT;
                end
            end

            // ----------------------------------------------------------------
            // Read Current Register (0x04)
            // ----------------------------------------------------------------
            ST_RD_CURRENT: begin
                if (!i2c_busy) begin
                    i2c_addr     <= `INA226_ADDR;
                    i2c_reg      <= `REG_CURRENT;
                    i2c_cmd_read <= 1'b1;
                    state        <= ST_WAIT_CUR;
                end
            end

            ST_WAIT_CUR: begin
                if (i2c_err) begin
                    state <= ST_ERROR;
                end else if (i2c_done) begin
                    reg_wr_addr <= `REG_CURRENT;
                    reg_wr_data <= i2c_rd_data;
                    reg_wr_en   <= 1'b1;
                    state       <= ST_RD_POWER;
                end
            end

            // ----------------------------------------------------------------
            // Read Power Register (0x03)
            // ----------------------------------------------------------------
            ST_RD_POWER: begin
                if (!i2c_busy) begin
                    i2c_addr     <= `INA226_ADDR;
                    i2c_reg      <= `REG_POWER;
                    i2c_cmd_read <= 1'b1;
                    state        <= ST_WAIT_PWR;
                end
            end

            ST_WAIT_PWR: begin
                if (i2c_err) begin
                    state <= ST_ERROR;
                end else if (i2c_done) begin
                    reg_wr_addr <= `REG_POWER;
                    reg_wr_data <= i2c_rd_data;
                    reg_wr_en   <= 1'b1;
                    meas_valid  <= 1'b1;   // one complete cycle done
                    err_cnt     <= 4'd0;   // reset error counter on success
                    state       <= ST_RD_SHUNT;  // loop back
                end
            end

            // ----------------------------------------------------------------
            // Error handler: retry up to MAX_RETRIES, then assert ctrl_err
            // ----------------------------------------------------------------
            ST_ERROR: begin
                if (err_cnt >= MAX_RETRIES) begin
                    ctrl_err <= 1'b1;
                    // Stay in error state - top-level must reset
                end else begin
                    err_cnt <= err_cnt + 4'd1;
                    // Retry from beginning of measurement cycle
                    state   <= ST_RD_SHUNT;
                end
            end

            default: state <= ST_RESET;
        endcase
    end
end

endmodule
