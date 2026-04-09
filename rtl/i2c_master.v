// =============================================================================
// File        : i2c_master.v
// Project     : INA226 I2C Current Sensor Interface
// Target      : DE1-SoC (Cyclone V FPGA)
// Description : I2C Master Controller
//               - Generates 400 kHz SCL from 50 MHz FPGA clock
//               - Manages SDA/SCL open-drain lines
//               - Generates START / Repeated-START / STOP conditions
//               - Performs 8-bit byte transfers with ACK/NACK detection
//               - Supports multi-byte write (reg pointer + 2 data bytes)
//               - Supports multi-byte read  (write pointer, repeated START,
//                 read 2 bytes)
//               - Error detection: NACK, bus timeout
//
// I2C Timing (400 kHz Fast-Mode, 50 MHz FPGA clock):
//   SCL period  = 125 FPGA clocks (2.5 us)
//   Quarter-period = 31 clocks
//   Phases: 0=SCL_LO, 1=SCL_RISE, 2=SCL_HI(sample), 3=SCL_FALL
//
// Interface:
//   clk        - 50 MHz system clock
//   rst_n      - Active-low synchronous reset
//   i2c_addr   - 7-bit device address
//   reg_addr   - 8-bit register pointer
//   wr_data    - 16-bit data to write
//   cmd_write  - Pulse: initiate a 16-bit register write
//   cmd_read   - Pulse: initiate a 16-bit register read
//   rd_data    - 16-bit data returned from read
//   busy       - High while transaction is in progress
//   done       - One-cycle pulse when transaction completes successfully
//   err        - One-cycle pulse on error (NACK / timeout)
//   scl_oe     - 1=drive SCL low (open-drain); 0=release (pulled high)
//   sda_oe     - 1=drive SDA low (open-drain); 0=release (pulled high)
//   sda_in     - registered SDA line input from I/O cell
// =============================================================================
`timescale 1ns/1ps
`include "defines.v"

module i2c_master (
    input  wire        clk,
    input  wire        rst_n,
    // Command interface
    input  wire [6:0]  i2c_addr,
    input  wire [7:0]  reg_addr,
    input  wire [15:0] wr_data,
    input  wire        cmd_write,
    input  wire        cmd_read,
    // Result interface
    output reg  [15:0] rd_data,
    output reg         busy,
    output reg         done,
    output reg         err,
    // I2C lines (open-drain)
    output reg         scl_oe,
    output reg         sda_oe,
    input  wire        sda_in
);

// ---------------------------------------------------------------------------
// Quarter-period clock divider
// Four phases per SCL period: SCL_LO(0), RISE(1), SCL_HI(2), FALL(3)
// ---------------------------------------------------------------------------
reg [6:0] clk_cnt;
reg [1:0] phase;

wire phase_tick = (clk_cnt == `I2C_CLK_DIV);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clk_cnt <= 7'd0;
        phase   <= 2'd0;
    end else if (busy) begin
        if (phase_tick) begin
            clk_cnt <= 7'd0;
            phase   <= phase + 2'd1;
        end else begin
            clk_cnt <= clk_cnt + 7'd1;
        end
    end else begin
        clk_cnt <= 7'd0;
        phase   <= 2'd0;
    end
end

// Phase-tick pulses (fired at the END of each phase, i.e. transition tick)
wire tick_p0 = phase_tick && (phase == 2'd0); // leaving phase 0 -> entering phase 1
wire tick_p1 = phase_tick && (phase == 2'd1); // leaving phase 1 -> entering phase 2
wire tick_p2 = phase_tick && (phase == 2'd2); // leaving phase 2 -> entering phase 3
wire tick_p3 = phase_tick && (phase == 2'd3); // leaving phase 3 -> entering phase 0

// ---------------------------------------------------------------------------
// FSM state encoding (local, avoids include naming conflict)
// ---------------------------------------------------------------------------
localparam ST_IDLE        = 4'd0;
localparam ST_START       = 4'd1;
localparam ST_ADDR_WR     = 4'd2;  // Send addr + W
localparam ST_ADDR_WR_ACK = 4'd3;
localparam ST_REG_PTR     = 4'd4;  // Send register pointer byte
localparam ST_REG_PTR_ACK = 4'd5;
localparam ST_WR_DATA     = 4'd6;  // Write data byte(s)
localparam ST_WR_DATA_ACK = 4'd7;
localparam ST_RSTART      = 4'd8;  // Repeated START
localparam ST_ADDR_RD     = 4'd9;  // Send addr + R
localparam ST_ADDR_RD_ACK = 4'd10;
localparam ST_RD_DATA     = 4'd11; // Read data byte(s)
localparam ST_RD_ACK      = 4'd12; // Master ACK/NACK
localparam ST_STOP        = 4'd13;
localparam ST_ERROR       = 4'd14;

reg [3:0]  state;
reg [2:0]  bit_cnt;    // bit index within current byte (7 down to 0)
reg        byte_hi;    // 0=high byte, 1=low byte for 16-bit data
reg        is_read;    // latched at start: 1=read operation

// Shift / data registers
reg [7:0]  tx_byte;
reg [15:0] rx_shift;

// Latched inputs
reg [6:0]  addr_l;
reg [7:0]  reg_l;
reg [15:0] wr_l;

// Timeout watchdog
reg [19:0] timeout_cnt;
localparam TIMEOUT_LIMIT = 20'd500_000; // ~10 ms at 50 MHz

// ---------------------------------------------------------------------------
// Main FSM
// ---------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= ST_IDLE;
        busy        <= 1'b0;
        done        <= 1'b0;
        err         <= 1'b0;
        scl_oe      <= 1'b0;
        sda_oe      <= 1'b0;
        rd_data     <= 16'd0;
        bit_cnt     <= 3'd7;
        byte_hi     <= 1'b1;
        is_read     <= 1'b0;
        tx_byte     <= 8'd0;
        rx_shift    <= 16'd0;
        addr_l      <= 7'd0;
        reg_l       <= 8'd0;
        wr_l        <= 16'd0;
        timeout_cnt <= 20'd0;
    end else begin
        done <= 1'b0;
        err  <= 1'b0;

        // ---- Timeout watchdog ----
        if (busy) begin
            if (timeout_cnt >= TIMEOUT_LIMIT) begin
                timeout_cnt <= 20'd0;
                state       <= ST_ERROR;
            end else begin
                timeout_cnt <= timeout_cnt + 20'd1;
            end
        end else begin
            timeout_cnt <= 20'd0;
        end

        case (state)

            // ----------------------------------------------------------------
            ST_IDLE: begin
                scl_oe <= 1'b0;
                sda_oe <= 1'b0;
                if (cmd_write || cmd_read) begin
                    addr_l  <= i2c_addr;
                    reg_l   <= reg_addr;
                    wr_l    <= wr_data;
                    is_read <= cmd_read;
                    busy    <= 1'b1;
                    state   <= ST_START;
                end
            end

            // ----------------------------------------------------------------
            // START: drive SDA low while SCL is high
            // We enter with SCL/SDA released (both high).
            // Drive SDA low immediately, then drive SCL low after a quarter period.
            // ----------------------------------------------------------------
            ST_START: begin
                sda_oe <= 1'b1;   // SDA low (START)
                if (tick_p0) begin
                    scl_oe  <= 1'b1;            // SCL low
                    tx_byte <= {addr_l, 1'b0};  // write address frame
                    bit_cnt <= 3'd7;
                    state   <= ST_ADDR_WR;
                end
            end

            // ----------------------------------------------------------------
            // Send address + W (7-bit addr, R/W=0)
            // ----------------------------------------------------------------
            ST_ADDR_WR: begin
                // Phase 0 (SCL low): set SDA before SCL rises
                if (tick_p3) sda_oe <= ~tx_byte[bit_cnt];
                if (tick_p0) scl_oe <= 1'b0;   // SCL rises
                if (tick_p2) scl_oe <= 1'b1;   // SCL falls
                if (tick_p2) begin
                    if (bit_cnt == 3'd0) begin
                        sda_oe  <= 1'b0;        // release SDA for ACK
                        state   <= ST_ADDR_WR_ACK;
                    end else begin
                        bit_cnt <= bit_cnt - 3'd1;
                    end
                end
            end

            // ----------------------------------------------------------------
            ST_ADDR_WR_ACK: begin
                if (tick_p0) scl_oe <= 1'b0;   // SCL rises
                if (tick_p1) begin
                    if (sda_in) state <= ST_ERROR; // NACK
                end
                if (tick_p2) begin
                    scl_oe  <= 1'b1;            // SCL falls
                    tx_byte <= reg_l;
                    bit_cnt <= 3'd7;
                    state   <= ST_REG_PTR;
                end
            end

            // ----------------------------------------------------------------
            // Send register pointer byte
            // ----------------------------------------------------------------
            ST_REG_PTR: begin
                if (tick_p3) sda_oe <= ~tx_byte[bit_cnt];
                if (tick_p0) scl_oe <= 1'b0;
                if (tick_p2) scl_oe <= 1'b1;
                if (tick_p2) begin
                    if (bit_cnt == 3'd0) begin
                        sda_oe <= 1'b0;
                        state  <= ST_REG_PTR_ACK;
                    end else begin
                        bit_cnt <= bit_cnt - 3'd1;
                    end
                end
            end

            // ----------------------------------------------------------------
            ST_REG_PTR_ACK: begin
                if (tick_p0) scl_oe <= 1'b0;
                if (tick_p1) begin
                    if (sda_in) state <= ST_ERROR;
                end
                if (tick_p2) begin
                    scl_oe <= 1'b1;
                    if (is_read) begin
                        sda_oe <= 1'b0;     // release SDA before repeated START
                        state  <= ST_RSTART;
                    end else begin
                        tx_byte <= wr_l[15:8];
                        bit_cnt <= 3'd7;
                        byte_hi <= 1'b1;
                        state   <= ST_WR_DATA;
                    end
                end
            end

            // ----------------------------------------------------------------
            // Write data byte (high then low)
            // ----------------------------------------------------------------
            ST_WR_DATA: begin
                if (tick_p3) sda_oe <= ~tx_byte[bit_cnt];
                if (tick_p0) scl_oe <= 1'b0;
                if (tick_p2) scl_oe <= 1'b1;
                if (tick_p2) begin
                    if (bit_cnt == 3'd0) begin
                        sda_oe <= 1'b0;
                        state  <= ST_WR_DATA_ACK;
                    end else begin
                        bit_cnt <= bit_cnt - 3'd1;
                    end
                end
            end

            // ----------------------------------------------------------------
            ST_WR_DATA_ACK: begin
                if (tick_p0) scl_oe <= 1'b0;
                if (tick_p1) begin
                    if (sda_in) state <= ST_ERROR;
                end
                if (tick_p2) begin
                    scl_oe <= 1'b1;
                    if (byte_hi) begin
                        tx_byte <= wr_l[7:0];
                        bit_cnt <= 3'd7;
                        byte_hi <= 1'b0;
                        state   <= ST_WR_DATA;
                    end else begin
                        state <= ST_STOP;
                    end
                end
            end

            // ----------------------------------------------------------------
            // Repeated START: SCL low -> SCL high -> SDA high -> SDA low
            // ----------------------------------------------------------------
            ST_RSTART: begin
                if (tick_p3) begin
                    scl_oe <= 1'b0;   // SCL rises
                    sda_oe <= 1'b0;   // SDA high
                end
                if (tick_p1) begin
                    sda_oe <= 1'b1;   // SDA falls while SCL is high = repeated START
                end
                if (tick_p2) begin
                    scl_oe  <= 1'b1;            // SCL falls
                    tx_byte <= {addr_l, 1'b1};  // read address frame
                    bit_cnt <= 3'd7;
                    state   <= ST_ADDR_RD;
                end
            end

            // ----------------------------------------------------------------
            // Send address + R
            // ----------------------------------------------------------------
            ST_ADDR_RD: begin
                if (tick_p3) sda_oe <= ~tx_byte[bit_cnt];
                if (tick_p0) scl_oe <= 1'b0;
                if (tick_p2) scl_oe <= 1'b1;
                if (tick_p2) begin
                    if (bit_cnt == 3'd0) begin
                        sda_oe <= 1'b0;
                        state  <= ST_ADDR_RD_ACK;
                    end else begin
                        bit_cnt <= bit_cnt - 3'd1;
                    end
                end
            end

            // ----------------------------------------------------------------
            ST_ADDR_RD_ACK: begin
                if (tick_p0) scl_oe <= 1'b0;
                if (tick_p1) begin
                    if (sda_in) state <= ST_ERROR;
                end
                if (tick_p2) begin
                    scl_oe  <= 1'b1;
                    sda_oe  <= 1'b0;   // release SDA for device to drive
                    bit_cnt <= 3'd7;
                    byte_hi <= 1'b1;
                    rx_shift<= 16'd0;
                    state   <= ST_RD_DATA;
                end
            end

            // ----------------------------------------------------------------
            // Read data bit by bit (device drives SDA)
            // ----------------------------------------------------------------
            ST_RD_DATA: begin
                if (tick_p0) scl_oe <= 1'b0;   // SCL rises
                if (tick_p1) begin
                    rx_shift <= {rx_shift[14:0], sda_in};  // sample on SCL high
                end
                if (tick_p2) scl_oe <= 1'b1;
                if (tick_p2) begin
                    if (bit_cnt == 3'd0) begin
                        state <= ST_RD_ACK;
                    end else begin
                        bit_cnt <= bit_cnt - 3'd1;
                    end
                end
            end

            // ----------------------------------------------------------------
            // Master ACK (after high byte) or NACK (after low byte)
            // ----------------------------------------------------------------
            ST_RD_ACK: begin
                if (tick_p3) begin
                    // ACK=SDA low (more data), NACK=SDA high (last byte)
                    sda_oe <= byte_hi;  // byte_hi=1: send ACK; byte_hi=0: send NACK
                end
                if (tick_p0) scl_oe <= 1'b0;
                if (tick_p2) begin
                    scl_oe <= 1'b1;
                    if (byte_hi) begin
                        // Done with high byte, read low byte next
                        sda_oe  <= 1'b0;  // release SDA
                        bit_cnt <= 3'd7;
                        byte_hi <= 1'b0;
                        state   <= ST_RD_DATA;
                    end else begin
                        // Done with both bytes
                        rd_data <= rx_shift;
                        sda_oe  <= 1'b0;
                        state   <= ST_STOP;
                    end
                end
            end

            // ----------------------------------------------------------------
            // STOP: SCL low, SDA low -> SCL high -> SDA high
            // ----------------------------------------------------------------
            ST_STOP: begin
                if (tick_p3) begin
                    sda_oe <= 1'b1;   // SDA low
                    scl_oe <= 1'b1;   // SCL low (ensure)
                end
                if (tick_p0) scl_oe <= 1'b0;   // SCL rises
                if (tick_p1) sda_oe <= 1'b0;   // SDA rises while SCL high = STOP
                if (tick_p2) begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end
            end

            // ----------------------------------------------------------------
            // ERROR: release bus lines, signal error, return to idle
            // ----------------------------------------------------------------
            ST_ERROR: begin
                scl_oe      <= 1'b0;
                sda_oe      <= 1'b0;
                busy        <= 1'b0;
                err         <= 1'b1;
                timeout_cnt <= 20'd0;
                state       <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
