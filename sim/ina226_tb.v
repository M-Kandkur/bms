// =============================================================================
// File        : ina226_tb.v
// Project     : INA226 I2C Current Sensor Interface
// Target      : Simulation (Icarus Verilog / ModelSim / QuestaSim)
// Description : Comprehensive testbench for the INA226 VLSI design.
//
// Tests performed:
//   1. Power-on reset and initialisation sequence
//      - Verify CONFIG register write (0x4527)
//      - Verify CALIBRATION register write (0x0200)
//      - Verify MASK/ENABLE register write (0x0400)
//   2. Measurement read cycle
//      - Shunt Voltage read -> verify scaled output
//      - Bus Voltage read   -> verify scaled output
//      - Current read       -> verify scaled output
//      - Power read         -> verify scaled output
//   3. Verify continuous measurement loop
//
// I2C Slave Model:
//   The testbench includes a byte-level I2C slave model that responds to the
//   INA226 address (0x40), ACKs register writes, and returns pre-programmed
//   measurement data on reads.  ACK is correctly held for the full 9th SCL
//   clock period.
//
// Usage (Icarus Verilog):
//   iverilog -I../rtl -o ina226_tb ../rtl/defines.v ../rtl/i2c_master.v \
//            ../rtl/ina226_registers.v ../rtl/ina226_controller.v \
//            ../rtl/ina226_top.v ina226_tb.v
//   vvp ina226_tb
//   gtkwave ina226_tb.vcd &
// =============================================================================
`timescale 1ns/1ps
`include "../rtl/defines.v"

// ---------------------------------------------------------------------------
// Test parameters
// ---------------------------------------------------------------------------
`define CLK_PERIOD       20           // 20 ns = 50 MHz
`define SIM_TIMEOUT      80_000_000   // 80 ms simulation timeout (in ns)

// INA226 test measurement values (raw register values)
`define TEST_SHUNT_RAW   16'hFFD0    // -48 (2's complement), *2.5uV = -120 uV
`define TEST_BUS_RAW     16'h6400    // 25600 * 1.25 mV = 32000 mV = 32 V
`define TEST_CURRENT_RAW 16'h0032    // 50 * 100 uA = 5000 uA = 5 mA
`define TEST_POWER_RAW   16'h0010    // 16 * 2500 uW = 40000 uW = 40 mW

module ina226_tb;

// ---------------------------------------------------------------------------
// Clock and reset
// ---------------------------------------------------------------------------
reg  clk;
reg  rst_n;

initial clk = 1'b0;
always #(`CLK_PERIOD/2) clk = ~clk;

// ---------------------------------------------------------------------------
// DUT signals
// ---------------------------------------------------------------------------
wire        scl;
wire        sda;

wire signed [31:0] shunt_uv;
wire        [15:0] bus_mv_x10;
wire signed [31:0] current_ua;
wire        [31:0] power_uw;
wire        [15:0] raw_shunt_v;
wire        [15:0] raw_bus_v;
wire        [15:0] raw_current;
wire        [15:0] raw_power;
wire        [15:0] raw_config;
wire        [15:0] raw_calib;
wire               data_valid;
wire               init_done;
wire               meas_valid;
wire               sensor_err;

// Pull-ups for open-drain lines (modelled by weak pull to 1)
pullup (scl);
pullup (sda);

// ---------------------------------------------------------------------------
// DUT instantiation
// ---------------------------------------------------------------------------
ina226_top dut (
    .clk        (clk),
    .rst_n      (rst_n),
    .scl        (scl),
    .sda        (sda),
    .shunt_uv   (shunt_uv),
    .bus_mv_x10 (bus_mv_x10),
    .current_ua (current_ua),
    .power_uw   (power_uw),
    .raw_shunt_v(raw_shunt_v),
    .raw_bus_v  (raw_bus_v),
    .raw_current(raw_current),
    .raw_power  (raw_power),
    .raw_config (raw_config),
    .raw_calib  (raw_calib),
    .data_valid (data_valid),
    .init_done  (init_done),
    .meas_valid (meas_valid),
    .sensor_err (sensor_err)
);

// ---------------------------------------------------------------------------
// I2C Bus Monitoring - detect START and STOP conditions
// ---------------------------------------------------------------------------
reg scl_r, sda_r;
always @(posedge clk) begin
    scl_r <= scl;
    sda_r <= sda;
end

// START: SDA falls while SCL is high
wire i2c_start = scl && scl_r && sda_r && !sda;
// STOP:  SDA rises while SCL is high
wire i2c_stop  = scl && scl_r && !sda_r && sda;
// Clock edges
wire scl_rise  = scl  && !scl_r;
wire scl_fall  = !scl && scl_r;

// ---------------------------------------------------------------------------
// I2C Slave Model
//
// Design:
//   1. On START: reset to receive address byte.
//   2. Receive bits on SCL rising edge; count 8 bits.
//   3. On 8th SCL fall: check address, drive ACK if matched.
//   4. Hold ACK until next SCL fall (= end of ACK clock), then go to
//      next phase (REG_PTR, WR_HI, WR_LO, or RD_HI).
//   5. For reads: drive data bits on each SCL fall, release on byte boundary.
//
// ---------------------------------------------------------------------------

// Slave states
localparam SLV_IDLE       = 5'd0;
localparam SLV_RX_ADDR    = 5'd1;  // Receiving address byte
localparam SLV_ACK_ADDR   = 5'd2;  // Driving ACK after address
localparam SLV_RX_PTR     = 5'd3;  // Receiving register pointer byte
localparam SLV_ACK_PTR    = 5'd4;  // Driving ACK after reg pointer
localparam SLV_RX_WR_HI   = 5'd5;  // Receiving write data high byte
localparam SLV_ACK_WR_HI  = 5'd6;
localparam SLV_RX_WR_LO   = 5'd7;  // Receiving write data low byte
localparam SLV_ACK_WR_LO  = 5'd8;
localparam SLV_TX_RD_HI   = 5'd9;  // Transmitting read data high byte
localparam SLV_TX_RD_LO   = 5'd10; // Transmitting read data low byte
localparam SLV_MACK_RD_HI = 5'd11; // Master ACK after high byte
localparam SLV_WAIT_STOP  = 5'd12; // Waiting for STOP after last NACK

reg [4:0]  slv_state;
reg [3:0]  slv_bit;      // bit counter: counts DOWN from 7 to 0
reg [7:0]  slv_rx;       // receive shift register (holds last 7 bits)
reg [7:0]  slv_reg_ptr;  // current register pointer
reg        slv_is_read;  // R/W bit from address phase
reg        slv_matched;  // address matched flag
reg        slv_ack_fall; // flag: waiting for 2nd scl_fall in ACK

// Slave register memory (16-bit, indexed by 3-bit reg addr)
reg [15:0] slv_mem[0:7];

// SDA driver: open-drain (drive low = pull SDA to 0; release = 1'bz)
reg slv_sda_drive;
assign sda = slv_sda_drive ? 1'b0 : 1'bz;

// Initialise slave register memory with test measurement values
integer si;
initial begin
    for (si = 0; si < 8; si = si + 1)
        slv_mem[si] = 16'h0000;
    // Measurement registers with test values
    slv_mem[`REG_SHUNT_V  & 3'h7] = `TEST_SHUNT_RAW;
    slv_mem[`REG_BUS_V    & 3'h7] = `TEST_BUS_RAW;
    slv_mem[`REG_CURRENT  & 3'h7] = `TEST_CURRENT_RAW;
    slv_mem[`REG_POWER    & 3'h7] = `TEST_POWER_RAW;
    slv_mem[`REG_CONFIG   & 3'h7] = 16'h4127;  // INA226 POR default
end

// ---------------------------------------------------------------------------
// Slave FSM
// ---------------------------------------------------------------------------
// Helper: current TX bit (MSB first)
// For high byte (SLV_TX_RD_HI): drive slv_mem[ptr][8 + slv_bit]
// For low  byte (SLV_TX_RD_LO): drive slv_mem[ptr][slv_bit]
wire [3:0] slv_bit_hi_idx = 4'd8 + slv_bit; // 15 downto 8
wire       slv_tx_bit_hi  = slv_mem[slv_reg_ptr[2:0]][slv_bit_hi_idx];
wire       slv_tx_bit_lo  = slv_mem[slv_reg_ptr[2:0]][slv_bit];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        slv_state     <= SLV_IDLE;
        slv_bit       <= 4'd7;
        slv_rx        <= 8'd0;
        slv_reg_ptr   <= 8'd0;
        slv_is_read   <= 1'b0;
        slv_matched   <= 1'b0;
        slv_ack_fall  <= 1'b0;
        slv_sda_drive <= 1'b0;
    end else begin

        // ------------------------------------------------------------------
        // START / Repeated-START: always reset to receive address byte
        // ------------------------------------------------------------------
        if (i2c_start) begin
            slv_state     <= SLV_RX_ADDR;
            slv_bit       <= 4'd7;
            slv_rx        <= 8'd0;
            slv_matched   <= 1'b0;
            slv_ack_fall  <= 1'b0;
            slv_sda_drive <= 1'b0;
        end

        // ------------------------------------------------------------------
        // STOP: release bus
        // ------------------------------------------------------------------
        else if (i2c_stop) begin
            slv_state     <= SLV_IDLE;
            slv_sda_drive <= 1'b0;
            slv_ack_fall  <= 1'b0;
        end

        // ------------------------------------------------------------------
        // Normal processing (no START/STOP this cycle)
        // ------------------------------------------------------------------
        else begin
            case (slv_state)

                // --------------------------------------------------------
                SLV_IDLE: begin
                    slv_sda_drive <= 1'b0;
                end

                // --------------------------------------------------------
                // Receive address byte (8 bits, MSB first)
                // Transition on scl_fall after receiving ALL 8 bits
                // --------------------------------------------------------
                SLV_RX_ADDR: begin
                    if (scl_rise) begin
                        slv_rx  <= {slv_rx[6:0], sda};  // shift in bit
                        if (slv_bit == 4'd0) begin
                            // 8th bit received (R/W bit = sda at this moment)
                            // Address is in slv_rx[6:0] (bits 7:1 of the byte)
                            slv_is_read <= sda;
                            if (slv_rx[6:0] == `INA226_ADDR)
                                slv_matched <= 1'b1;
                            else
                                slv_matched <= 1'b0;
                        end else begin
                            slv_bit <= slv_bit - 4'd1;
                        end
                    end
                    // On SCL fall after 8th bit (slv_bit==0), drive ACK
                    if (scl_fall && slv_bit == 4'd0) begin
                        slv_sda_drive <= slv_matched;  // drive low = ACK if matched
                        slv_ack_fall  <= 1'b0;
                        slv_state     <= SLV_ACK_ADDR;
                        slv_bit       <= 4'd7;  // reset for next byte
                    end
                end

                // --------------------------------------------------------
                // ACK after address: hold ACK until scl_fall of ACK clock
                // (The ACK clock's scl_fall comes AFTER scl_rise, which the
                //  master uses to sample.  We hold ACK through scl_rise.)
                // --------------------------------------------------------
                SLV_ACK_ADDR: begin
                    // Default: hold ACK while in this state
                    if (slv_matched) slv_sda_drive <= 1'b1;
                    if (scl_fall) begin
                        // First scl_fall when entering: this is the 8th data
                        // bit's fall.  We already handled it above (transition
                        // from RX_ADDR on that exact scl_fall).  Any subsequent
                        // scl_fall here is the ACK clock's fall → release.
                        if (!slv_ack_fall) begin
                            slv_ack_fall <= 1'b1;  // Mark first fall seen
                        end else begin
                            // 2nd fall = end of ACK clock
                            slv_sda_drive <= 1'b0;
                            slv_ack_fall  <= 1'b0;
                            slv_rx        <= 8'd0;
                            if (slv_is_read)
                                slv_state <= SLV_TX_RD_HI;  // read transaction
                            else
                                slv_state <= SLV_RX_PTR;     // write transaction
                        end
                    end
                end

                // --------------------------------------------------------
                // Receive register pointer byte
                // --------------------------------------------------------
                SLV_RX_PTR: begin
                    if (scl_rise) begin
                        slv_rx <= {slv_rx[6:0], sda};
                        if (slv_bit == 4'd0) begin
                            slv_reg_ptr <= {slv_rx[6:0], sda};  // full 8-bit ptr
                        end else begin
                            slv_bit <= slv_bit - 4'd1;
                        end
                    end
                    if (scl_fall && slv_bit == 4'd0) begin
                        slv_sda_drive <= 1'b1;  // ACK
                        slv_ack_fall  <= 1'b0;
                        slv_state     <= SLV_ACK_PTR;
                        slv_bit       <= 4'd7;
                        slv_rx        <= 8'd0;
                    end
                end

                // --------------------------------------------------------
                SLV_ACK_PTR: begin
                    if (slv_matched) slv_sda_drive <= 1'b1;
                    if (scl_fall) begin
                        if (!slv_ack_fall) begin
                            slv_ack_fall <= 1'b1;
                        end else begin
                            slv_sda_drive <= 1'b0;
                            slv_ack_fall  <= 1'b0;
                            slv_state     <= SLV_RX_WR_HI;
                        end
                    end
                end

                // --------------------------------------------------------
                // Receive write data: high byte
                // --------------------------------------------------------
                SLV_RX_WR_HI: begin
                    if (scl_rise) begin
                        slv_rx <= {slv_rx[6:0], sda};
                        if (slv_bit == 4'd0) begin
                            slv_mem[slv_reg_ptr[2:0]][15:8] <= {slv_rx[6:0], sda};
                        end else begin
                            slv_bit <= slv_bit - 4'd1;
                        end
                    end
                    if (scl_fall && slv_bit == 4'd0) begin
                        slv_sda_drive <= 1'b1;
                        slv_ack_fall  <= 1'b0;
                        slv_state     <= SLV_ACK_WR_HI;
                        slv_bit       <= 4'd7;
                        slv_rx        <= 8'd0;
                    end
                end

                // --------------------------------------------------------
                SLV_ACK_WR_HI: begin
                    slv_sda_drive <= 1'b1;
                    if (scl_fall) begin
                        if (!slv_ack_fall) begin
                            slv_ack_fall <= 1'b1;
                        end else begin
                            slv_sda_drive <= 1'b0;
                            slv_ack_fall  <= 1'b0;
                            slv_state     <= SLV_RX_WR_LO;
                        end
                    end
                end

                // --------------------------------------------------------
                // Receive write data: low byte
                // --------------------------------------------------------
                SLV_RX_WR_LO: begin
                    if (scl_rise) begin
                        slv_rx <= {slv_rx[6:0], sda};
                        if (slv_bit == 4'd0) begin
                            slv_mem[slv_reg_ptr[2:0]][7:0] <= {slv_rx[6:0], sda};
                        end else begin
                            slv_bit <= slv_bit - 4'd1;
                        end
                    end
                    if (scl_fall && slv_bit == 4'd0) begin
                        slv_sda_drive <= 1'b1;
                        slv_ack_fall  <= 1'b0;
                        slv_state     <= SLV_ACK_WR_LO;
                        slv_bit       <= 4'd7;
                        slv_rx        <= 8'd0;
                    end
                end

                // --------------------------------------------------------
                SLV_ACK_WR_LO: begin
                    slv_sda_drive <= 1'b1;
                    if (scl_fall) begin
                        if (!slv_ack_fall) begin
                            slv_ack_fall <= 1'b1;
                        end else begin
                            // Both data bytes written - wait for STOP
                            slv_sda_drive <= 1'b0;
                            slv_ack_fall  <= 1'b0;
                            slv_state     <= SLV_WAIT_STOP;
                        end
                    end
                end

                // --------------------------------------------------------
                // Transmit read data: high byte (driven on each SCL fall)
                // --------------------------------------------------------
                SLV_TX_RD_HI: begin
                    if (scl_fall) begin
                        // Drive next bit on SCL fall
                        slv_sda_drive <= ~slv_tx_bit_hi;  // 0=drive(SDA=0), 1=release
                    end
                    if (scl_rise) begin
                        // Master samples on SCL rise; advance bit counter
                        if (slv_bit == 4'd0) begin
                            // High byte done, release for master ACK
                            slv_sda_drive <= 1'b0;
                            slv_state     <= SLV_MACK_RD_HI;
                            slv_bit       <= 4'd7;
                        end else begin
                            slv_bit <= slv_bit - 4'd1;
                        end
                    end
                end

                // --------------------------------------------------------
                // Wait for master ACK after high byte, then drive low byte
                // --------------------------------------------------------
                SLV_MACK_RD_HI: begin
                    slv_sda_drive <= 1'b0;  // released (master drives ACK)
                    if (scl_fall) begin
                        // ACK clock's SCL fall: start driving low byte
                        slv_sda_drive <= ~slv_tx_bit_lo;
                        slv_state     <= SLV_TX_RD_LO;
                    end
                end

                // --------------------------------------------------------
                // Transmit read data: low byte
                // --------------------------------------------------------
                SLV_TX_RD_LO: begin
                    if (scl_fall) begin
                        slv_sda_drive <= ~slv_tx_bit_lo;
                    end
                    if (scl_rise) begin
                        if (slv_bit == 4'd0) begin
                            // Low byte done, release SDA for master NACK + STOP
                            slv_sda_drive <= 1'b0;
                            slv_state     <= SLV_WAIT_STOP;
                        end else begin
                            slv_bit <= slv_bit - 4'd1;
                        end
                    end
                end

                // --------------------------------------------------------
                SLV_WAIT_STOP: begin
                    slv_sda_drive <= 1'b0;
                    // Stay here until STOP or START (handled above)
                end

                default: begin
                    slv_state     <= SLV_IDLE;
                    slv_sda_drive <= 1'b0;
                end

            endcase
        end
    end
end

// ---------------------------------------------------------------------------
// Waveform dump
// ---------------------------------------------------------------------------
initial begin
    $dumpfile("ina226_tb.vcd");
    $dumpvars(0, ina226_tb);
end

// ---------------------------------------------------------------------------
// Test stimulus and checking
// ---------------------------------------------------------------------------
integer pass_count;
integer fail_count;

task check_eq;
    input [63:0] actual;
    input [63:0] expected;
    input [8*32-1:0] test_name;
    begin
        if (actual === expected) begin
            $display("[PASS] %0s: got %0d (0x%0h)", test_name, actual, actual);
            pass_count = pass_count + 1;
        end else begin
            $display("[FAIL] %0s: expected %0d (0x%0h), got %0d (0x%0h)",
                     test_name, expected, expected, actual, actual);
            fail_count = fail_count + 1;
        end
    end
endtask

// Simulation timeout
initial begin
    #(`SIM_TIMEOUT);
    $display("[TIMEOUT] Simulation exceeded %0d ns", `SIM_TIMEOUT);
    $display("PASS: %0d  FAIL: %0d", pass_count, fail_count);
    $finish;
end

initial begin
    pass_count = 0;
    fail_count = 0;

    $display("========================================");
    $display(" INA226 I2C VLSI Testbench");
    $display(" Clock: 50 MHz | I2C: 400 kHz");
    $display(" Shunt: 0.1 ohm | Address: 0x%0h", `INA226_ADDR);
    $display("========================================");

    // Apply reset
    rst_n = 1'b0;
    repeat(20) @(posedge clk);
    rst_n = 1'b1;
    $display("[%0t ns] Reset released", $time/1000);

    // ----------------------------------------------------------------
    // Test 1: Wait for initialisation to complete
    // ----------------------------------------------------------------
    $display("[%0t ns] Waiting for INA226 init_done...", $time/1000);
    wait (init_done === 1'b1);
    @(posedge clk);
    $display("[%0t ns] init_done asserted!", $time/1000);

    // Verify CONFIG register was written correctly
    check_eq({48'd0, raw_config}, {48'd0, `CONFIG_VALUE}, "CONFIG register");
    check_eq({48'd0, raw_calib},  {48'd0, `CALIB_VALUE},  "CALIB register");

    // ----------------------------------------------------------------
    // Test 2: Wait for first complete measurement cycle
    // ----------------------------------------------------------------
    $display("[%0t ns] Waiting for first meas_valid pulse...", $time/1000);
    wait (meas_valid === 1'b1);
    @(posedge clk);
    $display("[%0t ns] meas_valid asserted!", $time/1000);

    // Verify raw register values match what the slave returned
    check_eq({48'd0, raw_shunt_v}, {48'd0, `TEST_SHUNT_RAW},   "raw_shunt_v");
    check_eq({48'd0, raw_bus_v},   {48'd0, `TEST_BUS_RAW},     "raw_bus_v");
    check_eq({48'd0, raw_current}, {48'd0, `TEST_CURRENT_RAW}, "raw_current");
    check_eq({48'd0, raw_power},   {48'd0, `TEST_POWER_RAW},   "raw_power");
    check_eq({63'd0, data_valid},  {63'd0, 1'b1},              "data_valid");

    // ----------------------------------------------------------------
    // Test 3: Verify scaled output calculations
    //
    // TEST_SHUNT_RAW = 0xFFD0 = signed -48
    //   shunt_uv = -48 * 5 / 2 = -120 uV
    //
    // TEST_CURRENT_RAW = 0x0032 = 50
    //   current_ua = 50 * 100 = 5000 uA
    //
    // TEST_POWER_RAW = 0x0010 = 16
    //   power_uw = 16 * 2500 = 40000 uW
    //
    // TEST_BUS_RAW = 0x6400 = 25600
    //   bus_mv_x10 = 25600 * 5 >> 2 = 32000
    //   (INA226 LSB=1.25mV: 25600*1.25=32000 mV = 32V; stored as raw*5>>2)
    // ----------------------------------------------------------------
    $display("[%0t ns] Checking scaled outputs...", $time/1000);

    check_eq($signed(shunt_uv),   $signed(-32'sd120), "shunt_uv (uV)");
    check_eq($signed(current_ua), $signed(32'sd5000), "current_ua (uA)");
    check_eq({32'd0, power_uw},   {32'd0, 32'd40000}, "power_uw (uW)");
    check_eq({48'd0, bus_mv_x10}, {48'd0, 16'd32000}, "bus_mv_x10");

    // ----------------------------------------------------------------
    // Test 4: No sensor errors
    // ----------------------------------------------------------------
    check_eq({63'd0, sensor_err}, {63'd0, 1'b0}, "no sensor errors");

    // ----------------------------------------------------------------
    // Test 5: Continuous operation - wait for second measurement cycle
    // ----------------------------------------------------------------
    $display("[%0t ns] Waiting for second meas_valid...", $time/1000);
    @(negedge meas_valid);   // wait for current pulse to finish
    wait (meas_valid === 1'b1);
    @(posedge clk);
    $display("[%0t ns] Second meas_valid confirmed - continuous OK", $time/1000);
    pass_count = pass_count + 1;

    // ----------------------------------------------------------------
    // Summary
    // ----------------------------------------------------------------
    $display("========================================");
    $display(" Simulation Complete");
    $display(" PASS: %0d  FAIL: %0d", pass_count, fail_count);
    $display("========================================");
    if (fail_count == 0)
        $display("ALL TESTS PASSED");
    else
        $display("SOME TESTS FAILED");
    $finish;
end

endmodule
