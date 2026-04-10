// ============================================================
// bms_step1_tb.v — Testbench for BMS Step 1
//
// Provides:
//   • 50 MHz clock & reset
//   • Simple INA226 I2C slave model (addr 0x40)
//   • UART RX capture → $write to stdout
//   • Measurement triggers via hierarchical force
// ============================================================
`timescale 1ns/1ps
`include "defines.v"

module bms_step1_tb;

    // =========================================================
    // Clock & Reset
    // =========================================================
    reg clk;
    reg rst_n;

    initial clk = 1'b0;
    always  #10 clk = ~clk;    // 50 MHz  (period = 20 ns)

    // =========================================================
    // DUT
    // =========================================================
    wire scl_wire;
    wire sda_wire;
    wire uart_tx_wire;
    wire led_ready;
    wire led_error;

    // Open-drain pull-ups
    pullup (scl_wire);
    pullup (sda_wire);

    bms_top dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .scl         (scl_wire),
        .sda         (sda_wire),
        .uart_tx_pin (uart_tx_wire),
        .led_ready   (led_ready),
        .led_error   (led_error)
    );

    // =========================================================
    // INA226 I2C slave model (address 0x40)
    // Fixed register values:
    //   0x00 CONFIG  = 0x4527
    //   0x01 SHUNT_V = 0x0640  → 1600 × 2.5 µV = 4.0 mV → 40 mA
    //   0x02 BUS_V   = 0x30D4  → bits[15:3]=0x61A=1562 × 1.25 mV = 12500 mV
    //   0x03 POWER   = 0x001E  → 30 × 25 mW = 750 mW
    //   0x04 CURRENT = 0x0028  → 40 mA  (1 mA/LSB, CAL=512)
    //   0x05 CALIB   = 0x0200
    // =========================================================
    reg [15:0] slave_regs [0:5];
    initial begin
        slave_regs[0] = 16'h4527;
        slave_regs[1] = 16'h0640;
        slave_regs[2] = 16'h2710;   // BUS_V: 10000 × 1.25 mV = 12500 mV (12.5 V)
        slave_regs[3] = 16'h001E;
        slave_regs[4] = 16'h0028;
        slave_regs[5] = 16'h0200;
    end

    // Slave SDA driver
    reg slave_sda_oe;
    reg slave_sda_val;
    assign sda_wire = slave_sda_oe ? slave_sda_val : 1'bz;

    // I2C event detection (registered, 1-cycle latency)
    reg scl_d, sda_d;
    always @(posedge clk) begin scl_d <= scl_wire; sda_d <= sda_wire; end

    wire scl_rise = (!scl_d) && scl_wire;
    wire scl_fall = ( scl_d) && (!scl_wire);
    // START: SDA fell in a cycle where SCL was high the previous cycle.
    // Using scl_d (not scl_wire) handles simultaneous SCL+SDA fall from master.
    wire start_cond = scl_d && (sda_d) && (!sda_wire);
    wire stop_cond  = scl_wire && (!sda_d) && (sda_wire);   // SDA↑ while SCL hi

    // Slave state machine
    localparam [3:0]
        SS_IDLE      = 4'd0,
        SS_ADDR      = 4'd1,
        SS_ADDR_ACK  = 4'd2,
        SS_REG       = 4'd3,
        SS_REG_ACK   = 4'd4,
        SS_WR_H      = 4'd5,
        SS_WR_H_ACK  = 4'd6,
        SS_WR_L      = 4'd7,
        SS_WR_L_ACK  = 4'd8,
        SS_TX_H      = 4'd9,
        SS_TX_H_ACK  = 4'd10,
        SS_TX_L      = 4'd11,
        SS_TX_L_ACK  = 4'd12;

    reg [3:0]  ss;
    reg [3:0]  bit_idx;
    reg [7:0]  rx_byte;
    reg [7:0]  reg_ptr;
    reg [7:0]  wr_high;
    reg [15:0] tx_word;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ss             <= SS_IDLE;
            slave_sda_oe   <= 1'b0;
            slave_sda_val  <= 1'b1;
            bit_idx        <= 4'd7;
            rx_byte        <= 8'd0;
            reg_ptr        <= 8'd0;
        end else if (start_cond) begin
            ss           <= SS_ADDR;
            bit_idx      <= 4'd7;
            rx_byte      <= 8'd0;
            slave_sda_oe <= 1'b0;
        end else if (stop_cond) begin
            ss           <= SS_IDLE;
            slave_sda_oe <= 1'b0;
        end else begin
            case (ss)
                SS_IDLE: ;

                // ---- Receive 8-bit address byte -------------
                SS_ADDR: begin
                    if (scl_rise) begin
                        rx_byte <= {rx_byte[6:0], sda_wire};
                        if (bit_idx == 4'd0)
                            ss <= SS_ADDR_ACK;
                        else
                            bit_idx <= bit_idx - 4'd1;
                    end
                end
                SS_ADDR_ACK: begin
                    // Master samples SDA 62 cycles after SCL rises.
                    // Drive ACK on scl_rise, release on the next scl_fall.
                    // slave_sda_oe=1 is used as "ACK has been driven" flag.
                    if (scl_rise) begin
                        if (rx_byte[7:1] == 7'h40) begin
                            slave_sda_oe  <= 1'b1;
                            slave_sda_val <= 1'b0; // ACK
                        end else
                            ss <= SS_IDLE;  // address mismatch → NACK
                    end else if (scl_fall && slave_sda_oe) begin
                        slave_sda_oe <= 1'b0; // release after ACK sampled
                        if (rx_byte[0]) begin  // Read
                            tx_word <= (reg_ptr < 8'd6) ?
                                        slave_regs[reg_ptr] : 16'hDEAD;
                            bit_idx <= 4'd15;
                            ss      <= SS_TX_H;
                        end else begin         // Write
                            bit_idx <= 4'd7;
                            rx_byte <= 8'd0;
                            ss      <= SS_REG;
                        end
                    end
                end

                // ---- Receive register pointer ---------------
                SS_REG: begin
                    if (scl_rise)
                        rx_byte <= {rx_byte[6:0], sda_wire};
                    if (scl_fall && bit_idx == 4'd0) begin
                        reg_ptr <= rx_byte;
                        ss      <= SS_REG_ACK;
                    end else if (scl_fall && bit_idx != 4'd0)
                        bit_idx <= bit_idx - 4'd1;
                end
                SS_REG_ACK: begin
                    if (scl_rise) begin
                        slave_sda_oe  <= 1'b1;
                        slave_sda_val <= 1'b0; // ACK
                    end else if (scl_fall && slave_sda_oe) begin
                        slave_sda_oe <= 1'b0;
                        bit_idx <= 4'd7;
                        rx_byte <= 8'd0;
                        ss      <= SS_WR_H;
                    end
                end

                // ---- Receive write high byte ----------------
                SS_WR_H: begin
                    if (scl_rise)
                        rx_byte <= {rx_byte[6:0], sda_wire};
                    if (scl_fall && bit_idx == 4'd0) begin
                        wr_high <= rx_byte;
                        ss      <= SS_WR_H_ACK;
                    end else if (scl_fall && bit_idx != 4'd0)
                        bit_idx <= bit_idx - 4'd1;
                end
                SS_WR_H_ACK: begin
                    if (scl_rise) begin
                        slave_sda_oe  <= 1'b1;
                        slave_sda_val <= 1'b0; // ACK
                    end else if (scl_fall && slave_sda_oe) begin
                        slave_sda_oe <= 1'b0;
                        bit_idx <= 4'd7;
                        rx_byte <= 8'd0;
                        ss      <= SS_WR_L;
                    end
                end

                // ---- Receive write low byte -----------------
                SS_WR_L: begin
                    if (scl_rise)
                        rx_byte <= {rx_byte[6:0], sda_wire};
                    if (scl_fall && bit_idx == 4'd0) begin
                        if (reg_ptr < 8'd6)
                            slave_regs[reg_ptr] <= {wr_high, rx_byte};
                        ss <= SS_WR_L_ACK;
                    end else if (scl_fall && bit_idx != 4'd0)
                        bit_idx <= bit_idx - 4'd1;
                end
                SS_WR_L_ACK: begin
                    if (scl_rise) begin
                        slave_sda_oe  <= 1'b1;
                        slave_sda_val <= 1'b0; // ACK
                    end else if (scl_fall && slave_sda_oe) begin
                        slave_sda_oe <= 1'b0;
                        ss <= SS_IDLE; // master sends STOP
                    end
                end

                // ---- Transmit high byte (bits 15..8) --------
                SS_TX_H: begin
                    if (scl_fall) begin
                        slave_sda_oe  <= 1'b1;
                        slave_sda_val <= tx_word[15];
                        tx_word       <= {tx_word[14:0], 1'b0};
                        if (bit_idx == 4'd8) begin
                            bit_idx <= 4'd7;
                            ss      <= SS_TX_H_ACK;
                        end else
                            bit_idx <= bit_idx - 4'd1;
                    end
                end
                SS_TX_H_ACK: begin
                    if (scl_fall) begin
                        slave_sda_oe <= 1'b0; // release for master ACK
                        ss <= SS_TX_L;
                    end
                end

                // ---- Transmit low byte (bits 7..0) ----------
                SS_TX_L: begin
                    if (scl_fall) begin
                        slave_sda_oe  <= 1'b1;
                        slave_sda_val <= tx_word[15];
                        tx_word       <= {tx_word[14:0], 1'b0};
                        if (bit_idx == 4'd0) begin
                            ss <= SS_TX_L_ACK;
                        end else
                            bit_idx <= bit_idx - 4'd1;
                    end
                end
                SS_TX_L_ACK: begin
                    if (scl_fall) begin
                        slave_sda_oe <= 1'b0; // release for NACK/STOP
                        ss <= SS_IDLE;
                    end
                end

                default: ss <= SS_IDLE;
            endcase
        end
    end

    // =========================================================
    // UART RX capture
    //
    // Standard 8-N-1 reception:
    //   1. Detect falling edge of TX (start bit)
    //   2. Wait 1.5 baud periods to reach centre of D0
    //   3. Sample 8 bits, one per baud period
    //   4. Print the reconstructed byte
    //
    // Baud period = 434 clock cycles (50 MHz / 115200)
    // 1.5 × 434 - 1 (detection latency) = 650 cycles initial wait
    // =========================================================
    localparam integer BAUD_15  = 650;   // 1.5 baud periods − 1
    localparam integer BAUD_FULL = 434;  // 1 baud period

    reg        uart_busy;
    integer    uart_cnt;
    reg [2:0]  uart_bit;
    reg [7:0]  uart_byte;
    reg        tx_prev;

    initial begin
        uart_busy = 0;
        uart_cnt  = 0;
        uart_bit  = 0;
        uart_byte = 0;
        tx_prev   = 1;
    end

    always @(posedge clk) begin
        if (!uart_busy) begin
            // Detect falling edge (start bit)
            if (tx_prev && !uart_tx_wire) begin
                uart_busy <= 1;
                uart_cnt  <= BAUD_15;
                uart_bit  <= 3'd0;
                uart_byte <= 8'd0;
            end
            tx_prev <= uart_tx_wire;
        end else begin
            if (uart_cnt == 0) begin
                if (uart_bit < 3'd7) begin
                    uart_byte <= {uart_tx_wire, uart_byte[7:1]}; // LSB first
                    uart_bit  <= uart_bit + 3'd1;
                    uart_cnt  <= BAUD_FULL - 1;
                end else begin
                    // Last (8th) bit
                    uart_byte <= {uart_tx_wire, uart_byte[7:1]};
                    // Print on next cycle via separate always
                    uart_busy <= 1'b0;
                    tx_prev   <= uart_tx_wire;
                end
            end else begin
                uart_cnt <= uart_cnt - 1;
            end
        end
    end

    // Print the byte when uart_busy falls
    reg uart_busy_d;
    always @(posedge clk) begin
        uart_busy_d <= uart_busy;
        if (uart_busy_d && !uart_busy)
            $write("%c", uart_byte);
    end

    // =========================================================
    // Simulation control
    //
    // Timeline (all times from reset release):
    //   0         : Header starts transmitting (~4.86 ms = 242,880 cyc)
    //   300,000   : Header done. Force trigger #1 → init+read (≈22k cyc)
    //   350,000   : conv_valid = 1. Force trigger #2 → data snap + read
    //   550,000   : UART drains data line #1. Force trigger #3
    //   750,000   : UART drains data line #2. Force trigger #4
    //   1,000,000 : UART drains data line #3. Done.
    // =========================================================
    task force_trigger;
        begin
            force dut.trigger = 1'b1;
            @(posedge clk);
            force dut.trigger = 1'b0;
            @(posedge clk);
            release dut.trigger;
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        $display("[TB] Reset released at t=%0t", $time);

        // Wait for header to finish transmitting
        // Header = 56 bytes × 10 bits × 434 cyc = 242,840 cyc
        repeat (300_000) @(posedge clk);

        // Trigger 1: initialise INA226 + first read cycle
        $display("[TB] Trigger 1 (init)");
        force_trigger;

        // Wait for init + all 4 register reads to complete
        // I2C at 400 kHz from 50 MHz = 125 cyc/bit. Full init+read ≈ 36k cycles.
        repeat (100_000) @(posedge clk);

        // Trigger 2: conv_valid = 1, formatter snapshots data
        $display("[TB] Trigger 2 (data)");
        force_trigger;

        // Wait for data line 1 to drain (33 bytes × 4340 cyc ≈ 143,220 cyc)
        repeat (200_000) @(posedge clk);

        // Trigger 3: second data snapshot
        $display("[TB] Trigger 3 (data)");
        force_trigger;

        repeat (200_000) @(posedge clk);

        // Trigger 4: third data snapshot
        $display("[TB] Trigger 4 (data)");
        force_trigger;

        repeat (200_000) @(posedge clk);

        $display("\n[TB] Simulation complete at t=%0t", $time);
        $finish;
    end

    // =========================================================
    // VCD waveform dump
    // =========================================================
    initial begin
        $dumpfile("bms_step1.vcd");
        $dumpvars(0, bms_step1_tb);
    end

    // =========================================================
    // Watchdog (200 ms sim time = 10 M cycles)
    // =========================================================
    initial begin
        #(10_000_000 * 20);
        $display("[TB] WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
