// ============================================================
// bms_top.v — Top-level BMS Step 1
//
// Hierarchy:
//   bms_top
//     ├── i2c_master
//     ├── ina226_controller
//     ├── ina226_registers
//     ├── data_converter
//     ├── uart_baud_gen
//     └── uart_tx
//
// Every 1 second the formatter FSM serialises a line:
//   "TTTTTTTT,VVVVV,[-]IIIII,PPPPPP,ST\r\n"  (all numeric fields in hex)
// plus a header line on startup.
// ============================================================
`include "defines.v"
module bms_top (
    input  wire clk,        // 50 MHz
    input  wire rst_n,      // active-low reset (KEY0 on DE1-SoC)

    // I2C (open-drain – connect 4.7 kΩ pull-ups to 3.3 V)
    inout  wire scl,
    inout  wire sda,

    // UART TX (GPIO or UART header on DE1-SoC)
    output wire uart_tx_pin,

    // Status LEDs (optional)
    output wire led_ready,
    output wire led_error
);

    // =========================================================
    // 1-second trigger counter
    // =========================================================
    reg [25:0] sec_cnt;
    reg        trigger;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sec_cnt <= 26'd0;
            trigger <= 1'b0;
        end else if (sec_cnt == `MEAS_INTERVAL - 1) begin
            sec_cnt <= 26'd0;
            trigger <= 1'b1;
        end else begin
            sec_cnt <= sec_cnt + 1'b1;
            trigger <= 1'b0;
        end
    end

    // =========================================================
    // Timestamp counter (ms, wraps ~49 days)
    // =========================================================
    reg [31:0] timestamp_ms;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            timestamp_ms <= 32'd0;
        else if (trigger)
            timestamp_ms <= timestamp_ms + 32'd1000;
    end

    // =========================================================
    // I2C open-drain tri-state
    // =========================================================
    wire scl_oe, sda_oe, sda_in_w;
    assign scl      = scl_oe ? 1'b0 : 1'bz;
    assign sda      = sda_oe ? 1'b0 : 1'bz;
    assign sda_in_w = sda;

    // =========================================================
    // I2C master
    // =========================================================
    wire        i2c_start_w, i2c_rw_w;
    wire [6:0]  i2c_addr_w;
    wire [7:0]  i2c_reg_w;
    wire [15:0] i2c_wdata_w, i2c_rdata_w;
    wire        i2c_done_w, i2c_ack_error_w;

    i2c_master u_i2c (
        .clk       (clk),
        .rst_n     (rst_n),
        .cmd_start (i2c_start_w),
        .cmd_rw    (i2c_rw_w),
        .cmd_addr  (i2c_addr_w),
        .cmd_reg   (i2c_reg_w),
        .cmd_wdata (i2c_wdata_w),
        .rdata     (i2c_rdata_w),
        .done      (i2c_done_w),
        .ack_error (i2c_ack_error_w),
        .scl_oe    (scl_oe),
        .sda_oe    (sda_oe),
        .sda_in    (sda_in_w)
    );

    // =========================================================
    // INA226 controller (FSM)
    // =========================================================
    wire        reg_wr_en_w;
    wire [2:0]  reg_wr_sel_w;
    wire [15:0] reg_wr_data_w;
    wire        error_flag_w, initialised_w;

    ina226_controller u_ctrl (
        .clk          (clk),
        .rst_n        (rst_n),
        .trigger      (trigger),
        .i2c_start    (i2c_start_w),
        .i2c_rw       (i2c_rw_w),
        .i2c_addr     (i2c_addr_w),
        .i2c_reg      (i2c_reg_w),
        .i2c_wdata    (i2c_wdata_w),
        .i2c_rdata    (i2c_rdata_w),
        .i2c_done     (i2c_done_w),
        .i2c_ack_error(i2c_ack_error_w),
        .reg_wr_en    (reg_wr_en_w),
        .reg_wr_sel   (reg_wr_sel_w),
        .reg_wr_data  (reg_wr_data_w),
        .error_flag   (error_flag_w),
        .initialised  (initialised_w)
    );

    // =========================================================
    // Register file
    // =========================================================
    wire [15:0] shunt_raw_w, bus_raw_w, power_raw_w, current_raw_w;
    wire        data_valid_w;

    ina226_registers u_regs (
        .clk              (clk),
        .rst_n            (rst_n),
        .wr_en            (reg_wr_en_w),
        .wr_sel           (reg_wr_sel_w),
        .wr_data          (reg_wr_data_w),
        .shunt_voltage_raw(shunt_raw_w),
        .bus_voltage_raw  (bus_raw_w),
        .power_raw        (power_raw_w),
        .current_raw      (current_raw_w),
        .data_valid       (data_valid_w)
    );

    // =========================================================
    // Data converter
    // =========================================================
    wire [31:0] voltage_mv_w, shunt_uv_w, current_ma_w, power_mw_w;
    wire        shunt_neg_w, current_neg_w, conv_valid_w;

    data_converter u_conv (
        .clk              (clk),
        .rst_n            (rst_n),
        .shunt_voltage_raw(shunt_raw_w),
        .bus_voltage_raw  (bus_raw_w),
        .power_raw        (power_raw_w),
        .current_raw      (current_raw_w),
        .data_valid       (data_valid_w),
        .voltage_mv       (voltage_mv_w),
        .shunt_uv         (shunt_uv_w),
        .shunt_neg        (shunt_neg_w),
        .current_ma       (current_ma_w),
        .current_neg      (current_neg_w),
        .power_mw         (power_mw_w),
        .conv_valid       (conv_valid_w)
    );

    // =========================================================
    // UART baud generator + TX
    // =========================================================
    wire baud_tick_w;
    uart_baud_gen u_baud (
        .clk   (clk),
        .rst_n (rst_n),
        .tick  (baud_tick_w)
    );

    wire       tx_busy_w;
    reg  [7:0] tx_data_r;
    reg        tx_valid_r;

    uart_tx u_uart (
        .clk        (clk),
        .rst_n      (rst_n),
        .baud_tick  (baud_tick_w),
        .data_in    (tx_data_r),
        .data_valid (tx_valid_r),
        .tx         (uart_tx_pin),
        .busy       (tx_busy_w)
    );

    // =========================================================
    // Snapshot registers — latched when trigger fires with data
    // =========================================================
    reg [31:0] snap_ts;
    reg [31:0] snap_v;
    reg [31:0] snap_i;
    reg        snap_i_neg;
    reg [31:0] snap_p;
    reg        snap_err;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snap_ts    <= 32'd0;
            snap_v     <= 32'd0;
            snap_i     <= 32'd0;
            snap_i_neg <= 1'b0;
            snap_p     <= 32'd0;
            snap_err   <= 1'b0;
        end else if (trigger && conv_valid_w) begin
            snap_ts    <= timestamp_ms;
            snap_v     <= voltage_mv_w;
            snap_i     <= current_ma_w;
            snap_i_neg <= current_neg_w;
            snap_p     <= power_mw_w;
            snap_err   <= error_flag_w;
        end
    end

    // =========================================================
    // UART formatter FSM
    //
    // Output format (all numeric fields hexadecimal):
    //   Header:  "Timestamp(ms),Voltage(mV),Current(mA),Power(mW),Status\r\n"
    //   Data:    "TTTTTTTT,VVVVV,[-]IIIII,PPPPPP,ST\r\n"
    //     T = timestamp ms  (8 hex chars)
    //     V = voltage mV    (5 hex chars)
    //     I = |current| mA  (5 hex chars, prefixed +/-)
    //     P = power mW      (6 hex chars)
    //     S = "OK" or "ERR"
    // =========================================================

    // 64-byte transmit buffer
    reg [7:0] tx_buf [0:63];
    reg [5:0] tx_buf_len;
    reg [5:0] tx_buf_idx;

    localparam [1:0]
        FMT_IDLE = 2'd0,
        FMT_SNAP = 2'd1,
        FMT_SEND = 2'd2;

    reg [1:0] fmt_state;
    reg       header_sent;

    // ---- hex nibble helper ----------------------------------
    function [7:0] hex_nibble;
        input [3:0] n;
        hex_nibble = (n < 4'd10) ? (8'h30 + {4'd0, n})
                                 : (8'h37 + {4'd0, n}); // 0x37 + n gives 'A'-'F'
    endfunction
    // Note: 'A' = 0x41 = 0x37 + 10, 'F' = 0x46 = 0x37 + 15  ✓

    // ---- formatter FSM --------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fmt_state   <= FMT_IDLE;
            header_sent <= 1'b0;
            tx_buf_len  <= 6'd0;
            tx_buf_idx  <= 6'd0;
            tx_valid_r  <= 1'b0;
            tx_data_r   <= 8'd0;
        end else begin
            tx_valid_r <= 1'b0;   // default: no TX request this cycle

            case (fmt_state)

                // --------------------------------------------------
                FMT_IDLE: begin
                    if (!header_sent) begin
                        // Load 56-byte header
                        // "Timestamp(ms),Voltage(mV),Current(mA),Power(mW),Status\r\n"
                        tx_buf[0]  <= "T"; tx_buf[1]  <= "i"; tx_buf[2]  <= "m";
                        tx_buf[3]  <= "e"; tx_buf[4]  <= "s"; tx_buf[5]  <= "t";
                        tx_buf[6]  <= "a"; tx_buf[7]  <= "m"; tx_buf[8]  <= "p";
                        tx_buf[9]  <= "("; tx_buf[10] <= "m"; tx_buf[11] <= "s";
                        tx_buf[12] <= ")"; tx_buf[13] <= ","; tx_buf[14] <= "V";
                        tx_buf[15] <= "o"; tx_buf[16] <= "l"; tx_buf[17] <= "t";
                        tx_buf[18] <= "a"; tx_buf[19] <= "g"; tx_buf[20] <= "e";
                        tx_buf[21] <= "("; tx_buf[22] <= "m"; tx_buf[23] <= "V";
                        tx_buf[24] <= ")"; tx_buf[25] <= ","; tx_buf[26] <= "C";
                        tx_buf[27] <= "u"; tx_buf[28] <= "r"; tx_buf[29] <= "r";
                        tx_buf[30] <= "e"; tx_buf[31] <= "n"; tx_buf[32] <= "t";
                        tx_buf[33] <= "("; tx_buf[34] <= "m"; tx_buf[35] <= "A";
                        tx_buf[36] <= ")"; tx_buf[37] <= ","; tx_buf[38] <= "P";
                        tx_buf[39] <= "o"; tx_buf[40] <= "w"; tx_buf[41] <= "e";
                        tx_buf[42] <= "r"; tx_buf[43] <= "("; tx_buf[44] <= "m";
                        tx_buf[45] <= "W"; tx_buf[46] <= ")"; tx_buf[47] <= ",";
                        tx_buf[48] <= "S"; tx_buf[49] <= "t"; tx_buf[50] <= "a";
                        tx_buf[51] <= "t"; tx_buf[52] <= "u"; tx_buf[53] <= "s";
                        tx_buf[54] <= "\r"; tx_buf[55] <= "\n";
                        tx_buf_len  <= 6'd56;
                        tx_buf_idx  <= 6'd0;
                        header_sent <= 1'b1;
                        fmt_state   <= FMT_SEND;
                    end else if (trigger && conv_valid_w) begin
                        fmt_state <= FMT_SNAP;
                    end
                end

                // --------------------------------------------------
                // Build data line using snapshotted values
                // --------------------------------------------------
                FMT_SNAP: begin
                    // Timestamp: 8 hex digits
                    tx_buf[0]  <= hex_nibble(snap_ts[31:28]);
                    tx_buf[1]  <= hex_nibble(snap_ts[27:24]);
                    tx_buf[2]  <= hex_nibble(snap_ts[23:20]);
                    tx_buf[3]  <= hex_nibble(snap_ts[19:16]);
                    tx_buf[4]  <= hex_nibble(snap_ts[15:12]);
                    tx_buf[5]  <= hex_nibble(snap_ts[11:8]);
                    tx_buf[6]  <= hex_nibble(snap_ts[7:4]);
                    tx_buf[7]  <= hex_nibble(snap_ts[3:0]);
                    tx_buf[8]  <= ",";
                    // Voltage mV: 5 hex digits (max 36000 = 0x8CA0 fits in 17 bits)
                    tx_buf[9]  <= hex_nibble(snap_v[19:16]);
                    tx_buf[10] <= hex_nibble(snap_v[15:12]);
                    tx_buf[11] <= hex_nibble(snap_v[11:8]);
                    tx_buf[12] <= hex_nibble(snap_v[7:4]);
                    tx_buf[13] <= hex_nibble(snap_v[3:0]);
                    tx_buf[14] <= ",";
                    // Current mA: sign + 5 hex digits
                    tx_buf[15] <= snap_i_neg ? "-" : "+";
                    tx_buf[16] <= hex_nibble(snap_i[19:16]);
                    tx_buf[17] <= hex_nibble(snap_i[15:12]);
                    tx_buf[18] <= hex_nibble(snap_i[11:8]);
                    tx_buf[19] <= hex_nibble(snap_i[7:4]);
                    tx_buf[20] <= hex_nibble(snap_i[3:0]);
                    tx_buf[21] <= ",";
                    // Power mW: 6 hex digits
                    tx_buf[22] <= hex_nibble(snap_p[23:20]);
                    tx_buf[23] <= hex_nibble(snap_p[19:16]);
                    tx_buf[24] <= hex_nibble(snap_p[15:12]);
                    tx_buf[25] <= hex_nibble(snap_p[11:8]);
                    tx_buf[26] <= hex_nibble(snap_p[7:4]);
                    tx_buf[27] <= hex_nibble(snap_p[3:0]);
                    tx_buf[28] <= ",";
                    // Status
                    tx_buf[29] <= snap_err ? "E" : "O";
                    tx_buf[30] <= snap_err ? "R" : "K";
                    tx_buf[31] <= "\r";
                    tx_buf[32] <= "\n";
                    tx_buf_len <= 6'd33;
                    tx_buf_idx <= 6'd0;
                    fmt_state  <= FMT_SEND;
                end

                // --------------------------------------------------
                // Drain buffer byte-by-byte via uart_tx
                // --------------------------------------------------
                FMT_SEND: begin
                    if (tx_buf_idx < tx_buf_len) begin
                        // Gate on both busy AND valid: the UART takes one clock
                        // cycle to assert busy after seeing data_valid, so we
                        // must not send a new byte in that gap cycle.
                        if (!tx_busy_w && !tx_valid_r) begin
                            tx_data_r  <= tx_buf[tx_buf_idx];
                            tx_valid_r <= 1'b1;
                            tx_buf_idx <= tx_buf_idx + 6'd1;
                        end
                    end else begin
                        fmt_state <= FMT_IDLE;
                    end
                end

                default: fmt_state <= FMT_IDLE;
            endcase
        end
    end

    // =========================================================
    // Status LEDs
    // =========================================================
    assign led_ready = initialised_w;
    assign led_error = error_flag_w;

endmodule
