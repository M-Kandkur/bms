// ============================================================
// i2c_master.v — I2C master controller (400 kHz)
// Generated from 50 MHz FPGA clock.
// Supports single-byte write and two-byte register read.
// Open-drain: drive 0 to pull low, release (1) to float high.
// ============================================================
`include "defines.v"
module i2c_master (
    input  wire       clk,
    input  wire       rst_n,

    // Command interface
    input  wire       cmd_start,      // pulse: start a transaction
    input  wire       cmd_rw,         // 0=write, 1=read
    input  wire [6:0] cmd_addr,       // 7-bit I2C slave address
    input  wire [7:0] cmd_reg,        // register address to access
    input  wire [15:0] cmd_wdata,     // data to write (16-bit)
    output reg  [15:0] rdata,         // data read back
    output reg        done,           // pulse when transaction complete
    output reg        ack_error,      // NACK received

    // I2C bus (open-drain)
    output reg        scl_oe,         // 1 = drive SCL low
    output reg        sda_oe,         // 1 = drive SDA low
    input  wire       sda_in         // sampled SDA line
);

    // ---- Bit-clock divider ------------------------------------
    // half-period = 62 cycles → 400 kHz
    reg [5:0] clk_cnt;
    reg       clk_en;   // pulses at half-period boundaries

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= 6'd0;
            clk_en  <= 1'b0;
        end else if (clk_cnt == `I2C_HALF_PER - 1) begin
            clk_cnt <= 6'd0;
            clk_en  <= 1'b1;
        end else begin
            clk_cnt <= clk_cnt + 1'b1;
            clk_en  <= 1'b0;
        end
    end

    // ---- Quarter-period pulse (used for SCL edges) -----------
    // SCL high for 1 half-period, low for 1 half-period
    // We track phase inside each bit: 0=SCL_LO, 1=SCL_HI
    reg scl_phase; // alternates on each clk_en

    // ---- FSM states ------------------------------------------
    localparam [4:0]
        S_IDLE       = 5'd0,
        S_START      = 5'd1,   // generate START
        S_ADDR       = 5'd2,   // send address + R/W
        S_ADDR_ACK   = 5'd3,
        S_REG        = 5'd4,   // send register byte
        S_REG_ACK    = 5'd5,
        S_WDATA_H    = 5'd6,   // write high byte
        S_WDATA_H_ACK= 5'd7,
        S_WDATA_L    = 5'd8,   // write low byte
        S_WDATA_L_ACK= 5'd9,
        S_RSTART     = 5'd10,  // repeated START
        S_RADDR      = 5'd11,  // send address + R
        S_RADDR_ACK  = 5'd12,
        S_READ_H     = 5'd13,  // read high byte
        S_READ_H_ACK = 5'd14,
        S_READ_L     = 5'd15,  // read low byte
        S_READ_L_ACK = 5'd16,
        S_STOP       = 5'd17,
        S_DONE       = 5'd18;

    reg [4:0] state;
    reg [7:0] shift;    // shift register
    reg [2:0] bit_idx;  // current bit (7 downto 0)
    reg [7:0] rx_byte;  // received byte accumulator

    // latch cmd inputs at start
    reg        lrw;
    reg [6:0]  laddr;
    reg [7:0]  lreg;
    reg [15:0] lwdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            scl_oe    <= 1'b0;
            sda_oe    <= 1'b0;
            done      <= 1'b0;
            ack_error <= 1'b0;
            rdata     <= 16'd0;
            scl_phase <= 1'b0;
            bit_idx   <= 3'd7;
            shift     <= 8'd0;
            rx_byte   <= 8'd0;
        end else begin
            done      <= 1'b0;
            ack_error <= 1'b0;

            case (state)
                // ------------------------------------------------
                S_IDLE: begin
                    scl_oe <= 1'b0;
                    sda_oe <= 1'b0;
                    if (cmd_start) begin
                        lrw    <= cmd_rw;
                        laddr  <= cmd_addr;
                        lreg   <= cmd_reg;
                        lwdata <= cmd_wdata;
                        state  <= S_START;
                        scl_phase <= 1'b1; // SCL already high
                    end
                end

                // ---- START condition: SDA falls while SCL high --
                S_START: begin
                    if (clk_en) begin
                        sda_oe    <= 1'b1;   // pull SDA low  (START; SCL was high)
                        scl_oe    <= 1'b1;   // pull SCL low  (begin bit clock)
                        scl_phase <= 1'b0;   // SCL is now low → if-branch sends bit 7
                        shift     <= {laddr, 1'b0}; // addr + W
                        bit_idx   <= 3'd7;
                        state     <= S_ADDR;
                    end
                end

                // ---- Send 8 bits (address byte) -----------------
                S_ADDR: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            // SCL rising edge – set up data
                            sda_oe <= ~shift[7];
                            scl_oe <= 1'b0;  // release SCL
                        end else begin
                            // SCL falling edge
                            scl_oe <= 1'b1;
                            if (bit_idx == 3'd0) begin
                                state <= S_ADDR_ACK;
                            end else begin
                                shift   <= {shift[6:0], 1'b0};
                                bit_idx <= bit_idx - 1'b1;
                            end
                        end
                    end
                end

                // ---- Sample ACK ---------------------------------
                S_ADDR_ACK: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b0; // release SDA for ACK
                            scl_oe <= 1'b0; // SCL high
                        end else begin
                            // sample SDA
                            if (sda_in) begin
                                ack_error <= 1'b1;
                                state     <= S_STOP;
                            end else begin
                                scl_oe  <= 1'b1;
                                shift   <= lreg;
                                bit_idx <= 3'd7;
                                state   <= S_REG;
                            end
                        end
                    end
                end

                // ---- Send register address byte -----------------
                S_REG: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= ~shift[7];
                            scl_oe <= 1'b0;
                        end else begin
                            scl_oe <= 1'b1;
                            if (bit_idx == 3'd0)
                                state <= S_REG_ACK;
                            else begin
                                shift   <= {shift[6:0], 1'b0};
                                bit_idx <= bit_idx - 1'b1;
                            end
                        end
                    end
                end

                S_REG_ACK: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b0;
                            scl_oe <= 1'b0;
                        end else begin
                            if (sda_in) begin
                                ack_error <= 1'b1;
                                state     <= S_STOP;
                            end else begin
                                scl_oe <= 1'b1;
                                if (lrw) begin
                                    // Read transaction: send repeated START
                                    state <= S_RSTART;
                                end else begin
                                    shift   <= lwdata[15:8];
                                    bit_idx <= 3'd7;
                                    state   <= S_WDATA_H;
                                end
                            end
                        end
                    end
                end

                // ---- Write high byte ----------------------------
                S_WDATA_H: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= ~shift[7];
                            scl_oe <= 1'b0;
                        end else begin
                            scl_oe <= 1'b1;
                            if (bit_idx == 3'd0)
                                state <= S_WDATA_H_ACK;
                            else begin
                                shift   <= {shift[6:0], 1'b0};
                                bit_idx <= bit_idx - 1'b1;
                            end
                        end
                    end
                end

                S_WDATA_H_ACK: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b0;
                            scl_oe <= 1'b0;
                        end else begin
                            if (sda_in) begin
                                ack_error <= 1'b1;
                                state     <= S_STOP;
                            end else begin
                                scl_oe  <= 1'b1;
                                shift   <= lwdata[7:0];
                                bit_idx <= 3'd7;
                                state   <= S_WDATA_L;
                            end
                        end
                    end
                end

                // ---- Write low byte -----------------------------
                S_WDATA_L: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= ~shift[7];
                            scl_oe <= 1'b0;
                        end else begin
                            scl_oe <= 1'b1;
                            if (bit_idx == 3'd0)
                                state <= S_WDATA_L_ACK;
                            else begin
                                shift   <= {shift[6:0], 1'b0};
                                bit_idx <= bit_idx - 1'b1;
                            end
                        end
                    end
                end

                S_WDATA_L_ACK: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b0;
                            scl_oe <= 1'b0;
                        end else begin
                            scl_oe <= 1'b1;
                            if (sda_in) ack_error <= 1'b1;
                            state <= S_STOP;
                        end
                    end
                end

                // ---- Repeated START for read --------------------
                S_RSTART: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b0; // SDA high
                            scl_oe <= 1'b0; // SCL high
                        end else begin
                            sda_oe  <= 1'b1; // SDA falls → rSTART
                            scl_oe  <= 1'b1; // SCL low
                            shift   <= {laddr, 1'b1}; // addr + R
                            bit_idx <= 3'd7;
                            state   <= S_RADDR;
                        end
                    end
                end

                S_RADDR: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= ~shift[7];
                            scl_oe <= 1'b0;
                        end else begin
                            scl_oe <= 1'b1;
                            if (bit_idx == 3'd0)
                                state <= S_RADDR_ACK;
                            else begin
                                shift   <= {shift[6:0], 1'b0};
                                bit_idx <= bit_idx - 1'b1;
                            end
                        end
                    end
                end

                S_RADDR_ACK: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b0;
                            scl_oe <= 1'b0;
                        end else begin
                            if (sda_in) begin
                                ack_error <= 1'b1;
                                state     <= S_STOP;
                            end else begin
                                scl_oe  <= 1'b1;
                                bit_idx <= 3'd7;
                                rx_byte <= 8'd0;
                                state   <= S_READ_H;
                            end
                        end
                    end
                end

                // ---- Receive high byte --------------------------
                S_READ_H: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b0; // release SDA
                            scl_oe <= 1'b0; // SCL high
                        end else begin
                            // sample on falling edge (after SCL high)
                            rx_byte <= {rx_byte[6:0], sda_in};
                            scl_oe  <= 1'b1;
                            if (bit_idx == 3'd0)
                                state <= S_READ_H_ACK;
                            else
                                bit_idx <= bit_idx - 1'b1;
                        end
                    end
                end

                S_READ_H_ACK: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b1; // ACK: master pulls SDA low
                            scl_oe <= 1'b0;
                        end else begin
                            rdata[15:8] <= rx_byte;
                            scl_oe      <= 1'b1;
                            bit_idx     <= 3'd7;
                            rx_byte     <= 8'd0;
                            state       <= S_READ_L;
                        end
                    end
                end

                // ---- Receive low byte ---------------------------
                S_READ_L: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b0;
                            scl_oe <= 1'b0;
                        end else begin
                            rx_byte <= {rx_byte[6:0], sda_in};
                            scl_oe  <= 1'b1;
                            if (bit_idx == 3'd0)
                                state <= S_READ_L_ACK;
                            else
                                bit_idx <= bit_idx - 1'b1;
                        end
                    end
                end

                S_READ_L_ACK: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b0; // NACK: master releases SDA (high)
                            scl_oe <= 1'b0;
                        end else begin
                            rdata[7:0] <= rx_byte;
                            scl_oe     <= 1'b1;
                            state      <= S_STOP;
                        end
                    end
                end

                // ---- STOP condition: SDA rises while SCL high ---
                S_STOP: begin
                    if (clk_en) begin
                        scl_phase <= ~scl_phase;
                        if (!scl_phase) begin
                            sda_oe <= 1'b1; // hold SDA low
                            scl_oe <= 1'b0; // release SCL
                        end else begin
                            sda_oe <= 1'b0; // release SDA → STOP
                            state  <= S_DONE;
                        end
                    end
                end

                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
