// ============================================================
// ina226_controller.v — FSM to initialise INA226 and
// continuously read shunt voltage, bus voltage, power, current.
// ============================================================
`include "defines.v"
module ina226_controller (
    input  wire        clk,
    input  wire        rst_n,

    // Trigger: sample one set of readings
    input  wire        trigger,

    // I2C master command interface
    output reg         i2c_start,
    output reg         i2c_rw,       // 0=write, 1=read
    output reg  [6:0]  i2c_addr,
    output reg  [7:0]  i2c_reg,
    output reg  [15:0] i2c_wdata,
    input  wire [15:0] i2c_rdata,
    input  wire        i2c_done,
    input  wire        i2c_ack_error,

    // Register file write port
    output reg         reg_wr_en,
    output reg  [2:0]  reg_wr_sel,
    output reg  [15:0] reg_wr_data,

    // Status
    output reg         error_flag,
    output reg         initialised
);

    localparam [3:0]
        S_IDLE       = 4'd0,
        S_INIT_CFG   = 4'd1,   // write configuration register
        S_WAIT_CFG   = 4'd2,
        S_INIT_CAL   = 4'd3,   // write calibration register
        S_WAIT_CAL   = 4'd4,
        S_READ_SHUNT = 4'd5,
        S_WAIT_SHUNT = 4'd6,
        S_READ_BUS   = 4'd7,
        S_WAIT_BUS   = 4'd8,
        S_READ_PWR   = 4'd9,
        S_WAIT_PWR   = 4'd10,
        S_READ_CUR   = 4'd11,
        S_WAIT_CUR   = 4'd12,
        S_DONE       = 4'd13;

    reg [3:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            i2c_start   <= 1'b0;
            i2c_rw      <= 1'b0;
            i2c_addr    <= `INA226_ADDR;
            i2c_reg     <= 8'd0;
            i2c_wdata   <= 16'd0;
            reg_wr_en   <= 1'b0;
            reg_wr_sel  <= 3'd0;
            reg_wr_data <= 16'd0;
            error_flag  <= 1'b0;
            initialised <= 1'b0;
        end else begin
            i2c_start  <= 1'b0;
            reg_wr_en  <= 1'b0;

            case (state)
                // ---- Wait for first trigger ------------------
                S_IDLE: begin
                    if (trigger && !initialised)
                        state <= S_INIT_CFG;
                    else if (trigger && initialised)
                        state <= S_READ_SHUNT;
                end

                // ---- Write CONFIG register -------------------
                S_INIT_CFG: begin
                    i2c_start  <= 1'b1;
                    i2c_rw     <= 1'b0;
                    i2c_addr   <= `INA226_ADDR;
                    i2c_reg    <= `REG_CONFIG;
                    i2c_wdata  <= `INA226_CFG;
                    state      <= S_WAIT_CFG;
                end
                S_WAIT_CFG: begin
                    if (i2c_done) begin
                        if (i2c_ack_error) error_flag <= 1'b1;
                        state <= S_INIT_CAL;
                    end
                end

                // ---- Write CALIBRATION register --------------
                S_INIT_CAL: begin
                    i2c_start  <= 1'b1;
                    i2c_rw     <= 1'b0;
                    i2c_addr   <= `INA226_ADDR;
                    i2c_reg    <= `REG_CALIB;
                    i2c_wdata  <= `INA226_CAL;
                    state      <= S_WAIT_CAL;
                end
                S_WAIT_CAL: begin
                    if (i2c_done) begin
                        if (i2c_ack_error) error_flag <= 1'b1;
                        initialised <= 1'b1;
                        state       <= S_READ_SHUNT;
                    end
                end

                // ---- Read SHUNT VOLTAGE (0x01) ---------------
                S_READ_SHUNT: begin
                    i2c_start <= 1'b1;
                    i2c_rw    <= 1'b1;
                    i2c_addr  <= `INA226_ADDR;
                    i2c_reg   <= `REG_SHUNT_V;
                    state     <= S_WAIT_SHUNT;
                end
                S_WAIT_SHUNT: begin
                    if (i2c_done) begin
                        if (i2c_ack_error) error_flag <= 1'b1;
                        reg_wr_en   <= 1'b1;
                        reg_wr_sel  <= 3'd0; // SEL_SHUNT
                        reg_wr_data <= i2c_rdata;
                        state       <= S_READ_BUS;
                    end
                end

                // ---- Read BUS VOLTAGE (0x02) -----------------
                S_READ_BUS: begin
                    i2c_start <= 1'b1;
                    i2c_rw    <= 1'b1;
                    i2c_addr  <= `INA226_ADDR;
                    i2c_reg   <= `REG_BUS_V;
                    state     <= S_WAIT_BUS;
                end
                S_WAIT_BUS: begin
                    if (i2c_done) begin
                        if (i2c_ack_error) error_flag <= 1'b1;
                        reg_wr_en   <= 1'b1;
                        reg_wr_sel  <= 3'd1; // SEL_BUS
                        reg_wr_data <= i2c_rdata;
                        state       <= S_READ_PWR;
                    end
                end

                // ---- Read POWER (0x03) -----------------------
                S_READ_PWR: begin
                    i2c_start <= 1'b1;
                    i2c_rw    <= 1'b1;
                    i2c_addr  <= `INA226_ADDR;
                    i2c_reg   <= `REG_POWER;
                    state     <= S_WAIT_PWR;
                end
                S_WAIT_PWR: begin
                    if (i2c_done) begin
                        if (i2c_ack_error) error_flag <= 1'b1;
                        reg_wr_en   <= 1'b1;
                        reg_wr_sel  <= 3'd2; // SEL_POWER
                        reg_wr_data <= i2c_rdata;
                        state       <= S_READ_CUR;
                    end
                end

                // ---- Read CURRENT (0x04) ---------------------
                S_READ_CUR: begin
                    i2c_start <= 1'b1;
                    i2c_rw    <= 1'b1;
                    i2c_addr  <= `INA226_ADDR;
                    i2c_reg   <= `REG_CURRENT;
                    state     <= S_WAIT_CUR;
                end
                S_WAIT_CUR: begin
                    if (i2c_done) begin
                        if (i2c_ack_error) error_flag <= 1'b1;
                        reg_wr_en   <= 1'b1;
                        reg_wr_sel  <= 3'd3; // SEL_CURRENT
                        reg_wr_data <= i2c_rdata;
                        state       <= S_DONE;
                    end
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
