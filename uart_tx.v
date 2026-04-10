// ============================================================
// uart_tx.v — 8-N-1 UART transmitter (115200 baud)
// ============================================================
module uart_tx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       baud_tick,  // one pulse per bit period
    input  wire [7:0] data_in,
    input  wire       data_valid, // pulse to start transmission
    output reg        tx,
    output reg        busy
);
    // FSM states
    localparam IDLE  = 2'd0;
    localparam START = 2'd1;
    localparam DATA  = 2'd2;
    localparam STOP  = 2'd3;

    reg [1:0] state;
    reg [7:0] shift;
    reg [2:0] bit_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            tx      <= 1'b1;
            busy    <= 1'b0;
            shift   <= 8'd0;
            bit_cnt <= 3'd0;
        end else begin
            case (state)
                IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    if (data_valid) begin
                        shift <= data_in;
                        busy  <= 1'b1;
                        state <= START;
                    end
                end
                START: begin
                    if (baud_tick) begin
                        tx      <= 1'b0;   // start bit
                        bit_cnt <= 3'd0;
                        state   <= DATA;
                    end
                end
                DATA: begin
                    if (baud_tick) begin
                        tx    <= shift[0];
                        shift <= {1'b0, shift[7:1]};
                        if (bit_cnt == 3'd7)
                            state <= STOP;
                        else
                            bit_cnt <= bit_cnt + 1'b1;
                    end
                end
                STOP: begin
                    if (baud_tick) begin
                        tx    <= 1'b1;   // stop bit
                        state <= IDLE;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
