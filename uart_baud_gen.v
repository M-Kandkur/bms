// ============================================================
// uart_baud_gen.v — Baud-rate tick generator
// 50 MHz clock → 115200 baud tick (one cycle pulse per bit)
// ============================================================
`include "defines.v"
module uart_baud_gen (
    input  wire clk,
    input  wire rst_n,
    output reg  tick        // one-cycle pulse every bit period
);
    reg [15:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt  <= 16'd0;
            tick <= 1'b0;
        end else if (cnt == `UART_DIV - 1) begin
            cnt  <= 16'd0;
            tick <= 1'b1;
        end else begin
            cnt  <= cnt + 1'b1;
            tick <= 1'b0;
        end
    end
endmodule
