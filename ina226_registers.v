// ============================================================
// ina226_registers.v — Register file for INA226 readings
// Stores the four 16-bit measurement values.
// ============================================================
module ina226_registers (
    input  wire        clk,
    input  wire        rst_n,

    // Write port (from controller)
    input  wire        wr_en,
    input  wire [2:0]  wr_sel,    // which register: 0-3
    input  wire [15:0] wr_data,

    // Read outputs (always available)
    output reg  [15:0] shunt_voltage_raw,
    output reg  [15:0] bus_voltage_raw,
    output reg  [15:0] power_raw,
    output reg  [15:0] current_raw,
    output reg         data_valid    // all 4 registers have been written
);

    // sel encoding
    localparam SEL_SHUNT   = 3'd0;
    localparam SEL_BUS     = 3'd1;
    localparam SEL_POWER   = 3'd2;
    localparam SEL_CURRENT = 3'd3;

    reg [3:0] valid_mask; // one bit per register

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shunt_voltage_raw <= 16'd0;
            bus_voltage_raw   <= 16'd0;
            power_raw         <= 16'd0;
            current_raw       <= 16'd0;
            valid_mask        <= 4'd0;
            data_valid        <= 1'b0;
        end else begin
            if (wr_en) begin
                case (wr_sel)
                    SEL_SHUNT:   begin shunt_voltage_raw <= wr_data; valid_mask[0] <= 1'b1; end
                    SEL_BUS:     begin bus_voltage_raw   <= wr_data; valid_mask[1] <= 1'b1; end
                    SEL_POWER:   begin power_raw         <= wr_data; valid_mask[2] <= 1'b1; end
                    SEL_CURRENT: begin current_raw       <= wr_data; valid_mask[3] <= 1'b1; end
                    default: ;
                endcase
            end
            data_valid <= &valid_mask;
        end
    end
endmodule
