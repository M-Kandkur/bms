// ============================================================
// data_converter.v — Raw INA226 registers → physical units
//
// Bus voltage  : reg[15:3] × 1.25 mV  → result in mV (32-bit)
// Shunt voltage: signed reg × 2.5 µV  → result in µV (32-bit signed)
// Current      : signed reg × 1 mA    → result in mA (32-bit signed)
//                (1 mA/LSB when CAL=512, Rshunt=0.1 Ω)
// Power        : reg × 25 mW          → result in mW (32-bit)
// ============================================================
module data_converter (
    input  wire        clk,
    input  wire        rst_n,

    // Raw inputs from register file
    input  wire [15:0] shunt_voltage_raw,
    input  wire [15:0] bus_voltage_raw,
    input  wire [15:0] power_raw,
    input  wire [15:0] current_raw,
    input  wire        data_valid,

    // Converted outputs
    output reg  [31:0] voltage_mv,      // bus voltage in mV
    output reg  [31:0] shunt_uv,        // shunt voltage in µV (unsigned magnitude)
    output reg         shunt_neg,       // 1 = negative shunt voltage
    output reg  [31:0] current_ma,      // current magnitude in mA
    output reg         current_neg,     // 1 = negative current
    output reg  [31:0] power_mw,        // power in mW
    output reg         conv_valid       // outputs are valid
);

    // Bus voltage: full 16-bit register × 1.25 mV  (INA226 uses all 16 bits,
    // unlike INA219 which has status bits in [2:0]).
    // 1.25 mV = 5/4 mV → voltage_mv = bus_voltage_raw * 5 / 4

    // Shunt voltage is signed 16-bit, LSB = 2.5 µV
    // 2.5 µV = 5/2 µV → multiply by 5 then shift right 1
    wire        shunt_sign = shunt_voltage_raw[15];
    wire [15:0] shunt_mag  = shunt_sign ? (~shunt_voltage_raw + 1'b1)
                                        : shunt_voltage_raw;

    // Current register is signed 16-bit, 1 mA/LSB
    wire        cur_sign = current_raw[15];
    wire [15:0] cur_mag  = cur_sign ? (~current_raw + 1'b1)
                                    : current_raw;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            voltage_mv  <= 32'd0;
            shunt_uv    <= 32'd0;
            shunt_neg   <= 1'b0;
            current_ma  <= 32'd0;
            current_neg <= 1'b0;
            power_mw    <= 32'd0;
            conv_valid  <= 1'b0;
        end else begin
            conv_valid <= data_valid;
            if (data_valid) begin
                // Voltage: full 16-bit register × 5 / 4  (× 1.25 mV)
                voltage_mv  <= ({16'd0, bus_voltage_raw} * 32'd5) >> 2;

                // Shunt voltage: mag * 5 / 2 µV
                shunt_uv    <= ({16'd0, shunt_mag} * 32'd5) >> 1;
                shunt_neg   <= shunt_sign;

                // Current: magnitude directly in mA (1 mA/LSB)
                current_ma  <= {16'd0, cur_mag};
                current_neg <= cur_sign;

                // Power: reg * 25 mW
                power_mw    <= {16'd0, power_raw} * 32'd25;
            end
        end
    end
endmodule
