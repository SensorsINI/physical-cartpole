module myproject (
    input wire          ap_clk,
    input wire          ap_rst,
    input wire          ap_start,
    input wire          ap_done,
    input wire          ap_idle,
    input wire          ap_ready,
    input wire  [697:0] input_1_V,
    input wire          input_1_V_ap_vld,
    output wire [999:0] layer9_out_0_V,
    output wire         layer9_out_0_V_ap_vld
);

    net net(
        .in(input_1_V),
        .out(layer9_out_0_V)
    );

endmodule : myproject