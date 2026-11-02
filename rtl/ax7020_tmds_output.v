`timescale 1ns / 1ps

module ax7020_tmds_lane (
    input  wire       pixel_clk,
    input  wire       serial_clk,
    input  wire       reset,
    input  wire       enable,
    input  wire [9:0] symbol,
    output wire       pin_p,
    output wire       pin_n
);
    wire shift1, shift2;
    wire serial_data;
    wire serial_tristate;

    OSERDESE2 #(
        .DATA_RATE_OQ("DDR"), .DATA_RATE_TQ("BUF"),
        .DATA_WIDTH(10), .SERDES_MODE("MASTER"),
        .TRISTATE_WIDTH(1)
    ) master (
        .OQ(serial_data), .CLK(serial_clk), .CLKDIV(pixel_clk),
        .D1(symbol[0]), .D2(symbol[1]), .D3(symbol[2]),
        .D4(symbol[3]), .D5(symbol[4]), .D6(symbol[5]),
        .D7(symbol[6]), .D8(symbol[7]),
        .SHIFTIN1(shift1), .SHIFTIN2(shift2),
        .OCE(1'b1), .RST(reset), .T1(~enable), .T2(~enable),
        .T3(~enable), .T4(~enable), .TCE(1'b1),
        .OFB(), .TFB(), .TQ(serial_tristate), .SHIFTOUT1(), .SHIFTOUT2()
    );
    OSERDESE2 #(
        .DATA_RATE_OQ("DDR"), .DATA_RATE_TQ("BUF"),
        .DATA_WIDTH(10), .SERDES_MODE("SLAVE"),
        .TRISTATE_WIDTH(1)
    ) slave (
        .OQ(), .CLK(serial_clk), .CLKDIV(pixel_clk),
        .D1(1'b0), .D2(1'b0), .D3(symbol[8]), .D4(symbol[9]),
        .D5(1'b0), .D6(1'b0), .D7(1'b0), .D8(1'b0),
        .SHIFTIN1(1'b0), .SHIFTIN2(1'b0),
        .OCE(1'b1), .RST(reset), .T1(1'b0), .T2(1'b0),
        .T3(1'b0), .T4(1'b0), .TCE(1'b0),
        .OFB(), .TFB(), .TQ(), .SHIFTOUT1(shift1),
        .SHIFTOUT2(shift2)
    );
    OBUFTDS output_buffer (
        .I(serial_data), .T(serial_tristate), .O(pin_p), .OB(pin_n)
    );
endmodule

module ax7020_tmds_output (
    input  wire        pixel_clk,
    input  wire        serial_clk,
    input  wire        reset,
    input  wire        enable,
    input  wire [23:0] rgb,
    input  wire        de,
    input  wire        hsync,
    input  wire        vsync,
    output wire [3:0] tmds_p,
    output wire [3:0] tmds_n
);
    wire [9:0] symbol [0:2];
    ax7020_tmds_encoder blue (
        .pixel_clk(pixel_clk), .reset(reset),
        .data(rgb[7:0]), .de(de),
        .control({vsync, hsync}), .symbol(symbol[0])
    );
    ax7020_tmds_encoder green (
        .pixel_clk(pixel_clk), .reset(reset),
        .data(rgb[15:8]), .de(de),
        .control(2'b00), .symbol(symbol[1])
    );
    ax7020_tmds_encoder red (
        .pixel_clk(pixel_clk), .reset(reset),
        .data(rgb[23:16]), .de(de),
        .control(2'b00), .symbol(symbol[2])
    );
    genvar lane;
    generate for (lane = 0; lane < 3; lane = lane + 1) begin : data_lane
        ax7020_tmds_lane serializer (
            .pixel_clk(pixel_clk), .serial_clk(serial_clk),
            .reset(reset), .enable(enable), .symbol(symbol[lane]),
            .pin_p(tmds_p[lane]), .pin_n(tmds_n[lane])
        );
    end endgenerate
    ax7020_tmds_lane clock_lane (
        .pixel_clk(pixel_clk), .serial_clk(serial_clk),
        .reset(reset), .enable(enable), .symbol(10'b0000011111),
        .pin_p(tmds_p[3]), .pin_n(tmds_n[3])
    );
endmodule
