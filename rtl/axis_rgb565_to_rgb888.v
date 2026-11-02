`timescale 1ns / 1ps

module axis_rgb565_to_rgb888 #(
    parameter AXIS_PATTERN_TEST = 1'b0
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TDATA" *)
    input  wire [15:0] s_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TVALID" *)
    input  wire        s_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TREADY" *)
    output wire        s_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TUSER" *)
    input  wire        s_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 S_AXIS TLAST" *)
    input  wire        s_axis_tlast,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    output wire [23:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output wire        m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input  wire        m_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TUSER" *)
    output wire        m_axis_tuser,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output wire        m_axis_tlast
);

    wire [4:0] red5   = s_axis_tdata[15:11];
    wire [5:0] green6 = s_axis_tdata[10:5];
    wire [4:0] blue5  = s_axis_tdata[4:0];
    reg [10:0] pattern_x;
    reg [9:0]  pattern_y;
    reg [23:0] pattern_rgb;

    // Optional diagnostic pattern generated inside the AXI4-Stream path.  It
    // preserves VDMA TVALID/TREADY/TUSER/TLAST, so Video Out and its async FIFO
    // are exercised while the DDR pixel contents themselves are ignored.
    always @(posedge aclk) begin
        if (!aresetn) begin
            pattern_x <= 11'd0;
            pattern_y <= 10'd0;
        end else if (s_axis_tvalid && s_axis_tready) begin
            if (s_axis_tuser) begin
                pattern_x <= 11'd1;
                pattern_y <= 10'd0;
            end else if (s_axis_tlast) begin
                pattern_x <= 11'd0;
                pattern_y <= pattern_y + 1'b1;
            end else begin
                pattern_x <= pattern_x + 1'b1;
            end
        end
    end

    wire [10:0] pattern_pixel_x = s_axis_tuser ? 11'd0 : pattern_x;

    always @* begin
        if (pattern_pixel_x < 11'd100)
            pattern_rgb = 24'hFFFFFF;
        else if (pattern_pixel_x < 11'd200)
            pattern_rgb = 24'hFFFF00;
        else if (pattern_pixel_x < 11'd300)
            pattern_rgb = 24'h00FFFF;
        else if (pattern_pixel_x < 11'd400)
            pattern_rgb = 24'h00FF00;
        else if (pattern_pixel_x < 11'd500)
            pattern_rgb = 24'hFF00FF;
        else if (pattern_pixel_x < 11'd600)
            pattern_rgb = 24'hFF0000;
        else if (pattern_pixel_x < 11'd700)
            pattern_rgb = 24'h0000FF;
        else
            pattern_rgb = 24'h848284;
    end

    // Expand RGB565 to the RGB888 byte ordering expected by Video Out.
    assign m_axis_tdata  = AXIS_PATTERN_TEST ? pattern_rgb :
                           {{red5, red5[4:2]},
                            {green6, green6[5:4]},
                            {blue5, blue5[4:2]}};
    assign m_axis_tvalid = s_axis_tvalid;
    assign s_axis_tready = m_axis_tready;
    assign m_axis_tuser  = s_axis_tuser;
    assign m_axis_tlast  = s_axis_tlast;

    // In normal operation the RGB565 conversion remains combinational.

endmodule
