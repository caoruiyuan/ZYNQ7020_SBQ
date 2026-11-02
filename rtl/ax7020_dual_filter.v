// Simultaneous AN9238 capture and matched 8:1 decimation on both channels.
// q3_fir_0 must be regenerated for a 65 MHz clock before implementation.
module ax7020_dual_filter (
    input  wire        adc_clk,
    input  wire        adc_sample_clk,
    input  wire        resetn,
    input  wire [11:0] adc_ch1_data,
    input  wire [11:0] adc_ch2_data,
    input  wire        adc_ch1_otr,
    input  wire        adc_ch2_otr,
    output wire        adc_ch1_clk,
    output wire        adc_ch2_clk,
    output wire        adc_valid,
    output wire signed [15:0] ch1_raw,
    output wire signed [15:0] ch2_raw,
    output wire        ch1_overrange,
    output wire        ch2_overrange,
    output reg  [31:0] time_word,
    output reg  signed [23:0] ch1_filtered,
    output reg  signed [23:0] ch2_filtered,
    output reg         pair_valid,
    output reg         flow_error
);
    wire signed [15:0] ch1_sample;
    wire signed [15:0] ch2_sample;
    wire signed [23:0] ch1_cic;
    wire signed [23:0] ch2_cic;
    wire ch1_cic_valid;
    wire ch2_cic_valid;
    wire [23:0] ch1_fir;
    wire [23:0] ch2_fir;
    wire ch1_fir_valid;
    wire ch2_fir_valid;
    wire ch1_fir_ready;
    wire ch2_fir_ready;

    an9238_dual_capture u_capture (
        .adc_clk(adc_clk), .adc_sample_clk(adc_sample_clk), .resetn(resetn),
        .adc_ch1_data(adc_ch1_data), .adc_ch2_data(adc_ch2_data),
        .adc_ch1_otr(adc_ch1_otr), .adc_ch2_otr(adc_ch2_otr),
        .adc_ch1_clk(adc_ch1_clk), .adc_ch2_clk(adc_ch2_clk),
        .ch1_sample(ch1_sample), .ch2_sample(ch2_sample),
        .ch1_overrange(ch1_overrange), .ch2_overrange(ch2_overrange),
        .sample_valid(adc_valid)
    );
    assign ch1_raw = ch1_sample;
    assign ch2_raw = ch2_sample;

    q3_cic_decimator #(.DECIMATION(8)) u_ch1_cic (
        .clk(adc_clk), .resetn(resetn),
        .din(ch1_sample), .din_valid(adc_valid),
        .dout(ch1_cic), .dout_valid(ch1_cic_valid)
    );
    q3_cic_decimator #(.DECIMATION(8)) u_ch2_cic (
        .clk(adc_clk), .resetn(resetn),
        .din(ch2_sample), .din_valid(adc_valid),
        .dout(ch2_cic), .dout_valid(ch2_cic_valid)
    );

    q3_fir_0 u_ch1_fir (
        .aclk(adc_clk), .aresetn(resetn),
        .s_axis_data_tvalid(ch1_cic_valid),
        .s_axis_data_tready(ch1_fir_ready),
        .s_axis_data_tdata(ch1_cic),
        .m_axis_data_tvalid(ch1_fir_valid),
        .m_axis_data_tdata(ch1_fir)
    );
    q3_fir_0 u_ch2_fir (
        .aclk(adc_clk), .aresetn(resetn),
        .s_axis_data_tvalid(ch2_cic_valid),
        .s_axis_data_tready(ch2_fir_ready),
        .s_axis_data_tdata(ch2_cic),
        .m_axis_data_tvalid(ch2_fir_valid),
        .m_axis_data_tdata(ch2_fir)
    );

    function [15:0] signed16;
        input signed [23:0] sample;
        begin
            if (sample > 24'sd32767)
                signed16 = 16'h7fff;
            else if (sample < -24'sd32768)
                signed16 = 16'h8000;
            else
                signed16 = sample[15:0];
        end
    endfunction

    always @(posedge adc_clk) begin
        if (!resetn) begin
            time_word <= 32'd0;
            ch1_filtered <= 24'sd0;
            ch2_filtered <= 24'sd0;
            pair_valid <= 1'b0;
            flow_error <= 1'b0;
        end else begin
            pair_valid <= ch1_fir_valid && ch2_fir_valid;
            if (ch1_fir_valid && ch2_fir_valid) begin
                ch1_filtered <= ch1_fir;
                ch2_filtered <= ch2_fir;
                time_word <= {signed16(ch2_fir), signed16(ch1_fir)};
            end
            if (ch1_cic_valid != ch2_cic_valid ||
                ch1_fir_valid != ch2_fir_valid ||
                (ch1_cic_valid && !ch1_fir_ready) ||
                (ch2_cic_valid && !ch2_fir_ready))
                flow_error <= 1'b1;
        end
    end
endmodule
