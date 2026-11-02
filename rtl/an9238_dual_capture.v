// Two AD9238 channels use the same 65 MHz encode and capture clocks.
// The sample clock phase must be chosen from the AD9238 output-delay budget
// and checked against the routed input timing on AX7020.
module an9238_dual_capture (
    input  wire               adc_clk,
    input  wire               adc_sample_clk,
    input  wire               resetn,
    input  wire [11:0]        adc_ch1_data,
    input  wire [11:0]        adc_ch2_data,
    input  wire               adc_ch1_otr,
    input  wire               adc_ch2_otr,
    output wire               adc_ch1_clk,
    output wire               adc_ch2_clk,
    output wire signed [15:0] ch1_sample,
    output wire signed [15:0] ch2_sample,
    output reg                ch1_overrange,
    output reg                ch2_overrange,
    output reg                sample_valid
);

    (* ASYNC_REG = "TRUE" *) reg [1:0] sample_reset_sync;
    wire sample_resetn = sample_reset_sync[1];

    always @(posedge adc_sample_clk or negedge resetn) begin
        if (!resetn)
            sample_reset_sync <= 2'b00;
        else
            sample_reset_sync <= {sample_reset_sync[0], 1'b1};
    end

    (* IOB = "TRUE" *) reg [11:0] ch1_input;
    (* IOB = "TRUE" *) reg [11:0] ch2_input;
    (* IOB = "TRUE" *) reg ch1_otr_input;
    (* IOB = "TRUE" *) reg ch2_otr_input;
    reg input_valid;

    always @(posedge adc_sample_clk) begin
        if (!sample_resetn) begin
            ch1_input <= 12'd0;
            ch2_input <= 12'd0;
            ch1_otr_input <= 1'b0;
            ch2_otr_input <= 1'b0;
            input_valid <= 1'b0;
        end else begin
            ch1_input <= adc_ch1_data;
            ch2_input <= adc_ch2_data;
            ch1_otr_input <= adc_ch1_otr;
            ch2_otr_input <= adc_ch2_otr;
            input_valid <= 1'b1;
        end
    end

    reg [11:0] ch1_code;
    reg [11:0] ch2_code;
    always @(posedge adc_clk or negedge resetn) begin
        if (!resetn) begin
            ch1_code <= 12'd0;
            ch2_code <= 12'd0;
            ch1_overrange <= 1'b0;
            ch2_overrange <= 1'b0;
            sample_valid <= 1'b0;
        end else begin
            ch1_code <= ch1_input;
            ch2_code <= ch2_input;
            ch1_overrange <= ch1_otr_input;
            ch2_overrange <= ch2_otr_input;
            sample_valid <= input_valid;
        end
    end

    // AD9238 offset binary: 0x800 is zero, 0x000 is -2048.
    assign ch1_sample = {{4{~ch1_code[11]}}, ~ch1_code[11], ch1_code[10:0]};
    assign ch2_sample = {{4{~ch2_code[11]}}, ~ch2_code[11], ch2_code[10:0]};

    ODDR #(.DDR_CLK_EDGE("SAME_EDGE")) u_ch1_clk (
        .Q(adc_ch1_clk), .C(adc_clk), .CE(1'b1),
        .D1(1'b1), .D2(1'b0), .R(~resetn), .S(1'b0)
    );
    ODDR #(.DDR_CLK_EDGE("SAME_EDGE")) u_ch2_clk (
        .Q(adc_ch2_clk), .C(adc_clk), .CE(1'b1),
        .D1(1'b1), .D2(1'b0), .R(~resetn), .S(1'b0)
    );
endmodule
