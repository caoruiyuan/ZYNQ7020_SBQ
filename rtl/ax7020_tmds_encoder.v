`timescale 1ns / 1ps

module ax7020_tmds_encoder (
    input  wire       pixel_clk,
    input  wire       reset,
    input  wire [7:0] data,
    input  wire       de,
    input  wire [1:0] control,
    output reg  [9:0] symbol
);
    reg [8:0] transition_data;
    reg [3:0] ones_data;
    reg [3:0] ones_encoded;
    reg signed [5:0] disparity = 6'sd0;
    reg signed [5:0] balance;
    reg use_xor;
    integer i;

    always @* begin
        ones_data = 4'd0;
        ones_encoded = 4'd0;
        transition_data = 9'd0;
        for (i = 0; i < 8; i = i + 1)
            ones_data = ones_data + data[i];
        use_xor = ones_data < 4 || (ones_data == 4 && data[0]);
        transition_data[0] = data[0];
        for (i = 1; i < 8; i = i + 1)
            transition_data[i] = use_xor ?
                transition_data[i-1] ^ data[i] :
                ~(transition_data[i-1] ^ data[i]);
        transition_data[8] = use_xor;
        for (i = 0; i < 8; i = i + 1)
            ones_encoded = ones_encoded + transition_data[i];
        balance = $signed({2'b00, ones_encoded}) * 2 - 8;
    end

    always @(posedge pixel_clk) begin
        if (reset || !de) begin
            disparity <= 6'sd0;
            case (control)
                2'b00: symbol <= 10'b1101010100;
                2'b01: symbol <= 10'b0010101011;
                2'b10: symbol <= 10'b0101010100;
                default: symbol <= 10'b1010101011;
            endcase
        end else if (disparity == 0 || ones_encoded == 4) begin
            symbol <= {~transition_data[8], transition_data[8],
                       transition_data[8] ? transition_data[7:0] :
                                            ~transition_data[7:0]};
            disparity <= transition_data[8] ?
                         disparity + balance : disparity - balance;
        end else if ((disparity > 0 && balance > 0) ||
                     (disparity < 0 && balance < 0)) begin
            symbol <= {1'b1, transition_data[8], ~transition_data[7:0]};
            disparity <= disparity - balance +
                         (transition_data[8] ? 6'sd2 : 6'sd0);
        end else begin
            symbol <= {1'b0, transition_data[8], transition_data[7:0]};
            disparity <= disparity + balance -
                         (transition_data[8] ? 6'sd0 : 6'sd2);
        end
    end
endmodule
