`timescale 1ns / 1ps

// Four-stage CIC decimator used by the question-3 measurement chain.
//
// The arithmetic intentionally wraps at INTERNAL_WIDTH.  This is the normal
// CIC implementation: as long as every stage uses the same two's-complement
// width, the comb section removes integrator wraparound.  Two guard bits are
// retained above the theoretical 16 + 4*log2(16) = 32 bit minimum.
module q3_cic_decimator #(
    parameter integer INPUT_WIDTH    = 16,
    parameter integer INTERNAL_WIDTH = 34,
    parameter integer OUTPUT_WIDTH   = 24,
    parameter integer DECIMATION     = 16
) (
    input  wire                          clk,
    input  wire                          resetn,
    input  wire signed [INPUT_WIDTH-1:0] din,
    input  wire                          din_valid,
    output reg  signed [OUTPUT_WIDTH-1:0] dout,
    output reg                           dout_valid
);

    localparam integer GAIN_SHIFT = 4 * $clog2(DECIMATION);

    reg signed [INTERNAL_WIDTH-1:0] integrator_1;
    reg signed [INTERNAL_WIDTH-1:0] integrator_2;
    reg signed [INTERNAL_WIDTH-1:0] integrator_3;
    reg signed [INTERNAL_WIDTH-1:0] integrator_4;

    reg signed [INTERNAL_WIDTH-1:0] comb_delay_1;
    reg signed [INTERNAL_WIDTH-1:0] comb_delay_2;
    reg signed [INTERNAL_WIDTH-1:0] comb_delay_3;
    reg signed [INTERNAL_WIDTH-1:0] comb_delay_4;
    reg signed [INTERNAL_WIDTH-1:0] comb_pipe_1;
    reg signed [INTERNAL_WIDTH-1:0] comb_pipe_2;
    reg signed [INTERNAL_WIDTH-1:0] comb_pipe_3;
    reg signed [INTERNAL_WIDTH-1:0] comb_pipe_4;
    reg [3:0] decimation_count;
    reg [4:0] comb_valid_pipe;

    wire signed [INTERNAL_WIDTH-1:0] din_extended =
        {{(INTERNAL_WIDTH-INPUT_WIDTH){din[INPUT_WIDTH-1]}}, din};

    // Convergent rounding is not required here because the following FIR and
    // end-to-end calibration remove the sub-LSB bias.  Symmetric half-away
    // rounding keeps positive and negative tones balanced.
    function automatic signed [OUTPUT_WIDTH-1:0] normalize_cic;
        input signed [INTERNAL_WIDTH-1:0] value;
        reg signed [INTERNAL_WIDTH-1:0] rounded;
        begin
            if (value >= 0)
                rounded = value + ({{(INTERNAL_WIDTH-1){1'b0}}, 1'b1}
                                   <<< (GAIN_SHIFT-1));
            else
                rounded = value - ({{(INTERNAL_WIDTH-1){1'b0}}, 1'b1}
                                   <<< (GAIN_SHIFT-1));
            normalize_cic = rounded >>> GAIN_SHIFT;
        end
    endfunction

    always @(posedge clk) begin
        if (!resetn) begin
            integrator_1   <= {INTERNAL_WIDTH{1'b0}};
            integrator_2   <= {INTERNAL_WIDTH{1'b0}};
            integrator_3   <= {INTERNAL_WIDTH{1'b0}};
            integrator_4   <= {INTERNAL_WIDTH{1'b0}};
            comb_delay_1   <= {INTERNAL_WIDTH{1'b0}};
            comb_delay_2   <= {INTERNAL_WIDTH{1'b0}};
            comb_delay_3   <= {INTERNAL_WIDTH{1'b0}};
            comb_delay_4   <= {INTERNAL_WIDTH{1'b0}};
            comb_pipe_1    <= {INTERNAL_WIDTH{1'b0}};
            comb_pipe_2    <= {INTERNAL_WIDTH{1'b0}};
            comb_pipe_3    <= {INTERNAL_WIDTH{1'b0}};
            comb_pipe_4    <= {INTERNAL_WIDTH{1'b0}};
            decimation_count <= 4'd0;
            comb_valid_pipe <= 5'd0;
            dout           <= {OUTPUT_WIDTH{1'b0}};
            dout_valid     <= 1'b0;
        end else begin
            dout_valid <= comb_valid_pipe[4];
            if (comb_valid_pipe[4])
                dout <= normalize_cic(comb_pipe_4);

            comb_valid_pipe <= {comb_valid_pipe[3:0], 1'b0};

            if (din_valid) begin
                integrator_1 <= integrator_1 + din_extended;
                integrator_2 <= integrator_2 + integrator_1;
                integrator_3 <= integrator_3 + integrator_2;
                integrator_4 <= integrator_4 + integrator_3;

                if (decimation_count == DECIMATION-1) begin
                    decimation_count <= 4'd0;
                    comb_pipe_1  <= integrator_4 - comb_delay_1;
                    comb_delay_1 <= integrator_4;
                    comb_valid_pipe[0] <= 1'b1;
                end else begin
                    decimation_count <= decimation_count + 1'b1;
                end
            end

            if (comb_valid_pipe[0]) begin
                comb_pipe_2  <= comb_pipe_1 - comb_delay_2;
                comb_delay_2 <= comb_pipe_1;
            end
            if (comb_valid_pipe[1]) begin
                comb_pipe_3  <= comb_pipe_2 - comb_delay_3;
                comb_delay_3 <= comb_pipe_2;
            end
            if (comb_valid_pipe[2]) begin
                comb_pipe_4  <= comb_pipe_3 - comb_delay_4;
                comb_delay_4 <= comb_pipe_3;
            end
        end
    end

endmodule
