`timescale 1ns / 1ps

// AX7020 simultaneous two-channel PL measurement pipeline.
//
// This module is instantiated as a Vivado Block Design Module Reference.
// It deliberately contains no AXI-Lite slave.  Control comes from an AXI GPIO
// cell in the block design, while measurement data is emitted as AXI Stream
// for an AXI DMA S2MM channel.
module ax7020_dual_pipeline (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ADC_CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ADC_CLK, FREQ_HZ 65000000, ASSOCIATED_BUSIF M_AXIS, ASSOCIATED_RESET adc_resetn" *)
    input  wire        adc_clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ADC_SAMPLE_CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ADC_SAMPLE_CLK, FREQ_HZ 65000000" *)
    input  wire        adc_sample_clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 ADC_RESETN RST" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ADC_RESETN, POLARITY ACTIVE_LOW" *)
    input  wire        adc_resetn,
    input  wire        adc_clk_locked,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CTRL_CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME CTRL_CLK, FREQ_HZ 100000000, ASSOCIATED_RESET ctrl_resetn" *)
    input  wire        ctrl_clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 CTRL_RESETN RST" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME CTRL_RESETN, POLARITY ACTIVE_LOW" *)
    input  wire        ctrl_resetn,

    input  wire [11:0] adc_ch1_data,
    input  wire [11:0] adc_ch2_data,
    input  wire        adc_ch1_otr,
    input  wire        adc_ch2_otr,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ADC_CH1_CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ADC_CH1_CLK, FREQ_HZ 65000000" *)
    output wire        adc_ch1_clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ADC_CH2_CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME ADC_CH2_CLK, FREQ_HZ 65000000" *)
    output wire        adc_ch2_clk,

    input  wire [2:0]  gpio_control,
    output wire [31:0] gpio_status,
    output reg         frame_done_irq,

    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME M_AXIS, TDATA_NUM_BYTES 4, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 65000000" *)
    output reg  [31:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TKEEP" *)
    output reg  [3:0]  m_axis_tkeep,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output reg         m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output reg         m_axis_tlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input  wire        m_axis_tready
);

    localparam integer FRAME_LENGTH       = 16384;
    localparam integer FFT_FIRST_BIN      = 1;
    localparam integer FFT_LAST_BIN       = 6650;
    localparam integer FFT_VALID_SLOTS    = 6650;
    localparam integer FFT_PADDED_SLOTS   = 6656;
    localparam integer HEADER_WORDS       = 16;
    localparam integer FFT_SIDE_WORDS     = FFT_PADDED_SLOTS * 2;
    localparam integer TRAILER_WORDS      = 16;
    localparam integer TIME_FIRST_WORD    = HEADER_WORDS;
    localparam integer FFT_POS_FIRST_WORD = TIME_FIRST_WORD + FRAME_LENGTH;
    localparam integer FFT_NEG_FIRST_WORD = FFT_POS_FIRST_WORD + FFT_SIDE_WORDS;
    localparam integer RAW_FIRST_WORD     = FFT_NEG_FIRST_WORD + FFT_SIDE_WORDS;
    localparam integer TRAILER_FIRST_WORD = RAW_FIRST_WORD + FRAME_LENGTH;
    localparam integer PACKET_WORDS       = TRAILER_FIRST_WORD + TRAILER_WORDS;

    localparam [3:0] STATE_IDLE       = 4'd0;
    localparam [3:0] STATE_HEADER     = 4'd1;
    localparam [3:0] STATE_CONFIG     = 4'd2;
    localparam [3:0] STATE_CAPTURE    = 4'd3;
    localparam [3:0] STATE_WAIT_FFT   = 4'd4;
    localparam [3:0] STATE_POS_PAD    = 4'd5;
    localparam [3:0] STATE_NEG_PAD    = 4'd6;
    localparam [3:0] STATE_TRAILER    = 4'd7;
    localparam [3:0] STATE_RAW        = 4'd8;

    // ---------------------------------------------------------------------
    // Resets and AXI-GPIO clock-domain crossing
    // ---------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [2:0] trigger_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] soft_reset_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] slow_mode_sync;
    wire trigger_event = trigger_sync[2] ^ trigger_sync[1];
    wire soft_reset_adc = soft_reset_sync[1];

    always @(posedge adc_clk) begin
        if (!adc_resetn) begin
            trigger_sync    <= 3'b000;
            soft_reset_sync <= 2'b00;
            slow_mode_sync  <= 2'b00;
        end else begin
            trigger_sync    <= {trigger_sync[1:0], gpio_control[0]};
            soft_reset_sync <= {soft_reset_sync[0], gpio_control[1]};
            slow_mode_sync  <= {slow_mode_sync[0], gpio_control[2]};
        end
    end

    wire pipeline_resetn = adc_resetn & ~soft_reset_adc;

    // ---------------------------------------------------------------------
    // Simultaneous AN9238 capture and matched 8:1 CIC/FIR paths
    // ---------------------------------------------------------------------
    wire adc_valid;
    wire signed [15:0] ch1_raw;
    wire signed [15:0] ch2_raw;
    wire ch1_overrange;
    wire ch2_overrange;
    wire [31:0] time_word;
    wire signed [23:0] ch1_filtered;
    wire signed [23:0] ch2_filtered;
    wire pair_valid;
    wire filter_flow_error;

    ax7020_dual_filter u_filter (
        .adc_clk(adc_clk), .adc_sample_clk(adc_sample_clk),
        .resetn(pipeline_resetn),
        .adc_ch1_data(adc_ch1_data), .adc_ch2_data(adc_ch2_data),
        .adc_ch1_otr(adc_ch1_otr), .adc_ch2_otr(adc_ch2_otr),
        .adc_ch1_clk(adc_ch1_clk), .adc_ch2_clk(adc_ch2_clk),
        .adc_valid(adc_valid),
        .ch1_raw(ch1_raw), .ch2_raw(ch2_raw),
        .ch1_overrange(ch1_overrange), .ch2_overrange(ch2_overrange),
        .time_word(time_word),
        .ch1_filtered(ch1_filtered), .ch2_filtered(ch2_filtered),
        .pair_valid(pair_valid), .flow_error(filter_flow_error)
    );

    // ---------------------------------------------------------------------
    // Streaming frame controller and quarter-wave periodic Hann ROM
    // ---------------------------------------------------------------------
    // A quarter-wave Q17 ROM serves both channels at each sample index.
    (* rom_style = "block" *)
    reg [17:0] hann_quarter_memory [0:4096];

    initial begin
        $readmemh("hann_q17_quarter_16384.mem", hann_quarter_memory);
    end

    function automatic [12:0] hann_address;
        input [13:0] index;
        begin
            case (index[13:12])
                2'b00: hann_address = {1'b0, index[11:0]};
                2'b01: hann_address = 14'h2000 - index;
                2'b10: hann_address = index - 14'h2000;
                default: hann_address = 15'h4000 - index;
            endcase
        end
    endfunction

    function automatic hann_complement;
        input [13:0] index;
        begin
            hann_complement = (index[13:12] == 2'b01) ||
                              (index[13:12] == 2'b10);
        end
    endfunction

    function automatic signed [17:0] saturate_fft18;
        input signed [24:0] value;
        begin
            if (value > 25'sd131071)
                saturate_fft18 = 18'sh1ffff;
            else if (value < -25'sd131072)
                saturate_fft18 = 18'sh20000;
            else
                saturate_fft18 = value[17:0];
        end
    endfunction

    reg [3:0]  state;
    reg [13:0] capture_index;
    reg [13:0] raw_capture_index;
    reg [13:0] raw_stream_index;
    reg        raw_capture_active;
    reg        raw_capture_complete;
    reg        raw_fetch_pending;
    reg [31:0] raw_read_data;
    (* ram_style = "block" *) reg [31:0] raw_capture_memory [0:FRAME_LENGTH-1];
    reg [3:0]  sample_skip;
    reg        slow_mode;
    reg [14:0] fft_output_index;
    reg [13:0] fft_stored_count;
    reg [15:0] frame_sequence;
    reg [4:0]  header_index;
    reg [4:0]  trailer_index;
    reg [4:0]  pad_word_index;

    reg signed [23:0] window_ch1_sample;
    reg signed [23:0] window_ch2_sample;
    reg [17:0] window_coefficient;
    reg        window_complement;
    reg        window_pending;
    reg        window_pending_last;
    reg signed [41:0] window_ch1_product;
    reg signed [41:0] window_ch2_product;
    reg        product_pending;
    reg        product_pending_last;
    wire signed [41:0] window_ch1_rounded =
        window_ch1_product[41] ? (window_ch1_product - 42'sd65536) :
                                 (window_ch1_product + 42'sd65536);
    wire signed [41:0] window_ch2_rounded =
        window_ch2_product[41] ? (window_ch2_product - 42'sd65536) :
                                 (window_ch2_product + 42'sd65536);
    wire signed [17:0] window_ch1_fft18 =
        saturate_fft18($signed(window_ch1_rounded) >>> 17);
    wire signed [17:0] window_ch2_fft18 =
        saturate_fft18($signed(window_ch2_rounded) >>> 17);

    reg [47:0] fft_pair_data;
    reg        fft_pair_pending;
    reg        fft_pair_imag;
    reg [31:0] ch1_overrange_count;
    reg [31:0] ch2_overrange_count;

    reg trigger_overrun;
    reg fir_flow_error;
    reg fft_event_error;
    reg fft_done_latched;
    reg fft_done_toggle;

    always @(posedge adc_clk) begin
        if (pipeline_resetn && raw_capture_active && adc_valid)
            raw_capture_memory[raw_capture_index] <= {ch2_raw, ch1_raw};
        if (pipeline_resetn && state == STATE_RAW && !raw_fetch_pending)
            raw_read_data <= raw_capture_memory[raw_stream_index];
    end

    // ---------------------------------------------------------------------
    // FFT AXI Stream interfaces
    // ---------------------------------------------------------------------
    reg [7:0] fft_config_tdata;
    reg       fft_config_tvalid;
    wire      fft_config_tready;
    reg [47:0] fft_input_tdata;
    reg        fft_input_tvalid;
    wire       fft_input_tready;
    reg        fft_input_tlast;
    wire [47:0] fft_output_tdata;
    wire        fft_output_tvalid;
    wire        fft_output_tready;
    wire        fft_output_tlast;
    wire [7:0]  fft_output_tuser;
    wire [7:0]  fft_status_tdata;
    wire        fft_status_tvalid;
    wire        fft_status_tready;
    wire        fft_event_frame_started;
    wire        fft_event_tlast_unexpected;
    wire        fft_event_tlast_missing;
    wire        fft_event_data_in_channel_halt;
    wire        fft_event_data_out_channel_halt;
    wire        fft_event_status_channel_halt;
    reg  [7:0]  fft_last_tuser;
    reg  [7:0]  fft_last_status;

    q3_fft_0 u_fft (
        .aclk                          (adc_clk),
        .aresetn                       (pipeline_resetn),
        .s_axis_config_tdata           (fft_config_tdata),
        .s_axis_config_tvalid          (fft_config_tvalid),
        .s_axis_config_tready          (fft_config_tready),
        .s_axis_data_tdata             (fft_input_tdata),
        .s_axis_data_tvalid            (fft_input_tvalid),
        .s_axis_data_tready            (fft_input_tready),
        .s_axis_data_tlast             (fft_input_tlast),
        .m_axis_data_tdata             (fft_output_tdata),
        .m_axis_data_tuser             (fft_output_tuser),
        .m_axis_data_tvalid            (fft_output_tvalid),
        .m_axis_data_tready            (fft_output_tready),
        .m_axis_data_tlast             (fft_output_tlast),
        .m_axis_status_tdata           (fft_status_tdata),
        .m_axis_status_tvalid          (fft_status_tvalid),
        .m_axis_status_tready          (fft_status_tready),
        .event_frame_started           (fft_event_frame_started),
        .event_tlast_unexpected        (fft_event_tlast_unexpected),
        .event_tlast_missing           (fft_event_tlast_missing),
        .event_status_channel_halt     (fft_event_status_channel_halt),
        .event_data_in_channel_halt    (fft_event_data_in_channel_halt),
        .event_data_out_channel_halt   (fft_event_data_out_channel_halt)
    );

    /*
     * The 8.125 MSPS FIR stream intentionally contains gaps relative to the
     * 65 MHz FFT clock, and selected FFT bins intentionally apply output
     * backpressure while their real/imaginary words are packetized.  Channel
     * halt events therefore describe legal non-realtime AXIS throttling, not
     * corrupted data.  Only TLAST mismatches are framing errors.
     */
    wire fft_protocol_error = fft_event_tlast_unexpected |
                              fft_event_tlast_missing;
    wire stream_can_advance = !m_axis_tvalid || m_axis_tready;
    wire fft_pos_bin_selected = (fft_output_index >= FFT_FIRST_BIN) &&
                                (fft_output_index <= FFT_LAST_BIN);
    wire fft_neg_bin_selected =
        (fft_output_index >= FRAME_LENGTH-FFT_LAST_BIN) &&
        (fft_output_index <= FRAME_LENGTH-FFT_FIRST_BIN);
    wire fft_bin_selected = fft_pos_bin_selected || fft_neg_bin_selected;
    assign fft_output_tready = (state == STATE_WAIT_FFT) &&
                               !fft_pair_pending;
    assign fft_status_tready = 1'b1;

    // ---------------------------------------------------------------------
    // Packet word generation
    // ---------------------------------------------------------------------
    function automatic [31:0] header_word;
        input [3:0] index;
        begin
            case (index)
                4'd0:  header_word = 32'h41374432; // "A7D2"
                4'd1:  header_word = 32'h0003_0002; // protocol 3.2
                4'd2:  header_word = {16'd0, frame_sequence};
                4'd3:  header_word = PACKET_WORDS;
                4'd4:  header_word = FRAME_LENGTH;
                4'd5:  header_word = FRAME_LENGTH;
                4'd6:  header_word = 32'd65000000;
                4'd7:  header_word = slow_mode ? 32'd507812 : 32'd8125000;
                4'd8:  header_word = 32'd12;
                4'd9:  header_word = FFT_FIRST_BIN;
                4'd10: header_word = FFT_LAST_BIN;
                4'd11: header_word = FFT_VALID_SLOTS;
                4'd12: header_word = FFT_PADDED_SLOTS;
                4'd13: header_word = 32'd2;
                4'd14: header_word = 32'd1;
                4'd15: header_word = FRAME_LENGTH;
                default: header_word = 32'd0;
            endcase
        end
    endfunction

    wire [7:0] final_status = {
        3'b000,
        fft_event_error,
        fir_flow_error,
        trigger_overrun,
        1'b0,
        1'b1
    };

    function automatic [31:0] trailer_word;
        input [3:0] index;
        begin
            case (index)
                4'd0:  trailer_word = 32'h454E4437; // "END7"
                4'd1:  trailer_word = {16'd0, frame_sequence};
                4'd2:  trailer_word = {24'd0, final_status};
                4'd3:  trailer_word = {24'd0, fft_last_tuser};
                4'd4:  trailer_word = {29'd0,
                                      fft_event_data_out_channel_halt,
                                      fft_event_tlast_missing,
                                      fft_event_tlast_unexpected};
                4'd5:  trailer_word = {24'd0, fft_last_status};
                4'd6:  trailer_word = ch1_overrange_count;
                4'd7:  trailer_word = ch2_overrange_count;
                4'd8:  trailer_word = {17'd0, fft_output_index};
                4'd9:  trailer_word = {18'd0, fft_stored_count};
                4'd10: trailer_word = FFT_PADDED_SLOTS-FFT_VALID_SLOTS;
                4'd11: trailer_word = raw_capture_complete ?
                                       FRAME_LENGTH : raw_capture_index;
                4'd15: trailer_word = PACKET_WORDS;
                default: trailer_word = 32'd0;
            endcase
        end
    endfunction

    // ---------------------------------------------------------------------
    // ADC-domain controller
    // ---------------------------------------------------------------------
    always @(posedge adc_clk) begin
        if (!pipeline_resetn) begin
            state                <= STATE_IDLE;
            capture_index        <= 14'd0;
            raw_capture_index    <= 14'd0;
            raw_stream_index     <= 14'd0;
            raw_capture_active   <= 1'b0;
            raw_capture_complete <= 1'b0;
            raw_fetch_pending    <= 1'b0;
            sample_skip          <= 4'd0;
            slow_mode            <= 1'b0;
            fft_output_index     <= 15'd0;
            fft_stored_count     <= 14'd0;
            frame_sequence       <= 16'd0;
            header_index         <= 5'd0;
            trailer_index        <= 5'd0;
            pad_word_index       <= 5'd0;
            window_ch1_sample    <= 24'sd0;
            window_ch2_sample    <= 24'sd0;
            window_coefficient   <= 18'd0;
            window_complement    <= 1'b0;
            window_pending       <= 1'b0;
            window_pending_last  <= 1'b0;
            window_ch1_product   <= 42'sd0;
            window_ch2_product   <= 42'sd0;
            product_pending      <= 1'b0;
            product_pending_last <= 1'b0;
            fft_pair_data        <= 48'd0;
            fft_pair_pending     <= 1'b0;
            fft_pair_imag        <= 1'b0;
            ch1_overrange_count  <= 32'd0;
            ch2_overrange_count  <= 32'd0;
            trigger_overrun      <= 1'b0;
            fir_flow_error       <= 1'b0;
            fft_event_error      <= 1'b0;
            fft_done_latched     <= 1'b0;
            fft_done_toggle      <= 1'b0;
            fft_last_tuser       <= 8'd0;
            fft_last_status      <= 8'd0;
            fft_config_tdata     <= 8'h01;
            fft_config_tvalid    <= 1'b0;
            fft_input_tdata      <= 48'd0;
            fft_input_tvalid     <= 1'b0;
            fft_input_tlast      <= 1'b0;
            m_axis_tdata         <= 32'd0;
            m_axis_tkeep         <= 4'hF;
            m_axis_tvalid        <= 1'b0;
            m_axis_tlast         <= 1'b0;
        end else begin
            if (raw_capture_active && adc_valid) begin
                if (raw_capture_index == FRAME_LENGTH-1) begin
                    raw_capture_active <= 1'b0;
                    raw_capture_complete <= 1'b1;
                end else begin
                    raw_capture_index <= raw_capture_index + 1'b1;
                end
            end
            // Sticky error capture, cleared by a new accepted trigger.
            if (fft_protocol_error)
                fft_event_error <= 1'b1;
            if (fft_status_tvalid)
                fft_last_status <= fft_status_tdata;
            if (fft_config_tvalid && fft_config_tready)
                fft_config_tvalid <= 1'b0;

            // Hold FFT input until it is accepted.
            if (fft_input_tvalid && fft_input_tready) begin
                fft_input_tvalid <= 1'b0;
                fft_input_tlast  <= 1'b0;
            end

            // The output register is held until downstream accepts it.
            if (m_axis_tvalid && m_axis_tready) begin
                m_axis_tvalid <= 1'b0;
                m_axis_tlast  <= 1'b0;
            end

            // Quarter-Hann ROM read, quadrant reconstruction and multiply.
            if (window_pending) begin
                window_ch1_product   <= $signed(window_ch1_sample) *
                    $signed({1'b0, window_complement ?
                        (18'd131071-window_coefficient) :
                        window_coefficient});
                window_ch2_product   <= $signed(window_ch2_sample) *
                    $signed({1'b0, window_complement ?
                        (18'd131071-window_coefficient) :
                        window_coefficient});
                product_pending      <= 1'b1;
                product_pending_last <= window_pending_last;
                window_pending       <= 1'b0;
            end
            if (product_pending && (!fft_input_tvalid || fft_input_tready)) begin
                    // Symmetric rounding to Q0 followed by saturation to the
                    // 18-bit FFT input.  The 24-bit raw FIR word is unaffected.
                    fft_input_tdata <= {
                        {{6{window_ch2_fft18[17]}}, window_ch2_fft18},
                        {{6{window_ch1_fft18[17]}}, window_ch1_fft18}
                    };
                    fft_input_tvalid <= 1'b1;
                    fft_input_tlast  <= product_pending_last;
                    product_pending  <= 1'b0;
            end

            // Unselected bins are consumed immediately. Selected complex
            // values are held while their real and imaginary words stream.
            if (fft_output_tvalid && fft_output_tready) begin
                if (fft_bin_selected) begin
                    fft_pair_data    <= fft_output_tdata;
                    fft_pair_pending <= 1'b1;
                    fft_pair_imag    <= 1'b0;
                    fft_stored_count <= fft_stored_count + 1'b1;
                end
                fft_output_index <= fft_output_index + 1'b1;
                if (fft_output_tlast) begin
                    fft_last_tuser   <= fft_output_tuser;
                    fft_done_latched <= 1'b1;
                    fft_done_toggle  <= ~fft_done_toggle;
                end
            end

            if (trigger_event) begin
                if (state == STATE_IDLE) begin
                    frame_sequence       <= frame_sequence + 1'b1;
                    capture_index        <= 14'd0;
                    raw_capture_index    <= 14'd0;
                    raw_stream_index     <= 14'd0;
                    raw_capture_active   <= 1'b1;
                    raw_capture_complete <= 1'b0;
                    raw_fetch_pending    <= 1'b0;
                    sample_skip          <= 4'd0;
                    slow_mode            <= slow_mode_sync[1];
                    fft_output_index     <= 15'd0;
                    fft_stored_count     <= 14'd0;
                    ch1_overrange_count  <= 32'd0;
                    ch2_overrange_count  <= 32'd0;
                    trigger_overrun      <= 1'b0;
                    fir_flow_error       <= 1'b0;
                    fft_event_error      <= 1'b0;
                    fft_done_latched     <= 1'b0;
                    window_pending       <= 1'b0;
                    product_pending      <= 1'b0;
                    fft_input_tvalid     <= 1'b0;
                    fft_config_tvalid    <= 1'b1;
                    fft_pair_pending     <= 1'b0;
                    header_index         <= 5'd0;
                    state                <= STATE_HEADER;
                end else begin
                    trigger_overrun <= 1'b1;
                end
            end

            case (state)
                STATE_IDLE: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                end

                STATE_HEADER: begin
                    if ((header_index < HEADER_WORDS) &&
                        stream_can_advance) begin
                        m_axis_tdata  <= header_word(header_index[3:0]);
                        m_axis_tkeep  <= 4'hf;
                        m_axis_tvalid <= 1'b1;
                        if (header_index == HEADER_WORDS-1) begin
                            header_index <= HEADER_WORDS;
                        end else begin
                            header_index <= header_index + 1'b1;
                        end
                    end else if ((header_index == HEADER_WORDS) &&
                                 stream_can_advance) begin
                        header_index <= 5'd0;
                        state <= fft_config_tvalid ?
                                 STATE_CONFIG : STATE_CAPTURE;
                    end
                end

                STATE_CONFIG: begin
                    if (!fft_config_tvalid)
                        state             <= STATE_CAPTURE;
                end

                STATE_CAPTURE: begin
                    if (adc_valid && ch1_overrange)
                        ch1_overrange_count <= ch1_overrange_count + 1'b1;
                    if (adc_valid && ch2_overrange)
                        ch2_overrange_count <= ch2_overrange_count + 1'b1;
                    if (filter_flow_error)
                        fir_flow_error <= 1'b1;
                    if (pair_valid) begin
                        sample_skip <= sample_skip + 1'b1;
                    end
                    if (pair_valid && (!slow_mode || sample_skip == 4'd0)) begin
                        if (window_pending || product_pending ||
                            (fft_input_tvalid && !fft_input_tready) ||
                            !stream_can_advance) begin
                            fir_flow_error <= 1'b1;
                        end else begin
                            m_axis_tdata <= time_word;
                            m_axis_tkeep <= 4'hf;
                            m_axis_tvalid <= 1'b1;
                            window_ch1_sample   <= ch1_filtered;
                            window_ch2_sample   <= ch2_filtered;
                            window_coefficient  <=
                                hann_quarter_memory[
                                    hann_address(capture_index)];
                            window_complement   <=
                                hann_complement(capture_index);
                            window_pending      <= 1'b1;
                            window_pending_last <= (capture_index ==
                                                    FRAME_LENGTH-1);
                            if (capture_index == FRAME_LENGTH-1) begin
                                state <= STATE_WAIT_FFT;
                            end else begin
                                capture_index <= capture_index + 1'b1;
                            end
                        end
                    end
                end

                STATE_WAIT_FFT: begin
                    if (fft_pair_pending && stream_can_advance) begin
                        m_axis_tkeep  <= 4'hf;
                        m_axis_tvalid <= 1'b1;
                        if (!fft_pair_imag) begin
                            m_axis_tdata <=
                                {{14{fft_pair_data[17]}},
                                  fft_pair_data[17:0]};
                            fft_pair_imag <= 1'b1;
                        end else begin
                            m_axis_tdata <=
                                {{14{fft_pair_data[41]}},
                                  fft_pair_data[41:24]};
                            fft_pair_imag    <= 1'b0;
                            fft_pair_pending <= 1'b0;
                        end
                    end else if (fft_output_index == FFT_LAST_BIN+1 &&
                                 !fft_pair_pending && stream_can_advance) begin
                        pad_word_index <= 5'd0;
                        state <= STATE_POS_PAD;
                    end else if (fft_done_latched &&
                                 !fft_pair_pending && stream_can_advance) begin
                        pad_word_index <= 5'd0;
                        state <= STATE_NEG_PAD;
                    end
                end

                STATE_POS_PAD, STATE_NEG_PAD: begin
                    if (stream_can_advance) begin
                        m_axis_tdata  <= 32'd0;
                        m_axis_tkeep  <= 4'hf;
                        m_axis_tvalid <= 1'b1;
                        if (pad_word_index ==
                            ((FFT_PADDED_SLOTS-FFT_VALID_SLOTS)*2)-1) begin
                            pad_word_index <= 5'd0;
                            if (state == STATE_POS_PAD)
                                state <= STATE_WAIT_FFT;
                            else begin
                                raw_stream_index <= 14'd0;
                                raw_fetch_pending <= 1'b0;
                                state <= STATE_RAW;
                            end
                        end else begin
                            pad_word_index <= pad_word_index + 1'b1;
                        end
                    end
                end

                STATE_RAW: begin
                    if (!raw_fetch_pending) begin
                        raw_fetch_pending <= 1'b1;
                    end else if (stream_can_advance) begin
                        m_axis_tdata <= raw_read_data;
                        m_axis_tkeep <= 4'hf;
                        m_axis_tvalid <= 1'b1;
                        raw_fetch_pending <= 1'b0;
                        if (raw_stream_index == FRAME_LENGTH-1) begin
                            trailer_index <= 5'd0;
                            state <= STATE_TRAILER;
                        end else begin
                            raw_stream_index <= raw_stream_index + 1'b1;
                        end
                    end
                end

                STATE_TRAILER: begin
                    if ((trailer_index < TRAILER_WORDS) &&
                        stream_can_advance) begin
                        m_axis_tdata  <= trailer_word(trailer_index[3:0]);
                        m_axis_tkeep  <= 4'hf;
                        m_axis_tvalid <= 1'b1;
                        m_axis_tlast  <= (trailer_index ==
                                          TRAILER_WORDS-1);
                        if (trailer_index == TRAILER_WORDS-1) begin
                            trailer_index <= TRAILER_WORDS;
                        end else begin
                            trailer_index <= trailer_index + 1'b1;
                        end
                    end else if ((trailer_index == TRAILER_WORDS) &&
                                 stream_can_advance) begin
                        trailer_index <= 5'd0;
                        fft_done_latched <= 1'b0;
                        state <= STATE_IDLE;
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

    // ---------------------------------------------------------------------
    // Status and completion pulse into the 100 MHz AXI/control domain
    // ---------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [2:0] done_toggle_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] locked_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] done_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] overrun_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] fir_error_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] fft_error_sync;
    wire busy_ctrl;
    wire streaming_ctrl;
    wire [15:0] frame_sequence_ctrl;

    xpm_cdc_single #(
        .DEST_SYNC_FF  (2),
        .INIT_SYNC_FF  (1),
        .SIM_ASSERT_CHK(0),
        .SRC_INPUT_REG (1)
    ) u_busy_cdc (
        .src_clk (adc_clk),
        .src_in  (state != STATE_IDLE),
        .dest_clk(ctrl_clk),
        .dest_out(busy_ctrl)
    );

    xpm_cdc_single #(
        .DEST_SYNC_FF  (2),
        .INIT_SYNC_FF  (1),
        .SIM_ASSERT_CHK(0),
        .SRC_INPUT_REG (1)
    ) u_streaming_cdc (
        .src_clk (adc_clk),
        .src_in  ((state == STATE_HEADER) ||
                  (state == STATE_CAPTURE) ||
                  (state == STATE_WAIT_FFT) ||
                  (state == STATE_POS_PAD) ||
                  (state == STATE_NEG_PAD) ||
                  (state == STATE_RAW) ||
                  (state == STATE_TRAILER)),
        .dest_clk(ctrl_clk),
        .dest_out(streaming_ctrl)
    );

    // The sequence counter changes only when a new multi-millisecond frame
    // starts, so it remains stable for far longer than this two-stage array
    // synchronizer latency.
    xpm_cdc_array_single #(
        .DEST_SYNC_FF  (2),
        .INIT_SYNC_FF  (1),
        .SIM_ASSERT_CHK(0),
        .SRC_INPUT_REG (1),
        .WIDTH         (16)
    ) u_sequence_cdc (
        .src_clk (adc_clk),
        .src_in  (frame_sequence),
        .dest_clk(ctrl_clk),
        .dest_out(frame_sequence_ctrl)
    );

    always @(posedge ctrl_clk or negedge ctrl_resetn) begin
        if (!ctrl_resetn) begin
            done_toggle_sync     <= 3'b000;
            locked_sync          <= 2'b00;
            done_sync            <= 2'b00;
            overrun_sync         <= 2'b00;
            fir_error_sync       <= 2'b00;
            fft_error_sync       <= 2'b00;
            frame_done_irq       <= 1'b0;
        end else begin
            done_toggle_sync <= {done_toggle_sync[1:0], fft_done_toggle};
            locked_sync      <= {locked_sync[0], adc_clk_locked};
            done_sync        <= {done_sync[0], fft_done_latched};
            overrun_sync     <= {overrun_sync[0], trigger_overrun};
            fir_error_sync   <= {fir_error_sync[0], fir_flow_error};
            fft_error_sync   <= {fft_error_sync[0], fft_event_error};
            frame_done_irq <= done_toggle_sync[2] ^ done_toggle_sync[1];
        end
    end

    assign gpio_status = {
        frame_sequence_ctrl,
        8'd0,
        overrun_sync[1],
        fft_error_sync[1],
        fir_error_sync[1],
        1'b0,
        streaming_ctrl,
        done_sync[1],
        busy_ctrl,
        locked_sync[1]
    };

endmodule
