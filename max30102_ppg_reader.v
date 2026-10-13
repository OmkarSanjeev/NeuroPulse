// MAX30102 IR PPG reader for DE2-115 / 50 MHz FPGA clock.
// Provides raw IR samples and a small FPGA-side beat/BPM estimate.
// This is an academic/demo implementation, not a medical instrument.
// Beat-detection port is based on SparkFun's MAX3010x heartRate.cpp, which
// carries this notice: Copyright (C) 2016 Maxim Integrated Products, Inc.
// Permission is granted to use, copy, modify, and distribute this software
// provided this notice is included. The software is provided without warranty.
`timescale 1ns/1ps

module max30102_ppg_reader #(
    parameter integer TICKS = 31,             // 50 MHz / (4*TICKS) ~= 403 kHz I2C
    parameter integer POLL_GAP_TICKS = 8065,  // about 20 ms at TICKS=31 (~50 Hz)
    parameter integer MIN_PULSE_AMP = 20,
    parameter integer MAX_PULSE_AMP = 1000,
    parameter integer SAMPLE_RATE_HZ = 100    // 400 Hz sensor rate averaged 4:1
)(
    input  wire        clk,
    input  wire        rst_n,
    inout  wire        sda,
    output reg         scl,

    output reg  [17:0] ir_sample,
    output reg         sample_valid,
    output reg         beat_detected,
    output reg  [7:0]  bpm,
    output reg         bpm_valid,
    output reg         sensor_ready,
    output reg         ack_error
);

    localparam [7:0] MAX_W = 8'hAE, MAX_R = 8'hAF;

    // MAX30102 SDA is open drain: drive zero or release the bus.
    reg sda_oe, sda_out;
    assign sda = (sda_oe && !sda_out) ? 1'b0 : 1'bz;

    localparam [2:0] ST_START = 3'd0,
                     ST_TX    = 3'd1,
                     ST_RSTRT = 3'd2,
                     ST_RX    = 3'd3,
                     ST_STOP  = 3'd4,
                     ST_DONE  = 3'd5,
                     ST_GAP   = 3'd6;

    reg [2:0] state;
    reg [1:0] phase;
    reg [31:0] tick_count, gap_count;
    reg [3:0] bit_count, byte_count, buffer_index;
    reg [7:0] tx_byte, rx_shift;
    reg [7:0] reg_buf, data_buf;
    reg [7:0] read_buffer [0:5];
    reg [1:0] tx_kind;       // 0=write address, 1=register, 2=read address, 3=data
    reg       write_mode;
    reg       transaction_nack;
    reg       init_complete;
    reg       read_pending;
    reg [3:0] init_index;
    reg [7:0] init_reg, init_data;

    // Configuration follows the provided SparkFun example, adapted to the
    // MAX30102's two optical channels: average 4, red+IR, 400 Hz, 411 us,
    // 4096 nA range, red current 0x0A, IR current 0x1F.
    always @(*) begin
        case (init_index)
            4'd0: begin init_reg = 8'h09; init_data = 8'h00; end // stop while configuring
            4'd1: begin init_reg = 8'h04; init_data = 8'h00; end // FIFO write pointer
            4'd2: begin init_reg = 8'h05; init_data = 8'h00; end // FIFO overflow counter
            4'd3: begin init_reg = 8'h06; init_data = 8'h00; end // FIFO read pointer
            4'd4: begin init_reg = 8'h08; init_data = 8'h5F; end // average 4, rollover
            4'd5: begin init_reg = 8'h0A; init_data = 8'h2F; end // 4096 range, 400 Hz, 411 us
            4'd6: begin init_reg = 8'h0C; init_data = 8'h0A; end // red LED current
            4'd7: begin init_reg = 8'h0D; init_data = 8'h1F; end // IR LED current
            default: begin init_reg = 8'h09; init_data = 8'h03; end // SpO2: red + IR
        endcase
    end

    wire [17:0] fifo_red = {read_buffer[0], read_buffer[1], read_buffer[2][7:6]};
    wire [17:0] fifo_ir  = {read_buffer[3], read_buffer[4], read_buffer[5][7:6]};

    // SparkFun PBA stages: Q15 DC estimator, 12-tap FIR, zero-crossing peaks.
    reg signed [35:0] dc_accum;
    reg signed [15:0] fir_buffer [0:31];
    reg [4:0] fir_offset;
    reg signed [15:0] ac_previous, ac_signal_min, ac_signal_max;
    reg positive_edge, negative_edge;
    reg [15:0] beat_interval;
    reg [7:0] rates [0:3];
    reg [1:0] rate_index;
    reg [10:0] rate_sum;

    wire signed [35:0] ir_q15 = $signed({1'b0, fifo_ir, 15'b0});
    wire signed [35:0] dc_error = ir_q15 - dc_accum;
    wire signed [35:0] dc_level = dc_accum >>> 15;
    wire signed [18:0] fir_input_wide = $signed({1'b0, fifo_ir}) - dc_level;
    wire signed [15:0] fir_input = fir_input_wide[15:0];
    wire signed [31:0] fir_scaled = fir_sum >>> 15;
    wire signed [15:0] fir_current = fir_scaled[15:0];
    wire signed [16:0] pulse_amplitude_signed =
        $signed({ac_signal_max[15], ac_signal_max}) - $signed({ac_signal_min[15], ac_signal_min});
    wire [16:0] pulse_amplitude = pulse_amplitude_signed;
    wire [31:0] bpm_numerator = 32'd60 * SAMPLE_RATE_HZ;
    wire [31:0] bpm_instant = (beat_interval != 0) ? (bpm_numerator / beat_interval) : 0;
    wire [10:0] next_rate_sum = rate_sum - rates[rate_index] + bpm_instant[7:0];

    function signed [15:0] fir_coefficient;
        input integer index;
        begin
            case (index)
                0: fir_coefficient = 16'd172;
                1: fir_coefficient = 16'd321;
                2: fir_coefficient = 16'd579;
                3: fir_coefficient = 16'd927;
                4: fir_coefficient = 16'd1360;
                5: fir_coefficient = 16'd1858;
                6: fir_coefficient = 16'd2390;
                7: fir_coefficient = 16'd2916;
                8: fir_coefficient = 16'd3391;
                9: fir_coefficient = 16'd3768;
                10: fir_coefficient = 16'd4012;
                default: fir_coefficient = 16'd4096;
            endcase
        end
    endfunction

    function signed [31:0] fir_multiply;
        input signed [15:0] a;
        input signed [15:0] b;
        reg signed [31:0] a_ext, b_ext;
        begin
            a_ext = {{16{a[15]}}, a};
            b_ext = {{16{b[15]}}, b};
            fir_multiply = a_ext * b_ext;
        end
    endfunction

    function signed [15:0] fir_sample_at;
        input [4:0] index;
        begin
            if (index == fir_offset)
                fir_sample_at = fir_input;
            else
                fir_sample_at = fir_buffer[index];
        end
    endfunction

    integer i, j;
    reg signed [31:0] fir_sum;
    always @(*) begin
        fir_sum = fir_multiply(fir_coefficient(11), fir_sample_at(fir_offset - 5'd11));
        for (j = 0; j < 11; j = j + 1)
            fir_sum = fir_sum + fir_multiply(fir_coefficient(j),
                fir_sample_at(fir_offset - j) + fir_sample_at(fir_offset - 5'd22 + j));
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl <= 1'b1;
            sda_oe <= 1'b1;
            sda_out <= 1'b1;
            state <= ST_DONE;
            phase <= 2'd0;
            tick_count <= 0;
            gap_count <= 0;
            bit_count <= 0;
            byte_count <= 0;
            buffer_index <= 0;
            tx_byte <= 0;
            reg_buf <= 0;
            data_buf <= 0;
            rx_shift <= 0;
            tx_kind <= 0;
            write_mode <= 1'b0;
            transaction_nack <= 1'b0;
            init_complete <= 1'b0;
            read_pending <= 1'b0;
            init_index <= 0;
            ir_sample <= 0;
            sample_valid <= 1'b0;
            beat_detected <= 1'b0;
            bpm <= 0;
            bpm_valid <= 1'b0;
            sensor_ready <= 1'b0;
            ack_error <= 1'b0;
            dc_accum <= 0;
            ac_previous <= 0;
            ac_signal_max <= 0;
            ac_signal_min <= 0;
            fir_offset <= 0;
            positive_edge <= 1'b0;
            negative_edge <= 1'b0;
            beat_interval <= 0;
            rate_index <= 0;
            rate_sum <= 0;
            for (i = 0; i < 32; i = i + 1)
                fir_buffer[i] <= 0;
            for (i = 0; i < 4; i = i + 1)
                rates[i] <= 0;
        end else begin
            sample_valid <= 1'b0;
            beat_detected <= 1'b0;
            bpm_valid <= 1'b0;

            if (tick_count < TICKS-1) begin
                tick_count <= tick_count + 1'b1;
            end else begin
                tick_count <= 0;
                phase <= phase + 1'b1;

                case (state)
                    ST_START: begin
                        case (phase)
                            2'd0: begin sda_oe <= 1'b1; sda_out <= 1'b1; scl <= 1'b1; end
                            2'd2: sda_out <= 1'b0;
                            2'd3: begin scl <= 1'b0; state <= ST_TX; bit_count <= 0; end
                            default: ;
                        endcase
                    end

                    ST_TX: begin
                        case (phase)
                            2'd0: begin
                                scl <= 1'b0;
                                if (bit_count < 8) begin sda_oe <= 1'b1; sda_out <= tx_byte[7-bit_count]; end
                                else sda_oe <= 1'b0;
                            end
                            2'd2: scl <= 1'b1;
                            2'd3: begin
                                if (bit_count == 8) begin
                                    bit_count <= 0;
                                    if (sda == 1'b1) begin ack_error <= 1'b1; transaction_nack <= 1'b1; end
                                    case (tx_kind)
                                        2'd0: begin tx_byte <= reg_buf; tx_kind <= 2'd1; end
                                        2'd1: begin
                                            if (write_mode) begin tx_byte <= data_buf; tx_kind <= 2'd3; end
                                            else state <= ST_RSTRT;
                                        end
                                        2'd2: begin state <= ST_RX; buffer_index <= 0; end
                                        default: state <= ST_STOP;
                                    endcase
                                end else bit_count <= bit_count + 1'b1;
                            end
                            default: ;
                        endcase
                    end

                    ST_RSTRT: begin
                        case (phase)
                            2'd0: begin sda_oe <= 1'b1; sda_out <= 1'b1; scl <= 1'b0; end
                            2'd1: scl <= 1'b1;
                            2'd2: sda_out <= 1'b0;
                            2'd3: begin
                                scl <= 1'b0;
                                tx_byte <= MAX_R;
                                tx_kind <= 2'd2;
                                state <= ST_TX;
                                bit_count <= 0;
                            end
                            default: ;
                        endcase
                    end

                    ST_RX: begin
                        case (phase)
                            2'd0: begin
                                scl <= 1'b0;
                                if (bit_count < 8) sda_oe <= 1'b0;
                                else begin sda_oe <= 1'b1; sda_out <= (buffer_index == byte_count-1); end
                            end
                            2'd2: scl <= 1'b1;
                            2'd3: begin
                                if (bit_count < 8) begin
                                    rx_shift <= {rx_shift[6:0], sda};
                                    bit_count <= bit_count + 1'b1;
                                end else begin
                                    bit_count <= 0;
                                    read_buffer[buffer_index] <= rx_shift;
                                    if (buffer_index == byte_count-1) state <= ST_STOP;
                                    else buffer_index <= buffer_index + 1'b1;
                                end
                            end
                            default: ;
                        endcase
                    end

                    ST_STOP: begin
                        case (phase)
                            2'd0: begin scl <= 1'b0; sda_oe <= 1'b1; sda_out <= 1'b0; end
                            2'd1: scl <= 1'b1;
                            2'd2: sda_out <= 1'b1;
                            2'd3: state <= ST_DONE;
                            default: ;
                        endcase
                    end

                    ST_DONE: begin
                        if (phase == 2'd0) begin
                            if (init_index < 4'd9) begin
                                tx_byte <= MAX_W;
                                reg_buf <= init_reg;
                                data_buf <= init_data;
                                write_mode <= 1'b1;
                                tx_kind <= 2'd0;
                                transaction_nack <= 1'b0;
                                init_index <= init_index + 1'b1;
                                state <= ST_START;
                            end else if (!init_complete) begin
                                init_complete <= 1'b1;
                                sensor_ready <= !ack_error && !transaction_nack;
                                gap_count <= POLL_GAP_TICKS;
                                state <= ST_GAP;
                            end else if (read_pending) begin
                                read_pending <= 1'b0;
                                if (!transaction_nack) begin
                                    ir_sample <= fifo_ir;
                                    sample_valid <= 1'b1;

                                    // Match the Arduino PBA DC estimator and 12-tap FIR.
                                    dc_accum <= dc_accum + (dc_error >>> 4);
                                    fir_buffer[fir_offset] <= fir_input;
                                    fir_offset <= fir_offset + 1'b1;
                                    ac_previous <= fir_current;
                                    if (beat_interval != 16'hFFFF) beat_interval <= beat_interval + 1'b1;

                                    if ((ac_previous < 0) && (fir_current >= 0)) begin
                                        if ((pulse_amplitude > MIN_PULSE_AMP) && (pulse_amplitude < MAX_PULSE_AMP) &&
                                            (bpm_instant > 20) && (bpm_instant < 255)) begin
                                            beat_detected <= 1'b1;
                                            bpm <= next_rate_sum[9:2];
                                            bpm_valid <= 1'b1;
                                            rate_sum <= next_rate_sum;
                                            rates[rate_index] <= bpm_instant[7:0];
                                            rate_index <= rate_index + 1'b1;
                                            beat_interval <= 0;
                                        end
                                        ac_signal_max <= 0;
                                        positive_edge <= 1'b1;
                                        negative_edge <= 1'b0;
                                    end else if ((ac_previous > 0) && (fir_current <= 0)) begin
                                        positive_edge <= 1'b0;
                                        negative_edge <= 1'b1;
                                        ac_signal_min <= 0;
                                    end else if (positive_edge && (fir_current > ac_previous)) begin
                                        ac_signal_max <= fir_current;
                                    end else if (negative_edge && (fir_current < ac_previous)) begin
                                        ac_signal_min <= fir_current;
                                    end
                                end
                                gap_count <= POLL_GAP_TICKS;
                                state <= ST_GAP;
                            end
                        end
                    end

                    ST_GAP: begin
                        if (phase == 2'd0) begin
                            if (gap_count != 0) gap_count <= gap_count - 1'b1;
                            else begin
                                write_mode <= 1'b0;
                                sda_oe <= 1'b1;
                                sda_out <= 1'b1;
                                scl <= 1'b1;
                                tx_byte <= MAX_W;
                                reg_buf <= 8'h07; // FIFO data register
                                byte_count <= 4'd6; // red (3 bytes) then IR (3 bytes)
                                tx_kind <= 2'd0;
                                transaction_nack <= 1'b0;
                                read_pending <= 1'b1;
                                state <= ST_START;
                            end
                        end
                    end

                    default: state <= ST_DONE;
                endcase
            end
        end
    end
endmodule
