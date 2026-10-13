`timescale 1ns/1ps

// Standalone DE2-115 top-level for checking a MAX30102 without the full DSP.
// HEX0-2 show estimated BPM; HEX3-5 show the upper 12 bits of raw IR in hex.
// LEDG0=sensor initialized, LEDG1=sample activity, LEDG2=I2C NACK/error.
module max30102_sanity_top (
    input  wire       CLOCK_50,
    input  wire [3:0] KEY,
    inout  wire       I2C_SDA,
    output wire       I2C_SCL,
    output wire [8:0] LEDG,
    output wire [6:0] HEX0,
    output wire [6:0] HEX1,
    output wire [6:0] HEX2,
    output wire [6:0] HEX3,
    output wire [6:0] HEX4,
    output wire [6:0] HEX5
);
    wire rst_n = KEY[0];
    wire [17:0] ir_sample;
    wire sample_valid, beat_detected, bpm_valid, sensor_ready, ack_error;
    wire [7:0] bpm;

    max30102_ppg_reader u_ppg (
        .clk(CLOCK_50),
        .rst_n(rst_n),
        .sda(I2C_SDA),
        .scl(I2C_SCL),
        .ir_sample(ir_sample),
        .sample_valid(sample_valid),
        .beat_detected(beat_detected),
        .bpm(bpm),
        .bpm_valid(bpm_valid),
        .sensor_ready(sensor_ready),
        .ack_error(ack_error)
    );

    // Toggle at 1 Hz when samples arrive, making acquisition visible by eye.
    reg [5:0] sample_divider;
    reg sample_activity;
    always @(posedge CLOCK_50 or negedge rst_n) begin
        if (!rst_n) begin
            sample_divider <= 0;
            sample_activity <= 0;
        end else if (sample_valid) begin
            if (sample_divider == 6'd49) begin
                sample_divider <= 0;
                sample_activity <= ~sample_activity;
            end else begin
                sample_divider <= sample_divider + 1'b1;
            end
        end
    end

    assign LEDG[0] = sensor_ready;
    assign LEDG[1] = sample_activity;
    assign LEDG[2] = ack_error;
    assign LEDG[8:3] = 6'b0;

    function [6:0] seven_segment;
        input [3:0] value;
        begin
            // Active-low segments, ordered HEX[6:0] = g..a.
            case (value)
                4'h0: seven_segment = 7'b1000000;
                4'h1: seven_segment = 7'b1111001;
                4'h2: seven_segment = 7'b0100100;
                4'h3: seven_segment = 7'b0110000;
                4'h4: seven_segment = 7'b0011001;
                4'h5: seven_segment = 7'b0010010;
                4'h6: seven_segment = 7'b0000010;
                4'h7: seven_segment = 7'b1111000;
                4'h8: seven_segment = 7'b0000000;
                4'h9: seven_segment = 7'b0010000;
                4'hA: seven_segment = 7'b0001000;
                4'hB: seven_segment = 7'b0000011;
                4'hC: seven_segment = 7'b1000110;
                4'hD: seven_segment = 7'b0100001;
                4'hE: seven_segment = 7'b0000110;
                default: seven_segment = 7'b0001110;
            endcase
        end
    endfunction

    assign HEX0 = seven_segment(bpm % 10);
    assign HEX1 = seven_segment((bpm / 10) % 10);
    assign HEX2 = seven_segment((bpm / 100) % 10);
    assign HEX3 = seven_segment(ir_sample[17:14]);
    assign HEX4 = seven_segment(ir_sample[13:10]);
    assign HEX5 = seven_segment(ir_sample[9:6]);
endmodule
