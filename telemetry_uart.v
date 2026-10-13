`timescale 1ns/1ps

module telemetry_uart #(
    parameter CLOCK_HZ = 50000000,
    parameter BAUD = 9600,
    parameter FRAME_INTERVAL = 10000000
)(
    input wire clk, input wire rst_n,
    input wire [15:0] heart_rate_bpm, irregularity_pct_x10,
    input wire signed [15:0] left_angle_q8_8, right_angle_q8_8, filtered_ppg_q8_8,
    input wire [1:0] risk,
    output wire tx
);
    localparam DIVISOR = CLOCK_HZ / BAUD;
    reg tx_reg, uart_busy;
    reg uart_start;
    reg [7:0] uart_data;
    reg [9:0] uart_shift;
    reg [15:0] baud_count;
    reg [3:0] bit_index;
    reg [23:0] frame_timer;
    reg active, waiting, byte_started;
    reg [5:0] char_index;
    reg [15:0] bpm_frame, irregularity_frame;
    reg signed [15:0] left_frame, right_frame, ppg_frame;
    reg [1:0] risk_frame;
    assign tx = tx_reg;

    function [7:0] record_char(input [5:0] pos);
        integer field_num, column;
        reg [15:0] value;
        reg negative;
        begin
            field_num = pos / 7;
            column = pos % 7;
            value = 0;
            negative = 0;
            case (field_num)
                0: value = bpm_frame;
                1: value = irregularity_frame;
                2: begin value = left_frame[15] ? (~left_frame + 1'b1) : left_frame; negative = left_frame[15]; end
                3: begin value = right_frame[15] ? (~right_frame + 1'b1) : right_frame; negative = right_frame[15]; end
                4: begin value = ppg_frame[15] ? (~ppg_frame + 1'b1) : ppg_frame; negative = ppg_frame[15]; end
                default: value = {14'd0, risk_frame};
            endcase
            if (pos == 41) record_char = 8'h0A;
            else if (column == 6) record_char = ",";
            else if (column == 0) record_char = negative ? "-" : " ";
            else case (column)
                1: record_char = "0" + ((value / 10000) % 10);
                2: record_char = "0" + ((value / 1000) % 10);
                3: record_char = "0" + ((value / 100) % 10);
                4: record_char = "0" + ((value / 10) % 10);
                default: record_char = "0" + (value % 10);
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_reg <= 1; uart_busy <= 0; uart_shift <= 10'h3FF;
            baud_count <= 0; bit_index <= 0;
        end else if (uart_start && !uart_busy) begin
            uart_busy <= 1; tx_reg <= 0; uart_shift <= {1'b1, uart_data, 1'b0};
            baud_count <= 0; bit_index <= 0;
        end else if (uart_busy) begin
            if (baud_count == DIVISOR-1) begin
                baud_count <= 0;
                if (bit_index == 8) begin tx_reg <= 1; bit_index <= 9; end
                else if (bit_index == 9) begin tx_reg <= 1; uart_busy <= 0; end
                else begin tx_reg <= uart_shift[1]; uart_shift <= {1'b1, uart_shift[9:1]}; bit_index <= bit_index + 1'b1; end
            end else baud_count <= baud_count + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_timer <= 0; active <= 0; waiting <= 0; byte_started <= 0;
            char_index <= 0; uart_start <= 0; uart_data <= 0;
            bpm_frame <= 0; irregularity_frame <= 0; left_frame <= 0; right_frame <= 0; ppg_frame <= 0; risk_frame <= 0;
        end else begin
            uart_start <= 0;
            if (!active) begin
                if (frame_timer == FRAME_INTERVAL-1) begin
                    frame_timer <= 0;
                    bpm_frame <= heart_rate_bpm; irregularity_frame <= irregularity_pct_x10;
                    left_frame <= left_angle_q8_8; right_frame <= right_angle_q8_8;
                    ppg_frame <= filtered_ppg_q8_8; risk_frame <= risk;
                    active <= 1; char_index <= 0; waiting <= 0; byte_started <= 0;
                end else frame_timer <= frame_timer + 1'b1;
            end else if (!waiting && !uart_busy) begin
                uart_data <= record_char(char_index);
                uart_start <= 1; waiting <= 1; byte_started <= 0;
            end else if (waiting && !byte_started && uart_busy) byte_started <= 1;
            else if (waiting && byte_started && !uart_busy) begin
                waiting <= 0; byte_started <= 0;
                if (char_index == 41) active <= 0;
                else char_index <= char_index + 1'b1;
            end
        end
    end
endmodule
