`timescale 1ns/1ps

module de2_115_neuropulse_top #(
    parameter PPG_BASELINE_SHIFT = 6,
    parameter ACCEL_BASELINE_SHIFT = 4,
    parameter signed [15:0] PEAK_THRESHOLD_Q8_8 = 16'sd1280,
    parameter [15:0] IRREGULAR_THRESHOLD_X10 = 16'd200,
    parameter [15:0] HIGH_THRESHOLD_X10 = 16'd450,
    parameter signed [15:0] DRIFT_THRESHOLD_Q8_8 = 16'sd1280
)(
    input wire CLOCK_50,
    input wire [3:0] KEY,
    inout wire I2C_SDA,
    output wire I2C_SCL,
    output wire UART_TXD,
    output wire [7:0] LEDG,
    output wire [7:0] LCD_DATA,
    output wire LCD_EN, LCD_RS, LCD_RW, LCD_ON, LCD_BLON
);
    wire rst_n = KEY[0];
    reg key_sync, key_previous;
    wire arm_start = key_previous && !key_sync;
    always @(posedge CLOCK_50 or negedge rst_n) begin
        if (!rst_n) begin key_sync <= 1; key_previous <= 1; end
        else begin key_sync <= KEY[1]; key_previous <= key_sync; end
    end

    wire ppg_valid, gyro_valid, ack_error;
    wire [15:0] ppg_data;
    wire signed [15:0] accel_data, gyro_data;
    i2c_sensor_reader u_sensors (
        .clk(CLOCK_50), .rst_n(rst_n), .sda(I2C_SDA), .scl(I2C_SCL),
        .ppg_sample_valid(ppg_valid), .ppg_sample(ppg_data), .accel_sample(accel_data),
        .gyro_sample_valid(gyro_valid), .gyro_sample(gyro_data),
        .ack_error(ack_error)
    );

    wire signed [15:0] filtered_ppg;
    wire peak_detected, drift_flag, test_active, test_done, test_complete, overall_valid;
    wire [15:0] rr_interval_ms, heart_rate_bpm, irregularity_pct_x10;
    wire signed [15:0] angle_q8_8;
    wire [1:0] pulse_risk, overall_risk;
    wire [3:0] countdown_sec;
    stroke_screen_top #(
        .DATA_WIDTH(16), .PPG_BASELINE_SHIFT(PPG_BASELINE_SHIFT),
        .ACCEL_BASELINE_SHIFT(ACCEL_BASELINE_SHIFT), .PEAK_THRESHOLD_Q8_8(PEAK_THRESHOLD_Q8_8),
        .IRREGULAR_THRESHOLD_X10(IRREGULAR_THRESHOLD_X10), .HIGH_THRESHOLD_X10(HIGH_THRESHOLD_X10),
        .DRIFT_THRESHOLD_Q8_8(DRIFT_THRESHOLD_Q8_8)
    ) u_dsp (
        .clk(CLOCK_50), .rst_n(rst_n),
        .ppg_sample_valid(ppg_valid), .ppg_sample_in(ppg_data), .accel_sample_in(accel_data),
        .arm_test_start(arm_start), .gyro_sample_valid(gyro_valid), .gyro_sample_in(gyro_data),
        .filtered_ppg_out(filtered_ppg), .peak_detected(peak_detected),
        .rr_interval_ms(rr_interval_ms), .heart_rate_bpm(heart_rate_bpm),
        .irregularity_pct_x10(irregularity_pct_x10), .pulse_risk(pulse_risk),
        .angle_q8_8(angle_q8_8), .drift_flag(drift_flag),
        .test_active(test_active), .test_done(test_done), .test_complete(test_complete),
        .countdown_sec(countdown_sec),
        .overall_risk(overall_risk), .overall_valid(overall_valid)
    );

    telemetry_uart u_telemetry (
        .clk(CLOCK_50), .rst_n(rst_n), .heart_rate_bpm(heart_rate_bpm),
        .irregularity_pct_x10(irregularity_pct_x10), .left_angle_q8_8(angle_q8_8),
        .right_angle_q8_8(16'sd0), .filtered_ppg_q8_8(filtered_ppg),
        .risk(overall_risk), .tx(UART_TXD)
    );

    lcd_risk_display u_lcd (
        .clk(CLOCK_50), .rst_n(rst_n), .overall_risk(overall_risk),
        .heart_rate_bpm(heart_rate_bpm), .angle_q8_8(angle_q8_8), .arm_test_active(test_active),
        .countdown_sec(countdown_sec), .lcd_data(LCD_DATA), .lcd_en(LCD_EN), .lcd_rs(LCD_RS),
        .lcd_rw(LCD_RW), .lcd_on(LCD_ON), .lcd_blon(LCD_BLON)
    );

    reg [25:0] alive_count;
    reg [24:0] heartbeat_count;
    reg ppg_alive, heartbeat;

    always @(posedge CLOCK_50 or negedge rst_n) begin
        if (!rst_n) begin
            alive_count <= 0;
            ppg_alive <= 0;
        end else if (ppg_valid) begin
            alive_count <= 0;
            ppg_alive <= 1;
        end else if (alive_count == 26'd49_999_999) begin
            ppg_alive <= 0;
        end else begin
            alive_count <= alive_count + 1'b1;
        end
    end

    always @(posedge CLOCK_50 or negedge rst_n) begin
        if (!rst_n) begin
            heartbeat_count <= 0;
            heartbeat <= 0;
        end else if (heartbeat_count == 25'd24_999_999) begin
            heartbeat_count <= 0;
            heartbeat <= ~heartbeat;
        end else begin
            heartbeat_count <= heartbeat_count + 1'b1;
        end
    end
    assign LEDG[0] = ppg_alive && !ack_error;
    assign LEDG[1] = heartbeat;
    assign LEDG[2] = (overall_risk != 2'b00);
    assign LEDG[3] = (overall_risk == 2'b10);
    assign LEDG[7:4] = 4'b0000;
endmodule