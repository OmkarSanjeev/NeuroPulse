`timescale 1ns/1ps

// Serial, fixed-point LMS canceller. Inputs and output use signed Q8.8.
module lms_adaptive_filter #(
    parameter DATA_WIDTH = 16,
    parameter FRAC_BITS = 8,
    parameter TAPS = 4,
    parameter MU_SHIFT = 7
)(
    input wire clk,
    input wire rst_n,
    input wire sample_valid,
    input wire signed [DATA_WIDTH-1:0] desired_in,
    input wire signed [DATA_WIDTH-1:0] reference_in,
    output reg cleaned_valid,
    output reg signed [DATA_WIDTH-1:0] cleaned_out
);
    localparam IDX_WIDTH = $clog2(TAPS+1);
    localparam IDLE = 2'd0, MAC = 2'd1, UPDATE = 2'd2, DONE = 2'd3;
    reg [1:0] state;
    reg [IDX_WIDTH-1:0] idx;
    reg signed [DATA_WIDTH-1:0] ref_line [0:TAPS-1];
    reg signed [DATA_WIDTH-1:0] weights [0:TAPS-1];
    reg signed [2*DATA_WIDTH-1:0] acc;
    reg signed [DATA_WIDTH-1:0] desired_reg, error_reg;
    wire signed [2*DATA_WIDTH-1:0] mac_product = weights[idx] * ref_line[idx];
    wire signed [2*DATA_WIDTH-1:0] update_product = error_reg * ref_line[idx];
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; idx <= 0; acc <= 0; desired_reg <= 0;
            error_reg <= 0; cleaned_valid <= 0; cleaned_out <= 0;
            for (i = 0; i < TAPS; i = i + 1) begin
                ref_line[i] <= 0;
                weights[i] <= 0;
            end
        end else begin
            cleaned_valid <= 0;
            case (state)
                IDLE: if (sample_valid) begin
                    desired_reg <= desired_in;
                    for (i = TAPS-1; i > 0; i = i - 1)
                        ref_line[i] <= ref_line[i-1];
                    ref_line[0] <= reference_in;
                    acc <= 0;
                    idx <= 0;
                    state <= MAC;
                end
                MAC: begin
                    acc <= acc + mac_product;
                    if (idx == TAPS-1) begin
                        error_reg <= desired_reg - ((acc + mac_product) >>> FRAC_BITS);
                        idx <= 0;
                        state <= UPDATE;
                    end else idx <= idx + 1'b1;
                end
                UPDATE: begin
                    weights[idx] <= weights[idx] + (update_product >>> (FRAC_BITS + MU_SHIFT));
                    if (idx == TAPS-1) state <= DONE;
                    else idx <= idx + 1'b1;
                end
                DONE: begin
                    cleaned_out <= error_reg;
                    cleaned_valid <= 1;
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

// Rising-threshold peak detector with a sample-count lockout.
module ppg_peak_detector #(
    parameter DATA_WIDTH = 16,
    parameter signed PEAK_THRESHOLD = 16'sd1280,
    parameter MIN_DISTANCE = 25
)(
    input wire clk, input wire rst_n, input wire sample_valid,
    input wire signed [DATA_WIDTH-1:0] ppg_in,
    output reg peak_valid
);
    reg signed [DATA_WIDTH-1:0] previous;
    reg [15:0] lockout;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin previous <= 0; lockout <= 0; peak_valid <= 0; end
        else begin
            peak_valid <= 0;
            if (sample_valid) begin
                previous <= ppg_in;
                if (lockout != 0) lockout <= lockout - 1'b1;
                else if (ppg_in >= PEAK_THRESHOLD && previous < PEAK_THRESHOLD) begin
                    peak_valid <= 1;
                    lockout <= MIN_DISTANCE;
                end
            end
        end
    end
endmodule

module pulse_interval_analyzer #(
    parameter FS_HZ = 50,
    parameter WINDOW_LEN = 8,
    parameter IRREGULAR_THRESHOLD_X10 = 200,
    parameter HIGH_THRESHOLD_X10 = 450
)(
    input wire clk, input wire rst_n,
    input wire sample_tick, input wire peak_valid,
    output reg [15:0] rr_interval_ms,
    output reg [15:0] heart_rate_bpm,
    output reg [15:0] irregularity_pct_x10,
    output reg [1:0] pulse_risk,
    output reg score_valid
);
    reg [15:0] sample_count, previous_rr;
    reg have_peak;
    reg [7:0] irregular_history;
    reg [3:0] history_count;
    reg [31:0] difference, normalized_difference;
    integer j;
    reg [3:0] irregular_count;
    wire [31:0] rr_ms_now = sample_count * (1000 / FS_HZ);

    always @* begin
        irregular_count = 0;
        for (j = 0; j < WINDOW_LEN; j = j + 1)
            irregular_count = irregular_count + irregular_history[j];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_count <= 0; previous_rr <= 0; have_peak <= 0;
            irregular_history <= 0; history_count <= 0;
            rr_interval_ms <= 0; heart_rate_bpm <= 0;
            irregularity_pct_x10 <= 0; pulse_risk <= 0; score_valid <= 0;
        end else begin
            score_valid <= 0;
            if (sample_tick) sample_count <= sample_count + 1'b1;
            if (peak_valid) begin
                if (!have_peak) have_peak <= 1;
                else if (rr_ms_now >= 200 && rr_ms_now <= 3000) begin
                    rr_interval_ms <= rr_ms_now[15:0];
                    heart_rate_bpm <= 60000 / rr_ms_now;
                    if (previous_rr != 0) begin
                        difference = (rr_ms_now > previous_rr) ? rr_ms_now - previous_rr : previous_rr - rr_ms_now;
                        normalized_difference = (difference * 1000) / previous_rr;
                        irregular_history <= {irregular_history[WINDOW_LEN-2:0], normalized_difference >= IRREGULAR_THRESHOLD_X10};
                        previous_rr <= rr_ms_now[15:0];
                        if (history_count < WINDOW_LEN) history_count <= history_count + 1'b1;
                        if (history_count >= WINDOW_LEN-1) begin
                            irregularity_pct_x10 <= ((irregular_count + (normalized_difference >= IRREGULAR_THRESHOLD_X10)) * 1000) / WINDOW_LEN;
                            pulse_risk <= (((irregular_count + (normalized_difference >= IRREGULAR_THRESHOLD_X10)) * 1000) / WINDOW_LEN >= HIGH_THRESHOLD_X10) ? 2'd2 :
                                          (((irregular_count + (normalized_difference >= IRREGULAR_THRESHOLD_X10)) * 1000) / WINDOW_LEN >= IRREGULAR_THRESHOLD_X10) ? 2'd1 : 2'd0;
                            score_valid <= 1;
                        end
                    end else previous_rr <= rr_ms_now[15:0];
                end
                sample_count <= 0;
            end
        end
    end
endmodule

module arm_drift_test #(
    parameter DATA_WIDTH = 16,
    parameter TEST_SAMPLES = 500,
    parameter signed [DATA_WIDTH-1:0] THRESHOLD_Q8_8 = 16'sd1280
)(
    input wire clk, input wire rst_n, input wire test_start,
    input wire gyro_valid,
    input wire signed [DATA_WIDTH-1:0] gyro_in,
    output reg test_active, output reg test_done, output reg test_complete,
    output reg signed [DATA_WIDTH-1:0] angle_q8_8,
    output reg drift_flag, output reg [3:0] countdown_sec
);
    reg [$clog2(TEST_SAMPLES+1)-1:0] sample_count;
    reg [5:0] second_count;
    wire signed [31:0] product = gyro_in * 16'sd10;
    wire signed [DATA_WIDTH-1:0] increment = product >>> 8;
    wire signed [DATA_WIDTH:0] next_angle = angle_q8_8 + increment;
    wire [DATA_WIDTH:0] magnitude = next_angle[DATA_WIDTH] ? -next_angle : next_angle;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            test_active <= 0; test_done <= 0; test_complete <= 0; sample_count <= 0; second_count <= 0;
            angle_q8_8 <= 0;
            drift_flag <= 0; countdown_sec <= 10;
        end else begin
            test_done <= 0;
            if (test_start && !test_active) begin
                test_active <= 1; sample_count <= 0; second_count <= 0;
                angle_q8_8 <= 0;
                drift_flag <= 0; test_complete <= 0; countdown_sec <= 10;
            end else if (test_active) begin
                if (gyro_valid) begin
                    angle_q8_8 <= next_angle[DATA_WIDTH-1:0];
                    sample_count <= sample_count + 1'b1;
                    if (second_count == 49) begin
                        second_count <= 0;
                        if (countdown_sec != 0) countdown_sec <= countdown_sec - 1'b1;
                    end else second_count <= second_count + 1'b1;
                    if (sample_count == TEST_SAMPLES-1) begin
                        test_active <= 0; test_done <= 1; test_complete <= 1; countdown_sec <= 0;
                        drift_flag <= magnitude >= THRESHOLD_Q8_8;
                    end
                end
            end
        end
    end
endmodule

module risk_fusion_engine(
    input wire clk, input wire rst_n,
    input wire pulse_score_valid, input wire [1:0] pulse_risk,
    input wire drift_test_done, input wire drift_flag,
    output reg [1:0] overall_risk, output reg overall_valid
);
    reg [1:0] pulse_latched;
    reg drift_latched;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_latched <= 0; drift_latched <= 0; overall_risk <= 0; overall_valid <= 0;
        end else begin
            overall_valid <= 0;
            if (pulse_score_valid) pulse_latched <= pulse_risk;
            if (drift_test_done) drift_latched <= drift_flag;
            if (pulse_score_valid || drift_test_done) begin
                if ((drift_test_done ? drift_flag : drift_latched) || pulse_risk == 2)
                    overall_risk <= 2;
                else if ((pulse_score_valid ? pulse_risk : pulse_latched) == 1)
                    overall_risk <= 1;
                else overall_risk <= 0;
                overall_valid <= 1;
            end
        end
    end
endmodule

module stroke_screen_top #(
    parameter DATA_WIDTH = 16,
    parameter PPG_BASELINE_SHIFT = 6,
    parameter ACCEL_BASELINE_SHIFT = 4,
    parameter signed [DATA_WIDTH-1:0] PEAK_THRESHOLD_Q8_8 = 16'sd1280,
    parameter [15:0] IRREGULAR_THRESHOLD_X10 = 16'd200,
    parameter [15:0] HIGH_THRESHOLD_X10 = 16'd450,
    parameter signed [DATA_WIDTH-1:0] DRIFT_THRESHOLD_Q8_8 = 16'sd1280
)(
    input wire clk, input wire rst_n,
    input wire ppg_sample_valid, input wire [DATA_WIDTH-1:0] ppg_sample_in,
    input wire signed [DATA_WIDTH-1:0] accel_sample_in,
    input wire arm_test_start,
    input wire gyro_sample_valid,
    input wire signed [DATA_WIDTH-1:0] gyro_sample_in,
    output wire signed [DATA_WIDTH-1:0] filtered_ppg_out,
    output wire peak_detected,
    output wire [15:0] rr_interval_ms, heart_rate_bpm, irregularity_pct_x10,
    output wire [1:0] pulse_risk,
    output wire signed [DATA_WIDTH-1:0] angle_q8_8,
    output wire drift_flag, test_active, test_done, test_complete,
    output wire [3:0] countdown_sec,
    output wire [1:0] overall_risk, output wire overall_valid
);
    reg [16:0] ppg_baseline;
    reg signed [15:0] accel_baseline;
    reg ppg_baseline_valid, accel_baseline_valid;
    wire signed [17:0] ppg_difference = $signed({1'b0, ppg_sample_in}) - $signed({1'b0, ppg_baseline});
    wire signed [16:0] accel_difference = $signed(accel_sample_in) - $signed(accel_baseline);
    wire signed [20:0] ppg_scaled = ppg_difference <<< 3;
    wire signed [18:0] accel_scaled = accel_difference <<< 3;
    wire signed [DATA_WIDTH-1:0] ppg_q8_8 = !ppg_baseline_valid ? 16'sd0 : (ppg_scaled > 32767) ? 16'sh7fff : (ppg_scaled < -32768) ? 16'sh8000 : ppg_scaled[15:0];
    wire signed [DATA_WIDTH-1:0] accel_q8_8 = (accel_scaled > 32767) ? 16'sh7fff : (accel_scaled < -32768) ? 16'sh8000 : accel_scaled[15:0];
    wire clean_valid;
    wire [15:0] pulse_bpm;
    wire pulse_score_valid;
    wire drift_test_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ppg_baseline <= 0; accel_baseline <= 0;
            ppg_baseline_valid <= 0; accel_baseline_valid <= 0;
        end
        else begin
            if (ppg_sample_valid) begin
                if (!ppg_baseline_valid) begin ppg_baseline <= {1'b0, ppg_sample_in}; ppg_baseline_valid <= 1; end
                else ppg_baseline <= ppg_baseline + (ppg_difference >>> PPG_BASELINE_SHIFT);
            end
            if (gyro_sample_valid) begin
                if (!accel_baseline_valid) begin accel_baseline <= accel_sample_in; accel_baseline_valid <= 1; end
                else accel_baseline <= accel_baseline + (accel_difference >>> ACCEL_BASELINE_SHIFT);
            end
        end
    end

    lms_adaptive_filter #(.DATA_WIDTH(DATA_WIDTH)) u_lms (
        .clk(clk), .rst_n(rst_n), .sample_valid(ppg_sample_valid),
        .desired_in(ppg_q8_8), .reference_in(accel_q8_8),
        .cleaned_valid(clean_valid), .cleaned_out(filtered_ppg_out)
    );
    ppg_peak_detector #(.DATA_WIDTH(DATA_WIDTH), .PEAK_THRESHOLD(PEAK_THRESHOLD_Q8_8)) u_peak (
        .clk(clk), .rst_n(rst_n), .sample_valid(clean_valid),
        .ppg_in(filtered_ppg_out), .peak_valid(peak_detected)
    );
    pulse_interval_analyzer #(
        .FS_HZ(50), .IRREGULAR_THRESHOLD_X10(IRREGULAR_THRESHOLD_X10),
        .HIGH_THRESHOLD_X10(HIGH_THRESHOLD_X10)
    ) u_interval (
        .clk(clk), .rst_n(rst_n), .sample_tick(ppg_sample_valid), .peak_valid(peak_detected),
        .rr_interval_ms(rr_interval_ms), .heart_rate_bpm(pulse_bpm),
        .irregularity_pct_x10(irregularity_pct_x10), .pulse_risk(pulse_risk), .score_valid(pulse_score_valid)
    );
    arm_drift_test #(.DATA_WIDTH(DATA_WIDTH), .THRESHOLD_Q8_8(DRIFT_THRESHOLD_Q8_8)) u_drift (
        .clk(clk), .rst_n(rst_n), .test_start(arm_test_start),
        .gyro_valid(gyro_sample_valid), .gyro_in(gyro_sample_in),
        .test_active(test_active), .test_done(drift_test_done), .test_complete(test_complete),
        .angle_q8_8(angle_q8_8), .drift_flag(drift_flag), .countdown_sec(countdown_sec)
    );
    assign test_done = drift_test_done;
    risk_fusion_engine u_fusion (
        .clk(clk), .rst_n(rst_n), .pulse_score_valid(pulse_score_valid), .pulse_risk(pulse_risk),
        .drift_test_done(drift_test_done), .drift_flag(drift_flag),
        .overall_risk(overall_risk), .overall_valid(overall_valid)
    );
    assign heart_rate_bpm = pulse_bpm;
endmodule
