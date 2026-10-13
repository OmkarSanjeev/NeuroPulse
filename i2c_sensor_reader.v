//==========================================================================
// i2c_sensor_reader.v  --  rewritten for DE2-115 (Cyclone IV, 50 MHz)
//
// Polled I2C master for:
//   * MAX30102 PPG @ 7'h57  : reads 3 FIFO bytes per poll (0x07)
//   * MPU6050        @ 7'h68 : burst-reads 14 bytes from 0x3B
//
// Open-drain SDA. One SCL bit-period = 4 quarter-phases of TICKS system
// clocks (100 kHz default). SCL is FSM-driven (held high at idle) so real
// START / repeated-START / STOP conditions are generated, and every
// transmitted byte is followed by a real ACK bit.
//
// BOTH sensors are now properly initialised (this was missing before):
//   MAX30102: cleared FIFO, rollover, HR mode, 50 Hz/18-bit samples, IR LED
//   MPU6050 : woken (AD0 strapped low)
//==========================================================================
`timescale 1ns/1ps

module i2c_sensor_reader #(
    parameter TICKS           = 125,   // 50 MHz / (4*125) = 100 kHz SCL
    parameter POLL_GAP_TICKS  = 850    // 50 MHz clocks; ~20 ms between PPG/IMU rounds
)(
    input  wire        clk,
    input  wire        rst_n,
    inout  wire        sda,
    output reg         scl,

    output reg         ppg_sample_valid,
    output reg signed [15:0] ppg_sample,
    output reg signed [15:0] accel_sample,
    output reg         gyro_sample_valid,
    output reg signed [15:0] gyro_sample,

    output reg         ack_error     // sticky flag: a slave NACKed (debug)
);

    // ---- 7-bit device addresses (<<1, R/W bit 0) ----
    localparam MAX_W = 8'hAE, MAX_R = 8'hAF;   // MAX30102 @ 0x57
    localparam MPU_W = 8'hD0, MPU_R = 8'hD1; // MPU6050: 0x68, AD0 low

    // ---- open-drain SDA ----
    reg sda_oe, sda_out;
    assign sda = (sda_oe && !sda_out) ? 1'b0 : 1'bz;

    // ---- FSM states ----
    localparam ST_START = 4'd0,   // generate START condition
               ST_TX    = 4'd1,   // transmit byte_reg (9 bits incl. ACK)
               ST_RSTRT = 4'd2,   // generate repeated START
               ST_RX    = 4'd3,   // receive byte_cnt bytes (master ACK/NACK)
               ST_STOP  = 4'd4,   // generate STOP condition
               ST_DONE  = 4'd5,   // post-process / pick next transaction
               ST_GAP   = 4'd6;   // idle gap between polls

    reg [3:0] state;
    reg [1:0] ph;                 // quarter-phase inside a bit: 0..3
    reg [7:0] tick;               // system-clock counter within a phase
    reg [3:0] bit_cnt;            // 0..7 = data bits, 8 = ACK bit
    reg [7:0] byte_reg;           // byte currently being transmitted
    reg [1:0] tx_kind;            // 0: dev write-addr, 1: reg, 2: dev read-addr, 3: write data
    reg       write_mode;         // 1 = register-write transaction (init)
    reg [7:0] reg_buf;            // register address for this transaction
    reg [7:0] data_buf;           // data byte for register-write
    reg [3:0] byte_cnt;           // bytes to read (3 = MAX30102, 14 = MPU6050)
    reg [7:0] shift;              // RX shift register
    reg [7:0] rd_buf [0:13];      // burst-read buffer
    reg [3:0] buf_idx;
    reg       dev;                // 0 = MAX30102, 1 = MPU6050
    reg [3:0] init_idx;           // 0..7 init writes, 8 = init finished
    reg       polling;            // 1 = init done, normal polling active
    reg [15:0] gap_cnt;

    // ---- init program lookup: (addr, reg, data) ----
    reg [7:0] init_addr, init_reg, init_data;
    always @(*) begin
        case (init_idx)
            3'd0: begin init_addr = MAX_W;   init_reg = 8'h09; init_data = 8'h02; end // enter heart-rate mode
            3'd1: begin init_addr = MAX_W;   init_reg = 8'h04; init_data = 8'h00; end // clear FIFO write pointer
            3'd2: begin init_addr = MAX_W;   init_reg = 8'h05; init_data = 8'h00; end // clear overflow counter
            3'd3: begin init_addr = MAX_W;   init_reg = 8'h06; init_data = 8'h00; end // clear FIFO read pointer
            3'd4: begin init_addr = MAX_W;   init_reg = 8'h08; init_data = 8'h1F; end // FIFO rollover
            3'd5: begin init_addr = MAX_W;   init_reg = 8'h0A; init_data = 8'h03; end // 50 Hz, 18-bit
            3'd6: begin init_addr = MAX_W;   init_reg = 8'h0C; init_data = 8'h24; end // Red LED current
            default: begin init_addr = MPU_W; init_reg = 8'h6B; init_data = 8'h00; end // wake MPU6050
        endcase
    end

    //========================================================================
    // Main FSM: advances one quarter-phase every TICKS system clocks.
    // Convention per bit:  ph0 SCL low (+set SDA) -> ph1 setup -> ph2 SCL
    // high -> ph3 sample.  SDA is only changed while SCL is low.
    //========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl <= 1'b1; sda_oe <= 1'b1; sda_out <= 1'b1;
            state <= ST_DONE; ph <= 2'd0; tick <= 8'd0; bit_cnt <= 4'd0;
            tx_kind <= 2'd0; write_mode <= 1'b0; byte_cnt <= 4'd0; buf_idx <= 4'd0;
            dev <= 1'b0; init_idx <= 4'd0; gap_cnt <= 16'd0; shift <= 8'd0;
            polling <= 1'b0;
            ppg_sample_valid <= 1'b0; gyro_sample_valid <= 1'b0;
            ppg_sample <= 16'sd0; accel_sample <= 16'sd0;
            gyro_sample <= 16'sd0;
            ack_error <= 1'b0;
        end else if (tick != TICKS-1) begin
            ppg_sample_valid <= 1'b0;
            gyro_sample_valid <= 1'b0;
            tick <= tick + 1'b1;
        end else begin
            ppg_sample_valid <= 1'b0;
            gyro_sample_valid <= 1'b0;
            tick <= 8'd0;
            ph   <= ph + 2'd1;
            case (state)

                //----------------------------------------------------------
                // START: SDA 1->0 while SCL high, then pull SCL low
                //----------------------------------------------------------
                ST_START: begin
                    case (ph)
                        2'd0: begin sda_oe <= 1'b1; sda_out <= 1'b1; scl <= 1'b1; end
                        2'd2: sda_out <= 1'b0;
                        2'd3: begin
                            scl <= 1'b0;
                            state <= ST_TX; bit_cnt <= 4'd0; tx_kind <= 2'd0;
                        end
                        default: ;
                    endcase
                end

                //----------------------------------------------------------
                // TX byte_reg (8 bits) + ACK sample (bit_cnt == 8)
                //----------------------------------------------------------
                ST_TX: begin
                    case (ph)
                        2'd0: begin
                            scl <= 1'b0;
                            if (bit_cnt < 8) begin
                                sda_oe  <= 1'b1;
                                sda_out <= byte_reg[7 - bit_cnt];
                            end else begin
                                sda_oe  <= 1'b0;      // release SDA for ACK
                            end
                        end
                        2'd2: scl <= 1'b1;
                        2'd3: begin
                            if (bit_cnt == 4'd8) begin
                                bit_cnt <= 4'd0;
                                if (sda == 1'b1) ack_error <= 1'b1;  // NACK
                                case (tx_kind)
                                    2'd0: begin               // sent write addr
                                        byte_reg <= reg_buf;
                                        tx_kind  <= 2'd1;
                                    end
                                    2'd1: begin               // sent reg byte
                                        if (write_mode) begin
                                            byte_reg <= data_buf;
                                            tx_kind  <= 2'd3; // -> write data
                                        end else begin
                                            state <= ST_RSTRT;
                                        end
                                    end
                                    2'd2: begin               // sent read addr
                                        state   <= ST_RX;
                                        buf_idx <= 4'd0;
                                    end
                                    default: state <= ST_STOP; // write data sent
                                endcase
                            end else begin
                                bit_cnt <= bit_cnt + 1'b1;
                            end
                        end
                        default: ;
                    endcase
                end

                //----------------------------------------------------------
                // Repeated START: SDA 1->0 while SCL high, then SCL low
                //----------------------------------------------------------
                ST_RSTRT: begin
                    case (ph)
                        2'd0: begin sda_oe <= 1'b1; sda_out <= 1'b1; scl <= 1'b0; end
                        2'd1: scl <= 1'b1;
                        2'd2: sda_out <= 1'b0;
                        2'd3: begin
                            scl <= 1'b0;
                            byte_reg <= dev ? MPU_R : MAX_R;
                            tx_kind  <= 2'd2;
                            state    <= ST_TX; bit_cnt <= 4'd0;
                        end
                        default: ;
                    endcase
                end

                //----------------------------------------------------------
                // RX byte_cnt bytes. Master ACKs every byte except the last
                // (NACK), which terminates the burst.
                //----------------------------------------------------------
                ST_RX: begin
                    case (ph)
                        2'd0: begin
                            scl <= 1'b0;
                            if (bit_cnt < 8) begin
                                sda_oe <= 1'b0;                 // slave drives
                            end else begin
                                sda_oe  <= 1'b1;
                                sda_out <= (buf_idx == byte_cnt-1); // ACK=0 / NACK=1
                            end
                        end
                        2'd2: scl <= 1'b1;
                        2'd3: begin
                            if (bit_cnt < 8) begin
                                shift   <= {shift[6:0], sda};
                                bit_cnt <= bit_cnt + 1'b1;
                            end else begin
                                bit_cnt <= 4'd0;
                                rd_buf[buf_idx] <= shift;
                                if (buf_idx == byte_cnt-1)
                                    state <= ST_STOP;
                                else
                                    buf_idx <= buf_idx + 1'b1;
                            end
                        end
                        default: ;
                    endcase
                end

                //----------------------------------------------------------
                // STOP: SDA 0->1 while SCL high, bus left released-high
                //----------------------------------------------------------
                ST_STOP: begin
                    case (ph)
                        2'd0: begin scl <= 1'b0; sda_oe <= 1'b1; sda_out <= 1'b0; end
                        2'd1: scl <= 1'b1;
                        2'd2: sda_out <= 1'b1;      // STOP condition
                        2'd3: state <= ST_DONE;
                        default: ;
                    endcase
                end

                //----------------------------------------------------------
                // Post-process, then choose the next transaction
                //----------------------------------------------------------
                ST_DONE: begin
                    if (ph == 2'd0) begin
                        if (init_idx < 4'd8) begin
                            // ---- launch next init register-write ----
                            byte_reg   <= init_addr;
                            reg_buf    <= init_reg;
                            data_buf   <= init_data;
                            write_mode <= 1'b1;
                            init_idx   <= init_idx + 1'b1;
                            state      <= ST_START;
                        end else if (!polling) begin
                            // ---- init just finished: start polling, no data yet ----
                            polling <= 1'b1;
                            dev     <= 1'b0;
                            gap_cnt <= 16'd0;
                            state   <= ST_GAP;
                        end else begin
                            // ---- latch sensor data ----
                            if (!dev) begin
                                // FIFO sample is 18-bit, left-justified across 3 bytes.
                                // Keep its upper 16 bits for the signed DSP input.
                                ppg_sample       <= {rd_buf[0], rd_buf[1]};
                                ppg_sample_valid <= 1'b1;
                            end else begin
                                // MPU6050 burst from 0x3B:
                                // [0:1]AX [2:3]AY [4:5]AZ [6:7]TEMP [8:9]GX [10:11]GY [12:13]GZ
                                accel_sample      <= {rd_buf[4], rd_buf[5]}; // ACCEL_Z
                                gyro_sample       <= {rd_buf[8], rd_buf[9]}; // GYRO_X
                                gyro_sample_valid <= 1'b1;
                            end
                            dev      <= ~dev;
                            gap_cnt  <= POLL_GAP_TICKS[15:0];
                            state    <= ST_GAP;
                        end
                    end
                end

                //----------------------------------------------------------
                // Idle gap between polls (10 us per count)
                //----------------------------------------------------------
                ST_GAP: begin
                    if (ph == 2'd0) begin
                        if (gap_cnt != 16'd0) begin
                            gap_cnt <= gap_cnt - 1'b1;
                        end else begin
                            write_mode <= 1'b0;
                            sda_oe <= 1'b1; sda_out <= 1'b1; scl <= 1'b1;
                            if (!dev) begin
                                byte_reg <= MAX_W;
                                reg_buf  <= 8'h07;   // MAX30102 FIFO data
                                byte_cnt <= 4'd3;
                            end else begin
                                byte_reg <= MPU_W;
                                reg_buf  <= 8'h3B;   // MPU6050 ACCEL_XOUT_H
                                byte_cnt <= 4'd14;
                            end
                            state <= ST_START;
                        end
                    end
                end

                default: state <= ST_GAP;
            endcase
        end
    end

endmodule
