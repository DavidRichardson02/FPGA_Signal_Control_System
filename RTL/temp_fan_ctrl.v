`timescale 1ns/1ps

// ======================================================================================
// File      : temp_fan_ctrl.v  (pedagogically documented, proportional control)
// Project   : FPGA_Dynamic_Spatial_Mapping_Temperature_Control_Project
// ======================================================================================
module temp_fan_ctrl #(
    // --------------------------------------------------------------------------
    // Q1.15 / Q2.14 controller parameters
    // --------------------------------------------------------------------------
    // ===================== Temperature setpoints (Q1.15) ======================
    // Mapping: sample_q15 ≈ ((T_F - 32)/90) * 2^15
    //   72°F -> sample_q15 ≈ 14564 (0x38E4)
    //   78°F -> sample_q15 ≈ 16748 (0x416C)
    parameter signed [15:0] T_SET_OCC_Q15   = 16'sd14564, // 72°F
    parameter signed [15:0] T_SET_UNOCC_Q15 = 16'sd16748, // 78°F

    // ===================== Proportional gains (Q2.14) =========================
    // Using a = 90/2^15 ≈ 0.00274658°F/LSB and
    //   Kp_phys_occ   ≈ 0.10 duty/°F  -> KP_OCC_Q14   ≈ 4
    //   Kp_phys_unocc ≈ 0.05 duty/°F  -> KP_UNOCC_Q14 ≈ 2
    parameter [15:0] KP_OCC_Q14   = 16'd4, // ≈0.10 duty/°F
    parameter [15:0] KP_UNOCC_Q14 = 16'd2, // ≈0.05 duty/°F

    // Small bias duty in Q1.15 so the fan doesn’t stall at very low duty.
    parameter signed [15:0] U_BIAS_Q15       = 16'sd1024, // ≈0.031 in Q1.15

    // PIR “boost” duty (Q1.15) when motion is detected and PIR is enabled.
    parameter        [15:0] DUTY_PIR_Q15     = 16'h6000,  // ≈0.75 duty in Q1.15

    // Control tick divider: 100e6 / CTRL_DIV = control loop update frequency.
    parameter integer        CTRL_DIV        = 5_000_000  // 20 Hz @ 100 MHz
)(
    input  wire        clk_100mhz,
    input  wire        rst,

    // Temperature sample from xadc_sampler
    input  wire [15:0] sample_q15,   // signed Q1.15 code
    input  wire        sample_vld,

    // Debounced button controls
    input  wire        btn_d_level,  // (unused here, kept for interface compat)
    input  wire        btn_d_pulse,

    // Motion contribution from PIR conditioner
    input  wire        pir_on,

    // Front-panel enable switches
    input  wire        sw_temp_en,
    input  wire        sw_manual_en,
    input  wire        sw_pir_en,

    // Fan drive outputs
    output wire        fan_pwm_ja3,
    output reg         fan_en_ja4,

    // Status taps
    output wire        fan_temp_on,
    output wire        fan_manual_on,
    output wire        fan_pir_on,

    // Extended debug/telemetry outputs
    output wire [15:0] duty_final_q15_dbg   // final Q1.15 duty command
);

    // =========================================================================
    // Local Q1.15 constants
    // =========================================================================
    localparam signed [15:0] ONE_Q15  = 16'sh7FFF; // ≈ +1.0
    localparam signed [15:0] ZERO_Q15 = 16'sd0;

    // =========================================================================
    // 1) Register latest temperature sample on sample_vld
    // =========================================================================
    reg signed [15:0] temp_q15_reg;

    always @(posedge clk_100mhz) begin
        if (rst) begin
            temp_q15_reg <= 16'sd0;
        end else if (sample_vld) begin
            temp_q15_reg <= sample_q15;
        end
    end

    // =========================================================================
    // 2) Manual override: T-flip-flop driven by BTN D pulse
    // =========================================================================
    reg manual_on_raw = 1'b0;

    always @(posedge clk_100mhz) begin
        if (rst) begin
            manual_on_raw <= 1'b0;
        end else if (btn_d_pulse) begin
            manual_on_raw <= ~manual_on_raw;
        end
    end

    // =========================================================================
    // 3) Slow control tick generator
    // =========================================================================
    reg [31:0] ctl_div_cnt = 32'd0;
    reg        ctl_tick_en = 1'b0;

    always @(posedge clk_100mhz) begin
        if (rst) begin
            ctl_div_cnt <= 32'd0;
            ctl_tick_en <= 1'b0;
        end else begin
            if (ctl_div_cnt == CTRL_DIV - 1) begin
                ctl_div_cnt <= 32'd0;
                ctl_tick_en <= 1'b1;
            end else begin
                ctl_div_cnt <= ctl_div_cnt + 1'b1;
                ctl_tick_en <= 1'b0;
            end
        end
    end

    // =========================================================================
    // 4) Temperature-based proportional controller math
    // =========================================================================
    // Occupancy flag (only matters for choosing gains/setpoints)
    wire        occ =
        (sw_pir_en && pir_on);

    // Occupancy-dependent setpoint and gain
    wire signed [15:0] t_set_q15 =
        occ ? T_SET_OCC_Q15  : T_SET_UNOCC_Q15;

    wire signed [15:0] kp_q14 =
        occ ? KP_OCC_Q14     : KP_UNOCC_Q14;

    // Error (Q1.15): positive when hotter than setpoint
    wire signed [15:0] e_q15 =
        temp_q15_reg - t_set_q15;

    // Multiply: Q1.15 * Q2.14 → Q3.29
    wire signed [31:0] p_tmp =
        $signed(e_q15) * $signed(kp_q14);

    // Align back to Q1.15: shift right by 14 fractional bits
    wire signed [31:0] p_shift =
        p_tmp >>> 14;

    // Proportional term in Q1.15
    wire signed [15:0] u_p_q15 =
        p_shift[15:0];

    // Add small bias (Q1.15) → 17-bit sum for clamping
    wire signed [16:0] u_sum =
        $signed(u_p_q15) + $signed(U_BIAS_Q15);

    // =========================================================================
    // 4b) Register temperature duty on control tick
    // =========================================================================
    reg signed [15:0] duty_temp_q15 = 16'sd0; // [0..1] Q1.15

    always @(posedge clk_100mhz) begin
        if (rst) begin
            duty_temp_q15 <= 16'sd0;
        end else if (ctl_tick_en) begin
            if (sw_temp_en) begin
                // Clamp u_sum into [0, ONE_Q15]
                if (u_sum <= 0) begin
                    duty_temp_q15 <= ZERO_Q15;
                end else if (u_sum >= {1'b0, ONE_Q15}) begin
                    duty_temp_q15 <= ONE_Q15;
                end else begin
                    duty_temp_q15 <= u_sum[15:0];
                end
            end else begin
                duty_temp_q15 <= ZERO_Q15;
            end
        end
        // Between control ticks, hold previous duty_temp_q15
    end

    // =========================================================================
    // 5) PIR “boost” duty and manual full-on duty (Q1.15 contributors)
    // =========================================================================
    wire [15:0] duty_pir_q15 =
        (sw_pir_en && pir_on) ? DUTY_PIR_Q15 : 16'd0;

    wire [15:0] duty_manual_q15 =
        (sw_manual_en && manual_on_raw) ? ONE_Q15[15:0] : 16'd0;

    // =========================================================================
    // 6) Combine contributors via max() in Q1.15
    // =========================================================================
    wire [15:0] duty_temp_or_pir_q15 =
        (duty_temp_q15 >= duty_pir_q15) ? duty_temp_q15 : duty_pir_q15;

    wire [15:0] duty_final_q15 =
        (duty_manual_q15 >= duty_temp_or_pir_q15) ? duty_manual_q15
                                                  : duty_temp_or_pir_q15;

    assign duty_final_q15_dbg = duty_final_q15;

    // =========================================================================
    // 7) Status taps and fan_on flag
    // =========================================================================
    assign fan_temp_on   = sw_temp_en   && (duty_temp_q15   != ZERO_Q15);
    assign fan_manual_on = sw_manual_en &&  manual_on_raw;
    assign fan_pir_on    = sw_pir_en    &&  pir_on;

    wire fan_on_int = (duty_final_q15 != ZERO_Q15);

    assign fan_on = fan_on_int;

    // =========================================================================
    // 8) Map Q1.15 duty_final_q15 into 12-bit duty for pwm_dac
    // =========================================================================
    wire [15:0] duty_q15_u =
        duty_final_q15[15] ? 16'd0 : duty_final_q15; // guard against sign

    wire [27:0] duty_prod =
        duty_q15_u * 12'd4095;   // 16+12 = 28 bits

    // Equivalent to (duty_q15_u * 4095) >> 15
    wire [11:0] duty =
        duty_prod[26:15];

    // =========================================================================
    // 9) 24.4 kHz PWM (variable duty) to fan driver (JA3)
    // =========================================================================
    pwm_dac #(.CTR_W(12)) u_fan_pwm (
        .clk  (clk_100mhz),
        .rst  (rst),
        .duty (duty),
        .pwm  (fan_pwm_ja3)
    );

    // =========================================================================
    // 10) JA4: logic enable mirror of fan_on
    // =========================================================================
    always @(posedge clk_100mhz) begin
        if (rst) begin
            fan_en_ja4 <= 1'b0;
        end else begin
            fan_en_ja4 <= fan_on_int;
        end
    end

endmodule
