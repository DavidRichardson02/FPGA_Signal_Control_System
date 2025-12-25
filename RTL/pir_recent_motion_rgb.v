`timescale 1ns/1ps

// ============================================================================
// Module    : pir_recent_motion_rgb
// Project   : FPGA_Signal_Control_System / Spatial Mapping + Temp Control
//
// Purpose
//   Visualize the PIR "recent motion" hold interval as a color gradient on an
//   RGB LED. The LED is full green immediately after motion (maximum time left),
//   gradually shifts through yellow, and becomes full red as the hold interval
//   approaches expiration.
//
// Interface
//   * clk, rst     : 100 MHz fabric clock and synchronous reset.
//   * pir_rise     : 1-clock pulse when PIR goes 0→1 after conditioning.
//   * pir_on       : Level ("recent motion active"), e.g., pir_active from
//                    pir_conditioner, also fed into temp_fan_ctrl.
//   * rgb_r/g/b    : Active-HIGH RGB LED outputs (e.g., led0_r/g/b in XDC).
//
// Notes
//   * Uses a 1 ms tick derived from clk, but does NOT generate a separate clock.
//     All timing is via tick-enables.
//   * HOLD_S should match pir_conditioner.HOLD_S so the color fade spans the
//     same time window as the PIR "recent motion" level.
// ============================================================================
module pir_recent_motion_rgb #(
    parameter integer CLK_HZ  = 100_000_000,  // fabric clock frequency
    parameter integer HOLD_S  = 15          // PIR hold time in seconds
) (
    input  wire clk,
    input  wire rst,

    input  wire pir_rise,   // 1-cycle pulse on rising PIR (from pir_conditioner)
    input  wire pir_on,     // level: recent motion active (pir_active)

    output wire rgb_r,      // drive to led0_r in XDC
    output wire rgb_g,      // drive to led0_g in XDC
    output wire rgb_b       // drive to led0_b in XDC (unused here -> off)
);

    // ------------------------------------------------------------------------
    // 1 ms tick generator (tick-enable, not a clock)
    // ------------------------------------------------------------------------
    localparam integer MS_DIV   = CLK_HZ / 1000;      // cycles per millisecond
    localparam integer MS_DIV_W = $clog2(MS_DIV);

    reg [MS_DIV_W-1:0] ms_div_cnt = {MS_DIV_W{1'b0}};
    reg                ms_tick    = 1'b0;

    always @(posedge clk) begin
        if (rst) begin
            ms_div_cnt <= {MS_DIV_W{1'b0}};
            ms_tick    <= 1'b0;
        end else begin
            if (ms_div_cnt == MS_DIV-1) begin
                ms_div_cnt <= {MS_DIV_W{1'b0}};
                ms_tick    <= 1'b1;    // assert for exactly one clk cycle
            end else begin
                ms_div_cnt <= ms_div_cnt + 1'b1;
                ms_tick    <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Time-left representation as 8-bit "level"
    //   * HOLD_MS    : total hold time in milliseconds
    //   * LED_MAX    : 255 levels of brightness / "time left"
    //   * LED_STEP_MS: number of ms between brightness decrements
    //
    // Result: led_level starts at 255, decrements to 0 over the hold time.
    // ------------------------------------------------------------------------
    localparam integer HOLD_MS     = HOLD_S * 1000;      // total hold in ms
    localparam integer LED_MAX     = 255;
    // Integer division: one brightness step ~ every HOLD_MS / 255 ms.
    // For HOLD_S=120 s ⇒ HOLD_MS=120000 ms ⇒ LED_STEP_MS=470 ms.
    localparam integer LED_STEP_MS = (HOLD_MS / LED_MAX) > 0 ? (HOLD_MS / LED_MAX) : 1;
    localparam integer LED_STEP_W  = $clog2(LED_STEP_MS);

    reg [LED_STEP_W-1:0] step_ms_cnt = {LED_STEP_W{1'b0}};
    reg [7:0]            led_level   = 8'd0;  // 0 = expired, 255 = full time left

    always @(posedge clk) begin
        if (rst) begin
            step_ms_cnt <= {LED_STEP_W{1'b0}};
            led_level   <= 8'd0;
        end else begin
            if (pir_rise) begin
                // New motion event: reload to full "time left"
                led_level   <= 8'hFF;
                step_ms_cnt <= {LED_STEP_W{1'b0}};
            end else if (ms_tick && (led_level != 8'd0)) begin
                // Every LED_STEP_MS milliseconds, decrement one brightness level
                if (step_ms_cnt == LED_STEP_MS-1) begin
                    step_ms_cnt <= {LED_STEP_W{1'b0}};
                    led_level   <= led_level - 8'd1;
                end else begin
                    step_ms_cnt <= step_ms_cnt + 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Map led_level to R/G intensities:
    //   * Full green   (time just reloaded)  : led_level=255, red_level=0
    //   * Yellow-ish   (midway)             : red_level≈green_level
    //   * Full red     (time nearly expired): led_level≈0, red_level≈255
    // ------------------------------------------------------------------------
    reg [7:0] red_level   = 8'd0;
    reg [7:0] green_level = 8'd0;

    always @(posedge clk) begin
        if (rst) begin
            red_level   <= 8'd0;
            green_level <= 8'd0;
        end else begin
            red_level   <= 8'hFF - led_level;
            green_level <= led_level;
        end
    end

    // ------------------------------------------------------------------------
    // Simple 8-bit PWM engine.
    //   * PWM counter runs at clk (100 MHz).
    //   * Duty cycles proportional to red_level / green_level.
    //   * Outputs are gated by pir_on, so the LED is OFF when there is NO
    //     "recent motion" according to pir_conditioner.
// ------------------------------------------------------------------------
    reg [7:0] pwm_cnt = 8'd0;

    always @(posedge clk) begin
        if (rst) begin
            pwm_cnt <= 8'd0;
        end else begin
            pwm_cnt <= pwm_cnt + 8'd1;
        end
    end

    assign rgb_r = pir_on && (pwm_cnt < red_level);
    assign rgb_g = pir_on && (pwm_cnt < green_level);
    assign rgb_b = 1'b0;  // unused for this visualization (blue off)

endmodule
