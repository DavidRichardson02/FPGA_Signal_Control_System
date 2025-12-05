// ============================================================================
// rotary_bar_overlay.v
//   VGA overlay: vertical side bar driven by rotary encoder.
//
//   - Draws a vertical bar near the right edge of the screen.
//   - Bar height ∝ |enc_pos| (signed 16-bit position).
//   - Color indicates direction:
//        enc_dir=1 (CW)  → green-ish bar
//        enc_dir=0 (CCW) → red-ish bar
//   - A slowly decaying "activity" intensity is pumped by enc_step_pulse,
//     so you see a fade-out after spinning.
// ============================================================================

`timescale 1ns/1ps

module rotary_bar_overlay #(
    parameter integer H_RES        = 640,
    parameter integer V_RES        = 480,
    // X-range of the bar region (inclusive)
    parameter integer BAR_X0       = 600,
    parameter integer BAR_X1       = 620,
    // Center Y of the bar (around mid-screen)
    parameter integer CENTER_Y     = V_RES/2,
    // Max bar half-height in pixels (clamp)
    parameter integer BAR_MAX_PIX  = (V_RES/2) - 10,
    // Right-shift to scale |enc_pos| → pixels (1 << POS_SHIFT counts per pixel)
    parameter integer POS_SHIFT    = 5   // 2^5 = 32 counts per pixel
)(
    input  wire             clk_pix,        // Pixel clock (same domain as pix_x/pix_y)
    input  wire             rst,            // Synchronous, active-HIGH

    // Current pixel position + valid flag
    input  wire [9:0]       pix_x,          // 0..H_RES-1
    input  wire [9:0]       pix_y,          // 0..V_RES-1
    input  wire             video_active,   // 1 when inside active video region

    // Rotary encoder state
    input  wire signed [15:0] enc_pos,       // Signed position counter
    input  wire               enc_step_pulse,// 1-cycle pulse per detent (pixel domain)
    input  wire               enc_dir,       // 1=CW, 0=CCW

    // Base color from underlying scene
    input  wire [3:0]       base_r,
    input  wire [3:0]       base_g,
    input  wire [3:0]       base_b,

    // Final blended color (with bar overlay when active)
    output reg  [3:0]       out_r,
    output reg  [3:0]       out_g,
    output reg  [3:0]       out_b
);
    // ------------------------------------------------------------------------
    // Local 10-bit versions of integer parameters (for clean vector usage)
    // ------------------------------------------------------------------------
    localparam [9:0] BAR_X0_10          = BAR_X0;
    localparam [9:0] BAR_X1_10          = BAR_X1;
    localparam [9:0] CENTER_Y_10        = CENTER_Y;
    localparam [9:0] BAR_MAX_PIX_10     = BAR_MAX_PIX;
    localparam [9:0] V_RES_10           = V_RES;
    localparam [9:0] V_RES_MINUS_ONE_10 = V_RES - 1;

    // ------------------------------------------------------------------------
    // 1) Activity intensity: pumped by enc_step_pulse, decays once per frame.
    // ------------------------------------------------------------------------
    reg [7:0] enc_activity = 8'd0;

    wire at_frame_start = video_active && (pix_x == 10'd0) && (pix_y == 10'd0);

    always @(posedge clk_pix) begin
        if (rst) begin
            enc_activity <= 8'd0;
        end else begin
            if (enc_step_pulse) begin
                enc_activity <= 8'hFF;          // full brightness on new step
            end else if (at_frame_start && (enc_activity != 8'd0)) begin
                enc_activity <= enc_activity - 8'd1;  // slow decay per frame
            end
        end
    end

    // Map 8-bit activity to 4-bit intensity (0..15) by taking the top nibble.
    wire [3:0] intensity = enc_activity[7:4];

    // ------------------------------------------------------------------------
    // 2) Compute bar half-height (in pixels) from |enc_pos|.
    //    bar_height = min( |enc_pos| >> POS_SHIFT, BAR_MAX_PIX )
    // ------------------------------------------------------------------------
    wire signed [15:0] pos_s;
    assign pos_s = enc_pos;

    wire [15:0] pos_mag;
    assign pos_mag = pos_s[15] ? (~pos_s + 16'd1) : pos_s; // abs

    // Scale down by POS_SHIFT (simple shift → cheap).
    wire [15:0] pos_scaled;
    assign pos_scaled = pos_mag >> POS_SHIFT;

    // Clamp to BAR_MAX_PIX.
    wire [9:0] bar_height;
    assign bar_height =
        (pos_scaled[9:0] > BAR_MAX_PIX_10) ? BAR_MAX_PIX_10 : pos_scaled[9:0];

    // Direction sign: +1 for enc_pos>=0, -1 for enc_pos<0.
    //   enc_pos >= 0 → bar grows upwards.
    //   enc_pos <  0 → bar grows downwards.
    wire pos_nonneg = ~pos_s[15];

    // ------------------------------------------------------------------------
    // 3) Is the current pixel inside the bar region?
    // ------------------------------------------------------------------------
    wire in_bar_x = (pix_x >= BAR_X0_10) && (pix_x <= BAR_X1_10);

    // Compute vertical bounds for bar:
    //   If pos_nonneg:
    //       y_lo = CENTER_Y - bar_height
    //       y_hi = CENTER_Y
    //   Else:
    //       y_lo = CENTER_Y
    //       y_hi = CENTER_Y + bar_height
    reg [9:0] bar_y_lo;
    reg [9:0] bar_y_hi;

    integer tmp_hi;
    always @* begin
        if (pos_nonneg) begin
            // Positive position → bar above center
            bar_y_hi = CENTER_Y_10;
            // avoid underflow at top
            if (CENTER_Y_10 > bar_height)
                bar_y_lo = CENTER_Y_10 - bar_height;
            else
                bar_y_lo = 10'd0;
        end else begin
            // Negative position → bar below center
            bar_y_lo = CENTER_Y_10;

            // compute CENTER_Y + bar_height safely in an integer
            tmp_hi = CENTER_Y_10 + bar_height;

            // avoid overflow at bottom
            if (tmp_hi < V_RES)
                bar_y_hi = tmp_hi[9:0];
            else
                bar_y_hi = V_RES_MINUS_ONE_10;
        end
    end

    wire in_bar_y = (pix_y >= bar_y_lo) && (pix_y <= bar_y_hi);
    //wire in_bar   = video_active && in_bar_x && in_bar_y && (bar_height != 10'd0);
    wire in_bar   = video_active && in_bar_x && in_bar_y;
    // ^ remove the && (bar_height != 10'd0)

    // ------------------------------------------------------------------------
    // 4) Choose bar color based on direction and intensity.
    //
    //    - enc_dir = 1 → green bar (0, intensity, 0)
    //    - enc_dir = 0 → red   bar (intensity, 0, 0)
    // ------------------------------------------------------------------------
    wire [3:0] bar_r = (enc_dir) ? 4'd0      : intensity;
    wire [3:0] bar_g = (enc_dir) ? intensity : 4'd0;
    wire [3:0] bar_b = 4'd0;

    // ------------------------------------------------------------------------
    // 5) Final color mux:
    //     - If we’re in the bar region and intensity>0 → draw bar color.
    //     - Otherwise → pass through base color unchanged.
    // ------------------------------------------------------------------------
    wire bar_visible = in_bar && (intensity != 4'd0);

    always @* begin
        if (bar_visible) begin
            out_r = bar_r;
            out_g = bar_g;
            out_b = bar_b;
        end else begin
            out_r = base_r;
            out_g = base_g;
            out_b = base_b;
        end
    end

endmodule
