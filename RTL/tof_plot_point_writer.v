`timescale 1ns/1ps


// ============================================================================
// Module : tof_plot_point_writer
// Role   : Convert ToF polar data (dist_mm, theta_q15) into a single
//          framebuffer pixel (x,y) and assert a 1-cycle write strobe.
//
//   * Runs in system (100 MHz) domain.
//   * Accepts unsigned distance in millimetres and Q1.15 angle fraction.
//   * Maps angle → x_pixel (0..255), distance → y_pixel (0..255).
//   * Writes a '1' into the framebuffer at (x,y) for each valid sample.
//   * Gated by clear_busy so no new writes occur during frame clears.
//
// Mapping strategy (simple, tunable):
//   - x = floor(theta_q15 * 256)  for theta in [0,1) turn
//       → use upper 8 bits: x = theta_q15[15:8]  (treat as unsigned fraction)
//   - r_clamped = min(dist_mm, R_MAX_MM)
//   - y_scaled  = (r_clamped * 255) / R_MAX_MM  (0=near, 255=far)
//   - y = 255 - y_scaled  (near at bottom of screen, far at top)
// ============================================================================
module tof_plot_point_writer #(
    parameter [15:0] R_MAX_MM = 16'd2000
)(
    input  wire        clk_sys,
    input  wire        rst_sys,

    input  wire        clear_busy,   // inhibit writes while clearing

    // ToF sample
    input  wire [15:0] dist_mm,
    input  wire        dist_vld,
    input  wire [15:0] theta_q15,    // 0..1-turn fraction (treated as unsigned)

    // Framebuffer write port (to be arbitrated upstream)
    output reg         wr_en,
    output reg  [7:0]  wr_x,
    output reg  [7:0]  wr_y,
    output reg         wr_data
);
    // Clamp distance to configured maximum
    wire [15:0] r_clamped =
        (dist_mm > R_MAX_MM) ? R_MAX_MM : dist_mm;

    // Multiply by 255 to normalize to [0,255]
    wire [23:0] mult = r_clamped * 8'd255;

    // Division by constant R_MAX_MM → synthesizer will implement
    wire [7:0] y_scaled = (R_MAX_MM == 0) ? 8'd0 : mult / R_MAX_MM;

    // Final Y index: invert so near objects appear towards bottom
    wire [7:0] y_fb = 8'd255 - y_scaled;

    // X index from angle: upper 8 bits of fraction
    // (Assumes theta_q15 is used as an unsigned fraction 0..1; ignore sign bit)
    wire [7:0] x_fb = theta_q15[15:8];

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            wr_en   <= 1'b0;
            wr_x    <= 8'd0;
            wr_y    <= 8'd0;
            wr_data <= 1'b0;
        end else begin
            // Default: no write
            wr_en   <= 1'b0;
            wr_data <= 1'b0;

            if (dist_vld && !clear_busy) begin
                // Emit a 1-cycle write at (x_fb, y_fb)
                wr_en   <= 1'b1;
                wr_x    <= x_fb;
                wr_y    <= y_fb;
                wr_data <= 1'b1;  // mark hit
            end
        end
    end

endmodule
