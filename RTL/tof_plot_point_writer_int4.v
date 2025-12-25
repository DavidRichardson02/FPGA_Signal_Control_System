`timescale 1ns/1ps
// ============================================================================
// Module : tof_plot_point_writer_int4
// Role   : Convert ToF polar sample into framebuffer (x,y) and write intensity.
//
// Write policy:
//   - On each dist_vld (and !decay_busy), write HIT_LEVEL into framebuffer.
//   - Distance scaled exactly like original (optionally rounded here).
// ============================================================================
module tof_plot_point_writer_int4 #(
    parameter [15:0] R_MAX_MM   = 16'd2000,
    parameter [3:0]  HIT_LEVEL  = 4'hF      // intensity written on each hit
)(
    input  wire        clk_sys,
    input  wire        rst_sys,

    input  wire        decay_busy,   // inhibit writes while decay FSM owns BRAM

    input  wire [15:0] dist_mm,
    input  wire        dist_vld,
    input  wire [15:0] theta_q15,

    output reg         wr_en,
    output reg  [7:0]  wr_x,
    output reg  [7:0]  wr_y,
    output reg  [3:0]  wr_data
);
    // Clamp distance to [0..R_MAX_MM]
    wire [15:0] r_clamped = (dist_mm > R_MAX_MM) ? R_MAX_MM : dist_mm;

    // Scale to [0..255]: y_scaled = round(r * 255 / R_MAX_MM)
    wire [23:0] mult_255      = r_clamped * 8'd255;
    wire [23:0] mult_rounded  = mult_255 + {8'd0, (R_MAX_MM >> 1)}; // rounding term
    wire [7:0]  y_scaled      = (R_MAX_MM == 0) ? 8'd0 : (mult_rounded / R_MAX_MM);

    // y_fb = 255 - y_scaled (near bottom, far top)
    wire [7:0] y_fb = 8'd255 - y_scaled;

    // x_fb = floor(theta * 256) ≈ theta_q15[15:8]
    wire [7:0] x_fb = theta_q15[15:8];

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            wr_en   <= 1'b0;
            wr_x    <= 8'd0;
            wr_y    <= 8'd0;
            wr_data <= 4'd0;
        end else begin
            wr_en <= 1'b0;

            if (dist_vld && !decay_busy) begin
                wr_en   <= 1'b1;
                wr_x    <= x_fb;
                wr_y    <= y_fb;
                wr_data <= HIT_LEVEL;
            end
        end
    end
endmodule
