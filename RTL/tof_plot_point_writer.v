`timescale 1ns/1ps
// ============================================================================
// Module : tof_plot_point_writer
// Role   : Convert ToF polar samples (dist_mm, theta_q15) into framebuffer
//          coordinates (x_fb, y_fb) and write '1' into a 256x256 bit-plane.
//
// Key behaviors
//   - System clock domain (clk_sys).
//   - Accepts a 1-cycle dist_vld strobe indicating a new polar sample.
//   - Converts:
//        x_fb := floor(theta * 256)          (theta_q15 treated as 0..1 turn)
//        y_fb := 255 - round(dist * 255 / R_MAX_MM)
//   - Gated by clear_busy: no writes during frame clear.
//   - Optional "stamp" mode: writes a (2R+1)x(2R+1) square around (x_fb,y_fb)
//     over multiple cycles (micro-burst).
//
// Notes on synthesis
//   - Division by constant R_MAX_MM and by constant 5 (if used elsewhere) is
//     synthesizable; tools will implement via mult+shift or dedicated divider.
//   - Rounding is implemented by adding half the divisor before division.
//
// ============================================================================

module tof_plot_point_writer #(
    parameter [15:0] R_MAX_MM      = 16'd2000,

    // Stamp radius in pixels:
    //   0 => 1x1 (single pixel)
    //   1 => 3x3
    //   2 => 5x5, etc.
    parameter integer STAMP_R       = 0
)(
    input  wire        clk_sys,
    input  wire        rst_sys,

    input  wire        clear_busy,

    input  wire [15:0] dist_mm,
    input  wire        dist_vld,
    input  wire [15:0] theta_q15,

    output reg         wr_en,
    output reg  [7:0]  wr_x,
    output reg  [7:0]  wr_y,
    output reg         wr_data
);

    // ------------------------------------------------------------------------
    // 1) Clamp distance to [0..R_MAX_MM]
    // ------------------------------------------------------------------------
    wire [15:0] r_clamped = (dist_mm > R_MAX_MM) ? R_MAX_MM : dist_mm;

    // ------------------------------------------------------------------------
    // 2) y_scaled = round(r_clamped * 255 / R_MAX_MM)
    //     - Multiply into wider type (up to 16+8 = 24 bits)
    //     - Add half divisor for rounding (R_MAX_MM/2)
    // ------------------------------------------------------------------------
    wire [23:0] mult_255 = r_clamped * 8'd255;

    // Rounding term: + R_MAX_MM/2 in the same scale as numerator
    // Because denominator is R_MAX_MM, rounding is: (N + D/2) / D
    wire [23:0] mult_rounded = mult_255 + {8'd0, (R_MAX_MM >> 1)};

    wire [7:0] y_scaled = (R_MAX_MM == 0) ? 8'd0 : (mult_rounded / R_MAX_MM);

    // Invert so near is bottom, far is top
    wire [7:0] y_center = 8'd255 - y_scaled;

    // ------------------------------------------------------------------------
    // 3) x_center from theta fraction
    //    theta_q15 treated as unsigned 0..(almost 1.0)
    //    x = floor(theta * 256) ≈ upper 8 bits of Q1.15
    // ------------------------------------------------------------------------
    wire [7:0] x_center = theta_q15[15:8];

    // ------------------------------------------------------------------------
    // 4) Stamp micro-FSM (optional multi-cycle write burst)
    //
    // If STAMP_R==0:
    //   - We emit exactly one write for each dist_vld sample.
    //
    // If STAMP_R>0:
    //   - On dist_vld, we latch center (x_center,y_center).
    //   - Then we walk offsets dx,dy over a square and emit one write per cycle.
    //   - This means a single sensor sample produces multiple framebuffer writes.
    //
    // Upstream arbitration note:
    //   - Current top-level mux selects clear or plot writer. That mux is
    //     compatible with this burst approach as long as:
    //       * clear_busy remains low during stamping
    //       * accept multiple plot writes per sample
    // ------------------------------------------------------------------------

    localparam integer DIAM = (2*STAMP_R + 1);

    // Center latches (hold stable during stamping burst)
    reg [7:0] x0, y0;

    // Stamping control
    reg stamping;
    reg signed [7:0] dx, dy;   // small signed offsets in range [-STAMP_R..+STAMP_R]

    // Compute candidate pixel with saturation to [0..255]
    function automatic [7:0] sat_u8;
        input signed [9:0] val; // allow small negatives/positives
    begin
        if (val < 0)        sat_u8 = 8'd0;
        else if (val > 255) sat_u8 = 8'd255;
        else                sat_u8 = val[7:0];
    end
    endfunction

    wire [7:0] x_stamp = sat_u8($signed({1'b0,x0}) + $signed(dx));
    wire [7:0] y_stamp = sat_u8($signed({1'b0,y0}) + $signed(dy));

    // ------------------------------------------------------------------------
    // 5) Sequential control
    // ------------------------------------------------------------------------
    always @(posedge clk_sys) begin
        if (rst_sys) begin
            wr_en    <= 1'b0;
            wr_x     <= 8'd0;
            wr_y     <= 8'd0;
            wr_data  <= 1'b0;

            x0       <= 8'd0;
            y0       <= 8'd0;

            stamping <= 1'b0;
            dx       <= 8'sd0;
            dy       <= 8'sd0;

        end else begin
            // default: no write unless we explicitly emit one this cycle
            wr_en   <= 1'b0;
            wr_data <= 1'b0;

            // ------------------------------------------------------------
            // Start stamping burst when a new sample arrives
            // ------------------------------------------------------------
            if (!stamping) begin
                if (dist_vld && !clear_busy) begin
                    // latch center point for this sample
                    x0 <= x_center;
                    y0 <= y_center;

                    if (STAMP_R == 0) begin
                        // single-pixel mode: emit one write now
                        wr_en   <= 1'b1;
                        wr_x    <= x_center;
                        wr_y    <= y_center;
                        wr_data <= 1'b1;
                    end else begin
                        // burst stamping mode: initialize offset walk
                        stamping <= 1'b1;
                        dx       <= -STAMP_R;
                        dy       <= -STAMP_R;
                    end
                end
            end else begin
                // ------------------------------------------------------------
                // Burst stamping: emit one write per cycle while stamping==1
                // ------------------------------------------------------------
                if (!clear_busy) begin
                    // emit current offset pixel
                    wr_en   <= 1'b1;
                    wr_x    <= x_stamp;
                    wr_y    <= y_stamp;
                    wr_data <= 1'b1;

                    // advance dx,dy over square
                    if (dx == STAMP_R) begin
                        dx <= -STAMP_R;
                        if (dy == STAMP_R) begin
                            // finished entire square
                            dy       <= -STAMP_R;
                            stamping <= 1'b0;
                        end else begin
                            dy <= dy + 8'sd1;
                        end
                    end else begin
                        dx <= dx + 8'sd1;
                    end
                end
                // If clear_busy becomes 1 mid-stamp, we simply pause stamping
                // (wr_en stays low due to default) and resume when clear_busy drops.
            end
        end
    end

endmodule
