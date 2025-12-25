`timescale 1ns/1ps

// ============================================================================
// Module : vga_status_overlay_accel_only
// Role   : Overlay an ACCEL widget (X/Y/Z bars + bubble level) in a 4×N grid
//          on the right side of a 640×480 VGA frame.
//          This is a minimal variant focused on accelerometer visualization.
// ============================================================================
module vga_status_overlay_accel_only (
    input  wire        pix_clk,
    input  wire        rst,           // pix-domain reset

    // Pixel position & timing
    input  wire [9:0]  hcount,
    input  wire [9:0]  vcount,
    input  wire        active_video,
    input  wire        frame_tick,    // not used here, reserved

    // Base RGB (e.g. background or upstream graphics)
    input  wire [3:0]  base_r,
    input  wire [3:0]  base_g,
    input  wire [3:0]  base_b,

    // Accelerometer data in pix domain
    input  wire signed [7:0] accel_x_pix,
    input  wire signed [7:0] accel_y_pix,
    input  wire signed [7:0] accel_z_pix,
    input  wire              accel_update,    // 1-cycle pulse on new sample

    // Output RGB
    output wire [3:0] vga_r,
    output wire [3:0] vga_g,
    output wire [3:0] vga_b
);

    // ------------------------------------------------------------------------
    // Right-panel grid (4×GRID_COLS)
    // ------------------------------------------------------------------------
    localparam integer SCREEN_W       = 640;
    localparam integer SCREEN_H       = 480;

    localparam integer RIGHT_PANEL_X0 = 480;
    localparam integer RIGHT_PANEL_W  = 160;

    localparam integer GRID_ROWS      = 4;
    localparam integer GRID_COLS      = 2;   // adjust as needed

    localparam integer CELL_W         = RIGHT_PANEL_W / GRID_COLS;
    localparam integer CELL_H         = SCREEN_H      / GRID_ROWS;

    // ACCEL widget at row 3, col 2 (human) => row=2, col=1 (0-based)
    localparam integer ACCEL_ROW_IDX  = 2;
    localparam integer ACCEL_COL_IDX  = 1;

    localparam integer ACCEL_X0       = RIGHT_PANEL_X0 + ACCEL_COL_IDX * CELL_W;
    localparam integer ACCEL_Y0       =                  ACCEL_ROW_IDX * CELL_H;
    localparam integer ACCEL_W        = CELL_W;
    localparam integer ACCEL_H        = CELL_H;

    // ------------------------------------------------------------------------
    // 3×5 glyph helper for X/Y/Z
    // ------------------------------------------------------------------------
    function glyph3x5_pixel;
        input [1:0] char_sel; // 0='X',1='Y',2='Z'
        input [2:0] gx;       // 0..2
        input [2:0] gy;       // 0..4
        reg  [14:0] bits;
        integer idx;
    begin
        case (char_sel)
            2'd0: bits = 15'b101_010_010_010_101; // X
            2'd1: bits = 15'b101_010_010_010_010; // Y
            2'd2: bits = 15'b111_001_010_100_111; // Z
            default: bits = 15'b000_000_000_000_000;
        endcase

        if (gx < 3 && gy < 5) begin
            idx = gy*3 + gx;          // 0..14
            glyph3x5_pixel = bits[14-idx];
        end else begin
            glyph3x5_pixel = 1'b0;
        end
    end
    endfunction

    // ------------------------------------------------------------------------
    // ACCEL widget – bars + bubble + glyphs
    // ------------------------------------------------------------------------
    wire in_accel_panel =
        (hcount >= ACCEL_X0) && (hcount < ACCEL_X0 + ACCEL_W) &&
        (vcount >= ACCEL_Y0) && (vcount < ACCEL_Y0 + ACCEL_H);

    wire [9:0] accel_x_local = hcount - ACCEL_X0;
    wire [9:0] accel_y_local = vcount - ACCEL_Y0;

    // Scaling
    localparam signed [7:0] ACC_FULL = 8'sd64;

    function [6:0] accel_mag_to_px;
        input signed [7:0] v;
        reg   signed [7:0] abs_v;
    begin
        abs_v = v[7] ? -v : v;
        if (abs_v > ACC_FULL)
            abs_v = ACC_FULL;
        accel_mag_to_px = (abs_v * (ACCEL_W - 24)) / ACC_FULL;
    end
    endfunction

    wire [6:0] len_x = accel_mag_to_px(accel_x_pix);
    wire [6:0] len_y = accel_mag_to_px(accel_y_pix);
    wire [6:0] len_z = accel_mag_to_px(accel_z_pix);

    // Bars + glyph layout
    localparam integer ACCEL_BAR_LEFT   = 12;
    localparam integer ACCEL_BAR_TOP_X  = 18;
    localparam integer ACCEL_BAR_TOP_Y  = 34;
    localparam integer ACCEL_BAR_TOP_Z  = 50;
    localparam integer ACCEL_BAR_H      = 8;

    wire [9:0] bar_x_end_x = ACCEL_BAR_LEFT + len_x;
    wire [9:0] bar_x_end_y = ACCEL_BAR_LEFT + len_y;
    wire [9:0] bar_x_end_z = ACCEL_BAR_LEFT + len_z;

    wire in_bar_x_row = (accel_y_local >= ACCEL_BAR_TOP_X) &&
                        (accel_y_local <  ACCEL_BAR_TOP_X + ACCEL_BAR_H);

    wire in_bar_y_row = (accel_y_local >= ACCEL_BAR_TOP_Y) &&
                        (accel_y_local <  ACCEL_BAR_TOP_Y + ACCEL_BAR_H);

    wire in_bar_z_row = (accel_y_local >= ACCEL_BAR_TOP_Z) &&
                        (accel_y_local <  ACCEL_BAR_TOP_Z + ACCEL_BAR_H);

    wire in_bar_x =
        in_bar_x_row &&
        (accel_x_local >= ACCEL_BAR_LEFT) &&
        (accel_x_local <  bar_x_end_x);

    wire in_bar_y =
        in_bar_y_row &&
        (accel_x_local >= ACCEL_BAR_LEFT) &&
        (accel_x_local <  bar_x_end_y);

    wire in_bar_z =
        in_bar_z_row &&
        (accel_x_local >= ACCEL_BAR_LEFT) &&
        (accel_x_local <  bar_x_end_z);

    wire in_bar_baseline_x =
        in_bar_x_row &&
        (accel_x_local >= ACCEL_BAR_LEFT) &&
        (accel_x_local <  ACCEL_W - 8);

    wire in_bar_baseline_y =
        in_bar_y_row &&
        (accel_x_local >= ACCEL_BAR_LEFT) &&
        (accel_x_local <  ACCEL_W - 8);

    wire in_bar_baseline_z =
        in_bar_z_row &&
        (accel_x_local >= ACCEL_BAR_LEFT) &&
        (accel_x_local <  ACCEL_W - 8);

    // Glyphs (3×5)
    localparam integer GLYPH_X0 = 3;
    localparam integer GLYPH_W  = 3;
    localparam integer GLYPH_H  = 5;

    // 'X'
    wire in_x_glyph_box =
        (accel_x_local >= GLYPH_X0) &&
        (accel_x_local <  GLYPH_X0 + GLYPH_W) &&
        (accel_y_local >= ACCEL_BAR_TOP_X) &&
        (accel_y_local <  ACCEL_BAR_TOP_X + GLYPH_H);

    wire [2:0] x_glyph_gx = accel_x_local - GLYPH_X0;
    wire [2:0] x_glyph_gy = accel_y_local - ACCEL_BAR_TOP_X;

    wire x_glyph_pixel =
        in_x_glyph_box &&
        glyph3x5_pixel(2'd0, x_glyph_gx, x_glyph_gy);

    // 'Y'
    wire in_y_glyph_box =
        (accel_x_local >= GLYPH_X0) &&
        (accel_x_local <  GLYPH_X0 + GLYPH_W) &&
        (accel_y_local >= ACCEL_BAR_TOP_Y) &&
        (accel_y_local <  ACCEL_BAR_TOP_Y + GLYPH_H);

    wire [2:0] y_glyph_gx = accel_x_local - GLYPH_X0;
    wire [2:0] y_glyph_gy = accel_y_local - ACCEL_BAR_TOP_Y;

    wire y_glyph_pixel =
        in_y_glyph_box &&
        glyph3x5_pixel(2'd1, y_glyph_gx, y_glyph_gy);

    // 'Z'
    wire in_z_glyph_box =
        (accel_x_local >= GLYPH_X0) &&
        (accel_x_local <  GLYPH_X0 + GLYPH_W) &&
        (accel_y_local >= ACCEL_BAR_TOP_Z) &&
        (accel_y_local <  ACCEL_BAR_TOP_Z + GLYPH_H);

    wire [2:0] z_glyph_gx = accel_x_local - GLYPH_X0;
    wire [2:0] z_glyph_gy = accel_y_local - ACCEL_BAR_TOP_Z;

    wire z_glyph_pixel =
        in_z_glyph_box &&
        glyph3x5_pixel(2'd2, z_glyph_gx, z_glyph_gy);

    // Bubble-level
    localparam integer BUBBLE_CENTER_X = ACCEL_W/2;
    localparam integer BUBBLE_CENTER_Y = ACCEL_H - 26;
    localparam integer BUBBLE_R        = 20;
    localparam integer DOT_R           = 3;

    wire signed [9:0] bubble_dx = accel_x_pix >>> 1;
    wire signed [9:0] bubble_dy = accel_y_pix >>> 1;

    wire signed [9:0] dot_cx = BUBBLE_CENTER_X + bubble_dx;
    wire signed [9:0] dot_cy = BUBBLE_CENTER_Y - bubble_dy;

    wire signed [10:0] rel_x = accel_x_local - BUBBLE_CENTER_X;
    wire signed [10:0] rel_y = accel_y_local - BUBBLE_CENTER_Y;
    wire [21:0] dist2_center = rel_x*rel_x + rel_y*rel_y;
    wire [21:0] bubble_r2    = BUBBLE_R*BUBBLE_R;
    wire [21:0] bubble_r2_in = (BUBBLE_R-2)*(BUBBLE_R-2);

    wire in_bubble_ring =
          (dist2_center <= bubble_r2)
       && (dist2_center >= bubble_r2_in);

    wire signed [10:0] rel_dot_x = accel_x_local - dot_cx;
    wire signed [10:0] rel_dot_y = accel_y_local - dot_cy;
    wire [21:0] dot_dist2 = rel_dot_x*rel_dot_x + rel_dot_y*rel_dot_y;
    wire [21:0] dot_r2    = DOT_R*DOT_R;

    wire in_bubble_dot = (dot_dist2 <= dot_r2);

    // Title strip
    localparam integer ACCEL_TITLE_H = 12;
    wire in_title_strip = (accel_y_local < ACCEL_TITLE_H);

    // Color generation
    reg  [3:0] accel_r, accel_g, accel_b;
    wire       accel_draw;

    always @* begin
        accel_r = 4'h0;
        accel_g = 4'h0;
        accel_b = 4'h0;

        if (!in_accel_panel) begin
            // no contribution
        end else begin
            // Base panel bg
            accel_r = 4'h1;
            accel_g = 4'h1;
            accel_b = 4'h2;

            // Title
            if (in_title_strip) begin
                accel_r = 4'h4;
                accel_g = 4'h4;
                accel_b = 4'h8;
            end

            // Baselines
            if (in_bar_baseline_x || in_bar_baseline_y || in_bar_baseline_z) begin
                accel_r = 4'h3;
                accel_g = 4'h3;
                accel_b = 4'h5;
            end

            // Bars
            if (in_bar_x) begin
                accel_r = 4'hF;
                accel_g = 4'h3;
                accel_b = 4'h3;
            end

            if (in_bar_y) begin
                accel_r = 4'h3;
                accel_g = 4'hF;
                accel_b = 4'h3;
            end

            if (in_bar_z) begin
                accel_r = 4'h3;
                accel_g = 4'h4;
                accel_b = 4'hF;
            end

            // Bubble ring
            if (in_bubble_ring) begin
                accel_r = 4'hF;
                accel_g = 4'hF;
                accel_b = 4'hF;
            end

            // Bubble dot
            if (in_bubble_dot) begin
                accel_r = 4'hF;
                accel_g = 4'hF;
                accel_b = 4'h0;
            end

            // Glyphs override for legibility
            if (x_glyph_pixel) begin
                accel_r = 4'hF;
                accel_g = 4'h6;
                accel_b = 4'h6;
            end
            if (y_glyph_pixel) begin
                accel_r = 4'h6;
                accel_g = 4'hF;
                accel_b = 4'h6;
            end
            if (z_glyph_pixel) begin
                accel_r = 4'h6;
                accel_g = 4'h6;
                accel_b = 4'hF;
            end
        end
    end

    assign accel_draw = in_accel_panel;

    // ------------------------------------------------------------------------
    // Final RGB mux
    // ------------------------------------------------------------------------
    reg [3:0] rgb_r, rgb_g, rgb_b;

    always @* begin
        rgb_r = base_r;
        rgb_g = base_g;
        rgb_b = base_b;

        if (accel_draw) begin
            rgb_r = accel_r;
            rgb_g = accel_g;
            rgb_b = accel_b;
        end
    end

    // Optionally blank when not in active video
    assign vga_r = active_video ? rgb_r : 4'h0;
    assign vga_g = active_video ? rgb_g : 4'h0;
    assign vga_b = active_video ? rgb_b : 4'h0;

endmodule
