`timescale 1ns/1ps

// ============================================================================
// File    : vga_range_plot_top.v
// Project : FPGA_Dynamic_Spatial_Mapping_Temperature_Control_Project
//
// Hierarchy role
// ----------------------------------------------------------------------------
//   This block is the "VGA background engine" for the system. It owns:
//     1) Pixel clocking + timing:
//          - Derives a 25 MHz pixel clock (pix_clk) from the 100 MHz fabric
//            clock (clk_100mhz) using clk_div_4.
//          - Generates 640x480@60 Hz timing (hcount, vcount, active_video,
//            vga_hsync_n, vga_vsync_n) via vga_timing_640x480_60.
//          - Emits a frame_tick pulse at the top-left of the active region
//            (1 cycle, pix_clk domain).
//
//     2) ToF range-plot framebuffer (256x256 bit-plane):
//          - Maintains a dual-port BRAM storing a 256x256 1-bit range image
//            in "framebuffer coordinates" (x_fb,y_fb) ∈ [0..255]².
//          - Port A (clk_100mhz domain) is written by:
//                * tof_frame_clear_fsm : clears all pixels once per sweep.
//                * tof_plot_point_writer : stamps ToF polar samples into
//                  framebuffer pixels based on (dist_mm,theta_q15).
//          - Port B (pix_clk domain) is read according to the current VGA
//            pixel (hcount,vcount) within the plot window.
//
//     3) Geometry mapping: screen → framebuffer:
//          - Left-half plotting region:
//                X: hcount ∈ [0..319]
//                Y: vcount ∈ [PLOT_Y0..PLOT_Y1]  (256-row window)
//          - Maps screen coordinates to framebuffer indices:
//                fb_rd_x = hcount[8:1]    (horizontal downsample)
//                fb_rd_y = vcount_offset[7:0], with
//                vcount_offset = vcount - PLOT_Y0.
//          - Uses this mapping both for reading fb_rd_q and for computing
//            auxiliary screen-space landmarks (axes, range rings).
//
//     4) Distance “range rings” and axes (cosmetic scaffolding):
//          - Precomputes three horizontal ring lines corresponding to
//            1/4, 1/2, and 3/4 of R_MAX_MM, mapped through the same radial
//            scaling as tof_plot_point_writer.
//          - Draws:
//                * Horizontal baseline at the bottom of the plot box
//                  (dist=0 → PLOT_Y1).
//                * Vertical axis at angle ≈ 0 (x_fb = 0).
//          - These are rendered as simple colored lines over the plot.
//
//     5) Logo viewport hook (MATLAB / UART-loaded logo window):
//          - Reserves a 160x120 region in the top-left of the active screen:
//                hcount ∈ [0..159], vcount ∈ [0..119].
//          - Provides logo_x/logo_y as 320x240 coordinates:
//                logo_x = 2*(hcount - LOGO_X0)
//                logo_y = 2*(vcount - LOGO_Y0)
//            so an external 320x240 RGB444 store (e.g., image_dualbuf_320x240)
//            can be sampled and downscaled into this region.
//          - Consumes logo_rgb from that external store and places it with
//            highest priority inside the logo box.
//
//     6) Buffer-bank indicator “LED” for logo double-buffering:
//          - Uses logo_buf_sel (0/1, pixel-domain view of the active logo bank)
//            to draw a tiny 8x8 tile in the upper-right corner of the logo
//            region.
//          - Bank 0 → red square; Bank 1 → green square.
//            This provides a visual confirmation of which logo buffer is
//            currently displayed and that swaps happen cleanly at frame edges.
//
//     7) Background-only output for HUD overlay:
//          - Exports:
//                pix_clk, hcount, vcount, active_video, frame_tick, rgb_plot
//            as the "background" pipeline.
//          - A higher-level module (e.g., vga_status_overlay) uses these
//            signals to draw telemetry widgets, HUD elements, and then maps
//            its rgb_out to the external VGA DAC pins.
//
// Parameters
// ----------------------------------------------------------------------------
//   R_MAX_MM
//     - Physical distance (mm) corresponding to full-scale in the radial
//       mapping used by tof_plot_point_writer. Controls the vertical scaling
//       of the polar ToF samples and the ring positions.
//   TEMP_Q15_COLD / TEMP_Q15_HOT
//     - Legacy parameters from an earlier revision where temperature bars
//       were drawn in this module. Kept for configuration continuity and
//       potential future reuse; not directly used in the current logic.
//
// Clock / reset domains
// ----------------------------------------------------------------------------
//   - System domain: clk_100mhz, rst_sys (active-HIGH, synchronous).
//        * ToF write-side logic (tof_frame_clear_fsm, tof_plot_point_writer).
//   - Pixel domain: pix_clk (25 MHz), rst_pix (rst_sys synchronized into
//     pix_clk domain).
//        * VGA timing, logo viewport coordinate generation, framebuffer read
//          mapping, and colorizer.
//
// CDC notes
// ----------------------------------------------------------------------------
//   - This module assumes dist_mm/dist_vld/theta_q15/sweep_wrap are already
//     synchronous to clk_100mhz.
//   - BRAM read/write port separation enforces a clean split between system-
//     and pixel-domain logic with no derived clocks beyond clk_div_4.
//
// ============================================================================

module vga_range_plot_top #(
    // Maximum range used to scale distance into vertical pixel index
    parameter [15:0] R_MAX_MM        = 16'd2000,

    // Legacy temperature range parameters (currently unused here; reserved)
    parameter signed [15:0] TEMP_Q15_COLD = 16'sd4096,   // ~60°F equivalent
    parameter signed [15:0] TEMP_Q15_HOT  = 16'sd16384   // ~90°F equivalent
)(
    // ------------------------------------------------------------------------
    // Clocks and reset
    // ------------------------------------------------------------------------
    input  wire        clk_100mhz,     // system/fabric clock (100 MHz)
    input  wire        rst_sys,        // synchronous, active-HIGH reset

    // ------------------------------------------------------------------------
    // ToF polar data (system domain, 100 MHz)
    //   - dist_mm   : radial distance in millimetres (unsigned).
    //   - dist_vld  : 1-cycle strobe when dist_mm is valid.
    //   - theta_q15 : angle in Q1.15 turns (0.0..~1.0).
    //   - sweep_wrap: 1-cycle pulse at end of a full sweep (triggers clear).
    // ------------------------------------------------------------------------
    input  wire [15:0] dist_mm,
    input  wire        dist_vld,
    input  wire [15:0] theta_q15,
    input  wire        sweep_wrap,

    // ------------------------------------------------------------------------
    // VGA timing + background RGB outputs (for overlay chain)
    // ------------------------------------------------------------------------
    output wire        pix_clk,       // 25 MHz pixel clock
    output wire [9:0]  hcount,        // 0..799 (includes blanking)
    output wire [9:0]  vcount,        // 0..524 (includes blanking)
    output wire        active_video,  // 1 when inside 640x480 active area
    output wire        frame_tick,    // 1-cycle pulse at (h=0,v=0) active
    output wire [11:0] rgb_plot,      // background RGB (4:4:4)

    // ------------------------------------------------------------------------
    // VGA sync outputs (still driven at this level)
    // ------------------------------------------------------------------------
    output wire        vga_hsync_n,
    output wire        vga_vsync_n,
    
    // ------------------------------------------------------------------------
    // Logo viewport hook
    //   - logo_x, logo_y : 320x240 coordinates used to sample an external
    //                      dual-buffered logo store.
    //   - logo_rgb       : 12-bit RGB444 pixel from that store.
    //   - logo_buf_sel   : which logo buffer bank (0/1) is currently displayed
    //                      in the pixel domain; used to draw an on-screen
    //                      indicator tile.
    // ------------------------------------------------------------------------
    output wire [8:0]  logo_x,        // 0..319
    output wire [7:0]  logo_y,        // 0..239
    input  wire [11:0] logo_rgb,      // RGB444 from logo dual buffer
    input  wire        logo_buf_sel   // 0/1: currently displayed logo bank
);

    // ========================================================================
    // 1) Clocking: derive 25 MHz pixel clock from 100 MHz system clock
    // ========================================================================
    // pix_clk is the sole clock for all pixel-domain logic in this module.
    wire rst_pix;   // reset synchronized into pixel domain

    clk_div_4 u_clk_div_4 (
        .clk_in (clk_100mhz),
        .rst_in (rst_sys),
        .clk_out(pix_clk)           // 25 MHz pixel clock
    );

    // Simple 2-FF reset synchronizer into pixel domain
    reg [1:0] rst_pix_ff;
    always @(posedge pix_clk) begin
        rst_pix_ff <= {rst_pix_ff[0], rst_sys};
    end
    assign rst_pix = rst_pix_ff[1];

    // ========================================================================
    // 2) VGA timing core: 640x480@60 Hz, 25 MHz pixel clock
    // ========================================================================
    wire hsync_n;
    wire vsync_n;

    vga_timing_640x480_60 u_vga_timing (
        .clk_pix      (pix_clk),
        .rst_pix      (rst_pix),
        .hcount       (hcount),
        .vcount       (vcount),
        .hsync_n      (hsync_n),
        .vsync_n      (vsync_n),
        .active_video (active_video)
    );

    assign vga_hsync_n = hsync_n;
    assign vga_vsync_n = vsync_n;

    // "Frame tick" = 1-cycle pulse at top-left of the active 640x480 region.
    assign frame_tick =
        active_video &&
        (hcount == 10'd0) &&
        (vcount == 10'd0);

    // ========================================================================
    // 2.5) Logo region geometry + viewport coordinates
    //      - Screen window: 160x120 at top-left of active area.
    //      - Logo coords : 320x240 via simple 2x upscale.
    // ========================================================================
    localparam [9:0] LOGO_X0 = 10'd0;
    localparam [9:0] LOGO_X1 = 10'd159;   // 160 px wide
    localparam [9:0] LOGO_Y0 = 10'd0;
    localparam [9:0] LOGO_Y1 = 10'd119;   // 120 px tall
    
    wire in_logo_region =
        active_video &&
        (hcount >= LOGO_X0) && (hcount <= LOGO_X1) &&
        (vcount >= LOGO_Y0) && (vcount <= LOGO_Y1);
    
    reg [8:0] logo_x_r; // 0..319
    reg [7:0] logo_y_r; // 0..239
    
    always @(posedge pix_clk) begin
        if (rst_pix) begin
            logo_x_r <= 9'd0;
            logo_y_r <= 8'd0;
        end else if (in_logo_region) begin
            // 2x upsample in both X and Y: (0..159) → (0..318)
            logo_x_r <= (hcount - LOGO_X0) << 1;
            logo_y_r <= (vcount - LOGO_Y0) << 1;
        end else begin
            // Outside logo region; park coordinates at 0 to avoid spurious
            // addresses in the external logo store.
            logo_x_r <= 9'd0;
            logo_y_r <= 8'd0;
        end
    end
    
    assign logo_x = logo_x_r;
    assign logo_y = logo_y_r;

    // ========================================================================
    // 3) ToF framebuffer: 256x256 bit-plane, dual-port BRAM
    // ========================================================================
    // Port A (clk_100mhz): write side (clear FSM or plot writer).
    wire        plot_wr_en;
    wire [7:0]  plot_wr_x;
    wire [7:0]  plot_wr_y;
    wire        plot_wr_data;

    wire        clear_busy;
    wire        clear_wr_en;
    wire [7:0]  clear_wr_x;
    wire [7:0]  clear_wr_y;
    wire        clear_wr_data;

    wire        fb_wr_en    = clear_busy ? clear_wr_en    : plot_wr_en;
    wire [7:0]  fb_wr_x     = clear_busy ? clear_wr_x     : plot_wr_x;
    wire [7:0]  fb_wr_y     = clear_busy ? clear_wr_y     : plot_wr_y;
    wire        fb_wr_data  = clear_busy ? clear_wr_data  : plot_wr_data;

    // Port B (pix_clk): read side (used by the colorizer).
    reg  [7:0] fb_rd_x;
    reg  [7:0] fb_rd_y;
    wire       fb_rd_q;      // 1-bit pixel value from framebuffer

    tof_plot_bram_dp u_tof_fb (
        // Port A: write side (system domain)
        .clk_a (clk_100mhz),
        .we_a  (fb_wr_en),
        .x_a   (fb_wr_x),
        .y_a   (fb_wr_y),
        .d_a   (fb_wr_data),

        // Port B: read side (pixel domain)
        .clk_b (pix_clk),
        .x_b   (fb_rd_x),
        .y_b   (fb_rd_y),
        .q_b   (fb_rd_q)
    );

    // ========================================================================
    // 4) Frame-clear FSM: wipe framebuffer once per sweep
    // ========================================================================
    // sweep_wrap is a 1-cycle pulse in clk_100mhz domain when the surveyor
    // completes a full sweep. This triggers tof_frame_clear_fsm to iterate
    // over the 256x256 address space and clear all bits.
    // ADDR_W=16 is sufficient for 256*256 = 65536 entries.
    // ========================================================================
    tof_frame_clear_fsm #(
        .ADDR_W(16)
    ) u_frame_clear (
        .clk_sys     (clk_100mhz),
        .rst_sys     (rst_sys),
        .start_clear (sweep_wrap),
        .busy        (clear_busy),
        .wr_en       (clear_wr_en),
        .wr_x        (clear_wr_x),
        .wr_y        (clear_wr_y),
        .wr_data     (clear_wr_data)
    );

    // ========================================================================
    // 5) ToF polar → framebuffer point writer (system domain)
    // ========================================================================
    // tof_plot_point_writer performs the polar-to-Cartesian indexing and
    // stamps a single bit into the 256x256 framebuffer whenever dist_vld=1.
    // R_MAX_MM controls the gain between physical mm and framebuffer radius.
    // ========================================================================
    tof_plot_point_writer #(
        .R_MAX_MM(R_MAX_MM)
    ) u_plot_writer (
        .clk_sys     (clk_100mhz),
        .rst_sys     (rst_sys),
        .clear_busy  (clear_busy),
        .dist_mm     (dist_mm),
        .dist_vld    (dist_vld),
        .theta_q15   (theta_q15),
        .wr_en       (plot_wr_en),
        .wr_x        (plot_wr_x),
        .wr_y        (plot_wr_y),
        .wr_data     (plot_wr_data)
    );

    // ========================================================================
    // 6) VGA-side mapping: 640x480 → 256x256 ToF framebuffer indices
    // ========================================================================
    // Left-half plot box:
    //   - X: 0..319  (320 pixels wide)
    //   - Y: PLOT_Y0..PLOT_Y1 (256 pixels tall)
    //
    // The Y coordinate is chosen so the plot sits directly under the logo band.
    // NOTE: LOGO_Y1 = 119, so LOGO_Y1+1 = 120; the current code retains a
    //       small 1-pixel offset (PLOT_Y0 = 1) for flexibility.
    // ========================================================================
    localparam integer PLOT_X0 = 0;
    localparam integer PLOT_X1 = 319;
    localparam integer PLOT_Y0 = 10'd0 + 1;       // could be LOGO_Y1 + 1
    localparam integer PLOT_Y1 = 10'd119 + 255;   // 256 rows below PLOT_Y0

    wire in_plot_box =
        (hcount >= PLOT_X0) && (hcount <= PLOT_X1) &&
        (vcount >= PLOT_Y0) && (vcount <= PLOT_Y1);

    wire [9:0] vcount_offset = vcount - PLOT_Y0;

    // Screen → framebuffer index mapping
    always @(posedge pix_clk) begin
        if (rst_pix) begin
            fb_rd_x <= 8'd0;
            fb_rd_y <= 8'd0;
        end else begin
            if (active_video && in_plot_box) begin
                // simple downsample horizontally; 320px → 160 fb columns
                fb_rd_x <= hcount[8:1];
                fb_rd_y <= vcount_offset[7:0];
            end else begin
                fb_rd_x <= 8'd0;
                fb_rd_y <= 8'd0;
            end
        end
    end

    // ========================================================================
    // 6.5) Precompute simple range “rings” inside the plot box
    // ========================================================================
    // We choose 3 distances: 1/4, 1/2, 3/4 of R_MAX_MM, and compute their
    // corresponding framebuffer Y indices using the same radial mapping
    // that tof_plot_point_writer uses:
    //
    //   scaled = (dist_mm * 255) / R_MAX_MM
    //   y_fb   = 255 - scaled   (far → top, near → bottom)
    //
    // Then we map y_fb into screen coordinates:
    //
    //   y_screen = PLOT_Y0 + {2'b00, y_fb}
    //
    // so each “ring” becomes a horizontal scanline across the full plot box.
    // ========================================================================
    localparam [15:0] RING0_MM = R_MAX_MM / 4;          // 25% of range
    localparam [15:0] RING1_MM = (R_MAX_MM / 4) * 2;    // 50% of range
    localparam [15:0] RING2_MM = (R_MAX_MM / 4) * 3;    // 75% of range

    // scale distance → [0..255] like tof_plot_point_writer
    wire [23:0] ring0_mult = RING0_MM * 8'd255;
    wire [23:0] ring1_mult = RING1_MM * 8'd255;
    wire [23:0] ring2_mult = RING2_MM * 8'd255;

    wire [7:0] ring0_scaled = (R_MAX_MM == 0) ? 8'd0 : ring0_mult / R_MAX_MM;
    wire [7:0] ring1_scaled = (R_MAX_MM == 0) ? 8'd0 : ring1_mult / R_MAX_MM;
    wire [7:0] ring2_scaled = (R_MAX_MM == 0) ? 8'd0 : ring2_mult / R_MAX_MM;

    // y_fb = 255 - scaled  (near bottom, far top)
    wire [7:0] ring0_y_fb = 8'd255 - ring0_scaled;
    wire [7:0] ring1_y_fb = 8'd255 - ring1_scaled;
    wire [7:0] ring2_y_fb = 8'd255 - ring2_scaled;

    // Convert framebuffer Y (0..255) to screen Y (PLOT_Y0..PLOT_Y1)
    wire [9:0] ring0_y_screen = PLOT_Y0 + {2'b00, ring0_y_fb};
    wire [9:0] ring1_y_screen = PLOT_Y0 + {2'b00, ring1_y_fb};
    wire [9:0] ring2_y_screen = PLOT_Y0 + {2'b00, ring2_y_fb};

    // Horizontal range rings: full width of plot box
    wire ring0_px = in_plot_box && (vcount == ring0_y_screen);
    wire ring1_px = in_plot_box && (vcount == ring1_y_screen);
    wire ring2_px = in_plot_box && (vcount == ring2_y_screen);

    wire any_ring_px = ring0_px | ring1_px | ring2_px;

    // Simple axes:
    //   - Horizontal baseline at dist=0 (bottom of plot box).
    //   - Vertical axis at angle ≈ 0 (x_fb=0 → left edge).
    wire baseline_px = in_plot_box && (vcount == PLOT_Y1);

    // Recover current framebuffer X index from hcount mapping (hcount[8:1])
    wire [7:0] x_fb_current = hcount[8:1];
    wire       y_axis_px    = in_plot_box && (x_fb_current == 8'd0);

    wire any_axis_px = baseline_px | y_axis_px;

    // ========================================================================
    // 7) Colorizer: Logo region (top-left) + ToF plot + overlays
    // ========================================================================
    // Priority order:
    //   1. Background default: black.
    //   2. Logo region: logo_rgb (under everything else within logo box).
    //   3. ToF plot in left-half: gray background, green “hits”.
    //   4. Range rings + axes: drawn over ToF plot.
    //   5. Buffer indicator tile: overrides a tiny portion of the logo box
    //      to show which logo bank is active.
    // ========================================================================
    reg [3:0] r_reg, g_reg, b_reg;

    // Indicator "LED" region: tiny 8x8 tile in upper-right of logo region
    localparam [9:0] BUF_LED_X0 = 10'd150;
    localparam [9:0] BUF_LED_X1 = 10'd159;
    localparam [9:0] BUF_LED_Y0 = 10'd0;
    localparam [9:0] BUF_LED_Y1 = 10'd7;
    
    wire in_buf_led =
        active_video &&
        (hcount >= BUF_LED_X0) && (hcount <= BUF_LED_X1) &&
        (vcount >= BUF_LED_Y0) && (vcount <= BUF_LED_Y1);
    
    always @(posedge pix_clk) begin
        if (rst_pix) begin
            r_reg <= 4'h0;
            g_reg <= 4'h0;
            b_reg <= 4'h0;
        end else if (!active_video) begin
            // Blanking: drive black
            r_reg <= 4'h0;
            g_reg <= 4'h0;
            b_reg <= 4'h0;
        end else begin
            // Default background inside active region: black
            r_reg <= 4'h0;
            g_reg <= 4'h0;
            b_reg <= 4'h0;
    
            // ----------------------------------------------------------------
            // Logo region (base, highest priority in its box except indicator)
            // ----------------------------------------------------------------
            if (in_logo_region) begin
                // logo_rgb already packed as 4:4:4
                r_reg <= logo_rgb[11:8];
                g_reg <= logo_rgb[7:4];
                b_reg <= logo_rgb[3:0];
            end
            // ----------------------------------------------------------------
            // ToF plot in the left half (under HUD overlay in higher modules)
            // ----------------------------------------------------------------
            else if (in_plot_box) begin
                if (fb_rd_q) begin
                    // ToF hit: bright green pixel
                    r_reg <= 4'h0;
                    g_reg <= 4'hF;
                    b_reg <= 4'h0;
                end else begin
                    // In plot box but no hit: dim gray background
                    r_reg <= 4'h3;
                    g_reg <= 4'h3;
                    b_reg <= 4'h3;
                end
            end
    
            // ----------------------------------------------------------------
            // Overlays on top of plot: range rings and axes
            // ----------------------------------------------------------------
            if (in_plot_box && any_ring_px) begin
                r_reg <= 4'h0;
                g_reg <= 4'h8;
                b_reg <= 4'h8;   // cyan-ish rings
            end
    
            if (in_plot_box && any_axis_px) begin
                r_reg <= 4'hF;
                g_reg <= 4'hF;
                b_reg <= 4'hF;   // white axes
            end
    
            // ----------------------------------------------------------------
            // On-screen logo buffer indicator (overrides small portion of logo)
            // ----------------------------------------------------------------
            if (in_buf_led) begin
                if (logo_buf_sel == 1'b0) begin
                    // Bank 0 → red square
                    r_reg <= 4'hF;
                    g_reg <= 4'h0;
                    b_reg <= 4'h0;
                end else begin
                    // Bank 1 → green square
                    r_reg <= 4'h0;
                    g_reg <= 4'hF;
                    b_reg <= 4'h0;
                end
            end
        end
    end

    // ========================================================================
    // 8) Export packed RGB as background image
    // ========================================================================
    assign rgb_plot = {r_reg, g_reg, b_reg};

endmodule
