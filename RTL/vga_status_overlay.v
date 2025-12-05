`timescale 1ns/1ps

// ============================================================================
// File    : vga_status_overlay.v
// Project : FPGA_Dynamic_Spatial_Mapping_Temperature_Control_Project
//
// Hierarchy role
// ----------------------------------------------------------------------------
//   This block is the "HUD overlay engine" for the VGA subsystem. It sits
//   logically *after* vga_range_plot_top:
//
//       vga_range_plot_top  →  vga_status_overlay  →  VGA DAC pins
//
//   and is responsible for painting a structured telemetry dashboard in the
//   right half of the 640x480 frame while passing through the ToF plot image
//   on the left half.
//
//   Conceptually, it takes:
//     - A background pixel stream (rgb_bg) containing the left-hand ToF map,
//     - A 4×N grid region in the right-hand half of the screen,
//     - A collection of slow telemetry/status values in the 100 MHz system
//       domain (temperature, fan duty, UART activity, angle, PIR, rotary),
//   and overlays:
//
//     1) A 4-column by N-row right-panel layout (grid geometry):
//          - Columns: 4 equal-width slices spanning hcount 320..639.
//          - Rows:
//              ROW0 : Legend band ("TOF", "TMP", "SRV", "SYS").
//              ROW1 : Fan mode tiles ("TEMP", "MAN", "PIR", "FAN").
//              ROW2 : Status tiles for TOF/TMP/SRV/SYS health.
//              ROW3 : UART bitfield tiles + hex counter + PIR streak.
//              ROW4 : Angle indicator (heading digits + compass stack).
//              ROW5 : Temperature and duty numeric glyphs above bars.
//              ROW6 : Fan speed gauge arc tiles (2×4 segments).
//          - A footer band below the grid (UART activity bar, spinner tail).
//
//     2) Clean clock domain crossing (CDC) and frame-stable HUD:
//          - All telemetry comes from the 100 MHz system domain.
//          - Each signal is 2-FF synchronized into pix_clk.
//          - Slowly varying quantities (temp, duty, theta, fan bits, switches,
//            accepted UART byte, encoder position/dir) are *snapshotted once
//            per frame* using frame_tick so the HUD is visually stable within
//            a frame.
//
//     3) Temperature bar + numeric display (Q1.15):
//          - sample_q15 is a Q1.15 temperature code derived from the XADC
//            channel (scaled such that TEMP_Q15_COLD and TEMP_Q15_HOT map
//            to the bottom and top of a fixed vertical axis).
//          - The bar is rendered in COL2 with a 3-zone color split:
//                cold → blue-ish, mid → green, hot → red.
//          - A coarse 8-bit temperature code (temp_code_pix) drives a
//            3-digit 5×7 font ("TEMP" numeric value) near the bar.
//          - TEMP_Q15_COLD/HOT are aligned with the Q1.15 transfer function
//            of the XADC temperature sensor status registers. :contentReference[oaicite:0]{index=0}
//
//     4) Fan duty bar + numeric + gauge arc:
//          - duty_q15 is Q1.15 (0..1.0) duty, clamped to [0..100%].
//          - A vertical duty bar in COL3 is filled with green intensity
//            proportional to duty.
//          - A 3-digit "0–100" 5×7 numeric readout sits above the duty bar.
//          - A 2×4 “arc” of segments in ROW6 (cols2–3) lights progressively
//            with duty_pct_0_100, providing an at-a-glance speed gauge.
//
//     5) Fan mode tiles (ROW1):
//          - 4 tiles: TEMP / MAN / PIR / FAN.
//          - Each tile encodes both *enable* (slide switch) and *active*
//            (fan_*_on / fan_on) state using a color scheme:
//                * Disabled channel        → dim gray.
//                * Enabled, not active     → dim colored tile.
//                * Enabled, active         → bright colored tile.
//          - “FAN” tile reflects the aggregated fan_on_pix state.
//
//     6) Temperature tick labels and markers:
//          - Three labeled ticks ("60F", "75F", "90F") are printed to the
//            left of the temperature bar using character glyphs.
//          - Tiny horizontal bars mark the corresponding temperatures on the
//            vertical axis, sharing the same BAR_BOTTOM_Y / TEMP_AXIS_TOP_Y.
//
//     7) UART telemetry HUD (ROW3 + footer):
//          - UART bit tiles: 8 tiles in ROW3 over columns 0–1 show the last
//            *accepted* UART byte’s bits [7:0] as bright cyan (1) or dim gray
//            (0); this mirrors the stream bytes flowing to the PC.
//          - UART hex counter: lower 16 bits of uart_byte_count (4 hex digits)
//            are rendered via the 5×7 font under the bit tiles.
//          - UART footer band: a color-coded horizontal bar at the bottom of
//            the panel with length proportional to uart_level (decays per
//            frame). The bar color is derived from uart_byte_pix.
//
//     8) PIR “motion streak” HUD (ROW3, cols2–3):
//          - pir_rise kicks pir_level to 0xFF; pir_level decays once per frame.
//          - A horizontal band spanning COL2..COL3 with width proportional to
//            pir_level is drawn in green/yellow, giving a decaying “motion
//            streak” visualization of PIR events.
//
//     9) Angle indicator (heading compass):
//          - theta_q15 (Q1.15 turns) is converted to integer degrees [0..359].
//          - A 3-digit 5×7 numeric heading (000–359) is rendered in ROW4,COL0.
//          - A vertical 4-tile “compass stack” (N/E/S/W) in ROW4,COL3 gives a
//            directional highlight: the active tile is bright white; others
//            are light gray.
//
//    10) Rotary encoder “spinner bar” HUD:
//          - enc_pos is a signed 16-bit position; |enc_pos| controls the
//            height of a vertical bar near the far right (COL3).
//          - enc_dir selects color hue (CW→greenish, CCW→reddish/magenta).
//          - rot_level is pumped by motion (step or position change) and
//            decays per frame, providing a brightness “trail” on the bar.
//
//    11) Status tiles + legend:
//          - Row2 status tiles (TOF/TMP/SRV/SYS) summarize system health:
//                 TOF : UART/ToF streaming alive (uart_level>0).
//                 TMP : temperature in “safe” band (not in hot segment).
//                 SRV : survey/rotary activity (rot_level>0).
//                 SYS : aggregate of fan_on, PIR activity, and UART activity.
//          - Row0 legend band (“TOF / TMP / SRV / SYS”) is rendered using
//            the same 4 columns, with colored background stripes in COL0..3
//            and white glyphs centered within.
//
//    12) Color layering / priority:
//          - Default base: rgb_bg from vga_range_plot_top.
//          - Then legend stripes, fan tiles, status tiles, gauge segments,
//            numeric glyphs, PIR streak, UART tiles, compass, spinner bar,
//            and labels are drawn in a deterministic top-down order so that
//            text and indicators sit cleanly on top of the background.
//
// Parameters
// ----------------------------------------------------------------------------
//   TEMP_Q15_COLD / TEMP_Q15_HOT
//     - Define the Q1.15 temperature codes that map to the bottom (cold) and
//       top (hot) of the temperature bar. They should be consistent with the
//       XADC’s temperature-sensor transfer function and any preceding scaling
//       in the XADC reader.
//
// Clock / reset domains
// ----------------------------------------------------------------------------
//   - Pixel domain: pix_clk, rst (active-high, pixel domain).
//        * Owns hcount, vcount, active_video, frame_tick, rgb_bg.
//        * Owns all HUD rendering, glyph generation, and color composition.
//   - System domain (100 MHz, not visible here):
//        * sample_q15, duty_q15, fan_*, uart_*, byte_accepted, theta_q15,
//          pir_*, enc_* are all generated in the system domain and must be
//          clean and synchronous there before entering this module.
//
// CDC notes
// ----------------------------------------------------------------------------
//   - Each slow control signal (fan bits, switches, sample_q15, duty_q15,
//     theta_q15, uart_tx_vld/byte, byte_accepted, pir signals, encoder
//     position/dir/step) is:
//        1) Passed through a 2-flop synchronizer into pix_clk.
//        2) Snapshotted on frame_tick where frame coherency matters (e.g.,
//           temp, duty, heading, fan bits, enables, last accepted byte).
//   - Activity-like pulses (uart_tx_vld, byte_accepted, pir_rise,
//     enc_step_pulse) are edge-detected in pix_clk to form one-cycle pulses
//     for decaying intensity meters (uart_level, pir_level, rot_level).
//
// External dependencies
// ----------------------------------------------------------------------------
//   - vga_char_glyph
//        * Simple 8×8-ish bitmap font renderer instanced repeatedly to draw
//          "TEMP", "MAN", "PIR", "FAN", "TOF", "TMP", "SRV", "SYS",
//          numeric tick labels "60F/75F/90F", and other legend text.
//   - Upstream module vga_range_plot_top
//        * Provides pix_clk, rst_pix (→ rst here), hcount, vcount,
//          active_video, frame_tick, and rgb_plot (as rgb_bg here).
//
// ============================================================================

module vga_status_overlay #(
    // Temperature range in Q1.15 mapped to full bar height
    parameter signed [15:0] TEMP_Q15_COLD = 16'sd4096,   // ~60°F equivalent
    parameter signed [15:0] TEMP_Q15_HOT  = 16'sd16384   // ~90°F equivalent
)(
    input  wire        pix_clk,
    input  wire        rst,

    // Pixel location + active-video flag
    input  wire [9:0]  hcount,
    input  wire [9:0]  vcount,
    input  wire        active_video,
    input  wire        frame_tick,   // 1-cycle pulse at top-left (pix_clk)

    // Background RGB (ToF plot result). 4:4:4 = {R[3:0],G[3:0],B[3:0]}
    input  wire [11:0] rgb_bg,

    // Temperature (Q1.15), duty (Q1.15) and fan status bits (system domain)
    input  wire  signed [15:0] sample_q15,   // sys domain (temperature code)
    input  wire  signed [15:0] duty_q15,     // sys domain (final fan duty Q1.15)
    input  wire         fan_temp_on,         // sys domain
    input  wire         fan_manual_on,       // sys domain
    input  wire         fan_pir_on,          // sys domain
    input  wire         fan_on,              // sys domain

    // UART activity taps (system domain, tx_vld is a one-cycle pulse there)
    input  wire [7:0]   uart_tx_byte,        // sys domain
    input  wire         uart_tx_vld,         // sys domain

    // Accepted-byte strobe from UART path (sys domain)
    input  wire         byte_accepted,       // tx_vld && tx_rdy in sys domain

    // Angle & PIR telemetry (system domain)
    input  wire  signed [15:0] theta_q15,    // sys domain, Q1.15 turns
    input  wire         pir_active,          // sys domain level
    input  wire         pir_rise,            // sys domain, 1-cycle pulse

    // Rotary encoder telemetry (system domain)
    input  wire  signed [15:0] enc_pos,         // signed position counter
    input  wire                enc_step_pulse,  // 1-cycle per detent (sys)
    input  wire                enc_dir,         // 1=CW, 0=CCW (sys)

    // Slide switch enables (system domain, optional)
    input  wire        sw_temp_en,      // enable TEMP-based cooling
    input  wire        sw_manual_en,    // enable MANUAL cooling
    input  wire        sw_pir_en,       // enable PIR-based cooling

    // Final RGB output after overlay
    output reg  [11:0] rgb_out
);

    // ------------------------------------------------------------------------
    // 0) CDC: synchronize slow status signals into pix_clk domain
    // ------------------------------------------------------------------------
    // Fan bits: 2-FF sync, then latched once per frame for display
    reg fan_temp_s0,   fan_temp_s1;
    reg fan_manual_s0, fan_manual_s1;
    reg fan_pir_s0,    fan_pir_s1;
    reg fan_on_s0,     fan_on_s1;

    // Slide switch enables: 2-FF sync + frame snapshot
    reg sw_temp_s0,   sw_temp_s1,   sw_temp_pix;
    reg sw_manual_s0, sw_manual_s1, sw_manual_pix;
    reg sw_pir_s0,    sw_pir_s1,    sw_pir_pix;

    reg fan_temp_pix;
    reg fan_manual_pix;
    reg fan_pir_pix;
    reg fan_on_pix;

    // Temperature: 2-FF bus sync, then latched once per frame
    reg signed [15:0] sample_s0;
    reg signed [15:0] sample_s1;
    reg signed [15:0] sample_q15_pix;

    // Duty: 2-FF bus sync, then latched once per frame
    reg signed [15:0] duty_s0;
    reg signed [15:0] duty_s1;
    reg signed [15:0] duty_q15_pix;

    // UART: 2-FF pulse sync + byte sync, then edge detect in pix_clk domain
    reg        tx_vld_s0, tx_vld_s1;
    reg        tx_vld_prev;
    wire       tx_vld_sync;
    wire       tx_vld_rise;

    reg  [7:0] tx_byte_s0, tx_byte_s1;
    reg  [7:0] uart_byte_pix;            // last transmitted byte in pix domain
    reg  [7:0] uart_byte_accepted_pix;   // last *accepted* byte in pix domain

    assign tx_vld_sync = tx_vld_s1;
    assign tx_vld_rise = tx_vld_sync & ~tx_vld_prev;  // 1-cycle pulse in pix_clk

    // Theta: 2-FF sync + frame snapshot
    reg signed [15:0] theta_s0;
    reg signed [15:0] theta_s1;
    reg signed [15:0] theta_q15_pix;
       // Frame-stable integer heading (0..359°) derived from Q1.15 turns
    reg [8:0]        theta_deg_pix;
    
    
    // PIR: 2-FF sync for level + edge detect in pix_clk domain
    reg pir_act_s0, pir_act_s1;
    reg pir_rise_s0, pir_rise_s1;
    reg pir_rise_prev;
    wire pir_rise_pix;

    assign pir_rise_pix = pir_rise_s1 & ~pir_rise_prev;

    // byte_accepted: 2-FF sync + edge detect for counters/activity
    reg ba_s0, ba_s1, ba_prev;
    wire ba_sync  = ba_s1;
    wire ba_rise  = ba_sync & ~ba_prev;

    // UART byte counter in pix domain
    reg [31:0] uart_byte_count;

    // Temperature code snapshot for numeric display
    reg [7:0] temp_code_pix;

    // PIR streak intensity
    reg [7:0] pir_level;

    // Rotary encoder CDC into pixel domain
    reg  signed [15:0] enc_pos_s0, enc_pos_s1, enc_pos_pix, enc_pos_prev;
    reg                enc_dir_s0, enc_dir_s1, enc_dir_pix;
    reg                enc_step_s0, enc_step_s1, enc_step_prev;
    wire               enc_step_pix_pulse;

    assign enc_step_pix_pulse = enc_step_s1 & ~enc_step_prev;

    // Rotary activity level (0..255) – for brightness of spinner bar
    reg [7:0] rot_level;
    // Helper: convert Q1.15 turns → integer degrees 0..359 for display/compass
    function [8:0] q15_to_deg;
        input signed [15:0] th_in;
        reg   signed [15:0] th_clamped;
        reg   [31:0]        mult;
        begin
            th_clamped = th_in;
            if (th_clamped < 0)
                th_clamped = 16'sd0;
            else if (th_clamped > 16'sh7FFF)
                th_clamped = 16'sh7FFF;

            mult       = th_clamped * 9'd360;   // Q1.15 * 360
            q15_to_deg = mult >> 15;            // → 0..359
        end
    endfunction




    always @(posedge pix_clk) begin
        if (rst) begin
            // fan bits sync
            fan_temp_s0    <= 1'b0; fan_temp_s1    <= 1'b0;
            fan_manual_s0  <= 1'b0; fan_manual_s1  <= 1'b0;
            fan_pir_s0     <= 1'b0; fan_pir_s1     <= 1'b0;
            fan_on_s0      <= 1'b0; fan_on_s1      <= 1'b0;
            fan_temp_pix   <= 1'b0;
            fan_manual_pix <= 1'b0;
            fan_pir_pix    <= 1'b0;
            fan_on_pix     <= 1'b0;

            // temp sync
            sample_s0       <= 16'sd0;
            sample_s1       <= 16'sd0;
            sample_q15_pix  <= 16'sd0;
            temp_code_pix   <= 8'd0;

            // duty sync
            duty_s0         <= 16'sd0;
            duty_s1         <= 16'sd0;
            duty_q15_pix    <= 16'sd0;

            // UART sync
            tx_vld_s0    <= 1'b0;
            tx_vld_s1    <= 1'b0;
            tx_vld_prev  <= 1'b0;
            tx_byte_s0   <= 8'h00;
            tx_byte_s1   <= 8'h00;
            uart_byte_pix<= 8'h00;

            // Theta sync
            theta_s0       <= 16'sd0;
            theta_s1       <= 16'sd0;
            theta_q15_pix  <= 16'sd0;
            theta_deg_pix  <= 9'd0;


            // PIR sync & streak
            pir_act_s0     <= 1'b0;
            pir_act_s1     <= 1'b0;
            pir_rise_s0    <= 1'b0;
            pir_rise_s1    <= 1'b0;
            pir_rise_prev  <= 1'b0;
            pir_level      <= 8'd0;

            // byte_accepted sync & counter
            ba_s0          <= 1'b0;
            ba_s1          <= 1'b0;
            ba_prev        <= 1'b0;
            uart_byte_count<= 32'd0;
            uart_byte_accepted_pix <= 8'h00;

            // rotary encoder CDC + activity
            enc_pos_s0     <= 16'sd0;
            enc_pos_s1     <= 16'sd0;
            enc_pos_pix    <= 16'sd0;
            enc_pos_prev   <= 16'sd0;
            enc_dir_s0     <= 1'b0;
            enc_dir_s1     <= 1'b0;
            enc_dir_pix    <= 1'b0;
            enc_step_s0    <= 1'b0;
            enc_step_s1    <= 1'b0;
            enc_step_prev  <= 1'b0;
            rot_level      <= 8'd0;

            // switch sync snapshots
            sw_temp_s0     <= 1'b0; sw_temp_s1   <= 1'b0; sw_temp_pix   <= 1'b0;
            sw_manual_s0   <= 1'b0; sw_manual_s1 <= 1'b0; sw_manual_pix <= 1'b0;
            sw_pir_s0      <= 1'b0; sw_pir_s1    <= 1'b0; sw_pir_pix    <= 1'b0;
        end else begin
            // --- fan bits ---
            fan_temp_s0   <= fan_temp_on;
            fan_temp_s1   <= fan_temp_s0;
            fan_manual_s0 <= fan_manual_on;
            fan_manual_s1 <= fan_manual_s0;
            fan_pir_s0    <= fan_pir_on;
            fan_pir_s1    <= fan_pir_s0;
            fan_on_s0     <= fan_on;
            fan_on_s1     <= fan_on_s0;

            // snapshot once per frame for display stability
            if (frame_tick) begin
                fan_temp_pix   <= fan_temp_s1;
                fan_manual_pix <= fan_manual_s1;
                fan_pir_pix    <= fan_pir_s1;
                fan_on_pix     <= fan_on_s1;
            end

            // --- slide switch enables ---
            sw_temp_s0   <= sw_temp_en;
            sw_temp_s1   <= sw_temp_s0;
            sw_manual_s0 <= sw_manual_en;
            sw_manual_s1 <= sw_manual_s0;
            sw_pir_s0    <= sw_pir_en;
            sw_pir_s1    <= sw_pir_s0;

            if (frame_tick) begin
                sw_temp_pix   <= sw_temp_s1;
                sw_manual_pix <= sw_manual_s1;
                sw_pir_pix    <= sw_pir_s1;
            end

            // --- temperature Q1.15 ---
            sample_s0 <= sample_q15;
            sample_s1 <= sample_s0;
            if (frame_tick) begin
                sample_q15_pix <= sample_s1;
                temp_code_pix  <= sample_s1[15:8]; // coarse 0..255 for digits
            end

            // --- duty Q1.15 ---
            duty_s0 <= duty_q15;
            duty_s1 <= duty_s0;
            if (frame_tick) begin
                duty_q15_pix <= duty_s1;
            end

            // --- UART pulse + byte ---
            tx_vld_s0   <= uart_tx_vld;
            tx_vld_s1   <= tx_vld_s0;
            tx_vld_prev <= tx_vld_sync;

            tx_byte_s0  <= uart_tx_byte;
            tx_byte_s1  <= tx_byte_s0;

            // On a synchronized rising edge, latch the byte into pix domain
            if (tx_vld_rise) begin
                uart_byte_pix <= tx_byte_s1;
            end

            // --- Theta ---
            theta_s0 <= theta_q15;
            theta_s1 <= theta_s0;
            if (frame_tick) begin
                theta_q15_pix <= theta_s1;
                // Frame-stable integer heading for digits + compass
                theta_deg_pix <= q15_to_deg(theta_s1);
            end


            // --- PIR level & rise ---
            pir_act_s0    <= pir_active;
            pir_act_s1    <= pir_act_s0;

            pir_rise_s0   <= pir_rise;
            pir_rise_s1   <= pir_rise_s0;
            pir_rise_prev <= pir_rise_s1;

            // PIR streak: kick on rise, decay per frame
            if (pir_rise_pix) begin
                pir_level <= 8'hFF;
            end else if (frame_tick && pir_level != 8'd0) begin
                pir_level <= pir_level - 8'd4; // decay speed
            end

            // --- byte_accepted sync & counter ---
            ba_s0   <= byte_accepted;
            ba_s1   <= ba_s0;
            ba_prev <= ba_sync;

            if (ba_rise) begin
                // Count bytes and snapshot the last *accepted* byte
                uart_byte_count        <= uart_byte_count + 32'd1;
                uart_byte_accepted_pix <= tx_byte_s1;
            end

            // --- rotary encoder CDC + activity ------------------------------
            enc_pos_s0    <= enc_pos;
            enc_pos_s1    <= enc_pos_s0;
            enc_pos_pix   <= enc_pos_s1;

            enc_dir_s0    <= enc_dir;
            enc_dir_s1    <= enc_dir_s0;
            enc_dir_pix   <= enc_dir_s1;

            enc_step_s0   <= enc_step_pulse;
            enc_step_s1   <= enc_step_s0;
            enc_step_prev <= enc_step_s1;

            if ((enc_pos_pix != enc_pos_prev) || enc_step_pix_pulse) begin
                rot_level <= 8'hFF;
            end else if (frame_tick && rot_level != 8'd0) begin
                rot_level <= rot_level - 8'd4;   // decay per frame
            end

            enc_pos_prev <= enc_pos_pix;
        end
    end

    // ------------------------------------------------------------------------
    // 1) Simple color constants (4-bit per channel)
    // ------------------------------------------------------------------------
    localparam [11:0] COL_BLACK   = 12'h000;
    localparam [11:0] COL_RED     = 12'hF00;
    localparam [11:0] COL_DARKRED = 12'h400;
    localparam [11:0] COL_GREEN   = 12'h0F0;
    localparam [11:0] COL_DARKGRN = 12'h060;
    localparam [11:0] COL_GRAY    = 12'h444;
    localparam [11:0] COL_LIGHT   = 12'h888;
    localparam [11:0] COL_TEXT    = 12'hFFF; // white text
    localparam [11:0] COL_CYAN    = 12'h0FF; // UART counters / bits
    localparam [11:0] COL_YELLOW  = 12'hFF0;
    localparam [11:0] COL_BLUE    = 12'h00F;
    localparam [11:0] COL_MAGENTA = 12'hF0F;

    localparam [11:0] COL_DIMGRAY   = 12'h222;
    localparam [11:0] COL_ORANGE    = 12'hF80;
    localparam [11:0] COL_DARKORNG  = 12'h630;
    localparam [11:0] COL_BLUEBRITE = 12'h04F;
    localparam [11:0] COL_BLUEDIM   = 12'h027;
    localparam [11:0] COL_GREENDIM  = 12'h050;

    // ------------------------------------------------------------------------
    // 2) Grid geometry for right panel (4 columns × N rows)
    // ------------------------------------------------------------------------
    localparam [9:0] PANEL_X_L = 10'd320;
    localparam [9:0] PANEL_X_R = 10'd639;
    localparam [9:0] PANEL_Y_T = 10'd16;
    localparam [9:0] CELL_H    = 10'd32;   // normal row height
    localparam [9:0] LEGEND_H  = 10'd16;   // *short* legend row height
    localparam [9:0] CELL_W    = 10'd80;

    // Columns [0..3]
    localparam [9:0] COL0_L = PANEL_X_L + 10'd0*CELL_W;
    localparam [9:0] COL0_R = PANEL_X_L + 10'd1*CELL_W - 1;
    localparam [9:0] COL1_L = PANEL_X_L + 10'd1*CELL_W;
    localparam [9:0] COL1_R = PANEL_X_L + 10'd2*CELL_W - 1;
    localparam [9:0] COL2_L = PANEL_X_L + 10'd2*CELL_W;
    localparam [9:0] COL2_R = PANEL_X_L + 10'd3*CELL_W - 1;
    localparam [9:0] COL3_L = PANEL_X_L + 10'd3*CELL_W;
    localparam [9:0] COL3_R = PANEL_X_L + 10'd4*CELL_W - 1;

    // Rows
    localparam [9:0] ROW0_T = PANEL_Y_T;
    localparam [9:0] ROW0_B = PANEL_Y_T + LEGEND_H - 1;  // legend (shorter)
    
    localparam [9:0] ROW1_T = ROW0_B + 1;
    localparam [9:0] ROW1_B = ROW1_T + CELL_H - 1;       // fan mode tiles
    
    localparam [9:0] ROW2_T = ROW1_B + 1;
    localparam [9:0] ROW2_B = ROW2_T + CELL_H - 1;       // status bits
    
    localparam [9:0] ROW3_T = ROW2_B + 1;
    localparam [9:0] ROW3_B = ROW3_T + CELL_H - 1;       // UART hex + PIR streak
    
    localparam [9:0] ROW4_T = ROW3_B + 1;
    localparam [9:0] ROW4_B = ROW4_T + CELL_H - 1;       // compass + heading digits
    
    localparam [9:0] ROW5_T = ROW4_B + 1;
    localparam [9:0] ROW5_B = ROW5_T + CELL_H - 1;       // temp/duty digits
    
    localparam [9:0] ROW6_T = ROW5_B + 1;
    localparam [9:0] ROW6_B = ROW6_T + CELL_H - 1;       // fan gauge, etc. 

    // Helper wires for column membership
    wire in_col0 = (hcount >= COL0_L) && (hcount <= COL0_R);
    wire in_col1 = (hcount >= COL1_L) && (hcount <= COL1_R);
    wire in_col2 = (hcount >= COL2_L) && (hcount <= COL2_R);
    wire in_col3 = (hcount >= COL3_L) && (hcount <= COL3_R);

    // ------------------------------------------------------------------------
    // 3) UART activity: decaying level (0..255) in pix_clk domain
    // ------------------------------------------------------------------------
    reg [7:0] uart_level = 8'd0;

    always @(posedge pix_clk) begin
        if (rst) begin
            uart_level <= 8'd0;
        end else begin
            if (tx_vld_rise) begin
                uart_level <= 8'hFF;
            end else if (frame_tick && (uart_level != 8'd0)) begin
                uart_level <= uart_level - 8'd1;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 4) Temperature & duty bar heights
    // ------------------------------------------------------------------------
    localparam [9:0] BAR_BOTTOM_Y = 10'd440;  // footer band / bar baseline

    // Temperature bar geometry (right-panel column 2)
    localparam integer TEMP_BAR_H      = 10'd160;               // px of vertical range
    localparam [9:0]   TEMP_AXIS_TOP_Y = BAR_BOTTOM_Y - TEMP_BAR_H + 1;

    // Temperature range in Q1.15
    wire signed [16:0] temp_range_q15 = TEMP_Q15_HOT - TEMP_Q15_COLD;

    // Signed delta from "cold" endpoint
    wire signed [16:0] temp_delta_raw =
        $signed(sample_q15_pix) - $signed(TEMP_Q15_COLD);

    // Clamp into [0 .. temp_range_q15]
    wire [16:0] temp_delta_clamped =
        (temp_delta_raw <= 0)              ? 17'd0 :
        (temp_delta_raw >= temp_range_q15) ? temp_range_q15[16:0] :
                                             temp_delta_raw[16:0];

    // Scale into 0..TEMP_BAR_H using a 32-bit multiply and integer divide
    wire [31:0] temp_mult = temp_delta_clamped * TEMP_BAR_H;

    wire [7:0] temp_height_raw =
        (temp_range_q15 != 0) ? temp_mult / temp_range_q15[16:0] : 8'd0;

    wire [7:0] temp_height =
        (temp_height_raw > (TEMP_BAR_H-1)) ? (TEMP_BAR_H-1) : temp_height_raw;

    // Y coordinate where the filled bar begins (higher temp → bar grows upward)
    wire [9:0] temp_bar_top_y = BAR_BOTTOM_Y - temp_height;

    // Simple three-zone color classification based on bar fill fraction
    localparam integer TEMP_SEG = TEMP_BAR_H / 3;

    wire [7:0] temp_fraction = temp_height;  // 0..TEMP_BAR_H-1
    wire       temp_cold  = (temp_fraction < TEMP_SEG);
    wire       temp_mid   = (temp_fraction >= TEMP_SEG) &&
                            (temp_fraction < 2*TEMP_SEG);
    wire       temp_hot   = (temp_fraction >= 2*TEMP_SEG);

    // Duty uses straightforward clamped Q1.15 mapping
    wire signed [15:0] q15_duty_raw = duty_q15_pix;

    wire [15:0] q15_duty_clamped =
        q15_duty_raw[15]           ? 16'd0    :
        (q15_duty_raw > 16'h7FFF)  ? 16'h7FFF :
                                     q15_duty_raw;

    wire [7:0] duty_pix = q15_duty_clamped[14:7];

    wire [9:0] duty_bar_top_y = (BAR_BOTTOM_Y > {2'b00, duty_pix}) ?
                                (BAR_BOTTOM_Y - {2'b00, duty_pix}) :
                                TEMP_AXIS_TOP_Y;

    // Duty percent 0..100 (re-used for digits and fan gauge)
    wire [7:0] duty_pct_raw = q15_duty_clamped[15:8];
    wire [6:0] duty_pct_0_100 =
        (duty_pct_raw > 8'd100) ? 7'd100 : duty_pct_raw[6:0];

    // ------------------------------------------------------------------------
    // 5) Fan mode tiles (ROW1, 4 equal slices across columns)
    // ------------------------------------------------------------------------
    wire in_fan_row = (vcount >= ROW1_T) && (vcount <= ROW1_B);

    wire in_box_temp = in_fan_row && in_col1;   // TEMP channel (COL1)
    wire in_box_man  = in_fan_row && in_col0;   // MANUAL channel (COL0)
    wire in_box_pir  = in_fan_row && in_col2;   // PIR channel (COL2)
    wire in_box_fan  = in_fan_row && in_col3;   // FAN aggregate (COL3)

    // ------------------------------------------------------------------------
    // Fan tile text labels ("TEMP","MAN","PIR","FAN") in ROW1
    // ------------------------------------------------------------------------
    localparam [9:0] FAN_LABEL_Y = ROW1_T + 10'd8;

    // TEMP over col1 ("TEMP")
    wire temp_lbl_T, temp_lbl_E, temp_lbl_M, temp_lbl_P;
    vga_char_glyph u_lbl_temp_T (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL1_L + 10'd8),
        .y0           (FAN_LABEL_Y),
        .char_code    ("T"),
        .pixel_on     (temp_lbl_T)
    );
    vga_char_glyph u_lbl_temp_E (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL1_L + 10'd16),
        .y0           (FAN_LABEL_Y),
        .char_code    ("E"),
        .pixel_on     (temp_lbl_E)
    );
    vga_char_glyph u_lbl_temp_M (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL1_L + 10'd24),
        .y0           (FAN_LABEL_Y),
        .char_code    ("M"),
        .pixel_on     (temp_lbl_M)
    );
    vga_char_glyph u_lbl_temp_P (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL1_L + 10'd32),
        .y0           (FAN_LABEL_Y),
        .char_code    ("P"),
        .pixel_on     (temp_lbl_P)
    );

    // MAN over col0
    wire man_lbl_M, man_lbl_A, man_lbl_N;
    vga_char_glyph u_lbl_man_M (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL0_L + 10'd8),
        .y0           (FAN_LABEL_Y),
        .char_code    ("M"),
        .pixel_on     (man_lbl_M)
    );
    vga_char_glyph u_lbl_man_A (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL0_L + 10'd16),
        .y0           (FAN_LABEL_Y),
        .char_code    ("A"),
        .pixel_on     (man_lbl_A)
    );
    vga_char_glyph u_lbl_man_N (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL0_L + 10'd24),
        .y0           (FAN_LABEL_Y),
        .char_code    ("N"),
        .pixel_on     (man_lbl_N)
    );

    // PIR over col2
    wire pir_lbl_P, pir_lbl_I, pir_lbl_R;
    vga_char_glyph u_lbl_pir_P (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL2_L + 10'd8),
        .y0           (FAN_LABEL_Y),
        .char_code    ("P"),
        .pixel_on     (pir_lbl_P)
    );
    vga_char_glyph u_lbl_pir_I (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL2_L + 10'd16),
        .y0           (FAN_LABEL_Y),
        .char_code    ("I"),
        .pixel_on     (pir_lbl_I)
    );
    vga_char_glyph u_lbl_pir_R (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL2_L + 10'd24),
        .y0           (FAN_LABEL_Y),
        .char_code    ("R"),
        .pixel_on     (pir_lbl_R)
    );

    // FAN over col3
    wire fan_lbl_F, fan_lbl_A, fan_lbl_N;
    vga_char_glyph u_lbl_fan_F (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL3_L + 10'd8),
        .y0           (FAN_LABEL_Y),
        .char_code    ("F"),
        .pixel_on     (fan_lbl_F)
    );
    vga_char_glyph u_lbl_fan_A (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL3_L + 10'd16),
        .y0           (FAN_LABEL_Y),
        .char_code    ("A"),
        .pixel_on     (fan_lbl_A)
    );
    vga_char_glyph u_lbl_fan_N (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (COL3_L + 10'd24),
        .y0           (FAN_LABEL_Y),
        .char_code    ("N"),
        .pixel_on     (fan_lbl_N)
    );

    wire fan_text_on =
        temp_lbl_T | temp_lbl_E | temp_lbl_M | temp_lbl_P |
        man_lbl_M  | man_lbl_A  | man_lbl_N  |
        pir_lbl_P  | pir_lbl_I  | pir_lbl_R  |
        fan_lbl_F  | fan_lbl_A  | fan_lbl_N;

    // ------------------------------------------------------------------------
    // 6) UART activity footer band (bottom, under grid)
    // ------------------------------------------------------------------------
    // A short bar just above the compass / heading row, visually grouped
    // with the UART bit tiles and hex counter instead of at the bottom.
    localparam [9:0] UART_BAR_Y_TOP = ROW4_T + 10'd2;
    localparam [9:0] UART_BAR_Y_BOT = UART_BAR_Y_TOP + 10'd7; // 8 px tall

    localparam [9:0] UART_BAR_X_L   = PANEL_X_L;

    wire in_uart_bar_y =
        (vcount >= UART_BAR_Y_TOP) && (vcount <= UART_BAR_Y_BOT);

    wire [9:0] uart_bar_x_r =
        (UART_BAR_X_L + {2'b00, uart_level}) > (10'd639>>1) ?
        (10'd639>>1) : (UART_BAR_X_L + {2'b00, uart_level});

    wire in_uart_bar =
        in_uart_bar_y &&
        (hcount >= UART_BAR_X_L) &&
        (hcount <= uart_bar_x_r);

    // ------------------------------------------------------------------------
    // 7) Temperature / duty vertical bars (aligned to columns 2 & 3)
    // ------------------------------------------------------------------------
    localparam [9:0] TEMP_BAR_X_L = COL2_L + 10'd8;
    localparam [9:0] TEMP_BAR_X_R = COL2_L + 10'd27;

    wire in_temp_bar_x =
        (hcount >= TEMP_BAR_X_L) && (hcount <= TEMP_BAR_X_R);
    wire in_temp_bar_y =
        (vcount >= temp_bar_top_y) && (vcount <= BAR_BOTTOM_Y);
    wire in_temp_bar = in_temp_bar_x && in_temp_bar_y;

    localparam [9:0] DUTY_BAR_X_L = COL3_L + 10'd8;
    localparam [9:0] DUTY_BAR_X_R = COL3_L + 10'd27;

    wire in_duty_bar_x =
        (hcount >= DUTY_BAR_X_L) && (hcount <= DUTY_BAR_X_R);
    wire in_duty_bar_y =
        (vcount >= duty_bar_top_y) && (vcount <= BAR_BOTTOM_Y);
    wire in_duty_bar = in_duty_bar_x && in_duty_bar_y;

    // ------------------------------------------------------------------------
    // 8) Fan speed gauge arc (2×4 segments in row6 occupying cols2–3)
    // ------------------------------------------------------------------------
    localparam integer FAN_SEG_COUNT = 8;
    wire [3:0] fan_seg_limit = (duty_pct_0_100 * FAN_SEG_COUNT) / 7'd100;

    localparam [9:0] FAN_Y_TOP1 = ROW6_T + 10'd4;
    localparam [9:0] FAN_Y_BOT1 = FAN_Y_TOP1 + 10'd9;
    localparam [9:0] FAN_Y_TOP2 = FAN_Y_TOP1 + 10'd10;
    localparam [9:0] FAN_Y_BOT2 = FAN_Y_TOP2 + 10'd9;

    // four segments per row within cols2–3
    localparam [9:0] FAN_X0_L = COL3_L + 10'd24,  FAN_X0_R = FAN_X0_L + 10'd7;
    localparam [9:0] FAN_X1_L = FAN_X0_R + 10'd4, FAN_X1_R = FAN_X1_L + 10'd7;
    localparam [9:0] FAN_X2_L = FAN_X1_R + 10'd4, FAN_X2_R = FAN_X2_L + 10'd7;
    localparam [9:0] FAN_X3_L = FAN_X2_R + 10'd4, FAN_X3_R = FAN_X3_L + 10'd7;

    localparam [9:0] FAN_X4_L = FAN_X0_L;
    localparam [9:0] FAN_X4_R = FAN_X0_R;
    localparam [9:0] FAN_X5_L = FAN_X1_L;
    localparam [9:0] FAN_X5_R = FAN_X1_R;
    localparam [9:0] FAN_X6_L = FAN_X2_L;
    localparam [9:0] FAN_X6_R = FAN_X2_R;
    localparam [9:0] FAN_X7_L = FAN_X3_L;
    localparam [9:0] FAN_X7_R = FAN_X3_R;

    wire seg0_px = (vcount>=FAN_Y_TOP1 && vcount<=FAN_Y_BOT1 &&
                    hcount>=FAN_X0_L   && hcount<=FAN_X0_R);
    wire seg1_px = (vcount>=FAN_Y_TOP1 && vcount<=FAN_Y_BOT1 &&
                    hcount>=FAN_X1_L   && hcount<=FAN_X1_R);
    wire seg2_px = (vcount>=FAN_Y_TOP1 && vcount<=FAN_Y_BOT1 &&
                    hcount>=FAN_X2_L   && hcount<=FAN_X2_R);
    wire seg3_px = (vcount>=FAN_Y_TOP1 && vcount<=FAN_Y_BOT1 &&
                    hcount>=FAN_X3_L   && hcount<=FAN_X3_R);
    wire seg4_px = (vcount>=FAN_Y_TOP2 && vcount<=FAN_Y_BOT2 &&
                    hcount>=FAN_X4_L   && hcount<=FAN_X4_R);
    wire seg5_px = (vcount>=FAN_Y_TOP2 && vcount<=FAN_Y_BOT2 &&
                    hcount>=FAN_X5_L   && hcount<=FAN_X5_R);
    wire seg6_px = (vcount>=FAN_Y_TOP2 && vcount<=FAN_Y_BOT2 &&
                    hcount>=FAN_X6_L   && hcount<=FAN_X6_R);
    wire seg7_px = (vcount>=FAN_Y_TOP2 && vcount<=FAN_Y_BOT2 &&
                    hcount>=FAN_X7_L   && hcount<=FAN_X7_R);

    // ------------------------------------------------------------------------
    // 9) Digit glyph ROM (5x7) for temperature / duty / UART / theta
    // ------------------------------------------------------------------------
    function [34:0] digit_5x7_bitmap;
        input [3:0] d;
        begin
            case (d)
                4'd0: digit_5x7_bitmap = 35'b01110_10001_10011_10101_11001_10001_01110;
                4'd1: digit_5x7_bitmap = 35'b00100_01100_00100_00100_00100_00100_01110;
                4'd2: digit_5x7_bitmap = 35'b01110_10001_00001_00010_00100_01000_11111;
                4'd3: digit_5x7_bitmap = 35'b01110_10001_00001_00110_00001_10001_01110;
                4'd4: digit_5x7_bitmap = 35'b00010_00110_01010_10010_11111_00010_00010;
                4'd5: digit_5x7_bitmap = 35'b11111_10000_11110_00001_00001_10001_01110;
                4'd6: digit_5x7_bitmap = 35'b00110_01000_10000_11110_10001_10001_01110;
                4'd7: digit_5x7_bitmap = 35'b11111_00001_00010_00100_01000_01000_01000;
                4'd8: digit_5x7_bitmap = 35'b01110_10001_10001_01110_10001_10001_01110;
                4'd9: digit_5x7_bitmap = 35'b01110_10001_10001_01111_00001_00010_01100;
                default: digit_5x7_bitmap = 35'b00000_00000_00000_00000_00000_00000_00000;
            endcase
        end
    endfunction

    function digit_pixel_on;
        input [3:0] d;
        input [2:0] row;   // 0..6 (0 = top)
        input [2:0] col;   // 0..4 (0 = left)
        reg [34:0] bits;
        integer idx;
        begin
            bits = digit_5x7_bitmap(d);
            idx  = (6 - row)*5 + (4 - col);  // flip vertical & horizontal
            digit_pixel_on = bits[idx];
        end
    endfunction

    localparam integer DIG_W  = 5;
    localparam integer DIG_H  = 7;
    localparam integer DIG_SP = 1;





    // ------------------------------------------------------------------------
    // 10) Temperature numeric glyphs (3 digits) – near temp bar
    // ------------------------------------------------------------------------
    localparam integer NUM_MARGIN    = 4;
    localparam [9:0]   TEMP_TX_X_L   = COL2_L + 10'd8;
    localparam [9:0]   TEMP_TX_Y_TOP = BAR_BOTTOM_Y - (DIG_H + NUM_MARGIN);

    wire in_temp_text_box =
        (hcount >= TEMP_TX_X_L) &&
        (hcount <  TEMP_TX_X_L + 3*DIG_W + 2*DIG_SP) &&
        (vcount >= TEMP_TX_Y_TOP) &&
        (vcount <  TEMP_TX_Y_TOP + DIG_H);

    reg  [3:0] temp_d2, temp_d1, temp_d0;
    reg        temp_digit_px_on;
    reg  [3:0] temp_digit_sel;
    reg  [2:0] temp_row, temp_col;

    integer t_val;
    integer temp_xw;
    integer temp_digit_idx;
    integer temp_col_in_digit;

    always @* begin
        t_val = temp_code_pix;
        if (t_val > 999) t_val = 999;

        temp_d2 = t_val / 100;
        t_val   = t_val % 100;
        temp_d1 = t_val / 10;
        temp_d0 = t_val % 10;
    end

    always @* begin
        temp_digit_px_on = 1'b0;
        temp_digit_sel   = 4'd0;
        temp_row         = 3'd0;
        temp_col         = 3'd0;

        if (in_temp_text_box) begin
            temp_row = vcount - TEMP_TX_Y_TOP;
            temp_xw  = hcount - TEMP_TX_X_L;

            if (temp_xw < DIG_W) begin
                temp_digit_idx    = 0;
                temp_col_in_digit = temp_xw;
            end else if (temp_xw < DIG_W + DIG_SP + DIG_W) begin
                if (temp_xw < DIG_W + DIG_SP) begin
                    temp_digit_idx    = -1;
                    temp_col_in_digit = 0;
                end else begin
                    temp_digit_idx    = 1;
                    temp_col_in_digit = temp_xw - (DIG_W + DIG_SP);
                end
            end else begin
                if (temp_xw < 2*DIG_W + 2*DIG_SP) begin
                    temp_digit_idx    = -1;
                    temp_col_in_digit = 0;
                end else begin
                    temp_digit_idx    = 2;
                    temp_col_in_digit = temp_xw - (2*DIG_W + 2*DIG_SP);
                end
            end

            if (temp_digit_idx == 0) begin
                temp_digit_sel   = temp_d2;
                temp_col         = temp_col_in_digit[2:0];
                temp_digit_px_on = digit_pixel_on(temp_digit_sel,
                                                  temp_row,
                                                  temp_col);
            end else if (temp_digit_idx == 1) begin
                temp_digit_sel   = temp_d1;
                temp_col         = temp_col_in_digit[2:0];
                temp_digit_px_on = digit_pixel_on(temp_digit_sel,
                                                  temp_row,
                                                  temp_col);
            end else if (temp_digit_idx == 2) begin
                temp_digit_sel   = temp_d0;
                temp_col         = temp_col_in_digit[2:0];
                temp_digit_px_on = digit_pixel_on(temp_digit_sel,
                                                  temp_row,
                                                  temp_col);
            end
        end
    end

    // ------------------------------------------------------------------------
    // 11) Fan duty numeric glyphs (0–100 %) – near duty bar
    // ------------------------------------------------------------------------
    localparam [9:0] DUTY_TX_X_L   = COL3_L + 10'd8;
    localparam [9:0] DUTY_TX_Y_TOP = BAR_BOTTOM_Y - (DIG_H + NUM_MARGIN);
    
    wire in_duty_text_box =
        (hcount >= DUTY_TX_X_L) &&
        (hcount <  DUTY_TX_X_L + 3*DIG_W + 2*DIG_SP) &&
        (vcount >= DUTY_TX_Y_TOP) &&
        (vcount <  DUTY_TX_Y_TOP + DIG_H);

    reg  [3:0] duty_d2, duty_d1, duty_d0;
    reg        duty_digit_px_on;
    reg  [3:0] duty_digit_sel;
    reg  [2:0] duty_row, duty_col;

    integer d_val;
    integer duty_xw;
    integer duty_digit_idx;
    integer duty_col_in_digit;

    always @* begin
        d_val = duty_pct_0_100;
        if (d_val > 999) d_val = 999;

        duty_d2 = d_val / 100;
        d_val   = d_val % 100;
        duty_d1 = d_val / 10;
        duty_d0 = d_val % 10;
    end

    always @* begin
        duty_digit_px_on = 1'b0;
        duty_digit_sel   = 4'd0;
        duty_row         = 3'd0;
        duty_col         = 3'd0;

        if (in_duty_text_box) begin
            duty_row = vcount - DUTY_TX_Y_TOP;
            duty_xw  = hcount - DUTY_TX_X_L;

            if (duty_xw < DIG_W) begin
                duty_digit_idx    = 0;
                duty_col_in_digit = duty_xw;
            end else if (duty_xw < DIG_W + DIG_SP + DIG_W) begin
                if (duty_xw < DIG_W + DIG_SP) begin
                    duty_digit_idx    = -1;
                    duty_col_in_digit = 0;
                end else begin
                    duty_digit_idx    = 1;
                    duty_col_in_digit = duty_xw - (DIG_W + DIG_SP);
                end
            end else begin
                if (duty_xw < 2*DIG_W + 2*DIG_SP) begin
                    duty_digit_idx    = -1;
                    duty_col_in_digit = 0;
                end else begin
                    duty_digit_idx    = 2;
                    duty_col_in_digit = duty_xw - (2*DIG_W + 2*DIG_SP);
                end
            end

            if (duty_digit_idx == 0) begin
                duty_digit_sel   = duty_d2;
                duty_col         = duty_col_in_digit[2:0];
                duty_digit_px_on = digit_pixel_on(duty_digit_sel,
                                                  duty_row,
                                                  duty_col);
            end else if (duty_digit_idx == 1) begin
                duty_digit_sel   = duty_d1;
                duty_col         = duty_col_in_digit[2:0];
                duty_digit_px_on = digit_pixel_on(duty_digit_sel,
                                                  duty_row,
                                                  duty_col);
            end else if (duty_digit_idx == 2) begin
                duty_digit_sel   = duty_d0;
                duty_col         = duty_col_in_digit[2:0];
                duty_digit_px_on = digit_pixel_on(duty_digit_sel,
                                                  duty_row,
                                                  duty_col);
            end
        end
    end

    // ------------------------------------------------------------------------
    // 10.5) Temperature tick labels + markers (60°F / 75°F / 90°F)
    // ------------------------------------------------------------------------
    localparam [9:0] TICK_LABEL_X = TEMP_BAR_X_L - 10'd32; // "60F"/"75F"/"90F"
    localparam [9:0] TICK_BAR_X0  = TEMP_BAR_X_L - 10'd6;  // small marker bar
    localparam [9:0] TICK_BAR_X1  = TEMP_BAR_X_L - 10'd1;

    localparam [9:0] TICK_Y_60 = BAR_BOTTOM_Y - 10'd8;         // near bottom
    localparam [9:0] TICK_Y_90 = TEMP_AXIS_TOP_Y + 10'd4;      // near top
    localparam [9:0] TICK_Y_75 = (TICK_Y_60 + TICK_Y_90) >> 1; // midway

    wire tick60_c0, tick60_c1, tick60_c2;
    wire tick75_c0, tick75_c1, tick75_c2;
    wire tick90_c0, tick90_c1, tick90_c2;

    // "60F"
    vga_char_glyph u_tick60_6 (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (TICK_LABEL_X),
        .y0           (TICK_Y_60),
        .char_code    ("6"),
        .pixel_on     (tick60_c0)
    );
    vga_char_glyph u_tick60_0 (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (TICK_LABEL_X + 10'd8),
        .y0           (TICK_Y_60),
        .char_code    ("0"),
        .pixel_on     (tick60_c1)
    );
    vga_char_glyph u_tick60_F (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (TICK_LABEL_X + 10'd16),
        .y0           (TICK_Y_60),
        .char_code    ("F"),
        .pixel_on     (tick60_c2)
    );

    // "75F"
    vga_char_glyph u_tick75_7 (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (TICK_LABEL_X),
        .y0           (TICK_Y_75),
        .char_code    ("7"),
        .pixel_on     (tick75_c0)
    );
    vga_char_glyph u_tick75_5 (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (TICK_LABEL_X + 10'd8),
        .y0           (TICK_Y_75),
        .char_code    ("5"),
        .pixel_on     (tick75_c1)
    );
    vga_char_glyph u_tick75_F (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (TICK_LABEL_X + 10'd16),
        .y0           (TICK_Y_75),
        .char_code    ("F"),
        .pixel_on     (tick75_c2)
    );

    // "90F"
    vga_char_glyph u_tick90_9 (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (TICK_LABEL_X),
        .y0           (TICK_Y_90),
        .char_code    ("9"),
        .pixel_on     (tick90_c0)
    );
    vga_char_glyph u_tick90_0 (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (TICK_LABEL_X + 10'd8),
        .y0           (TICK_Y_90),
        .char_code    ("0"),
        .pixel_on     (tick90_c1)
    );
    vga_char_glyph u_tick90_F (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (TICK_LABEL_X + 10'd16),
        .y0           (TICK_Y_90),
        .char_code    ("F"),
        .pixel_on     (tick90_c2)
    );

    wire tick_text_on =
        tick60_c0 | tick60_c1 | tick60_c2 |
        tick75_c0 | tick75_c1 | tick75_c2 |
        tick90_c0 | tick90_c1 | tick90_c2;

    // Small horizontal tick bars aligned with labels
    wire tick_bar_60 = active_video &&
                       (hcount >= TICK_BAR_X0) && (hcount <= TICK_BAR_X1) &&
                       (vcount >= TICK_Y_60 + 10'd3) && (vcount <= TICK_Y_60 + 10'd4);

    wire tick_bar_75 = active_video &&
                       (hcount >= TICK_BAR_X0) && (hcount <= TICK_BAR_X1) &&
                       (vcount >= TICK_Y_75 + 10'd3) && (vcount <= TICK_Y_75 + 10'd4);

    wire tick_bar_90 = active_video &&
                       (hcount >= TICK_BAR_X0) && (hcount <= TICK_BAR_X1) &&
                       (vcount >= TICK_Y_90 + 10'd3) && (vcount <= TICK_Y_90 + 10'd4);

    wire tick_bar_on = tick_bar_60 | tick_bar_75 | tick_bar_90;

    // ------------------------------------------------------------------------
    // 12) UART byte counters + bit tiles geometry
    // ------------------------------------------------------------------------
    // Horizontal span: COL0_L..COL1_R with a small margin
    localparam [9:0] UART_BITS_X_L = COL0_L + 10'd4;
    localparam [9:0] UART_BITS_X_R = COL1_R - 10'd4;

    // Each bit tile has fixed width; last tile gets any remainder
    localparam [9:0] UART_BIT_W =
        (UART_BITS_X_R - UART_BITS_X_L + 1) / 8;

    // X ranges for each bit tile (MSB..LSB)
    localparam [9:0] UART_B7_X0 = UART_BITS_X_L + 10'd0*UART_BIT_W;
    localparam [9:0] UART_B6_X0 = UART_BITS_X_L + 10'd1*UART_BIT_W;
    localparam [9:0] UART_B5_X0 = UART_BITS_X_L + 10'd2*UART_BIT_W;
    localparam [9:0] UART_B4_X0 = UART_BITS_X_L + 10'd3*UART_BIT_W;
    localparam [9:0] UART_B3_X0 = UART_BITS_X_L + 10'd4*UART_BIT_W;
    localparam [9:0] UART_B2_X0 = UART_BITS_X_L + 10'd5*UART_BIT_W;
    localparam [9:0] UART_B1_X0 = UART_BITS_X_L + 10'd6*UART_BIT_W;
    localparam [9:0] UART_B0_X0 = UART_BITS_X_L + 10'd7*UART_BIT_W;

    localparam [9:0] UART_B7_X1 = UART_B7_X0 + UART_BIT_W - 1;
    localparam [9:0] UART_B6_X1 = UART_B6_X0 + UART_BIT_W - 1;
    localparam [9:0] UART_B5_X1 = UART_B5_X0 + UART_BIT_W - 1;
    localparam [9:0] UART_B4_X1 = UART_B4_X0 + UART_BIT_W - 1;
    localparam [9:0] UART_B3_X1 = UART_B3_X0 + UART_BIT_W - 1;
    localparam [9:0] UART_B2_X1 = UART_B2_X0 + UART_BIT_W - 1;
    localparam [9:0] UART_B1_X1 = UART_B1_X0 + UART_BIT_W - 1;
    localparam [9:0] UART_B0_X1 = UART_BITS_X_R; // ensure no right-edge gap

    wire [15:0] uart_cnt16 = uart_byte_count[15:0];

    wire [3:0] uart_hex3 = uart_cnt16[15:12];
    wire [3:0] uart_hex2 = uart_cnt16[11: 8];
    wire [3:0] uart_hex1 = uart_cnt16[ 7: 4];
    wire [3:0] uart_hex0 = uart_cnt16[ 3: 0];

    localparam integer UART_HEX_W = 4*DIG_W + 3*DIG_SP;

    localparam [9:0] UART_TX_X_L =
        UART_BITS_X_L +
        (((UART_BITS_X_R - UART_BITS_X_L + 10'd1) - UART_HEX_W) >> 1);
 
 
    // Hex byte counter glyphs in the lower half of ROW3, below bit tiles
    localparam [9:0] UART_TX_Y_TOP = ROW3_T + 10'd18;
    wire in_uart_text_box =
        (hcount >= UART_TX_X_L) &&
        (hcount <  UART_TX_X_L + UART_HEX_W) &&
        (vcount >= UART_TX_Y_TOP) &&
        (vcount <  UART_TX_Y_TOP + DIG_H);

    reg  [3:0] uart_digit_sel;
    reg        uart_digit_px_on;

    integer uart_xw;
    integer uart_row;
    integer uart_digit_idx;
    integer uart_col_in_digit;

    always @* begin
        uart_digit_px_on = 1'b0;
        uart_digit_sel   = 4'd0;

        if (in_uart_text_box) begin
            uart_xw  = hcount - UART_TX_X_L;
            uart_row = vcount - UART_TX_Y_TOP;

            uart_digit_idx    = uart_xw / (DIG_W + DIG_SP);
            uart_col_in_digit = uart_xw % (DIG_W + DIG_SP);

            if (uart_col_in_digit < DIG_W &&
                uart_digit_idx >= 0 &&
                uart_digit_idx < 4) begin
                case (uart_digit_idx)
                    0: uart_digit_sel = uart_hex3;
                    1: uart_digit_sel = uart_hex2;
                    2: uart_digit_sel = uart_hex1;
                    3: uart_digit_sel = uart_hex0;
                    default: uart_digit_sel = 4'd0;
                endcase

                uart_digit_px_on =
                    digit_pixel_on(uart_digit_sel,
                                   uart_row[2:0],
                                   uart_col_in_digit[2:0]);
            end
        end
    end

    // ------------------------------------------------------------------------
    // 13) Angle indicator – compass stack & heading digits in row4
    // ------------------------------------------------------------------------
    // Direction flags from frame-stable integer heading
    wire dir_N = (theta_deg_pix <  9'd45) || (theta_deg_pix >= 9'd315);
    wire dir_E = (theta_deg_pix >= 9'd45) && (theta_deg_pix < 9'd135);
    wire dir_S = (theta_deg_pix >= 9'd135)&& (theta_deg_pix < 9'd225);
    wire dir_W = (theta_deg_pix >= 9'd225)&& (theta_deg_pix < 9'd315);





    // Heading digits – row4, col0
    localparam [9:0] THETA_TX_X_L   = COL0_L + 10'd8;
    localparam [9:0] THETA_TX_Y_TOP = ROW4_T + 10'd4;

    wire in_theta_text_box =
        (hcount >= THETA_TX_X_L) &&
        (hcount <  THETA_TX_X_L + 3*DIG_W + 2*DIG_SP) &&
        (vcount >= THETA_TX_Y_TOP) &&
        (vcount <  THETA_TX_Y_TOP + DIG_H);

    reg  [3:0] th_d2, th_d1, th_d0;
    reg        theta_digit_px_on;
    reg  [3:0] theta_digit_sel;
    reg  [2:0] theta_row, theta_col;

    integer th_val;
    integer theta_xw;
    integer theta_digit_idx;
    integer theta_col_in_digit;

    always @* begin
        th_val = theta_deg_pix;       // 0..359, frame-stable
        if (th_val > 999) th_val = 999;

        th_d2 = th_val / 100;
        th_val= th_val % 100;
        th_d1 = th_val / 10;
        th_d0 = th_val % 10;
    end

    always @* begin
        theta_digit_px_on = 1'b0;
        theta_digit_sel   = 4'd0;
        theta_row         = 3'd0;
        theta_col         = 3'd0;

        if (in_theta_text_box) begin
            theta_row = vcount - THETA_TX_Y_TOP;
            theta_xw  = hcount - THETA_TX_X_L;

            if (theta_xw < DIG_W) begin
                theta_digit_idx    = 0;
                theta_col_in_digit = theta_xw;
            end else if (theta_xw < DIG_W + DIG_SP + DIG_W) begin
                if (theta_xw < DIG_W + DIG_SP) begin
                    theta_digit_idx    = -1;
                    theta_col_in_digit = 0;
                end else begin
                    theta_digit_idx    = 1;
                    theta_col_in_digit = theta_xw - (DIG_W + DIG_SP);
                end
            end else begin
                if (theta_xw < 2*DIG_W + 2*DIG_SP) begin
                    theta_digit_idx    = -1;
                    theta_col_in_digit = 0;
                end else begin
                    theta_digit_idx    = 2;
                    theta_col_in_digit = theta_xw - (2*DIG_W + 2*DIG_SP);
                end
            end

            if (theta_digit_idx == 0) begin
                theta_digit_sel   = th_d2;
                theta_col         = theta_col_in_digit[2:0];
                theta_digit_px_on = digit_pixel_on(theta_digit_sel,
                                                   theta_row,
                                                   theta_col);
            end else if (theta_digit_idx == 1) begin
                theta_digit_sel   = th_d1;
                theta_col         = theta_col_in_digit[2:0];
                theta_digit_px_on = digit_pixel_on(theta_digit_sel,
                                                   theta_row,
                                                   theta_col);
            end else if (theta_digit_idx == 2) begin
                theta_digit_sel   = th_d0;
                theta_col         = theta_col_in_digit[2:0];
                theta_digit_px_on = digit_pixel_on(theta_digit_sel,
                                                   theta_row,
                                                   theta_col);
            end
        end
    end



    // Compass rose – 3×3 cross layout in ROW4, COL3
    // ------------------------------------------------------------
    // Layout inside the row4 / col3 cell:
    //           [   N   ]
    //     [  W  ][  C  ][  E  ]
    //           [   S   ]
    //
    // C = center tile (light gray background), N/E/S/W arms highlight
    // when their corresponding dir_* flag is active.

    localparam integer COMP_TILE_W = 6;
    localparam integer COMP_TILE_H = 6;
    localparam integer COMP_GAP    = 2;

    // Center tile position within col3,row4 cell
    localparam [9:0] COMP_CENTER_X_L =
        COL3_L + ((CELL_W - COMP_TILE_W) >> 1);
    localparam [9:0] COMP_CENTER_X_R =
        COMP_CENTER_X_L + COMP_TILE_W - 1;

    localparam [9:0] COMP_CENTER_Y_T =
        ROW4_T + ((CELL_H - COMP_TILE_H) >> 1);
    localparam [9:0] COMP_CENTER_Y_B =
        COMP_CENTER_Y_T + COMP_TILE_H - 1;

    // N tile directly above center
    localparam [9:0] COMP_N_X_L = COMP_CENTER_X_L;
    localparam [9:0] COMP_N_X_R = COMP_CENTER_X_R;
    localparam [9:0] COMP_N_Y_T = COMP_CENTER_Y_T - (COMP_TILE_H + COMP_GAP);
    localparam [9:0] COMP_N_Y_B = COMP_N_Y_T + COMP_TILE_H - 1;

    // S tile directly below center
    localparam [9:0] COMP_S_X_L = COMP_CENTER_X_L;
    localparam [9:0] COMP_S_X_R = COMP_CENTER_X_R;
    localparam [9:0] COMP_S_Y_T = COMP_CENTER_Y_B + COMP_GAP + 1;
    localparam [9:0] COMP_S_Y_B = COMP_S_Y_T + COMP_TILE_H - 1;

    // W tile to the left of center
    localparam [9:0] COMP_W_X_L = COMP_CENTER_X_L - (COMP_TILE_W + COMP_GAP);
    localparam [9:0] COMP_W_X_R = COMP_W_X_L + COMP_TILE_W - 1;
    localparam [9:0] COMP_W_Y_T = COMP_CENTER_Y_T;
    localparam [9:0] COMP_W_Y_B = COMP_CENTER_Y_B;

    // E tile to the right of center
    localparam [9:0] COMP_E_X_L = COMP_CENTER_X_R + COMP_GAP + 1;
    localparam [9:0] COMP_E_X_R = COMP_E_X_L + COMP_TILE_W - 1;
    localparam [9:0] COMP_E_Y_T = COMP_CENTER_Y_T;
    localparam [9:0] COMP_E_Y_B = COMP_CENTER_Y_B;

    // Pixel membership for each direction arm
    wire comp_N_px = (hcount >= COMP_N_X_L) && (hcount <= COMP_N_X_R) &&
                     (vcount >= COMP_N_Y_T) && (vcount <= COMP_N_Y_B);
    wire comp_E_px = (hcount >= COMP_E_X_L) && (hcount <= COMP_E_X_R) &&
                     (vcount >= COMP_E_Y_T) && (vcount <= COMP_E_Y_B);
    wire comp_S_px = (hcount >= COMP_S_X_L) && (hcount <= COMP_S_X_R) &&
                     (vcount >= COMP_S_Y_T) && (vcount <= COMP_S_Y_B);
    wire comp_W_px = (hcount >= COMP_W_X_L) && (hcount <= COMP_W_X_R) &&
                     (vcount >= COMP_W_Y_T) && (vcount <= COMP_W_Y_B);

    // Center tile (optional: always light gray background cross)
    wire comp_C_px = (hcount >= COMP_CENTER_X_L) && (hcount <= COMP_CENTER_X_R) &&
                     (vcount >= COMP_CENTER_Y_T) && (vcount <= COMP_CENTER_Y_B);

    wire any_comp_px = comp_N_px | comp_E_px | comp_S_px | comp_W_px | comp_C_px;




    // ------------------------------------------------------------------------
    // 14) PIR “motion streak” – row3, columns 2–3
    // ------------------------------------------------------------------------
    wire in_pir_band_y = (vcount >= ROW3_T) && (vcount <= ROW3_B);
    wire [9:0] pir_x_r =
        (COL2_L + {2'b00, pir_level}) > COL3_R ?
        COL3_R : (COL2_L + {2'b00, pir_level});

    wire in_pir_streak =
        in_pir_band_y &&
        (hcount >= COL2_L) &&
        (hcount <= pir_x_r);

    // ------------------------------------------------------------------------
    // 14.5) UART bit tiles – row3, columns 0–1
    // ------------------------------------------------------------------------
    localparam [9:0] UART_BITS_Y_T = ROW3_T + 10'd6;
    localparam [9:0] UART_BITS_Y_B = ROW3_B - 10'd6;
    
    wire in_uart_bits_y =
        (vcount >= UART_BITS_Y_T) && (vcount <= UART_BITS_Y_B);
    
    // Pixel-membership flags for each bit tile (MSB..LSB)
    wire uart_bit7_px = in_uart_bits_y &&
                        (hcount >= UART_B7_X0) && (hcount <= UART_B7_X1);
    wire uart_bit6_px = in_uart_bits_y &&
                        (hcount >= UART_B6_X0) && (hcount <= UART_B6_X1);
    wire uart_bit5_px = in_uart_bits_y &&
                        (hcount >= UART_B5_X0) && (hcount <= UART_B5_X1);
    wire uart_bit4_px = in_uart_bits_y &&
                        (hcount >= UART_B4_X0) && (hcount <= UART_B4_X1);
    wire uart_bit3_px = in_uart_bits_y &&
                        (hcount >= UART_B3_X0) && (hcount <= UART_B3_X1);
    wire uart_bit2_px = in_uart_bits_y &&
                        (hcount >= UART_B2_X0) && (hcount <= UART_B2_X1);
    wire uart_bit1_px = in_uart_bits_y &&
                        (hcount >= UART_B1_X0) && (hcount <= UART_B1_X1);
    wire uart_bit0_px = in_uart_bits_y &&
                        (hcount >= UART_B0_X0) && (hcount <= UART_B0_X1);
    
    // Bit values from the last accepted byte
    wire uart_bit7_val = uart_byte_accepted_pix[7];
    wire uart_bit6_val = uart_byte_accepted_pix[6];
    wire uart_bit5_val = uart_byte_accepted_pix[5];
    wire uart_bit4_val = uart_byte_accepted_pix[4];
    wire uart_bit3_val = uart_byte_accepted_pix[3];
    wire uart_bit2_val = uart_byte_accepted_pix[2];
    wire uart_bit1_val = uart_byte_accepted_pix[1];
    wire uart_bit0_val = uart_byte_accepted_pix[0];

    // ------------------------------------------------------------------------
    // 15) Rotary encoder “spinner bar” – far right, bottom region
    // ------------------------------------------------------------------------
    localparam [9:0] ROT_X_L        = COL3_R - 10'd15;
    localparam [9:0] ROT_X_R        = COL3_R;
    localparam [9:0] ROT_Y_BOTTOM   = 10'd430;
    localparam [7:0] ROT_MAX_HEIGHT = 8'd200;

    wire [15:0] enc_pos_abs =
        enc_pos_pix[15] ? (~enc_pos_pix + 16'sd1) : enc_pos_pix;

    wire [7:0] enc_pos_mag      = enc_pos_abs[15:8];
    wire [7:0] enc_pos_mag_clip =
        (enc_pos_mag > ROT_MAX_HEIGHT) ? ROT_MAX_HEIGHT : enc_pos_mag;

    wire [7:0] rot_height = enc_pos_mag_clip;

    wire [9:0] rot_bar_top_y =
        (ROT_Y_BOTTOM > {2'b00, rot_height}) ?
        (ROT_Y_BOTTOM - {2'b00, rot_height}) :
        ROW1_T;  // don't go into legend row

    wire in_rot_bar =
        (hcount >= ROT_X_L) && (hcount <= ROT_X_R) &&
        (vcount >= rot_bar_top_y) && (vcount <= ROT_Y_BOTTOM);

    // ------------------------------------------------------------------------
    // 16) Status bitfield tiles – row2, cols0..3 (TOF/TMP/SRV/SYS)
    // ------------------------------------------------------------------------
    wire in_stat_row = (vcount >= ROW2_T) && (vcount <= ROW2_B);
    
    wire in_stat_tof = in_stat_row && in_col0;  // TOF
    wire in_stat_tmp = in_stat_row && in_col1;  // TMP
    wire in_stat_srv = in_stat_row && in_col2;  // SRV
    wire in_stat_sys = in_stat_row && in_col3;  // SYS
    
    // Semantic status flags
    wire stat_tof_ok = (uart_level != 8'd0);       // ToF/UART streaming alive
    wire stat_tmp_ok = ~temp_hot;                  // not in hottest band
    wire stat_srv_on = (rot_level != 8'd0);        // rotary active
    wire stat_sys_ok = fan_on_pix | pir_act_s1 | ba_sync; // aggregate OK



    // ------------------------------------------------------------------------
    // 17) Legend row – colored band + status legend text
    // ------------------------------------------------------------------------
    localparam [9:0] LEG_H   = 10'd12;                 // legend height in pixels
    localparam [9:0] LEG_Y_T = ROW0_T;                 // start at top of row0
    localparam [9:0] LEG_Y_B = ROW0_T + LEG_H - 1;     // end of legend band

    localparam [9:0] LEG_TEXT_W   = 10'd24;            // 3 chars * 8 px
    localparam [9:0] LEG_TEXT_PAD = (CELL_W - LEG_TEXT_W) >> 1;

    wire in_legend_band = (vcount >= LEG_Y_T) && (vcount <= LEG_Y_B);
    
    wire in_leg_temp = in_legend_band && in_col0;
    wire in_leg_duty = in_legend_band && in_col1;
    wire in_leg_uart = in_legend_band && in_col2;
    wire in_leg_rot  = in_legend_band && in_col3;

    function [9:0] stat_label_x;
        input [1:0] col;
        begin
            case (col)
                2'd0: stat_label_x = COL0_L + LEG_TEXT_PAD;
                2'd1: stat_label_x = COL1_L + LEG_TEXT_PAD;
                2'd2: stat_label_x = COL2_L + LEG_TEXT_PAD;
                2'd3: stat_label_x = COL3_L + LEG_TEXT_PAD;
                default: stat_label_x = COL0_L + LEG_TEXT_PAD;
            endcase
        end
    endfunction
    
    localparam [9:0] STATUS_LEG_Y = ROW0_T + 10'd4;

    // Legend glyphs: TOF / TMP / SRV / SYS
    wire tof_T_px, tof_O_px, tof_F_px;
    wire tmp_T_px, tmp_M_px, tmp_P_px;
    wire srv_S_px, srv_R_px, srv_V_px;
    wire sys_S_px, sys_Y_px, sys_S2_px;

    // Column 0: "TOF"
    vga_char_glyph u_leg_tof_T (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd0)),
        .y0           (STATUS_LEG_Y),
        .char_code    ("T"),
        .pixel_on     (tof_T_px)
    );
    vga_char_glyph u_leg_tof_O (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd0) + 10'd8),
        .y0           (STATUS_LEG_Y),
        .char_code    ("O"),
        .pixel_on     (tof_O_px)
    );
    vga_char_glyph u_leg_tof_F (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd0) + 10'd16),
        .y0           (STATUS_LEG_Y),
        .char_code    ("F"),
        .pixel_on     (tof_F_px)
    );

    // Column 1: "TMP"
    vga_char_glyph u_leg_tmp_T (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd1)),
        .y0           (STATUS_LEG_Y),
        .char_code    ("T"),
        .pixel_on     (tmp_T_px)
    );
    vga_char_glyph u_leg_tmp_M (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd1) + 10'd8),
        .y0           (STATUS_LEG_Y),
        .char_code    ("M"),
        .pixel_on     (tmp_M_px)
    );
    vga_char_glyph u_leg_tmp_P (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd1) + 10'd16),
        .y0           (STATUS_LEG_Y),
        .char_code    ("P"),
        .pixel_on     (tmp_P_px)
    );

    // Column 2: "SRV"
    vga_char_glyph u_leg_srv_S (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd2)),
        .y0           (STATUS_LEG_Y),
        .char_code    ("S"),
        .pixel_on     (srv_S_px)
    );
    vga_char_glyph u_leg_srv_R (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd2) + 10'd8),
        .y0           (STATUS_LEG_Y),
        .char_code    ("R"),
        .pixel_on     (srv_R_px)
    );
    vga_char_glyph u_leg_srv_V (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd2) + 10'd16),
        .y0           (STATUS_LEG_Y),
        .char_code    ("V"),
        .pixel_on     (srv_V_px)
    );

    // Column 3: "SYS"
    vga_char_glyph u_leg_sys_S0 (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd3)),
        .y0           (STATUS_LEG_Y),
        .char_code    ("S"),
        .pixel_on     (sys_S_px)
    );
    vga_char_glyph u_leg_sys_Y (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd3) + 10'd8),
        .y0           (STATUS_LEG_Y),
        .char_code    ("Y"),
        .pixel_on     (sys_Y_px)
    );
    vga_char_glyph u_leg_sys_S1 (
        .clk_pix      (pix_clk),
        .rst_pix      (rst),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .x0           (stat_label_x(2'd3) + 10'd16),
        .y0           (STATUS_LEG_Y),
        .char_code    ("S"),
        .pixel_on     (sys_S2_px)
    );

    wire status_legend_on =
        tof_T_px | tof_O_px | tof_F_px |
        tmp_T_px | tmp_M_px | tmp_P_px |
        srv_S_px | srv_R_px | srv_V_px |
        sys_S_px | sys_Y_px | sys_S2_px;

    // ------------------------------------------------------------------------
    // 18) Final color layering
    // ------------------------------------------------------------------------
    always @* begin
        if (!active_video) begin
            rgb_out = COL_BLACK;
        end else begin
            // Base color: background ToF plot
            rgb_out = rgb_bg;

            // Legend row background stripes
            if (in_leg_temp) rgb_out = COL_BLUE;     // ToF / TEMP legend
            if (in_leg_duty) rgb_out = COL_GREEN;    // Duty / TMP
            if (in_leg_uart) rgb_out = COL_CYAN;     // UART
            if (in_leg_rot)  rgb_out = COL_MAGENTA;  // SRV / SYS

            // Fan mode tiles (row1) with enable + on/off semantics

            // MANUAL tile (col0)
            if (in_box_man) begin
                if (!sw_manual_pix) begin
                    rgb_out = COL_DIMGRAY;
                end else if (fan_manual_pix) begin
                    rgb_out = COL_BLUEBRITE;
                end else begin
                    rgb_out = COL_BLUEDIM;
                end
            end

            // TEMP tile (col1)
            if (in_box_temp) begin
                if (!sw_temp_pix) begin
                    rgb_out = COL_DIMGRAY;
                end else if (fan_temp_pix) begin
                    rgb_out = COL_ORANGE;
                end else begin
                    rgb_out = COL_DARKORNG;
                end
            end

            // PIR tile (col2)
            if (in_box_pir) begin
                if (!sw_pir_pix) begin
                    rgb_out = COL_DIMGRAY;
                end else if (fan_pir_pix) begin
                    rgb_out = COL_GREEN;
                end else begin
                    rgb_out = COL_GREENDIM;
                end
            end

            // FAN aggregate tile (col3)
            if (in_box_fan) begin
                rgb_out = fan_on_pix ? COL_TEXT : COL_MAGENTA;
            end

            // Status tiles (row2): TOF/TMP/SRV/SYS
            if (in_stat_tof) begin
                rgb_out = stat_tof_ok ? COL_CYAN : COL_DIMGRAY;
            end

            if (in_stat_tmp) begin
                if (!stat_tmp_ok) begin
                    rgb_out = COL_RED;
                end else if (temp_cold) begin
                    rgb_out = COL_BLUE;
                end else begin
                    rgb_out = COL_GREEN;
                end
            end

            if (in_stat_srv) begin
                rgb_out = stat_srv_on ? COL_GREEN : COL_DIMGRAY;
            end

            if (in_stat_sys) begin
                rgb_out = stat_sys_ok ? COL_MAGENTA : COL_DIMGRAY;
            end

            // UART activity footer band
            if (in_uart_bar) begin
                rgb_out = { uart_byte_pix[7:4], uart_byte_pix[3:0], 4'h4 };
            end

            // PIR motion streak (row3, cols2–3)
            if (in_pir_streak) begin
                rgb_out = pir_act_s1 ? COL_GREEN : COL_YELLOW;
            end

            // UART bit tiles: 1 → bright cyan, 0 → dim gray
            if (uart_bit7_px) rgb_out = uart_bit7_val ? COL_CYAN : COL_DIMGRAY;
            if (uart_bit6_px) rgb_out = uart_bit6_val ? COL_CYAN : COL_DIMGRAY;
            if (uart_bit5_px) rgb_out = uart_bit5_val ? COL_CYAN : COL_DIMGRAY;
            if (uart_bit4_px) rgb_out = uart_bit4_val ? COL_CYAN : COL_DIMGRAY;
            if (uart_bit3_px) rgb_out = uart_bit3_val ? COL_CYAN : COL_DIMGRAY;
            if (uart_bit2_px) rgb_out = uart_bit2_val ? COL_CYAN : COL_DIMGRAY;
            if (uart_bit1_px) rgb_out = uart_bit1_val ? COL_CYAN : COL_DIMGRAY;
            if (uart_bit0_px) rgb_out = uart_bit0_val ? COL_CYAN : COL_DIMGRAY;

            // Fan speed gauge segments (row6)
            if (seg0_px) rgb_out = (fan_seg_limit > 4'd0) ? COL_GREEN : COL_DARKGRN;
            if (seg1_px) rgb_out = (fan_seg_limit > 4'd1) ? COL_GREEN : COL_DARKGRN;
            if (seg2_px) rgb_out = (fan_seg_limit > 4'd2) ? COL_GREEN : COL_DARKGRN;
            if (seg3_px) rgb_out = (fan_seg_limit > 4'd3) ? COL_GREEN : COL_DARKGRN;
            if (seg4_px) rgb_out = (fan_seg_limit > 4'd4) ? COL_GREEN : COL_DARKGRN;
            if (seg5_px) rgb_out = (fan_seg_limit > 4'd5) ? COL_GREEN : COL_DARKGRN;
            if (seg6_px) rgb_out = (fan_seg_limit > 4'd6) ? COL_GREEN : COL_DARKGRN;
            if (seg7_px) rgb_out = (fan_seg_limit > 4'd7) ? COL_GREEN : COL_DARKGRN;

            // Temperature bar (col2): blue → green → red
            if (in_temp_bar) begin
                if (temp_cold) begin
                    rgb_out = 12'h04F; // blue-ish
                end else if (temp_mid) begin
                    rgb_out = COL_GREEN;
                end else begin
                    rgb_out = COL_RED;
                end
            end

            // Duty bar (col3): green intensity ∝ duty
            if (in_duty_bar) begin
                rgb_out = { 4'h0, duty_pix[7:4], 4'h0 };
            end

            // Temperature numeric glyphs
            if (temp_digit_px_on) begin
                rgb_out = COL_TEXT;
            end

            // Duty numeric glyphs
            if (duty_digit_px_on) begin
                rgb_out = COL_TEXT;
            end

            // UART byte count glyphs
            if (uart_digit_px_on) begin
                rgb_out = COL_CYAN;
            end

            // Heading digits
            if (theta_digit_px_on) begin
                rgb_out = COL_TEXT;
            end

            // Temperature tick bars & labels
            if (tick_bar_on) begin
                rgb_out = COL_TEXT;
            end
            if (tick_text_on) begin
                rgb_out = COL_TEXT;
            end

            // Compass tiles: gray background, white on active direction
            if (any_comp_px) begin
                rgb_out = COL_LIGHT;
            end
            if (comp_N_px && dir_N) rgb_out = COL_TEXT;
            if (comp_E_px && dir_E) rgb_out = COL_TEXT;
            if (comp_S_px && dir_S) rgb_out = COL_TEXT;
            if (comp_W_px && dir_W) rgb_out = COL_TEXT;

            // Rotary spinner bar – draw last so it sits on top at far right
            if (in_rot_bar) begin
                case (enc_dir_pix)
                    1'b1: begin
                        // CW → green-ish bar
                        rgb_out = { rot_level[7:4] >> 1,  // R
                                    rot_level[7:4],       // G
                                    4'h0 };
                    end
                    1'b0: begin
                        // CCW → red/magenta bar
                        rgb_out = { rot_level[7:4],       // R
                                    4'h0,
                                    rot_level[7:4] >> 1 }; // B
                    end
                endcase
            end

            // Status legend and fan tile labels (white text)
            if (status_legend_on) begin
                rgb_out = COL_TEXT;
            end
            if (fan_text_on) begin
                rgb_out = COL_TEXT;
            end
        end
    end

endmodule
