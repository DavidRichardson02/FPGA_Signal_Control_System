`timescale 1ns/1ps
// ============================================================================
// Module : tof_7seg_telem_debug
// Role   : Format ToF distance + status as a compact telemetry view on the
//          Nexys A7 8-digit seven-segment display, using sevenseg_hex_debug.
//
// External view (digits: [7] = leftmost .. [0] = rightmost)
//
//   [7]      [6]      [5]      [4]      [3]      [2]      [1]   [0]
//   ST_HI    ST_LO    D3       D2       D1       D0       0     0
//
//   - D3..D0 = dist_mm[15:0] as 4 hex digits (MSB at digit 5).
//   - ST_HI  = tof_status[7:4] (upper status nibble).
//   - ST_LO  = tof_status[3:0] (lower status nibble).
//   - Digits 1,0 reserved (currently 0).
//
// Telemetry cues:
//   - DP lit on the two status digits (7 and 6) → “these are flags”.
//   - If any error bit (tof_status[7] or [6]) is set, digits [7:6]
//     blink at ~1 Hz via digit_en[7:6].
//   - telem_enable=0 blanks all digits (but the scan engine stays alive).
//
// Integration notes:
//   - Wraps sevenseg_hex_debug’s *telemetry* mode:
//        • telem_word   → carries {dist_mm, 8'h00}.
//        • telem_status → tof_status.
//        • telem_enable → pass-through.
//   - hex7..hex0 are still driven for completeness, but ignored whenever
//     telem_enable=1 inside sevenseg_hex_debug.
// ============================================================================

module tof_7seg_telem_debug #(
    parameter integer CLK_HZ      = 100_000_000,
    parameter integer REFRESH_HZ  = 1000,
    // Blink frequency for error highlight (Hz)
    parameter integer BLINK_HZ    = 1
)(
    input  wire        clk,
    input  wire        rst,

    input  wire [15:0] dist_mm,
    input  wire [7:0]  tof_status,

    // 1 → show telemetry; 0 → blank all digits
    input  wire        telem_enable,

    output wire [7:0]  an,   // ACTIVE-LOW digit enables (to board)
    output wire [6:0]  seg,  // ACTIVE-LOW segments
    output wire        dp    // ACTIVE-LOW decimal point
);

    // ------------------------------------------------------------------------
    // 1) Hex nibbles for legacy mode (not used when telem_enable=1, but we
    //    still drive them cleanly to keep the underlying core well-defined).
    // ------------------------------------------------------------------------
    wire [3:0] hex7 = 4'h0;
    wire [3:0] hex6 = 4'h0;
    wire [3:0] hex5 = 4'h0;
    wire [3:0] hex4 = 4'h0;
    wire [3:0] hex3 = 4'h0;
    wire [3:0] hex2 = 4'h0;
    wire [3:0] hex1 = 4'h0;
    wire [3:0] hex0 = 4'h0;

    // ------------------------------------------------------------------------
    // 2) Telemetry packing for sevenseg_hex_debug
    // ------------------------------------------------------------------------
    // sevenseg_hex_debug (in telemetry mode) maps:
    //
    //   digits 0..5  ← telem_word[23:0] as 6 hex nibbles:
    //                    nib0 = [ 3: 0] → digit 0 (rightmost)
    //                    nib1 = [ 7: 4] → digit 1
    //                    nib2 = [11: 8] → digit 2
    //                    nib3 = [15:12] → digit 3
    //                    nib4 = [19:16] → digit 4
    //                    nib5 = [23:20] → digit 5
    //
    //   digits 6..7  ← telem_status[7:0] (flags, hex)
    //                    digit 6 = telem_status[3:0]
    //                    digit 7 = telem_status[7:4]
    //
    // We want:
    //   [7] ST_HI = tof_status[7:4]
    //   [6] ST_LO = tof_status[3:0]
    //   [5] D3    = dist_mm[15:12]
    //   [4] D2    = dist_mm[11: 8]
    //   [3] D1    = dist_mm[ 7: 4]
    //   [2] D0    = dist_mm[ 3: 0]
    //   [1] 0
    //   [0] 0
    //
    // That corresponds to:
    //   telem_word[23:20] = D3
    //   telem_word[19:16] = D2
    //   telem_word[15:12] = D1
    //   telem_word[11: 8] = D0
    //   telem_word[ 7: 4] = 4'h0
    //   telem_word[ 3: 0] = 4'h0
    // → telem_word = { D3, D2, D1, D0, 4'h0, 4'h0 }.
    // More simply: telem_word = { dist_mm, 8'h00 } (we only use [23:0]).
    // ------------------------------------------------------------------------

    wire [31:0] telem_word =
        { dist_mm, 8'h00 };  // upper 8 bits unused by the core

    wire [7:0] telem_status = tof_status;

    // ------------------------------------------------------------------------
    // 3) Error detection + blink generator
    // ------------------------------------------------------------------------
    // Error policy:
    //   - has_error is asserted if either of the two MSB flags are set:
    //       tof_status[7] : irq_timeout
    //       tof_status[6] : i2c_err_latched
    //
    //   - When has_error=1, we blink the two status digits (7,6) at BLINK_HZ.
    // ------------------------------------------------------------------------
    wire has_error = |tof_status[7:6];

    localparam integer BLINK_DIV = (BLINK_HZ <= 0) ? (CLK_HZ/2) : (CLK_HZ / (2*BLINK_HZ));
    localparam integer BLINK_W   = $clog2(BLINK_DIV);

    reg [BLINK_W-1:0] blink_cnt = {BLINK_W{1'b0}};
    reg               blink     = 1'b0;

    always @(posedge clk) begin
        if (rst) begin
            blink_cnt <= {BLINK_W{1'b0}};
            blink     <= 1'b0;
        end else begin
            if (blink_cnt == BLINK_DIV-1) begin
                blink_cnt <= {BLINK_W{1'b0}};
                blink     <= ~blink;
            end else begin
                blink_cnt <= blink_cnt + {{(BLINK_W-1){1'b0}},1'b1};
            end
        end
    end

    // ------------------------------------------------------------------------
    // 4) digit_en mask: panel blanking + error blinking
    // ------------------------------------------------------------------------
    // digit_en[i] = 1 → digit i may light (subject to segment pattern).
    // digit_en[i] = 0 → digit i forced off at anode-level.
    //
    // telem_enable semantics:
    //   - 0 → panel blank: all digits off.
    //   - 1 → show telemetry; if has_error=1, blink digits 7 and 6.
    // ------------------------------------------------------------------------
    reg [7:0] digit_en_mask;

    always @* begin
        if (!telem_enable) begin
            // Global blanking: entire panel off
            digit_en_mask = 8'h00;
        end else begin
            // Baseline: all digits enabled
            digit_en_mask = 8'hFF;

            // Blink status digits when error present
            if (has_error) begin
                digit_en_mask[7] = blink;
                digit_en_mask[6] = blink;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 5) Decimal point mini-legend
    //     - Light DP on the two status digits (7,6) to “tag” them as flags.
    //     - ACTIVE-LOW: DP_MASK bit=1 → DP ON for that digit index.
    // ------------------------------------------------------------------------
    localparam [7:0] DP_MASK_STATUS = 8'b1100_0000; // dp on digits 7 and 6

    // ------------------------------------------------------------------------
    // 6) Underlying seven-segment engine
    // ------------------------------------------------------------------------
    sevenseg_hex_debug #(
        .CLK_HZ      (CLK_HZ),
        .REFRESH_HZ  (REFRESH_HZ),
        .DP_MASK     (DP_MASK_STATUS),
        .SELF_TEST   (1'b0)
    ) u_ss_tof (
        .clk          (clk),
        .rst          (rst),

        // Legacy hex inputs (ignored when telem_enable=1 inside the core)
        .hex7         (hex7),
        .hex6         (hex6),
        .hex5         (hex5),
        .hex4         (hex4),
        .hex3         (hex3),
        .hex2         (hex2),
        .hex1         (hex1),
        .hex0         (hex0),

        // Runtime per-digit enable mask
        .digit_en     (digit_en_mask),

        // Telemetry view inputs
        .telem_enable (telem_enable),   // 1 → use telem_word/status mapping
        .telem_word   (telem_word),
        .telem_status (telem_status),

        // Physical outputs
        .an           (an),
        .seg          (seg),
        .dp           (dp)
    );

endmodule
