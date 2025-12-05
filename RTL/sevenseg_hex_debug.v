`timescale 1ns/1ps
// ============================================================================
// sevenseg_hex_debug.v
//   8-digit, time-multiplexed seven-seg driver with simple telemetry view.
//
//   Base behavior (unchanged):
//     - ACTIVE-LOW segments/anodes (matches Nexys A7).
//     - Displays 8 hex nibbles (hex7..hex0), rightmost digit = hex0.
//     - Single 100 MHz clock domain; scan rate = REFRESH_HZ,
//       per-digit refresh ≈ REFRESH_HZ / 8.
//     - DP_MASK parameter: per-digit static decimal point control.
//     - SELF_TEST parameter: moving "8" pattern for bring-up.
//     - digit_en[7:0] input: runtime per-digit enable/blank mask.
//         • digit_en[i] = 1 → digit i may light as usual.
//         • digit_en[i] = 0 → digit i forced off (an[i] = 1).
//
//   Telemetry extension (new):
//     - telem_enable selects between "raw hex" mode and "telemetry" mode.
//         • telem_enable = 0 → original behavior (hex7..hex0 used directly).
//         • telem_enable = 1 → ignore hex7..hex0 and instead display:
//               * telem_word[23:0]  on digits 0..5 (right to left, hex).
//               * telem_status[7:0] on digits 6..7 (status flags, hex).
//     - SELF_TEST still overrides both modes when set.
//
//   Recommended packing for ToF telemetry:
//     - telem_word  = {8'h00, dist_mm};    // dist_mm = {MSB, LSB}
//     - telem_status= tof_status;          // status[7:0] from tof_sensor
//     → digits (7..0) = {sts[7:4], sts[3:0], MSB[7:4], MSB[3:0],
//                        LSB[7:4], LSB[3:0], word[7:4], word[3:0]}
// ============================================================================

module sevenseg_hex_debug #(
    parameter integer CLK_HZ      = 100_000_000,
    parameter integer REFRESH_HZ  = 1000,
    // Bit i = 1 → light decimal point on digit i (0=rightmost, 7=leftmost)
    parameter [7:0]   DP_MASK     = 8'b0000_0000,
    // If SELF_TEST=1, ignore data inputs and show a scanning "8" pattern
    parameter         SELF_TEST   = 1'b0
)(
    input  wire        clk,
    input  wire        rst,          // synchronous, active-HIGH

    // Raw hex nibbles (legacy/debug view)
    input  wire [3:0]  hex7,
    input  wire [3:0]  hex6,
    input  wire [3:0]  hex5,
    input  wire [3:0]  hex4,
    input  wire [3:0]  hex3,
    input  wire [3:0]  hex2,
    input  wire [3:0]  hex1,
    input  wire [3:0]  hex0,

    // Runtime per-digit enable mask (active-HIGH):
    //   digit_en[i] = 1 → digit i can light
    //   digit_en[i] = 0 → digit i is blanked
    input  wire [7:0]  digit_en,

    // ------------------------------------------------------------------------
    // Telemetry view inputs (NEW)
    // ------------------------------------------------------------------------
    // When telem_enable = 1, the display ignores hex7..hex0 and instead shows:
    //   digits 0..5 (right→left): telem_word[23:0] (hex)
    //   digits 6..7 (left two):   telem_status[7:0] (hex flags)
    input  wire        telem_enable,
    input  wire [31:0] telem_word,    // e.g., {8'h00, dist_mm}
    input  wire [7:0]  telem_status,  // e.g., tof_sensor.status

    output reg  [7:0]  an,           // ACTIVE-LOW digit enables
    output reg  [6:0]  seg,          // ACTIVE-LOW {A,B,C,D,E,F,G}
    output reg         dp            // ACTIVE-LOW decimal point
);

    // ------------------------------------------------------------------------
    // 0) Basic parameter sanity check (simulation-time only)
    // ------------------------------------------------------------------------
    initial begin
        if (REFRESH_HZ <= 0) begin
            $error("sevenseg_hex_debug: REFRESH_HZ must be > 0 (got %0d)", REFRESH_HZ);
        end
    end

    // ------------------------------------------------------------------------
    // 1) Refresh tick (no derived clocks)
    //     - tick_refresh pulses at REFRESH_HZ.
    //     - Each of 8 digits is active 1/8th of the time, so per-digit
    //       refresh ≈ REFRESH_HZ / 8.
    // ------------------------------------------------------------------------
    localparam integer REFRESH_DIV = CLK_HZ / REFRESH_HZ;
    localparam integer REFRESH_W   = $clog2(REFRESH_DIV);

    reg [REFRESH_W-1:0] ref_cnt      = {REFRESH_W{1'b0}};
    reg                 tick_refresh = 1'b0;

    always @(posedge clk) begin
        if (rst) begin
            ref_cnt      <= {REFRESH_W{1'b0}};
            tick_refresh <= 1'b0;
        end else if (ref_cnt == REFRESH_DIV-1) begin
            ref_cnt      <= {REFRESH_W{1'b0}};
            tick_refresh <= 1'b1;
        end else begin
            ref_cnt      <= ref_cnt + 1'b1;
            tick_refresh <= 1'b0;
        end
    end

    // ------------------------------------------------------------------------
    // 2) Digit scan index: 0..7
    // ------------------------------------------------------------------------
    reg [2:0] scan_idx = 3'd0;

    always @(posedge clk) begin
        if (rst) begin
            scan_idx <= 3'd0;
        end else if (tick_refresh) begin
            scan_idx <= scan_idx + 3'd1;  // wraps 7→0 automatically
        end
    end

    // ------------------------------------------------------------------------
    // 3) Nibble selection with telemetry override and SELF_TEST
    // ------------------------------------------------------------------------
    // We build an internal array of 8 nibbles, nib0..nib7 (0 = rightmost).
    // Priority:
    //   1) SELF_TEST = 1 → all digits show '8'.
    //   2) telem_enable = 1 → use telem_word/telem_status mapping.
    //   3) otherwise → use raw hex7..hex0 directly.
    // ------------------------------------------------------------------------
    reg [3:0] nib0, nib1, nib2, nib3, nib4, nib5, nib6, nib7;
    reg [3:0] cur_nibble;

    always @* begin
        if (SELF_TEST) begin
            // Self-test: every digit is hard-coded to "8"
            nib0 = 4'h8;
            nib1 = 4'h8;
            nib2 = 4'h8;
            nib3 = 4'h8;
            nib4 = 4'h8;
            nib5 = 4'h8;
            nib6 = 4'h8;
            nib7 = 4'h8;
        end else if (telem_enable) begin
            // ---------------- Telemetry view ----------------
            // telem_word[23:0] on digits 0..5
            nib0 = telem_word[ 3: 0];
            nib1 = telem_word[ 7: 4];
            nib2 = telem_word[11: 8];
            nib3 = telem_word[15:12];
            nib4 = telem_word[19:16];
            nib5 = telem_word[23:20];

            // telem_status[7:0] on digits 6..7 (status flags in hex)
            nib6 = telem_status[3:0];   // lower flags
            nib7 = telem_status[7:4];   // upper flags
        end else begin
            // ---------------- Legacy/raw hex view ----------------
            nib0 = hex0;  // rightmost
            nib1 = hex1;
            nib2 = hex2;
            nib3 = hex3;
            nib4 = hex4;
            nib5 = hex5;
            nib6 = hex6;
            nib7 = hex7;  // leftmost
        end

        // Current digit nibble based on scan_idx
        case (scan_idx)
            3'd0: cur_nibble = nib0;
            3'd1: cur_nibble = nib1;
            3'd2: cur_nibble = nib2;
            3'd3: cur_nibble = nib3;
            3'd4: cur_nibble = nib4;
            3'd5: cur_nibble = nib5;
            3'd6: cur_nibble = nib6;
            3'd7: cur_nibble = nib7;
            default: cur_nibble = 4'h0;
        endcase
    end

    // ------------------------------------------------------------------------
    // 4) Hex to 7-seg decode (ACTIVE-LOW: 0 = ON)
    // ------------------------------------------------------------------------
    always @* begin
        case (cur_nibble)
            4'h0: seg = 7'b0000001; // "0"
            4'h1: seg = 7'b1001111; // "1"
            4'h2: seg = 7'b0010010; // "2"
            4'h3: seg = 7'b0000110; // "3"
            4'h4: seg = 7'b1001100; // "4"
            4'h5: seg = 7'b0100100; // "5"
            4'h6: seg = 7'b0100000; // "6"
            4'h7: seg = 7'b0001111; // "7"
            4'h8: seg = 7'b0000000; // "8"
            4'h9: seg = 7'b0000100; // "9"
            4'hA: seg = 7'b0001000; // "A"
            4'hB: seg = 7'b1100000; // "b"
            4'hC: seg = 7'b0110001; // "C"
            4'hD: seg = 7'b1000010; // "d"
            4'hE: seg = 7'b0110000; // "E"
            4'hF: seg = 7'b0111000; // "F"
            default: seg = 7'b1111111; // all OFF
        endcase
    end

    // ------------------------------------------------------------------------
    // 5) Decimal point: parameterizable per digit (ACTIVE-LOW)
    //     - DP_MASK[i] = 1 → turn DP ON for digit i
    //     - digit index: 0 = rightmost, 7 = leftmost
    //
    //   Note: You can use DP_MASK as a static telemetry hint, e.g.
    //     - Mark the two status digits (6,7) with DP on to distinguish them.
    // ------------------------------------------------------------------------
    always @* begin
        // default: OFF (1 = inactive on active-low hardware)
        dp = 1'b1;
        case (scan_idx)
            3'd0: dp = DP_MASK[0] ? 1'b0 : 1'b1;
            3'd1: dp = DP_MASK[1] ? 1'b0 : 1'b1;
            3'd2: dp = DP_MASK[2] ? 1'b0 : 1'b1;
            3'd3: dp = DP_MASK[3] ? 1'b0 : 1'b1;
            3'd4: dp = DP_MASK[4] ? 1'b0 : 1'b1;
            3'd5: dp = DP_MASK[5] ? 1'b0 : 1'b1;
            3'd6: dp = DP_MASK[6] ? 1'b0 : 1'b1;
            3'd7: dp = DP_MASK[7] ? 1'b0 : 1'b1;
            default: dp = 1'b1;
        endcase
    end

    // ------------------------------------------------------------------------
    // 6) Anode scan (ACTIVE-LOW) + runtime enable mask
    //     - If digit_en[scan_idx] = 0, that digit is forced OFF.
    // ------------------------------------------------------------------------
    always @* begin
        // default: all digits OFF
        an = 8'b1111_1111;

        case (scan_idx)
            3'd0: an = digit_en[0] ? 8'b1111_1110 : 8'b1111_1111; // rightmost
            3'd1: an = digit_en[1] ? 8'b1111_1101 : 8'b1111_1111;
            3'd2: an = digit_en[2] ? 8'b1111_1011 : 8'b1111_1111;
            3'd3: an = digit_en[3] ? 8'b1111_0111 : 8'b1111_1111;
            3'd4: an = digit_en[4] ? 8'b1110_1111 : 8'b1111_1111;
            3'd5: an = digit_en[5] ? 8'b1101_1111 : 8'b1111_1111;
            3'd6: an = digit_en[6] ? 8'b1011_1111 : 8'b1111_1111;
            3'd7: an = digit_en[7] ? 8'b0111_1111 : 8'b1111_1111; // leftmost
            default: an = 8'b1111_1111;
        endcase
    end

endmodule
