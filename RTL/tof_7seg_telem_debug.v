`timescale 1ns/1ps
// ============================================================================
// tof_7seg_telem_debug.v
//   Tiny wrapper that formats ToF distance + status onto the Nexys 8-digit
//   seven-segment display using sevenseg_hex_debug.
//
// Layout (digit index: 7 = leftmost .. 0 = rightmost):
//
//   [7] [6] [5] [4] [3] [2] [1] [0]
//    D3  D2  D1  D0  ST  ST  --  --
//
//   - D3..D0 = dist_mm[15:0] as 4 hex digits (MSB at digit 7).
//   - ST/ST  = tof_status[7:0] as 2 hex digits.
//   - Digits 1,0 reserved (currently 0).
//
// Telemetry cues:
//   - DP lit on the two status digits (3 and 2) → "these are flags".
//   - If any error bit (tof_status[7] or [6]) is set, status digits blink
//     at ~1 Hz via digit_en[3:2].
//   - telem_enable=0 blanks all digits but keeps the driver alive.
// ============================================================================

module tof_7seg_telem_debug #(
    parameter integer CLK_HZ     = 100_000_000,
    parameter integer REFRESH_HZ = 1000
)(
    input  wire        clk,
    input  wire        rst,

    input  wire [15:0] dist_mm,
    input  wire [7:0]  tof_status,

    // 1 → drive the panel; 0 → blank all digits (but keep scanning)
    input  wire        telem_enable,

    output wire [7:0]  an,   // ACTIVE-LOW digit enables (to board)
    output wire [6:0]  seg,  // ACTIVE-LOW segments
    output wire        dp    // ACTIVE-LOW decimal point
);

    // ------------------------------------------------------------------------
    // 1) Pack distance + status into 8 hex nibbles
    // ------------------------------------------------------------------------
    wire [3:0] hex7 = dist_mm[15:12];
    wire [3:0] hex6 = dist_mm[11: 8];
    wire [3:0] hex5 = dist_mm[ 7: 4];
    wire [3:0] hex4 = dist_mm[ 3: 0];

    wire [3:0] hex3 = tof_status[7:4];   // status high nibble (irq_timeout, i2c_err, etc.)
    wire [3:0] hex2 = tof_status[3:0];   // status low nibble (vdd_ok, chip_ready, enout)
    wire [3:0] hex1 = 4'h0;              // reserved for future use
    wire [3:0] hex0 = 4'h0;              // reserved for future use

    // ------------------------------------------------------------------------
    // 2) Slow blink generator for error highlighting (~1 Hz @ 100 MHz)
    // ------------------------------------------------------------------------
    // Any error? Use the two MSBs: [7]=irq_timeout, [6]=i2c_err_latched
    wire has_error = |tof_status[7:6];

    reg [25:0] blink_div;
    reg        blink;

    always @(posedge clk) begin
        if (rst) begin
            blink_div <= 26'd0;
            blink     <= 1'b0;
        end else begin
            blink_div <= blink_div + 26'd1;
            if (blink_div == 26'd0) begin
                blink <= ~blink;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 3) digit_en mask: per-digit blanking + error blinking + master enable
    // ------------------------------------------------------------------------
    reg [7:0] digit_en_mask;

    always @* begin
        if (!telem_enable) begin
            // Global blanking: all digits off
            digit_en_mask = 8'h00;
        end else begin
            // Nominally: all digits on
            digit_en_mask = 8'hFF;

            // For the two status digits (3 and 2), blink when has_error=1
            if (has_error) begin
                digit_en_mask[3] = blink;
                digit_en_mask[2] = blink;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 4) Decimal point mini-legend:
    //     - Light DP on the two status digits (3 and 2).
    //     - sevenseg_hex_debug has a *parameter* DP_MASK, so we choose it
    //       statically here (dp lit whenever those digits are active).
    // ------------------------------------------------------------------------
    localparam [7:0] DP_MASK_STATUS = 8'b0000_1100; // dp on digits 3 and 2

    // ------------------------------------------------------------------------
    // 5) Underlying seven-seg engine
    // ------------------------------------------------------------------------
    sevenseg_hex_debug #(
        .CLK_HZ     (CLK_HZ),
        .REFRESH_HZ (REFRESH_HZ),
        .DP_MASK    (DP_MASK_STATUS),
        .SELF_TEST  (1'b0)
    ) u_ss_tof (
        .clk      (clk),
        .rst      (rst),

        .hex7     (hex7),
        .hex6     (hex6),
        .hex5     (hex5),
        .hex4     (hex4),
        .hex3     (hex3),
        .hex2     (hex2),
        .hex1     (hex1),
        .hex0     (hex0),

        .digit_en (digit_en_mask),

        .an       (an),
        .seg      (seg),
        .dp       (dp)
    );

endmodule
