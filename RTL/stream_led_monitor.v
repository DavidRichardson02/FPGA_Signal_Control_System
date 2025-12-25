`timescale 1ns/1ps
// ============================================================================
// Module        : stream_led_monitor
// Project       : FPGA_Spatial_Mapping_Project (Nexys A7-100T)
//
// Role in System
// ----------------------------------------------------------------------------
//   Provide a *human-visible* indication that a byte-stream interface is alive
//   and (optionally) how "fast" it is running, using the eight lower user LEDs.
//
//   The module is intentionally agnostic to protocol framing; it only cares
//   about a 1-cycle strobe `byte_accepted` that fires when a single byte has
//   been successfully handed to a downstream sink (e.g. UART TX).
//
//   Typical hookup (as used in spatial_mapping_temperature_control_top):
//
//       byte_accepted = pkt_vld & pkt_rdy;
//
//   where:
//     - pkt_vld : asserted by mapper/packetizer when tx_byte is valid.
//     - pkt_rdy : asserted by UART when it is ready to accept a byte.
//     - A byte transfer occurs on cycles where (pkt_vld && pkt_rdy) = 1.
//
//   This module turns that handshake strobe into:
//
//     1) blink
//        - A single-bit “heartbeat” that toggles on *every* accepted byte.
//        - At moderate rates this appears as a fast flicker.
//        - Can be routed to a dedicated LED if desired.
//
//     2) leds[7:0]  (two possible modes)
//        MODE_ACTIVITY_BAR (default):
//          - 8-bit activity counter that increments once every 2^DEC_SHIFT
//            accepted bytes (wraps naturally).
//          - Creates a “twinkling bar” pattern while the stream is alive.
//
//        MODE_RATE_BAR:
//          - Approximates throughput as “bytes per time window”.
//          - Counts accepted bytes in a fixed time window (RATE_WINDOW_MS).
//          - At each window boundary, maps the count into a bar-graph:
//              * LED0 lights when bytes ≥ 1 * RATE_BYTES_PER_STEP
//              * LED1 lights when bytes ≥ 2 * RATE_BYTES_PER_STEP
//                ...
//              * LED7 lights when bytes ≥ 8 * RATE_BYTES_PER_STEP
//          - Creates a coarse, monotonic bar that grows/shrinks with rate.
//
//   Either mode uses the same external interface; only parameters differ.
//
// External Interface
// ----------------------------------------------------------------------------
//   clk           : System fabric clock (100 MHz in this design).
//   rst           : Synchronous, active-HIGH reset.
//   byte_accepted : 1-cycle strobe, asserted exactly when one byte enters the
//                   downstream sink (e.g., pkt_vld & pkt_rdy).
//
//   leds[7:0]     : 8-bit telemetry bar for the stream.
//   blink         : Toggles on EVERY accepted byte (heartbeat).
//
// Parameterization
// ----------------------------------------------------------------------------
//   CLK_HZ : fabric clock frequency in Hz.
//   DEC_SHIFT : activity-mode decimation exponent.
//       - Number of accepted bytes per LED increment = 2^DEC_SHIFT.
//       - DEC_SHIFT = 0 → increment LEDs on every accepted byte.
//       - DEC_SHIFT = 5 → increment LEDs every 32 accepted bytes.
//
//   MODE : selects how leds[7:0] are interpreted:
//       MODE_ACTIVITY_BAR = 0 (default, original behavior).
//       MODE_RATE_BAR     = 1 (bytes-per-window rate bar).
//
//   RATE_WINDOW_MS : length of the rate-measurement window in milliseconds.
//   RATE_BYTES_PER_STEP : bytes-per-window per LED step in rate mode.
//       - If a window accumulates N bytes, then:
//             leds[i] = 1  iff  N ≥ (i+1) * RATE_BYTES_PER_STEP.
//
// Design Notes
// ----------------------------------------------------------------------------
//   • All logic is synchronous to 'clk'; there are NO gated/derived clocks.
//   • The original behavior (DEC_SHIFT-based activity bar) is preserved when
//     MODE = MODE_ACTIVITY_BAR. Existing instantiations remain valid.
//   • Rate-bar logic is “always on” internally but pruned by synthesis if
//     MODE is left at the default (0) in most tools.
//
// ============================================================================

module stream_led_monitor #(
    // Fabric clock frequency (Hz)
    parameter integer CLK_HZ              = 100_000_000,

    // Activity mode decimation: events per LED increment = 2^DEC_SHIFT.
    parameter integer DEC_SHIFT          = 5,   // 2^5 = 32 bytes per LED increment

    // LED display mode
    parameter integer MODE               = 0,   // 0 = ACTIVITY_BAR, 1 = RATE_BAR

    // Rate-bar configuration (used when MODE = 1)
    parameter integer RATE_WINDOW_MS     = 100, // time window length in ms
    parameter integer RATE_BYTES_PER_STEP= 32   // bytes per step in that window
)(
    input  wire        clk,           // Fabric/system clock
    input  wire        rst,           // Synchronous, active-HIGH reset
    input  wire        byte_accepted, // 1-cycle pulse when a stream byte is accepted

    output reg  [7:0]  leds,          // Telemetry bar (mode-dependent)
    output reg         blink          // Toggles on EVERY accepted byte (heartbeat)
);

    // =========================================================================
    // 0. Mode enumeration (purely for readability)
    // =========================================================================
    localparam integer MODE_ACTIVITY_BAR = 0;
    localparam integer MODE_RATE_BAR     = 1;

    // =========================================================================
    // 1. Activity-bar machinery (original behavior)
    // =========================================================================
    //
    //   - ev_cnt: event counter used to decimate byte_accepted pulses.
    //   - leds_activity: local 8-bit bar that wraps naturally.
    //
    //   Effective behavior:
    //     EW = max(1, DEC_SHIFT)
    //     - ev_cnt is EW-bit unsigned and increments on each byte_accepted.
    //     - When previous ev_cnt was all 1s ( &ev_cnt == 1 ), we have seen
    //       2^EW events and increment leds_activity by 1 (wraps mod 256).
    //
    //   This exactly matches original LED behavior when MODE=0.
    // -------------------------------------------------------------------------

    localparam integer EW = (DEC_SHIFT < 1) ? 1 : DEC_SHIFT;
    reg [EW-1:0]  ev_cnt;
    reg [7:0]     leds_activity;

    // =========================================================================
    // 2. Rate-bar machinery (bytes per time window)
    // =========================================================================
    //
    //   - A "window" is RATE_WINDOW_MS milliseconds long.
    //   - Within each window we count the number of accepted bytes.
    //   - At the end of the window we:
    //       • latch the count into byte_rate_sample,
    //       • compute a bar pattern leds_rate based on thresholds.
    //
    //   Window tick derivation:
    //     RATE_WINDOW_CLKS = (CLK_HZ / 1000) * RATE_WINDOW_MS
    //     - Assumes CLK_HZ is divisible by 1000; otherwise the actual
    //       window length will be slightly quantized by integer arithmetic.
    // -------------------------------------------------------------------------

    localparam integer RATE_WINDOW_CLKS =
        (RATE_WINDOW_MS <= 0) ? CLK_HZ : (CLK_HZ / 1000) * RATE_WINDOW_MS;

    localparam integer RATE_CNT_W = $clog2(RATE_WINDOW_CLKS);

    reg [RATE_CNT_W-1:0] win_cnt;          // clock cycles within current window
    reg [15:0]           byte_accum;       // bytes counted in current window
    reg [15:0]           byte_rate_sample; // latched at window boundary
    reg [7:0]            leds_rate;        // bar pattern derived from sample

    integer i; // used for threshold loop

    // =========================================================================
    // 3. Main sequential process (heartbeat, both LED modes)
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            //-----------------------------------------------------------------
            // Reset: clear all state and drive outputs to known values
            //-----------------------------------------------------------------
            blink            <= 1'b0;

            ev_cnt           <= {EW{1'b0}};
            leds_activity    <= 8'd0;

            win_cnt          <= {RATE_CNT_W{1'b0}};
            byte_accum       <= 16'd0;
            byte_rate_sample <= 16'd0;
            leds_rate        <= 8'd0;

            leds             <= 8'd0;
        end else begin
            //-----------------------------------------------------------------
            // 3.1 Heartbeat and activity-bar decimator
            //-----------------------------------------------------------------
            if (byte_accepted) begin
                // Heartbeat: toggles on *every* accepted byte.
                blink <= ~blink;

                // Event counter: counts accepted bytes modulo 2^EW.
                ev_cnt <= ev_cnt + 1'b1;

                // Decimated LED update: once per 2^EW accepted bytes.
                // '&ev_cnt' uses the "old" value (pre-increment), so this
                // fires exactly when ev_cnt was all 1s on the previous cycle.
                if (&ev_cnt) begin
                    leds_activity <= leds_activity + 8'd1;
                end
            end

            //-----------------------------------------------------------------
            // 3.2 Rate-window accumulation
            //-----------------------------------------------------------------
            // Window clock counter (free-running)
            if (win_cnt == RATE_WINDOW_CLKS - 1) begin
                // End of window: latch the accumulated byte count
                win_cnt          <= {RATE_CNT_W{1'b0}};
                byte_rate_sample <= byte_accum;
                byte_accum       <= 16'd0;

                // Compute new bar pattern based on byte_rate_sample.
                // Thresholds: (i+1)*RATE_BYTES_PER_STEP.
                for (i = 0; i < 8; i = i + 1) begin
                    if (byte_rate_sample >= ( (i+1) * RATE_BYTES_PER_STEP ))
                        leds_rate[i] <= 1'b1;
                    else
                        leds_rate[i] <= 1'b0;
                end
            end else begin
                // Still inside the window: advance clock and accumulate bytes
                win_cnt <= win_cnt + {{(RATE_CNT_W-1){1'b0}}, 1'b1};

                if (byte_accepted) begin
                    // Accumulate total bytes seen in this window (saturate at max)
                    if (byte_accum != 16'hFFFF)
                        byte_accum <= byte_accum + 16'd1;
                end
            end

            //-----------------------------------------------------------------
            // 3.3 Select which internal LED representation drives the outputs
            //-----------------------------------------------------------------
            case (MODE)
                MODE_ACTIVITY_BAR: leds <= leds_activity;
                MODE_RATE_BAR:     leds <= leds_rate;
                default:           leds <= leds_activity; // safe fallback
            endcase
        end
    end

endmodule
