`timescale 1ns/1ps

// ======================================================================================
// File    : pir_conditioner.v (pedagogically documented)
// Module  : pir_conditioner
// Role    : Condition a raw PIR sensor’s digital output so it is safe and convenient
//           for synchronous logic.
//
// High-level behavior
// -------------------
// Many off-the-shelf PIR modules expose a “motion” digital output, but that signal is
// not immediately friendly for FPGA logic because:
//
//   • It is asynchronous to the fabric clock → risk of metastability.
//   • It may chatter around edges, especially during warm-up or marginal motion.
//   • The sensor itself often requires 30–60 seconds of warm-up after power-up.
//   • Typical applications want a “recent motion” window, not just an instantaneous edge.
//
// This block performs four key conditioning steps:
//
//   1) 2-FF synchronizer:
//        - Brings pir_raw into the clk domain (metastability hygiene).
//
//   2) Debounce (DEBOUNCE_MS):
//        - Requires the synchronized level to remain stable for DEBOUNCE_MS before
//          accepting it as the new debounced state.
//
//   3) Warm-up mask (WARMUP_S):
//        - Ignore any motion edges until WARMUP_S seconds after reset/power-up.
//        - Prevents the module from reacting to early sensor chatter.
//
//   4) Event + hold behavior (HOLD_S):
//        - On each *debounced* rising edge (after warm-up), emit:
//              • rise_pulse : a 1-clock strobe, convenient for edge-triggered logic.
//              • active     : a level that stays HIGH for HOLD_S seconds since the
//                             *last* motion event (stretch / “recent motion” window).
//
// Parameterization & scaling
// --------------------------
//   CLK_HZ      : fabric clock frequency in Hz (e.g., 100_000_000 for 100 MHz).
//   DEBOUNCE_MS : debounce time in milliseconds.
//   WARMUP_S    : warm-up mask duration in seconds (ignore PIR output before this).
//   HOLD_S      : “recent motion” hold time in seconds.
//
// Internally:
//   • A full-rate debounce counter runs at CLK_HZ.
//   • A 1 ms timebase is derived from CLK_HZ and used for warm-up and hold timers.
//   • Counter widths are sized via a CLOG2 utility to avoid overflow.
//
// Clocking & reset conventions
// ----------------------------
//   clk : single fabric clock domain (e.g., 100 MHz).
//   rst : synchronous, active-HIGH reset; clears all internal state and returns
//         outputs to a safe idle state (active=0, rise_pulse=0).
//
// Typical usage (top-level)
// -------------------------
//   pir_conditioner #(
//       .CLK_HZ      (100_000_000),
//       .DEBOUNCE_MS (20),
//       .WARMUP_S    (30),
//       .HOLD_S      (30)
//   ) u_pir_conditioner (
//       .clk        (clk_100MHz),
//       .rst        (rst),
//       .pir_raw    (pir_raw),
//       .active     (pir_active),   // “motion recently detected” window
//       .rise_pulse (pir_rise)      // 1-cycle strobe on each new motion event
//   );
//
// ======================================================================================
module pir_conditioner #(
    parameter integer CLK_HZ      = 100_000_000, // fabric clock rate in Hz
    parameter integer DEBOUNCE_MS = 20,          // input must be stable this long (ms)
    parameter integer WARMUP_S    = 30,          // warm-up mask duration in seconds
    parameter integer HOLD_S      = 15          // hold active HIGH this long after event (s)
)(
    input  wire clk,        // fabric clock
    input  wire rst,        // synchronous, active-HIGH reset
    input  wire pir_raw,    // asynchronous PIR digital output (3V3 TTL from sensor board)
    output reg  active,     // HIGH while motion is “recent” (HOLD_S window)
    output reg  rise_pulse  // 1-cycle strobe on each new debounced rising edge
);

    // -----------------------------------------------------------------------------
    // 0) Utility: integer ceil(log2) for safe counter sizing
    // -----------------------------------------------------------------------------
    // CLOG2(N) returns the minimum number of bits required to represent [0 .. N-1].
    // Used below to size debounce, warm-up, and hold counters without overflow.
    function integer CLOG2;
        input integer v;
        integer i;
        begin
            i = v - 1;
            for (CLOG2 = 0; i > 0; CLOG2 = CLOG2 + 1)
                i = i >> 1;
        end
    endfunction

    // -----------------------------------------------------------------------------
    // 1) 2-FF synchronizer
    // -----------------------------------------------------------------------------
    // Purpose:
    //   pir_raw is asynchronous with respect to clk, so we use a standard 2-flip-flop
    //   synchronizer to greatly reduce metastability probability before further logic.
    //
    // Notes:
    //   • s0 captures pir_raw; s1 captures s0.
    //   • Any combinational logic should use s1, not pir_raw/s0.
    reg s0 = 1'b0;
    reg s1 = 1'b0;

    always @(posedge clk) begin
        s0 <= pir_raw;
        s1 <= s0;
    end

    // -----------------------------------------------------------------------------
    // 2) Debounce at full clock rate
    // -----------------------------------------------------------------------------
    // Concept:
    //   - If the synchronized level (s1) matches the debounced state (db), the input
    //     is considered stable and the debounce counter is reset.
    //   - If s1 differs from db, we start counting clock cycles.
    //   - Only if s1 remains different for DB_TICKS consecutive cycles do we accept
    //     the new value and update db.
    //
    // DB_TICKS = (CLK_HZ / 1000) * DEBOUNCE_MS
    //   Example (100 MHz, 20 ms):
    //     DB_TICKS = (100e6 / 1000) * 20 = 2_000_000 cycles.
    localparam integer DB_TICKS = (CLK_HZ / 1000) * DEBOUNCE_MS;
    localparam integer DB_W     = CLOG2(DB_TICKS + 1);

    reg [DB_W-1:0] db_cnt = {DB_W{1'b0}};  // debounce counter
    reg            db     = 1'b0;          // debounced PIR level

    always @(posedge clk) begin
        if (rst) begin
            db_cnt <= {DB_W{1'b0}};
            db     <= 1'b0;
        end else if (s1 == db) begin
            // Input agrees with current debounced level → restart timer
            db_cnt <= {DB_W{1'b0}};
        end else if (db_cnt == DB_TICKS[DB_W-1:0] - 1) begin
            // Input has remained different for full debounce window → accept new state
            db     <= s1;
            db_cnt <= {DB_W{1'b0}};
        end else begin
            // Still waiting to see if the new level is stable
            db_cnt <= db_cnt + 1'b1;
        end
    end

    // -----------------------------------------------------------------------------
    // 3) 1 ms timebase (for warm-up and hold timers)
    // -----------------------------------------------------------------------------
    // We derive a 1 ms “tick” from CLK_HZ. This provides a convenient human-scale
    // timebase to express WARMUP_S and HOLD_S as millisecond counts.
    //
    //   MS_DIV    = CLK_HZ / 1000
    //   ms_tick   = 1 for one clk cycle every millisecond, else 0.
    //
    // Example for CLK_HZ = 100 MHz:
    //   MS_DIV  = 100,000
    //   MS_DIV_W = CLOG2(100,000) = 17 bits
    localparam integer MS_DIV    = CLK_HZ / 1000;
    localparam integer MS_DIV_W  = (MS_DIV <= 1) ? 1 : CLOG2(MS_DIV);

    reg [MS_DIV_W-1:0] ms_div_cnt = {MS_DIV_W{1'b0}};
    reg                ms_tick    = 1'b0;

    always @(posedge clk) begin
        if (rst) begin
            ms_div_cnt <= {MS_DIV_W{1'b0}};
            ms_tick    <= 1'b0;
        end else if (ms_div_cnt == MS_DIV - 1) begin
            ms_div_cnt <= {MS_DIV_W{1'b0}};
            ms_tick    <= 1'b1;   // 1-cycle pulse every millisecond
        end else begin
            ms_div_cnt <= ms_div_cnt + 1'b1;
            ms_tick    <= 1'b0;
        end
    end

    // -----------------------------------------------------------------------------
    // 4) Warm-up mask (WARMUP_S seconds)
    // -----------------------------------------------------------------------------
    // Many PIR modules specify a warm-up period (30–60 s) during which the output
    // may chatter or flicker as the sensor stabilizes. We count WARMUP_S * 1000
    // milliseconds and ignore any rising edges before this period completes.
    localparam integer WU_MS = WARMUP_S * 1000;
    localparam integer WU_W  = (WU_MS <= 1) ? 1 : CLOG2(WU_MS + 1);

    reg  [WU_W-1:0] wu_ms_cnt = {WU_W{1'b0}};
    wire            warm_done;

    always @(posedge clk) begin
        if (rst) begin
            wu_ms_cnt <= {WU_W{1'b0}};
        end else if (ms_tick && (wu_ms_cnt < WU_MS)) begin
            wu_ms_cnt <= wu_ms_cnt + 1'b1;
        end
        // Once wu_ms_cnt reaches WU_MS, it stops incrementing. warm_done then remains 1.
    end

    assign warm_done = wu_ms_cnt >= WU_MS;

    // -----------------------------------------------------------------------------
    // 5) Rising-edge detection on debounced level (post warm-up)
    // -----------------------------------------------------------------------------
    // We detect a rising edge on the debounced signal db, but only *after* warm-up
    // is complete. Any edges before warm_done=1 are masked.
    reg db_z = 1'b0;  // previous debounced level

    always @(posedge clk) begin
        db_z <= db;
    end

    wire rise = (db & ~db_z) & warm_done;

    // -----------------------------------------------------------------------------
    // 6) Hold/stretch logic for "active" level
    // -----------------------------------------------------------------------------
    // Design:
    //   - Maintain a millisecond countdown hold_ms_cnt.
    //   - On each qualified rise:
    //        * Set active      = 1.
    //        * Reload hold_ms_cnt to HD_MS = HOLD_S * 1000.
    //        * Emit rise_pulse = 1 for this clock cycle.
    //   - While active==1 and ms_tick==1:
    //        * Decrement hold_ms_cnt until it reaches 0.
    //        * When hold_ms_cnt reaches 0, clear active.
    //
    // This implements a “motion recently detected” window of HOLD_S seconds that is
    // extended on each new motion event.
    localparam integer HD_MS = HOLD_S * 1000;
    localparam integer HD_W  = (HD_MS <= 1) ? 1 : CLOG2(HD_MS + 1);

    reg [HD_W-1:0] hold_ms_cnt = {HD_W{1'b0}};

    always @(posedge clk) begin
        if (rst) begin
            active      <= 1'b0;
            rise_pulse  <= 1'b0;
            hold_ms_cnt <= {HD_W{1'b0}};
        end else begin
            // Default: no edge this cycle
            rise_pulse <= 1'b0;

            if (rise) begin
                // New debounced motion event (post warm-up)
                active      <= 1'b1;
                hold_ms_cnt <= HD_MS;
                rise_pulse  <= 1'b1;  // 1-clk strobe
            end else if (active && ms_tick) begin
                // While “recent motion” is active, count down once per millisecond
                if (hold_ms_cnt == 0) begin
                    active <= 1'b0;   // timeout expired → no longer “recent”
                end else begin
                    hold_ms_cnt <= hold_ms_cnt - 1'b1;
                end
            end
        end
    end

endmodule

// ======================================================================================
// Implementation notes & practical guidance
// --------------------------------------------------------------------------------------
// • Debounce window:
//     For CLK_HZ=100 MHz and DEBOUNCE_MS=20, the input must remain different from the
//     current debounced value for 2_000_000 clocks (~20 ms) before db updates. This
//     filters brief spikes on the PIR output.
//
// • Warm-up & hold quantization:
//     WARMUP_S and HOLD_S are quantized to 1 ms resolution via ms_tick. For human-scale
//     intervals (seconds to minutes), 1 ms granularity is more than sufficient.
//
// • Reset behavior:
//     rst clears all state, including the warm-up counter and hold timer. After reset,
//     the module behaves as if power-up just occurred: it enforces the full WARMUP_S
//     delay again.
//
// • CDC / timing:
//     All logic is synchronous to clk. pir_raw should be driven by a board-level signal
//     compatible with the FPGA I/O banking (e.g., 3V3 LVCMOS) and is safely synchronized
//     before entering any counters or comparators.
//
// • Usage with fan/lighting control:
//     - active is ideal for gating a fan, light, or other actuator that should remain
//       on for some time after motion. It acts as a “recent motion” window.
//     - rise_pulse is convenient for logging, event counters, or any one-shot behavior
//       that should occur on each new motion detection.
//
// ======================================================================================
