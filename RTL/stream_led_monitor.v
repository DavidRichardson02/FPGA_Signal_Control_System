`timescale 1ns/1ps
// ============================================================================
// Module        : stream_led_monitor
// Project       : FPGA_Spatial_Mapping_Project (Nexys A7-100T)
//
// Role in System
// ----------------------------------------------------------------------------
//   Provide a *human-visible* indication that a byte-stream interface is alive.
//   It does NOT know about protocol details; it only looks at a 1-cycle strobe
//   indicating that a byte was *successfully accepted* by the downstream sink.
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
//   This module turns that handshake strobe into two observables:
//
//     1) blink
//        - A single-bit “heartbeat” that toggles state on *every* accepted byte.
//        - At moderate byte rates this will appear as a fast flicker.
//        - Can route this to a single LED as a coarse “link active” indicator.
//
//     2) leds[7:0]
//        - An 8-bit activity counter that increments once every 2^DEC_SHIFT
//          accepted bytes.
//        - This decimation slows the update rate so humans can see change even
//          when the underlying stream is hundreds or thousands of bytes/s.
//        - Mapped directly to the Nexys A7 user LEDs as a “bar” that fills,
//          wraps, and keeps changing while the stream is alive.
//
//   Example with ToF mapper + UART at ~20 frames/s and 13 bytes/frame:
//     - Throughput ≈ 260 bytes/s.
//     - With DEC_SHIFT = 5 (2^5 = 32 bytes per LED increment):
//           ~260 / 32 ≈ 8.1 LED increments per second.
//       This yields a comfortably visible “twinkling” LED bar.
//
// External Interface
// ----------------------------------------------------------------------------
//   clk           : System fabric clock (100 MHz in this design).
//   rst           : Synchronous, active-HIGH reset. Clears all state.
//   byte_accepted : 1-cycle strobe, asserted exactly when one byte enters the
//                   downstream sink (e.g., tx_vld & tx_rdy).
//
//   leds[7:0]     : 8-bit activity counter. Increments (wraps naturally) once
//                   every 2^DEC_SHIFT accepted bytes.
//   blink         : Toggles on every accepted byte (no decimation).
//
// Parameterization
// ----------------------------------------------------------------------------
//   DEC_SHIFT : integer ≥ 0
//       - Sets the decimation factor between accepted bytes and LED increments.
//       - Effective “events per LED step” = 2^DEC_SHIFT.
//       - Internally, an event counter of width EW = max(1, DEC_SHIFT) is used.
//         • If DEC_SHIFT = 0, EW=1 → LEDs increment every 1 byte.
//         • If DEC_SHIFT = 5, EW=5 → LEDs increment every 32 bytes.
//       - The synthesis logic is independent of the absolute clk frequency; only
//         the rate of byte_accepted matters.
//
// Design Notes
// ----------------------------------------------------------------------------
//   • All logic is fully synchronous to 'clk'; there are NO gated or derived
//     clocks inside this module.
//   • 'ev_cnt' is an unsigned free-running event counter that increments on
//     every asserted byte_accepted.
//   • The LED increment condition uses a reduction-AND (&ev_cnt) *before*
//     the non-blocking assignment update, so the LEDs are bumped on the cycle
//     where ev_cnt transitions from all 1s → 0 (i.e., once per 2^EW events).
//   • On reset, all outputs are cleared:
//       leds  = 0 → LED bar starts empty.
//       blink = 0 → heartbeat starts from a known state.
//       ev_cnt= 0 → decimation phase aligned to reset.
//
// ============================================================================

module stream_led_monitor #(
    // Number of accepted bytes per LED update = 2^DEC_SHIFT.
    //   DEC_SHIFT = 0 ⇒ increment LEDs on every accepted byte.
    //   DEC_SHIFT = 5 ⇒ increment LEDs every 32 accepted bytes.
    parameter integer DEC_SHIFT = 5  // 2^5 = 32 bytes per LED increment (good for ~100–1000 B/s)
)(
    input  wire        clk,           // Fabric/system clock (100 MHz in this design)
    input  wire        rst,           // Synchronous, active-HIGH reset
    input  wire        byte_accepted, // 1-cycle pulse when a stream byte is accepted
                                      // (e.g., pkt_vld & pkt_rdy in UART path)

    output reg  [7:0]  leds,          // Decimated activity counter (mapped to user LEDs)
    output reg         blink          // Toggles on EVERY accepted byte (heartbeat)
);

    // ----------------------------------------------------------------------------
    // Event counter width (EW)
    // ----------------------------------------------------------------------------
    //   The event counter 'ev_cnt' decimates the byte_accepted pulse train so
    //   that leds[7:0] changes at a human-visible rate.
    //
    //   Effective behavior:
    //     - Width EW = max(1, DEC_SHIFT).
    //       • If DEC_SHIFT <= 0 → EW = 1 → increment LEDs every byte.
    //       • If DEC_SHIFT  = 5 → EW = 5 → increment LEDs every 32 bytes.
    //
    //   On each asserted byte_accepted:
    //     - 'ev_cnt' increments by 1 (mod 2^EW).
    //     - If the *previous* value of ev_cnt was all 1s ( &ev_cnt == 1 ),
    //       then we have just completed a full count of 2^EW events, so we
    //       increment leds[7:0].
    //
    //   Note on non-blocking assignments:
    //     - In the always block, RHS expressions use the "old" ev_cnt value.
    //       Therefore, the check `if (&ev_cnt)` fires exactly once per wrap.
    // ----------------------------------------------------------------------------
    localparam integer EW = (DEC_SHIFT < 1) ? 1 : DEC_SHIFT;
    reg [EW-1:0] ev_cnt;

    // ----------------------------------------------------------------------------
    // Main sequential process
    // ----------------------------------------------------------------------------
    //   • On reset:
    //       leds   ← 0
    //       blink  ← 0
    //       ev_cnt ← 0
    //
    //   • On each clock with byte_accepted asserted:
    //       blink  ← ~blink          (heartbeat toggles every byte)
    //       ev_cnt ← ev_cnt + 1      (free-running event counter)
    //       if (&ev_cnt)             (previous value was all ones)
    //           leds ← leds + 1      (decimated LED bar increments, wraps naturally)
    //
    //   • When byte_accepted = 0:
    //       - All state holds; no ghost activity.
    // ----------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            leds   <= 8'd0;
            blink  <= 1'b0;
            ev_cnt <= {EW{1'b0}};
        end else begin
            if (byte_accepted) begin
                // Heartbeat: toggles on *every* accepted byte.
                blink <= ~blink;

                // Event counter: counts accepted bytes modulo 2^EW.
                ev_cnt <= ev_cnt + 1'b1;

                // Decimated LED update: once per 2^EW accepted bytes.
                // Because of non-blocking semantics, '&ev_cnt' sees the "old"
                // value before the increment, so this condition is true
                // exactly when ev_cnt was all 1s on the previous cycle.
                if (&ev_cnt) begin
                    leds <= leds + 8'd1;
                end
            end
        end
    end

endmodule
