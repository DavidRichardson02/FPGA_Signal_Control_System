`timescale 1ns/1ps  // [opcode:`timescale] [operands: timeunit=1ns, timeprecision=1ps]
// Theory:
//   - Simulation time unit is 1 ns; simulator rounds fractional delays to 1 ps.
//   - Synthesis ignores `timescale, but testbenches/#delays/$time depend on it.
//   - Keep this consistent across CDC-related modules to make waveform correlation
//     (sys_clk vs pix_clk behavior) easier when debugging handshakes.

// =============================================================================
// MODULE: accel_cdc_bridge
// =============================================================================
// PURPOSE
//   Safely transfer *coherent* accelerometer samples from the 100 MHz system clock
//   domain into the ~25 MHz VGA pixel clock domain for HUD rendering.
//
//   This bridge transfers TWO parallel representations of the SAME physical sample:
//     1) accel_*_q7_*  : Q7-scaled signed 8-bit values (HUD-friendly).
//     2) accel_*_raw_* : raw signed 8-bit sensor registers (debug / bringup).
//
//   The key requirement is COHERENCY:
//     - X/Y/Z must belong to the same sample.
//     - Raw and Q7 must belong to the same sample.
//     - The pix-domain must never see “mixed” tuples (e.g., X from sample N,
//       Y from sample N+1) or raw from one sample and Q7 from another.
//
// APPROACH (REQ/ACK TOGGLE HANDSHAKE, HOLD-REG CDC)
//   This is a classic CDC pattern for multi-bit buses when the source can hold
//   data stable while the destination captures it:
//
//   SYS domain (producer, sys_clk):
//     - When a new sample is ready (accel_sample_valid_sys), latch all 6 bytes
//       into holding registers (x_q7_buf, ..., z_raw_buf).
//     - Toggle req_tog_sys to indicate “new bundle available.”
//     - Crucially: do NOT overwrite holding registers again until the previous
//       bundle has been acknowledged.
//
//   PIX domain (consumer, pix_clk):
//     - 2-FF synchronize req_tog_sys into pix_clk (req_tog_pix_ff1/ff2).
//     - Detect a toggle edge (req_tog_pix_ff1 XOR req_tog_pix_ff2).
//     - On detection, capture the entire holding-register bundle into pixel regs.
//     - Generate a 1-cycle pulse accel_update_pix to mark “new sample latched.”
//     - Assert acknowledgement by updating ack_tog_pix_reg to match the synced req.
//
//   SYS domain then 2-FF synchronizes ack_tog_pix back and only accepts the next
//   sample once ack matches req (“sys_ack_seen”).
//
// WHY THIS IS SAFE FOR A MULTI-BIT BUS
//   req_tog_sys is the only signal that crosses domains “as an event.”
//   The payload bus (the holding regs) is not synchronized bit-by-bit;
//   instead we guarantee it is stable across the entire capture window by:
//
//     (1) SYS only updates holding regs at accel_sample_valid_sys AND when the
//         previous transfer has been acknowledged (sys_ack_seen).
//     (2) Therefore, after req toggles, the holding regs remain constant until
//         the PIX side captures and sends the ack.
//
//   That stability window is what makes capturing x_q7_buf/.../z_raw_buf coherent
//   even though they are multi-bit signals.
//
// LIMITATIONS / DESIGN NOTES
//   - Throughput is limited by the handshake: SYS will drop/skip samples if
//     accel_sample_valid_sys asserts faster than PIX can ack.
//     (In use case: 100 Hz accel vs 25 MHz pix → plenty of margin.)
//   - This design intentionally prioritizes correctness/coherency over throughput.
//   - Reset behavior: sys_rst and pix_rst are assumed synchronous to their
//     respective clocks. If pix_rst is derived from sys_rst, ensure it is either
//     asserted long enough or synchronized for clean deassertion.
//   - The ack generation uses the post-synchronized request state; this prevents
//     “ping-pong” race conditions when req toggles around reset edges.
//
// INTERFACE SUMMARY
//   Inputs (sys_clk domain):
//     accel_*_q7_sys           : scaled signed 8-bit XYZ
//     accel_*_raw_sys          : raw signed 8-bit XYZ
//     accel_sample_valid_sys   : pulse indicating a new sample bundle is ready
//
//   Outputs (pix_clk domain):
//     accel_*_q7_pix           : coherent scaled XYZ captured in pix domain
//     accel_*_raw_pix          : coherent raw XYZ captured in pix domain
//     accel_update_pix         : 1-cycle strobe when pix regs update
//
// =============================================================================

module accel_cdc_bridge (
    // -------------------------------------------------------------------------
    // System domain (100 MHz)
    // -------------------------------------------------------------------------
    input  wire              sys_clk,
    input  wire              sys_rst,

    // Scaled Q7 (sys)
    input  wire signed [7:0] accel_x_q7_sys,
    input  wire signed [7:0] accel_y_q7_sys,
    input  wire signed [7:0] accel_z_q7_sys,

    // Raw (sys)
    input  wire signed [7:0] accel_x_raw_sys,
    input  wire signed [7:0] accel_y_raw_sys,
    input  wire signed [7:0] accel_z_raw_sys,

    // 1-cycle pulse: “new raw+q7 sample bundle ready” (sys_clk)
    input  wire              accel_sample_valid_sys,

    // -------------------------------------------------------------------------
    // Pixel domain (25 MHz)
    // -------------------------------------------------------------------------
    input  wire              pix_clk,
    input  wire              pix_rst,

    // Scaled Q7 (pix)
    output reg  signed [7:0] accel_x_q7_pix,
    output reg  signed [7:0] accel_y_q7_pix,
    output reg  signed [7:0] accel_z_q7_pix,

    // Raw (pix)
    output reg  signed [7:0] accel_x_raw_pix,
    output reg  signed [7:0] accel_y_raw_pix,
    output reg  signed [7:0] accel_z_raw_pix,

    // 1-cycle pulse: “pix-domain regs updated with a new coherent sample”
    output reg               accel_update_pix
);

    // =========================================================================
    // SYS domain holding registers (payload storage)
    // =========================================================================
    // These registers form the “stable payload bus” that the pix domain captures.
    // They update ONLY when:
    //   (a) a new sample arrives (accel_sample_valid_sys), AND
    //   (b) the previous sample has been acknowledged (sys_ack_seen).
    reg signed [7:0] x_q7_buf, y_q7_buf, z_q7_buf;
    reg signed [7:0] x_raw_buf, y_raw_buf, z_raw_buf;

    // =========================================================================
    // Handshake signals
    // =========================================================================
    // req_tog_sys:
    //   - toggled by sys domain to announce “new payload ready.”
    // ack_tog_pix:
    //   - toggled/updated by pix domain to acknowledge capture.
    //
    // Because these are single-bit toggles, we can safely synchronize them with
    // standard 2-flip-flop synchronizers in each direction.
    reg req_tog_sys;
    reg ack_tog_sys_ff1, ack_tog_sys_ff2;

    // ack toggle generated in pix domain (wire view in sys domain)
    wire ack_tog_pix;

    // sys_ack_seen:
    //   true when the synchronized ack equals the current request state,
    //   meaning “the last request has been observed and acknowledged.”
    wire sys_ack_seen = (ack_tog_sys_ff2 == req_tog_sys);

    // =========================================================================
    // SYS clocked process: latch payload + launch request toggle
    // =========================================================================
    always @(posedge sys_clk) begin
        if (sys_rst) begin
            // Payload reset
            x_q7_buf <= 8'sd0;  y_q7_buf <= 8'sd0;  z_q7_buf <= 8'sd0;
            x_raw_buf<= 8'sd0;  y_raw_buf<= 8'sd0;  z_raw_buf<= 8'sd0;

            // Handshake reset
            req_tog_sys     <= 1'b0;
            ack_tog_sys_ff1 <= 1'b0;
            ack_tog_sys_ff2 <= 1'b0;
        end else begin
            // (1) synchronize ACK back into sys domain
            ack_tog_sys_ff1 <= ack_tog_pix;
            ack_tog_sys_ff2 <= ack_tog_sys_ff1;

            // (2) accept new sample only if previous has been acknowledged
            if (accel_sample_valid_sys && sys_ack_seen) begin
                // Latch BOTH representations atomically (same sample)
                x_q7_buf  <= accel_x_q7_sys;
                y_q7_buf  <= accel_y_q7_sys;
                z_q7_buf  <= accel_z_q7_sys;

                x_raw_buf <= accel_x_raw_sys;
                y_raw_buf <= accel_y_raw_sys;
                z_raw_buf <= accel_z_raw_sys;

                // Launch a new transfer by toggling the request flag
                req_tog_sys <= ~req_tog_sys;
            end
        end
    end

    // =========================================================================
    // PIX domain: synchronize request toggle, capture payload, send ACK
    // =========================================================================
    reg req_tog_pix_ff1, req_tog_pix_ff2;
    reg ack_tog_pix_reg;

    assign ack_tog_pix = ack_tog_pix_reg;

    always @(posedge pix_clk) begin
        if (pix_rst) begin
            // Sync flops reset
            req_tog_pix_ff1 <= 1'b0;
            req_tog_pix_ff2 <= 1'b0;

            // Ack reset
            ack_tog_pix_reg <= 1'b0;

            // Outputs reset
            accel_x_q7_pix   <= 8'sd0;
            accel_y_q7_pix   <= 8'sd0;
            accel_z_q7_pix   <= 8'sd0;

            accel_x_raw_pix  <= 8'sd0;
            accel_y_raw_pix  <= 8'sd0;
            accel_z_raw_pix  <= 8'sd0;

            accel_update_pix <= 1'b0;
        end else begin
            // Default: no update pulse
            accel_update_pix <= 1'b0;

            // (1) 2-FF synchronize request toggle into pix domain
            req_tog_pix_ff1 <= req_tog_sys;
            req_tog_pix_ff2 <= req_tog_pix_ff1;

            // (2) Detect a toggle edge => new coherent payload available
            if (req_tog_pix_ff1 ^ req_tog_pix_ff2) begin
                // Capture BOTH representations from stable holding regs
                accel_x_q7_pix  <= x_q7_buf;
                accel_y_q7_pix  <= y_q7_buf;
                accel_z_q7_pix  <= z_q7_buf;

                accel_x_raw_pix <= x_raw_buf;
                accel_y_raw_pix <= y_raw_buf;
                accel_z_raw_pix <= z_raw_buf;

                // Pulse: indicates pix outputs updated this cycle
                accel_update_pix <= 1'b1;

                // (3) Acknowledge: match the post-sync request state
                // This causes sys_ack_seen to become true after sys-side resync.
                ack_tog_pix_reg <= req_tog_pix_ff2;
            end
        end
    end

endmodule
