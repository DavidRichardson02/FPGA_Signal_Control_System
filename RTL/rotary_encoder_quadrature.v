/*==============================================================================
  Module  : rotary_encoder_quadrature
  Target  : KS0013 keyestudio Rotary Encoder on Nexys A7 Pmod (e.g., JC)
  Clock   : Single 100 MHz fabric domain

  Purpose :
    Front-end for a simple, 2-channel mechanical rotary encoder with pushbutton
    (CLK, DT, SW). It:

      - Synchronizes and debounces the raw encoder pins into the 100 MHz domain.
      - Detects clean falling edges on the CLK channel (one per detent).
      - On each debounced falling edge of CLK, samples DT to determine
        direction (CLK/DT quadrature), matching the Arduino example:
            * DT = 1 → "forward"  (step_dir = 1)
            * DT = 0 → "reverse"  (step_dir = 0)
      - Emits:
            step_pulse : 1-cycle pulse per detent.
            step_dir   : direction for that step (1=CW, 0=CCW).
            pos        : signed 16-bit position counter (for debug/LEDs).
      - Also debounces the pushbutton SW into:
            btn_level  : debounced level.
            btn_pulse  : 1-cycle rising-edge pulse.

  Electrical note:
    The KS0013 board is basically a mechanical encoder + button to GND; the
    microcontroller normally provides pull-ups on the lines. On the Nexys A7,
    treat the encoder outputs as *inputs only* and enable internal pull-ups or
    add external resistors to 3V3. Do NOT drive the Pmod pins from 5V logic.

==============================================================================*/

`timescale 1ns/1ps

module rotary_encoder_quadrature #(
    // Fabric clock frequency (Hz).
    parameter integer CLK_HZ       = 100_000_000,
    // Debounce interval for mechanical contacts (µs).
    parameter integer DEBOUNCE_US  = 500        // 0.5 ms per stable edge
)(
    input  wire clk,           // 100 MHz fabric clock.
    input  wire rst,           // Synchronous, active-HIGH reset.

    input  wire enc_a_raw,     // Encoder "CLK" pin (quadrature channel A).
    input  wire enc_b_raw,     // Encoder "DT"  pin (quadrature channel B).
    input  wire btn_raw,       // Encoder pushbutton "SW" pin.

    output reg        step_pulse,  // 1-cycle pulse per detent.
    output reg        step_dir,    // 1 = forward (DT high on A falling), 0 = reverse.
    output reg signed [15:0] pos,  // Signed position counter (for debug).

    output wire       btn_level,   // Debounced button level.
    output wire       btn_pulse    // 1-cycle rising-edge pulse on press.
);

    // -------------------------------------------------------------------------
    // Derived constants: debounce window in clock ticks and its width.
    // -------------------------------------------------------------------------
    function integer CLOG2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (CLOG2 = 0; v > 0; CLOG2 = CLOG2 + 1)
                v = v >> 1;
        end
    endfunction

    localparam integer DB_TICKS =
        (CLK_HZ / 1_000_000) * DEBOUNCE_US;       // e.g. 100 MHz * 0.5 ms = 50_000
    localparam integer DB_W = CLOG2(DB_TICKS+1);  // counter width

    // -------------------------------------------------------------------------
    // 2-FF synchronizers for the three raw inputs (A, B, SW).
    // -------------------------------------------------------------------------
    reg a_s0 = 1'b0, a_s1 = 1'b0;
    reg b_s0 = 1'b0, b_s1 = 1'b0;
    reg sw_s0 = 1'b0, sw_s1 = 1'b0;

    always @(posedge clk) begin
        a_s0  <= enc_a_raw;
        a_s1  <= a_s0;

        b_s0  <= enc_b_raw;
        b_s1  <= b_s0;

        sw_s0 <= btn_raw;
        sw_s1 <= sw_s0;
    end

    // -------------------------------------------------------------------------
    // Debounce channel A (CLK): we care about clean falling edges.
    // -------------------------------------------------------------------------
    reg [DB_W-1:0] a_cnt = {DB_W{1'b0}};
    reg            a_db  = 1'b1;  // assume pull-up → idle HIGH
    reg            a_db_prev = 1'b1;

    always @(posedge clk) begin
        if (rst) begin
            a_cnt     <= {DB_W{1'b0}};
            a_db      <= 1'b1;
            a_db_prev <= 1'b1;
        end else begin
            if (a_s1 == a_db) begin
                // No change; reset debounce timer.
                a_cnt <= {DB_W{1'b0}};
            end else if (a_cnt == DB_TICKS[DB_W-1:0]) begin
                // Stable at new level for full window → accept.
                a_db      <= a_s1;
                a_cnt     <= {DB_W{1'b0}};
            end else begin
                a_cnt <= a_cnt + 1'b1;
            end

            a_db_prev <= a_db;
        end
    end

    wire a_fall = a_db_prev & ~a_db;  // debounced falling edge on A.

    // -------------------------------------------------------------------------
    // Debounce channel B (DT) for a clean direction sample.
    // -------------------------------------------------------------------------
    reg [DB_W-1:0] b_cnt = {DB_W{1'b0}};
    reg            b_db  = 1'b1;  // assume pull-up
    // No need for b_db_prev; we only care about level.

    always @(posedge clk) begin
        if (rst) begin
            b_cnt <= {DB_W{1'b0}};
            b_db  <= 1'b1;
        end else begin
            if (b_s1 == b_db) begin
                b_cnt <= {DB_W{1'b0}};
            end else if (b_cnt == DB_TICKS[DB_W-1:0]) begin
                b_db  <= b_s1;
                b_cnt <= {DB_W{1'b0}};
            end else begin
                b_cnt <= b_cnt + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Debounce pushbutton SW → btn_level + btn_pulse.
    // -------------------------------------------------------------------------
    reg [DB_W-1:0] sw_cnt = {DB_W{1'b0}};
    reg            sw_db  = 1'b1;
    reg            sw_db_prev = 1'b1;

    always @(posedge clk) begin
        if (rst) begin
            sw_cnt     <= {DB_W{1'b0}};
            sw_db      <= 1'b1;
            sw_db_prev <= 1'b1;
        end else begin
            if (sw_s1 == sw_db) begin
                sw_cnt <= {DB_W{1'b0}};
            end else if (sw_cnt == DB_TICKS[DB_W-1:0]) begin
                sw_db  <= sw_s1;
                sw_cnt <= {DB_W{1'b0}};
            end else begin
                sw_cnt <= sw_cnt + 1'b1;
            end

            sw_db_prev <= sw_db;
        end
    end

    assign btn_level = sw_db;
    assign btn_pulse = sw_db & ~sw_db_prev;  // 1-cycle rising-edge pulse.

    // -------------------------------------------------------------------------
    // Step generation + position counter
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            step_pulse <= 1'b0;
            step_dir   <= 1'b0;
            pos        <= 16'sd0;
        end else begin
            // Default: no step this cycle.
            step_pulse <= 1'b0;

            if (a_fall) begin
                // On each debounced falling edge of A, sample B to determine direction.
                // Match Arduino example: DT=1 → forward (++), DT=0 → backward (--).
                step_dir   <= b_db;
                step_pulse <= 1'b1;

                if (b_db) begin
                    pos <= pos + 16'sd1;
                end else begin
                    pos <= pos - 16'sd1;
                end
            end
        end
    end

endmodule
