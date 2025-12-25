`timescale 1ns/1ps
// Theory: Simulation uses 1 ns as the fundamental time unit; any fractional delays
//         are rounded to the nearest 1 ps. Synthesis tools ignore `timescale but
//         simulators depend on it for delays, #timing, and $time resolution.

/*==============================================================================
  Module  : sevenseg_clock_buttons
  Project : FPGA_Spatial_Mapping_Project (Nexys A7-100T)
  Clock   : Single 100 MHz fabric domain; all “slow” behaviors via tick enables.

  Role / High-Level Behavior
  --------------------------
  This module implements a self-contained “human interface clock” on the Nexys
  A7’s 8-digit, common-anode seven-segment display. Features:

    - HH:MM:SS timekeeping (24-hour format, 00:00:00 → 23:59:59).
    - Two pushbuttons for setting the time:
        * btnu : increment minutes (with hour carry).
        * btnr : increment hours (wrap 23 → 0).
    - A 1 Hz “heartbeat” LED proving that the clock/tick logic is alive.
    - Optional bring-up mode (DEBUG_BRINGUP=1) that scans a bright “8” across
      all digits to validate pinout, segment polarity, and anode ordering.

  Board-Level Assumptions (Nexys A7)
  ----------------------------------
    - Seven-segment display is common-anode and ACTIVE-LOW:
        * segment[6:0] = {A,B,C,D,E,F,G}:
            0 → segment LED ON, 1 → segment LED OFF.
        * dp           : 0 → decimal point ON, 1 → OFF.
        * anode[i]     : 0 → digit i ENABLED (anode[0] is rightmost digit).
    - Buttons are ACTIVE-HIGH.
        * btnc : reset (top-level typically debounces then passes it in).
        * btnu : increment minutes.
        * btnr : increment hours.
    - clk_100mhz is the on-board 100 MHz oscillator supplying the system clock.

  Design Approach & Timing Model
  ------------------------------
    - Single synchronous clock domain (clk_100mhz).
    - No derived/gated clocks are generated inside this module. All “slower”
      behaviors are realized as 1-cycle clock-enable pulses:
        * tick_1hz     : 1-cycle strobe at 1 Hz (timekeeper).
        * tick_refresh : 1-cycle strobe at ~1 kHz (7-seg multiplex refresh).
    - Asynchronous button inputs (btnu, btnr) are:
        1) 2-FF synchronized into the 100 MHz domain.
        2) Debounced using a 5 ms stable-time qualification window.
        3) Converted into clean 1-cycle rising-edge pulses (min_inc, hr_inc).

  Reset Policy
  ------------
    - btnc is treated as a synchronous, active-HIGH reset for all state in this
      module. In a typical design, the top-level has already debounced and
      synchronized btnc into a global reset (rst), which is then wired here.
    - Using only synchronous reset keeps STA simple and avoids hazards related
      to asynchronous reset de-assertion.

  Display Strategy
  ----------------
    - 7-seg is time-multiplexed:
        * A 3-bit scan index selects which digit is currently active.
        * On each tick_refresh (~1 kHz), the active digit advances.
        * A shared segment pattern and decimal point (cur_bcd + cur_dp) are
          driven for the currently selected digit.
    - Binary → BCD:
        * Hours, minutes, and seconds are kept in binary (hh:mm:ss).
        * Simple /10 and %10 operations derive BCD digits (synthesize easily).
    - Colon behavior:
        * The “colon” is synthesized by blinking the decimal point on the
          two minute digits (digits 2 and 3) using a bit of the seconds counter
          (seconds[0]) as a low-frequency toggle.

  Debug / Bring-Up Mode (DEBUG_BRINGUP = 1)
  -----------------------------------------
    - The main HH:MM:SS logic is bypassed.
    - The module instead:
        * marches an “8” across the digits at ~1 kHz, with all segments ON
          (except dp OFF).
        * uses a simple one-hot ACTIVE-LOW anode pattern.
    - This mode is ideal for validating:
        * XDC pin mapping (segments/anodes).
        * Segment polarity (ACTIVE-LOW correctness).
        * Anode ordering (which digit is which).

  Integration Notes
  -----------------
    - This module is purely a display/timekeeping peripheral; it has no
      knowledge of other subsystems (XADC, ToF, fan control, etc.).
    - It is safe to instantiate anywhere as long as:
        * clk_100mhz is a stable 100 MHz clock.
        * btnc/btnu/btnr are connected to appropriate debounced or raw
          pushbutton inputs as desired.
    - ledx is an optional heartbeat output; if left unconstrained in XDC,
      the tool may trim it.

==============================================================================*/
module sevenseg_clock_buttons #(
    parameter DEBUG_BRINGUP = 0  // 1 = bring-up scan test, 0 = full HH:MM:SS clock
)(
    input  wire        clk_100mhz, // 100 MHz system clock (global timing domain)
    input  wire        btnc,       // active-HIGH reset (synchronous in this module)
    input  wire        btnu,       // active-HIGH pushbutton: increment MINUTES
    input  wire        btnr,       // active-HIGH pushbutton: increment HOURS

    // Seven-segment outputs (ACTIVE-LOW per Nexys A7 reference manual)
    output reg  [7:0]  anode,      // digit enables, anode[0] = rightmost (LS digit)
    output reg  [6:0]  segment,    // segments {A,B,C,D,E,F,G}, low = ON
    output reg         dp,         // decimal point,             low = ON

    // Optional heartbeat LED to prove the 100 MHz clock/tick is alive.
    // If unconstrained in XDC, the tool will trim it (fine for space saving).
    output reg         ledx
);

    //==========================================================================
    // 1) Clock-enable tick generators (no derived clocks)
    //--------------------------------------------------------------------------
    // We count 100 MHz cycles and emit a 1-cycle pulse when each divider hits:
    //   - tick_1hz     : 1 Hz (timekeeping).
    //   - tick_refresh : ~1 kHz (7-seg multiplex refresh).
    //
    // Rationale:
    //   • Keep ALL logic in a single 100 MHz clock domain.
    //   • Use enables (tick pulses) to gate state updates at slower rates.
    //==========================================================================

    localparam integer SEC_DIV     = 100_000_000; // 100e6 / 100e6 = 1 Hz
    localparam integer REFRESH_DIV = 100_000;     // 100e6 / 100e3 ≈ 1 kHz

    // --- 1 Hz tick for timekeeping
    reg [26:0] sec_cnt   = 27'd0;  // ceil(log2(1e8)) = 27
    reg        tick_1hz  = 1'b0;   // one-cycle strobe each second

    always @(posedge clk_100mhz) begin
        if (btnc) begin
            sec_cnt  <= 27'd0;     // reset seconds counter
            tick_1hz <= 1'b0;      // deassert 1 Hz tick
        end else if (sec_cnt == SEC_DIV-1) begin
            sec_cnt  <= 27'd0;
            tick_1hz <= 1'b1;      // assert for this cycle only
        end else begin
            sec_cnt  <= sec_cnt + 1'b1;
            tick_1hz <= 1'b0;
        end
    end

    // --- ~1 kHz tick to refresh the multiplexed display
    reg [16:0] ref_cnt      = 17'd0; // ceil(log2(1e5)) = 17
    reg        tick_refresh = 1'b0;

    always @(posedge clk_100mhz) begin
        if (btnc) begin
            ref_cnt      <= 17'd0;
            tick_refresh <= 1'b0;
        end else if (ref_cnt == REFRESH_DIV-1) begin
            ref_cnt      <= 17'd0;
            tick_refresh <= 1'b1;
        end else begin
            ref_cnt      <= ref_cnt + 1'b1;
            tick_refresh <= 1'b0;
        end
    end

    //==========================================================================
    // 2) Heartbeat LED — toggles at 1 Hz to prove the system is alive
    //--------------------------------------------------------------------------
    // A simple visual “alive” indicator tied to the 1 Hz tick. This helps
    // verify that:
    //   - clk_100mhz is present and stable.
    //   - The SEC_DIV tick generator is functioning.
    //==========================================================================

    always @(posedge clk_100mhz) begin
        if (btnc)
            ledx <= 1'b0;
        else if (tick_1hz)
            ledx <= ~ledx;
    end

    //==========================================================================
    // 3) DEBUG_BRINGUP path:
    //    Scan a bright “8” across the digits for hardware checkout
    //--------------------------------------------------------------------------
    // When DEBUG_BRINGUP=1:
    //   - Ignore the clock/timekeeping logic.
    //   - On each tick_refresh, advance a 3-bit scan index.
    //   - Drive all segments ON (digit pattern “8”), with decimal point OFF.
    //   - Assert exactly one ACTIVE-LOW anode at a time, marching right→left.
    //
    // This mode is used to:
    //   - Verify XDC pin mapping for segments and anodes.
    //   - Confirm ACTIVE-LOW polarity.
    //   - Check that digit ordering matches expectations.
    //==========================================================================

    generate if (DEBUG_BRINGUP) begin : g_bringup

        // Digit scan index (0..7), one per 7-seg digit
        reg [2:0] scan_idx = 3'd0;

        always @(posedge clk_100mhz) begin
            if (btnc)
                scan_idx <= 3'd0;
            else if (tick_refresh)
                scan_idx <= scan_idx + 3'd1;
        end

        // Constant “8” pattern: all segments ON, decimal point OFF (ACTIVE-LOW).
        localparam [6:0] SEG_8 = 7'b0000000;

        always @(posedge clk_100mhz) begin
            segment <= SEG_8;
            dp      <= 1'b1; // OFF (active-LOW)
        end

        // One-hot ACTIVE-LOW anode march from rightmost to leftmost
        always @(posedge clk_100mhz) begin
            if (btnc) begin
                anode <= 8'hFF;         // all digits OFF (inactive HIGH)
            end else if (tick_refresh) begin
                case (scan_idx)
                    3'd0: anode <= 8'b1111_1110; // anode[0] low -> rightmost ON
                    3'd1: anode <= 8'b1111_1101;
                    3'd2: anode <= 8'b1111_1011;
                    3'd3: anode <= 8'b1111_0111;
                    3'd4: anode <= 8'b1110_1111;
                    3'd5: anode <= 8'b1101_1111;
                    3'd6: anode <= 8'b1011_1111;
                    3'd7: anode <= 8'b0111_1111; // anode[7] low -> leftmost ON
                    default: anode <= 8'hFF;
                endcase
            end
        end

    //==========================================================================
    // 4) Full design path:
    //    HH:MM:SS clock + button-based time set + multiplexed 7-seg display
    //--------------------------------------------------------------------------
    // When DEBUG_BRINGUP=0, synthesize the full clock:
    //
    //   4.a) Utility CLOG2 function (for counter sizing).
    //   4.b) Button conditioning (sync + debounce + edge-detect).
    //   4.c) Timekeeping state (seconds, minutes, hours).
    //   4.d) Binary → BCD conversion for each displayed digit.
    //   4.e) Display scan index and per-digit BCD selection.
    //   4.f) 7-seg decode (BCD → segment pattern) and anode control.
    //
    // This block implements a 24-hour, 1-second-resolution clock that wraps:
    //   23:59:59 + 1s → 00:00:00
    //==========================================================================

    end else begin : g_full_design

        //----------------------------------------------------------------------
        // 4.a) Utility: integer CLOG2 (ceil(log2(value)))
        //----------------------------------------------------------------------
        // Used to size counters based on a maximum count at elaboration time.
        //----------------------------------------------------------------------

        function integer CLOG2;
            input integer value;
            integer v;
            begin
                v = value - 1;
                for (CLOG2 = 0; v > 0; CLOG2 = CLOG2 + 1)
                    v = v >> 1;
            end
        endfunction

        //----------------------------------------------------------------------
        // 4.b) Button conditioning for btnu (minutes) and btnr (hours)
        //----------------------------------------------------------------------
        // Stages:
        //   (1) 2-FF synchronizers into clk_100mhz domain.
        //   (2) 5 ms debounce window using counters.
        //   (3) Rising-edge detect to produce 1-cycle increment pulses:
        //           min_inc : increment minutes (with hour carry).
        //           hr_inc  : increment hours (wrap 23→0).
        //
        // Debounce constants:
        //   DB_TIME_MS = 5 ms
        //   DB_MAX     = 100_000 * 5 = 500,000 cycles @ 100 MHz
        //   DB_W       = ceil(log2(500,001)) = 19 bits
        //----------------------------------------------------------------------

        // (1) 2-FF synchronizers for async buttons
        reg u_s0, u_s1;  // sync chain for btnu
        reg r_s0, r_s1;  // sync chain for btnr

        always @(posedge clk_100mhz) begin
            u_s0 <= btnu;  u_s1 <= u_s0;
            r_s0 <= btnr;  r_s1 <= r_s0;
        end

        // Debounce window parameters
        localparam integer DB_TIME_MS = 5;
        localparam integer DB_MAX     = (100_000 * DB_TIME_MS); // 500,000
        localparam integer DB_W       = CLOG2(DB_MAX+1);

        // (2) Debounce BTN U (minutes)
        reg [DB_W-1:0] u_cnt = {DB_W{1'b0}};
        reg            u_db  = 1'b0; // debounced level

        always @(posedge clk_100mhz) begin
            if (btnc) begin
                u_cnt <= {DB_W{1'b0}};
                u_db  <= 1'b0;
            end else if (u_s1 == u_db) begin
                // Input matches debounced level: restart timer
                u_cnt <= {DB_W{1'b0}};
            end else if (u_cnt == DB_MAX[DB_W-1:0]) begin
                // Input has been stable for entire debounce window: accept new level
                u_db  <= u_s1;
                u_cnt <= {DB_W{1'b0}};
            end else begin
                // Input differs from debounced level but not yet stable long enough
                u_cnt <= u_cnt + 1'b1;
            end
        end

        // (2) Debounce BTN R (hours)
        reg [DB_W-1:0] r_cnt = {DB_W{1'b0}};
        reg            r_db  = 1'b0; // debounced level

        always @(posedge clk_100mhz) begin
            if (btnc) begin
                r_cnt <= {DB_W{1'b0}};
                r_db  <= 1'b0;
            end else if (r_s1 == r_db) begin
                r_cnt <= {DB_W{1'b0}};
            end else if (r_cnt == DB_MAX[DB_W-1:0]) begin
                r_db  <= r_s1;
                r_cnt <= {DB_W{1'b0}};
            end else begin
                r_cnt <= r_cnt + 1'b1;
            end
        end

        // (3) Rising-edge detect to create clean 1-cycle increment pulses
        reg u_db_d, r_db_d;

        always @(posedge clk_100mhz) begin
            u_db_d <= u_db;
            r_db_d <= r_db;
        end

        wire min_inc =  u_db & ~u_db_d; // “press” pulse for minutes
        wire hr_inc  =  r_db & ~r_db_d; // “press” pulse for hours

        //----------------------------------------------------------------------
        // 4.c) Timekeeper: hh:mm:ss with 24-hour rollover
        //----------------------------------------------------------------------
        //   - seconds : 0..59
        //   - minutes : 0..59
        //   - hours   : 0..23
        //
        // Priority:
        //   1) btnc reset → full reset to 00:00:00
        //   2) hr_inc     → increment hours only (wrap 23→0)
        //   3) min_inc    → increment minutes; if 59→0, carry hour
        //   4) tick_1hz   → normal timekeeping
        //----------------------------------------------------------------------

        reg [5:0] seconds = 6'd0; // 0..59
        reg [5:0] minutes = 6'd0; // 0..59
        reg [4:0] hours   = 5'd0; // 0..23

        always @(posedge clk_100mhz) begin
            if (btnc) begin
                seconds <= 6'd0;
                minutes <= 6'd0;
                hours   <= 5'd0;
            end else if (hr_inc) begin
                // Hour set takes priority
                hours <= (hours == 23) ? 5'd0 : hours + 1'b1;
            end else if (min_inc) begin
                // Minute set, with hour carry
                if (minutes == 59) begin
                    minutes <= 6'd0;
                    hours   <= (hours == 23) ? 5'd0 : hours + 1'b1;
                end else begin
                    minutes <= minutes + 1'b1;
                end
            end else if (tick_1hz) begin
                // Normal timekeeping each 1 Hz tick
                if (seconds == 59) begin
                    seconds <= 6'd0;
                    if (minutes == 59) begin
                        minutes <= 6'd0;
                        hours   <= (hours == 23) ? 5'd0 : hours + 1'b1;
                    end else begin
                        minutes <= minutes + 1'b1;
                    end
                end else begin
                    seconds <= seconds + 1'b1;
                end
            end
        end

        //----------------------------------------------------------------------
        // 4.d) Binary → BCD nibbles for each displayed digit
        //----------------------------------------------------------------------
        // BCD mapping (per digit position, rightmost = digit 0):
        //   digit 0 : seconds units  (sec_u)
        //   digit 1 : seconds tens   (sec_t)
        //   digit 2 : minutes units  (min_u)
        //   digit 3 : minutes tens   (min_t)
        //   digit 4 : hours   units  (hr_u)
        //   digit 5 : hours   tens   (hr_t)
        //   digit 6 : blank (4'hF)
        //   digit 7 : blank (4'hF)
        //
        // Simple /10 and %10 operations synthesize acceptably for these small
        // ranges and are clearer than manual LUT-based conversion.
        //----------------------------------------------------------------------

        wire [3:0] sec_u = seconds % 10;
        wire [3:0] sec_t = seconds / 10;
        wire [3:0] min_u = minutes % 10;
        wire [3:0] min_t = minutes / 10;
        wire [3:0] hr_u  = hours   % 10;
        wire [3:0] hr_t  = hours   / 10;

        //----------------------------------------------------------------------
        // 4.e) Display scan index: which digit is active right now?
        //----------------------------------------------------------------------
        //   - scan_idx cycles 0..7 at ~1 kHz (tick_refresh).
        //   - For each scan_idx, we select:
        //       * cur_bcd : which BCD digit to display.
        //       * cur_dp  : decimal point state for that digit.
        //
        // Colon behavior:
        //   - Use dp on the minutes digits (digits 2 and 3) to emulate a colon.
        //   - cur_dp is toggled by ~seconds[0], giving a low-frequency blink.
        //----------------------------------------------------------------------

        reg [2:0] scan_idx = 3'd0;

        always @(posedge clk_100mhz) begin
            if (btnc)
                scan_idx <= 3'd0;
            else if (tick_refresh)
                scan_idx <= scan_idx + 3'd1;
        end

        reg [3:0] cur_bcd;
        reg       cur_dp;

        always @* begin
            case (scan_idx)
                3'd0: begin cur_bcd = sec_u; cur_dp = 1'b1;          end
                3'd1: begin cur_bcd = sec_t; cur_dp = 1'b1;          end
                3'd2: begin cur_bcd = min_u; cur_dp = ~seconds[0];   end
                3'd3: begin cur_bcd = min_t; cur_dp = ~seconds[0];   end
                3'd4: begin cur_bcd = hr_u;  cur_dp = 1'b1;          end
                3'd5: begin cur_bcd = hr_t;  cur_dp = 1'b1;          end
                // Two leftmost digits reserved / blank
                3'd6: begin cur_bcd = 4'hF;  cur_dp = 1'b1;          end
                3'd7: begin cur_bcd = 4'hF;  cur_dp = 1'b1;          end
                default: begin cur_bcd = 4'hF; cur_dp = 1'b1;        end
            endcase
        end

        //----------------------------------------------------------------------
        // 4.f) 7-seg decode (BCD → ACTIVE-LOW segments) and anode control
        //----------------------------------------------------------------------
        // seven_decode:
        //   - Inputs: BCD digit 0..9, or 4'hF for “blank”.
        //   - Output: segment[6:0] with ACTIVE-LOW encoding.
        //
        // Registering outputs:
        //   - segment and dp are registered for clean timing to I/O pins.
        //   - anode is updated only on tick_refresh to match scan_idx updates.
        //----------------------------------------------------------------------

        function [6:0] seven_decode;
            input [3:0] bcd;
            begin
                case (bcd)
                    4'd0: seven_decode = 7'b1000000;
                    4'd1: seven_decode = 7'b1111001;
                    4'd2: seven_decode = 7'b0100100;
                    4'd3: seven_decode = 7'b0110000;
                    4'd4: seven_decode = 7'b0011001;
                    4'd5: seven_decode = 7'b0010010;
                    4'd6: seven_decode = 7'b0000010;
                    4'd7: seven_decode = 7'b1111000;
                    4'd8: seven_decode = 7'b0000000;
                    4'd9: seven_decode = 7'b0010000;
                    default: seven_decode = 7'b1111111; // all OFF (blank)
                endcase
            end
        endfunction

        // Register the segment outputs for clean timing
        always @(posedge clk_100mhz) begin
            if (btnc) begin
                segment <= 7'b1111111; // all segments OFF
                dp      <= 1'b1;       // OFF (active-LOW)
            end else begin
                segment <= seven_decode(cur_bcd);
                dp      <= cur_dp;
            end
        end

        // ACTIVE-LOW anode enables; advance only on refresh tick
        always @(posedge clk_100mhz) begin
            if (btnc) begin
                anode <= 8'hFF;       // all digits OFF
            end else if (tick_refresh) begin
                case (scan_idx)
                    3'd0: anode <= 8'b1111_1110;
                    3'd1: anode <= 8'b1111_1101;
                    3'd2: anode <= 8'b1111_1011;
                    3'd3: anode <= 8'b1111_0111;
                    3'd4: anode <= 8'b1110_1111;
                    3'd5: anode <= 8'b1101_1111;
                    3'd6: anode <= 8'b1011_1111;
                    3'd7: anode <= 8'b0111_1111;
                    default: anode <= 8'hFF;
                endcase
            end
        end

    end endgenerate

endmodule
