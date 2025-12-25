`timescale 1ns/1ps
/*==============================================================================
  Module  : i2c_master
  Role    : Tiny, single-byte, open-drain I²C master with explicit START/STOP/
            WRITE/READ commands, suitable as a reusable IP block behind higher-
            level protocol engines (e.g., tof_sensor for ISL29501).

  High-level behavior:
    - Presents a very small “transaction micro-engine” that:
        * Generates legal I²C waveforms on SCL/SDA (open-drain).
        * Issues START and STOP conditions on demand.
        * Transmits one 8-bit data byte and samples the slave ACK/NACK.
        * Receives one 8-bit data byte and returns it to the caller, with the
          option to send ACK (for more bytes) or NACK (for last byte).
    - All multi-byte sequences (e.g., address, register pointer, data bursts)
      are built by an upstream FSM (such as tof_sensor) by sequencing:
        cmd_start → cmd_write(addr) → cmd_write(reg) → cmd_read(data) → cmd_stop

  Open-drain bus conventions:
    - I²C is wired-AND with pull-ups; “1” is the idle level, “0” is driven.
    - This module *never* drives a logic ‘1’ on SCL/SDA. Instead it exposes:
        scl_oen, sda_oen : open-drain enables
          0 → actively pull the line LOW (drive 0)
          1 → release to Hi-Z and let the external pull-up drive HIGH
    - External pads must be constrained as open-drain (or normal outputs with
      external resistors) at the board/XDC level.

  Host command contract:
    - Inputs: cmd_start, cmd_stop, cmd_write, cmd_read, rd_send_nack, wr_byte.
    - Outputs: rd_byte, got_ack, busy, done, err.
    - Call pattern:
        • The host asserts exactly one of cmd_* HIGH for a *single* clk cycle.
        • If busy = 1, new commands are ignored (caller must wait).
        • For each accepted command:
            - busy goes high until the operation is complete.
            - done pulses HIGH for 1 clk at completion (falling edge of busy).
        • For WRITE:
            - wr_byte is shifted out, MSB first.
            - got_ack=1 if the slave drove SDA=0 on the 9th bit (ACK).
            - got_ack=0 (and err set) on NACK (SDA=1).
        • For READ:
            - rd_byte is filled with 8 bits sampled from SDA (MSB first).
            - rd_send_nack=1 → send NACK at end (last byte in sequence).
            - rd_send_nack=0 → send ACK (caller intends to read more bytes).

  Timing model:
    - Single clock domain (clk) for everything; no derived/secondary clocks.
    - Internal “half-bit” tick at:
          TICKS_PER_PHASE = CLK_HZ / (2 * I2C_HZ)
      Each I²C bit is split into:
        • LOW phase  (data setup/hold while SCL=0)
        • HIGH phase (sampling when SCL=1)
    - All SCL/SDA changes occur on phase boundaries driven by tick.
    - We *observe* scl_in before sampling to tolerate clock-stretching slaves
      (slaves may hold SCL LOW even when we release it).

  Safety / spec helpers:
    - ENFORCE_TBUF:
        • Inserts a STOP→START bus-free interval (tBUF) measured in fabric
          cycles; keeps us safely within I²C spec, especially at 100 kHz.
    - STRETCH_WD_US:
        • Optional watchdog that trips if scl_in remains LOW too long after we
          release SCL (scl_oen=1). On timeout:
            err is set and the FSM bails to ST_DONE.

  Design limitations (intentional for simplicity):
    - Single-byte, non-pipelined operations:
        • Each cmd_* handles only one byte; host must sequence multiple calls.
    - No arbitration / multi-master support:
        • We do not detect “arbitration lost” by watching SDA while we release.
    - No automatic bus recovery:
        • If a slave holds SDA LOW indefinitely, an upstream policy must decide
          whether to toggle SCL, reset, or otherwise clear the bus.

  Integration notes:
    - This block is designed to be driven by a higher-level controller (e.g.,
      tof_sensor). That controller:
        • Watches done and got_ack/err.
        • Chains commands to realize register read/write sequences and handles
          any error-response policy (retry, abort, etc.).
    - Typical configuration (400 kHz fast mode at 100 MHz fabric clock):
        CLK_HZ = 100_000_000, I2C_HZ = 400_000
      ⇒ TICKS_PER_PHASE ≈ 125, bit time ≈ 2.5 μs, SCL ≈ 400 kHz.

==============================================================================*/
module i2c_master #(
    // --------------------- Rate configuration --------------------------------
    parameter integer CLK_HZ = 100_000_000, // fabric clock frequency (Hz)
    parameter integer I2C_HZ = 100_000,     // target SCL bit rate (Hz): 100k/400k typical

    // --------------------- Spec helpers / safeguards -------------------------
    parameter integer ENFORCE_TBUF    = 1,  // 1=enforce STOP→START bus-free time (tBUF)
    parameter integer TBUF_US_DEFAULT = (I2C_HZ <= 100_000) ? 5 : 2,
    //   For Standard-mode (≤100 kHz) spec tBUF ≈ 4.7 μs → we round up to 5 μs.
    //   For Fast-mode (≤400 kHz) spec tBUF ≈ 1.3 μs → we round up to 2 μs.

    parameter integer STRETCH_WD_US   = 0   // 0=disabled; else watchdog for SCL held LOW
)(
    //==========================================================================
    // Fabric clock and reset
    //==========================================================================
    input  wire clk, // single timing domain; all flops triggered on posedge clk
    input  wire rst, // synchronous, active-HIGH reset for all state

    //==========================================================================
    // I²C pads (open-drain interface)
    //==========================================================================
    input  wire scl_in, // sampled SCL pad level after pull-up and wiring
    output reg  scl_oen,// open-drain enable for SCL: 0=drive LOW, 1=release (Hi-Z → HIGH via pull-up)

    input  wire sda_in, // sampled SDA pad level (data / ACK / NACK)
    output reg  sda_oen,// open-drain enable for SDA: 0=drive LOW, 1=release (Hi-Z → HIGH via pull-up)

    //==========================================================================
    // Host command interface (one command at a time)
    //==========================================================================
    input  wire        cmd_start,     // issue a START condition (SDA:1→0 while SCL=1)
    input  wire        cmd_stop,      // issue a STOP  condition (SDA:0→1 while SCL=1)
    input  wire        cmd_write,     // transmit one data byte from 'wr_byte'
    input  wire        cmd_read,      // receive one data byte into 'rd_byte'
    input  wire        rd_send_nack,  // READ: 1 => send NACK after byte, 0 => send ACK

    input  wire [7:0]  wr_byte,       // payload byte to transmit during WRITE
    output reg  [7:0]  rd_byte,       // payload byte captured during READ

    output reg         got_ack,       // after WRITE: 1 if slave drove ACK (SDA=0), 0 on NACK (SDA=1)
    output reg         busy,          // high while a command is executing; new commands ignored
    output reg         done,          // 1-cycle completion pulse for each accepted command
    output reg         err            // sticky error (e.g., NACK, watchdog) — clears on reset
);

    //==========================================================================
    // 1) Utility: integer ceil(log2(N)) for counter sizing
    //==========================================================================
    function integer CLOG2;
        input integer value;
        integer v;
        begin
            v = value - 1;                // count how many times we can halve until 0
            for (CLOG2 = 0; v > 0; CLOG2 = CLOG2 + 1)
                v = v >> 1;              // logical right shift by 1 ≡ floor(value/2)
        end
    endfunction

    //==========================================================================
    // 2) SCL timing generator — half-bit strobe 'tick'
    //
    //   I²C bit period Tbit = 1/I2C_HZ.
    //   We divide it into two equal phases:
    //      LOW phase  (SCL=0): SDA can change; slaves sample at end of phase.
    //      HIGH phase (SCL=1): we sample SDA; slaves may stretch by holding SCL low.
    //
    //   TICKS_PER_PHASE = CLK_HZ / (2*I2C_HZ)
    //   tick pulses once per phase boundary; all FSM transitions that affect SCL/SDA
    //   are locked to this strobe.
    //==========================================================================
    localparam integer TICKS_PER_PHASE = (CLK_HZ/(I2C_HZ*2)); // integer division; any rounding is small
    localparam integer TW              = CLOG2(TICKS_PER_PHASE);

    reg [TW-1:0] tickc;   // [opcode:reg]  [operands: counts phase cycles 0..TICKS_PER_PHASE-1]
    reg          tick;    // [opcode:reg]  [operands: 1-cycle strobe at each half-bit boundary]

    // ---- Static guards (compile-time checks) --------------------------------
    initial begin
        if (CLK_HZ < (2*I2C_HZ))
            $error("i2c_master: CLK_HZ (%0d) must be >= 2*I2C_HZ (%0d) for half-bit timing.",
                   CLK_HZ, I2C_HZ);
        if (TICKS_PER_PHASE < 2)
            $error("i2c_master: TICKS_PER_PHASE=%0d too small; raise CLK_HZ or lower I2C_HZ.",
                   TICKS_PER_PHASE);
    end

    always @(posedge clk) begin
        tick <= 1'b0;                                     // [opcode:nba] [operands: default deassert]
        if (rst) begin
            tickc <= {TW{1'b0}};                          // [opcode:nba] [operands: reset counter]
        end else if (tickc == TICKS_PER_PHASE-1) begin    // [opcode:test] [operands: terminal count?]
            tickc <= {TW{1'b0}};                          // [opcode:nba] [operands: wrap]
            tick  <= 1'b1;                                // [opcode:nba] [operands: emit half-bit strobe]
        end else begin
            tickc <= tickc + 1'b1;                        // [opcode:nba] [operands: advance]
        end
    end

    //==========================================================================
    // 3) Optional STOP→START bus-free timing (tBUF, I²C spec requirement)
    //
    //   After issuing a STOP, the bus should remain idle for at least tBUF
    //   before the next START. When ENFORCE_TBUF=1, we implement that gap as
    //   raw fabric cycles (independent of 'tick').
    //==========================================================================
    localparam integer TBUF_TICKS = (CLK_HZ/1_000_000) * TBUF_US_DEFAULT; // cycles for default µs
    localparam integer TBUF_W     = (TBUF_TICKS <= 1) ? 1 : CLOG2(TBUF_TICKS);
    reg [TBUF_W-1:0] tbufc;   // [opcode:reg] [operands: tBUF cycle counter]

    //==========================================================================
    // 4) Optional clock-stretch watchdog
    //
    //   Some slaves may hold SCL low (“stretching”) after we release it. This
    //   is legal within bounds, but a stuck-low SCL can deadlock the bus.
    //
    //   Policy:
    //     - If STRETCH_WD_US == 0 → disabled.
    //     - Else: count cycles where scl_oen==1 (we are releasing) *and*
    //             scl_in==0 (line is still low). If this persists for WD_TICKS
    //             cycles, set err and bail to ST_DONE.
    //==========================================================================
    localparam integer WD_TICKS = (STRETCH_WD_US == 0) ? 0 : (CLK_HZ/1_000_000) * STRETCH_WD_US;
    localparam integer WD_W     = (WD_TICKS <= 1) ? 1 : CLOG2(WD_TICKS);
    reg [WD_W-1:0] stretchc;

    //==========================================================================
    // 5) FSM state, datapath, and intent flags
    //==========================================================================
    localparam [3:0]
        IDLE         = 4'd0,   // waiting for a command
        ST_START_A   = 4'd1,   // START phase A: SDA:1→0 while SCL=1
        ST_START_B   = 4'd2,   // START phase B: pull SCL low → enter data clocking
        ST_STOP_A    = 4'd3,   // STOP phase A: ensure SDA low while SCL low; then release SCL
        ST_STOP_B    = 4'd4,   // STOP phase B: with SCL high, release SDA (0→1) → STOP
        ST_WBIT_LOW  = 4'd5,   // WRITE: low phase for current data bit (drive SDA)
        ST_WBIT_HIGH = 4'd6,   // WRITE: high phase; slave samples, then next bit / ACK
        ST_WACK_LOW  = 4'd7,   // WRITE: low phase for ACK bit (master releases SDA)
        ST_WACK_HIGH = 4'd8,   // WRITE: high phase; sample SDA (ACK/NACK)
        ST_RBIT_LOW  = 4'd9,   // READ: low phase; master releases SDA
        ST_RBIT_HIGH = 4'd10,  // READ: high phase; sample SDA into shift register
        ST_RACK_LOW  = 4'd11,  // READ: low phase; master drives ACK(0)/NACK(1)
        ST_RACK_HIGH = 4'd12,  // READ: high phase; complete handshake and commit byte
        ST_DONE      = 4'd13,  // 1-cycle completion pulse, then return to IDLE
        ST_TBUF      = 4'd14;  // enforce tBUF bus-free interval after STOP (if enabled)

    reg [3:0] st;      // [opcode:reg] [operands: main FSM state register]
    reg [7:0] sh;      // [opcode:reg] [operands: TX/RX shift register (MSB-first)]
    reg [2:0] bitc;    // [opcode:reg] [operands: bit counter 7..0]

    // “Intent” flags are helpful for debug/tracing (not required for function).
    reg       do_read;
    reg       do_write;
    reg       do_start;
    reg       do_stop;
    reg       want_nack; // READ: 1 => send NACK after byte (last byte), 0 => send ACK (more bytes to follow)

    //==========================================================================
    // 6) Main FSM — I²C bit-level waveforms and protocol adherence
    //
    //   Principles enforced:
    //     • SDA changes only while SCL is LOW.
    //     • SDA is sampled while SCL is HIGH (on a tick boundary).
    //     • START = SDA falling edge while SCL HIGH.
    //       STOP  = SDA rising edge while SCL HIGH.
    //     • Tolerant of clock stretching by always *observing* scl_in before
    //       sampling/advancing on HIGH phases.
    //==========================================================================
    always @(posedge clk) begin
        // Default: done is a 1-cycle pulse on the state that returns us to IDLE.
        done <= 1'b0; // [opcode:nba] [operands: default completion deasserted]

        if (rst) begin
            // -------- Global reset: release bus and clear all state ----------
            scl_oen   <= 1'b1;  // [opcode:nba] [operands: SCL released → HIGH via pull-up]
            sda_oen   <= 1'b1;  // [opcode:nba] [operands: SDA released → HIGH via pull-up]

            st        <= IDLE;  // [opcode:nba]
            busy      <= 1'b0;  // [opcode:nba]
            got_ack   <= 1'b0;  // [opcode:nba]
            err       <= 1'b0;  // [opcode:nba]

            rd_byte   <= 8'h00;
            sh        <= 8'h00;
            bitc      <= 3'd0;

            do_read   <= 1'b0;
            do_write  <= 1'b0;
            do_start  <= 1'b0;
            do_stop   <= 1'b0;
            want_nack <= 1'b0;

            tbufc     <= {TBUF_W{1'b0}};
            stretchc  <= {WD_W{1'b0}};
        end else begin
            // -----------------------------------------------------------------
            // Optional clock-stretch watchdog (if configured)
            // -----------------------------------------------------------------
            if (WD_TICKS != 0) begin
                if (scl_oen && !scl_in) begin
                    // We are *releasing* SCL, but the line is still LOW.
                    if (stretchc != WD_TICKS-1)
                        stretchc <= stretchc + 1'b1;
                    else
                        stretchc <= stretchc; // saturate
                end else begin
                    // Either we are actively driving LOW or line is HIGH → no stretch.
                    stretchc <= {WD_W{1'b0}};
                end
            end

            // If watchdog expires this cycle, override FSM behavior and terminate.
            if ((WD_TICKS != 0) && scl_oen && !scl_in && (stretchc == WD_TICKS-1)) begin
                err <= 1'b1;           // [opcode:nba] [operands: sticky error flag]
                st  <= ST_DONE;        // [opcode:nba] [operands: abort current command]
            end else begin
                // ---------------------------- FSM body -----------------------
                case (st)

                //----------------------------------------------------------------------
                // IDLE: interface is free. Accept exactly ONE command; priority:
                //         START > STOP > WRITE > READ.
                //----------------------------------------------------------------------
                IDLE: begin
                    busy      <= 1'b0;
                    got_ack   <= 1'b0;
                    do_read   <= 1'b0;
                    do_write  <= 1'b0;
                    do_start  <= 1'b0;
                    do_stop   <= 1'b0;
                    want_nack <= 1'b0;

                    //scl_oen   <= 1'b1; // release both lines → idle HIGH/HIGH
                    //sda_oen   <= 1'b1;

                    if (cmd_start) begin
                        busy     <= 1'b1;  // claim I²C master
                        do_start <= 1'b1;
                        st       <= ST_START_A;
                    end else if (cmd_stop) begin
                        busy    <= 1'b1;
                        do_stop <= 1'b1;
                        st      <= ST_STOP_A;
                    end else if (cmd_write) begin
                        busy     <= 1'b1;
                        do_write <= 1'b1;
                        sh       <= wr_byte; // preload TX shift register (MSB first)
                        bitc     <= 3'd7;    // start at bit[7]
                        st       <= ST_WBIT_LOW;
                    end else if (cmd_read) begin
                        busy      <= 1'b1;
                        do_read   <= 1'b0;   // (set to 1 if want to track; not used in logic)
                        bitc      <= 3'd7;   // will fill bits[7:0]
                        sh        <= 8'h00;  // clear RX buffer
                        want_nack <= rd_send_nack; // remember ACK/NACK intent for final handshake
                        st        <= ST_RBIT_LOW;
                    end
                end

                //----------------------------------------------------------------------
                // START condition (two phases)
                //   A: wait until SCL is HIGH, then SDA:1→0 while SCL stays HIGH.
                //   B: pull SCL LOW to enter bit-clock low phase.
                //----------------------------------------------------------------------
                ST_START_A: begin
                    scl_oen <= 1'b1; // release SCL → HIGH when bus idle
                    sda_oen <= 1'b1; // keep SDA released until legal START moment
                    if (scl_in && tick) begin
                        // Legal START: SDA falling while SCL=1
                        sda_oen <= 1'b0; // drive SDA LOW
                        st      <= ST_START_B;
                    end
                end

                ST_START_B: begin
                    if (tick) begin
                        scl_oen <= 1'b0; // pull SCL low → begin data clocking low phase
                        st      <= ST_DONE; // START command itself is now complete
                    end
                end

                //----------------------------------------------------------------------
                // STOP condition (two phases)
                //   A: ensure SDA=0 while SCL=0, then release SCL → HIGH.
                //   B: with SCL truly HIGH, release SDA → SDA rising while SCL=1.
                //----------------------------------------------------------------------
                ST_STOP_A: begin
                    sda_oen <= 1'b0; // drive SDA LOW
                    scl_oen <= 1'b0; // hold SCL LOW
                    if (tick) begin
                        scl_oen <= 1'b1; // release SCL; may be stretched by slaves
                        st      <= ST_STOP_B;
                    end
                end

                ST_STOP_B: begin
                    if (scl_in) begin
                        // With SCL high, releasing SDA gives us a legal STOP
                        sda_oen <= 1'b1; // SDA:0→1 while SCL=1
                        if (ENFORCE_TBUF && (TBUF_TICKS > 1)) begin
                            tbufc <= {TBUF_W{1'b0}};  // begin bus-free interval
                            st    <= ST_TBUF;
                        end else begin
                            st    <= ST_DONE;
                        end
                    end
                end

                //----------------------------------------------------------------------
                // tBUF: STOP→START bus-free time (if enabled)
                //   Hold both lines released HIGH for TBUF_TICKS fabric cycles.
                //----------------------------------------------------------------------
                ST_TBUF: begin
                    scl_oen <= 1'b1;
                    sda_oen <= 1'b1;
                    if (tbufc == TBUF_TICKS-1) begin
                        st <= ST_DONE;
                    end else begin
                        tbufc <= tbufc + 1'b1;
                    end
                end

                //----------------------------------------------------------------------
                // WRITE: transmit one data byte (MSB first), then sample ACK bit
                //   • For each data bit:
                //       - Low phase: drive SDA according to bit value, SCL low.
                //       - High phase: release SCL, wait for scl_in=1, hold, then
                //         advance to next bit or ACK cycle.
                //----------------------------------------------------------------------
                ST_WBIT_LOW: begin
                    // Open-drain encoding:
                    //   bit==0 → sda_oen=0 (drive LOW),
                    //   bit==1 → sda_oen=1 (release HIGH).
                    sda_oen <= sh[bitc];
                    scl_oen <= 1'b0;        // keep SCL low during data setup
                    if (tick) begin
                        scl_oen <= 1'b1;    // start HIGH phase
                        st      <= ST_WBIT_HIGH;
                    end
                end

                ST_WBIT_HIGH: begin
                    if (scl_in && tick) begin
                        if (bitc == 3'd0) begin
                            // All 8 bits transmitted; now handle 9th ACK bit.
                            st      <= ST_WACK_LOW;
                            sda_oen <= 1'b1; // release SDA so slave can drive ACK/NACK
                        end else begin
                            bitc <= bitc - 3'd1; // next bit index
                            st   <= ST_WBIT_LOW;
                        end
                    end
                end

                // ACK cycle: master clocks, slave drives SDA (ACK=0, NACK=1).
                ST_WACK_LOW: begin
                    scl_oen <= 1'b0;         // low phase before sampling ACK
                    if (tick) begin
                        scl_oen <= 1'b1;     // raise SCL for ACK sampling
                        st      <= ST_WACK_HIGH;
                    end
                end

                ST_WACK_HIGH: begin
                    if (scl_in && tick) begin
                        got_ack <= ~sda_in;  // ACK if SDA=0; NACK if SDA=1
                        if (sda_in) err <= 1'b1; // flag sticky error on NACK
                        st      <= ST_DONE;  // single-byte WRITE complete
                    end
                end

                //----------------------------------------------------------------------
                // READ: receive one data byte (MSB first), then send ACK/NACK
                //   • For each data bit:
                //       - Low phase: master releases SDA.
                //       - High phase: sample SDA into shift register.
                //----------------------------------------------------------------------
                ST_RBIT_LOW: begin
                    sda_oen <= 1'b1;  // release SDA so slave can drive data
                    scl_oen <= 1'b0;  // low phase
                    if (tick) begin
                        scl_oen <= 1'b1; // high phase for sampling
                        st      <= ST_RBIT_HIGH;
                    end
                end

                ST_RBIT_HIGH: begin
                    if (scl_in && tick) begin
                        sh[bitc] <= sda_in;  // sample current bit into correct position
                        if (bitc == 3'd0) begin
                            st <= ST_RACK_LOW;      // proceed to ACK/NACK bit
                        end else begin
                            bitc <= bitc - 3'd1;    // next bit
                            st   <= ST_RBIT_LOW;
                        end
                    end
                end

                // After reading a byte, master drives 9th bit: ACK(0) for “more”
                // or NACK(1) for “last byte”.
                ST_RACK_LOW: begin
                    scl_oen <= 1'b0;                          // low phase before handshake
                    sda_oen <= (want_nack ? 1'b1 : 1'b0);     // NACK => release (HIGH), ACK => drive LOW
                    if (tick) begin
                        scl_oen <= 1'b1;                      // raise SCL to clock handshake
                        st      <= ST_RACK_HIGH;
                    end
                end

                ST_RACK_HIGH: begin
                    if (scl_in && tick) begin
                        sda_oen <= 1'b1; // release SDA after the handshake bit
                        rd_byte <= sh;   // commit received byte to output register
                        st      <= ST_DONE;
                    end
                end

                //----------------------------------------------------------------------
                // DONE: common completion state
                //   - busy deasserted → interface free.
                //   - done pulses for exactly 1 cycle for host edge-detection.
                //----------------------------------------------------------------------
                ST_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;  // single-clock “operation finished” pulse
                    st   <= IDLE;
                end

                default: begin
                    st <= IDLE;   // defensive recovery path
                end
                endcase
            end // watchdog not tripped
        end
    end
endmodule
