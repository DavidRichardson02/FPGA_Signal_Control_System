`timescale 1ns/1ps  // [opcode: `timescale] [operands: timeunit=1ns, timeprecision=1ps]
                    // Simulation-only directive: time literals are in ns, rounded to 1 ps.
                    // Synthesis ignores this but testbenches (e.g., $time) obey it.

// ============================================================================
// mapper_packetizer.v  -- Build framed ToF packets and stream bytes over UART
// ============================================================================
//
// High-level role:
//   This block takes discrete Time-of-Flight (ToF) samples plus a heading and
//   status byte, snapshots them atomically, attaches a 32-bit timestamp, and
//   emits a framed byte stream suitable for a downstream UART or other
//   byte-oriented transport. It is intentionally *stateless* between packets
//   except for the free-running timestamp counter.
//
// Clock / reset / CDC:
//   • Single fabric clock domain 'clk'; no derived clocks are created.
//   • Synchronous, active-HIGH reset 'rst' initializes all internal state to
//     deterministic values.
//   • All incoming signals (sample_vld, theta_q15, dist_mm, status, tx_rdy)
//     are assumed synchronous to 'clk'. There are no internal synchronizers.
//
// Packet framing protocol (big-endian; CRC excludes sync headers):
//   Byte offsets (0-based) on the wire:
//     [0]  0x55          (sync header 1; NOT included in CRC)
//     [1]  0xAA          (sync header 2; NOT included in CRC)
//     [2]  t_us[31:24]   (timestamp in microseconds, MSB first)
//     [3]  t_us[23:16]
//     [4]  t_us[15:8]
//     [5]  t_us[7:0]
//     [6]  theta[15:8]   (signed Q1.15 turns; +0.5 turn ≡ 0x4000)
//     [7]  theta[7:0]
//     [8]  dist[15:8]    (unsigned millimeters)
//     [9]  dist[7:0]
//     [10] temp_q15[15:8]   (signed Q1.15 temperature estimate)
//     [11] temp_q15[7:0]
//     [12] duty_q15[15:8]   (signed Q1.15 fan duty from controller)
//     [13] duty_q15[7:0]
//     [14] status           (sensor quality/error bits)
//     [15] CRC[15:8]        (CRC16-CCITT over bytes [2..14])
//     [16] CRC[7:0]
//
//   CRC details:
//     • Polynomial: 0x1021 (CRC16-CCITT).
//     • Initialization: 16'hFFFF.
//     • Fold order: MSB-first, byte aligned; headers [0..1] are excluded.
//     • CRC now covers the extended payload bytes [2..14] (t_us, theta, dist,
//       temp_q15, duty_q15, status).
//
// Handshake contract (ready/valid, “fire-when-ready” style):
//   Upstream (producer):
//     • Asserts 'sample_vld' for exactly 1 clock cycle when a new ToF sample
//       is available.
//     • Contract: This pulse is only honored when the FSM is in P_IDLE; if
//       asserted in any other state, it is ignored (no internal queueing).
//
//   Downstream (consumer, e.g., UART byte-pusher):
//     • Asserts 'tx_rdy' whenever it can accept *one* more byte.
//     • This block asserts 'tx_vld' for exactly one cycle *only* when
//       'tx_rdy' is 1 in the same cycle. It never "parks" tx_vld high while
//       waiting for tx_rdy; instead it waits in state until tx_rdy is 1 and
//       then fires that byte.
//
//   Transfer semantics:
//     • A byte transfer occurs iff (tx_vld && tx_rdy) in the same clock.
//     • Packet length is fixed at 17 bytes.
//
// Timestamp model:
//   • Internally maintains a free-running microsecond counter 't_us'.
//   • Derived by dividing the fabric clock frequency (CLK_HZ) by 1e6.
//   • If CLK_HZ <= 1 MHz, each clock tick is treated as 1 µs (DIV <= 1).
//   • 'dbg_t_us' exposes this internal counter for logic-analyzer probing.
//
// Implementation notes:
//   • Inputs are snapshotted on 'sample_vld' in P_IDLE to avoid tearing.
//   • A simple FSM walks through the 17 bytes, feeding the CRC on payload
//     bytes [2..14] as they are emitted.
//   • 'theta_turn_q15' is presently unused but kept to preserve interface
//     compatibility with potential future formats.
// ============================================================================

module mapper_packetizer #(                         // [opcode:module] [operands: name=mapper_packetizer, has parameters]
    parameter integer CLK_HZ = 100_000_000          // [opcode:parameter] [operands: name=CLK_HZ, type=integer, default=1e8]
                                                    // Theory: drives 1 MHz microsecond tick via integer divider.
)(
    input  wire        clk,                         // [opcode:input] [operands:type=wire, name=clk]
                                                    // Theory: single synchronous domain; all flops posedge clk.
    input  wire        rst,                         // [opcode:input] [operands:type=wire, name=rst(active-high, sync)]
                                                    // Theory: synchronous reset → deterministic startup, no CDC.

    // New sample from scan scheduler / ToF + heading source
    input  wire        sample_vld,                  // [opcode:input] [operands: strobe=1 cycle]
                                                    // Contract: only honored in P_IDLE (no internal queue).
    input  wire [31:0] theta_turn_q15,              // [opcode:input] [operands: width=32, UNUSED in this format]
                                                    // Note: kept for interface compatibility; ignored by packet.
    input  wire [15:0] theta_q15,                   // [opcode:input] [operands: width=16, signed Q1.15 turns]
                                                    // Theory: fixed-point; +0.5 turn ≡ 0x4000; big-endian emit.
    input  wire [15:0] dist_mm,                     // [opcode:input] [operands: width=16, unsigned millimeters]
    input  wire [7:0]  status,                      // [opcode:input] [operands: width=8, sensor status/flags]

    // NEW: temperature and fan duty (Q1.15), snapshotted into the frame
    input  wire [15:0] temp_q15,                    // [opcode:input] [operands: width=16, signed Q1.15 temperature]
    input  wire [15:0] fan_duty_q15_dbg,            // [opcode:input] [operands: width=16, signed Q1.15 fan duty]

    // Byte stream out (ready/valid)
    output reg  [7:0]  tx_byte,                     // [opcode:output reg] [operands: width=8, current byte]
                                                    // Valid iff (tx_vld && tx_rdy) in same cycle.
    output reg         tx_vld,                      // [opcode:output reg] [operands: 1-cycle pulse]
                                                    // Design: never held high across backpressure.
    input  wire        tx_rdy,                      // [opcode:input] [operands: sink ready signal]

    // Exposed timestamp for debug/scoping
    output reg  [31:0] dbg_t_us                     // [opcode:output reg] [operands: width=32, mirrors t_us]
);                                                  // [opcode:portlist close] [operands: ')']

// ---------------- utilities ----------------
    function integer CLOG2;                         // [opcode:function] [operands: returns integer]
        input integer value;                        // [opcode:input(function)] [operands: value]
        integer v;                                  // [opcode:integer] [operands: local var v]
        begin                                       // [opcode:begin] [operands: start function body]
            v = value - 1;                          // [opcode:'='] [operands: v := value-1]
            for (CLOG2 = 0; v > 0; CLOG2 = CLOG2 + 1) // [opcode:for] [operands:init=0, cond=v>0, step=+1]
                v = v >> 1;                         // [opcode:'>>'] [operands: shift-right divides by 2]
        end                                         // [opcode:end] [operands: function body end]
    endfunction                                     // [opcode:endfunction] [operands:none]

// ---------------- 1 MHz timestamp ----------------
// Theory: Derive enable for 1-µs increments without a new clock domain.
// t_us increments every DIV fabric cycles (DIV = CLK_HZ / 1e6). If CLK_HZ ≤ 1e6,
// each clk tick itself is 1 µs (DIV≤1 branch).
    localparam integer DIV = (CLK_HZ/1_000_000);    // [opcode:localparam] [operands: DIV = integer division]
    localparam integer DW  = (DIV <= 1) ? 1 : CLOG2(DIV); // [opcode:localparam] [operands: counter width bits]
    reg [DW-1:0] usc;                               // [opcode:reg] [operands: µs divider counter, width=DW]
    reg [31:0]   t_us;                              // [opcode:reg] [operands: free-running timestamp (wraps)]

    always @(posedge clk) begin                     // [opcode:always] [operands:sens=posedge clk]
        if (rst) begin                              // [opcode:if] [operands: condition=rst]
            usc  <= {DW{1'b0}};                     // [opcode:nba '<='] [operands: replicate 0 DW times]
            t_us <= 32'd0;                          // [opcode:nba] [operands: literal 0, width=32]
        end                                         // [opcode:end] [operands: if-then block]
        // The following static condition is constant-folded by synthesis:
        else if (DIV <= 1) begin                    // [opcode:else if] [operands: compile-time true/false]
            t_us <= t_us + 32'd1;                   // [opcode:nba '+'] [operands: increment every clk]
        end else if (usc == DIV-1) begin            // [opcode:else if] [operands: equality compare (DW bits)]
            usc  <= {DW{1'b0}};                     // [opcode:nba] [operands: wrap to 0]
            t_us <= t_us + 32'd1;                   // [opcode:nba '+'] [operands: tick => +1 µs]
        end else begin                              // [opcode:else] [operands:none]
            usc <= usc + 1'b1;                      // [opcode:nba '+'] [operands: width-safe +1 (DW bits)]
        end                                         // [opcode:end] [operands: if/else chain]
    end                                             // [opcode:end] [operands: always block end]

// ---------------- Snapshot on sample ----------------
// Theory: De-couple incoming busses from stream timing; capture atomically.
    reg [31:0] t_latch;                             // [opcode:reg] [operands: latched t_us]
    reg [15:0] th_latch;                            // [opcode:reg] [operands: latched theta]
    reg [15:0] d_latch;                             // [opcode:reg] [operands: latched distance]
    reg [7:0]  s_latch;                             // [opcode:reg] [operands: latched status]
    // NEW: latched temperature and duty
    reg [15:0] temp_latch;                          // [opcode:reg] [operands: latched temp_q15]
    reg [15:0] duty_latch;                          // [opcode:reg] [operands: latched fan_duty_q15_dbg]

    wire [15:0] theta16 = theta_q15;                // [opcode:assign] [operands: pass-through 16-bit theta]
                                                    // Theory: If future formats want 32-bit, adapt here.

// ---------------- CRC16-CCITT (poly 0x1021, init 0xFFFF) ----------------
// Theory: Bit-serial LFSR update per byte, MSB-first, XOR-align byte at CRC high-end.
    function [15:0] crc16_ccitt;                    // [opcode:function] [operands: returns 16-bit]
        input [15:0] crc_in;                        // [opcode:input(function)] [operands: prior CRC]
        input [7:0]  data;                          // [opcode:input(function)] [operands: next byte]
        integer i;                                  // [opcode:integer] [operands: loop index]
        reg [15:0] c;                               // [opcode:reg] [operands: working CRC register]
        begin                                       // [opcode:begin]
            c = crc_in ^ {data, 8'h00};             // [opcode:'=','^','{}'] [operands: XOR with byte<<8]
            for (i = 0; i < 8; i = i + 1) begin     // [opcode:for] [operands: 8 per-bit steps]
                if (c[15])                          // [opcode:if] [operands: test MSB]
                    c = (c << 1) ^ 16'h1021;        // [opcode:'=','<<','^'] [operands: shift/xor poly]
                else
                    c = (c << 1);                   // [opcode:'=','<<'] [operands: shift only]
            end
            crc16_ccitt = c;                        // [opcode:function return] [operands: result=c]
        end                                         // [opcode:end]
    endfunction                                     // [opcode:endfunction]

// ---------------- Packet emit FSM ----------------
// Theory: “ready/valid in same cycle” producer. We assert tx_vld only inside
// if(tx_rdy) guards; this guarantees exactly-one-cycle transfers per byte.
//
// Note: PACK_LEN_BYTES is for documentation / debug.
    localparam integer PACK_LEN_BYTES = 17;         // [opcode:localparam] [operands: constant for docs]

    localparam [4:0]                                  // [opcode:localparam group] [operands: 5-bit encodings]
        P_IDLE   = 5'd0,                              // [opcode:const] [operands: idle]
        P_HDR0   = 5'd1,                              // [opcode:const] [operands: send 0x55]
        P_HDR1   = 5'd2,                              // [opcode:const] [operands: send 0xAA]
        P_T3     = 5'd3,                              // [opcode:const] [operands: t_us[31:24]]
        P_T2     = 5'd4,                              // [opcode:const] [operands: t_us[23:16]]
        P_T1     = 5'd5,                              // [opcode:const] [operands: t_us[15:8]]
        P_T0     = 5'd6,                              // [opcode:const] [operands: t_us[7:0]]
        P_TH1    = 5'd7,                              // [opcode:const] [operands: theta[15:8]]
        P_TH0    = 5'd8,                              // [opcode:const] [operands: theta[7:0]]
        P_D1     = 5'd9,                              // [opcode:const] [operands: dist[15:8]]
        P_D0     = 5'd10,                             // [opcode:const] [operands: dist[7:0]]
        P_TEMP1  = 5'd11,                             // [opcode:const] [operands: temp_q15[15:8]]
        P_TEMP0  = 5'd12,                             // [opcode:const] [operands: temp_q15[7:0]]
        P_DUTY1  = 5'd13,                             // [opcode:const] [operands: duty_q15[15:8]]
        P_DUTY0  = 5'd14,                             // [opcode:const] [operands: duty_q15[7:0]]
        P_S      = 5'd15,                             // [opcode:const] [operands: status]
        P_C1     = 5'd16,                             // [opcode:const] [operands: CRC[15:8]]
        P_C0     = 5'd17;                             // [opcode:const] [operands: CRC[7:0]]

    reg [4:0]  ps;                                   // [opcode:reg] [operands: present state]
    reg [15:0] crc;                                  // [opcode:reg] [operands: running CRC accumulator]

    always @(posedge clk) begin                      // [opcode:always] [operands:sens=posedge clk]
        tx_vld   <= 1'b0;                            // [opcode:nba] [operands: default deassert valid]
        dbg_t_us <= t_us;                            // [opcode:nba] [operands: expose internal timestamp]

        if (rst) begin                               // [opcode:if] [operands: reset path]
            ps        <= P_IDLE;                     // [opcode:nba] [operands: FSM to IDLE]
            tx_byte   <= 8'h00;                      // [opcode:nba] [operands: clear data bus]
            crc       <= 16'hFFFF;                   // [opcode:nba] [operands: CRC init per CCITT]

            // Optional: clean sim start (avoid X propagation for study)
            t_latch   <= 32'd0;                      // [opcode:nba] [operands: clear snapshot regs]
            th_latch  <= 16'd0;                      // [opcode:nba]
            d_latch   <= 16'd0;                      // [opcode:nba]
            s_latch   <= 8'd0;                       // [opcode:nba]
            temp_latch<= 16'd0;                      // [opcode:nba]
            duty_latch<= 16'd0;                      // [opcode:nba]
        end else begin                               // [opcode:else]
            case (ps)                                // [opcode:case] [operands: switch ps]

                // ---- Wait for a new sample (only state that latches inputs) ----
                P_IDLE: begin                        // [opcode:label] [operands: state=P_IDLE]
                    if (sample_vld) begin            // [opcode:if] [operands: edge-qualified strobe]
                        t_latch    <= t_us;          // [opcode:nba] [operands: snapshot timestamp]
                        th_latch   <= theta16;       // [opcode:nba] [operands: snapshot theta]
                        d_latch    <= dist_mm;       // [opcode:nba] [operands: snapshot distance]
                        s_latch    <= status;        // [opcode:nba] [operands: snapshot status]
                        temp_latch <= temp_q15;      // [opcode:nba] [operands: snapshot temperature]
                        duty_latch <= fan_duty_q15_dbg; // [opcode:nba] [operands: snapshot fan duty]
                        crc        <= 16'hFFFF;      // [opcode:nba] [operands: reset CRC accumulator]
                        ps         <= P_HDR0;        // [opcode:nba] [operands: start frame]
                    end                              // [opcode:end] [operands: if]
                end                                  // [opcode:end] [operands: state block]

                // ---- Headers (excluded from CRC) ----
                P_HDR0: if (tx_rdy) begin            // [opcode:if] [operands: handshake guard]
                    tx_byte <= 8'h55;                // [opcode:nba] [operands: header-0 (byte[0])]
                    tx_vld  <= 1'b1;                 // [opcode:nba] [operands: 1-cycle fire]
                    ps      <= P_HDR1;               // [opcode:nba] [operands: next byte]
                end                                   // [opcode:end]
                P_HDR1: if (tx_rdy) begin
                    tx_byte <= 8'hAA;                // [opcode:nba] [operands: header-1 (byte[1])]
                    tx_vld  <= 1'b1;                 // [opcode:nba]
                    ps      <= P_T3;                 // [opcode:nba] [operands: payload begins at byte[2]]
                end                                   // [opcode:end]

                // ---- Timestamp (big-endian) + CRC updates ----
                P_T3: if (tx_rdy) begin
                    tx_byte <= t_latch[31:24];       // [opcode:nba index] [operands: byte[2]]
                    tx_vld  <= 1'b1;                 // [opcode:nba]
                    crc     <= crc16_ccitt(crc, t_latch[31:24]); // [opcode:nba call]
                    ps      <= P_T2;                 // [opcode:nba]
                end
                P_T2: if (tx_rdy) begin
                    tx_byte <= t_latch[23:16];       // [opcode:nba index] [operands: byte[3]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, t_latch[23:16]);
                    ps      <= P_T1;
                end
                P_T1: if (tx_rdy) begin
                    tx_byte <= t_latch[15:8];        // [opcode:nba index] [operands: byte[4]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, t_latch[15:8]);
                    ps      <= P_T0;
                end
                P_T0: if (tx_rdy) begin
                    tx_byte <= t_latch[7:0];         // [opcode:nba index] [operands: byte[5]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, t_latch[7:0]);
                    ps      <= P_TH1;
                end

                // ---- Theta Q1.15 (big-endian) ----
                P_TH1: if (tx_rdy) begin
                    tx_byte <= th_latch[15:8];       // [opcode:nba index] [operands: byte[6]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, th_latch[15:8]);
                    ps      <= P_TH0;
                end
                P_TH0: if (tx_rdy) begin
                    tx_byte <= th_latch[7:0];        // [opcode:nba index] [operands: byte[7]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, th_latch[7:0]);
                    ps      <= P_D1;
                end

                // ---- Distance (big-endian) ----
                P_D1: if (tx_rdy) begin
                    tx_byte <= d_latch[15:8];        // [opcode:nba index] [operands: byte[8]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, d_latch[15:8]);
                    ps      <= P_D0;
                end
                P_D0: if (tx_rdy) begin
                    tx_byte <= d_latch[7:0];         // [opcode:nba index] [operands: byte[9]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, d_latch[7:0]);
                    ps      <= P_TEMP1;
                end

                // ---- Temperature (big-endian) ----
                P_TEMP1: if (tx_rdy) begin
                    tx_byte <= temp_latch[15:8];     // [opcode:nba index] [operands: byte[10]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, temp_latch[15:8]);
                    ps      <= P_TEMP0;
                end
                P_TEMP0: if (tx_rdy) begin
                    tx_byte <= temp_latch[7:0];      // [opcode:nba index] [operands: byte[11]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, temp_latch[7:0]);
                    ps      <= P_DUTY1;
                end

                // ---- Fan duty (big-endian) ----
                P_DUTY1: if (tx_rdy) begin
                    tx_byte <= duty_latch[15:8];     // [opcode:nba index] [operands: byte[12]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, duty_latch[15:8]);
                    ps      <= P_DUTY0;
                end
                P_DUTY0: if (tx_rdy) begin
                    tx_byte <= duty_latch[7:0];      // [opcode:nba index] [operands: byte[13]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, duty_latch[7:0]);
                    ps      <= P_S;
                end

                // ---- Status (last payload byte fed to CRC) ----
                P_S: if (tx_rdy) begin
                    tx_byte <= s_latch;              // [opcode:nba] [operands: byte[14]]
                    tx_vld  <= 1'b1;
                    crc     <= crc16_ccitt(crc, s_latch); // [opcode:nba call] [operands: final payload into CRC]
                    ps      <= P_C1;
                end

                // ---- CRC16 (big-endian) ----
                P_C1: if (tx_rdy) begin
                    tx_byte <= crc[15:8];            // [opcode:nba index] [operands: byte[15]]
                    tx_vld  <= 1'b1;
                    ps      <= P_C0;
                end
                P_C0: if (tx_rdy) begin
                    tx_byte <= crc[7:0];             // [opcode:nba index] [operands: byte[16]]
                    tx_vld  <= 1'b1;
                    ps      <= P_IDLE;               // [opcode:nba] [operands: frame complete]
                end

                default: ps <= P_IDLE;               // [opcode:default] [operands: defensive reset]
            endcase                                   // [opcode:endcase] [operands:none]
        end                                           // [opcode:end] [operands: else]
    end                                               // [opcode:end] [operands: always]
endmodule                                             // [opcode:endmodule] [operands:none]
