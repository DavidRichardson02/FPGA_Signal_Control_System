`timescale 1ns/1ps  // Simulation-only: time units in ns, resolution 1 ps.
// Synthesis tools ignore `timescale; simulators obey it for #delays and $time.

/*==============================================================================
  Module : uart_stream_tx
  Role   : 8-N-1 UART byte transmitter with ready/valid handshake.
           Generates a single asynchronous TX line (txo) from a stream of
           bytes coming from mapper_packetizer or any other producer.

  ------------- External interface (system side) --------------------------------
  Clock / reset:
    clk   : Fabric/system clock driving all sequential logic.
    rst   : Synchronous, active-HIGH reset. Puts TX line into idle (logic '1')
            and reports tx_rdy=1 (ready to accept the first byte).

  Stream-in handshake (matches mapper_packetizer’s contract):
    tx_byte[7:0] : Payload byte presented by the upstream mapper.
    tx_vld       : 1-cycle strobe: “tx_byte is valid **this** clock edge”.
    tx_rdy       : Level: 1 => this module can accept a new byte **now**.
                   A byte transfer occurs only on clk cycles where:
                        tx_vld && tx_rdy == 1
                   There is no internal FIFO; if tx_vld is asserted while
                   tx_rdy=0, that byte is ignored.

  UART physical output + status:
    txo   : Asynchronous UART TX line. Idle level is logic '1' (“mark”).
    busy  : High from the moment a byte is accepted until its entire frame
            (START + 8 data bits + STOP) has been shifted out.

  ------------- Framing convention (8-N-1) --------------------------------------
  For each accepted tx_byte:
    - START bit : 0
    - DATA bits : 8 bits, LSB first (tx_byte[0] → first data bit on the line)
    - STOP bit  : 1

  Internally we construct a 10-bit shift register:
      sh = { STOP(1), DATA[7:0], START(0) }
    and transmit the least significant bit (sh[0]) at each bit boundary. After
    each boundary the register is shifted right and a '1' is shifted into the
    MSB, so the line naturally returns to idle after the STOP bit.

  ------------- Baud timing model ----------------------------------------------
  Baud rate: BAUD [bits/second]
  Fabric clk: CLK_HZ [Hz]

  We approximate the UART bit period with an integer divider:
      CLKS_PER_BIT = floor(CLK_HZ / BAUD)

  A local counter 'bc' runs 0 .. CLKS_PER_BIT-1. When bc hits CLKS_PER_BIT-1 we:
    - Reset bc to 0 (start next bit cell),
    - Output the next framed bit (sh[0]) onto txo,
    - Shift sh and advance bit index bitc.

  Example (project default):
    CLK_HZ = 100 MHz
    BAUD   = 2 000 000
    → CLKS_PER_BIT = 50 clk cycles
    → bit time ≈ 50 * 10 ns = 500 ns → BAUD ≈ 2 Mb/s (small integer rounding).

  ------------- Design policies / limitations ----------------------------------
  • Single clock domain (clk). No derived or gated clocks; timing is simple.
  • No buffering: at most one byte “in flight” (in sh). Upstream must track
    tx_rdy to avoid dropping bytes.
  • No parity, no multi-stop, no break detection – strictly 8-N-1 framing.
  • Reset behavior is fully synchronous; there are no asynchronous clears on
    output flops. This avoids reset release hazards.

==============================================================================*/
module uart_stream_tx #(
  // Public parameters: system-level timing knobs.
  parameter integer CLK_HZ = 100_000_000,  // Fabric clock frequency (Hz).
  parameter integer BAUD   = 2_000_000     // UART baud rate (bits per second).
)(
  input  wire       clk,       // System/fabric clock.
  input  wire       rst,       // Synchronous, active-HIGH reset.

  // Ready/valid stream interface (matches mapper_packetizer)
  input  wire [7:0] tx_byte,   // Payload byte from mapper_packetizer.
  input  wire       tx_vld,    // 1-cycle "byte valid" strobe.
  output reg        tx_rdy,    // High when this module can accept a new byte.

  // UART physical output + status
  output reg        txo,       // Serial TX line (idle = 1, i.e., "mark").
  output reg        busy       // High while a full frame is being transmitted.
);

  // ---------------------------------------------------------------------------
  // Baud generator: derive bit period in fabric clocks
  // ---------------------------------------------------------------------------
  // Integer approximation to the ideal CLKS_PER_BIT = CLK_HZ / BAUD.
  // For legal parameter choices, CLKS_PER_BIT >= 1; we guard this at elaboration.
  localparam integer CLKS_PER_BIT = (BAUD <= 0) ? 1 : (CLK_HZ / BAUD);

  // Static parameter sanity check (sim-time only; ignored by synthesis):
  initial begin
    if (CLKS_PER_BIT < 1)
      $error("uart_stream_tx: CLKS_PER_BIT<1 (CLK_HZ=%0d, BAUD=%0d).",
             CLK_HZ, BAUD);
  end

  // Width for the fabric-clock counter that measures one UART bit time:
  //   bc : 0 .. CLKS_PER_BIT-1
  localparam integer CW = (CLKS_PER_BIT <= 1) ? 1 : $clog2(CLKS_PER_BIT);

  // ---------------------------------------------------------------------------
  // Framing resources: counters and shift register
  // ---------------------------------------------------------------------------
  // Total framed bits: 1 start + 8 data + 1 stop = 10.
  localparam integer BIT_TOTAL = 10;

  reg [CW-1:0] bc;    // Bit-time counter: counts fabric clocks inside one bit.
  reg [3:0]    bitc;  // Which bit of the current frame (0..BIT_TOTAL-1).
  reg [9:0]    sh;    // Shift register: {STOP, data[7:0], START}; LSB-first out.

  // ---------------------------------------------------------------------------
  // Main transmitter process
  //
  // Behavioral outline:
  //   • When !busy:
  //       - tx_rdy=1, txo held at idle(1).
  //       - On tx_vld==1, snapshot tx_byte into sh, assert busy, deassert tx_rdy.
  //   • When busy:
  //       - bc counts fabric clocks.
  //       - Each time bc hits CLKS_PER_BIT-1:
  //           txo <= sh[0];      // put current framed bit on the line
  //           sh  <= {1'b1, sh[9:1]};  // shift right, injecting '1' at MSB
  //           bitc <= bitc + 1;
  //         After BIT_TOTAL bits, we terminate the frame and re-enter IDLE.
  //
  // Note: tx_rdy is only asserted in the IDLE region; there is no hidden queue.
  // ---------------------------------------------------------------------------
  always @(posedge clk) begin
    if (rst) begin
      // ---------- Synchronous reset: deterministic, safe idle ----------
      txo    <= 1'b1;                 // Idle UART level (mark = logic '1').
      tx_rdy <= 1'b1;                 // Ready to accept the very first byte.
      busy   <= 1'b0;                 // No active transmission.

      bc     <= {CW{1'b0}};           // Reset bit-time counter.
      bitc   <= 4'd0;                 // Next frame starts at bit index 0.
      sh     <= 10'h3FF;              // All ones → corresponds to idle line.
    end else begin
      if (!busy) begin
        // ========================= IDLE PHASE =========================
        // TX line is held at idle. Upstream is allowed to hand us a byte
        // on any cycle where tx_vld=1 (tx_rdy is forced high here).
        txo    <= 1'b1;
        tx_rdy <= 1'b1;

        // Accept at most one byte per idle period; no buffering.
        if (tx_vld) begin
          // Construct the full 10-bit frame to send:
          //   {STOP(1), data[7:0], START(0)}
          sh     <= {1'b1, tx_byte, 1'b0};

          busy   <= 1'b1;             // From now until STOP has been shifted.
          tx_rdy <= 1'b0;             // Cannot accept another byte mid-frame.

          bitc   <= 4'd0;             // First bit to leave is index 0 (START).
          bc     <= {CW{1'b0}};       // Restart intra-bit timing.
        end

      end else begin
        // ======================= ACTIVE FRAME ========================
        // At this point, sh holds the remaining bits to emit and bitc counts
        // how many have already been shifted out.

        // Subdivide each bit into CLKS_PER_BIT fabric cycles.
        if (bc == CLKS_PER_BIT-1) begin
          // ---- Bit boundary: advance to the next UART bit ----
          bc   <= {CW{1'b0}};         // Wrap intra-bit counter.

          txo  <= sh[0];              // Drive current framed bit on the line.
          sh   <= {1'b1, sh[9:1]};    // Shift right; inject '1' → natural idle.

          bitc <= bitc + 4'd1;        // Track how many bits have been sent.

          // After the STOP bit (bitc == BIT_TOTAL-1), re-enter IDLE.
          if (bitc == BIT_TOTAL-1) begin
            busy   <= 1'b0;           // Frame complete.
            tx_rdy <= 1'b1;           // Ready for the next tx_byte.
            txo    <= 1'b1;           // Explicitly drive idle (mark) level.
          end
        end else begin
          // ---- Still inside current bit cell: just accumulate fabric cycles ----
          bc <= bc + {{(CW-1){1'b0}}, 1'b1};
        end
      end
    end
  end

endmodule

/*==============================================================================
  Implementation notes & integration guidance
  -------------------------------------------
  • With CLK_HZ=100 MHz and BAUD=2 000 000, this core produces a 2 Mb/s UART
    that matches the mapper_packetizer’s default testbench configuration.

  • Upstream (e.g., mapper_packetizer) should:
      - Drive tx_vld high for exactly one cycle with tx_byte stable.
      - Only present a new byte on cycles where tx_rdy is 1.
    Violating this contract will cause silent byte drops; this is intentional
    to keep logic small and deterministic.

  • Downstream:
      - txo can be routed directly to a USB-UART bridge or a pin header.
      - Ensure the external interface expects idle-high 8-N-1 framing.

  • If fractional-baud accuracy is critical, CLKS_PER_BIT can be tuned by
    choosing a more suitable CLK_HZ or BAUD. The integer division approach
    here is sufficient for most FPGA-to-USB serial links in lab environments.
==============================================================================*/
