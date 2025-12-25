`timescale 1ns/1ps
/*==============================================================================
  Module : tof_sensor
  Role   : Self-contained Pmod ToF (ISL29501) polling engine with Sample-Start
           (SS) pulse, IRQ wait/timeout handling, and I²C register readback.

  Context:
    - This block sits between:
        • The low-level I²C master (i2c_master) that bit-bangs SCL/SDA with
          ready/valid-style byte commands.
        • The rest of the system, which expects a clean “distance + status +
          valid” interface at a fixed measurement cadence.
    - It encapsulates:
        • One-time ISL29501 configuration over I²C.
        • Periodic Sample-Start (SS) pulses and IRQ wait with timeout.
        • Multi-transaction I²C reads of distance (MSB/LSB) and status.
        • Aggregation of error/status information into a single status byte.

  External Interfaces (fabric-level view)
  ---------------------------------------
    Clock / Reset
      clk  : fabric clock (e.g., 100 MHz). All timing is synchronous to this.
      rst  : synchronous, active-HIGH reset. Returns FSM to initialization
             sequence and clears internal counters/flags.

    I²C (open-drain) interface to shared bus
      scl_in   : sampled SCL pad level (from top-level inout).
      scl_oen  : SCL output-enable (1 => Hi-Z, 0 => drive LOW). Top-level
                 must convert this to actual open-drain behavior.
      sda_in   : sampled SDA pad level (from top-level inout).
      sda_oen  : SDA output-enable (1 => Hi-Z, 0 => drive LOW).

      These four signals typically connect directly to i2c_master, and the
      top-level module maps scl_oen/sda_oen into board pins with open-drain
      semantics (`assign SCL = scl_oen ? 1'bz : 1'b0;` etc.).

    Pmod ToF discrete pins
      tof_irq_n : ISL29501 interrupt output (open-drain, pulled up on Pmod).
      tof_ss    : Sample-Start output from FPGA into ISL29501. Polarity and
                  pulse width are parameterizable.

    Sensor outputs (to rest of system)
      dist_mm    : 16-bit raw distance code read from registers 0xD1/0xD2
                   (MSB/LSB) of the ISL29501.
      status     : 8-bit packed status:
                     [7] irq_timeout   : IRQ wait exceeded IRQ_TIMEOUT_TICKS.
                     [6] i2c_err       : latched OR of i2c_master.err across
                                         the current measurement cycle.
                     [5:3] reserved    : currently 3'b000.
                     [2] vdd_ok        : status register bit (0x02[2]).
                     [1] chip_ready    : status register bit (0x02[1]).
                     [0] enout         : status register bit (0x02[0]).
      sample_vld : 1-cycle strobe asserted when dist_mm and status are
                   simultaneously updated and stable.
      busy       : High whenever FSM is not in S_IDLE (i.e., during init or
                   any measurement/transaction activity).

  Measurement cadence & timing
  ----------------------------
    - MEAS_PERIOD_TICKS specifies the nominal measurement period in clk ticks.
      At 100 MHz and MEAS_PERIOD_TICKS=5_000_000, the system runs at ~20 Hz.
    - For each period:
        1) Emit an SS pulse on tof_ss for SS_PULSE_TICKS cycles.
        2) Wait for IRQ assertion (with polarity parameter IRQ_ACTIVE_LOW),
           or until IRQ_TIMEOUT_TICKS expires (timeout flag set).
        3) Perform three I²C transactions:
             (a) Read Distance MSB (0xD1).
             (b) Read Distance LSB (0xD2).
             (c) Read Status register 0x02.
        4) Pack results into dist_mm and status; pulse sample_vld for one clk.

  Initialization sequence (one-time after reset)
  ---------------------------------------------
    - Before entering the periodic loop, the FSM runs a small write-only table
      into the ISL29501:
          REG 0x10 ← sample_len       (integration period)
          REG 0x11 ← sample_period    (base sample period)
          REG 0x12 ← sample_skip      (sample period range multiplier)
          REG 0x13 ← 0x7D             (single-shot mode + light enabled)
          REG 0x60 ← 0x01             (data-ready interrupt on IRQ pin)
    - These values are chosen to:
        • Set a ~2.27 ms integration time.
        • Achieve an effective ~50 ms sample period (~20 Hz).
        • Enable single-shot mode triggered by SS, with light and calibration
          settings matching datasheet recommendations.
        • Configure IRQ pin for “data ready” events.

  Design / CDC considerations
  ---------------------------
    - Single synchronous clock domain (clk); no derived clocks.
    - i2c_master is controlled via 1-cycle strobes:
        cmd_start, cmd_stop, cmd_write, cmd_read, rd_send_nack, wr_byte
      and responds with busy, done, err, rd_byte, got_ack.
    - The external IRQ pin is:
        • 2-FF synchronized into clk domain.
        • Normalized to an internal active-HIGH irq_level based on
          IRQ_ACTIVE_LOW.
    - Measurement period generator:
        • Uses a 32-bit down-counter to emit fire_meas pulses.
        • Ties period reload to MEAS_PERIOD_TICKS; holds at zero while FSM
          is busy, so measurements don’t overlap.

  Usage in the larger system
  --------------------------
    - Top-level (e.g., spatial_mapping_temperature_control_top) wires:
        • scl_in/sda_in and scl_oen/sda_oen to the board pins (open-drain).
        • tof_irq_n/tof_ss to the Pmod ToF header.
        • dist_mm/status/sample_vld/busy into downstream logic such as
          mapper_packetizer.
    - On each sample_vld=1 cycle, downstream logic can capture dist_mm and
      status exactly once per completed measurement cycle.
==============================================================================*/

module tof_sensor #(
    parameter integer CLK_HZ             = 100_000_000, // fabric clock, Hz

    // Pmod ToF ISL29501 7-bit address (datasheet: 0xAE/0xAF R/W -> 0x57 7b)
    parameter [6:0]   I2C_ADDR7         = 7'h57,

    // Measurement cadence: 50 ms @ 100 MHz → ~20 Hz
    parameter integer MEAS_PERIOD_TICKS = 5_000_000,

    // --- GPIO behavior parameters ---
    parameter         SS_ACTIVE_HIGH    = 1'b1,         // 1 => SS asserted HIGH
    parameter         IRQ_ACTIVE_LOW    = 1'b1,         // 1 => IRQ_n active LOW
    parameter integer SS_PULSE_TICKS    = 500,          // SS pulse width in clk ticks
    parameter integer IRQ_TIMEOUT_TICKS = 1_000_000     // IRQ wait timeout in clk ticks
)(
    input  wire clk,  // fabric clock
    input  wire rst,  // synchronous, active-HIGH reset

    // I2C open-drain bus from/to i2c_master
    input  wire scl_in,
    output wire scl_oen,
    input  wire sda_in,
    output wire sda_oen,

    // Pmod ToF discrete pins
    input  wire tof_irq_n,  // interrupt from ISL29501 (open-drain, pulled up)
    output reg  tof_ss,     // Sample Start (push-pull, polarity-parameterized)

    // Outputs to rest of system
    output reg  [15:0] dist_mm,    // raw 16b distance {MSB,LSB} from 0xD1/0xD2
    output reg  [7:0]  status,     // packed status record (see S_DONE)
    output reg         sample_vld, // 1-cycle strobe when dist_mm/status update
    output reg         busy        // high whenever FSM != S_IDLE
);

    //==========================================================================
    // 1) i2c_master instance (byte-level START/STOP/WRITE/READ engine)
    //--------------------------------------------------------------------------
    // This submodule drives the SCL/SDA waveforms based on a simple command
    // interface:
    //   - cmd_start / cmd_stop   : emit START or STOP condition.
    //   - cmd_write / cmd_read   : write or read one byte.
    //   - rd_send_nack           : NACK the final byte of a read transaction.
    //   - wr_byte                : data to be transmitted on writes.
    //
    // Handshakes:
    //   - busy                   : master is currently shifting a command.
    //   - done                   : previous command finished; rd_byte valid.
    //   - err                    : sticky error (e.g., NACK or watchdog).
    //   - rd_byte                : last byte read from I²C bus.
    //   - got_ack                : ACK/NACK status for write operations.
    //==========================================================================
    reg        cmd_start, cmd_stop, cmd_write, cmd_read, rd_send_nack;
    reg  [7:0] wr_byte;
    wire [7:0] rd_byte;
    wire       got_ack;
    wire       m_busy;
    wire       m_done;
    wire       err;        // sticky error from i2c_master (e.g., NACK, watchdog)

    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(100_000)   // 400 kHz fast-mode I²C
    ) u_i2c (
        .clk(clk),
        .rst(rst),
        .scl_in(scl_in),
        .scl_oen(scl_oen),
        .sda_in(sda_in),
        .sda_oen(sda_oen),
        .cmd_start(cmd_start),
        .cmd_stop(cmd_stop),
        .cmd_write(cmd_write),
        .cmd_read(cmd_read),
        .rd_send_nack(rd_send_nack),
        .wr_byte(wr_byte),
        .rd_byte(rd_byte),
        .got_ack(got_ack),
        .busy(m_busy),
        .done(m_done),
        .err(err)
    );

    //==========================================================================
    // 2) Measurement period generator — one fire_meas per MEAS_PERIOD_TICKS
    //--------------------------------------------------------------------------
    // per_cnt is a 32-bit down-counter that:
    //   - Reloads with MEAS_PERIOD_TICKS_32 when a new cycle begins.
    //   - Decrements to zero while FSM is idle (busy=0).
    //   - Holds at zero while FSM is busy, preventing overlapping sequences.
    //
    // fire_meas is asserted when per_cnt == 0 and FSM is idle; it kicks off
    // the SS/IRQ/I²C transaction sequence.
    //==========================================================================
    localparam [31:0] MEAS_PERIOD_TICKS_32 = MEAS_PERIOD_TICKS[31:0];
    reg  [31:0] per_cnt;
    wire        fire_meas = (per_cnt == 32'd0);

    always @(posedge clk) begin
        if (rst) begin
            per_cnt <= 32'd0; // first cycle after reset fires immediately
        end else if (fire_meas && !busy) begin
            per_cnt <= MEAS_PERIOD_TICKS_32; // reload at start of each cycle
        end else if (per_cnt != 32'd0) begin
            per_cnt <= per_cnt - 32'd1;
        end
        // If per_cnt is 0 and busy=1, we hold at 0 until FSM returns to IDLE.
    end

  
  
    //==========================================================================
    // 3) ISL29501 register constants (subset used in this core)
    //--------------------------------------------------------------------------
    // Control/status
    localparam [7:0] REG_STATUS_ISL    = 8'h02; // enout/ready/vdd_ok

    // Sampling control
    localparam [7:0] REG_INTEG_PERIOD  = 8'h10; // Integration Period
    localparam [7:0] REG_SAMPLE_PERIOD = 8'h11; // Sample Period
    localparam [7:0] REG_SAMPLE_RANGE  = 8'h12; // Sample Period Range (not explicitly written here)
    localparam [7:0] REG_SAMPLE_CTRL   = 8'h13; // Sample Control

    // AGC control (mandatory in Digilent init block)
    localparam [7:0] REG_AGC_CTRL1     = 8'h18; // AGC control 1
    localparam [7:0] REG_AGC_CTRL2     = 8'h19; // AGC control 2

    // Interrupt control
    localparam [7:0] REG_INT_CONTROL   = 8'h60; // Interrupt Control

    // Emitter / driver control (mandatory in Digilent init block)
    localparam [7:0] REG_DRV_RANGE     = 8'h90; // Driver Range
    localparam [7:0] REG_EMIT_DAC      = 8'h91; // Emitter DAC

    // Data output (distance)
    localparam [7:0] REG_HI            = 8'hD1; // Distance Readout MSB
    localparam [7:0] REG_LO            = 8'hD2; // Distance Readout LSB

    // I²C address bytes (7-bit address plus R/W bit)
    wire [7:0] ADDR_W = {I2C_ADDR7, 1'b0}; // addr + W
    wire [7:0] ADDR_R = {I2C_ADDR7, 1'b1}; // addr + R

    //==========================================================================
    // 4) One-time initialization table (write-only)
    //--------------------------------------------------------------------------
    // This is a *line-by-line* mirror of the Digilent Pmod ToF initialization
    // sequence (PmodToF_Initialize / EEPROM restore), cf. Digilent table:
    //
    //   0x10 0x04   Integration Period
    //   0x11 0x6E   Sample Period
    //   0x13 0x71   Sample Control
    //   0x18 0x22   AGC Control 1
    //   0x19 0x22   AGC Control 2
    //   0x60 0x01   Interrupt Control (Data Ready on IRQ_n)
    //   0x90 0x0F   Driver Range
    //   0x91 0xFF   Emitter DAC
    //
    // No extra “helper” writes are added here, so the FPGA config and the
    // Pmod ToF reference design are bit-for-bit identical.
    //--------------------------------------------------------------------------
    localparam [7:0] INTEG_PERIOD_PMOD  = 8'h04; // REG 0x10 ← 0x04
    localparam [7:0] SAMPLE_PERIOD_PMOD = 8'h6E; // REG 0x11 ← 0x6E
    localparam [7:0] SAMPLE_CTRL_PMOD   = 8'h71; // REG 0x13 ← 0x71
    localparam [7:0] AGC_CTRL1_PMOD     = 8'h22; // REG 0x18 ← 0x22
    localparam [7:0] AGC_CTRL2_PMOD     = 8'h22; // REG 0x19 ← 0x22
    localparam [7:0] INT_MODE_PMOD      = 8'h01; // REG 0x60 ← 0x01 (data-ready IRQ)
    localparam [7:0] DRV_RANGE_PMOD     = 8'h0F; // REG 0x90 ← 0x0F
    localparam [7:0] EMIT_DAC_PMOD      = 8'hFF; // REG 0x91 ← 0xFF

    // Number of init entries (0..INIT_COUNT-1)
    localparam integer INIT_COUNT = 8;

    // Register/value table (order matches Digilent table exactly)
    localparam [7:0] INIT_REG0 = REG_INTEG_PERIOD;
    localparam [7:0] INIT_VAL0 = INTEG_PERIOD_PMOD;

    localparam [7:0] INIT_REG1 = REG_SAMPLE_PERIOD;
    localparam [7:0] INIT_VAL1 = SAMPLE_PERIOD_PMOD;

    localparam [7:0] INIT_REG2 = REG_SAMPLE_CTRL;
    localparam [7:0] INIT_VAL2 = SAMPLE_CTRL_PMOD;

    localparam [7:0] INIT_REG3 = REG_AGC_CTRL1;
    localparam [7:0] INIT_VAL3 = AGC_CTRL1_PMOD;

    localparam [7:0] INIT_REG4 = REG_AGC_CTRL2;
    localparam [7:0] INIT_VAL4 = AGC_CTRL2_PMOD;

    localparam [7:0] INIT_REG5 = REG_INT_CONTROL;
    localparam [7:0] INIT_VAL5 = INT_MODE_PMOD;

    localparam [7:0] INIT_REG6 = REG_DRV_RANGE;
    localparam [7:0] INIT_VAL6 = DRV_RANGE_PMOD;

    localparam [7:0] INIT_REG7 = REG_EMIT_DAC;
    localparam [7:0] INIT_VAL7 = EMIT_DAC_PMOD;

    // Index into the init table (0..INIT_COUNT-1)
    reg [2:0] init_idx;
    reg [7:0] init_reg_cur, init_val_cur;

    // Combinational mapping from init_idx → current {reg,val}
    always @* begin
        case (init_idx)
            3'd0: begin init_reg_cur = INIT_REG0; init_val_cur = INIT_VAL0; end // 0x10 ← 0x04
            3'd1: begin init_reg_cur = INIT_REG1; init_val_cur = INIT_VAL1; end // 0x11 ← 0x6E
            3'd2: begin init_reg_cur = INIT_REG2; init_val_cur = INIT_VAL2; end // 0x13 ← 0x71
            3'd3: begin init_reg_cur = INIT_REG3; init_val_cur = INIT_VAL3; end // 0x18 ← 0x22
            3'd4: begin init_reg_cur = INIT_REG4; init_val_cur = INIT_VAL4; end // 0x19 ← 0x22
            3'd5: begin init_reg_cur = INIT_REG5; init_val_cur = INIT_VAL5; end // 0x60 ← 0x01
            3'd6: begin init_reg_cur = INIT_REG6; init_val_cur = INIT_VAL6; end // 0x90 ← 0x0F
            3'd7: begin init_reg_cur = INIT_REG7; init_val_cur = INIT_VAL7; end // 0x91 ← 0xFF
            default: begin init_reg_cur = 8'h00;   init_val_cur = 8'h00;   end
        endcase
    end

  
  
  

 
    //==========================================================================
    // 5) SS polarity + IRQ synchronizer + timeout machinery
    //--------------------------------------------------------------------------
    // SS polarity:
    //   - SS_ASSERT_LEVEL : logical level that asserts SS to the ISL29501.
    //   - SS_IDLE_LEVEL   : the opposite level, used when idle.
    //
    // IRQ path:
    //   - tof_irq_n is asynchronous; we pass it through a 2-FF synchronizer.
    //   - irq_level is an internal, active-HIGH “data ready” flag regardless
    //     of external polarity defined by IRQ_ACTIVE_LOW.
    //
    // Timing:
    //   - ss_cnt   : counts SS_PULSE_TICKS while the SS_ASSERT state holds.
    //   - irq_cnt  : counts down from IRQ_TIMEOUT_TICKS; if it hits zero
    //               before irq_level is asserted, irq_timed_out is set.
    //==========================================================================
    localparam SS_ASSERT_LEVEL = 1'b0;// SS asserted low, falling edge //(SS_ACTIVE_HIGH ? 1'b1 : 1'b0);
    localparam SS_IDLE_LEVEL   = ~SS_ASSERT_LEVEL;

    // Cast timing parameters to 32-bit counters.
    localparam [31:0] SS_PULSE_TICKS_32    = SS_PULSE_TICKS[31:0];
    localparam [31:0] IRQ_TIMEOUT_TICKS_32 = IRQ_TIMEOUT_TICKS[31:0];

    reg [31:0] ss_cnt;        // SS assertion countdown
    reg [31:0] irq_cnt;       // IRQ wait countdown
    reg        irq_timed_out; // set when IRQ wait hits timeout

    // Two-flop synchronizer for external IRQ pin, then normalized to active-high.
    reg irq_meta, irq_sync;
    always @(posedge clk) begin
        if (rst) begin
            irq_meta <= (IRQ_ACTIVE_LOW ? 1'b1 : 1'b0);
            irq_sync <= (IRQ_ACTIVE_LOW ? 1'b1 : 1'b0);
        end else begin
            irq_meta <= tof_irq_n;
            irq_sync <= irq_meta;
        end
    end

    // Canonical "data-ready" level: 1 means IRQ asserted, 0 means idle.
    wire irq_level = IRQ_ACTIVE_LOW ? ~irq_sync : irq_sync;

    //==========================================================================
    // 6) FSM state encoding
    //--------------------------------------------------------------------------
    // States cover:
    //   - One-time init writes (S_INIT_*).
    //   - Periodic measurement loop:
    //       SS pulse, IRQ wait, distance MSB/LSB reads, status read, publish.
    //==========================================================================
    localparam [5:0]
        // One-time init sequence
        S_INIT_START     = 6'd0,
        S_INIT_ADDRW     = 6'd1,
        S_INIT_ADDRW_ACK = 6'd2,
        S_INIT_REG       = 6'd3,
        S_INIT_REG_ACK   = 6'd4,
        S_INIT_DATA      = 6'd5,
        S_INIT_DATA_ACK  = 6'd6,
        S_INIT_STOP      = 6'd7,
        S_INIT_NEXT      = 6'd8,

        // Periodic measurement loop
        S_IDLE           = 6'd9,
        S_FIRE           = 6'd10,
        S_SS_ASSERT      = 6'd11,
        S_SS_WAITIRQ     = 6'd12,
        S_SS_FBACK       = 6'd13,

        // Read high byte sequence
        S_ST1            = 6'd14,
        S_ADDRW1         = 6'd15,
        S_ADDRW1_ACK     = 6'd16,
        S_REGPTR         = 6'd17,
        S_REGPTR_ACK     = 6'd18,
        S_REPSTART       = 6'd19,
        S_ADDRR1         = 6'd20,
        S_ADDRR1_ACK     = 6'd21,
        S_READ1          = 6'd22,
        S_RACK1          = 6'd23,
        S_STOP1          = 6'd24,

        // Read low byte sequence
        S_ST2            = 6'd25,
        S_ADDRW2         = 6'd26,
        S_ADDRW2_ACK     = 6'd27,
        S_REGPTR2        = 6'd28,
        S_REGPTR2_ACK    = 6'd29,
        S_REPSTART2      = 6'd30,
        S_ADDRR2         = 6'd31,
        S_ADDRR2_ACK     = 6'd32,
        S_READ2          = 6'd33,
        S_RNACK2         = 6'd34,
        S_STOP2          = 6'd35,

        // Read status register sequence
        S_STS_W          = 6'd36,
        S_STS_WA         = 6'd37,
        S_STS_PTR        = 6'd38,
        S_STS_PTRA       = 6'd39,
        S_STS_RS         = 6'd40,
        S_STS_RA         = 6'd41,
        S_STS_RD         = 6'd42,
        S_STS_RN         = 6'd43,

        // Publish result
        S_DONE           = 6'd44,
        S_STS_WACK       = 6'd45,  // ACK after writing ADDR_W
        S_STS_PTR_ACK    = 6'd46;  // ACK after writing REG_STATUS_ISL

    reg [5:0] st;
    reg [7:0] rb_hi, rb_lo, rb_sts; // readback latches for distance + status

    //==========================================================================
    // 7) Aggregated I²C error latch
    //--------------------------------------------------------------------------
    // i2c_err_latched is cleared at the beginning of each new measurement
    // cycle (when we leave IDLE due to fire_meas) and ORs in any error
    // indication from i2c_master (err) during the sequence. It is reported
    // in status[6].
    //==========================================================================
    reg i2c_err_latched;

    always @(posedge clk) begin
        if (rst) begin
            i2c_err_latched <= 1'b0;
        end else begin
            if (st == S_IDLE && fire_meas)
                i2c_err_latched <= 1'b0;
            else if (err)
                i2c_err_latched <= 1'b1;
        end
    end

    //==========================================================================
    // 8) Main control FSM — SS/IRQ + I²C sequencing
    //--------------------------------------------------------------------------
    // All control outputs and status flags are driven from this single
    // sequential process:
    //   - Command strobes into i2c_master.
    //   - Tof SS pulse and IRQ timeout.
    //   - Init table stepping and measurement loop.
    //   - Final packing of dist_mm and status with sample_vld pulse.
    //==========================================================================
    always @(posedge clk) begin
        // ---- Default strobes and outputs (reset every cycle) ----
        cmd_start    <= 1'b0;
        cmd_stop     <= 1'b0;
        cmd_write    <= 1'b0;
        cmd_read     <= 1'b0;
        rd_send_nack <= 1'b0;
        wr_byte      <= 8'h00;
        sample_vld   <= 1'b0;
        busy         <= (st != S_IDLE);

        // Default SS line to idle level unless a state asserts it.
        tof_ss       <= SS_IDLE_LEVEL;

        if (rst) begin
            // Global reset state
            st            <= S_INIT_START;
            dist_mm       <= 16'd0;
            status        <= 8'd0;
            rb_hi         <= 8'h00;
            rb_lo         <= 8'h00;
            rb_sts        <= 8'h00;
            ss_cnt        <= 32'd0;
            irq_cnt       <= 32'd0;
            irq_timed_out <= 1'b0;
            init_idx      <= 3'd0;
        end else begin
            case (st)

                //==============================================================
                // One-time initialization writes
                //==============================================================
                S_INIT_START: begin
                    if (init_idx == INIT_COUNT[2:0]) begin
                        // All init entries processed; enter normal IDLE loop.
                        st <= S_IDLE;
                    end else begin
                        cmd_start <= 1'b1;       // START condition
                        st        <= S_INIT_ADDRW;
                    end
                end

                S_INIT_ADDRW: begin
                    if (!m_busy) begin
                        wr_byte   <= ADDR_W;
                        cmd_write <= 1'b1;
                        st        <= S_INIT_ADDRW_ACK;
                    end
                end

                S_INIT_ADDRW_ACK: begin
                    if (m_done)
                        st <= S_INIT_REG;
                end

                S_INIT_REG: begin
                    wr_byte   <= init_reg_cur;
                    cmd_write <= 1'b1;
                    st        <= S_INIT_REG_ACK;
                end

                S_INIT_REG_ACK: begin
                    if (m_done)
                        st <= S_INIT_DATA;
                end

                S_INIT_DATA: begin
                    wr_byte   <= init_val_cur;
                    cmd_write <= 1'b1;
                    st        <= S_INIT_DATA_ACK;
                end

                S_INIT_DATA_ACK: begin
                    if (m_done)
                        st <= S_INIT_STOP;
                end

                S_INIT_STOP: begin
                    cmd_stop <= 1'b1;  // STOP condition, release bus
                    st       <= S_INIT_NEXT;
                end

                S_INIT_NEXT: begin
                    init_idx <= init_idx + 3'd1;
                    st       <= S_INIT_START;
                end

                //==============================================================
                // Periodic measurement loop
                //==============================================================
                S_IDLE: begin
                    irq_timed_out <= 1'b0;  // clear timeout flag each cycle
                    if (fire_meas)
                        st <= S_FIRE;
                end

                S_FIRE: begin
                    // Initialize SS pulse and IRQ timeout counters.
                    ss_cnt  <= SS_PULSE_TICKS_32;
                    irq_cnt <= IRQ_TIMEOUT_TICKS_32;
                    st      <= S_SS_ASSERT;
                end

                //---- SS pulse generator ----
                S_SS_ASSERT: begin
                    // Drive SS asserted for SS_PULSE_TICKS cycles.
                    tof_ss <= SS_ASSERT_LEVEL;
                    if (ss_cnt != 32'd0) begin
                        ss_cnt <= ss_cnt - 32'd1;
                    end else begin
                        // Pulse complete; SS falls back to idle via default.
                        st <= S_SS_WAITIRQ;
                    end
                end

                //---- IRQ level-sensitive wait with timeout ----
                S_SS_WAITIRQ: begin
                    if (irq_level) begin
                        // IRQ asserted: proceed to distance read sequence.
                        st <= S_ST1;
                    end else if (irq_cnt != 32'd0) begin
                        irq_cnt <= irq_cnt - 32'd1;
                    end else begin
                        // Timeout: never saw IRQ asserted in the window.
                        irq_timed_out <= 1'b1;
                        st            <= S_SS_FBACK;
                    end
                end

                S_SS_FBACK: begin
                    // Simple fallback: proceed to measurement read anyway.
                    st <= S_ST1;
                end

                //==============================================================
                // Transaction 1: Read Distance MSB (REG_HI)
                //==============================================================
                S_ST1: begin
                    cmd_start <= 1'b1;
                    st        <= S_ADDRW1;
                end

                S_ADDRW1: begin
                    if (!m_busy) begin
                        wr_byte   <= ADDR_W;
                        cmd_write <= 1'b1;
                        st        <= S_ADDRW1_ACK;
                    end
                end

                S_ADDRW1_ACK: begin
                    if (m_done)
                        st <= S_REGPTR;
                end

                S_REGPTR: begin
                    wr_byte   <= REG_HI;
                    cmd_write <= 1'b1;
                    st        <= S_REGPTR_ACK;
                end

                S_REGPTR_ACK: begin
                    if (m_done)
                        st <= S_REPSTART;
                end

                S_REPSTART: begin
                    cmd_start <= 1'b1;   // repeated START, no STOP in between
                    st        <= S_ADDRR1;
                end

                S_ADDRR1: begin
                    if (!m_busy) begin
                        wr_byte   <= ADDR_R;
                        cmd_write <= 1'b1;
                        st        <= S_ADDRR1_ACK;
                    end
                end

                S_ADDRR1_ACK: begin
                    if (m_done)
                        st <= S_READ1;
                end

                S_READ1: begin
                    cmd_read     <= 1'b1;
                    rd_send_nack <= 1'b0; // ACK this byte
                    st           <= S_RACK1;
                end

                S_RACK1: begin
                    if (m_done) begin
                        rb_hi <= rd_byte;
                        st    <= S_STOP1;
                    end
                end

                S_STOP1: begin
                    cmd_stop <= 1'b1;
                    st       <= S_ST2;
                end

                //==============================================================
                // Transaction 2: Read Distance LSB (REG_LO)
                //==============================================================
                S_ST2: begin
                    cmd_start <= 1'b1;
                    st        <= S_ADDRW2;
                end

                S_ADDRW2: begin
                    if (!m_busy) begin
                        wr_byte   <= ADDR_W;
                        cmd_write <= 1'b1;
                        st        <= S_ADDRW2_ACK;
                    end
                end

                S_ADDRW2_ACK: begin
                    if (m_done)
                        st <= S_REGPTR2;
                end

                S_REGPTR2: begin
                    wr_byte   <= REG_LO;
                    cmd_write <= 1'b1;
                    st        <= S_REGPTR2_ACK;
                end

                S_REGPTR2_ACK: begin
                    if (m_done)
                        st <= S_REPSTART2;
                end

                S_REPSTART2: begin
                    cmd_start <= 1'b1;
                    st        <= S_ADDRR2;
                end

                S_ADDRR2: begin
                    if (!m_busy) begin
                        wr_byte   <= ADDR_R;
                        cmd_write <= 1'b1;
                        st        <= S_ADDRR2_ACK;
                    end
                end

                S_ADDRR2_ACK: begin
                    if (m_done)
                        st <= S_READ2;
                end

                S_READ2: begin
                    cmd_read     <= 1'b1;
                    rd_send_nack <= 1'b1; // NACK final distance byte
                    st           <= S_RNACK2;
                end

                S_RNACK2: begin
                    if (m_done) begin
                        rb_lo <= rd_byte;
                        st    <= S_STOP2;
                    end
                end

                S_STOP2: begin
                    cmd_stop <= 1'b1;
                    st       <= S_STS_W;
                end

                //==============================================================
                // Transaction 3: Read Status Register (0x02)
                //==============================================================
                //==============================================================
                      // Transaction 3: Read Status Register (0x02)
                      //   Pattern is exactly like the distance reads:
                      //     START → ADDR_W → REG_PTR → repSTART → ADDR_R → READ+NACK → STOP
                      //==============================================================
                      S_STS_W: begin
                          // START for status write phase
                          cmd_start <= 1'b1;
                          st        <= S_STS_WA;
                      end
      
                      // Send ADDR_W (write)
                      S_STS_WA: begin
                          if (!m_busy) begin
                              wr_byte   <= ADDR_W;
                              cmd_write <= 1'b1;
                              st        <= S_STS_WACK;
                          end
                      end
      
                      // Wait for ADDR_W write to finish
                      S_STS_WACK: begin
                          if (m_done)
                              st <= S_STS_PTR;
                      end
      
                      // Send register pointer = REG_STATUS_ISL
                      S_STS_PTR: begin
                          if (!m_busy) begin
                              wr_byte   <= REG_STATUS_ISL;
                              cmd_write <= 1'b1;
                              st        <= S_STS_PTR_ACK;
                          end
                      end
      
                      // Wait for REG_STATUS_ISL write to finish
                      S_STS_PTR_ACK: begin
                          if (m_done) begin
                              // Issue repeated START for read phase
                              cmd_start <= 1'b1;
                              st        <= S_STS_RS;
                          end
                      end
      
                      // After repeated START, send ADDR_R (read)
                      S_STS_RS: begin
                          if (!m_busy) begin
                              wr_byte   <= ADDR_R;
                              cmd_write <= 1'b1;
                              st        <= S_STS_RA;
                          end
                      end
      
                      // When ADDR_R write completes, trigger single-byte READ with NACK
                      S_STS_RA: begin
                          if (m_done) begin
                              cmd_read     <= 1'b1;
                              rd_send_nack <= 1'b1;  // single-byte read → NACK
                              st           <= S_STS_RD;
                          end
                      end
      
                      // Capture status byte once read completes
                      S_STS_RD: begin
                          if (m_done) begin
                              rb_sts <= rd_byte; // raw status bits from 0x02
                              st     <= S_STS_RN;
                          end
                      end
      
                      // Final STOP for the status transaction
                      S_STS_RN: begin
                          cmd_stop <= 1'b1;
                          st       <= S_DONE;
                      end

                //==============================================================
                // DONE — publish results and pulse sample_vld
                //==============================================================
                S_DONE: begin
                    dist_mm <= {rb_hi, rb_lo};

                    status <= {
                        irq_timed_out,          // [7] IRQ wait timeout
                        i2c_err_latched,        // [6] aggregated I²C error
                        3'b000,                 // [5:3] reserved
                        rb_sts[2],              // [2] vdd_ok
                        rb_sts[1],              // [1] chip_ready
                        rb_sts[0]               // [0] enout
                    };

                    sample_vld <= 1'b1; // 1-cycle strobe
                    st         <= S_IDLE;
                end

                default: begin
                    st <= S_IDLE;
                end
            endcase
        end
    end

endmodule
