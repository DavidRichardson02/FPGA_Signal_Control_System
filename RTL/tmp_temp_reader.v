`timescale 1ns/1ps
/*==============================================================================
  Module : tmp_temp_reader
  Role   : Minimal, self-contained reader stub for an on-board digital
           temperature sensor (ADT7420-class) on the TMP_SCL/TMP_SDA bus.

  Integration context:
    - This block is instantiated by spatial_mapping_temperature_control_top and
      wired to the on-board TMP I²C pins:

          TMP_SCL ↔ tmp_scl_in/tmp_scl_oen   (open-drain)
          TMP_SDA ↔ tmp_sda_in/tmp_sda_oen   (open-drain)

    - In the *full* design, this module would:
        • Periodically configure the ADT7420 (one-time init).
        • Trigger temperature conversions at MEAS_PERIOD_TICKS cadence.
        • Read the 16-bit temperature register over I²C.
        • Convert the raw °C value into the same Q1.15 “Fahrenheit-like”
          scale used by the XADC path so that:
              72°F → ~16'sd14564
              78°F → ~16'sd16748
          match the fan controller thresholds.

    - In this stub implementation:
        • No real I²C transactions are performed.
        • The TMP bus is always released (tmp_scl_oen = tmp_sda_oen = 1).
        • A synthetic, constant Q1.15 temperature code is emitted
          periodically with a 1-cycle temp_vld pulse.
        • status and busy are held at 0.

  Design goal for now:
    - Eliminate Vivado's DRC INBB-3 ("black box instance") by providing a
      synthesizable implementation with the correct port and parameter
      interface, while keeping the behavior benign.
    - Later, you can replace the internal "fake temperature" logic with a
      proper ADT7420 I²C state machine without changing the top-level ports.

==============================================================================*/
module tmp_temp_reader #(
    // Fabric clock frequency (Hz)
    parameter integer CLK_HZ            = 100_000_000,

    // How often to update temp_q15 (in clk cycles). For example:
    //   MEAS_PERIOD_TICKS = CLK_HZ / 5  → 5 Hz
    //                     = 5_000_000  → 20 Hz at 100 MHz
    parameter integer MEAS_PERIOD_TICKS = 5_000_000,

    // 7-bit I²C address of the digital temperature sensor (ADT7420 default: 0x4B)
    parameter [6:0]   I2C_ADDR7         = 7'h4B,

    // Fixed-point scale factor from ADT7420 °C code to Q1.15 "board temperature"
    //
    // In the *final* implementation, these parameters would be used as:
    //
    //   temp_c16 = raw_temp[15:3];                      // signed, °C * 16
    //   prod     = temp_c16 * SCALE_Q15_PER_C;          // 16×16→32
    //   scaled   = prod >>> 8;                          // divide by 256
    //   temp_q15 = scaled + OFFSET_Q15;
    //
    // For now, they are kept as parameters for future use and for interface
    // consistency; the stub below simply emits a constant temp_q15.
    parameter signed [15:0] SCALE_Q15_PER_C = 16'sd10483,
    parameter signed [15:0] OFFSET_Q15      = -16'sd13
)(
    // Clock / reset (fabric domain)
    input  wire              clk,
    input  wire              rst,

    // I²C open-drain interface to TMP sensor
    input  wire              tmp_scl_in,   // not used in stub
    input  wire              tmp_sda_in,   // not used in stub
    output reg               tmp_scl_oen,  // 1=release, 0=drive 0
    output reg               tmp_sda_oen,  // 1=release, 0=drive 0

    // Canonical temperature output (aligned with XADC Q1.15 scale)
    output reg  signed [15:0] temp_q15,    // Q1.15 temperature code
    output reg                temp_vld,    // 1-cycle strobe per "conversion"

    // Optional status / diagnostics
    output reg  [7:0]         status,      // reserved for error/flag bits
    output reg                busy         // 1=I²C transaction in progress
);

    //==========================================================================
    // 1. Synthetic Q1.15 temperature generator
    //==========================================================================
    // For now, emit a fixed "room" temperature in the same code space as the
    // XADC-based path. A good choice is somewhere between your occupied and
    // unoccupied thresholds (72°F and 78°F). We can, for example, park this at
    // 75°F mid-way, or simply reuse one of the thresholds.
    //
    // Here, we use the occupied setpoint (72°F) code:
    //   T_SET_OCC_Q15 = 16'sd14564
    // so that if you switch temp_src_sel to TEMP_SRC_TMP, the fan controller
    // will see a sensible "room temperature".
    //==========================================================================

    localparam signed [15:0] TEMP_ROOM_Q15 = 16'sd14564;  // ≈72°F in your scale

    // Down-counter for periodic update cadence
    localparam integer CTR_W = (MEAS_PERIOD_TICKS <= 1) ? 1 :
                               $clog2(MEAS_PERIOD_TICKS+1);

    reg [CTR_W-1:0] tick_cnt = {CTR_W{1'b0}};

    //==========================================================================
    // 2. Stub behavior
    //   - Keep I²C bus released (OE=1 → SCL/SDA high-Z from FPGA).
    //   - Never assert busy (no real transactions).
    //   - Emit TEMP_ROOM_Q15 once per MEAS_PERIOD_TICKS with a 1-cycle temp_vld.
    //   - Keep status at 0 for now.
    //==========================================================================

    always @(posedge clk) begin
        if (rst) begin
            // Reset all outputs to benign defaults
            tmp_scl_oen <= 1'b1;          // release bus
            tmp_sda_oen <= 1'b1;          // release bus

            temp_q15    <= 16'sd0;
            temp_vld    <= 1'b0;

            status      <= 8'h00;
            busy        <= 1'b0;

            tick_cnt    <= {CTR_W{1'b0}};
        end else begin
            // Always keep the bus released in the stub
            tmp_scl_oen <= 1'b1;
            tmp_sda_oen <= 1'b1;

            // No I²C activity → not busy
            busy        <= 1'b0;

            // Default: deassert temp_vld, status stays zero
            temp_vld    <= 1'b0;
            status      <= 8'h00;

            // Periodic synthetic update
            if (MEAS_PERIOD_TICKS > 0) begin
                if (tick_cnt == MEAS_PERIOD_TICKS[CTR_W-1:0]) begin
                    tick_cnt <= {CTR_W{1'b0}};

                    // "New sample" event: latch constant temp and pulse temp_vld
                    temp_q15 <= TEMP_ROOM_Q15;
                    temp_vld <= 1'b1;
                end else begin
                    tick_cnt <= tick_cnt + 1'b1;
                end
            end else begin
                // Degenerate case: MEAS_PERIOD_TICKS == 0 → always valid
                temp_q15 <= TEMP_ROOM_Q15;
                temp_vld <= 1'b1;
            end
        end
    end

endmodule
