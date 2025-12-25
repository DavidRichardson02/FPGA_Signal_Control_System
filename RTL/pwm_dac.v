`timescale 1ns/1ps

/*==============================================================================
  File    : pwm_dac.v
  Module  : pwm_dac
  Project : FPGA_Dynamic_Spatial_Mapping_Temperature_Control_Project

  Role in System
  ------------------------------------------------------------------------------
  Generic, parameterizable PWM (pulse-width modulation) core used in this
  project as:

    • A 12-bit “PWM DAC” (pwm_out) in spatial_mapping_temperature_control_top:
        - CTR_W = 12, Fclk = 100 MHz → Fpwm ≈ 24.414 kHz
        - Low-pass filtered by an external RC to approximate an analog voltage
          corresponding to a Q1.15 sensor value.
    • A general-purpose duty controller for loads such as:
        - Fan drive (0% or 100% duty) via a FET on JA3.
        - Potential LED dimming or other duty-based actuation paths.

  Conceptual Behavior
  ------------------------------------------------------------------------------
  • The module implements a classic “counter compare” PWM:
      - An N-bit counter (N = CTR_W) free-runs from 0 to 2^N−1.
      - The output pwm is HIGH while ctr < duty and LOW otherwise.

        For each 2^N-cycle frame:
          HIGH clocks per frame  = duty ∈ [0, 2^N−1]
          Duty cycle (fraction)  = duty / 2^N
          PWM carrier frequency  = Fclk / 2^N

  • If the output is passed through an RC filter with cutoff fc ≪ Fpwm, the
    average output voltage approximates:
          Vout ≈ VDD * (duty / 2^N)
    with ripple determined by filter attenuation at Fpwm and its harmonics.

  Interface Summary
  ------------------------------------------------------------------------------
    Parameters:
      CTR_W      : Counter width (bits) and duty resolution.
                   PWM frame length = 2^CTR_W clocks.

    Ports:
      clk        : Fabric clock driving the counter and comparator.
      rst        : Synchronous, active-HIGH reset. Forces deterministic startup
                   (ctr=0, pwm=0) on assertion.
      duty       : Unsigned N-bit (CTR_W) duty command in [0, 2^N−1].
      pwm        : Active-HIGH PWM waveform. Intended to drive:
                     - FET gate / LED / digital load directly, or
                     - RC filter node for DAC behavior.

  Sizing & Example for This Project
  ------------------------------------------------------------------------------
    • In spatial_mapping_temperature_control_top, the instantiation is:
          pwm_dac #(.CTR_W(12)) u_pwm(...)
      with clk = 100 MHz. Then:

        Fpwm = 100e6 / 2^12 ≈ 24.414 kHz

      This is:
        - Above most audio band → acceptable for fan drive and LED use.
        - High enough that a modest RC (e.g., 6.8 kΩ · 100 nF) yields a
          reasonably smooth analog approximation for visualization.

  Timing / CDC Considerations
  ------------------------------------------------------------------------------
    • All logic is synchronous to 'clk'; no derived clocks are generated.
    • 'duty' must be stable in this clock domain; if it originates elsewhere,
      synchronize or register it before driving pwm_dac.
    • The comparator path (ctr < duty) is purely combinational; for very high
      Fclk and/or large CTR_W, adding a local register for 'duty' may help
      timing closure.

==============================================================================*/
module pwm_dac #(
    // (PUBLIC) PWM resolution in bits; frame = 2^CTR_W clocks.
    // Example: CTR_W=12 at 100 MHz → Fpwm ≈ 24.4 kHz.
    parameter CTR_W = 12
)(
    input  wire              clk,   // (IN)  system/fabric clock
    input  wire              rst,   // (IN)  synchronous, active-HIGH reset
    input  wire [CTR_W-1:0]  duty,  // (IN)  desired HIGH time per frame; 0..2^CTR_W−1
    output reg               pwm    // (OUT) PWM waveform (active-HIGH)
);

    //==========================================================================
    // 1) Free-running phase counter (modulo 2^CTR_W)
    //==========================================================================
    // The counter represents the “phase” within a PWM frame. Because 'ctr' is
    // declared CTR_W bits wide, incrementing by 1 each cycle naturally wraps at
    // 2^CTR_W via modular arithmetic. This wrap defines the frame boundary.
    //   • ctr = 0           : start of frame
    //   • ctr = 2^CTR_W−1   : last cycle of frame
    //   • Next clock wraps back to 0
    //==========================================================================
    reg [CTR_W-1:0] ctr;    // modulo-2^CTR_W phase counter

    //==========================================================================
    // 2) Sequential process: counter advance + duty comparator
    //==========================================================================
    // A single always block governs both the counter and the PWM output to keep
    // their behavior tightly aligned:
    //
    //   On reset:
    //     - ctr ← 0: start in a known phase position.
    //     - pwm ← 0: output LOW; avoids spurious HIGH pulses at release.
    //
    //   On each rising edge of clk (normal operation):
    //     - ctr  ← ctr + 1 (wraps at 2^CTR_W).
    //     - pwm  ← (ctr < duty).
    //
    // Duty semantics:
    //   • For ctr ∈ [0, duty−1], comparator is TRUE  → pwm = 1 (HIGH window).
    //   • For ctr ∈ [duty, 2^CTR_W−1], comparator is FALSE → pwm = 0.
    //
    // Thus, each frame contains exactly 'duty' HIGH cycles out of 2^CTR_W total.
    // Updating 'duty' mid-frame modifies the remaining portion of that frame;
    // for strictly “per-frame” updates, see the shadow-register variant below.
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            // Deterministic reset state:
            ctr <= {CTR_W{1'b0}};  // Counter reset to 0
            pwm <= 1'b0;           // Output LOW
        end else begin
            // Normal operation: increment phase and evaluate PWM comparator
            ctr <= ctr + 1'b1;     // Implicit modulo-2^CTR_W wrap
            pwm <= (ctr < duty);   // HIGH while current phase < requested duty
        end
    end

endmodule

/*==============================================================================
  Optional variants / refinements (reference only; not used in this project)
==============================================================================*/

//------------------------------------------------------------------------------
// 1) Full-scale 100% duty special-case
//------------------------------------------------------------------------------
// Some applications require a true DC HIGH when duty is at maximum (all ones).
// The standard comparator form gives a maximum duty of (2^N−1)/2^N (one LOW
// count each frame). The following modification preserves linearity for
// 0..(2^N−2) and forces 100% HIGH when duty==2^N−1.
//
// always @(posedge clk) begin
//   if (rst) begin
//     ctr <= 0;
//     pwm <= 1'b0;
//   end else begin
//     ctr <= ctr + 1'b1;
//     if (&duty) begin
//       // &duty == 1 when all bits are '1' → full scale
//       pwm <= 1'b1;                 // 100% HIGH
//     end else begin
//       pwm <= (ctr < duty);         // normal comparator region
//     end
//   end
// end

//------------------------------------------------------------------------------
// 2) Glitch-free per-frame updates via shadow register
//------------------------------------------------------------------------------
// If changes to 'duty' should only take effect at frame boundaries, latch the
// input into a shadow register when ctr indicates end-of-frame, and compare
// against that stable copy:
//
// reg [CTR_W-1:0] duty_q;
// always @(posedge clk) begin
//   if (rst) begin
//     ctr    <= 0;
//     duty_q <= 0;
//     pwm    <= 1'b0;
//   end else begin
//     // Advance phase
//     ctr <= ctr + 1'b1;
//
//     // At rollover (previous ctr was max), capture new duty command
//     if (ctr == {CTR_W{1'b1}}) begin
//       duty_q <= duty;
//     end
//
//     // Compare against frame-synchronous copy
//     pwm <= (ctr < duty_q);
//   end
// end

//------------------------------------------------------------------------------
// 3) Signed offset-binary input (for bipolar DAC behavior)
//------------------------------------------------------------------------------
// To interpret a signed N-bit input x ∈ [−2^(N−1), 2^(N−1)−1] and map to duty
// such that the average after RC filtering represents a bipolar analog level:
//
//   duty = x + 2^(N−1);  // offset-binary mapping
//
// This sets:
//   x = −2^(N−1)   → duty = 0     → Vout ≈ 0
//   x = 0          → duty = 2^(N−1) → Vout ≈ VDD/2
//   x = 2^(N−1)−1 → duty ≈ 2^N−1 → Vout ≈ VDD
//
// A Q1.(N−1) representation can then be used upstream for fixed-point math.
//==============================================================================```
