`timescale 1ns/1ps

// ======================================================================================
// File      : xadc_sampler.v
// Target    : Xilinx 7-Series (e.g., Artix-7 on Nexys A7) with on-chip XADC
// Role      : Single-channel XADC front-end for an external analog signal on VAUX3,
//             producing normalized signed Q1.15 samples plus a 1-cycle valid strobe.
//
// --------------------------------------------------------------------------------------
// High-level theory of operation
// --------------------------------------------------------------------------------------
// • The on-chip XADC is a successive-approximation (SAR) ADC with a programmable
//   sequencer that can step through internal sensors (temperature, VCC rails) and
//   external analog channels (VP/VN, VAUX[15:0]).
//
// • In this design we:
//     - Enable the continuous channel sequencer.
//     - Configure it to convert a small set of channels, including VAUX3.
//     - Use the DRP (Dynamic Reconfiguration Port) to read the "Latest Result" register
//       every time End-Of-Conversion (EOC) asserts.
//     - Filter by CHANNEL to accept only VAUX3 samples and ignore all others.
//
// • Sample formatting:
//     - The XADC returns a 12-bit unsigned code (0..4095) in DO[15:4].
//     - We treat this as UQ1.15 by left-shifting 4 bits (×16) → range ~[0, 1).
//     - We subtract 0x8000 to re-center around 0 → signed Q1.15 in ~[−1.0, +1.0).
//     - The result is presented on sample_q15 with a 1-cycle sample_vld strobe.
//
// --------------------------------------------------------------------------------------
// Board-level / bonding notes (Nexys A7 JXADC)
// --------------------------------------------------------------------------------------
// • The XADC's auxiliary analog inputs VAUX[15:0] are brought out to the JXADC header.
// • In *this* design using the VAUX3 pair wired in the XDC as:
//
//       vauxp_in → XA_P[3] (B16)
//       vauxn_in → XA_N[3] (B17)
//
//   which correspond internally to VAUXP[3] / VAUXN[3] and thus channel CH19.
//
// • XDC constraints:
//     - Use only LOC + IOSTANDARD LVCMOS33 on these analog pads for bookkeeping;
//       the XADC macro defines their real analog behavior. :contentReference[oaicite:1]{index=1}
//
// --------------------------------------------------------------------------------------
// Throughput & latency
// --------------------------------------------------------------------------------------
// • The continuous sequencer free-runs autonomously once configured.
// • Effective per-channel sample rate depends on:
//     - The XADC internal ADC clock (derived from DCLK via a divider).
//     - The number of enabled channels.
//     - Any averaging and extended acquisition settings.
// • This wrapper:
//     - Issues a 1-cycle sample_vld pulse each time a VAUX3 sample is latched.
//     - Provides a steady Q1.15 stream suitable for downstream DSP, monitoring,
//       or PWM “DAC” drive without additional scaling.
// ======================================================================================
module xadc_sampler (
    input  wire        clk,        // (IN) fabric clock for XADC DRP + status (e.g., 100 MHz)
    input  wire        rst,        // (IN) synchronous, active-high reset for this wrapper
    input  wire        vauxp_in,   // (IN) external analog positive input  → VAUXP[3] (JXADC B16)
    input  wire        vauxn_in,   // (IN) external analog negative input  → VAUXN[3] (JXADC B17)
    output reg  [15:0] sample_q15, // (OUT) signed Q1.15 sample; ~−1.0 at 0x8000, ~0.0 at 0x0000
    output reg         sample_vld  // (OUT) 1-cycle strobe: new VAUX3 sample latched this clk
);

    // ----------------------------------------------------------------------------------
    // 1) Build VAUX buses from the single external pair
    // ----------------------------------------------------------------------------------
    // The XADC primitive exposes 16-bit VAUXP/VAUXN buses; bit i corresponds to VAUXi.
    // We only use VAUX3 in this design, so we place vauxp_in/vauxn_in at bit[3] and
    // drive all other bits as 0.
    //
    //   vauxp_bus[3] = vauxp_in;   vauxp_bus[15:4,2:0] = 0
    //   vauxn_bus[3] = vauxn_in;   vauxn_bus[15:4,2:0] = 0
    //
    // If later want a different VAUX channel (e.g., VAUX6), this is the first place
    // to adjust: move the "1" position and update the CHANNEL constant and INIT_49/4B/4F.
    wire [15:0] vauxp_bus = {12'b0, vauxp_in, 3'b000}; // VAUXP[3] = vauxp_in
    wire [15:0] vauxn_bus = {12'b0, vauxn_in, 3'b000}; // VAUXN[3] = vauxn_in

    // CHANNEL bus code for VAUX3 as reported by XADC (CH16..CH31 map to VAUX0..VAUX15).
    //   CH16 → 5'h10 → VAUX0
    //   CH17 → 5'h11 → VAUX1
    //   CH18 → 5'h12 → VAUX2
    //   CH19 → 5'h13 → VAUX3   <-- selected external channel
    localparam [4:0] CH_VAUX3 = 5'h13;

    // ----------------------------------------------------------------------------------
    // 2) DRP / status wires from XADC primitive
    // ----------------------------------------------------------------------------------
    // The Dynamic Reconfiguration Port (DRP) provides read/write access to the XADC
    // register space. Here we only READ the "Latest Result" register (DADDR=0x00) on
    // every conversion via a DEN pulse derived directly from EOC.
    wire [15:0] do_bus;   // (OUT) DRP data bus; carries conversion data when DRDY=1
    wire        drdy;     // (OUT) DRP data-ready 1-cycle strobe following each DEN
    wire        eoc;      // (OUT) End-Of-Conversion pulse for each ADC conversion
    wire        eos;      // (OUT) End-Of-Sequence pulse after finishing enabled channel list
    wire [4:0]  channel;  // (OUT) Encodes which channel produced the latest result

    // ----------------------------------------------------------------------------------
    // 3) XADC primitive configuration
    // ----------------------------------------------------------------------------------
    // Reference: UG480 "7 Series FPGAs and Zynq-7000 SoC XADC" (continuous sequencer
    // example, trimmed and customized). :contentReference[oaicite:2]{index=2}
    //
    // Control / config registers (40h–42h):
    //   40h (CONFIG0) : averaging, DCLK divider, calibration averaging, etc.
    //   41h (CONFIG1) : sequencer mode, startup/calibration, ALM behavior.
    //   42h (CONFIG2) : selects use of channel sequencer register set.
    //
    // Channel sequencer registers (48h–4Fh):
    //   48h,49h : SEQ_CHx   = channel enable bitmaps
    //   4Ah,4Bh : SEQ_AVGx  = per-channel averaging enables
    //   4Ch,4Dh : SEQ_MODE  = external channel bipolar/unipolar selection
    //   4Eh,4Fh : SEQ_ACQx  = extended acquisition time bits (external channels)
    //
    // Policy in this design:
    //   • Enable Temp (CH0), VCCINT (CH1), and VAUX3 (CH19) in the sequencer.
    //   • Enable per-channel averaging for those three channels.
    //   • Keep all external channels UNIPOLAR at the XADC level; do bipolar mapping
    //     in fabric as Q1.15.
    //   • Give VAUX3 extended acquisition time (10 ADCCLK cycles) for better settling
    //     margin when driving from moderate source impedances (e.g., TMP36 + RC). :contentReference[oaicite:3]{index=3}
    XADC #(
        // CONFIG0 (40h): ADC clocking, internal averaging, miscellaneous.
        .INIT_40(16'h3000),

        // CONFIG1 (41h): continuous sequencer mode, calibration enabled.
        .INIT_41(16'h21AF),

        // CONFIG2 (42h): enable use of channel sequencer registers.
        .INIT_42(16'h0400),

        // -------- Channel enable bitmaps (48h,49h) --------
        // 48h SEQ_CH0: internal channels (Temp, VCCINT)
        //   bit8=1 → CH0 (Temp)
        //   bit9=1 → CH1 (VCCINT)
        .INIT_48(16'h0300),  // 0000_0011_0000_0000

        // 49h SEQ_CH1: VAUX channel enables
        //   bit3=1 → CH19 (VAUX3), all other VAUX channels disabled.
        .INIT_49(16'h0008),  // 0000_0000_0000_1000

        // -------- Per-channel averaging (4Ah,4Bh) --------
        // 4Ah SEQ_AVG0: same bit map as 48h (internal)
        .INIT_4A(16'h0300),  // average Temp (bit8) + VCCINT (bit9)

        // 4Bh SEQ_AVG1: VAUX channels. Average VAUX3 only.
        .INIT_4B(16'h0008),

        // -------- External channel mode: unipolar vs bipolar (4Ch,4Dh) --------
        // All external channels UNIPOLAR (0..Vref). Signed mapping is applied
        // later in fabric to produce Q1.15 centered at zero. :contentReference[oaicite:4]{index=4}
        .INIT_4C(16'h0000),
        .INIT_4D(16'h0000),

        // -------- Acquisition time extensions (4Eh,4Fh) --------
        // 0 = 4 ADCCLK cycles settle, 1 = 10 ADCCLK cycles (external channels only).
        // Extend settle for VAUX3 only (bit3=1).
        .INIT_4E(16'h0000),
        .INIT_4F(16'h0008),

        // Optional analog stimulus file for functional sim of the XADC behavioral model.
        .SIM_MONITOR_FILE("design.txt")
    ) U_XADC (
        .DCLK   (clk),          // DRP clock (fabric domain)
        .RESET  (rst),          // sync reset into XADC logic
        .DEN    (eoc),          // assert DEN on every EOC → read "Latest Result"
        .DWE    (1'b0),         // no writes, read-only DRP usage
        .DADDR  (7'h00),        // 0x00 = "Latest Result" status register
        .DI     (16'h0000),     // unused (no writes)
        .DO     (do_bus),       // conversion result / status
        .DRDY   (drdy),         // pulses one DCLK after DEN, when DO is valid

        .EOC    (eoc),          // End-Of-Conversion: looped back to DEN
        .EOS    (eos),          // End-Of-Sequence: available if needed
        .CHANNEL(channel),      // which channel produced the current DO value

        .VAUXP  (vauxp_bus),    // VAUXP[3] carries vauxp_in; others tied low
        .VAUXN  (vauxn_bus),    // VAUXN[3] carries vauxn_in; others tied low
        .VP     (1'b0),         // not using dedicated VP/VN
        .VN     (1'b0),

        .ALM    (),             // alarm outputs unused in this wrapper
        .BUSY   ()              // BUSY unused; could be probed in debug if desired
    );

    // ----------------------------------------------------------------------------------
    // 4) Data path: 12-bit unsigned ADC code → 16-bit signed Q1.15 + valid strobe
    // ----------------------------------------------------------------------------------
    // DO[15:4] = 12-bit unsigned conversion code (0..4095) for the channel named
    // on CHANNEL. DO[3:0] are status bits not used here.
    //
    // Mapping to Q1.15:
    //   1) Treat DO[15:4] as an unsigned 12-bit fixed-point value in [0, 4095].
    //   2) Append 4 zeros (<< 4) to obtain UQ1.15 in [0, 65520/65536) ≈ [0, 1).
    //   3) Subtract 0x8000 (0.5 in UQ1.15) to re-center around 0.0, yielding a signed
    //      Q1.15 value that spans approximately [−1.0, +1.0).
    //
    // Channel gating:
    //   - Because the sequencer also monitors Temp and VCCINT, DRDY can assert for
    //     channels we do NOT want to expose on sample_q15.
    //   - We therefore check (channel == CH_VAUX3) and only latch on those conversions.
    always @(posedge clk) begin
        if (rst) begin
            sample_q15 <= 16'sd0;  // represents ~0.0 in Q1.15
            sample_vld <= 1'b0;
        end else begin
            // Default: de-assert sample_vld. We only pulse it when a VAUX3 sample arrives.
            sample_vld <= 1'b0;

            if (drdy && (channel == CH_VAUX3)) begin
                sample_q15 <= {do_bus[15:4], 4'b0000} - 16'sh8000;
                sample_vld <= 1'b1;
            end
        end
    end

endmodule
