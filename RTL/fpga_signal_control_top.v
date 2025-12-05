`timescale 1 ps / 1 ps
`default_nettype none
// ============================================================================
// Top-level : fpga_signal_control_top
// Project   : FPGA_Signal_Control_System
//
// Role:
//   Board-level wrapper for the monolithic RTL core
//   "spatial_mapping_temperature_control_top".
// 
//   Exposes all major interfaces of the project, including:
//     - Clocks
//     - Buttons & switches
//     - Rotary encoder (survey spinner)
//     - PIR & ToF IRQ
//     - I²C to Pmod ToF (SCL/SDA)
//     - On-board TMP I²C + interrupts (TMP_*)
//     - LEDs (status / debug, packed into led[15:0])
//     - VGA (range plot + telemetry panel)
//     - Fan PWM
//     - Seven-segment display
//     - XADC aux analog inputs
//
// Notes:
//   - This module is user-owned and should be set as the Vivado top module.
//   - I²C pins are exposed as inout and constrained as open-drain with pullups
//     in the XDC (PULLUP TRUE, SLEW SLOW).
// ============================================================================

module fpga_signal_control_top (
    // --------------------------
    // Clocks
    // --------------------------
    input  wire        clk_100MHz,        // Primary board clock

    // --------------------------
    // Pushbuttons
    // --------------------------
    input  wire        btnc,              // Center button
    input  wire        btnd,              // Down button
    input  wire        btn_survey_start,  // Start survey
    input  wire        btn_survey_stop,   // Stop survey

    // --------------------------
    // Rotary encoder (JC)
    // --------------------------
    input  wire        jc_rot_a,      // Rotary A
    input  wire        jc_rot_b,      // Rotary B
    input  wire        jc_rot_sw,     // Rotary push switch

    // --------------------------
    // PIR motion sensor
    // --------------------------
    input  wire        pir_raw,    // PIR logic-level output

    // --------------------------
    // Mode/config switches
    // --------------------------
    input  wire        sw_dbg_mode,
    input  wire        sw_manual_en,
    input  wire        sw_pir_en,
    input  wire        sw_temp_en,
    input wire         sw_survey_manual,
    // --------------------------
    // ToF sensor interrupt (Pmod ToF IRQ)
    // --------------------------
    input  wire        tof_irq_n,   // Active-low ToF IRQ

    // --------------------------
    // I²C to Pmod ToF (open-drain, with board pullups from XDC)
    // --------------------------
    inout  wire        SCL,         // jb[3]
    inout  wire        SDA,         // jb[4]

    // --------------------------
    // On-board TMP I²C + interrupts
    // --------------------------
    inout  wire        TMP_SCL,
    inout  wire        TMP_SDA,
    input  wire        TMP_INT,
    input  wire        TMP_CT,

    // --------------------------
    // UART telemetry
    // --------------------------
    output wire        uart_txo,

    // --------------------------
    // LEDs (Nexys A7 user LEDs)
    // --------------------------
    /// LEDs for data stream visualization (UART byte-level activity)
    output wire        led0,
    output wire        led1,
    output wire        led2,
    output wire        led3,
    output wire        led4,
    output wire        led5,
    output wire        led6,
    output wire        led7,

    /// LEDs for fan + PIR output debugging
    output wire        led9,
    output wire        led10,
    output wire        led11, 
    output wire        led12,
    output wire        led13,
    output wire        led14,
    output wire        led15,
    
    
    // --- RGB LED 0 on Nexys A7 (LD16) ---
    output wire led0_r,
    output wire led0_g,
    output wire led0_b,
    
    
    // --------------------------
    // VGA 640x480 @ 60 Hz
    // --------------------------
    output wire        vga_hsync_n,   // Active-HIGH to connector (after invert)
    output wire        vga_vsync_n,   // Active-HIGH to connector (after invert)
    output wire [3:0]  vga_red,
    output wire [3:0]  vga_grn,
    output wire [3:0]  vga_blu,

    // --------------------------
    // Fan PWM
    // --------------------------
    output wire        fan_pwm_ja3,

    // --------------------------
    // Seven-segment display
    // --------------------------
    output wire [7:0]  anode,       // Anodes (active-low)
    output wire [6:0]  segment,     // Segments (active-low)
    output wire        dp,          // Decimal point (active-low)

    // --------------------------
    // XADC auxiliary inputs
    // --------------------------
    input  wire        vauxp_in,
    input  wire        vauxn_in
);

    // ------------------------------------------------------------------------
    // Internal nets to connect core → top-level buses and unused outputs.
    // ------------------------------------------------------------------------

    // Core LED scalar outputs
    wire led0_int;
    wire led1_int;
    wire led2_int;
    wire led3_int;
    wire led4_int;
    wire led5_int;
    wire led6_int;
    wire led7_int;

    wire led9_int;
    wire led10_int;
    wire led11_int;
    wire led12_int;
    wire led13_int;
    wire led14_int;
    wire led15_int;

    // RGB LED 
    wire led0_r_int;
    wire led0_g_int;
    wire led0_b_int;

    // UART debug and PWM DAC (internal for now)
    wire uart_txo_debug_int;
    wire pwm_out_int;

    // Fan enable (JA4) not exported as top-level yet
    wire fan_en_ja4_int;

    // ToF sample-start line (if you want to drive a Pmod SS pin later)
    wire tof_ss_int;

    // VGA sync from core (active-LOW in your original core)
    wire vga_hsync_n_int;
    wire vga_vsync_n_int;


    // ------------------------------------------------------------------------
    // Map core LED scalars to top-level LED ports
    // ------------------------------------------------------------------------
    assign led0  = led0_int;
    assign led1  = led1_int;
    assign led2  = led2_int;
    assign led3  = led3_int;
    assign led4  = led4_int;
    assign led5  = led5_int;
    assign led6  = led6_int;
    assign led7  = led7_int;

    assign led9  = led9_int;
    assign led10 = led10_int;
    assign led11 = led11_int;
    assign led12 = led12_int;
    assign led13 = led13_int;
    assign led14 = led14_int;
    assign led15 = led15_int;


    assign led0_r = led0_r_int;
    assign led0_g = led0_g_int;
    assign led0_b = led0_b_int;
    
    // ------------------------------------------------------------------------
    // VGA sync polarity adaptation (core uses *_n, board expects active-HIGH)
    // ------------------------------------------------------------------------
    assign vga_hsync_n = ~vga_hsync_n_int;
    assign vga_vsync_n = ~vga_vsync_n_int;

    // ------------------------------------------------------------------------
    // Core Instance: spatial_mapping_temperature_control_top
    // ------------------------------------------------------------------------
    spatial_mapping_temperature_control_top #(
        // No parameters yet; placeholder for future tunables
    ) u_spatial_mapping_temperature_control_top (
        //====================== Board I/O ====================================
        .clk_100MHz       (clk_100MHz),

        .btnc             (btnc),
        .btn_survey_start (btn_survey_start),
        .btn_survey_stop  (btn_survey_stop),
        .btnd             (btnd),

        .sw_temp_en       (sw_temp_en),
        .sw_manual_en     (sw_manual_en),
        .sw_pir_en        (sw_pir_en),
        .sw_dbg_mode      (sw_dbg_mode),
        .sw_survey_manual (sw_survey_manual),

        // I²C to Pmod ToF
        .SCL              (SCL),
        .SDA              (SDA),

        .tof_irq_n        (tof_irq_n),
        .tof_ss           (tof_ss_int),

        .vauxp_in         (vauxp_in),
        .vauxn_in         (vauxn_in),

        .anode            (anode),
        .segment          (segment),
        .dp               (dp),

        // LEDs for data stream visualization (UART byte-level activity)
        .led0             (led0_int),
        .led1             (led1_int),
        .led2             (led2_int),
        .led3             (led3_int),
        .led4             (led4_int),
        .led5             (led5_int),
        .led6             (led6_int),
        .led7             (led7_int),

        // LEDs for fan + PIR output debugging
        .led9             (led9_int),
        .led10            (led10_int),
        .led11            (led11_int),
        .led12            (led12_int),
        .led13            (led13_int),
        .led14            (led14_int),
        .led15            (led15_int),

        // RGB LED 0 on Nexys A7 (LD16)
        .led0_r           (led0_r_int),
        .led0_g           (led0_g_int),
        .led0_b           (led0_b_int),
 
        .uart_txo         (uart_txo),
        .uart_txo_debug   (uart_txo_debug_int),

        .pwm_out          (pwm_out_int),

        //====================== PIR + fan control I/O ========================
        .pir_raw          (pir_raw),

        .fan_pwm_ja3      (fan_pwm_ja3),
        .fan_en_ja4       (fan_en_ja4_int),

        // VGA outputs (core-side naming)
        .vga_red          (vga_red),
        .vga_grn          (vga_grn),
        .vga_blu          (vga_blu),
        .vga_hsync_n      (vga_hsync_n_int),
        .vga_vsync_n      (vga_vsync_n_int),

        // TMP on-board sensor I2C and interrupts
        .TMP_SCL          (TMP_SCL),
        .TMP_SDA          (TMP_SDA),
        .TMP_INT          (TMP_INT),
        .TMP_CT           (TMP_CT),

        // Rotary encoder (Pmod JC)
        .jc_rot_a         (jc_rot_a),
        .jc_rot_b         (jc_rot_b),
        .jc_rot_sw        (jc_rot_sw)
    );

endmodule

`default_nettype wire
