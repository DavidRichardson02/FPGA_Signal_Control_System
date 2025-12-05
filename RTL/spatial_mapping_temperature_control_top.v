`timescale 1ns/1ps  // [opcode:`timescale] [operands: timeunit=1ns, timeprecision=1ps]
// Theory: Simulation uses 1 ns as the fundamental time unit; any fractional delays
//         are rounded to the nearest 1 ps. Synthesis tools ignore `timescale but
//         simulators depend on it for delays, #timing, and $time resolution.

//==============================================================================
// spatial_mapping_temperature_control_top.v
//------------------------------------------------------------------------------
// Role (system-level):
//   Single top-level wrapper for the FPGA_Signal_Control_System on Nexys A7.
//   It ties together:
//
//   1) Clocking, reset, and button debounce.
//   2) XADC-based temperature sampling (Q1.15) plus PWM DAC telemetry.
//   3) Temperature + PIR + manual fan control (proportional controller).
//   4) PIR “recent motion” visualization on RGB LED.
//   5) I²C Pmod ToF (ISL29501) measurement engine and surveyor FSM.
//   6) Rotary encoder front-end, with shared step source between AUTO sweep
//      and MANUAL “spinner” modes.
//   7) Unified survey position and Q1.15 survey-level for VGA spinner bar.
//   8) UART TX pipeline: ToF/angle/temperature/fan duty packetizer plus
//      2 Mb/s UART serializer and LED stream monitor.
//   9) UART RX pipeline: host → FPGA logo frame loader, double-buffered
//      320×240 RGB logo memory for VGA overlay (e.g., MATLAB-rendered frames).
//  10) VGA pipeline:
//         - vga_range_plot_top: 640×480 @ 60 Hz timing,
//           left-half ToF polar/range plot,
//           logo viewport mapping (320×240 region).
//         - image_dualbuf_320x240_rgb12: logo frame store, double-buffered
//           across system (100 MHz) and pixel domains.
//         - vga_status_overlay: right-panel HUD with temperature bars,
//           fan tiles, UART counters, compass rose, PIR indicators, and
//           rotary spinner bar inputs.
//         - rotary_bar_overlay: extra encoder-driven vertical bar overlay.
//  11) Seven-segment interfaces:
//         - Clock/heartbeat (HH:MM) display.
//         - Debug hex readout (rotary position, angle index, fan duty).
//
// Clock domains:
//   - clk_100MHz : fabric/system domain (everything except VGA pixel path).
//   - pix_clk    : VGA pixel domain (~25 MHz), generated in vga_range_plot_top.
//   - All CDCs are handled inside the specialist modules:
//       * ToF I²C master uses scl/sda open-drain semantics only.
//       * vga_status_overlay and rotary_bar_overlay snapshot / synchronize
//         100 MHz telemetry into the pixel domain.
//       * image_dualbuf_320x240_rgb12 arbitrates between 100 MHz writes
//         and pix_clk reads via double-buffering + frame_swap.
//
// External host (MATLAB / WaveForms / custom tool) interaction:
//   - TX: mapper_packetizer → uart_stream_tx → FT2232 → PC
//   - RX: uart_rx → logo_uart_frame_loader → image_dualbuf_320x240_rgb12
//     The RX path allows the PC to stream full-logo frames as RGB444 pixels
//     into one buffer, then request a swap so VGA displays the new frame in
//     the 320×240 viewport announced by vga_range_plot_top.
//
//==============================================================================

module spatial_mapping_temperature_control_top(
    //====================== Board I/O ==================================================
    input  wire        clk_100MHz,    // [opcode:input]  [operands: 100 MHz system clock from board osc]
    // Theory: Every synchronous path is timed against this source; no secondary clocks.

    input  wire        btnc,          // [opcode:input]  [operands: center pushbutton (active-HIGH)]
    input  wire        btnu,          // [opcode:input]  [operands: top pushbutton (time-set / debug)]
    input  wire        btnr,          // [opcode:input]  [operands: right pushbutton (time-set / debug)]
    // Theory: BTN C is promoted to global synchronous reset; BTN U/R feed the clock display logic.

    input  wire        btn_survey_start, // [opcode:input] [operands: start automatic survey sweep]
    input  wire        btn_survey_stop,  // [opcode:input] [operands: stop automatic survey sweep]

    input  wire        btnd,          // [opcode:input]  [operands: manual fan toggle button]
    // Theory: Buttons are human-speed; they must be synchronized/debounced in their modules.

    input  wire        sw_temp_en,       // TEMP enable switch
    input  wire        sw_manual_en,     // MANUAL override enable
    input  wire        sw_pir_en,        // PIR-assisted enable
    input  wire        sw_dbg_mode,      // Debug mux for LEDs and seven-seg
    input  wire        sw_survey_manual, // 0=AUTO (surveyor FSM), 1=MANUAL (rotary encoder)
    input  wire        sw_logo_sel,      // 0=show logo buffer 0, 1=show buffer 1  

    inout  wire        SCL,        // [opcode:inout]  [operands: I²C clock (open-drain)]
    inout  wire        SDA,        // [opcode:inout]  [operands: I²C data  (open-drain)]
    // Theory: For open-drain, logic '1' is realized by Hi-Z + external pull-up; logic '0' by drive-0.
    //         Never drive a '1' actively; only release (Z) or pull low.

    input  wire        tof_irq_n,  // from Pmod ToF IRQ/INT pin (ISL29501 open-drain interrupt)
    output wire        tof_ss,     // to Pmod ToF SS pin (Sample Start)

    input  wire        vauxp_in,   // [opcode:input]  [operands: XADC VAUXP[1] analog pad]
    input  wire        vauxn_in,   // [opcode:input]  [operands: XADC VAUXN[1] analog pad]
    // Theory: These are routed to the XADC hard macro. Do not assign digital IOSTANDARDs in XDC.

    output wire [7:0]  anode,      // [opcode:output] [operands: 7-seg digit enables, ACTIVE-LOW]
    output wire [6:0]  segment,    // [opcode:output] [operands: segments A..G, ACTIVE-LOW]
    output wire        dp,         // [opcode:output] [operands: decimal point, ACTIVE-LOW]
    // Theory: Nexys A7 seven-seg is common-anode; sinking current (logic 0) turns a segment/digit ON.

    /// LEDs for data stream visualization (UART byte-level activity or encoder debug)
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
    output wire        led0_r,
    output wire        led0_g,
    output wire        led0_b,

    // UART streaming / host interface
    output wire        uart_txo,       // [opcode:output] [operands: UART TX (8N1) to FT2232 USB bridge]
    output wire        uart_txo_debug, // mirrored TXO for logic analyzer / AD2
    input  wire        uart_rxi,       // FTDI → FPGA RX pin   

    output wire        pwm_out,    // [opcode:output] [operands: ~24.4 kHz PWM "DAC" to RC filter]

    //====================== PIR + fan control I/O ===============================
    input  wire        pir_raw,    // [opcode:input]  [operands: digital OUT from PIR board]
    // Theory: PIR OUT is a 3V3 TTL-level signal: HIGH when motion detected (after warm-up).

    output wire        fan_pwm_ja3,// [opcode:output] [operands: PWM drive to JA3 (E18) → fan FET]
    output wire        fan_en_ja4, // [opcode:output] [operands: logic enable to JA4 (G17) → fan driver]
    
    // VGA outputs (must match your XDC)
    output wire [3:0]  vga_red,
    output wire [3:0]  vga_grn,
    output wire [3:0]  vga_blu,
    output wire        vga_hsync_n,
    output wire        vga_vsync_n,
  
    // TMP on-board sensor I2C and interrupts (constrained in XDC)
    inout  wire        TMP_SCL,
    inout  wire        TMP_SDA,
    input  wire        TMP_INT,
    input  wire        TMP_CT,    
    
    // --- Pmod JC : KS0013 Rotary Encoder (digital channels) ---
    input  wire        jc_rot_a,   // JC pin wired to encoder CLK
    input  wire        jc_rot_b,   // JC pin wired to encoder DT
    input  wire        jc_rot_sw   // JC pin wired to encoder SW (pushbutton) 
);

    // ----------------------------
    // Utility: integer ceiling log2
    // Mirrors the CLOG2 used in sevenseg_clock_buttons and other blocks.
    // ----------------------------
    function integer CLOG2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (CLOG2 = 0; v > 0; CLOG2 = CLOG2 + 1)
                v = v >> 1;
        end
    endfunction

    //==============================================================================
    // 1. Global reset path (BTN C → rst)
    //==============================================================================
    // Purpose:
    //   Generate a single, clean, synchronous, active-HIGH reset (rst) for the
    //   entire design from the raw center pushbutton (btnc).
    //
    // Method:
    //   - 2-flip-flop synchronizer brings btnc safely into the clk_100MHz domain.
    //   - 5 ms debounce window filters mechanical chatter and slow edges.
    //   - rst_db becomes the debounced level of btnc; exported as wire rst.
    //
    // Design choices:
    //   - Debounce time (5 ms) matches the sevenseg_clock_buttons policy so that
    //     human input feels consistent.
    //   - Synchronous reset avoids asynchronous reset de-assertion hazards and
    //     keeps STA simple (all timing relative to clk_100MHz).
    //------------------------------------------------------------------------------

    // (1) 2-FF synchronizer for the center button
    reg rst_s0 = 1'b0;
    reg rst_s1 = 1'b0;

    always @(posedge clk_100MHz) begin
        rst_s0 <= btnc;
        rst_s1 <= rst_s0;
    end

    // (2) Debounce window: 5 ms @ 100 MHz -> 500,000 cycles
    //     Same math as in sevenseg_clock_buttons: 100 MHz / 1000 = 100_000 cycles/ms
    localparam integer RST_DB_TIME_MS = 5;
    localparam integer RST_DB_MAX     = (100_000 * RST_DB_TIME_MS); // 500,000
    localparam integer RST_DB_W       = CLOG2(RST_DB_MAX+1);

    // Debounce state for center button
    reg [RST_DB_W-1:0] rst_cnt = {RST_DB_W{1'b0}};
    reg                rst_db  = 1'b0; // debounced level

    always @(posedge clk_100MHz) begin
        if (rst_s1 == rst_db) begin
            // Input matches debounced level: restart timer
            rst_cnt <= {RST_DB_W{1'b0}};
        end else if (rst_cnt == RST_DB_MAX[RST_DB_W-1:0]) begin
            // Input has been stable for the entire debounce window: accept new level
            rst_db  <= rst_s1;
            rst_cnt <= {RST_DB_W{1'b0}};
        end else begin
            // Input differs from debounced level but hasn't been stable long enough
            rst_cnt <= rst_cnt + 1'b1;
        end
    end

    // Clean, global, synchronous, debounced reset
    wire rst = rst_db;

    //==============================================================================
    // 2. Manual fan control input (BTN D → btn_d_level / btn_d_pulse)
    //==============================================================================
    // Purpose:
    //   Turn the noisy, asynchronous btnd input into clean, synchronous signals
    //   that the fan controller can safely use for a toggle-style manual override.
    //
    // Signals:
    //   - btn_d_level : debounced logic level of BTN D (in clk_100MHz domain).
    //   - btn_d_pulse : single-cycle pulse on each rising edge of btn_d_level.
    //
    // Method:
    //   - 2-FF synchronizer removes metastability from btnd.
    //   - 5 ms debounce window (same constants as for rst) enforces a stable
    //     level before updating d_db.
    //   - Edge detector (d_db_prev vs d_db) generates a one-shot pulse on press.
    //------------------------------------------------------------------------------

    reg d_s0      = 1'b0;
    reg d_s1      = 1'b0;
    reg [RST_DB_W-1:0] d_cnt = {RST_DB_W{1'b0}};
    reg d_db      = 1'b0;   // debounced level
    reg d_db_prev = 1'b0;   // for edge-detect

    always @(posedge clk_100MHz) begin
        // (1) 2-FF synchronizer for raw btnd input
        d_s0 <= btnd;
        d_s1 <= d_s0;

        // (2) Debounce: require 5 ms of stable d_s1 before updating d_db
        if (d_s1 == d_db) begin
            // Input matches current debounced level: reset timer
            d_cnt <= {RST_DB_W{1'b0}};
        end else if (d_cnt == RST_DB_MAX[RST_DB_W-1:0]) begin
            // Input has been stable for the entire debounce window: accept new level
            d_db  <= d_s1;
            d_cnt <= {RST_DB_W{1'b0}};
        end else begin
            // Input differs from debounced level but hasn't been stable long enough
            d_cnt <= d_cnt + 1'b1;
        end

        // (3) Track previous debounced level for edge detection
        d_db_prev <= d_db;
    end

    // Debounced level and 1-cycle rising-edge pulse
    wire btn_d_level = d_db;               // synchronous, debounced BTN D
    wire btn_d_pulse = d_db & ~d_db_prev;  // one-shot on press

    //==============================================================================
    // 3. Survey control buttons: start/stop pulses for surveyor_fsm
    //==============================================================================
    // These are parallel to BTN D logic: synchronized + debounced + edge-detected,
    // generating clean 1-cycle pulses used by surveyor_fsm.
    //------------------------------------------------------------------------------

    wire sweep_start_pulse;
    wire sweep_stop_pulse;

    reg sv_start_s0      = 1'b0, sv_start_s1      = 1'b0;
    reg [RST_DB_W-1:0] sv_start_cnt = {RST_DB_W{1'b0}};
    reg sv_start_db      = 1'b0;
    reg sv_start_db_prev = 1'b0;

    reg sv_stop_s0       = 1'b0, sv_stop_s1       = 1'b0;
    reg [RST_DB_W-1:0] sv_stop_cnt  = {RST_DB_W{1'b0}};
    reg sv_stop_db       = 1'b0;
    reg sv_stop_db_prev  = 1'b0;

    always @(posedge clk_100MHz) begin
        // --- START button conditioning ---
        sv_start_s0 <= btn_survey_start;
        sv_start_s1 <= sv_start_s0;

        if (sv_start_s1 == sv_start_db) begin
            sv_start_cnt <= {RST_DB_W{1'b0}};
        end else if (sv_start_cnt == RST_DB_MAX[RST_DB_W-1:0]) begin
            sv_start_db  <= sv_start_s1;
            sv_start_cnt <= {RST_DB_W{1'b0}};
        end else begin
            sv_start_cnt <= sv_start_cnt + 1'b1;
        end

        sv_start_db_prev <= sv_start_db;

        // --- STOP button conditioning ---
        sv_stop_s0 <= btn_survey_stop;
        sv_stop_s1 <= sv_stop_s0;

        if (sv_stop_s1 == sv_stop_db) begin
            sv_stop_cnt <= {RST_DB_W{1'b0}};
        end else if (sv_stop_cnt == RST_DB_MAX[RST_DB_W-1:0]) begin
            sv_stop_db  <= sv_stop_s1;
            sv_stop_cnt <= {RST_DB_W{1'b0}};
        end else begin
            sv_stop_cnt <= sv_stop_cnt + 1'b1;
        end

        sv_stop_db_prev <= sv_stop_db;
    end

    assign sweep_start_pulse = sv_start_db & ~sv_start_db_prev; // 1-cycle pulse
    assign sweep_stop_pulse  = sv_stop_db  & ~sv_stop_db_prev;  // 1-cycle pulse

    //==============================================================================
    // 4. Seven-segment HH:MM clock and heartbeat display
    //==============================================================================
    // Purpose:
    //   Provide a continuously-running human interface clock on the Nexys A7
    //   7-segment display, with button-based time adjustment and an implicit
    //   “heartbeat” (constant multiplexing activity) that shows the FPGA is alive.
    //
    // Integration:
    //   - Uses clk_100MHz as its only clock input.
    //   - Consumes debounced btnc (rst), plus btnu and btnr for time set.
    //   - Drives:
    //       an_clock[7:0]   : active-LOW digit enables (pre-mux).
    //       seg_clock[6:0]  : active-LOW segments.
    //       dp_clock        : active-LOW decimal point.
    //------------------------------------------------------------------------------

    wire [7:0] an_clock;
    wire [6:0] seg_clock;
    wire       dp_clock;

    sevenseg_clock_buttons #(
        .DEBUG_BRINGUP(0)
    ) u_clock (
        .clk_100mhz (clk_100MHz),
        .btnc       (rst),
        .btnu       (btnu),
        .btnr       (btnr),
        .anode      (an_clock),
        .segment    (seg_clock),
        .dp         (dp_clock)
    );

    //==============================================================================
    // 5. Board temperature acquisition → canonical Q1.15 temperature
    //==============================================================================
    // Goal:
    //   - Define a single canonical temperature bus for the entire design:
    //        samp_q15   : signed Q1.15 temperature.
    //        samp_valid : 1-cycle strobe when samp_q15 updates.
    //   - Currently sourced from the XADC front-end (analog board sensor) OR the
    //     on-board TMP I²C digital sensor (ADT7420-class).
    //
    // Convention:
    //   - Downstream logic (fan controller, PWM DAC, HUD, packetizer) must ONLY
    //     look at samp_q15 / samp_valid and remain agnostic to which physical
    //     sensor is providing the temperature.
    //------------------------------------------------------------------------------
    // On-board TMP sensor I2C bus (TMP_SCL/TMP_SDA) — wired similarly as OD.
     wire tmp_scl_in  = TMP_SCL;
     wire tmp_sda_in  = TMP_SDA;
     wire tmp_scl_oen;
     wire tmp_sda_oen;
 
     assign TMP_SCL = tmp_scl_oen ? 1'bz : 1'b0;
     assign TMP_SDA = tmp_sda_oen ? 1'bz : 1'b0;
     // Note: TMP_* master/controller is not instantiated in this top in the current revision.
    // -----------------------------
    // XADC-derived analog temperature
    // -----------------------------
    wire [15:0] xadc_temp_q15;
    wire        xadc_temp_vld;

    xadc_sampler u_xadc (
        .clk        (clk_100MHz),
        .rst        (rst),
        .vauxp_in   (vauxp_in),
        .vauxn_in   (vauxn_in),
        .sample_q15 (xadc_temp_q15),
        .sample_vld (xadc_temp_vld)
    );

    // -----------------------------
    // On-board TMP I²C temperature (digital)
    // -----------------------------
    // TMP_SCL / TMP_SDA are already wired as open-drain:
    //
    //   wire tmp_scl_in  = TMP_SCL;
    //   wire tmp_sda_in  = TMP_SDA;
    //   wire tmp_scl_oen;
    //   wire tmp_sda_oen;
    //
    //   assign TMP_SCL = tmp_scl_oen ? 1'bz : 1'b0;
    //   assign TMP_SDA = tmp_sda_oen ? 1'bz : 1'b0;
    //
    // This reader block owns the bus whenever "busy" is high.
    // -----------------------------

    wire signed [15:0] tmp_temp_q15;  // Q1.15 from TMP sensor
    wire               tmp_temp_vld;  // 1-cycle strobe per TMP conversion
    wire        [7:0]  tmp_status;    // optional status byte (alarms, errors)
    wire               tmp_busy;      // high while I²C transaction in-flight

    tmp_temp_reader #(
        .CLK_HZ            (100_000_000),
        .MEAS_PERIOD_TICKS (5_000_000),   // ~20 Hz, same cadence as ToF if desired
        .I2C_ADDR7         (7'h4B),       // ADT7420 default address
        .SCALE_Q15_PER_C   (16'sd300),    // TODO: calibrate vs XADC
        .OFFSET_Q15        (16'sd0)       // TODO: adjust for °C→°F or board offset
    ) u_tmp_temp (
        .clk         (clk_100MHz),
        .rst         (rst),

        .tmp_scl_in  (tmp_scl_in),
        .tmp_sda_in  (tmp_sda_in),
        .tmp_scl_oen (tmp_scl_oen),
        .tmp_sda_oen (tmp_sda_oen),

        .temp_q15    (tmp_temp_q15),
        .temp_vld    (tmp_temp_vld),

        .status      (tmp_status),
        .busy        (tmp_busy)
    );

   
   
    // -----------------------------------------------------------------------------
    // Temperature source MUX
    // -----------------------------------------------------------------------------
    // TEMP_SRC_XADC : use XADC-derived analog temperature
    // TEMP_SRC_TMP  : use on-board TMP I²C-derived digital temperature
    // -----------------------------------------------------------------------------
    localparam [0:0] TEMP_SRC_XADC = 1'b0;
    localparam [0:0] TEMP_SRC_TMP  = 1'b1;

    // For now, hard-wire to XADC. Later, drive this from a switch or config reg.
    reg temp_src_sel = TEMP_SRC_XADC;

    // Canonical board temperature used everywhere else in the design
    wire signed [15:0] samp_q15;
    wire               samp_valid;

    assign samp_q15   = (temp_src_sel == TEMP_SRC_TMP) ? tmp_temp_q15   : xadc_temp_q15;
    assign samp_valid = (temp_src_sel == TEMP_SRC_TMP) ? tmp_temp_vld   : xadc_temp_vld;


    //==============================================================================
    // 6. PIR motion front-end: pir_raw → pir_active, pir_rise
    //==============================================================================
    // Purpose:
    //   Convert the raw, asynchronous PIR output into two clean motion signals:
    //
    //     pir_active : level that stays HIGH for HOLD_S seconds after last motion
    //                  (“recent motion” window).
    //     pir_rise   : single-cycle pulse per new rising edge (event marker).
    //
    //------------------------------------------------------------------------------

    wire pir_active;
    wire pir_rise;

    pir_conditioner #(
        .CLK_HZ      (100_000_000), // fabric clock
        .DEBOUNCE_MS (20),
        .WARMUP_S    (30),
        .HOLD_S      (15)
    ) u_pir_conditioner (
        .clk        (clk_100MHz),
        .rst        (rst),
        .pir_raw    (pir_raw),
        .active     (pir_active),
        .rise_pulse (pir_rise)
    );
    
    //==============================================================================
    // 7. PIR "recent motion" RGB visualization (RGB LED 0)
    //==============================================================================
    // The RGB LED shows a “cooling” motion bar:
    //   - Full green right after motion (pir_rise event).
    //   - Fades toward red as HOLD_S expires.
    //   - Off (black) when pir_active is low.
    //------------------------------------------------------------------------------

    pir_recent_motion_rgb #(
        .CLK_HZ (100_000_000),
        .HOLD_S (15)                 // match pir_conditioner.HOLD_S
    ) pir_recent_motion_rgb_inst (
        .clk     (clk_100MHz),
        .rst     (rst),
        .pir_rise(pir_rise),
        .pir_on  (pir_active),
        .rgb_r   (led0_r),
        .rgb_g   (led0_g),
        .rgb_b   (led0_b)
    );

    //==============================================================================
    // 8. Q1.15 temperature sample → 12-bit PWM duty (pwm_out)
    //==============================================================================
    // This is a general-purpose PWM “DAC” driven by samp_q15.
    // Mapping:
    //   x ∈ Q1.15 → offset-binary x_off = x + 0x8000 ∈ [0, 2.0)
    //   u = x_off / 2 ∈ [0, 1) ⇒ duty ≈ floor(u * 4095)
    //
    // PWM carrier: 12-bit free-running counter @ 100 MHz ⇒ ~24.4 kHz PWM.
    //------------------------------------------------------------------------------
    // The PWM DAC now reflects the same canonical board temperature (samp_q15)
    // that the fan controller and HUD use.
    //------------------------------------------------------------------------------
    
    localparam [1:0] PWM_SRC_SAMPLE = 2'd0,
                     PWM_SRC_DC     = 2'd1;
    
    reg  [1:0]  pwm_src_sel = PWM_SRC_SAMPLE;
    reg  [15:0] pwm_dc_q15  = 16'sd0;
    
    // Use canonical temperature samp_q15 as the live PWM source when in SAMPLE mode
    wire [15:0] pwm_in_q15 =
        (pwm_src_sel == PWM_SRC_DC) ? pwm_dc_q15 : samp_q15;
    
    wire [15:0] x_off = pwm_in_q15 + 16'sh8000;
    wire [27:0] mult  = x_off * 12'd4095;
    wire [11:0] duty  = mult[27:16];
    
    pwm_dac #(.CTR_W(12)) u_pwm (
        .clk  (clk_100MHz),
        .rst  (rst),
        .duty (duty),
        .pwm  (pwm_out)
    );


    //==============================================================================
    // 9. Temperature + PIR + manual fan controller
    //==============================================================================
    // temp_fan_ctrl consumes:
    //   - samp_q15 / samp_valid : Q1.15 temperature estimate + strobe.
    //   - btn_d_*               : manual override button (toggle latch).
    //   - pir_active            : “recent motion” from PIR conditioner.
    //   - sw_*                  : enables for temp/manual/PIR contributions.
    //
    // Produces:
    //   - fan_pwm_ja3 / fan_en_ja4 : physical fan drive.
    //   - fan_temp_on, fan_manual_on, fan_pir_on : contribution flags.
    //   - duty_final_q15_dbg       : final Q1.15 duty (for HUD + debug).
    //------------------------------------------------------------------------------
//==============================================================================
    // 9. Temperature + PIR + manual fan controller
    //==============================================================================
    // Inputs:
    //   samp_q15   : canonical Q1.15 board temperature (from section 5).
    //   samp_valid : 1-cycle strobe on new sample.
    //------------------------------------------------------------------------------
    
    wire        fan_temp_on;
    wire        fan_manual_on;
    wire        fan_pir_on;
    wire        fan_on;
    wire [15:0] fan_duty_q15_dbg;
    
    temp_fan_ctrl #(
        .T_SET_OCC_Q15   (16'sd14564), // 72°F
        .T_SET_UNOCC_Q15 (16'sd16748), // 78°F
        .KP_OCC_Q14      (16'd4),
        .KP_UNOCC_Q14    (16'd2),
        .U_BIAS_Q15      (16'sd1024),
        .DUTY_PIR_Q15    (16'h6000),
        .CTRL_DIV        (5_000_000)   // ~20 Hz control loop
    ) u_temp_fan_ctrl (
        .clk_100mhz (clk_100MHz),
        .rst        (rst),
    
        // *** canonical board temperature ***
        .sample_q15 (samp_q15),
        .sample_vld (samp_valid),
    
        // Manual + PIR inputs
        .btn_d_level(btn_d_level),
        .btn_d_pulse(btn_d_pulse),
        .pir_on     (pir_active),
    
        // Enable switches
        .sw_temp_en   (sw_temp_en),
        .sw_manual_en (sw_manual_en),
        .sw_pir_en    (sw_pir_en),
    
        // Fan outputs
        .fan_pwm_ja3  (fan_pwm_ja3),
        .fan_en_ja4   (fan_en_ja4),
    
        // Contribution flags
        .fan_temp_on   (fan_temp_on),
        .fan_manual_on (fan_manual_on),
        .fan_pir_on    (fan_pir_on),
    
        // Debug duty tap
        .duty_final_q15_dbg (fan_duty_q15_dbg)
    );

    // LED debug mapping for fan logic
    assign led15 = fan_pir_on;    // PIR contribution to fan
    assign led14 = fan_manual_on; // Manual override contribution to fan
    assign led13 = fan_temp_on;   // Temperature contribution to fan
    
    assign led12 = sw_temp_en;    // TEMP enable switch state
    assign led11 = sw_manual_en;  // MANUAL enable switch state
    assign led10 = sw_pir_en;     // PIR enable switch state

    // Aggregate fan_on for VGA overlay
    assign fan_on = fan_temp_on | fan_manual_on | fan_pir_on;

    //==============================================================================
    // 10. I²C wiring (open-drain) and Pmod ToF polling engine
    //==============================================================================

    // I2C OD pads for Pmod ToF
    wire scl_in = SCL;
    wire sda_in = SDA;

    wire scl_oen; // 1=release, 0=drive 0
    wire sda_oen;

    assign SCL = scl_oen ? 1'bz : 1'b0;
    assign SDA = sda_oen ? 1'bz : 1'b0;


    // ToF polling engine (Pmod ToF ISL29501 @ 0x57, ~20 Hz nominal)
    wire [15:0] tof_dist_mm;
    wire [7:0]  tof_status;
    wire        tof_vld;
    wire        tof_busy;

    tof_sensor #(
        .CLK_HZ            (100_000_000),
        .I2C_ADDR7         (7'h57),
        .MEAS_PERIOD_TICKS (5_000_000),   // ~20 Hz
        .SS_ACTIVE_HIGH    (1'b0),
        .IRQ_ACTIVE_LOW    (1'b1),
        .SS_PULSE_TICKS    (500),
        .IRQ_TIMEOUT_TICKS (1_000_000)
    ) u_tof (
        .clk        (clk_100MHz),
        .rst        (rst),
        .scl_in     (scl_in),
        .scl_oen    (scl_oen),
        .sda_in     (sda_in),
        .sda_oen    (sda_oen),

        .tof_irq_n  (tof_irq_n),
        .tof_ss     (tof_ss),

        .dist_mm    (tof_dist_mm),
        .status     (tof_status),
        .sample_vld (tof_vld),
        .busy       (tof_busy)
    );

    assign led9 = tof_irq_n; // quick-view of interrupt line (active-low from sensor)

    //==============================================================================
    // 11. Rotary encoder front-end (manual spinner / surveyor input)
    //==============================================================================
    // rotary_encoder_quadrature:
    //   - Decodes quadrature CLK/DT into:
    //       * rot_step_pulse (1 cycle per detent).
    //       * rot_step_dir   (1=clockwise, 0=counter-clockwise).
    //   - Maintains a signed 16-bit position counter (rot_pos).
    //   - Debounces the encoder pushbutton (rot_btn_*).
    //------------------------------------------------------------------------------

    wire        rot_step_pulse;
    wire        rot_step_dir;
    wire signed [15:0] rot_pos;
    wire        rot_btn_level;
    wire        rot_btn_pulse;

    rotary_encoder_quadrature #(
        .CLK_HZ      (100_000_000),
        .DEBOUNCE_US (500)            // 0.5 ms; tweak if bounce observed
    ) u_rotenc (
        .clk        (clk_100MHz),
        .rst        (rst),

        .enc_a_raw  (jc_rot_a),
        .enc_b_raw  (jc_rot_b),
        .btn_raw    (jc_rot_sw),

        .step_pulse (rot_step_pulse),
        .step_dir   (rot_step_dir),
        .pos        (rot_pos),

        .btn_level  (rot_btn_level),
        .btn_pulse  (rot_btn_pulse)
    );

    // Encoder pushbutton as an additional survey start source (e.g., “start sweep”)
    wire encoder_start_pulse = rot_btn_pulse;
    wire sweep_start_any     = sweep_start_pulse | encoder_start_pulse;

    //==============================================================================
    // 12. Surveyor FSM + angle indexer (AUTO sweep mode)
    //==============================================================================

    // Outputs from surveyor FSM (ToF → gated sample stream)
    wire        survey_sample_vld;
    wire [15:0] survey_dist_mm;
    wire [7:0]  survey_status;

    wire        sweep_active;
    wire        step_pulse;   // from surveyor FSM
    wire        step_dir;     // from surveyor FSM

    surveyor_fsm #(
        .STEPS_PER_SWEEP(180)
    ) u_surveyor (
        .clk         (clk_100MHz),
        .rst         (rst),
        .sweep_start (sweep_start_any),
        .sweep_stop  (sweep_stop_pulse),

        .tof_vld     (tof_vld),
        .tof_dist_mm (tof_dist_mm),
        .tof_status  (tof_status),
        .tof_busy    (tof_busy),

        .sweep_active(sweep_active),
        .step_pulse  (step_pulse),
        .step_dir    (step_dir),

        .sample_vld  (survey_sample_vld),
        .dist_mm     (survey_dist_mm),
        .status      (survey_status)
    );

    // Step source MUX for angle_indexer:
    //   AUTO   (survey_manual_mode=0): use surveyor_fsm step_pulse/dir
    //   MANUAL (survey_manual_mode=1): use rotary encoder step_pulse/dir
    wire survey_manual_mode = sw_survey_manual;

    wire step_pulse_src;
    wire step_dir_src;
    assign step_pulse_src = survey_manual_mode ? rot_step_pulse : step_pulse;
    assign step_dir_src   = survey_manual_mode ? rot_step_dir   : step_dir;

    // Sample stream MUX:
    //   MANUAL: raw ToF samples (tof_*).
    //   AUTO  : surveyor gated samples (survey_*).
    wire        meas_vld;
    wire [15:0] meas_dist_mm;
    wire [7:0]  meas_status;

    assign meas_vld     = survey_manual_mode ? tof_vld        : survey_sample_vld;
    assign meas_dist_mm = survey_manual_mode ? tof_dist_mm    : survey_dist_mm;
    assign meas_status  = survey_manual_mode ? tof_status     : survey_status;

    // Unified survey "position" for VGA duty bar:
    //   Uses the same step_pulse_src / step_dir_src that feed angle_indexer.
    //   - Manual: tracks encoder.
    //   - Auto  : tracks sweep.
    reg signed [15:0] survey_pos = 16'sd0;
    
    always @(posedge clk_100MHz) begin
        if (rst) begin
            survey_pos <= 16'sd0;
        end else if (step_pulse_src) begin
            if (step_dir_src)
                survey_pos <= survey_pos + 16'sd1;  // CW → positive
            else
                survey_pos <= survey_pos - 16'sd1;  // CCW → negative
        end
    end

    // Absolute position → Q1.15 “survey level” for HUD slider
    wire signed [15:0] survey_pos_abs =
        survey_pos[15] ? -survey_pos : survey_pos;

    // Scaling: ~2^15 / 180 ≈ 182; saturate to 0x7FFF.
    wire [23:0] survey_mul   = survey_pos_abs * 9'd182;
    wire [15:0] survey_mul16 = survey_mul[15:0];

    wire signed [15:0] survey_level_q15 =
        (survey_mul16 > 16'h7FFF) ? 16'h7FFF : survey_mul16;

    // Angle indexer: single-turn θ in Q1.15 + multi-turn accumulator.
    wire [15:0] theta_q15;
    wire [31:0] theta_turn_q15;
    wire [15:0] angle_idx_dbg;

    angle_indexer #(
        .ANGLE_STEPS(180)
    ) u_angle (
        .clk            (clk_100MHz),
        .rst            (rst),
        .step_pulse     (step_pulse_src),
        .step_dir       (step_dir_src),
        .theta_q15      (theta_q15),
        .theta_turn_q15 (theta_turn_q15),
        .angle_idx      (angle_idx_dbg)
    );

    // Unified encoder/survey position for HUD & rotary_bar_overlay
    wire signed [15:0] enc_pos_mux;
    assign enc_pos_mux = survey_manual_mode ? rot_pos : survey_pos;

    //==============================================================================
    // 13. ToF → mapper_packetizer → UART stream (TX path)
    //==============================================================================

    wire [7:0]  pkt_byte;
    wire        pkt_vld;
    wire        pkt_rdy;
    wire [31:0] dbg_t_us;

    mapper_packetizer #(
        .CLK_HZ(100_000_000)
    ) u_pkt (
        .clk            (clk_100MHz),
        .rst            (rst),
        .sample_vld     (meas_vld),
        .theta_turn_q15 (theta_turn_q15),
        .theta_q15      (theta_q15),
        .dist_mm        (meas_dist_mm),
        .status         (meas_status),
        .temp_q15       (samp_q15),
        .fan_duty_q15_dbg(fan_duty_q15_dbg),
        .tx_byte        (pkt_byte),
        .tx_vld         (pkt_vld),
        .tx_rdy         (pkt_rdy),
        .dbg_t_us       (dbg_t_us)
    );

    uart_stream_tx #(
        .CLK_HZ(100_000_000),
        .BAUD  (2_000_000)
    ) u_uart_tx (
        .clk    (clk_100MHz),
        .rst    (rst),
        .tx_byte(pkt_byte),
        .tx_vld (pkt_vld),
        .tx_rdy (pkt_rdy),
        .txo    (uart_txo),
        .busy   (/* optional: hook to LED */)
    );

    assign uart_txo_debug = uart_txo;  // mirrored for scope / AD2

    // Byte accepted by UART shifter (for stream LEDs and logging)
    wire byte_accepted = pkt_vld & pkt_rdy;

    //==============================================================================
    // 14. UART RX path → logo frame loader → double-buffered logo memory
    //==============================================================================

    // UART RX: host → FPGA logo frames
    wire [7:0] rx_byte;
    wire       rx_vld;
    
    uart_rx #(
        .CLK_HZ(100_000_000),
        .BAUD  (2_000_000)
    ) u_uart_rx (
        .clk    (clk_100MHz),
        .rst    (rst),
        .rxi    (uart_rxi),
        .rx_byte(rx_byte),
        .rx_vld (rx_vld)
    );

    wire        logo_wr_en_sys;
    wire [16:0] logo_wr_addr_sys;
    wire [11:0] logo_wr_data_sys;

    wire        logo_swap_req_auto;   // from UART loader
    wire        logo_swap_req_sys;    // auto OR manual (to dualbuf)

    wire        logo_active_buf_sys;  // which bank is currently displayed (system view)
    wire        logo_write_buf_sys;   // which bank is being filled
    wire        logo_display_buf_pix; // pixel-domain view of active buffer


    logo_uart_frame_loader #(
        .WIDTH  (320),
        .HEIGHT (240)
    ) u_logo_uart_loader (
        .clk_sys       (clk_100MHz),
        .rst_sys       (rst),
        .rx_byte       (rx_byte),
        .rx_vld        (rx_vld),
        .write_buf_sys (logo_write_buf_sys),
        .logo_wr_en    (logo_wr_en_sys),
        .logo_wr_addr  (logo_wr_addr_sys),
        .logo_wr_data  (logo_wr_data_sys),
        .logo_swap_req (logo_swap_req_auto)   
    );
    
    
    //----------------------------------------------------------------------------
    // Manual logo buffer select (slide switch)
    //----------------------------------------------------------------------------
    // Goal:
    //   - sw_logo_sel = 0 → force display of buffer 0.
    //   - sw_logo_sel = 1 → force display of buffer 1.
    //
    // Mechanism:
    //   - logo_active_buf_sys is a 0/1 flag from image_dualbuf_320x240_rgb12
    //     indicating which bank is currently on-screen.
    //   - We synchronize sw_logo_sel into clk_100MHz and call it logo_bank_sel.
    //   - Whenever (logo_bank_sel != logo_active_buf_sys), we assert a manual
    //     swap request. The dual-buffer block:
    //        • syncs swap_req_sys into the pix_clk domain,
    //        • detects a rising edge,
    //        • toggles the display buffer at the next frame_tick_pix.
    //
    //   - Because the request is an XOR, it goes high once when you flip the
    //     switch (mismatch), stays high until the swap completes, then drops
    //     back to 0 when logo_active_buf_sys catches up.
    //----------------------------------------------------------------------------

    // 2-FF sync of sw_logo_sel into clk_100MHz
    reg logo_sel_s0 = 1'b0;
    reg logo_sel_s1 = 1'b0;

    always @(posedge clk_100MHz) begin
        if (rst) begin
            logo_sel_s0 <= 1'b0;
            logo_sel_s1 <= 1'b0;
        end else begin
            logo_sel_s0 <= sw_logo_sel;
            logo_sel_s1 <= logo_sel_s0;
        end
    end

    wire logo_bank_sel = logo_sel_s1;  // debounced-ish, synchronous level

    // Manual request: want buffer = logo_bank_sel, have buffer = logo_active_buf_sys
    wire logo_swap_req_manual = (logo_bank_sel != logo_active_buf_sys);

    // Final swap request into the dual-buffer logo memory:
    //   - auto (from UART loader after full frame)
    //   - manual (from slide switch selection)
    assign logo_swap_req_sys = logo_swap_req_auto | logo_swap_req_manual;



    // Pixel-domain logo interface (fed by VGA timing core)
    wire [8:0]  logo_x_pix;
    wire [7:0]  logo_y_pix;
    wire [11:0] logo_rgb_pix;

    // VGA timing / ToF plot outputs (declared here for clarity)
    wire        pix_clk;
    wire [9:0]  hcount;
    wire [9:0]  vcount;
    wire        active_video;
    wire        frame_tick_pix;
    wire [11:0] rgb_plot;
    wire [11:0] rgb_overlay;
    wire [3:0]  vga_r_ovl;
    wire [3:0]  vga_g_ovl;
    wire [3:0]  vga_b_ovl;

    // Double-buffered 320x240 logo store:
    //   - Receives writes over clk_100MHz.
    //   - Supplies reads over pix_clk.
    //   - Swaps buffers on logo_swap_req_sys at frame boundaries (frame_tick_pix).
    image_dualbuf_320x240_rgb12 u_logo_dualbuf (
        // Pixel domain
        .clk_pix        (pix_clk),
        .rst_pix        (rst),
        .x_pix          (logo_x_pix),
        .y_pix          (logo_y_pix),
        .frame_tick_pix (frame_tick_pix),
        .rgb_pix        (logo_rgb_pix),

        // System domain write side
        .clk_sys        (clk_100MHz),
        .rst_sys        (rst),
        .wr_en_sys      (logo_wr_en_sys),
        .wr_addr_sys    (logo_wr_addr_sys),
        .wr_data_sys    (logo_wr_data_sys),

        .swap_req_sys   (logo_swap_req_sys),

        .display_buf_pix(logo_display_buf_pix),
        .active_buf_sys (logo_active_buf_sys),
        .write_buf_sys  (logo_write_buf_sys)
    );

    //==============================================================================
    // 15. Stream LED monitor (TX activity bar) + debug mux to LEDs
    //==============================================================================

    wire [7:0] stream_leds;
    wire       stream_blink;

    stream_led_monitor #(
        .DEC_SHIFT(5)          // 2^5=32 bytes per LED increment
    ) u_stream_leds (
        .clk          (clk_100MHz),
        .rst          (rst),
        .byte_accepted(byte_accepted),
        .leds         (stream_leds),
        .blink        (stream_blink)
    );

    // Debug mode switch
    wire dbg_mode = sw_dbg_mode;

    // Encoder + angle debug nibble mapping for seven-seg debug instance
    wire [3:0] dbg_hex7, dbg_hex6, dbg_hex5, dbg_hex4;
    wire [3:0] dbg_hex3, dbg_hex2, dbg_hex1, dbg_hex0;
    
    assign dbg_hex7 = rot_pos[15:12];
    assign dbg_hex6 = rot_pos[11: 8];
    assign dbg_hex5 = rot_pos[ 7: 4];
    assign dbg_hex4 = rot_pos[ 3: 0];

    assign dbg_hex3 = angle_idx_dbg[7:4];
    assign dbg_hex2 = angle_idx_dbg[3:0];

    assign dbg_hex1 = fan_duty_q15_dbg[7:4];
    assign dbg_hex0 = fan_duty_q15_dbg[3:0];
    
    wire [7:0] dbg_an;
    wire [6:0] dbg_seg;
    wire       dbg_dp;
    
    sevenseg_hex_debug #(
        .CLK_HZ     (100_000_000),
        .REFRESH_HZ (1000),
        .DP_MASK    (8'b0000_0000),
        .SELF_TEST  (1'b0)
    ) u_ss_dbg (
        .clk   (clk_100MHz),
        .rst   (rst),
        .hex7  (dbg_hex7),
        .hex6  (dbg_hex6),
        .hex5  (dbg_hex5),
        .hex4  (dbg_hex4),
        .hex3  (dbg_hex3),
        .hex2  (dbg_hex2),
        .hex1  (dbg_hex1),
        .hex0  (dbg_hex0),
        .digit_en (8'hFF),
        .an    (dbg_an),
        .seg   (dbg_seg),
        .dp    (dbg_dp)
    );    

    // Seven-seg and LED muxing:
    //   dbg_mode = 0 → normal HH:MM clock on 7-seg, stream LEDs on led[7:0].
    //   dbg_mode = 1 → encoder/angle view on 7-seg, encoder LEDs on led[7:0].

    // Seven-seg output mux
    assign anode   = dbg_mode ? dbg_an   : an_clock;
    assign segment = dbg_mode ? dbg_seg  : seg_clock;
    assign dp      = dbg_mode ? dbg_dp   : dp_clock; 

    // LED0..LED7 mux
    wire [7:0] led_rot_view;

    assign led_rot_view[3:0] = rot_pos[3:0];   // low nibble of encoder position
    assign led_rot_view[4]   = rot_step_pulse; // 1-cycle pulse per detent
    assign led_rot_view[5]   = rot_step_dir;   // 1=CW, 0=CCW
    assign led_rot_view[7:6] = 2'b00;          // reserved

    wire [7:0] led_mux = dbg_mode ? led_rot_view : stream_leds;

    assign led7 = led_mux[7];
    assign led6 = led_mux[6];
    assign led5 = led_mux[5];
    assign led4 = led_mux[4];
    assign led3 = led_mux[3];
    assign led2 = led_mux[2];
    assign led1 = led_mux[1];
    assign led0 = led_mux[0];

    // Encoder position used for VGA overlay (system-domain view)
    wire signed [15:0] enc_pos_sys      = enc_pos_mux;
    wire               enc_step_pulse_sys = step_pulse_src;
    wire               enc_dir_sys        = step_dir_src;

    //==============================================================================
    // 16. VGA background: ToF plot + logo viewport
    //==============================================================================

    vga_range_plot_top #(
        .R_MAX_MM      (16'd2000),
        .TEMP_Q15_COLD (16'sd4096),
        .TEMP_Q15_HOT  (16'sd16384)
    ) u_vga (
        .clk_100mhz   (clk_100MHz),
        .rst_sys      (rst),

        // ToF polar samples
        .dist_mm      (meas_dist_mm),
        .dist_vld     (meas_vld),
        .theta_q15    (theta_q15),
        .sweep_wrap   (survey_manual_mode ? rot_step_pulse : step_pulse),

        // VGA timing + background RGB
        .pix_clk      (pix_clk),
        .hcount       (hcount),
        .vcount       (vcount),
        .active_video (active_video),
        .frame_tick   (frame_tick_pix),
        .rgb_plot     (rgb_plot),

        // HS/VS pins
        .vga_hsync_n  (vga_hsync_n),
        .vga_vsync_n  (vga_vsync_n),

        // Logo viewport mapping (320×240 region within 640×480)
        .logo_x       (logo_x_pix),
        .logo_y       (logo_y_pix),
        .logo_rgb     (logo_rgb_pix),
        .logo_buf_sel (logo_display_buf_pix)
    );

    //==============================================================================
    // 17. Status overlay (right-panel HUD) + rotary bar overlay
    //==============================================================================

    vga_status_overlay u_status (
        .pix_clk        (pix_clk),
        .rst            (rst),

        .hcount         (hcount),
        .vcount         (vcount),
        .active_video   (active_video),
        .frame_tick     (frame_tick_pix),

        .rgb_bg         (rgb_plot),

        // Telemetry (system-domain, snapshotted internally)
        .sample_q15     (samp_q15),
        .duty_q15       (survey_level_q15),
        .fan_temp_on    (fan_temp_on),
        .fan_manual_on  (fan_manual_on),
        .fan_pir_on     (fan_pir_on),
        .fan_on         (fan_on),

        .uart_tx_byte   (pkt_byte),
        .uart_tx_vld    (pkt_vld),
        .byte_accepted  (byte_accepted),

        .theta_q15      (theta_q15),
        .pir_active     (pir_active),
        .pir_rise       (pir_rise),

        .enc_pos        (enc_pos_sys),
        .enc_step_pulse (enc_step_pulse_sys),
        .enc_dir        (enc_dir_sys),

        .sw_temp_en     (sw_temp_en),
        .sw_manual_en   (sw_manual_en),
        .sw_pir_en      (sw_pir_en),

        .rgb_out        (rgb_overlay)
    );

    // Rotary encoder vertical side-bar overlay AFTER HUD composition
    rotary_bar_overlay #(
        .H_RES      (640),
        .V_RES      (480),
        .BAR_X0     (600),
        .BAR_X1     (620),
        .CENTER_Y   (480/2),
        .BAR_MAX_PIX((480/2) - 10),
        .POS_SHIFT  (5)          // 32 encoder counts per vertical pixel
    ) u_rotary_bar (
        .clk_pix        (pix_clk),
        .rst            (rst),

        .pix_x          (hcount),
        .pix_y          (vcount),
        .video_active   (active_video),

        // System-domain encoder taps (module performs its own CDC)
        .enc_pos        (enc_pos_sys),
        .enc_step_pulse (enc_step_pulse_sys),
        .enc_dir        (enc_dir_sys),

        // Base color is status overlay’s output
        .base_r         (rgb_overlay[11:8]),
        .base_g         (rgb_overlay[7:4]),
        .base_b         (rgb_overlay[3:0]),

        .out_r          (vga_r_ovl),
        .out_g          (vga_g_ovl),
        .out_b          (vga_b_ovl)
    );

    // Final VGA DAC drive
    assign vga_red  = vga_r_ovl;
    assign vga_grn  = vga_g_ovl;
    assign vga_blu  = vga_b_ovl;

    //==============================================================================
    // 18. Debug print / CSV logging: raw ToF vs surveyor distances
    //==============================================================================
    // Simulation-only instrumentation for correlating:
    //   - Raw tof_dist_mm/status at tof_vld
    //   - Surveyor-gated survey_dist_mm/status at survey_sample_vld
    // Outputs both to console and to dist_debug_log.csv.
    //------------------------------------------------------------------------------

    integer dist_log_fd;
    initial begin
        dist_log_fd = $fopen("dist_debug_log.csv", "w");
        if (dist_log_fd) begin
            $fwrite(dist_log_fd,
                "time_ns,tof_vld,tof_dist_mm,tof_status,survey_vld,survey_dist_mm,survey_status\n");
        end else begin
            $display("ERROR: could not open dist_debug_log.csv");
        end
    end
        
    always @(posedge clk_100MHz) begin
        if (tof_vld || survey_sample_vld) begin
            // Console logging
            $display("[%0t ns] tof_vld=%0d tof_dist=%0d mm status=0x%02h  |  survey_vld=%0d survey_dist=%0d mm status=0x%02h",
                     $time,
                     tof_vld,   tof_dist_mm,   tof_status,
                     survey_sample_vld, survey_dist_mm, survey_status);

            // CSV file logging
            if (dist_log_fd) begin
                $fwrite(dist_log_fd,
                    "%0t,%0d,%0d,0x%02h,%0d,%0d,0x%02h\n",
                    $time,
                    tof_vld,   tof_dist_mm,   tof_status,
                    survey_sample_vld, survey_dist_mm, survey_status);
            end
        end
    end

endmodule
