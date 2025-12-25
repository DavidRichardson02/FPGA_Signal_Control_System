`timescale 1ns/1ps

// ============================================================================
// Module : nexys_a7_accel_selftest_top
// Role   : Minimal bring-up harness for the Nexys A7 ADXL362 accelerometer.
//          - Instantiates only adxl362_if (which internally instantiates the
//            SPI master).
//          - Wires the FPGA pins directly to the accelerometer SPI pins.
//          - Drives LEDs with:
//                * DEVID register (0xAD expected)  → led[7:0]
//                * Sign bits of raw X/Y/Z          → led[10:8]
//                * init_done                       → led[11]
//                * devid_error                     → led[12]
//                * "sample_seen" stretched pulse   → led[13]
// ============================================================================

module nexys_a7_accel_selftest_top (
    input  wire clk_100MHz,   // Nexys A7 on-board 100 MHz clock
    input  wire btnc,         // center button as synchronous reset (active-high)

    // ADXL362 SPI pins (match these to .xdc!)
    input  wire ACL_MISO,
    output wire ACL_MOSI,
    output wire ACL_SCLK,
    output wire ACL_CSN,

 /// LEDs for data stream visualization (UART byte-level activity or encoder debug)
       output wire        led0,
       output wire        led1,
       output wire        led2,
       output wire        led3,
       output wire        led4,
       output wire        led5,
       output wire        led6,
       output wire        led7,
       output wire        led8,
       /// LEDs for fan + PIR output debugging
       output wire        led9,
       output wire        led10,
       output wire        led11, 
       output wire        led12,
       output wire        led13,
       output wire        led14,
       output wire        led15
);

    // ------------------------------------------------------------------------
    // Reset (simple: directly from btnC)
    // ------------------------------------------------------------------------
    wire rst = btnc;

    // ------------------------------------------------------------------------
    // Wires from adxl362_if
    // ------------------------------------------------------------------------
    wire signed [7:0] accel_x_q7;
    wire signed [7:0] accel_y_q7;
    wire signed [7:0] accel_z_q7;
    wire              sample_valid;
    wire              init_done;
    wire              devid_error;

    wire [7:0]        devid_debug;
    wire              devid_debug_valid;
    wire signed [7:0] raw_x_dbg;
    wire signed [7:0] raw_y_dbg;
    wire signed [7:0] raw_z_dbg;

    // ------------------------------------------------------------------------
    // Instantiate ADXL362 interface
    // ------------------------------------------------------------------------
    adxl362_if u_adxl362_if (
        .clk                  (clk_100mhz),
        .rst                  (rst),

        // SPI to accelerometer pins
        .ACL_MISO             (ACL_MISO),
        .ACL_MOSI             (ACL_MOSI),
        .ACL_SCLK             (ACL_SCLK),
        .ACL_CSN              (ACL_CSN),

        // Scaled Q7 outputs (unused here except for sign test if desired)
        .accel_x              (accel_x_q7),
        .accel_y              (accel_y_q7),
        .accel_z              (accel_z_q7),
        .sample_valid         (sample_valid),

        // Status
        .init_done            (init_done),
        .devid_error          (devid_error),

        // Debug outputs
        .devid_debug          (devid_debug),
        .devid_debug_valid    (devid_debug_valid),
        .raw_x_dbg            (raw_x_dbg),
        .raw_y_dbg            (raw_y_dbg),
        .raw_z_dbg            (raw_z_dbg)
    );

    // ------------------------------------------------------------------------
    // Stretch sample_valid so we can see it on an LED
    // ------------------------------------------------------------------------
    reg [23:0] sample_seen_ctr;

    always @(posedge clk_100mhz) begin
        if (rst) begin
            sample_seen_ctr <= 24'd0;
        end else begin
            if (sample_valid) begin
                // Load a visible duration (~0.16 s at 100 MHz)
                sample_seen_ctr <= 24'hFFFFFF;
            end else if (sample_seen_ctr != 24'd0) begin
                sample_seen_ctr <= sample_seen_ctr - 24'd1;
            end
        end
    end

    wire sample_seen = (sample_seen_ctr != 24'd0);


       // ------------------------------------------------------------------------
       // LED mapping (scalar form)
       // ------------------------------------------------------------------------
       // led0..7  : DEVID register (expect 0xAD)
       // led8     : sign of raw_x_dbg (1 = negative, 0 = positive/zero)
       // led9     : sign of raw_y_dbg
       // led10    : sign of raw_z_dbg
       // led11    : init_done (ADXL362 configured, measurement mode entered)
       // led12    : devid_error (DEVID != 0xAD)
       // led13    : sample_seen (pulses stretched)
       // led14    : unused (0)
       // led15    : unused (0)
       // ------------------------------------------------------------------------
       assign {led7, led6, led5, led4, led3, led2, led1, led0} = devid_debug;
   
       assign led8  = raw_x_dbg[7];
       assign led9  = raw_y_dbg[7];
       assign led10 = raw_z_dbg[7];
       assign led11 = init_done;
       assign led12 = devid_error;
       assign led13 = sample_seen;
       assign led14 = 1'b0;
       assign led15 = 1'b0;



endmodule
