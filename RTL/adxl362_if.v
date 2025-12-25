`timescale 1ns/1ps
// =============================================================================
// MODULE: adxl362_if
// =============================================================================
// PURPOSE
//   High-level ADXL362 accelerometer interface built around a byte-oriented SPI
//   master. This module is "register-level": it does not bit-bang SPI directly,
//   but instead sequences the ADXL362 command/address/data bytes by repeatedly
//   launching 8-bit SPI transactions.
//
// HIGH-LEVEL BEHAVIOR (BOOT SEQUENCE)
//   1) Wait for sensor power-up / internal startup time.
//   2) Read DEVID_AD register and compare against expected value (0xAD).
//   3) Write FILTER_CTL configuration (range, ODR, bandwidth, etc.).
//   4) Write POWER_CTL configuration (measurement mode enable).
//   5) Periodically read XDATA, YDATA, ZDATA (8-bit signed each).
//
// IMPORTANT MODELING IDEA: "BYTE TRANSACTION SPI"
//   The SPI master used here exposes a start/done handshake per byte:
//
//     - We present spi_tx_data (the byte we want to transmit).
//     - We pulse spi_start for 1 clk cycle (launches an 8-bit shift).
//     - The SPI master asserts spi_busy while shifting the 8 bits.
//     - When complete, it pulses/raises spi_done, and spi_rx_data is valid.
//
//   Because each byte is its own mini-transaction, the ADXL command framing:
//     [CMD][ADDR][DATA/DUMMY][DATA/DUMMY]...
//   is implemented by an FSM that iterates through states:
//     *_SEND  (issue spi_start with desired spi_tx_data)
//     *_WAIT  (hold CS low, wait for spi_done)
//
// CHIP-SELECT (CSN) HANDLING
//   ADXL362 expects CSN low for the entire multi-byte frame, so we keep CSN low
//   across the sequence of SEND/WAIT states and release CSN high only between
//   frames. To avoid multi-driver issues, CSN is registered via csn_reg <= csn_next.
//
// OUTPUTS
//   accel_x/y/z: signed Q7-scaled values for HUD usage (±64 ≈ full-scale).
//   sample_valid: pulses high for one clk cycle when a new XYZ sample is committed.
//   init_done: asserted once configuration writes complete.
//   devid_error: sticky high if the device ID read does not match EXPECT_DEVID.
//
// NOTES ON SCALING
//   Pipeline treats the 8-bit XDATA/YDATA/ZDATA registers as signed samples.
//   Then map them into a Q7 range via mg conversion and linear scaling.
//   (The constants here reflect chosen assumptions; verify against
//    configuration bits in FILTER_CTL and the datasheet if change range.)
// =============================================================================

module adxl362_if #(
    // Number of clk cycles to delay after reset before attempting first SPI access.
    // At 100 MHz: 1_000_000_000 cycles = 100 ms.
    parameter integer PWRUP_WAIT_CYCLES   = 32'd1_000_000_000,

    // Sample interval between XYZ burst reads.
    // At 100 MHz: 1_000_000 cycles = 10 ms (100 Hz update).
    parameter integer SAMPLE_WAIT_CYCLES  = 32'd1_000_000
)(
    // -----------------------
    // Clocking / Reset
    // -----------------------
    input  wire        clk,      // system clock domain (e.g., 100 MHz)
    input  wire        rst,      // synchronous reset, active-high

    // -----------------------
    // SPI Physical Signals
    // -----------------------
    input  wire        ACL_MISO,  // Master-In Slave-Out from ADXL362
    output wire        ACL_MOSI,  // Master-Out Slave-In to ADXL362
    output wire        ACL_SCLK,  // SPI clock to ADXL362
    output wire        ACL_CSN,   // Chip select (active-low) to ADXL362

    // -----------------------
    // Data Outputs
    // -----------------------
    output reg  signed [7:0] accel_x,     // Q7-like scaled X output
    output reg  signed [7:0] accel_y,     // Q7-like scaled Y output
    output reg  signed [7:0] accel_z,     // Q7-like scaled Z output
    
    output reg  signed [7:0] raw_x,       // NEW: raw XDATA sample (signed 8-bit)
    output reg  signed [7:0] raw_y,       // NEW: raw YDATA sample (signed 8-bit)
    output reg  signed [7:0] raw_z,       // NEW: raw ZDATA sample (signed 8-bit)
    output reg               sample_valid,// 1-cycle pulse when accel_* updated(new XYZ committed)

    // -----------------------
    // Status Outputs
    // -----------------------
    output reg               init_done,   // high once configuration completed
    output reg               devid_error, // sticky error on DEVID mismatch

    // -----------------------
    // Debug
    // -----------------------
    output reg  [7:0]        devid_debug,       // last captured DEVID byte
    output reg               devid_debug_valid, // indicates DEVID captured
    output reg  signed [7:0] raw_x_dbg,         // raw XDATA byte
    output reg  signed [7:0] raw_y_dbg,         // raw YDATA byte
    output reg  signed [7:0] raw_z_dbg          // raw ZDATA byte
);

    // =========================================================================
    // CSN REGISTERING (avoids multi-driver + gives clean timing)
    // =========================================================================
    // csn_next is purely combinational "what CSN should be in the next cycle"
    // csn_reg is the actual registered output that drives the pin.
    reg csn_reg, csn_next;
    assign ACL_CSN = csn_reg; // drive pin from registered value (glitch-free)

    // =========================================================================
    // ADXL362 COMMAND / REGISTER CONSTANTS
    // =========================================================================
    // SPI command bytes (per ADXL362 protocol)
    localparam [7:0] CMD_WRITE      = 8'h0A; // "write register(s)"
    localparam [7:0] CMD_READ       = 8'h0B; // "read register(s)"

    // Register map addresses
    localparam [7:0] REG_DEVID_AD   = 8'h00; // Device ID register (expects 0xAD)
    localparam [7:0] REG_FILTER_CTL = 8'h2C; // Range/ODR/bandwidth configuration
    localparam [7:0] REG_POWER_CTL  = 8'h2D; // Measurement mode control
    localparam [7:0] REG_XDATA      = 8'h08; // XDATA (then YDATA, then ZDATA)

    // Expected DEVID constant (from datasheet)
    localparam [7:0] EXPECT_DEVID   = 8'hAD;

    // Configuration writes (these are chosen bitfields)
    localparam [7:0] FILTER_CTL_CFG = 8'h93; // example config (verify bit meanings)
    localparam [7:0] POWER_CTL_CFG  = 8'h02; // measurement mode enable

    // =========================================================================
    // BYTE-ORIENTED SPI MASTER INTERFACE
    // =========================================================================
    // spi_tx_data: byte we want to transmit next
    // spi_start:   1-cycle pulse to begin shifting spi_tx_data
    // spi_busy:    asserted while SPI engine is shifting bits
    // spi_done:    asserted/pulsed when byte transaction completes
    // spi_rx_data: byte shifted in during the transaction (valid at spi_done)
    wire [7:0] spi_rx_data;
    reg  [7:0] spi_tx_data;
    reg        spi_start;
    wire       spi_busy;
    wire       spi_done;

    // SPI master instance: generates SCLK/MOSI and samples MISO
    spi_master_simple #(
        .CLKS_PER_HALF_BIT(50) // clk/(2*50) = 1 MHz SCLK when clk=100 MHz
    ) u_spi_master (
        .clk     (clk),
        .rst     (rst),
        .start   (spi_start),
        .tx_data (spi_tx_data),
        .rx_data (spi_rx_data),
        .busy    (spi_busy),
        .done    (spi_done),
        .miso    (ACL_MISO),
        .mosi    (ACL_MOSI),
        .sclk    (ACL_SCLK)
    );

    // =========================================================================
    // FSM STATE ENCODING
    // =========================================================================
    // We use explicit SEND/WAIT pairs because the SPI master is byte-based:
    //   SEND: drive spi_tx_data and pulse spi_start
    //   WAIT: hold CSN low and wait until spi_done arrives
    localparam [4:0]
        ST_PWRUP_WAIT          = 5'd0,

        // DEVID read transaction: [READ][ADDR][DUMMY->RX]
        ST_DEVID_CMD_SEND      = 5'd1,
        ST_DEVID_CMD_WAIT      = 5'd2,
        ST_DEVID_ADDR_SEND     = 5'd3,
        ST_DEVID_ADDR_WAIT     = 5'd4,
        ST_DEVID_DUMMY_SEND    = 5'd5,
        ST_DEVID_DUMMY_WAIT    = 5'd6,
        ST_DEVID_CHECK         = 5'd7,

        // FILTER_CTL write: [WRITE][ADDR][DATA]
        ST_FILT_CMD_SEND       = 5'd8,
        ST_FILT_CMD_WAIT       = 5'd9,
        ST_FILT_ADDR_SEND      = 5'd10,
        ST_FILT_ADDR_WAIT      = 5'd11,
        ST_FILT_DATA_SEND      = 5'd12,
        ST_FILT_DATA_WAIT      = 5'd13,

        // POWER_CTL write: [WRITE][ADDR][DATA]
        ST_PWR_CMD_SEND        = 5'd14,
        ST_PWR_CMD_WAIT        = 5'd15,
        ST_PWR_ADDR_SEND       = 5'd16,
        ST_PWR_ADDR_WAIT       = 5'd17,
        ST_PWR_DATA_SEND       = 5'd18,
        ST_PWR_DATA_WAIT       = 5'd19,

        // Wait between samples (no SPI activity)
        ST_WAIT_SAMPLE         = 5'd21,

        // XYZ burst read: [READ][XDATA_ADDR][DUMMY->X][DUMMY->Y][DUMMY->Z]
        ST_RD_CMD_SEND         = 5'd22,
        ST_RD_CMD_WAIT         = 5'd23,
        ST_RD_ADDR_SEND        = 5'd24,
        ST_RD_ADDR_WAIT        = 5'd25,
        ST_RD_X_SEND           = 5'd26,
        ST_RD_X_WAIT           = 5'd27,
        ST_RD_Y_SEND           = 5'd28,
        ST_RD_Y_WAIT           = 5'd29,
        ST_RD_Z_SEND           = 5'd30,
        ST_RD_Z_WAIT           = 5'd31;

    // Current state (registered) and next state (combinational)
    reg [4:0] state, state_next;

    // =========================================================================
    // TIMERS / COUNTERS
    // =========================================================================
    // pwrup_ctr: counts clock cycles after reset to satisfy startup delay.
    // sample_ctr: counts clock cycles between consecutive sample reads.
    reg [31:0] pwrup_ctr;
    reg [31:0] sample_ctr;

    // =========================================================================
    // TEMP STORAGE FOR XYZ BYTES DURING A BURST READ
    // =========================================================================
    // We capture X and Y in x_tmp/y_tmp as their bytes arrive.
    // Z arrives last; when Z arrives we commit all three outputs atomically.
    reg signed [7:0] x_tmp, y_tmp;

    // =========================================================================
    // SCALING FUNCTIONS (raw 8-bit sample -> mg -> Q7)
    // =========================================================================
    // These constants represent chosen scaling model.
    // If change range/ODR or use 12-bit registers instead, revisit them.
    localparam signed [15:0] ADXL_MG_PER_LSB = 16'sd16;   // assumed mg per LSB
    localparam signed [15:0] FS_MG           = 16'sd2000; // ±2g full scale in mg
    localparam signed [7:0]  Q7_FS           = 8'sd64;    // map ±FS_MG -> ±64

    // Convert signed 8-bit count to signed mg (16-bit) by sign-extending first.
    function automatic signed [15:0] raw8_to_mg;
        input signed [7:0] raw8;
        reg   signed [15:0] ext;
    begin
        // Sign-extend: replicate sign bit raw8[7] into upper bits.
        ext        = {{8{raw8[7]}}, raw8};

        // Multiply by mg/LSB scale factor.
        raw8_to_mg = ext * ADXL_MG_PER_LSB;
    end
    endfunction

    // Map mg -> Q7 range, saturating to prevent overflow / wrap.
    function automatic signed [7:0] scale_to_q7;
        input signed [15:0] mg_val;
        reg   signed [15:0] mg_clamped;
        reg   signed [31:0] prod;
    begin
        // Saturation clamp to ±FS_MG
        if (mg_val >  FS_MG)      mg_clamped =  FS_MG;
        else if (mg_val < -FS_MG) mg_clamped = -FS_MG;
        else                      mg_clamped =  mg_val;

        // Linear map: q7 = mg * (Q7_FS / FS_MG)
        // Use a wider intermediate to keep precision during multiply.
        prod        = mg_clamped * Q7_FS;
        scale_to_q7 = prod / FS_MG;
    end
    endfunction

    // =========================================================================
    // SEQUENTIAL PROCESS: STATE REGISTER + OUTPUT REGISTERS
    // =========================================================================
    // This always block:
    //   - updates state <= state_next
    //   - registers CSN output
    //   - updates counters
    //   - captures incoming SPI bytes at the correct WAIT states
    //   - pulses sample_valid when a full XYZ set is committed
    always @(posedge clk) begin
        if (rst) begin
            // -----------------------
            // State + counters reset
            // -----------------------
            state              <= ST_PWRUP_WAIT;
            pwrup_ctr          <= 32'd0;
            sample_ctr         <= 32'd0;

            // -----------------------
            // Output registers reset
            // -----------------------
            accel_x            <= 8'sd0;
            accel_y            <= 8'sd0;
            accel_z            <= 8'sd0;
            raw_x <= 8'sd0;
            raw_y <= 8'sd0;
            raw_z <= 8'sd0;

            // temp storage reset
            x_tmp              <= 8'sd0;
            y_tmp              <= 8'sd0;

            // strobes / flags reset
            sample_valid       <= 1'b0;
            init_done          <= 1'b0;
            devid_error        <= 1'b0;

            // CSN idles high (no transaction)
            csn_reg            <= 1'b1;

            // debug outputs reset
            devid_debug        <= 8'h00;
            devid_debug_valid  <= 1'b0;
            raw_x_dbg          <= 8'sd0;
            raw_y_dbg          <= 8'sd0;
            raw_z_dbg          <= 8'sd0;

        end else begin
            // ----------------------------------------------------------------
            // Update registered state and CSN from combinational decisions
            // ----------------------------------------------------------------
            state   <= state_next;
            csn_reg <= csn_next;

            // sample_valid is a pulse: default low unless explicitly asserted.
            sample_valid <= 1'b0;

            // ----------------------------------------------------------------
            // State-dependent sequential actions
            // ----------------------------------------------------------------
            case (state)

                // ------------------------------------------------------------
                // Power-up wait: count until reaching the configured delay.
                // ------------------------------------------------------------
                ST_PWRUP_WAIT: begin
                    if (pwrup_ctr < PWRUP_WAIT_CYCLES)
                        pwrup_ctr <= pwrup_ctr + 1'b1;
                end

                // ------------------------------------------------------------
                // Inter-sample wait: count until reaching the interval.
                // ------------------------------------------------------------
                ST_WAIT_SAMPLE: begin
                    if (sample_ctr < SAMPLE_WAIT_CYCLES)
                        sample_ctr <= sample_ctr + 1'b1;
                end

                // ------------------------------------------------------------
                // DEVID read capture:
                // On the dummy byte, the returned rx byte is the register data.
                // ------------------------------------------------------------
                ST_DEVID_DUMMY_WAIT: begin
                    if (spi_done) begin
                        devid_debug       <= spi_rx_data; // store for debug
                        devid_debug_valid <= 1'b1;         // indicates captured

                        // sticky error if mismatched
                        if (spi_rx_data != EXPECT_DEVID)
                            devid_error <= 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // POWER_CTL write completion:
                // When the DATA byte transaction finishes, configuration is done.
                // ------------------------------------------------------------
                ST_PWR_DATA_WAIT: begin
                    if (spi_done) begin
                        init_done <= 1'b1;
                    end
                end

                // ------------------------------------------------------------
                // XYZ burst read capture:
                // X arrives at end of X dummy transaction; store in x_tmp.
                // ------------------------------------------------------------
                ST_RD_X_WAIT: begin
                    if (spi_done)
                        x_tmp <= spi_rx_data;
                end

                // Y arrives at end of Y dummy transaction; store in y_tmp.
                ST_RD_Y_WAIT: begin
                    if (spi_done)
                        y_tmp <= spi_rx_data;
                end

                // Z arrives last; when Z arrives, commit X/Y/Z to outputs together.
                ST_RD_Z_WAIT: begin
                    if (spi_done) begin
                        // -------------------------
                        // 1) Commit RAW (atomic)
                        // -------------------------
                        raw_x <= x_tmp;
                        raw_y <= y_tmp;
                        raw_z <= spi_rx_data; // Z just arrived
                
                        // Optional: keep debug taps aligned
                        raw_x_dbg <= x_tmp;
                        raw_y_dbg <= y_tmp;
                        raw_z_dbg <= spi_rx_data;
                
                        // -------------------------
                        // 2) Commit SCALED Q7 (atomic)
                        // -------------------------
                        accel_x <= scale_to_q7(raw8_to_mg(x_tmp));
                        accel_y <= scale_to_q7(raw8_to_mg(y_tmp));
                        accel_z <= scale_to_q7(raw8_to_mg(spi_rx_data));
                
                        // -------------------------
                        // 3) Pulse valid (means BOTH raw and q7 updated)
                        // -------------------------
                        sample_valid <= 1'b1;
                
                        sample_ctr <= 32'd0;
                    end
                end


                default: begin
                    // Intentionally empty: most states are "pure sequencing"
                    // (their actions occur in combinational FSM outputs).
                end
            endcase
        end
    end

    // =========================================================================
    // COMBINATIONAL PROCESS: NEXT STATE + SPI CONTROL + CSN CONTROL
    // =========================================================================
    // This always block decides:
    //   - which state to go to next (state_next)
    //   - whether to launch an SPI transaction (spi_start/spi_tx_data)
    //   - whether CSN should be held low during the current state (csn_next)
    //
    // IMPORTANT:
    //   spi_start must only pulse for 1 clk cycle.
    //   We accomplish that by defaulting spi_start=0 and only setting it in *_SEND.
    always @* begin
        // -----------------------
        // Default assignments
        // -----------------------
        state_next  = state;  // hold state unless explicitly changed
        spi_tx_data = 8'h00;  // default "dummy byte"
        spi_start   = 1'b0;   // default: do not start SPI
        csn_next    = 1'b1;   // default: CSN high (no transaction)

        case (state)

            // ------------------------------------------------------------
            // POWER-UP WAIT -> begin DEVID read when counter expires
            // ------------------------------------------------------------
            ST_PWRUP_WAIT: begin
                if (pwrup_ctr >= PWRUP_WAIT_CYCLES)
                    state_next = ST_DEVID_CMD_SEND;
            end

            // ============================================================
            // DEVID READ: frame = [READ][ADDR][DUMMY->RX]
            // ============================================================

            // SEND: drive READ command byte (0x0B)
            ST_DEVID_CMD_SEND: begin
                csn_next = 1'b0; // CSN low begins the multi-byte frame
                if (!spi_busy) begin
                    spi_tx_data = CMD_READ;
                    spi_start   = 1'b1;
                    state_next  = ST_DEVID_CMD_WAIT;
                end
            end

            // WAIT: keep CSN low until byte finishes shifting
            ST_DEVID_CMD_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_DEVID_ADDR_SEND;
            end

            // SEND: address byte (which register to read)
            ST_DEVID_ADDR_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = REG_DEVID_AD;
                    spi_start   = 1'b1;
                    state_next  = ST_DEVID_ADDR_WAIT;
                end
            end

            // WAIT for address byte transaction to finish
            ST_DEVID_ADDR_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_DEVID_DUMMY_SEND;
            end

            // SEND: dummy byte; the returned rx byte is the DEVID register value
            ST_DEVID_DUMMY_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = 8'h00;
                    spi_start   = 1'b1;
                    state_next  = ST_DEVID_DUMMY_WAIT;
                end
            end

            // WAIT: capture happens in sequential block at spi_done
            ST_DEVID_DUMMY_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_DEVID_CHECK;
            end

            // End of DEVID frame; CSN returns high by default here.
            ST_DEVID_CHECK: begin
                // Proceed to FILTER_CTL configuration regardless;
                // devid_error will latch if mismatch is detected.
                state_next = ST_FILT_CMD_SEND;
            end

            // ============================================================
            // FILTER_CTL WRITE: frame = [WRITE][ADDR][DATA]
            // ============================================================

            ST_FILT_CMD_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = CMD_WRITE;
                    spi_start   = 1'b1;
                    state_next  = ST_FILT_CMD_WAIT;
                end
            end

            ST_FILT_CMD_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_FILT_ADDR_SEND;
            end

            ST_FILT_ADDR_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = REG_FILTER_CTL;
                    spi_start   = 1'b1;
                    state_next  = ST_FILT_ADDR_WAIT;
                end
            end

            ST_FILT_ADDR_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_FILT_DATA_SEND;
            end

            ST_FILT_DATA_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = FILTER_CTL_CFG;
                    spi_start   = 1'b1;
                    state_next  = ST_FILT_DATA_WAIT;
                end
            end

            ST_FILT_DATA_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_PWR_CMD_SEND;
            end

            // ============================================================
            // POWER_CTL WRITE: frame = [WRITE][ADDR][DATA]
            // ============================================================

            ST_PWR_CMD_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = CMD_WRITE;
                    spi_start   = 1'b1;
                    state_next  = ST_PWR_CMD_WAIT;
                end
            end

            ST_PWR_CMD_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_PWR_ADDR_SEND;
            end

            ST_PWR_ADDR_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = REG_POWER_CTL;
                    spi_start   = 1'b1;
                    state_next  = ST_PWR_ADDR_WAIT;
                end
            end

            ST_PWR_ADDR_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_PWR_DATA_SEND;
            end

            ST_PWR_DATA_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = POWER_CTL_CFG;
                    spi_start   = 1'b1;
                    state_next  = ST_PWR_DATA_WAIT;
                end
            end

            ST_PWR_DATA_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_WAIT_SAMPLE;
            end

            // ============================================================
            // SAMPLE LOOP: wait -> read burst -> wait -> ...
            // ============================================================

            ST_WAIT_SAMPLE: begin
                // Pure timer state: no SPI traffic.
                if (sample_ctr >= SAMPLE_WAIT_CYCLES)
                    state_next = ST_RD_CMD_SEND;
            end

            // ============================================================
            // XYZ READ BURST: frame = [READ][XDATA_ADDR][DUMMY->X][DUMMY->Y][DUMMY->Z]
            // ============================================================

            ST_RD_CMD_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = CMD_READ;
                    spi_start   = 1'b1;
                    state_next  = ST_RD_CMD_WAIT;
                end
            end

            ST_RD_CMD_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_RD_ADDR_SEND;
            end

            ST_RD_ADDR_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = REG_XDATA; // start at XDATA; device auto-increments
                    spi_start   = 1'b1;
                    state_next  = ST_RD_ADDR_WAIT;
                end
            end

            ST_RD_ADDR_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_RD_X_SEND;
            end

            // Dummy byte clocks out XDATA
            ST_RD_X_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = 8'h00;
                    spi_start   = 1'b1;
                    state_next  = ST_RD_X_WAIT;
                end
            end

            ST_RD_X_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_RD_Y_SEND;
            end

            // Dummy byte clocks out YDATA
            ST_RD_Y_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = 8'h00;
                    spi_start   = 1'b1;
                    state_next  = ST_RD_Y_WAIT;
                end
            end

            ST_RD_Y_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_RD_Z_SEND;
            end

            // Dummy byte clocks out ZDATA
            ST_RD_Z_SEND: begin
                csn_next = 1'b0;
                if (!spi_busy) begin
                    spi_tx_data = 8'h00;
                    spi_start   = 1'b1;
                    state_next  = ST_RD_Z_WAIT;
                end
            end

            // End of burst; Z capture occurs in sequential on spi_done.
            ST_RD_Z_WAIT: begin
                csn_next = 1'b0;
                if (spi_done)
                    state_next = ST_WAIT_SAMPLE;
            end

            // ------------------------------------------------------------
            // Safety default
            // ------------------------------------------------------------
            default: begin
                state_next = ST_PWRUP_WAIT;
            end

        endcase
    end

endmodule
