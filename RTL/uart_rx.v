`timescale 1ns/1ps
// ============================================================================
// Module : uart_rx
// Role   : Minimal 8-N-1 UART receiver.
//          - Line idles HIGH.
//          - Detects START bit (falling edge).
//          - Samples in the *middle* of each bit using an integer divider.
//          - Assembles 8 data bits (LSB first) and asserts rx_vld for 1 clk.
//
// Parameters:
//   CLK_HZ : fabric clock frequency (Hz).
//   BAUD   : serial bit rate (bits per second).
//
// Assumptions:
//   - |baud error| from integer division is small (well under ±2%).
//   - rxi is connected to an external UART TX (e.g., FT2232 on Nexys).
//   - rst is synchronous, active-high.
//
// Interface:
//   clk     : system clock (e.g. 100 MHz).
//   rst     : synchronous reset, active-high.
//   rxi     : asynchronous serial RX input (idle = 1).
//   rx_byte : received byte (valid when rx_vld is 1).
//   rx_vld  : 1-cycle strobe indicating a new byte is ready.
// ============================================================================

module uart_rx #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 2_000_000
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rxi,        // async serial input (idle high)

    output reg  [7:0] rx_byte,    // assembled data byte
    output reg        rx_vld      // 1-cycle strobe when rx_byte is valid
);
    // ------------------------------------------------------------------------
    // Baud timing: integer divider
    // ------------------------------------------------------------------------
    // We reuse the same DIV concept as uart_tx:
    //   Tbit ≈ DIV * Tclk  where  DIV = floor(CLK_HZ / BAUD)
    localparam integer DIV      = (CLK_HZ / BAUD);
    localparam integer DIV_HALF = (DIV >> 1);   // approximate half-bit delay

    // Counter for timing between samples
    reg [$clog2(DIV):0] baud_cnt;

    // ------------------------------------------------------------------------
    // Synchronize rxi to clk domain
    // ------------------------------------------------------------------------
    reg rxi_ff1, rxi_ff2;
    wire rxi_sync = rxi_ff2;

    always @(posedge clk) begin
        rxi_ff1 <= rxi;
        rxi_ff2 <= rxi_ff1;
    end

    // ------------------------------------------------------------------------
    // RX state machine
    //   IDLE  : wait for start bit (rxi goes low).
    //   START : wait half a bit period and re-check start bit (mid-cell).
    //   DATA  : sample 8 data bits at full bit intervals.
    //   STOP  : sample stop bit, then present byte and assert rx_vld.
// ------------------------------------------------------------------------
    localparam [1:0]
        S_IDLE  = 2'd0,
        S_START = 2'd1,
        S_DATA  = 2'd2,
        S_STOP  = 2'd3;

    reg [1:0] state;
    reg [2:0] bit_idx;       // counts 0..7 data bits
    reg [7:0] shreg;         // shift register for incoming bits (LSB first)

    // ------------------------------------------------------------------------
    // Main RX logic
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            baud_cnt <= {($clog2(DIV)+1){1'b0}};
            bit_idx  <= 3'd0;
            shreg    <= 8'h00;
            rx_byte  <= 8'h00;
            rx_vld   <= 1'b0;
        end else begin
            // Default: de-assert rx_vld each cycle unless we finish a byte
            rx_vld <= 1'b0;

            case (state)
                // ------------------------------------------------------------
                // IDLE: waiting for start bit (line goes from 1 → 0)
                // ------------------------------------------------------------
                S_IDLE: begin
                    baud_cnt <= {($clog2(DIV)+1){1'b0}};
                    bit_idx  <= 3'd0;
                    if (rxi_sync == 1'b0) begin
                        // Detected potential start bit
                        state    <= S_START;
                        baud_cnt <= {($clog2(DIV)+1){1'b0}};
                    end
                end

                // ------------------------------------------------------------
                // START: wait half a bit, then confirm line still low
                //        (sampling in the middle of the start bit)
                // ------------------------------------------------------------
                S_START: begin
                    baud_cnt <= baud_cnt + 1'b1;
                    if (baud_cnt == (DIV_HALF - 1)) begin
                        // Mid-bit sample
                        if (rxi_sync == 1'b0) begin
                            // Valid start bit; arm for first data bit
                            baud_cnt <= {($clog2(DIV)+1){1'b0}};
                            bit_idx  <= 3'd0;
                            state    <= S_DATA;
                        end else begin
                            // False start: go back to idle
                            state <= S_IDLE;
                        end
                    end
                end

                // ------------------------------------------------------------
                // DATA: sample 8 data bits at full bit intervals
                //       LSB is received first.
                // ------------------------------------------------------------
                S_DATA: begin
                    baud_cnt <= baud_cnt + 1'b1;
                    if (baud_cnt == (DIV - 1)) begin
                        baud_cnt <= {($clog2(DIV)+1){1'b0}};

                        // Sample the current data bit
                        shreg[bit_idx] <= rxi_sync;

                        if (bit_idx == 3'd7) begin
                            // Collected 8 bits; next sample will be STOP bit
                            state   <= S_STOP;
                        end
                        bit_idx <= bit_idx + 3'd1;
                    end
                end

                // ------------------------------------------------------------
                // STOP: sample stop bit after one more bit period.
                //       If it's high, accept the byte.
                // ------------------------------------------------------------
                S_STOP: begin
                    baud_cnt <= baud_cnt + 1'b1;
                    if (baud_cnt == (DIV - 1)) begin
                        baud_cnt <= {($clog2(DIV)+1){1'b0}};

                        // We *expect* rxi_sync == 1 here (stop bit = mark).
                        // Even if it's not, we still latch the byte and then
                        // return to IDLE; a higher-level layer can detect errors.
                        rx_byte <= shreg;
                        rx_vld  <= 1'b1;

                        state   <= S_IDLE;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
