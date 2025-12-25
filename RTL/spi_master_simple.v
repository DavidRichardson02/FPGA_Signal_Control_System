`timescale 1ns/1ps
// =============================================================================
// MODULE: spi_master_simple
// =============================================================================
// PURPOSE
//   Minimal single-byte SPI master with a start/busy/done handshake.
//
// INTERFACE CONTRACT (VERY IMPORTANT)
//   - One transaction = exactly 8 SCLK cycles (16 half-cycles).
//   - Assert 'start' for >=1 clk cycle when busy==0 to launch a byte.
//   - While busy==1, sclk toggles at a rate set by CLKS_PER_HALF_BIT.
//   - 'done' pulses high for 1 clk cycle when the byte finishes.
//   - 'rx_data' is valid in the same cycle as 'done' (registered).
//
// SPI MODE
//   This implementation is SPI Mode 0 (CPOL=0, CPHA=0):
//     * SCLK idles LOW when idle (CPOL=0)
//     * MOSI changes on the FALLING edge of SCLK
//     * MISO is sampled on the RISING edge of SCLK
//
// TIMING MODEL (INTERNAL)
//   We generate a "half-bit tick" every CLKS_PER_HALF_BIT cycles.
//   On each tick, we toggle SCLK.
//     - If we toggled LOW->HIGH (rising edge): sample MISO, possibly finish.
//     - If we toggled HIGH->LOW (falling edge): update MOSI for next bit.
// =============================================================================

module spi_master_simple #(
    parameter integer CLKS_PER_HALF_BIT = 50
)(
    input  wire       clk,
    input  wire       rst,

    input  wire       start,     // launch 1 byte when busy==0
    input  wire [7:0] tx_data,   // byte to transmit (MSB first)
    output reg  [7:0] rx_data,   // received byte (MSB first)
    output reg        busy,      // high during active transfer
    output reg        done,      // 1-clk pulse when transfer completes

    input  wire       miso,      // serial input from slave
    output wire       mosi,      // serial output to slave
    output wire       sclk       // serial clock
);

    // -------------------------------------------------------------------------
    // Registered outputs to the pins (important: makes edges deterministic)
    // -------------------------------------------------------------------------
    reg sclk_reg;   // internal registered SCLK
    reg mosi_reg;   // internal registered MOSI
    assign sclk = sclk_reg;
    assign mosi = mosi_reg;

    // -------------------------------------------------------------------------
    // Internal state for the current byte transfer
    // -------------------------------------------------------------------------
    reg [7:0] tx_latch;   // latched copy of tx_data for this transaction
    reg [7:0] rx_shift;   // accumulates sampled MISO bits

    reg [2:0] bit_index;  // which bit we are currently transferring (7 down to 0)

    reg [15:0] clk_count; // divides clk down to half-bit timing

    // -------------------------------------------------------------------------
    // Sequential logic
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            // Reset to idle, mode-0 idle clock
            busy      <= 1'b0;
            done      <= 1'b0;

            sclk_reg  <= 1'b0;   // CPOL=0 idle low
            mosi_reg  <= 1'b0;

            tx_latch  <= 8'h00;
            rx_shift  <= 8'h00;
            rx_data   <= 8'h00;

            bit_index <= 3'd0;
            clk_count <= 16'd0;

        end else begin
            // done is a pulse: default low each cycle unless we finish a byte
            done <= 1'b0;

            // =================================================================
            // IDLE STATE: not busy
            // =================================================================
            if (!busy) begin
                // Ensure outputs are in the "idle" SPI mode-0 configuration
                sclk_reg  <= 1'b0;
                clk_count <= 16'd0;

                // Start a new transaction when commanded
                if (start) begin
                    // Latch the byte so it cannot change mid-transfer
                    tx_latch  <= tx_data;

                    // Clear receiver accumulation
                    rx_shift  <= 8'h00;

                    // We transmit MSB first => start at bit 7
                    bit_index <= 3'd7;

                    // In mode 0, MOSI must be valid before first rising edge.
                    // So we drive MSB immediately upon start.
                    mosi_reg  <= tx_data[7];

                    // Enter busy state
                    busy      <= 1'b1;
                end

            end else begin
                // =================================================================
                // ACTIVE TRANSFER STATE: busy==1
                // =================================================================

                // Clock divider: wait CLKS_PER_HALF_BIT cycles, then toggle SCLK.
                if (clk_count == (CLKS_PER_HALF_BIT - 1)) begin
                    clk_count <= 16'd0;

                    // Toggle SCLK each half-bit tick
                    sclk_reg  <= ~sclk_reg;

                    // ------------------------------
                    // EDGE SEMANTICS (MODE 0)
                    // ------------------------------
                    // IMPORTANT subtlety:
                    // sclk_reg is the "old" value in this always block.
                    // When sclk_reg==0, we're about to create a rising edge (0->1).
                    // When sclk_reg==1, we're about to create a falling edge (1->0).

                    if (sclk_reg == 1'b0) begin
                        // =========================================================
                        // RISING EDGE: sample MISO for the current bit_index
                        // =========================================================
                        // Sample the slave output on the active edge.
                        rx_shift[bit_index] <= miso;

                        // If this was the last bit, complete the transaction.
                        if (bit_index == 3'd0) begin
                            busy     <= 1'b0;
                            done     <= 1'b1;

                            // Return SCLK to idle low immediately after finishing
                            sclk_reg <= 1'b0;

                            // Commit full received byte.
                            // rx_shift[0] has not yet updated (nonblocking), so inject miso directly.
                            rx_data  <= {rx_shift[7:1], miso};

                        end else begin
                            // Move to next bit (toward LSB) for the next cycle
                            bit_index <= bit_index - 3'd1;
                        end

                    end else begin
                        // =========================================================
                        // FALLING EDGE: update MOSI for the NEXT rising edge
                        // =========================================================
                        // After sampling bit_index on the rising edge, we decremented
                        // bit_index (unless we just finished). Therefore, the current
                        // value of bit_index now refers to the next bit to be sampled.
                        //
                        // We must present that bit on MOSI so it is stable before the
                        // next rising edge.
                        mosi_reg <= tx_latch[bit_index];
                    end

                end else begin
                    // Not time to toggle SCLK yet; keep counting.
                    clk_count <= clk_count + 16'd1;
                end
            end
        end
    end

endmodule
