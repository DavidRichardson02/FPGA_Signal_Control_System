`timescale 1ns/1ps
// ============================================================================
// Module : uart_tx
// Role   : Transmit bytes over an asynchronous serial line (UART) at a fixed
//          baud rate, using an internal clock divider.
//          - Frame: 1 start bit, 8 data bits, 1 stop bit (8-N-1)
//          - Idle line level: logic '1'
//          - Handshake: ready/valid (data accepted when valid && ready)
// Clock  : System clock (e.g., 100 MHz)
// Baud   : Default 2,000,000 baud when CLK_FREQ_HZ=100_000_000
// ============================================================================
module uart_tx #(
    parameter integer CLK_FREQ_HZ = 100_000_000,   // system clock
    parameter integer BAUD_HZ     = 2_000_000      // target baud
) (
    input  wire        clk,
    input  wire        rst,        // synchronous, active-high

    // Handshake interface
    input  wire [7:0]  data_in,    // byte to transmit
    input  wire        data_valid, // pulse/high when data_in is valid
    output wire        data_ready, // high when module can accept a byte

    // Status
    output reg         busy,       // high while a frame is in progress

    // UART output
    output reg         tx          // serial line (idle = 1)
);

    // ------------------------------------------------------------------------
    // Baud-rate generator
    // ------------------------------------------------------------------------
    //
    // Number of clk cycles per UART bit.
    // For CLK_FREQ_HZ=100 MHz and BAUD_HZ=2,000,000:
    //   BAUD_DIV = 100_000_000 / 2_000_000 = 50 (exact)
    //
    localparam integer BAUD_DIV = CLK_FREQ_HZ / BAUD_HZ;

    // Optional **sanity check** at elaboration/synthesis time
    initial begin
        if (CLK_FREQ_HZ % BAUD_HZ != 0) begin
            $display("WARNING: uart_tx: CLK_FREQ_HZ / BAUD_HZ is not an integer. Actual baud will be %0d", CLK_FREQ_HZ / BAUD_DIV);
        end
    end

    // Counter generates a single-cycle tick once per bit period
    reg [$clog2(BAUD_DIV)-1:0] baud_cnt = 0;
    wire                       baud_tick = (baud_cnt == 0);

    always @(posedge clk) begin
        if (rst) begin
            baud_cnt <= 0;
        end else if (busy) begin
            // Only tick while actively transmitting; otherwise hold at 0
            if (baud_cnt == BAUD_DIV - 1)
                baud_cnt <= 0;
            else
                baud_cnt <= baud_cnt + 1;
        end else begin
            baud_cnt <= 0;
        end
    end

    // ------------------------------------------------------------------------
    // TX state machine
    // ------------------------------------------------------------------------
    //
    // Frame structure:
    //   bit 0: start (0)
    //   bit 1-8: data bits (LSB first)
    //   bit 9: stop (1)
    //
    localparam integer TOTAL_BITS = 10; // 1 start + 8 data + 1 stop

    reg [3:0]  bit_index = 0;    // counts 0..9
    reg [7:0]  shift_reg = 8'h00;

    // Ready when not busy and not currently loading
    assign data_ready = ~busy;

    always @(posedge clk) begin
        if (rst) begin
            busy      <= 1'b0;
            tx        <= 1'b1;   // idle high
            bit_index <= 4'd0;
            shift_reg <= 8'h00;
        end else begin
            // ----------------------------------------------------------------
            // Accept a new byte when the line is idle and the user asserts
            // data_valid. We immediately schedule a start bit.
            // ----------------------------------------------------------------
            if (~busy && data_valid) begin
                busy      <= 1'b1;
                shift_reg <= data_in;
                bit_index <= 4'd0;   // start with start bit on next tick
                tx        <= 1'b0;   // drive start bit immediately (bit 0)
            end
            // ----------------------------------------------------------------
            // While busy, advance one bit every baud_tick.
            // ----------------------------------------------------------------
            else if (busy && baud_tick) begin
                bit_index <= bit_index + 1;

                case (bit_index)
                    // bit_index == 0: we already output start bit (0)
                    4'd0: begin
                        // next: first data bit (LSB)
                        tx <= shift_reg[0];
                    end

                    4'd1: tx <= shift_reg[1];
                    4'd2: tx <= shift_reg[2];
                    4'd3: tx <= shift_reg[3];
                    4'd4: tx <= shift_reg[4];
                    4'd5: tx <= shift_reg[5];
                    4'd6: tx <= shift_reg[6];
                    4'd7: tx <= shift_reg[7];

                    // After the 8th data bit, send the stop bit
                    4'd8: tx <= 1'b1; // stop bit

                    // After stop bit, return to idle
                    4'd9: begin
                        tx        <= 1'b1;   // idle
                        busy      <= 1'b0;
                        bit_index <= 4'd0;
                    end

                    default: begin
                        tx   <= 1'b1;
                        busy <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
