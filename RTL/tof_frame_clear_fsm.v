`timescale 1ns/1ps

// ============================================================================
// Module : tof_frame_clear_fsm
// Role   : Clear the entire 256x256 framebuffer once per sweep.
//
//   * IDLE → wait for start_clear.
//   * On start_clear, step through all 65,536 addresses and write 0.
//   * Asserts 'busy' while clearing; top-level arbitrates write port.
//
// Timing (at 100 MHz):
//   65,536 writes / 100e6 ≈ 0.655 ms per full clear.
// ============================================================================
module tof_frame_clear_fsm #(
    parameter integer ADDR_W = 16
)(
    input  wire              clk_sys,
    input  wire              rst_sys,
    input  wire              start_clear,  // 1-cycle pulse to begin clearing

    output reg               busy,         // 1 while clearing in progress
    output reg               wr_en,        // write enable to framebuffer
    output reg  [7:0]        wr_x,         // x coordinate (0..255)
    output reg  [7:0]        wr_y,         // y coordinate (0..255)
    output reg               wr_data       // pixel value (always 0 during clear)
);
    localparam [ADDR_W-1:0] MAX_ADDR = {ADDR_W{1'b1}}; // 16'hFFFF for ADDR_W=16

    // Simple 2-state FSM
    localparam [1:0]
        S_IDLE  = 2'd0,
        S_CLEAR = 2'd1;

    reg [1:0]        state;
    reg [ADDR_W-1:0] addr;

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            state   <= S_IDLE;
            addr    <= {ADDR_W{1'b0}};
            busy    <= 1'b0;
            wr_en   <= 1'b0;
            wr_x    <= 8'd0;
            wr_y    <= 8'd0;
            wr_data <= 1'b0;
        end else begin
            // Defaults
            wr_en   <= 1'b0;
            wr_data <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // IDLE: waiting for a clear request
                // -------------------------------------------------------------
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start_clear) begin
                        state <= S_CLEAR;
                        busy  <= 1'b1;
                        addr  <= {ADDR_W{1'b0}};
                    end
                end

                // -------------------------------------------------------------
                // CLEAR: walk through addresses 0..MAX_ADDR
                // -------------------------------------------------------------
                S_CLEAR: begin
                    busy    <= 1'b1;
                    wr_en   <= 1'b1;
                    wr_data <= 1'b0;         // clear pixel
                    wr_x    <= addr[7:0];    // x = low byte
                    wr_y    <= addr[15:8];   // y = high byte

                    if (addr == MAX_ADDR) begin
                        // Just wrote last pixel
                        state <= S_IDLE;
                        addr  <= {ADDR_W{1'b0}};
                    end else begin
                        addr <= addr + 1'b1;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
