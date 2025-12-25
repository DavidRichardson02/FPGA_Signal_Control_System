`timescale 1ns/1ps

// ============================================================================
// Module : tof_plot_bram_dp4
// Role   : 256x256 4-bit intensity framebuffer, dual-port with Port-A readback.
//
//   * Port A (clk_a): system domain. Synchronous read (q_a) + optional write.
//   * Port B (clk_b): pixel domain. Synchronous read (q_b).
//
// Addressing convention:
//   addr = {y[7:0], x[7:0]} → y as MSB, x as LSB.
//
// Notes:
//   - This is the BRAM primitive need for READ-MODIFY-WRITE (RMW) effects:
//       * decay sweep: I <- max(I - step, 0)
//       * accum writer: I <- min(I + HIT_INC, 15)
//   - Both decay and writer share Port A via top-level arbitration.
// ============================================================================
module tof_plot_bram_dp4 (
    // Port A (system domain): sync read + write
    input  wire        clk_a,
    input  wire        we_a,
    input  wire [7:0]  x_a,
    input  wire [7:0]  y_a,
    input  wire [3:0]  d_a,
    output reg  [3:0]  q_a,

    // Port B (pixel domain): sync read
    input  wire        clk_b,
    input  wire [7:0]  x_b,
    input  wire [7:0]  y_b,
    output reg  [3:0]  q_b
);
    localparam integer WIDTH  = 256;
    localparam integer HEIGHT = 256;
    localparam integer DEPTH  = WIDTH * HEIGHT;  // 65,536

    (* ram_style = "block" *) reg [3:0] mem [0:DEPTH-1];

    wire [15:0] addr_a = {y_a, x_a};
    wire [15:0] addr_b = {y_b, x_b};

    // Port A: synchronous read + optional write
    always @(posedge clk_a) begin
        q_a <= mem[addr_a];
        if (we_a) begin
            mem[addr_a] <= d_a;
        end
    end

    // Port B: synchronous read
    always @(posedge clk_b) begin
        q_b <= mem[addr_b];
    end

endmodule
