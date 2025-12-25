`timescale 1ns/1ps

// ============================================================================
// Module : tof_plot_bram_dp
// Role   : 256x256 bit-plane dual-port framebuffer for ToF plot.
//
//   * Port A (clk_a): write side in system (100 MHz) domain.
//   * Port B (clk_b): read side in pixel (25 MHz) domain.
//   * Each pixel is one bit (0=off, 1=on).
//
// Addressing convention:
//   addr = {y[7:0], x[7:0]} → y as MSB, x as LSB.
// ============================================================================
module tof_plot_bram_dp (
    // Write port (Port A) - system domain
    input  wire        clk_a,
    input  wire        we_a,
    input  wire [7:0]  x_a,
    input  wire [7:0]  y_a,
    input  wire        d_a,

    // Read port (Port B) - pixel domain
    input  wire        clk_b,
    input  wire [7:0]  x_b,
    input  wire [7:0]  y_b,
    output reg         q_b
);
    localparam integer WIDTH  = 256;
    localparam integer HEIGHT = 256;
    localparam integer DEPTH  = WIDTH * HEIGHT;  // 65,536

    // Synthesis attribute to encourage block RAM inference
    (* ram_style = "block" *) reg [0:0] mem [0:DEPTH-1];

    wire [15:0] addr_a = {y_a, x_a};  // 16-bit address for port A
    wire [15:0] addr_b = {y_b, x_b};  // 16-bit address for port B

    // Port A: write
    always @(posedge clk_a) begin
        if (we_a) begin
            mem[addr_a] <= d_a;
        end
    end

    // Port B: synchronous read
    always @(posedge clk_b) begin
        q_b <= mem[addr_b];
    end

endmodule
