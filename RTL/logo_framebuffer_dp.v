`timescale 1ns/1ps
// ============================================================================
// Module : logo_framebuffer_dp
// Role   : 320x240 RGB444 (12-bit) true dual-port framebuffer.
//
//   - Port A (clk_a): write side (system domain).
//   - Port B (clk_b): read side (pixel domain).
//
// Notes:
//   - mem[] is initialized from an external hex file:
//         image320x240_rgb12.hex
//     which must contain DEPTH=76800 lines of 3-digit hex values (000–FFF),
//     in row-major order: y=0..239, x=0..319.
// ============================================================================
module logo_framebuffer_dp #(
    parameter integer WIDTH  = 320,
    parameter integer HEIGHT = 240,
    parameter integer AW     = 17                 // ceil(log2(320*240)) = 17
)(
    // Write port (system / UART / host side)
    input  wire              clk_a,
    input  wire              we_a,
    input  wire [AW-1:0]     addr_a,
    input  wire [11:0]       din_a,

    // Read port (pixel / VGA side)
    input  wire              clk_b,
    input  wire [AW-1:0]     addr_b,
    output reg  [11:0]       dout_b
);
    localparam integer DEPTH = WIDTH * HEIGHT;    // 76800 for 320x240

    // Encourage BRAM usage
    (* ram_style = "block" *)
    reg [11:0] mem [0:DEPTH-1];

    // ------------------------------------------------------------------------
    // Optional: BRAM initialization from hex file
    //   - File name must match the one you add as a design source and/or
    //     mark as a Memory Initialization File in Vivado.
    //   - Path is relative to the synthesis/simulation working directory.
    // ------------------------------------------------------------------------
    initial begin
        // Each line: a 12-bit RGB444 word, written as 3-digit hex (000–FFF)
        // in row-major order: y=0..239, x=0..319.
        $readmemh("image320x240_rgb12.hex", mem);
    end

    // Port A: synchronous write (system domain)
    always @(posedge clk_a) begin
        if (we_a) begin
            mem[addr_a] <= din_a;
        end
    end

    // Port B: synchronous read (pixel domain)
    always @(posedge clk_b) begin
        dout_b <= mem[addr_b];
    end

endmodule
