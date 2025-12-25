`timescale 1ns/1ps

// ============================================================================
// Module : image_rom_320x240_rgb12
// Role   : 320x240 RGB 4:4:4 image ROM in block RAM.
//          - Data initialized from image320x240_rgb12.hex
//          - Address = y*320 + x  (row-major).
// ============================================================================
module image_rom_320x240_rgb12 (
    input  wire        clk,
    input  wire [8:0]  x,    // 0..319
    input  wire [7:0]  y,    // 0..239
    output reg  [11:0] rgb   // {R[11:8], G[7:4], B[3:0]}
);
    localparam integer WIDTH  = 320;
    localparam integer HEIGHT = 240;
    localparam integer DEPTH  = WIDTH * HEIGHT;   // 76,800

    // y*320 + x = y*(256+64) + x = (y<<8) + (y<<6) + x
    wire [16:0] addr = ( {y,8'b0} + {y,6'b0} ) + x;

    (* rom_style = "block" *) reg [11:0] mem [0:DEPTH-1];

    initial begin
        $readmemh("image320x240_rgb12.hex", mem);
    end

    always @(posedge clk) begin
        rgb <= mem[addr];
    end

endmodule
