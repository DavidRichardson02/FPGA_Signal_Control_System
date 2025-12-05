`timescale 1ns/1ps

// ============================================================================
// Module : vga_char_glyph
// Role   : Draw one 8x8 character at a fixed (x0,y0) location in the
//          pixel domain. Works in the VGA pixel clock domain.
// ============================================================================

module vga_char_glyph #(
    parameter integer CH_W = 8,
    parameter integer CH_H = 8
)(
    input  wire       clk_pix,
    input  wire       rst_pix,
    input  wire [9:0] hcount,       // from VGA timing core
    input  wire [9:0] vcount,
    input  wire       active_video, // from VGA timing core

    input  wire [9:0] x0,           // top-left X of character cell
    input  wire [9:0] y0,           // top-left Y of character cell
    input  wire [7:0] char_code,    // ASCII code

    output reg        pixel_on      // 1 when this char wants this pixel
);

    // Are we inside the 8x8 box for this character?
    wire in_box =
        active_video &&
        (hcount >= x0) && (hcount < x0 + CH_W) &&
        (vcount >= y0) && (vcount < y0 + CH_H);

    // Local coordinates within the 8x8 cell
    wire [9:0] rel_x = hcount - x0;
    wire [9:0] rel_y = vcount - y0;

    wire [2:0] col = rel_x[2:0];   // 0..7
    wire [2:0] row = rel_y[2:0];   // 0..7

    wire [7:0] row_bits;
    font8x8_basic u_font (
        .char_code(char_code),
        .row      (row),
        .row_bits (row_bits)
    );

    // MSB of row_bits is leftmost pixel
    wire font_bit = row_bits[7 - col];

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            pixel_on <= 1'b0;
        end else begin
            pixel_on <= in_box && font_bit;
        end
    end

endmodule
