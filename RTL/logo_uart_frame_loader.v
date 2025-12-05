`timescale 1ns/1ps
// ============================================================================
// Module : logo_uart_frame_loader
// Role   : Consume a continuous UART RX byte stream carrying 320x240 RGB444
//          pixels and write them into the *inactive* buffer of
//          image_dualbuf_320x240_rgb12. After a full frame is received,
//          emit a 1-cycle swap_req pulse so the display buffer flips
//          (at frame boundaries in the pixel domain).
//
// Data format (per your MATLAB sendLogoFrame()):
//   For each pixel (rgb12: [11:0] = {R[3:0], G[3:0], B[3:0]}):
//     byte_hi = rgb12[11:4]
//     byte_lo = {rgb12[3:0], 4'b0000}   // low nibble in [7:4]
//
// Stream order:
//   y = 0..239 (MATLAB 1..240)
//     x = 0..319 (MATLAB 1..320)
//
// Notes:
//   - No explicit frame header; we rely on exact 76800 pixels per frame.
//   - If alignment is lost, you would add a header/marker in a later rev.
// ============================================================================
module logo_uart_frame_loader #(
    parameter integer WIDTH  = 320,
    parameter integer HEIGHT = 240
)(
    input  wire        clk_sys,
    input  wire        rst_sys,

    // UART RX side (system domain)
    input  wire [7:0]  rx_byte,    // byte from UART RX core
    input  wire        rx_vld,     // 1-cycle strobe when rx_byte is valid

    // Connection into image_dualbuf_320x240_rgb12 (system side)
    input  wire        write_buf_sys,   // which bank is inactive (from dualbuf)
    output reg         logo_wr_en,      // write enable into dualbuf
    output reg  [16:0] logo_wr_addr,    // 0..(WIDTH*HEIGHT-1)
    output reg  [11:0] logo_wr_data,    // RGB444 pixel

    // Request to flip buffers (system → pixel)
    output reg         logo_swap_req    // 1-cycle pulse when a full frame is written
);
    localparam integer DEPTH = WIDTH * HEIGHT;   // 76800 for 320x240

    // We assemble each 12-bit pixel from two bytes:
    //   - First byte = high 8 bits (rgb12[11:4])
    //   - Second byte = low nibble (rgb12[3:0]) in bits [7:4].
    //
    // Control:
    //   expect_hi == 1 → next rx_byte is high 8 bits
    //   expect_hi == 0 → next rx_byte is low nibble, then we write the pixel.
    reg        expect_hi;
    reg [7:0]  hi_byte_latched;

    // Pixel address counter
    reg [16:0] pix_addr;

    // Combinational helper for the assembled 12-bit pixel
    wire [11:0] pixel_assembled =
        { hi_byte_latched, rx_byte[7:4] };  // {rgb[11:4], rgb[3:0]}

    always @(posedge clk_sys or posedge rst_sys) begin
        if (rst_sys) begin
            expect_hi     <= 1'b1;
            hi_byte_latched <= 8'h00;
            pix_addr      <= 17'd0;
            logo_wr_en    <= 1'b0;
            logo_wr_addr  <= 17'd0;
            logo_wr_data  <= 12'h000;
            logo_swap_req <= 1'b0;
        end else begin
            // Default strobes low each cycle
            logo_wr_en    <= 1'b0;
            logo_swap_req <= 1'b0;

            if (rx_vld) begin
                if (expect_hi) begin
                    // First byte of pixel: high 8 bits
                    hi_byte_latched <= rx_byte;
                    expect_hi       <= 1'b0;
                end else begin
                    // Second byte of pixel: low nibble in rx_byte[7:4]
                    logo_wr_en   <= 1'b1;
                    logo_wr_addr <= pix_addr;
                    logo_wr_data <= pixel_assembled;

                    // Advance pixel address
                    if (pix_addr == (DEPTH-1)) begin
                        pix_addr      <= 17'd0;
                        logo_swap_req <= 1'b1;  // full frame completed
                    end else begin
                        pix_addr <= pix_addr + 17'd1;
                    end

                    // Next pixel starts with high byte again
                    expect_hi <= 1'b1;
                end
            end
        end
    end

endmodule
