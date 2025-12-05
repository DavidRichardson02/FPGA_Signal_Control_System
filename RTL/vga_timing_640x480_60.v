`timescale 1ns/1ps

// ============================================================================
// Module : vga_timing_640x480_60
// Role   : Generate 640x480@60 Hz timing from a 25 MHz pixel clock.
//          - Active video region      : 640 x 480
//          - Total horizontal pixels  : 800
//          - Total vertical lines     : 525
//          - H sync pulse width       : 96 pixels (active low)
//          - V sync pulse width       : 2 lines (active low)
// ============================================================================
module vga_timing_640x480_60 (
    input  wire        clk_pix,       // 25 MHz pixel clock
    input  wire        rst_pix,       // synchronous reset (active-HIGH)
    output reg  [9:0]  hcount,        // 0..799
    output reg  [9:0]  vcount,        // 0..524
    output reg         hsync_n,
    output reg         vsync_n,
    output wire        active_video
);
    // Horizontal timings (in pixel clocks)
    localparam H_VISIBLE  = 640;
    localparam H_FP       = 16;
    localparam H_SYNC     = 96;
    localparam H_BP       = 48;
    localparam H_TOTAL    = H_VISIBLE + H_FP + H_SYNC + H_BP;  // 800

    // Vertical timings (in lines)
    localparam V_VISIBLE  = 480;
    localparam V_FP       = 10;
    localparam V_SYNC     = 2;
    localparam V_BP       = 33;
    localparam V_TOTAL    = V_VISIBLE + V_FP + V_SYNC + V_BP;  // 525

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            hcount <= 10'd0;
            vcount <= 10'd0;
            hsync_n <= 1'b1;
            vsync_n <= 1'b1;
        end else begin
            // Horizontal counter
            if (hcount == H_TOTAL-1) begin
                hcount <= 10'd0;

                // Vertical counter
                if (vcount == V_TOTAL-1) begin
                    vcount <= 10'd0;
                end else begin
                    vcount <= vcount + 10'd1;
                end
            end else begin
                hcount <= hcount + 10'd1;
            end

            // H sync: active LOW during sync interval
            if ((hcount >= H_VISIBLE + H_FP) &&
                (hcount <  H_VISIBLE + H_FP + H_SYNC)) begin
                hsync_n <= 1'b0;
            end else begin
                hsync_n <= 1'b1;
            end

            // V sync: active LOW during sync interval
            if ((vcount >= V_VISIBLE + V_FP) &&
                (vcount <  V_VISIBLE + V_FP + V_SYNC)) begin
                vsync_n <= 1'b0;
            end else begin
                vsync_n <= 1'b1;
            end
        end
    end

    assign active_video =
        (hcount < H_VISIBLE) &&
        (vcount < V_VISIBLE);

endmodule
