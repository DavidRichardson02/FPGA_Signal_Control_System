`timescale 1ns/1ps

// ============================================================================
// Module : image_dualbuf_320x240_rgb12
// Role   : 320x240 RGB 4:4:4 *double-buffered* image store.
//
//   - Pixel domain (clk_pix):
//       • x_pix,y_pix (0..319,0..239) → rgb_pix (12-bit).
//       • Uses display buffer 0 or 1, flipped only at frame_tick_pix.
//
//   - System domain (clk_sys):
//       • Writes new frame data into the *inactive* buffer.
//       • Linear write address wr_addr_sys (0..DEPTH-1).
//       • When a full frame is written, assert swap_req_sys (1-cycle).
//
//   - Buffer flip:
//       • swap_req_sys is synchronized into clk_pix domain.
//       • At the next frame_tick_pix, display buffer toggles.
//       • active_buf_sys mirrors which bank is currently on-screen.
//       • write_buf_sys = ~active_buf_sys → bank you should be filling.
//
// Addressing:
//   DEPTH = 320*240 = 76800
//   addr = y*320 + x = (y<<8) + (y<<6) + x
// ============================================================================
module image_dualbuf_320x240_rgb12 (
    // ---------------- Pixel (VGA) domain -----------------------------------
    input  wire        clk_pix,
    input  wire        rst_pix,
    input  wire [8:0]  x_pix,          // 0..319
    input  wire [7:0]  y_pix,          // 0..239
    input  wire        frame_tick_pix, // 1-cycle pulse at top-left of frame
    output reg  [11:0] rgb_pix,        // {R[11:8],G[7:4],B[3:0]}

    // ---------------- System (write) domain --------------------------------
    input  wire        clk_sys,
    input  wire        rst_sys,
    input  wire        wr_en_sys,      // write strobe (clk_sys)
    input  wire [16:0] wr_addr_sys,    // 0..DEPTH-1
    input  wire [11:0] wr_data_sys,    // 12-bit RGB444

    // Buffer swap control (system → pixel)
    input  wire        swap_req_sys,   // 1-cycle pulse in clk_sys domain

    // Status / helper outputs
    output wire        display_buf_pix, // 0/1 bank currently displayed (clk_pix)
    output wire        active_buf_sys,  // same as above, synced into clk_sys
    output wire        write_buf_sys    // = ~active_buf_sys
);
    // ------------------------------------------------------------------------
    // Constants / address generation
    // ------------------------------------------------------------------------
    localparam integer WIDTH  = 320;
    localparam integer HEIGHT = 240;
    localparam integer DEPTH  = WIDTH * HEIGHT;   // 76800
    localparam integer AW     = 17;               // enough for 0..DEPTH-1

    // addr = y*320 + x = (y<<8) + (y<<6) + x
    wire [AW-1:0] pix_addr = ( {y_pix,8'b0} + {y_pix,6'b0} ) + x_pix;

    // ------------------------------------------------------------------------
    // 1) Display buffer select (pixel domain) + swap_req sync
    // ------------------------------------------------------------------------
    reg display_buf_pix_r;

    assign display_buf_pix = display_buf_pix_r;

    // Sync swap_req_sys into clk_pix and detect rising edge
    reg swap_s0_pix, swap_s1_pix, swap_prev_pix;
    wire swap_sync_pix  = swap_s1_pix;
    wire swap_rise_pix  = swap_sync_pix & ~swap_prev_pix;

    always @(posedge clk_pix) begin
        if (rst_pix) begin
            swap_s0_pix       <= 1'b0;
            swap_s1_pix       <= 1'b0;
            swap_prev_pix     <= 1'b0;
            display_buf_pix_r <= 1'b0;  // start with bank 0
        end else begin
            // 2-FF sync
            swap_s0_pix   <= swap_req_sys;
            swap_s1_pix   <= swap_s0_pix;
            swap_prev_pix <= swap_sync_pix;

            // Toggle which buffer is displayed *only* at frame boundary
            if (frame_tick_pix && swap_rise_pix) begin
                display_buf_pix_r <= ~display_buf_pix_r;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 2) Display buffer status back into system domain (for write_buf_sys)
    // ------------------------------------------------------------------------
    reg disp_s0_sys, disp_s1_sys;

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            disp_s0_sys <= 1'b0;
            disp_s1_sys <= 1'b0;
        end else begin
            // 2-FF sync of display_buf_pix_r into clk_sys
            disp_s0_sys <= display_buf_pix_r;
            disp_s1_sys <= disp_s0_sys;
        end
    end

    assign active_buf_sys = disp_s1_sys;    // which bank is on-screen (0/1)
    assign write_buf_sys  = ~active_buf_sys; // which bank you should write

    // ------------------------------------------------------------------------
    // 3) Instantiate two BRAM framebuffers: mem0 and mem1
    //    - Both see the *same* pix_addr for reading.
    //    - Write side only touches the *inactive* buffer.
    // ------------------------------------------------------------------------
    wire [11:0] mem0_dout;
    wire [11:0] mem1_dout;

    // write enables for each bank in clk_sys domain
    wire we0_sys = wr_en_sys && (write_buf_sys == 1'b0);
    wire we1_sys = wr_en_sys && (write_buf_sys == 1'b1);

    // Bank 0
    logo_framebuffer_dp #(
        .WIDTH (WIDTH),
        .HEIGHT(HEIGHT),
        .AW    (AW)
    ) u_mem0 (
        .clk_a  (clk_sys),
        .we_a   (we0_sys),
        .addr_a (wr_addr_sys[AW-1:0]),
        .din_a  (wr_data_sys),
        .clk_b  (clk_pix),
        .addr_b (pix_addr),
        .dout_b (mem0_dout)
    );
    
    // Bank 1
    logo_framebuffer_dp #(
        .WIDTH (WIDTH),
        .HEIGHT(HEIGHT),
        .AW    (AW)
    ) u_mem1 (
        .clk_a  (clk_sys),
        .we_a   (we1_sys),
        .addr_a (wr_addr_sys[AW-1:0]),
        .din_a  (wr_data_sys),
        .clk_b  (clk_pix),
        .addr_b (pix_addr),
        .dout_b (mem1_dout)
    );

    // ------------------------------------------------------------------------
    // 4) Final mux to rgb_pix (pixel domain)
    // ------------------------------------------------------------------------
    always @(posedge clk_pix) begin
        if (rst_pix) begin
            rgb_pix <= 12'h000;
        end else begin
            rgb_pix <= (display_buf_pix_r == 1'b0) ? mem0_dout : mem1_dout;
        end
    end

endmodule
