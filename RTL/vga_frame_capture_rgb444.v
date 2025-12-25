// vga_frame_capture_rgb444.v  (Verilog-2001)
`timescale 1ns/1ps

module vga_frame_capture_rgb444 #(
    parameter H_VISIBLE   = 640,
    parameter V_VISIBLE   = 480,
    parameter MAX_FRAMES  = 3,
    parameter OUT_PREFIX  = "vga_frame_"   // string literal still OK as a param
)(
    input  wire        pix_clk,
    input  wire        rst_pix,
    input  wire        frame_tick,
    input  wire        active_video,
    input  wire [9:0]  hcount,
    input  wire [9:0]  vcount,
    input  wire [11:0] rgb444
);
    //initial begin
    //    $display("[SIM] CWD(PWD)=%s", $getenv("PWD"));
    //    $display("[SIM] XSIM_SNAPSHOT=%s", $getenv("XSIM_SNAPSHOT"));
    //end



    integer fd;
    integer frame_idx;

    // 4-bit -> 8-bit expand
    function [7:0] expand4;
        input [3:0] v;
        begin
            expand4 = {v, v};
        end
    endfunction

    // filename buffer (Verilog doesn't have string type)
    reg [8*256-1:0] fname;  // 256-char packed string

    task open_frame;
        begin
            // Build "prefixNNN.ppm" using $sformat into packed reg
            // NOTE: $sformat is supported in XSim even in Verilog mode.
            $sformat(fname, "%s%03d.ppm", OUT_PREFIX, frame_idx);

            fd = $fopen(fname, "wb");
            if (fd == 0) begin
                $display("ERROR: Could not open %s", fname);
                $finish;
            end

            // PPM header
            $fwrite(fd, "P6\n%d %d\n255\n", H_VISIBLE, V_VISIBLE);
            $display("[CAP] Opened %s", fname);
        end
    endtask

    task close_frame;
        begin
            if (fd != 0) begin
                $fclose(fd);
                fd = 0;
                $display("[CAP] Closed frame %0d", frame_idx);
            end
        end
    endtask

    reg [7:0] r, g, b;

    initial begin
        fd = 0;
        frame_idx = 0;
    end

    always @(posedge pix_clk) begin
        if (rst_pix) begin
            close_frame();
            frame_idx <= 0;
        end else begin
            if (frame_tick) begin
                close_frame();
                if (frame_idx < MAX_FRAMES) begin
                    open_frame();
                    frame_idx <= frame_idx + 1;
                end else begin
                    $display("[CAP] Captured %0d frames; stopping.", MAX_FRAMES);
                    $finish;
                end
            end

            if (active_video && (fd != 0)) begin
                if (hcount < H_VISIBLE && vcount < V_VISIBLE) begin
                    r = expand4(rgb444[11:8]);
                    g = expand4(rgb444[7:4]);
                    b = expand4(rgb444[3:0]);

                    // write binary bytes
                    $fwrite(fd, "%c%c%c", r, g, b);
                end
            end
        end
    end

endmodule
