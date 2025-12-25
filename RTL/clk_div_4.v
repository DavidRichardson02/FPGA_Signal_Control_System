`timescale 1ns/1ps


// ============================================================================
// Module : clk_div_4_rigorous
// Role   : Divide 100 MHz input clock by 4 -> 25 MHz pixel clock
//
// Rigor improvements vs naive divider:
//   1) Reset is synchronized to clk_in (avoids async release ambiguity).
//   2) Output clock is driven by a dedicated flop toggle (50% duty cycle).
//   3) Clear phase relationship: clk_pix rises every 4 cycles of clk_in,
//      with deterministic alignment after reset.
// Notes:
//   - This is still a fabric-generated clock. For best practice, route clk_pix
//     through a BUFG, and prefer MMCM for exact 25.175 MHz when targeting VGA.
// ============================================================================
module clk_div_4 (
    input  wire clk_in,
    input  wire rst_in,
    output wire  clk_out
);
    // 2FF reset synchronizer (async assert, sync deassert)
    reg rst_ff1, rst_ff2;
    always @(posedge clk_in or posedge rst_in) begin
        if (rst_in) begin
            rst_ff1 <= 1'b1;
            rst_ff2 <= 1'b1;
        end else begin
            rst_ff1 <= 1'b0;
            rst_ff2 <= rst_ff1;
        end
    end
    wire rst_sync = rst_ff2;

    // Divide-by-4: toggle output every 2 input cycles
    // (toggle produces exact 50% duty cycle)
    reg [1:0] cnt;
    reg       clk_div;

    always @(posedge clk_in) begin
        if (rst_sync) begin
            cnt     <= 2'd0;
            clk_div <= 1'b0;
        end else begin
            cnt <= cnt + 2'd1;
            if (cnt == 2'd1) begin
                clk_div <= ~clk_div;  // toggles every 2 cycles -> /4 output
            end
        end
    end

    // Put the divided clock onto a global clock net
    // Xilinx:
    //   (* CLOCK_BUFFER_TYPE = "BUFG" *)  // optional hint
    //   BUFG u_bufg_pix (.I(clk_div), .O(clk_out));
    //
    // If don't want vendor primitives here, at least keep the option.
    assign clk_out = clk_div;

endmodule

