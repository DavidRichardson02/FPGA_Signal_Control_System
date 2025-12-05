`timescale 1ns/1ps


// ============================================================================
// Module : clk_div_4
// Role   : Divide 100 MHz input clock by 4 → ~25 MHz pixel clock.
// Note   : For a production design, a PLL/MMCM (Clocking Wizard) is preferred
//          to hit 25.175 MHz exactly, but 25 MHz is acceptable for lab use.
// ============================================================================
module clk_div_4 (
    input  wire clk_in,
    input  wire rst_in,
    output reg  clk_out
);
    reg [1:0] div_cnt;

    always @(posedge clk_in) begin
        if (rst_in) begin
            div_cnt <= 2'b00;
            clk_out <= 1'b0;
        end else begin
            div_cnt <= div_cnt + 2'b01;
            clk_out <= div_cnt[1];   // divide-by-4
        end
    end

endmodule
