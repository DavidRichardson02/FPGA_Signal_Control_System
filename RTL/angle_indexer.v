`timescale 1ns/1ps
// ============================================================================
// angle_indexer.v  -- Discrete angle index + Q1.15 heading generator
//                     with directional steps (forward / reverse)
// ============================================================================
//
// Inputs
//   clk, rst       : single 100 MHz fabric clock, sync reset
//   step_pulse     : 1-cycle strobe = one angular bin advance
//   step_dir       : 1 = forward (increasing index), 0 = reverse (decreasing)
//
// Parameters
//   ANGLE_STEPS    : number of discrete angular bins per sweep
//
// Outputs
//   theta_q15      : unsigned Q1.15, modulo 1 turn (0 .. ~1.0)
//   theta_turn_q15 : signed Q1.15, multi-turn accumulator (can go ±)
//   angle_idx      : zero-extended discrete index (for debug / host)
// ============================================================================

module angle_indexer #(
    parameter integer ANGLE_STEPS = 180
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        step_pulse,      // one bin advance when 1
    input  wire        step_dir,        // 1 = forward, 0 = reverse

    output reg  [15:0] theta_q15,
    output reg  [31:0] theta_turn_q15,
    output reg  [15:0] angle_idx
);

    // ------------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------------
    function integer CLOG2;
        input integer value;
        integer v;
        begin
            v = value - 1;
            for (CLOG2 = 0; v > 0; CLOG2 = CLOG2 + 1)
                v = v >> 1;
        end
    endfunction

    localparam integer IDX_W         = CLOG2(ANGLE_STEPS);
    localparam integer Q15_ONE_TURN  = 1 << 15;                 // 32768
    localparam integer Q15_PER_STEP  = Q15_ONE_TURN / ANGLE_STEPS;

    // Sign-extended per-step increment for the 32-bit accumulator
    localparam signed [31:0] STEP_Q15_32 = Q15_PER_STEP;

    // Internal narrow index, maintained within [0 .. ANGLE_STEPS-1]
    reg [IDX_W-1:0] idx_narrow;

    // ------------------------------------------------------------------------
    // State update
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            idx_narrow      <= {IDX_W{1'b0}};
            angle_idx       <= 16'd0;
            theta_q15       <= 16'd0;
            theta_turn_q15  <= 32'sd0;
        end else begin
            if (step_pulse) begin
                // --- Index update with saturation at ends (safety) ---
                if (step_dir) begin
                    // forward
                    if (idx_narrow < ANGLE_STEPS-1)
                        idx_narrow <= idx_narrow + 1'b1;
                    // else: stay at max, ping-pong logic should prevent this
                end else begin
                    // reverse
                    if (idx_narrow > 0)
                        idx_narrow <= idx_narrow - 1'b1;
                    // else: stay at 0
                end

                // --- Q1.15 heading modulo 1 turn (wraps naturally) ---
                if (step_dir)
                    theta_q15 <= theta_q15 + Q15_PER_STEP[15:0];
                else
                    theta_q15 <= theta_q15 - Q15_PER_STEP[15:0];

                // --- Multi-turn signed accumulator ---
                if (step_dir)
                    theta_turn_q15 <= theta_turn_q15 + STEP_Q15_32;
                else
                    theta_turn_q15 <= theta_turn_q15 - STEP_Q15_32;

                // --- Zero-extend index for debug / host ---
                angle_idx <= { {(16-IDX_W){1'b0}}, idx_narrow };
            end
        end
    end

endmodule
