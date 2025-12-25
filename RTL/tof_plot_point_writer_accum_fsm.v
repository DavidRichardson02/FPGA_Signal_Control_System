`timescale 1ns/1ps
// ============================================================================
// Module : tof_plot_point_writer_accum_fsm
// Role   : Additive / accumulative persistence writer for the ToF framebuffer.
//
// Summary:
//   - Converts (dist_mm, theta_q15) to framebuffer (x_fb,y_fb)
//   - Performs a READ-MODIFY-WRITE (RMW) on the 4-bit intensity framebuffer:
//         I_new = min(I_old + HIT_INC, 15)
//   - Provides Port-A address continuously while active.
//   - Inhibits itself when decay_busy=1 (decay sweep owns Port A).
//
// Why this is the "next logical extension":
//   - With decay alone, every sample "punches" the pixel to a fixed intensity.
//   - With accumulation, repeated hits stabilize and brighten; noise remains dim.
//   - This is the classic oscilloscope / radar persistence behavior.
//
// BRAM assumptions (matches tof_plot_bram_dp4):
//   - Port A synchronous read: q_a is valid 1 clk after (x_a,y_a) are applied.
// ============================================================================
module tof_plot_point_writer_accum_fsm #(
    parameter [15:0] R_MAX_MM  = 16'd2000,
    parameter [3:0]  HIT_INC   = 4'd3   // energy added per hit (1..15)
)(
    input  wire        clk_sys,
    input  wire        rst_sys,

    input  wire        decay_busy,   // when 1, decay sweep owns Port A

    // ToF sample (system domain)
    input  wire [15:0] dist_mm,
    input  wire        dist_vld,
    input  wire [15:0] theta_q15,

    // BRAM Port-A interface (address always driven, WE only when writing)
    output reg  [7:0]  a_x,
    output reg  [7:0]  a_y,
    output reg         a_we,
    output reg  [3:0]  a_d,
    input  wire [3:0]  a_q          // synchronous readback from BRAM Port A
);

    // ------------------------------------------------------------------------
    // Helper: polar → framebuffer mapping (same mapping strategy as used)
    // ------------------------------------------------------------------------

    // Clamp distance
    wire [15:0] r_clamped = (dist_mm > R_MAX_MM) ? R_MAX_MM : dist_mm;

    // Scale distance to [0..255] with rounding:
    //   y_scaled = round(r * 255 / R_MAX_MM)
    wire [23:0] mult_255     = r_clamped * 8'd255;
    wire [23:0] mult_round   = mult_255 + {8'd0, (R_MAX_MM >> 1)};
    wire [7:0]  y_scaled     = (R_MAX_MM == 0) ? 8'd0 : (mult_round / R_MAX_MM);

    // Near bottom, far top
    wire [7:0] y_fb_now = 8'd255 - y_scaled;

    // Angle → x in [0..255]
    wire [7:0] x_fb_now = theta_q15[15:8];

    // ------------------------------------------------------------------------
    // Pending buffer (1-deep) so a hit arriving during an RMW doesn't vanish
    // ------------------------------------------------------------------------
    reg        pend_valid;
    reg [7:0]  pend_x;
    reg [7:0]  pend_y;

    // ------------------------------------------------------------------------
    // RMW FSM
    // ------------------------------------------------------------------------
    localparam [1:0]
        S_IDLE = 2'd0,
        S_SET  = 2'd1,   // present address for sync read
        S_WR   = 2'd2;   // write accumulated value
    reg [1:0] state, state_next;

    // Latched current target pixel (the one being processed)
    reg [7:0] cur_x;
    reg [7:0] cur_y;

    // Saturating add: I_new = min(I_old + HIT_INC, 15)
    wire [4:0] sum5 = {1'b0, a_q} + {1'b0, HIT_INC};
    wire [3:0] i_new = (sum5[4] || (sum5[3:0] > 4'hF)) ? 4'hF : sum5[3:0];

    // ------------------------------------------------------------------------
    // Sequential
    // ------------------------------------------------------------------------
    always @(posedge clk_sys) begin
        if (rst_sys) begin
            state      <= S_IDLE;

            a_x        <= 8'd0;
            a_y        <= 8'd0;
            a_we       <= 1'b0;
            a_d        <= 4'd0;

            cur_x      <= 8'd0;
            cur_y      <= 8'd0;

            pend_valid <= 1'b0;
            pend_x     <= 8'd0;
            pend_y     <= 8'd0;

        end else begin
            state <= state_next;

            // Default: no write unless we’re in S_WR
            a_we <= 1'b0;

            // Capture incoming samples into pending if we can't start immediately.
            // Rule:
            //   - If decay busy, we just pend (or overwrite if already pended).
            //   - If we're in the middle of an RMW, we pend one.
            if (dist_vld) begin
                if (decay_busy || (state != S_IDLE)) begin
                    // 1-deep queue (newest wins). If want "no drops ever",
                    // replace with a small FIFO.
                    pend_valid <= 1'b1;
                    pend_x     <= x_fb_now;
                    pend_y     <= y_fb_now;
                end
            end

            case (state)
                S_IDLE: begin
                    // If decay is running, we do nothing (Port A not ours).
                    if (!decay_busy) begin
                        if (pend_valid) begin
                            // Start processing pending sample
                            cur_x      <= pend_x;
                            cur_y      <= pend_y;
                            pend_valid <= 1'b0;

                            // Drive BRAM address (for sync read next cycle)
                            a_x <= pend_x;
                            a_y <= pend_y;
                        end else if (dist_vld) begin
                            // Start immediately on this incoming hit
                            cur_x <= x_fb_now;
                            cur_y <= y_fb_now;

                            a_x <= x_fb_now;
                            a_y <= y_fb_now;
                        end
                    end
                end

                S_SET: begin
                    // Address was already applied in S_IDLE transition; keep stable
                    a_x <= cur_x;
                    a_y <= cur_y;
                end

                S_WR: begin
                    // Write back accumulated intensity
                    a_x  <= cur_x;
                    a_y  <= cur_y;
                    a_d  <= i_new;
                    a_we <= 1'b1;
                end

                default: begin end
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Combinational next-state
    // ------------------------------------------------------------------------
    always @* begin
        state_next = state;

        case (state)
            S_IDLE: begin
                // We only advance if we actually launched a job and decay is not busy
                if (!decay_busy) begin
                    if (pend_valid || dist_vld)
                        state_next = S_SET;
                end
            end

            // One cycle later, a_q is valid -> write
            S_SET: state_next = S_WR;

            // After write, return idle
            S_WR:  state_next = S_IDLE;

            default: state_next = S_IDLE;
        endcase
    end

endmodule
