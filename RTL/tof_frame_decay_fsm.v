`timescale 1ns/1ps

// ============================================================================
// Module : tof_frame_decay_fsm
// Role   : Decay sweep over 256x256 intensity framebuffer (Port A RMW).
//
// Summary:
//   - On start_decay (1-cycle pulse), iterates addr 0..65535.
//   - For each pixel:
//       read I_old (sync; appears 1 cycle after address)
//       write I_new = (I_old > DECAY_STEP) ? (I_old - DECAY_STEP) : 0
//   - busy=1 while sweeping, so top-level can grant Port A exclusively.
//
// BRAM assumptions (matches tof_plot_bram_dp4 Port A):
//   - Port A synchronous read: rd_data_a is valid 1 clk after (wr_x,wr_y) address.
//   - We implement a 1-cycle pipeline:
//       cycle N   : present address for read
//       cycle N+1 : rd_data_a valid -> write back decayed value at previous address
//
// Interface matches vga_range_plot_top instantiation.
// ============================================================================
module tof_frame_decay_fsm #(
    parameter integer ADDR_W     = 16,
    parameter [3:0]   DECAY_STEP = 4'd1
)(
    input  wire              clk_sys,
    input  wire              rst_sys,
    input  wire              start_decay,   // 1-cycle pulse to begin decay sweep

    output reg               busy,          // 1 while sweep in progress

    output reg               wr_en,         // write enable to framebuffer (Port A)
    output reg  [7:0]        wr_x,          // x coordinate (0..255)
    output reg  [7:0]        wr_y,          // y coordinate (0..255)
    output reg  [3:0]        wr_data,       // decayed intensity

    input  wire [3:0]        rd_data_a      // sync readback from BRAM Port A
);
    localparam [ADDR_W-1:0] MAX_ADDR = {ADDR_W{1'b1}}; // 16'hFFFF for 256x256

    localparam [1:0]
        S_IDLE = 2'd0,
        S_RUN  = 2'd1;

    reg [1:0]        state;
    reg [ADDR_W-1:0] addr;

    // Pipeline register: previous address whose read data arrives "now"
    reg [ADDR_W-1:0] addr_d1;
    reg              have_d1;

    // Saturating subtract: I_new = max(I_old - DECAY_STEP, 0)
    wire [4:0] sub5 = {1'b0, rd_data_a} - {1'b0, DECAY_STEP};
    wire [3:0] i_new = sub5[4] ? 4'd0 : sub5[3:0]; // if borrow -> clamp to 0

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            state   <= S_IDLE;
            addr    <= {ADDR_W{1'b0}};
            addr_d1 <= {ADDR_W{1'b0}};
            have_d1 <= 1'b0;

            busy    <= 1'b0;
            wr_en   <= 1'b0;
            wr_x    <= 8'd0;
            wr_y    <= 8'd0;
            wr_data <= 4'd0;

        end else begin
            // defaults
            wr_en <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy    <= 1'b0;
                    have_d1 <= 1'b0;

                    if (start_decay) begin
                        // Start sweep at addr=0 (first cycle is "read prime")
                        state <= S_RUN;
                        busy  <= 1'b1;
                        addr  <= {ADDR_W{1'b0}};

                        // Present address immediately
                        wr_x <= 8'd0;
                        wr_y <= 8'd0;

                        // Prime pipeline
                        addr_d1 <= {ADDR_W{1'b0}};
                        have_d1 <= 1'b0;
                    end
                end

                S_RUN: begin
                    busy <= 1'b1;

                    // 1) Present current address for synchronous read
                    wr_x <= addr[7:0];
                    wr_y <= addr[15:8];

                    // 2) One cycle later, rd_data_a corresponds to addr_d1
                    if (have_d1) begin
                        wr_en   <= 1'b1;
                        wr_x    <= addr_d1[7:0];
                        wr_y    <= addr_d1[15:8];
                        wr_data <= i_new;
                    end

                    // 3) Advance pipeline register
                    addr_d1 <= addr;
                    have_d1 <= 1'b1;

                    // 4) Advance address / finish
                    if (addr == MAX_ADDR) begin
                        // We have *presented* the last address for read.
                        // We must do ONE MORE cycle to write back its decay.
                        //
                        // Easiest, deterministic approach:
                        //   - keep state S_RUN for one extra cycle by using have_d1.
                        //   - when addr==MAX_ADDR and have_d1==1 AND addr_d1==MAX_ADDR,
                        //     after writing it this cycle, exit.
                        //
                        // Here, we detect the "final writeback" condition:
                        if (have_d1 && (addr_d1 == MAX_ADDR)) begin
                            state <= S_IDLE;
                            busy  <= 1'b0;
                            addr  <= {ADDR_W{1'b0}};
                            have_d1 <= 1'b0;
                        end else begin
                            // Hold addr at MAX_ADDR until its writeback occurs
                            addr <= addr;
                        end
                    end else begin
                        addr <= addr + 1'b1;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
