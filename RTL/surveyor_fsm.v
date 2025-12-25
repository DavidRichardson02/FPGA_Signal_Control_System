`timescale 1ns/1ps
// ============================================================================
// surveyor_fsm.v  -- ToF surveyor front-end with ping-pong angular sweep
// ============================================================================
//
// Concept
// -------
//   • Explicit control of a survey sweep via 'sweep_start' / 'sweep_stop'.
//   • For each accepted ToF sample (tof_vld) while running:
//       - Emit one UART sample (sample_vld, dist_mm, status).
//       - Emit one step to the angle indexer + stepper (step_pulse, step_dir).
//       - Advance a local index 'idx' with ping-pong pattern:
//           0,1,2,...,MAX, MAX-1,...,1,0,1,... and so on.
//
// Parameters
// ----------
//   STEPS_PER_SWEEP : number of angular bins in [0 .. STEPS_PER_SWEEP-1]
//
// Inputs
// ------
//   clk, rst        : 100 MHz fabric clock, sync reset
//   sweep_start     : 1-cycle pulse to start sweep (ignored if already running)
//   sweep_stop      : 1-cycle pulse to stop and return to IDLE
//   tof_vld         : 1-cycle strobe from tof_sensor (new distance available)
//   tof_dist_mm     : distance in mm
//   tof_status      : 8-bit status from tof_sensor
//   tof_busy        : reserved for future gating
//
// Outputs
// -------
//   sweep_active    : 1 when in RUN state
//   step_pulse      : 1-cycle strobe per angular step
//   step_dir        : 1 = forward (increasing index), 0 = reverse
//   sample_vld      : 1-cycle strobe when dist_mm/status are valid
//   dist_mm, status : latched values for mapper_packetizer
// ============================================================================

module surveyor_fsm #(
    parameter integer STEPS_PER_SWEEP = 180
)(
    input  wire       clk,
    input  wire       rst,

    // Sweep control
    input  wire       sweep_start,   // 1-cycle strobe
    input  wire       sweep_stop,    // 1-cycle strobe

    // Upstream: ToF core
    input  wire       tof_vld,
    input  wire [15:0] tof_dist_mm,
    input  wire [7:0]  tof_status,
    input  wire        tof_busy,     // not used yet

    // Downstream: angle indexer + mechanical scanner
    output reg        sweep_active,
    output reg        step_pulse,
    output reg        step_dir,

    // Downstream: mapper_packetizer
    output reg        sample_vld,
    output reg [15:0] dist_mm,
    output reg [7:0]  status
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

    localparam integer IDX_W    = CLOG2(STEPS_PER_SWEEP);
    localparam [IDX_W-1:0] IDX_MIN = {IDX_W{1'b0}};
    localparam [IDX_W-1:0] IDX_MAX = STEPS_PER_SWEEP-1;

    // ------------------------------------------------------------------------
    // State + ping-pong index
    // ------------------------------------------------------------------------
    localparam ST_IDLE = 1'b0;
    localparam ST_RUN  = 1'b1;

    reg        state;
    reg        dir_fwd;          // 1 = moving upwards in index, 0 = moving down
    reg [IDX_W-1:0] idx;         // index of *current* view angle

    // ------------------------------------------------------------------------
    // Main sequential block
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state        <= ST_IDLE;
            sweep_active <= 1'b0;
            dir_fwd      <= 1'b1;
            idx          <= IDX_MIN;
            step_pulse   <= 1'b0;
            step_dir     <= 1'b1;
            sample_vld   <= 1'b0;
            dist_mm      <= 16'd0;
            status       <= 8'd0;
        end else begin
            // Default: no strobes each cycle
            step_pulse   <= 1'b0;
            sample_vld   <= 1'b0;

            case (state)
                // ------------------------------------------------------------
                ST_IDLE: begin
                    sweep_active <= 1'b0;

                    // Reset ping-pong state whenever we re-enter IDLE
                    dir_fwd <= 1'b1;
                    idx     <= IDX_MIN;

                    if (sweep_start) begin
                        // Arm a new sweep starting from IDX_MIN
                        state        <= ST_RUN;
                        sweep_active <= 1'b1;
                    end
                end

                // ------------------------------------------------------------
                ST_RUN: begin
                    sweep_active <= 1'b1;

                    if (sweep_stop) begin
                        state        <= ST_IDLE;
                        sweep_active <= 1'b0;
                    end else begin
                        // Accept ToF sample and advance sweep only on tof_vld
                        if (tof_vld) begin
                            // 1) Capture data for mapper_packetizer
                            dist_mm    <= tof_dist_mm;
                            status     <= tof_status;
                            sample_vld <= 1'b1;

                            // 2) Decide step direction + next index (ping-pong)
                            //    Current sample is at 'idx'. After this sample,
                            //    we step once so the *next* sample sees idx_next.
                            step_pulse <= 1'b1;

                            if (dir_fwd) begin
                                if (idx == IDX_MAX) begin
                                    // At upper boundary: reverse direction
                                    step_dir <= 1'b0;  // reverse step
                                    dir_fwd  <= 1'b0;
                                    if (IDX_MAX > IDX_MIN)
                                        idx <= IDX_MAX - 1'b1;
                                end else begin
                                    // Normal forward step
                                    step_dir <= 1'b1;
                                    idx      <= idx + 1'b1;
                                end
                            end else begin
                                // Currently sweeping downwards
                                if (idx == IDX_MIN) begin
                                    // At lower boundary: reverse direction
                                    step_dir <= 1'b1;  // forward step
                                    dir_fwd  <= 1'b1;
                                    if (IDX_MAX > IDX_MIN)
                                        idx <= IDX_MIN + 1'b1;
                                end else begin
                                    // Normal reverse step
                                    step_dir <= 1'b0;
                                    idx      <= idx - 1'b1;
                                end
                            end
                        end // if (tof_vld)
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
