`timescale 1ns/1ps
// ============================================================================
// sim_telemetry_logger_csv.v  (SIMULATION ONLY, Verilog-2001)
// Logs accel samples to CSV: idx,time_ns,state,raw_x,raw_y,raw_z,q7_x,q7_y,q7_z
//
// Notes on "Verilog-2001 refactor":
//   - No SystemVerilog "string" parameter: use packed byte vector.
//   - No SystemVerilog "final" block: close file via at-exit hook (optional)
//     or leave it to simulator; we also close on explicit simulation end.
// ============================================================================
module sim_telemetry_logger_csv #(
    // 64-char filename buffer (including .csv); adjust width if like
    parameter [8*64-1:0] OUT_FILE = "adxl362_telemetry.csv"
)(
    input  wire        clk,
    input  wire        rst,

    input  wire [4:0]         state,
    input  wire signed [7:0]  raw_x,
    input  wire signed [7:0]  raw_y,
    input  wire signed [7:0]  raw_z,

    input  wire signed [7:0]  q7_x,
    input  wire signed [7:0]  q7_y,
    input  wire signed [7:0]  q7_z,

    input  wire        sample_valid
);

`ifndef SYNTHESIS
    integer fd;
    integer sample_idx;

    initial begin
        fd = $fopen(OUT_FILE, "w");
        sample_idx = 0;

        if (fd == 0) begin
            $display("ERROR: could not open %0s", OUT_FILE);
            $finish;
        end

        $fwrite(fd, "idx,time_ns,state,raw_x,raw_y,raw_z,q7_x,q7_y,q7_z\n");
    end

    always @(posedge clk) begin
        if (rst) begin
            sample_idx <= 0;
        end else if (sample_valid) begin
            $fwrite(fd, "%0d,%0t,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                sample_idx, $time, state,
                raw_x, raw_y, raw_z,
                q7_x, q7_y, q7_z
            );
            sample_idx <= sample_idx + 1;
        end
    end

    // Verilog-2001 has no "final". Two practical options:
    //   (A) Close on an explicit end condition control in the testbench.
    //   (B) Best-effort close at time 0 when sim ends (simulators often close automatically).
    //
    // Here we provide a simple best-effort pattern: if simulation calls $finish,
    // many simulators still flush/close. If want deterministic close,
    // add a tb-driven 'done' signal and close when done==1.
    //
    // Example deterministic close (uncomment and wire "done" if desired):
    // input wire done;
    // always @(posedge clk) if (done && fd!=0) begin $fclose(fd); fd=0; end

`endif

endmodule
