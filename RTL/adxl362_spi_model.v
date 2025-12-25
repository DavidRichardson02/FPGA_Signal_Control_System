`timescale 1ns/1ps
// ============================================================================
// adxl362_spi_model.v  (SIMULATION MODEL, Verilog-2001)
// Minimal ADXL362-like SPI slave: supports CMD_READ/CMD_WRITE and a few regs.
// Notes on "Verilog-2001 refactor":
//   - No SystemVerilog keywords: no "automatic" block variables, no "string".
//   - Any temporary regs used inside always blocks are declared at module scope.
//   - Simulation-only behavior kept under `ifndef SYNTHESIS.
// ============================================================================
module adxl362_spi_model (
    input  wire csn,
    input  wire sclk,
    input  wire mosi,
    output reg  miso
);

`ifndef SYNTHESIS
    localparam [7:0] CMD_WRITE = 8'h0A;
    localparam [7:0] CMD_READ  = 8'h0B;

    // "Registers"
    reg [7:0] reg_devid_ad;
    reg [7:0] reg_filter_ctl;
    reg [7:0] reg_power_ctl;

    reg signed [7:0] reg_xdata;
    reg signed [7:0] reg_ydata;
    reg signed [7:0] reg_zdata;

    // Simple waveform generator for xyz
    integer t;

    // SPI shift regs
    reg [7:0] sh_in;
    reg [7:0] sh_out;
    integer   bitcnt;

    // Frame decode
    reg [7:0] cmd;
    reg [7:0] addr;
    reg       have_cmd;
    reg       have_addr;

    // Verilog-2001: temp byte must be declared at module scope (no block "automatic")
    reg [7:0] full_byte;

    // Address auto-increment read
    function [7:0] reg_read;
        input [7:0] a;
        begin
            case (a)
                8'h00: reg_read = reg_devid_ad;
                8'h2C: reg_read = reg_filter_ctl;
                8'h2D: reg_read = reg_power_ctl;
                8'h08: reg_read = reg_xdata;
                8'h09: reg_read = reg_ydata;
                8'h0A: reg_read = reg_zdata;
                default: reg_read = 8'h00;
            endcase
        end
    endfunction

    // Write helper
    task reg_write;
        input [7:0] a;
        input [7:0] d;
        begin
            case (a)
                8'h2C: reg_filter_ctl = d;
                8'h2D: reg_power_ctl  = d;
                default: ; // ignore others
            endcase
        end
    endtask

    // Initialize registers
    initial begin
        miso = 1'b0;
        reg_devid_ad   = 8'hAD;
        reg_filter_ctl = 8'h00;
        reg_power_ctl  = 8'h00;

        reg_xdata = 0;
        reg_ydata = 0;
        reg_zdata = 0;

        sh_in = 0;
        sh_out = 0;
        bitcnt = 0;
        cmd = 0;
        addr = 0;
        have_cmd = 0;
        have_addr = 0;
        full_byte = 8'h00;
        t = 0;
    end

    // Update xyz over time (only when "measurement mode" set)
    always begin
        #100000; // 100 us in sim time
        t = t + 1;
        if (reg_power_ctl[1]) begin
            reg_xdata <= $signed((t % 128) - 64);
            reg_ydata <= $signed(((t*2) % 128) - 64);
            reg_zdata <= $signed(64 - ((t*3) % 128));
        end else begin
            reg_xdata <= 0;
            reg_ydata <= 0;
            reg_zdata <= 0;
        end
    end

    // Reset frame decoding on CSN rising
    always @(posedge csn) begin
        have_cmd  <= 1'b0;
        have_addr <= 1'b0;
        bitcnt    <= 0;
        sh_in     <= 0;
        sh_out    <= 0;
        miso      <= 1'b0;
        full_byte <= 8'h00;
    end

    // Shift in MOSI on rising edge; byte-decode when 8 bits received
    // (SPI mode 0-ish: sample MOSI on posedge, drive MISO on negedge)
    always @(posedge sclk) begin
        if (!csn) begin
            sh_in   <= {sh_in[6:0], mosi};
            bitcnt  <= bitcnt + 1;

            if (bitcnt == 7) begin
                // Completed byte: assemble explicitly
                full_byte <= {sh_in[6:0], mosi};

                // Decode using the assembled byte.
                // IMPORTANT: since full_byte is assigned nonblocking above,
                // use the concatenation directly for decoding in this cycle.
                // (We keep full_byte for debug/visibility.)
                if (!have_cmd) begin
                    cmd      <= {sh_in[6:0], mosi};
                    have_cmd <= 1'b1;
                end else if (!have_addr) begin
                    addr      <= {sh_in[6:0], mosi};
                    have_addr <= 1'b1;

                    // Prepare first read byte immediately (shifts during next byte)
                    if (cmd == CMD_READ) begin
                        sh_out <= reg_read({sh_in[6:0], mosi});
                    end
                end else begin
                    // Data phase
                    if (cmd == CMD_WRITE) begin
                        reg_write(addr, {sh_in[6:0], mosi});
                        addr <= addr + 1;
                    end else if (cmd == CMD_READ) begin
                        // Each dummy clocks out current sh_out; now advance
                        addr   <= addr + 1;
                        sh_out <= reg_read(addr + 1);
                    end
                end

                // Reset bit counter for next byte
                bitcnt <= 0;
            end
        end
    end

    // Shift out MISO on falling edge
    always @(negedge sclk) begin
        if (!csn) begin
            miso   <= sh_out[7];
            sh_out <= {sh_out[6:0], 1'b0};
        end
    end

`else
    // Synthesis: tie off
    always @* miso = 1'b0;
`endif

endmodule
