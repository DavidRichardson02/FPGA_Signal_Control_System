`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// uart_keyboard_controller.v
// ----------------------------------------------------------------------------
// Purpose:
//   Lightweight UART command decoder that maps simple ASCII commands into
//   debounced-equivalent pulses and sticky overrides. This enables driving
//   front-panel button/switch behaviors from a USB keyboard connected through
//   the Nexys A7 USB-UART bridge (FTDI channel).
//
// Protocol:
//   Commands are framed as two ASCII characters: a leading '!' followed by a
//   command letter. Both bytes are consumed (dropped) from downstream UART
//   consumers when recognized.
//     !S / !s : survey START pulse
//     !T / !t : survey STOP pulse
//     !F / !f : manual FAN toggle pulse (BTN D equivalent)
//     !M / !m : toggle MANUAL enable override (sw_manual_en)
//     !P / !p : toggle PIR enable override    (sw_pir_en)
//     !U / !u : toggle TEMP enable override   (sw_temp_en)
//     !A / !a : toggle SURVEY MANUAL override (sw_survey_manual)
//     !L / !l : toggle LOGO select override   (sw_logo_sel)
//     !R / !r : clear all overrides to 0
//
// Outputs:
//   - Pulses assert for one clk cycle when their command is received.
//   - Override levels hold their state until toggled or cleared.
//   - consume drops command bytes from other UART RX consumers.
//
// Notes:
//   - The module is deliberately simple (no multi-byte buffering); it expects
//     at most one rx_vld pulse per byte from a parallel uart_rx block.
//   - Non-command bytes pass through to other logic; only the '!' prefix and
//     the immediate command byte are consumed.
// ============================================================================

module uart_keyboard_controller (
    input  wire clk,
    input  wire rst,

    input  wire [7:0] rx_byte,
    input  wire       rx_vld,

    output reg        consume,

    output reg  [7:0] last_cmd_byte,
    output reg        cmd_pulse,

    output reg        sweep_start_pulse,
    output reg        sweep_stop_pulse,
    output reg        fan_toggle_pulse,

    output reg        sw_temp_en_override,
    output reg        sw_manual_en_override,
    output reg        sw_pir_en_override,
    output reg        sw_survey_manual_override,
    output reg        sw_logo_sel_override
);

    reg awaiting_cmd = 1'b0;

    always @(posedge clk) begin
        if (rst) begin
            awaiting_cmd               <= 1'b0;
            consume                    <= 1'b0;
            last_cmd_byte              <= 8'h2D; // '-'
            cmd_pulse                  <= 1'b0;
            sweep_start_pulse          <= 1'b0;
            sweep_stop_pulse           <= 1'b0;
            fan_toggle_pulse           <= 1'b0;
            sw_temp_en_override        <= 1'b0;
            sw_manual_en_override      <= 1'b0;
            sw_pir_en_override         <= 1'b0;
            sw_survey_manual_override  <= 1'b0;
            sw_logo_sel_override       <= 1'b0;
        end else begin
            // Default to deasserted pulses/consume; set for one cycle on events.
            consume           <= 1'b0;
            cmd_pulse         <= 1'b0;
            sweep_start_pulse <= 1'b0;
            sweep_stop_pulse  <= 1'b0;
            fan_toggle_pulse  <= 1'b0;

            if (rx_vld) begin
                if (awaiting_cmd) begin
                    // Second byte following '!'
                    consume      <= 1'b1;
                    awaiting_cmd <= 1'b0;

                    case (rx_byte)
                        "S", "s": begin
                            sweep_start_pulse <= 1'b1;
                            last_cmd_byte     <= rx_byte;
                            cmd_pulse         <= 1'b1;
                        end
                        "T", "t": begin
                            sweep_stop_pulse <= 1'b1;
                            last_cmd_byte    <= rx_byte;
                            cmd_pulse        <= 1'b1;
                        end
                        "F", "f": begin
                            fan_toggle_pulse <= 1'b1;
                            last_cmd_byte    <= rx_byte;
                            cmd_pulse        <= 1'b1;
                        end
                        "M", "m": begin
                            sw_manual_en_override <= ~sw_manual_en_override;
                            last_cmd_byte         <= rx_byte;
                            cmd_pulse             <= 1'b1;
                        end
                        "P", "p": begin
                            sw_pir_en_override <= ~sw_pir_en_override;
                            last_cmd_byte      <= rx_byte;
                            cmd_pulse          <= 1'b1;
                        end
                        "U", "u": begin
                            sw_temp_en_override <= ~sw_temp_en_override;
                            last_cmd_byte       <= rx_byte;
                            cmd_pulse           <= 1'b1;
                        end
                        "A", "a": begin
                            sw_survey_manual_override <= ~sw_survey_manual_override;
                            last_cmd_byte             <= rx_byte;
                            cmd_pulse                 <= 1'b1;
                        end
                        "L", "l": begin
                            sw_logo_sel_override <= ~sw_logo_sel_override;
                            last_cmd_byte        <= rx_byte;
                            cmd_pulse            <= 1'b1;
                        end
                        "R", "r": begin
                            sw_temp_en_override       <= 1'b0;
                            sw_manual_en_override     <= 1'b0;
                            sw_pir_en_override        <= 1'b0;
                            sw_survey_manual_override <= 1'b0;
                            sw_logo_sel_override      <= 1'b0;
                            last_cmd_byte             <= rx_byte;
                            cmd_pulse                 <= 1'b1;
                        end
                        default: ;
                    endcase
                end else if (rx_byte == 8'h21 /* '!' */) begin
                    // Start of a command sequence
                    awaiting_cmd <= 1'b1;
                    consume      <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
