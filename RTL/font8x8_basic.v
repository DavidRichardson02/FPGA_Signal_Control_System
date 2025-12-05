`timescale 1ns/1ps

// ============================================================================
// Module : font8x8_basic
// Role   : Tiny 8x8 monochrome font ROM for a small ASCII subset.
//          row_bits[7] is the LEFT-most pixel in the row.
// ============================================================================

module font8x8_basic (
    input  wire [7:0] char_code,   // ASCII code ('0'..'9','T','E',...,' ')
    input  wire [2:0] row,         // 0..7 (top..bottom)
    output reg  [7:0] row_bits     // bit[7]=leftmost, bit[0]=rightmost
);

    always @* begin
        // Default: blank row
        row_bits = 8'b0000_0000;

        case (char_code)
            // ---------------- Digits ----------------
            "0": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1100;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0100_0010;
                    3'd4: row_bits = 8'b0100_0010;
                    3'd5: row_bits = 8'b0100_0010;
                    3'd6: row_bits = 8'b0011_1100;
                    default: ;
                endcase
            end

            "1": begin
                case (row)
                    3'd1: row_bits = 8'b0001_1000;
                    3'd2: row_bits = 8'b0010_1000;
                    3'd3: row_bits = 8'b0000_1000;
                    3'd4: row_bits = 8'b0000_1000;
                    3'd5: row_bits = 8'b0000_1000;
                    3'd6: row_bits = 8'b0011_1110;
                    default: ;
                endcase
            end

            "2": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1100;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0000_0100;
                    3'd4: row_bits = 8'b0000_1000;
                    3'd5: row_bits = 8'b0001_0000;
                    3'd6: row_bits = 8'b0111_1110;
                    default: ;
                endcase
            end

            "3": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1100;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0000_1100;
                    3'd4: row_bits = 8'b0000_1100;
                    3'd5: row_bits = 8'b0100_0010;
                    3'd6: row_bits = 8'b0011_1100;
                    default: ;
                endcase
            end

            "5": begin
                case (row)
                    3'd1: row_bits = 8'b0111_1110;
                    3'd2: row_bits = 8'b0100_0000;
                    3'd3: row_bits = 8'b0111_1100;
                    3'd4: row_bits = 8'b0000_0010;
                    3'd5: row_bits = 8'b0100_0010;
                    3'd6: row_bits = 8'b0011_1100;
                    default: ;
                endcase
            end

            "6": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1100;
                    3'd2: row_bits = 8'b0100_0000;
                    3'd3: row_bits = 8'b0111_1100;
                    3'd4: row_bits = 8'b0100_0010;
                    3'd5: row_bits = 8'b0100_0010;
                    3'd6: row_bits = 8'b0011_1100;
                    default: ;
                endcase
            end

            "7": begin
                case (row)
                    3'd1: row_bits = 8'b0111_1110;
                    3'd2: row_bits = 8'b0000_0010;
                    3'd3: row_bits = 8'b0000_0100;
                    3'd4: row_bits = 8'b0000_1000;
                    3'd5: row_bits = 8'b0001_0000;
                    3'd6: row_bits = 8'b0001_0000;
                    default: ;
                endcase
            end

            "8": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1100;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0011_1100;
                    3'd4: row_bits = 8'b0100_0010;
                    3'd5: row_bits = 8'b0100_0010;
                    3'd6: row_bits = 8'b0011_1100;
                    default: ;
                endcase
            end

            "9": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1100;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0100_0010;
                    3'd4: row_bits = 8'b0011_1110;
                    3'd5: row_bits = 8'b0000_0010;
                    3'd6: row_bits = 8'b0011_1100;
                    default: ;
                endcase
            end

            // ---------------- Capital letters ----------------
            "T": begin
                case (row)
                    3'd1: row_bits = 8'b0111_1110;
                    3'd2: row_bits = 8'b0001_1000;
                    3'd3: row_bits = 8'b0001_1000;
                    3'd4: row_bits = 8'b0001_1000;
                    3'd5: row_bits = 8'b0001_1000;
                    3'd6: row_bits = 8'b0001_1000;
                    default: ;
                endcase
            end

            "E": begin
                case (row)
                    3'd1: row_bits = 8'b0111_1110;
                    3'd2: row_bits = 8'b0100_0000;
                    3'd3: row_bits = 8'b0111_1100;
                    3'd4: row_bits = 8'b0100_0000;
                    3'd5: row_bits = 8'b0100_0000;
                    3'd6: row_bits = 8'b0111_1110;
                    default: ;
                endcase
            end

            "M": begin
                case (row)
                    3'd1: row_bits = 8'b0100_0010;
                    3'd2: row_bits = 8'b0110_0110;
                    3'd3: row_bits = 8'b0101_1010;
                    3'd4: row_bits = 8'b0100_0010;
                    3'd5: row_bits = 8'b0100_0010;
                    3'd6: row_bits = 8'b0100_0010;
                    default: ;
                endcase
            end

            "P": begin
                case (row)
                    3'd1: row_bits = 8'b0111_1100;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0111_1100;
                    3'd4: row_bits = 8'b0100_0000;
                    3'd5: row_bits = 8'b0100_0000;
                    3'd6: row_bits = 8'b0100_0000;
                    default: ;
                endcase
            end

            "A": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1100;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0100_0010;
                    3'd4: row_bits = 8'b0111_1110;
                    3'd5: row_bits = 8'b0100_0010;
                    3'd6: row_bits = 8'b0100_0010;
                    default: ;
                endcase
            end

            "N": begin
                case (row)
                    3'd1: row_bits = 8'b0100_0010;
                    3'd2: row_bits = 8'b0110_0010;
                    3'd3: row_bits = 8'b0101_0010;
                    3'd4: row_bits = 8'b0100_1010;
                    3'd5: row_bits = 8'b0100_0110;
                    3'd6: row_bits = 8'b0100_0010;
                    default: ;
                endcase
            end

            "I": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1100;
                    3'd2: row_bits = 8'b0001_1000;
                    3'd3: row_bits = 8'b0001_1000;
                    3'd4: row_bits = 8'b0001_1000;
                    3'd5: row_bits = 8'b0001_1000;
                    3'd6: row_bits = 8'b0011_1100;
                    default: ;
                endcase
            end

            "R": begin
                case (row)
                    3'd1: row_bits = 8'b0111_1100;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0111_1100;
                    3'd4: row_bits = 8'b0100_1000;
                    3'd5: row_bits = 8'b0100_0100;
                    3'd6: row_bits = 8'b0100_0010;
                    default: ;
                endcase
            end

            "F": begin
                case (row)
                    3'd1: row_bits = 8'b0111_1110;
                    3'd2: row_bits = 8'b0100_0000;
                    3'd3: row_bits = 8'b0111_1100;
                    3'd4: row_bits = 8'b0100_0000;
                    3'd5: row_bits = 8'b0100_0000;
                    3'd6: row_bits = 8'b0100_0000;
                    default: ;
                endcase
            end



            "O": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1100;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0100_0010;
                    3'd4: row_bits = 8'b0100_0010;
                    3'd5: row_bits = 8'b0100_0010;
                    3'd6: row_bits = 8'b0011_1100;
                    default: ;
                endcase
            end

            "S": begin
                case (row)
                    3'd1: row_bits = 8'b0011_1110;
                    3'd2: row_bits = 8'b0100_0000;
                    3'd3: row_bits = 8'b0011_1100;
                    3'd4: row_bits = 8'b0000_0010;
                    3'd5: row_bits = 8'b0000_0010;
                    3'd6: row_bits = 8'b0111_1100;
                    default: ;
                endcase
            end

            "V": begin
                case (row)
                    3'd1: row_bits = 8'b0100_0010;
                    3'd2: row_bits = 8'b0100_0010;
                    3'd3: row_bits = 8'b0100_0010;
                    3'd4: row_bits = 8'b0010_0100;
                    3'd5: row_bits = 8'b0010_0100;
                    3'd6: row_bits = 8'b0001_1000;
                    default: ;
                endcase
            end

            "Y": begin
                case (row)
                    3'd1: row_bits = 8'b0100_0010;
                    3'd2: row_bits = 8'b0010_0100;
                    3'd3: row_bits = 8'b0001_1000;
                    3'd4: row_bits = 8'b0001_1000;
                    3'd5: row_bits = 8'b0001_1000;
                    3'd6: row_bits = 8'b0001_1000;
                    default: ;
                endcase
            end




           "4": begin
                case (row)
                    3'd1: row_bits = 8'b0000_1100;
                    3'd2: row_bits = 8'b0001_0100;
                    3'd3: row_bits = 8'b0010_0100;
                    3'd4: row_bits = 8'b0111_1110;
                    3'd5: row_bits = 8'b0000_0100;
                    3'd6: row_bits = 8'b0000_0100;
                    default: ;
                endcase
            end


            // Space
            " ": begin
                // all zeros (already default)
            end

            default: begin
                // unimplemented characters → blank
            end
        endcase
    end

endmodule
