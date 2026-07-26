`timescale 1ns / 1ps

module msg_composer #(
    parameter PIXEL_WIDTH = 24, //8 bits per color
    parameter ROW_WIDTH = 10,
    parameter COL_WIDTH = 10
)(
    input logic [ROW_WIDTH - 1:0] row,
    input logic [COL_WIDTH - 1:0] col,
    input logic [PIXEL_WIDTH -1:0] pixel,

    output logic [15:0] [7:0] msg

);
import msg_pkg::*;

always_comb begin : msg_composer
    msg[0]  = CHAR_OPEN_BRACE;          // '{'
    msg[1]  = CHAR_R;                   // 'R'
    msg[2]  = 8'h00;                    // r2
    msg[3]  = {6'b0, row[9:8]};         // r1 
    msg[4]  = row[7:0];                 // r0 
    msg[5]  = CHAR_COMMA;               // ','
    msg[6]  = CHAR_C;                   // 'C'
    msg[7]  = 8'h00;                    // c2
    msg[8]  = {6'b0, col[9:8]};         // c1 
    msg[9]  = col[7:0];                 // c0 
    msg[10] = CHAR_COMMA;               // ','
    msg[11] = CHAR_P;                   // 'P'
    msg[12] = pixel[23:16];             // R channel
    msg[13] = pixel[15:8];              // G channel
    msg[14] = pixel[7:0];               // B channel
    msg[15] = CHAR_CLOSE_BRACE;         // '}'
end

endmodule