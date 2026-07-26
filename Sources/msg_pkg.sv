`timescale 1ps/1ps
package msg_pkg;
//ASCII Constants for validating message
localparam [7:0] CHAR_OPEN_BRACE = 8'h7B; // '{'
localparam [7:0] CHAR_CLOSE_BRACE = 8'h7D; // '}'
localparam [7:0] CHAR_R = 8'h52; // 'R'
localparam [7:0] CHAR_C = 8'h43; // 'C'
localparam [7:0] CHAR_V = 8'h56; // 'V'
localparam [7:0] ASCII_ZERO = 8'h30; // '0'
localparam [7:0] CHAR_COMMA = 8'h2C; // ','
localparam [7:0] CHAR_P = 8'h50; //'P'

endpackage : msg_pkg