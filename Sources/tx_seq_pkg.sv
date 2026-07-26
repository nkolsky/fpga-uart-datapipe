`timescale 1ps/1ps
package tx_seq_pkg;
    typedef enum logic [3:0] { 
        IDLE        = 4'b0000,
        POP_FIFO    = 4'b0001,
        WAIT_DATA   = 4'b0010,
        LATCH       = 4'b0011,
        WAIT_MAC    = 4'b0100,
        SEND        = 4'b0101,
        WAIT_BUSY   = 4'b0110,
        WAIT_DONE   = 4'b0111,
        NEXT        = 4'b1000,
        DONE        = 4'b1001    
    } state_t;
endpackage