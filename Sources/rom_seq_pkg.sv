package rom_sequencer_pkg;
    typedef enum logic [2:0] {
        IDLE       = 3'b000,
        READ_ROM   = 3'b001,
        WAIT_ROM   = 3'b010,
        LATCH      = 3'b011,
        PUSH       = 3'b100,
        NEXT_ADDR  = 3'b101,
        WAIT_DRAIN = 3'b110,
        SEQ_DONE   = 3'b111
    } state_t;
endpackage