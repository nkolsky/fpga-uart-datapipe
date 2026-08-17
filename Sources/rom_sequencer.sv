// -----------------------------------------------------------------------------
// rom_sequencer.sv
//
// Reads the RGB SRAM image data in address order and pushes the pixels into the
// image FIFO. The read side still uses rom_* naming for compatibility, but the
// actual source is the 100 MHz SRAM memory domain.
//
// The sequencer waits for each read to complete, latches the four-pixel word,
// and emits pixels one at a time while the FIFO keeps backpressure on
// almost_full. It is started by mem_interlock.read_go and is gated by the
// memory-domain arbitration.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps
import rom_sequencer_pkg::*;

module rom_sequencer #(
    parameter IMG_WIDTH = 256,
    parameter IMG_HEIGHT = 256,
    parameter PIXELS_PER_WORD = 4, // Number of pixels stored in each ROM word (32 bits / 8 bits per pixel)
    parameter ROM_DEPTH = (IMG_WIDTH * IMG_HEIGHT) / PIXELS_PER_WORD, // 4 pixels per ROM word (32 bits for 4 pixels of 8 bits each)
    parameter ROM_DATA_WIDTH = 8 * PIXELS_PER_WORD, // Each ROM word is 32 bits (4 pixels of 8 bits each)
    parameter ADDR_WIDTH = (ROM_DEPTH > 1) ? $clog2(ROM_DEPTH) : 1, // Number of bits needed to address ROM_DEPTH words (log2(16,384) = 14 bits for 16,384 words)
    parameter ROM_LATENCY = 1 // Number of cycles it takes for ROM to output data after receiving read enable signal
)(
    input logic clk,
    input logic rst_n,

    //control signal from RGF
    input logic start,

    //signals from FIFO
    input logic almost_full, // Signal from FIFO indicating it's almost full
    input logic almost_empty, // Signal from FIFO indicating it's almost empty

    //pixel values from ROM
    input logic [ROM_DATA_WIDTH-1:0] red_data, // 32 bits for 4 pixels of red channel
    input logic [ROM_DATA_WIDTH-1:0] green_data, // 32 bits
    input logic [ROM_DATA_WIDTH-1:0] blue_data, // 32 bits

    //output signals for reading from ROM
    output logic [ADDR_WIDTH-1:0] rom_addr, // Address to read from ROM
    output logic rom_rd_en, // Signal to enable reading from ROM

    //ouput values and signals to FIFO
    output logic wr_en, // Write enable signal for FIFO
    output logic [23:0] wr_data,

    //output signal to RGF to let know if the reading from ROM is done
    output logic seq_done,

    //output high when sequencer is active, low when IDLE
    output logic busy

);

//internal state wires
state_t current_state, next_state;

//internal address counter to keep track of which ROM address to read from
logic [ADDR_WIDTH-1:0] addr_counter; // Counter to keep track of which ROM address is being accessed (0 to 16,383 for 16,384 words in ROM)

//internal push counter to keep track of which pixel within the ROM word is being processed
logic [$clog2(PIXELS_PER_WORD)-1:0] push_ctr; 

//internal register to hold the latched pixel data from ROM
logic [23:0] pixels [PIXELS_PER_WORD-1:0]; // Array to hold the 4 pixels (24 bits each) latched from ROM

//internal counter to track ROM latency cycles
logic [$clog2(ROM_LATENCY+1):0] latency_counter; 

//Sequential logic block
always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin

        current_state <= IDLE;
        addr_counter <= '0;
        push_ctr <= '0;
        latency_counter <= '0;
        pixels <= '{default: '0}; // Clear pixel registers on reset

    end else begin

        current_state <= next_state;

        case(current_state) 

        WAIT_ROM: begin
            if(latency_counter == ROM_LATENCY - 1) begin
                latency_counter <= '0; // Reset latency counter after waiting for ROM latency cycles
            end else begin
                latency_counter <= latency_counter + 1; // Increment latency counter while waiting for ROM to output data
            end
        end
        LATCH: begin
            // Latch the pixel data from memory into internal registers.
            //
            // BYTE-LANE ORIENTATION -- the MSB lane is the LEFTMOST pixel.
            // Each 32-bit channel word packs four CONSECUTIVE pixels of one
            // colour channel, lowest pixel index in the most significant
            // byte (see memory_pkg.sv). pixels[] is pushed to the FIFO in
            // index order 0,1,2,3, so pixels[0] must be the leftmost pixel
            // of the word, which is bits [31:24].
            //
            // These comments previously read "Pixel 3" against [31:24] and
            // "Pixel 0" against [7:0] -- i.e. exactly backwards. The RTL was
            // always right; only the labels were wrong, which is why the
            // captured image never looked scrambled. Corrected here because
            // sram_wr_ctrl.sv derives its byte enable from this same
            // orientation, and a reader who trusted the old labels would
            // build the write path mirrored within every word.
            //
            // Confirmed by measurement: red word 4392 = 0x3D0000FF, and the
            // captured PNG at row 68, cols 160..163 reads 61, 0, 0, 255 --
            // [31:24] first.
            pixels[0] <= {red_data[31:24], green_data[31:24], blue_data[31:24]}; // pixel 0 of word - LEFTMOST
            pixels[1] <= {red_data[23:16], green_data[23:16], blue_data[23:16]}; // pixel 1 of word
            pixels[2] <= {red_data[15:8], green_data[15:8], blue_data[15:8]}; // pixel 2 of word
            pixels[3] <= {red_data[7:0], green_data[7:0], blue_data[7:0]}; // pixel 3 of word - RIGHTMOST

        end
        PUSH: begin
            push_ctr <= push_ctr + 1; // Increment push counter to move to the next pixel in the current ROM word
        end
        NEXT_ADDR: begin
            push_ctr <= 0; // Reset push counter to start pushing from the first pixel
            addr_counter <= addr_counter + 1; // Increment address counter to move to the next ROM word
        end
        default: ; // For other states, no sequential updates needed

        endcase
    end
end 


//next state logic block
always_comb begin : next_state_logic
    //default
    next_state = current_state;
    
    case(current_state)
        IDLE: begin
             if(start) begin
                next_state = READ_ROM;
            end else begin
                next_state = IDLE;
            end
        end
        READ_ROM: begin
            next_state = WAIT_ROM; // After issuing read enable, wait for ROM latency cycles before latching data
        end
        WAIT_ROM: begin
            if(latency_counter == ROM_LATENCY - 1) begin
                next_state = LATCH; // After waiting for ROM latency, move to latch state to capture pixel data
            end else begin
                next_state = WAIT_ROM; // Keep waiting until ROM latency cycles have passed
            end
        end
        LATCH: begin
            next_state = PUSH; // After latching pixel data, move to push state to write pixels to FIFO
        end
        PUSH: begin
            if(push_ctr == ($bits(push_ctr))'(PIXELS_PER_WORD - 1)) begin
                next_state = NEXT_ADDR; // After pushing all pixels from the current ROM word, move to next address state
            end else begin
                next_state = PUSH; // Keep pushing pixels until all pixels from the current ROM word have been processed
            end
        end
        NEXT_ADDR: begin 
            /* verilator lint_off WIDTHEXPAND */
            if (addr_counter == ($bits(addr_counter))'(ROM_DEPTH - 1)) begin
            /* verilator lint_on WIDTHEXPAND */
                next_state = SEQ_DONE; // If we've reached the last ROM address, we're done
            end else if(almost_full) begin
                next_state = WAIT_DRAIN; // If FIFO is almost full, wait until it's drained before pushing more pixels
            end else begin
                next_state = READ_ROM; // Otherwise, go back to read the next ROM word
            end
        end
        WAIT_DRAIN: begin
            if(almost_empty) begin
                next_state = READ_ROM; // Once FIFO is drained enough, go back to read the next ROM word
            end else begin
                next_state = WAIT_DRAIN; // Keep waiting until FIFO is drained
            end
        end
        SEQ_DONE: begin
            next_state = IDLE; // Stay in done state until reset
        end
        default: begin
            next_state = IDLE; // Default state
        end
    endcase
end : next_state_logic


//output logic 
always_ff @(posedge clk or negedge rst_n) begin : output_logic
    if(!rst_n) begin
        rom_rd_en <= 1'b0;
        rom_addr  <= '0;
        wr_en     <= 1'b0;
        wr_data   <= '0;
        seq_done  <= 1'b0;
    end else begin
        
        // defaults
        rom_rd_en <= 1'b0;
        rom_addr  <= '0;
        wr_en     <= 1'b0;
        wr_data   <= '0;
        seq_done  <= 1'b0;

        case(current_state)
            READ_ROM: begin
                rom_rd_en <= 1; // Assert read enable to start reading from ROM
                rom_addr <= addr_counter; // Set ROM address to current value of address counter
            end
            PUSH: begin
                wr_en <= 1; // Enable writing to FIFO
                wr_data <= pixels[push_ctr]; // Write the current pixel data to FIFO
            end
            SEQ_DONE: begin
                seq_done <= 1; // Signal that the sequence is done
            end
            default: ;
        endcase
    end
end : output_logic

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        busy <= 1'b0;
    end else begin
        busy <= (next_state != IDLE);
    end
end
endmodule
