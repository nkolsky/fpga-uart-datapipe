// rx_classifier.sv
// ----------------
// Registered latch of rx_parser outputs.
//
// Lab 9 requirement: "Classifier: Sampled → Holds the opcode and values
// for the sequencer after a valid parsing indication"
//
// Latches all parser fields on the cycle where BOTH:
//   msg_valid  (from rx_mac)  — a complete message was received
//   parse_valid (from rx_parser) — the message format is correct
//
// Outputs hold their values until the next valid message arrives.
// classifier_valid pulses for one cycle to notify the sequencer.
//
// On parse_error: classifier_valid stays low, outputs retain previous
// values, and classifier_error pulses for one cycle so the sequencer
// can optionally log/count framing errors.

`timescale 1ns/1ps

module rx_classifier (
    input  logic        clk,
    input  logic        rst_n,

    // From rx_mac
    input  logic        msg_valid,    // one-cycle pulse: new message available

    // From rx_parser (combinational, stable on msg_valid)
    input  logic        parse_valid,  // message format is correct
    input  logic        parse_error,  // message format is wrong
    input  logic [9:0]  row,
    input  logic [9:0]  col,
    input  logic [23:0] pixel,

    // To sequencer
    output logic        classifier_valid,  // one-cycle pulse: new valid command latched
    output logic        classifier_error,  // one-cycle pulse: framing error received
    output logic [9:0]  row_q,             // latched row (holds until next valid msg)
    output logic [9:0]  col_q,             // latched col
    output logic [23:0] pixel_q            // latched pixel {R,G,B}
);

// -------------------------------------------------------------------------
// Latch fields on valid message
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        row_q   <= 10'd0;
        col_q   <= 10'd0;
        pixel_q <= 24'd0;
    end else if (msg_valid && parse_valid) begin
        row_q   <= row;
        col_q   <= col;
        pixel_q <= pixel;
    end
end

// -------------------------------------------------------------------------
// Registered one-cycle output pulses (Moore style, matches rest of design)
// -------------------------------------------------------------------------
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        classifier_valid <= 1'b0;
        classifier_error <= 1'b0;
    end else begin
        classifier_valid <= msg_valid && parse_valid;
        classifier_error <= msg_valid && parse_error;
    end
end

endmodule : rx_classifier
