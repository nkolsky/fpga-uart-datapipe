module reset_synch (
    input logic clk,
    input logic async_rst_n,
    output logic sync_rst_n
);

logic rst_n_ff1, rst_n_ff2;

// Synchronize the asynchronous reset signal to the clock domain
always_ff @(posedge clk or negedge async_rst_n) begin
    if (!async_rst_n) begin // If the asynchronous reset is asserted, reset both flip-flops
        rst_n_ff1 <= 1'b0;
        rst_n_ff2 <= 1'b0;
    end else begin // Otherwise, propagate the reset signal through the flip-flops
        rst_n_ff1 <= 1'b1;
        rst_n_ff2 <= rst_n_ff1;
    end
end

assign sync_rst_n = rst_n_ff2;


endmodule