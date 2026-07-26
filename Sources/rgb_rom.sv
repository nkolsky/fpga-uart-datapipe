module rom #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 16384,
    parameter MEM_FILE   = ""
)(
    input  logic                         clk,
    input  logic                         en,
    input  logic [$clog2(DEPTH)-1:0]     addr,
    output logic [DATA_WIDTH-1:0]        dout
);

logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

initial begin
    $readmemh(MEM_FILE, mem);
end

// Registered output: 1 cycle latency
always_ff @(posedge clk) begin
    if (en)
        dout <= mem[addr];
end

endmodule : rom