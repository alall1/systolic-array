module pe #(
    parameter DATA_WIDTH = 8,   // operand width
    parameter ACC_WIDTH = 32    // accumulator width
)(
    input logic clk,
    input logic rst_n,  // active low
    input logic [DATA_WIDTH-1:0] in_a,      // input from left neighbor
    input logic [DATA_WIDTH-1:0] in_b,      // input from top neighbor
    input logic in_valid,                   // high when inputs are real data (high if PE is being used this cycle)
    output logic [DATA_WIDTH-1:0] out_a,    // registered copy of left neighbor input to send to right neighbor (passing along operand)
    output logic [DATA_WIDTH-1:0] out_b,    // registered copy of top neighbor input to send to bottom neighbor
    output logic [ACC_WIDTH-1:0] acc        // running total sum, read at drain time. Output of the PE
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_a <= '0;
        out_b <= '0;
        acc <= '0;
    end else begin
        out_a <= in_a;
        out_b <= in_b;
        acc <= acc + ($signed(in_a) * $signed(in_b));
    end
end

endmodule