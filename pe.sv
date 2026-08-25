module pe #(
    parameter DATA_WIDTH = 8,   // operand width
    parameter ACC_WIDTH = 32    // accumulator width
)(
    input logic clk,
    input logic rst_n,  // active low
    input logic signed [DATA_WIDTH-1:0] in_a,   // input from left neighbor
    input logic signed [DATA_WIDTH-1:0] in_b,   // input from top neighbor
    input logic in_valid,                       // high when inputs are real data (high if PE is being used this cycle)
    output logic [DATA_WIDTH-1:0] out_a,        // registered copy of left neighbor input to send to right neighbor (passing along operand)
    output logic [DATA_WIDTH-1:0] out_b,        // registered copy of top neighbor input to send to bottom neighbor
    output logic signed [ACC_WIDTH-1:0] acc,    // running total sum, read at drain time. Output of the PE
    output logic out_valid                      // in_valid input into the next PEs; operand validity, not accumulator sum validity
);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || !in_valid) begin
        out_a <= '0;
        out_b <= '0;
        acc <= '0;
        out_valid <= 1'b0;
    end else begin
        out_a <= in_a;
        out_b <= in_b;
        acc <= acc + (in_a * in_b); // multiply accumulate
        out_valid <= 1'b1;
    end
end

endmodule