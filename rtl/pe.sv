module pe #(
    parameter DATA_WIDTH = 8,   // operand width
    parameter ACC_WIDTH = 32    // accumulator width
)(
    input logic clk,
    input logic rst_n,  // active low
    input logic signed [DATA_WIDTH-1:0] in_a,   // input from left neighbor
    input logic signed [DATA_WIDTH-1:0] in_b,   // input from top neighbor
    input logic in_a_valid,                     // if input from left neighbor is valid
    input logic in_b_valid,                     // if input from top neighbor is valid
    input logic in_first,                       // high when PE needs to reset for the next matmul; if first -> acc = mult_result otherwise acc = acc + mult_result
    output logic signed [DATA_WIDTH-1:0] out_a, // registered copy of left neighbor input to send to right neighbor (passing along operand)
    output logic signed [DATA_WIDTH-1:0] out_b, // registered copy of top neighbor input to send to bottom neighbor
    output logic signed [ACC_WIDTH-1:0] acc,    // running total sum, read at drain time. Output of the PE
    output logic out_a_valid,                   // input a_valid into right neighbor, propagating rightwards
    output logic out_b_valid,                   // input b_valid into bottom neighbor, propagating downwards
    output logic out_first                      // in_first input into the next PEs; propagating acc "reset" through grid                  
);

logic in_valid;

assign in_valid = in_a_valid & in_b_valid;    // only if in_a and in_b are valid do the MAC, otherwise stay idle

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_a <= '0;
        out_b <= '0;
        acc <= '0;
        out_a_valid <= 1'b0;
        out_b_valid <= 1'b0;
        out_first <= 1'b0;
    end else if (!in_valid) begin
        out_a <= '0;
        out_b <= '0;
        acc <= acc;                 // acc keeps its value instead of resetting to 0
        out_a_valid <= in_a_valid;
        out_b_valid <= in_b_valid;
        out_first <= 1'b0;
    end else begin
        out_a <= in_a;
        out_b <= in_b;
        acc <= (in_first) ? (in_a * in_b) : (acc + (in_a * in_b)); // multiply accumulate
        out_a_valid <= in_a_valid;
        out_b_valid <= in_b_valid;
        out_first <= in_first;
    end
end

endmodule
