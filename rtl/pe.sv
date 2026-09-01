// import pe_pkg::*;

module pe //#(
//    parameter DATA_WIDTH = 8,   // operand width
//    parameter ACC_WIDTH = 32    // accumulator width
//)
import pe_pkg::*;
(
    input logic clk,
    input logic rst_n,  // active low

    input logic signed [DATA_WIDTH-1:0] in_a,       // input from left neighbor
    input logic signed [DATA_WIDTH-1:0] in_b,       // input from top neighbor
    output logic signed [DATA_WIDTH-1:0] out_a,     // registered copy of left neighbor input to send to right neighbor (passing along operand)
    output logic signed [DATA_WIDTH-1:0] out_b,     // registered copy of top neighbor input to send to bottom neighbor

    input logic in_a_valid,                         // if input from left neighbor is valid
    input logic in_b_valid,                         // if input from top neighbor is valid
    output logic out_a_valid,                       // input a_valid into right neighbor, propagating rightwards
    output logic out_b_valid,                       // input b_valid into bottom neighbor, propagating downwards

    input logic in_first,                           // high when PE needs to reset for the next matmul; if first -> acc = mult_result otherwise acc = acc + mult_result
    output logic out_first,                         // in_first input into the next PEs; propagating acc "reset" through grid

    output logic signed [ACC_WIDTH-1:0] acc,        // running total sum, read at drain time. Output of the PE

    input logic capture,                            // pulse that copies current acc -> shadow_out; broadcasted right now so only one input (later will be input + output for propagating)
    input logic shift_en,                           // pulse that enables shifting (shadow_in -> shadow_out)
    input logic signed [ACC_WIDTH-1:0] in_shadow,   // the shadow buffer value shifted down by the top neighbor, copied to shadow_out while shift_en
    output logic signed [ACC_WIDTH-1:0] out_shadow  // the actual shadow buffer register of this PE, shifts to bottom neighbor while shift_en
);

logic in_valid;

assign in_valid = in_a_valid & in_b_valid;    // only if in_a and in_b are valid do the MAC, otherwise stay idle

assert property (@(posedge clk) !(capture && shift_en)) 
    else $error("capture and shift_en both HIGH at time %0t", $time);   // catching if capture and shift_en are asserted simultaneously; should never happen

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        {out_a, out_b} <= '0;
        {out_a_valid, out_b_valid} <= '0;
        out_first <= 1'b0;

        acc <= '0;
        out_shadow <= '0;
    end else begin
        if (capture) out_shadow <= acc;
        else if (shift_en) out_shadow <= in_shadow;
        else out_shadow <= out_shadow;
        
        if (in_valid) begin
            {out_a, out_b} <= {in_a, in_b};
            {out_a_valid, out_b_valid} <= {in_a_valid, in_b_valid};
            out_first <= in_first;

            acc <= (in_first) ? (in_a * in_b) : (acc + (in_a * in_b)); // multiply accumulate
        end else begin
            {out_a, out_b} <= '0;
            {out_a_valid, out_b_valid} <= {in_a_valid, in_b_valid};
            out_first <= 1'b0;

            acc <= acc;                 // acc keeps its value instead of resetting to 0
        end
    end
end

endmodule
