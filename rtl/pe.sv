module pe 
import pe_pkg::*;
(
    input logic clk,
    input logic rst_n,  // active low

    input a_payload_t in_a,     // inputted a payload; contains first, valid, and data
    output a_payload_t out_a,   // outputted a payload (registered)

    input b_payload_t in_b,     // inputted a payload; contains valid, and data
    output b_payload_t out_b,   // outputted b payload (registered)

    output logic signed [ACC_WIDTH-1:0] acc,        // running total sum, read at drain time. Output of the PE

    input logic capture,                            // pulse that copies current acc -> shadow_out; broadcasted right now so only one input (later will be input + output for propagating)
    input logic shift_en,                           // pulse that enables shifting (shadow_in -> shadow_out)
    input logic signed [ACC_WIDTH-1:0] in_shadow,   // the shadow buffer value shifted down by the top neighbor, copied to shadow_out while shift_en
    output logic signed [ACC_WIDTH-1:0] out_shadow  // the actual shadow buffer register of this PE, shifts to bottom neighbor while shift_en
);

logic in_valid;
assign in_valid = in_a.valid & in_b.valid;    // only if in_a and in_b are valid do the MAC, otherwise stay idle

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // setting control signals to zero; data is don't care
        out_a.valid <= 1'b0;
        out_a.first <= 1'b0;
        out_b.valid <= 1'b0;

        acc <= '0;
        out_shadow <= '0;
    end else begin
        // shadow buffer logic
        if (capture) out_shadow <= acc;
        else if (shift_en) out_shadow <= in_shadow;
        else out_shadow <= out_shadow;

        // operands propagate as long as rst_n is HIGH
        out_a <= in_a;
        out_b <= in_b;
        
        // valid logic
        if (in_valid) acc <= (in_a.first) ? (in_a.data * in_b.data) : (acc + (in_a.data * in_b.data));  // multiply accumulate; if in_first, reset to multiply product
        else acc <= acc; // acc keeps its value instead of resetting to 0
    end
end

endmodule
