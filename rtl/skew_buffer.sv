module skew_buffer
import pe_pkg::*;
(
    input logic clk,
    input logic rst_n,  // active low

    input a_payload_t in_a [0:ARRAY_DIM-1],     // a column of A, starting from leftmost
    output a_payload_t out_a [0:ARRAY_DIM-1],

    input b_payload_t in_b [0:ARRAY_DIM-1],     // a row of B, starting from top
    output b_payload_t out_b [0:ARRAY_DIM-1]
);

// row i needs i delay registers -> max depth = ARRAY_DIM-1.
a_payload_t a_sr [0:ARRAY_DIM-1][0:ARRAY_DIM-1];
b_payload_t b_sr [0:ARRAY_DIM-1][0:ARRAY_DIM-1];

genvar i;

generate
    for (i = 0; i < ARRAY_DIM; i++) begin : g_row
        always_ff @(posedge clk) begin
            if (!rst_n) begin
                for (int d = 0; d < ARRAY_DIM; d++) a_sr[i][d] <= '0;
                for (int e = 0; e < ARRAY_DIM; e++) b_sr[i][e] <= '0;
            end else begin
                a_sr[i][0] <= in_a[i];
                b_sr[i][0] <= in_b[i];

                for (int d = 1; d < ARRAY_DIM; d++) a_sr[i][d] <= a_sr[i][d-1];
                for (int e = 1; e < ARRAY_DIM; e++) b_sr[i][e] <= b_sr[i][e-1];
            end
        end
        
        assign out_a[i] = a_sr[i][i];
        assign out_b[i] = b_sr[i][i];
    end
endgenerate

endmodule
