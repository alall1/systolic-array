module read_sequencer
import pe_pkg::*;
(
    input logic clk,
    input logic rst_n,  // active low

    input logic new,    // signaling new read

    input logic [x:0] M,
    input logic [x:0] K,
 
    output a_payload_t out_a [0:ARRAY_DIM-1],   // a column of A, starting from leftmost -> skew_buffer
    output b_payload_t out_b [0:ARRAY_DIM-1]    // a row of B, starting from top -> skew_buffer
);


on start:
    latch M, N, K, a_base, a_stride, b_base, b_stride   # hold stable for whole feed
    k <- 0
    busy <- 1

each cycle while busy:
    for lane in 0 .. n-1:
        # ---- A lane: element A[lane][k]  = column k, all rows ----
        if lane < M and k < K:
            out_a[lane].data  = buffer[ a_base + lane * a_stride + k ]
            out_a[lane].valid = 1
        else:
            out_a[lane].data  = 'x        # don't-care; valid gates it
            out_a[lane].valid = 0

        # ---- B lane: element B[k][lane]  = row k, all cols ----
        if lane < N and k < K:
            out_b[lane].data  = buffer[ b_base + k * b_stride + lane ]
            out_b[lane].valid = 1
        else:
            out_b[lane].data  = 'x
            out_b[lane].valid = 0

    first = (k == 0)          # see note on where this lives

    if k == K-1:
        busy <- 0
        done <- 1             # hand off to drain
    k <- k + 1

endmodule