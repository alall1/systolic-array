module read_sequencer
import pe_pkg::*;
(
    input logic clk,
    input logic rst_n,  // active low

    input logic start,

    output logic busy,
    output logic done,

    input logic [DIM_W-1:0] M,
    input logic [DIM_W-1:0] N,
    input logic [K_W-1:0] K,

    output logic rd_en,
    output logic [K_W-1:0] rd_col_a,    // which column of A (= k)
    output logic [K_W-1:0] rd_row_b,    // which row of B    (= k)

    // read from buffer
    input logic signed [DATA_WIDTH-1:0] rd_a [0:ARRAY_DIM-1],
    input logic signed [DATA_WIDTH-1:0] rd_b [0:ARRAY_DIM-1],

    output a_payload_t out_a [0:ARRAY_DIM-1],
    output b_payload_t out_b [0:ARRAY_DIM-1]
);
// config, captured at start and held until reset or done
logic [DIM_W-1:0] M_h, N_h;
logic [K_W-1:0] K_h;

// k counter and busy, and shadow (_d) aligning control with data returned from buffer one cycle after
logic [K_W-1:0] k;      // k (address issued this cycle)
logic [K_W-1:0] k_d;    // k (data arriving this cycle)
logic busy_d;           // was a real address issued last cycle?
logic last_d;           // did last cycle issue the final (k==K-1) address?

logic issuing, last;
assign issuing = busy && (k < K_h);   // a real address goes out this cycle
assign last = busy && (k == K_h - 1);

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        k <= '0;
        busy <= 1'b0;
        k_d <= '0;
        busy_d <= 1'b0;
        last_d <= 1'b0;
    end else begin
        // issue logic
        if (start && !busy) begin
            M_h <= M;
            N_h <= N;
            K_h <= K;
            k <= '0;
            busy <= 1'b1;
        end else if (busy) begin
            if (last) busy <= 1'b0; // feed ends after final address issued
            k <= k + 1'b1;
        end

        // delay logic
        k_d <= k;
        busy_d <= issuing;   // data next cycle is real if issued this cycle
        last_d <= last;
    end
end

// address generation, purely combinational; issued same cycle for k (one index per operand)
assign rd_en = issuing;
assign rd_col_a = k;
assign rd_row_b = k;

// payload assembly, purely combinational aligned to DATA cycle, wrapped with valid and first bits
always_comb begin
    for (int lane = 0; lane < ARRAY_DIM; lane++) begin
        // A lane: valid where the grid row exists (lane < M)
        if (busy_d && lane < M_h) begin
            out_a[lane].data = rd_a[lane];
            out_a[lane].valid = 1'b1;
            out_a[lane].first = (k_d == '0);
        end else begin
            out_a[lane].data = 'x;
            out_a[lane].valid = 1'b0;
            out_a[lane].first = 'x;
        end

        // B lane: valid where the grid col exists (lane < N)
        if (busy_d && lane < N_h) begin
            out_b[lane].data = rd_b[lane];
            out_b[lane].valid = 1'b1;
        end else begin
            out_b[lane].data = 'x;
            out_b[lane].valid = 1'b0;
        end
    end
end

// done asserted in the DATA cycle of last (last_d)
assign done = last_d;

endmodule
