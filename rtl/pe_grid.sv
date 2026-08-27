module pe_grid #(
    parameter DATA_WIDTH = 8,   // width of data inputs
    parameter ACC_WIDTH = 32,   // width of accumulators within PEs
    parameter N = 3             // generating an N x N array
)(
    input logic clk,
    input logic rst_n,
    input logic inp_valid,                                  // are the current a_in & b_in real values or filler
    input logic inp_first,                                  // high when starting a new matmul, comes from feeder
    input logic signed [DATA_WIDTH-1:0] a_in [0:N-1],       // the current column of matrix A entering the grid; a_in[0] is the first row, a_in[1] is the second, etc.
    input logic signed [DATA_WIDTH-1:0] b_in [0:N-1],       // the current row of matrix B entering the grid; b_in[0] is the first column, b_in[1] is the second, etc.
    output logic signed [ACC_WIDTH-1:0] out [0:N-1][0:N-1], // the output of the grid, the resulting matrix; ready when out_ready = 1
    output logic out_ready                                  // when outputs are ready to read; for now, after K + 2(N-1) cycles, when all PEs are done, but for future, after 2(N-1) cycles first PE output finishes and can be read in a ripple fashion
);

logic [DATA_WIDTH-1:0] a_bus [0:N-1][0:N];      // bus of all horizontal connections between PEs, 0:N columns because of rightmost PEs' out_a / out_b signals (unused)
logic [DATA_WIDTH-1:0] b_bus [0:N][0:N-1];      // bus of all vertical connections between PEs, O:N rows because of bottom PEs' out_a / out_b signals
logic in_first_bus [0:N][0:N];                  // bus of all PE "in_first" pins; O:N rows AND columns because each PE asserts "first" to the right and bottom adjacent PEs
assign in_first_bus[0][0] = inp_first;          // assigning the top-left PE (first to receive operands) to grid-level inp_first, which will then ripple through PEs, effectively "resetting" them for the next matmul

logic out_first_bus [0:N-1][0:N-1];             // bus of all PE "out_first" pins; each corresponds to a single PE; drives in_first_bus

localparam int FINISH = 3*N - 2;                // baked-in number of clock cycles until the NxN matmul is finished. 3N - 2, because only doing NxN * NxN matmuls FOR NOW. Change to 2(N - 1) later for ripple output
localparam int CNT_WIDTH = $clog2(FINISH + 1);  // calculating the number of bits needed to count up to FINISH; indexed at 0 so counter = FINISH - 1 when complete

logic [CNT_WIDTH-1:0] count;

genvar col, row;

generate
    for (row = 0; row < N; row++) begin : row_loop
        for (col = 0; col < N; col++) begin : col_loop
            pe #(
                .DATA_WIDTH(DATA_WIDTH),
                .ACC_WIDTH(ACC_WIDTH)
            ) pe (
                .clk(clk),
                .rst_n(rst_n),
                .in_a(a_bus[row][col]),             // from left neighbor [row][col-1]
                .in_b(b_bus[row][col]),             // from top neighbor [row-1][col]
                .in_valid(inp_valid),
                .in_first(in_first_bus[row][col]),  // from top OR left neighbor, continuous assign
                .out_a(a_bus[row][col+1]),          // giving A value to right neighbor
                .out_b(b_bus[row+1][col]),          // giving B value to bottom neighbor
                .acc(out[row][col]),
                .out_valid(),                       // unassigned for now, potentially used for propagating valid from PE to PE instead of broadcasting all at once
                .out_first(out_first_bus[row][col]) // becomes first[row+1][col] AND first[row][col+1], propagating first signal from PE to PE
            );
        end : col_loop
    end : row_loop
endgenerate

always_comb begin
    for (int i = 0; i < N; i++) begin
        a_bus[i][0] = a_in[i];
    end
    for (int j = 0; j < N; j++) begin
        b_bus[0][j] = b_in[j];
    end
end

always_comb begin
    for (int i = 0; i < N; i++) begin
        for (int j = 0; j < N; j++) begin
            if (i == 0) in_first_bus[i+1][j] = out_first_bus[i][j];     // only top row "first" signas propagate left; the rest propagate downward
            in_first_bus[i][j+1] = out_first_bus[i][j];
        end
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        count <= '0;
        out_ready <= 1'b0;
    end else if (count == CNT_WIDTH'(FINISH - 1)) begin   // when 3N - 2 clock cycles have passed and the matmul is finished (out is ready to read)
        count <= count;
        out_ready <= 1'b1;
    end else if (inp_first) begin
        count <= '0;
        out_ready <= 1'b0;
    end else begin
        count <= count + 1;
        out_ready <= 1'b0;
    end
end

endmodule
