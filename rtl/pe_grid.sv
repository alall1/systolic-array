module pe_grid #(
    parameter DATA_WIDTH = 8,   // width of data inputs
    parameter ACC_WIDTH = 32,   // width of accumulators within PEs
    parameter N = 3             // generating an N x N array
)(
    input logic clk,
    input logic rst_n,
    input logic inp_valid,                                  // are the current a_in & b_in real values or filler
    input logic signed [DATA_WIDTH-1:0] a_in [0:N-1],       // the current column of matrix A entering the grid; a_in[0] is the first row, a_in[1] is the second, etc.
    input logic signed [DATA_WIDTH-1:0] b_in [0:N-1],       // the current row of matrix B entering the grid; b_in[0] is the first column, b_in[1] is the second, etc.
    output logic signed [ACC_WIDTH-1:0] out [0:N-1][0:N-1], // the output of the grid, the resulting matrix; ready when done = 1
    output logic done                                       // when the matmul is done
);

logic [DATA_WIDTH-1:0] a_bus [0:N-1][0:N];      // bus of all horizontal connections between PEs, 0:N columns because of rightmost PEs' out_a / out_b signals (unused)
logic [DATA_WIDTH-1:0] b_bus [0:N][0:N-1];      // bus of all vertical connections between PEs, O:N rows because of bottom PEs' out_a / out_b signals

localparam int FINISH = 3*N - 2;                // baked-in number of clock cycles until the NxN matmul is finished. 3N - 2, because only doing NxN * NxN matmuls FOR NOW. Change to K + 2(N - 1) later
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
                .out_a(a_bus[row][col+1]),          // giving A value to right neighbor
                .out_b(b_bus[row+1][col]),          // giving B value to bottom neighbor
                .acc(out[row][col]),
                .out_valid()                        // unassigned for now, potentially used for propagating valid from PE to PE instead of broadcasting all at once
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

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        count <= '0;
        done <= 1'b0;
    end else if (count == CNT_WIDTH'(FINISH - 1)) begin   // when 3N - 2 clock cycles have passed and the matmul is finished (out is ready to read)
        count <= count;
        done <= 1'b1;
    end else begin
        count <= count + 1;
        done <= 1'b0;
    end
end

endmodule
