// import pe_pkg::*;

module pe_grid //#(
    //parameter DATA_WIDTH = 8,   // width of data inputs
    //parameter ACC_WIDTH = 32,   // width of accumulators within PEs
    //parameter N = 3             // generating an N x N array
//)
import pe_pkg::*;
(
    input logic clk,
    input logic rst_n,
    input logic inp_first,                                                  // high when starting a new matmul, comes from feeder
    input logic signed [DATA_WIDTH-1:0] a_in [0:ARRAY_DIM-1],               // the current column of matrix A entering the grid; a_in[0] is the first row, a_in[1] is the second, etc.
    input logic signed [DATA_WIDTH-1:0] b_in [0:ARRAY_DIM-1],               // the current row of matrix B entering the grid; b_in[0] is the first column, b_in[1] is the second, etc.
    input logic a_valid [0:ARRAY_DIM-1],                                    // the current valid signal column of matrix A entering the grid
    input logic b_valid [0:ARRAY_DIM-1],                                    // the current valid signal column of matrix B entering the grid
    output logic signed [ACC_WIDTH-1:0] out [0:ARRAY_DIM-1][0:ARRAY_DIM-1], // the output of the grid, the resulting matrix; ready when out_ready = 1
    output logic out_ready                                                  // when outputs are ready to read; for now, after K + 2(ARRAY_DIM-1) cycles, when all PEs are done, but for future, after 2(ARRAY_DIM-1) cycles first PE output finishes and can be read in a ripple fashion
);

localparam int FINISH = 3*ARRAY_DIM - 2;                // baked-in number of clock cycles until the NxN matmul is finished. 3N - 2, because only doing NxN * NxN matmuls FOR NOW. Change to 2(N - 1) later for ripple output
localparam int CNT_WIDTH = $clog2(FINISH + 1);  // calculating the number of bits needed to count up to FINISH; indexed at 0 so counter = FINISH - 1 when complete

logic [DATA_WIDTH-1:0] a_bus [0:ARRAY_DIM-1][0:ARRAY_DIM];      // bus of all horizontal connections between PEs, 0:N columns because of rightmost PEs' out_a / out_b signals (unused)
logic [DATA_WIDTH-1:0] b_bus [0:ARRAY_DIM][0:ARRAY_DIM-1];      // bus of all vertical connections between PEs, O:N rows because of bottom PEs' out_a / out_b signals

logic a_valid_bus [0:ARRAY_DIM-1][0:ARRAY_DIM];                 // bus of all horizontal valid signal connections between PEs, similar to a_bus (a_valid propagates with a's value)
logic b_valid_bus [0:ARRAY_DIM][0:ARRAY_DIM-1];                 // bus of all vertical valid signal connections between PEs, similar to b_bus (b_valid propagates with b's value)

logic in_first_bus [0:ARRAY_DIM][0:ARRAY_DIM];                  // bus of all PE "in_first" pins; O:N rows AND columns because each PE asserts "first" to the right and bottom adjacent PEs
assign in_first_bus[0][0] = inp_first;          // assigning the top-left PE (first to receive operands) to grid-level inp_first, which will then ripple through PEs, effectively "resetting" them for the next matmul

logic out_first_bus [0:ARRAY_DIM-1][0:ARRAY_DIM-1];             // bus of all PE "out_first" pins; each corresponds to a single PE; drives in_first_bus

logic [CNT_WIDTH-1:0] count;

genvar col, row;

generate
    for (row = 0; row < ARRAY_DIM; row++) begin : row_loop
        for (col = 0; col < ARRAY_DIM; col++) begin : col_loop
            pe //#(
                //.DATA_WIDTH(DATA_WIDTH),
                //.ACC_WIDTH(ACC_WIDTH)
            //) 
            pe (
                .clk(clk),
                .rst_n(rst_n),

                .in_a(a_bus[row][col]),                 // from left neighbor [row][col-1]
                .in_b(b_bus[row][col]),                 // from top neighbor [row-1][col]
                .out_a(a_bus[row][col+1]),              // giving A value to right neighbor
                .out_b(b_bus[row+1][col]),              // giving B value to bottom neighbor

                .in_a_valid(a_valid_bus[row][col]),
                .in_b_valid(b_valid_bus[row][col]),
                .out_a_valid(a_valid_bus[row][col+1]),
                .out_b_valid(b_valid_bus[row+1][col]),

                .in_first(in_first_bus[row][col]),
                .out_first(out_first_bus[row][col]),
                
                .acc(out[row][col]),

                .capture(1'b0),
                .shift_en(1'b0),
                .in_shadow('0),
                .out_shadow()
            );
        end : col_loop
    end : row_loop
endgenerate

// setting first row and column of the value buses to a_in and b_in
always_comb begin
    for (int i = 0; i < ARRAY_DIM; i++) begin
        a_bus[i][0] = a_in[i];
    end
    for (int j = 0; j < ARRAY_DIM; j++) begin
        b_bus[0][j] = b_in[j];
    end
end

// setting first row and column of valid buses to a_valid and b_valid
always_comb begin
    for (int k = 0; k < ARRAY_DIM; k++) begin
        a_valid_bus[k][0] = a_valid[k];
    end
    for (int l = 0; l < ARRAY_DIM; l++) begin
        b_valid_bus[0][l] = b_valid[l];
    end
end

// connecting the in_first_bus to the out_first_bus; this is to 
always_comb begin
    for (int i = 0; i < ARRAY_DIM; i++) begin
        for (int j = 0; j < ARRAY_DIM; j++) begin
            if (i == 0) in_first_bus[i][j+1] = out_first_bus[i][j];     // only top row "first" signas propagate left; the rest propagate downward
            in_first_bus[i+1][j] = out_first_bus[i][j];
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
