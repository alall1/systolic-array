module pe_grid 
import pe_pkg::*;
(
    input logic clk,
    input logic rst_n,

    input a_payload_t a_in [0:ARRAY_DIM-1],     // input into the left side of the array; note that the "first" signal propagates through the a_payload_t types
    input b_payload_t b_in [0:ARRAY_DIM-1],     // input into the top side of the array

    input logic capture,
    input logic shift_en,

    output logic signed [ACC_WIDTH-1:0] out [0:ARRAY_DIM-1][0:ARRAY_DIM-1], // the output of the grid, the resulting matrix; ready when out_ready = 1
    output logic out_ready,                                                 // when outputs are ready to read; for now, after 3(ARRAY_DIM) - 2 cycles, when all PEs are done, but for future, after 2(ARRAY_DIM-1) cycles first PE output finishes and can be read in a ripple fashion

    output logic [ACC_WIDTH-1:0] drain_out [0:ARRAY_DIM-1]
);

localparam int FINISH = 3*ARRAY_DIM - 2;                // baked-in number of clock cycles until the NxN matmul is finished. 3(ARRAY_DIM) - 2
localparam int CNT_WIDTH = $clog2(FINISH + 1);          // calculating the number of bits needed to count up to FINISH; indexed at 0 so counter = FINISH - 1 when complete
logic [CNT_WIDTH-1:0] count;

a_payload_t a_bus [0:ARRAY_DIM-1][0:ARRAY_DIM];      // bus of all horizontal connections between PEs, 0:N columns because of rightmost PEs' out_a / out_b signals (unused)
b_payload_t b_bus [0:ARRAY_DIM][0:ARRAY_DIM-1];      // bus of all vertical connections between PEs, O:N rows because of bottom PEs' out_a / out_b signals

genvar col, row;

generate
    for (row = 0; row < ARRAY_DIM; row++) begin : row_loop
        for (col = 0; col < ARRAY_DIM; col++) begin : col_loop
            pe pe (
                .clk(clk),
                .rst_n(rst_n),

                .in_a(a_bus[row][col]),
                .out_a(a_bus[row][col+1]),  // out_a feeds right neighbor

                .in_b(b_bus[row][col]),
                .out_b(b_bus[row+1][col]),  // out_b feeds bottom neighbor

                .acc(out[row][col]),

                .capture(1'b0),
                .shift_en(1'b0),
                .in_shadow('0),
                .out_shadow()
            );
        end : col_loop
    end : row_loop
endgenerate

// setting first row and column of the payload buses to a_in and b_in
always_comb begin
    for (int i = 0; i < ARRAY_DIM; i++) begin
        a_bus[i][0] = a_in[i];
    end
    for (int j = 0; j < ARRAY_DIM; j++) begin
        b_bus[0][j] = b_in[j];
    end
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        count <= '0;
    end else if (a_bus[0][0].first) begin
        count <= '0;
    end else begin
        count <= count + 1;
    end
end

assign out_ready = (count == CNT_WIDTH'(FINISH - 1));

endmodule
