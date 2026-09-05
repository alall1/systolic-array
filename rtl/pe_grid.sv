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
    output logic [ACC_WIDTH-1:0] drain_out [0:ARRAY_DIM-1]
);

genvar col, row;

localparam int FINISH = 3*ARRAY_DIM - 2;                // baked-in number of clock cycles until the NxN matmul is finished. 3(ARRAY_DIM) - 2
localparam int CNT_WIDTH = $clog2(FINISH + 1);          // calculating the number of bits needed to count up to FINISH; indexed at 0 so counter = FINISH - 1 when complete
logic [CNT_WIDTH-1:0] count;

a_payload_t a_bus [0:ARRAY_DIM-1][0:ARRAY_DIM];      // bus of all horizontal connections between PEs, 0:ARRAY_DIM columns because of rightmost PEs' out_a / out_b signals (unused)
b_payload_t b_bus [0:ARRAY_DIM][0:ARRAY_DIM-1];      // bus of all vertical connections between PEs, 0:ARRAY_DIM rows because of bottom PEs' out_a / out_b signals

logic [ACC_WIDTH-1:0] shadow_bus [0:ARRAY_DIM][0:ARRAY_DIM-1];    // array corresponding to each shadow register; first row shadow_reg[0][x] is 

generate
    for (row = 0; row < ARRAY_DIM; row++) begin : row_loop
        for (col = 0; col < ARRAY_DIM; col++) begin : col_loop
            pe pe (
                .clk(clk),
                .rst_n(rst_n),

                .in_a(a_bus[row][col]),
                .out_a(a_bus[row][col+1]),          // out_a feeds right neighbor

                .in_b(b_bus[row][col]),
                .out_b(b_bus[row+1][col]),          // out_b feeds bottom neighbor

                .acc(out[row][col]),

                .capture(capture),
                .shift_en(shift_en),
                .in_shadow(shadow_bus[row][col]),
                .out_shadow(shadow_bus[row+1][col]) // actual shadow register of each PE is shadow_bus[row+1]
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

// setting in_shadow for top row all zero
always_comb begin
    for (int i = 0; i < ARRAY_DIM; i++) begin
        shadow_bus[0][i] = '0;
    end
end

// drain_out is a combinational output of the bottom row's shadow registers
always_comb begin
    for (int i = 0; i < ARRAY_DIM; i++) begin
        drain_out[i] = shadow_bus[ARRAY_DIM][i];
    end
end

endmodule
