// DATA_WIDTH : operand width
`ifndef DATA_WIDTH
  `define DATA_WIDTH 8
`endif

// ACC_WIDTH : accumulator width
`ifndef ACC_WIDTH
  `define ACC_WIDTH 32
`endif

// ARRAY_DIM : array dimensions, square ARRAY_DIM x ARRAY_DIM grid
`ifndef ARRAY_DIM
  `define ARRAY_DIM 4
`endif

package pe_pkg;
    localparam int DATA_WIDTH = `DATA_WIDTH;
    localparam int ACC_WIDTH  = `ACC_WIDTH;
    localparam int ARRAY_DIM = `ARRAY_DIM;

    // A operand (west->east) carries the shared-K "first" bit; B (north->south) does not; width asymmetry is intentional
    // data [MSB] -> first [LSB]
    typedef struct packed {
        logic signed [DATA_WIDTH-1:0] data;
        logic valid;
        logic first;
    } a_payload_t;

    typedef struct packed {
        logic signed [DATA_WIDTH-1:0] data;
        logic valid;
    } b_payload_t;

endpackage : pe_pkg
