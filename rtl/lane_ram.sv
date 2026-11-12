// Tile-word RAM: one word = LANES lanes that move together in one cycle.
// Used three times (A, B, C memories); the tile-word layout lives entirely in
// the address wiring outside this module, so it does no arithmetic of its own.
// Write port with per-lane enables; registered read port, read-first, held
// while re is low; no reset (block-RAM style). Contract: docs/memory_contract.md §4
module lane_ram #(
    parameter int LANES  = 4,                       // words are LANES lanes wide
    parameter int WIDTH  = 8,                       // bits per lane
    parameter int ADDR_W = 6                        // 2**ADDR_W words
)
(
    input  logic clk,                               // no reset: block-RAM style (D6)

    // write port
    input  logic we,
    input  logic [ADDR_W-1:0] waddr,
    input  logic [LANES-1:0] wlane_en,              // lane l written iff set
    input  logic [WIDTH-1:0] wdata [0:LANES-1],

    // read port: registered, 1-cycle latency, read-first (D5), held while re is low
    input  logic re,
    input  logic [ADDR_W-1:0] raddr,
    output logic [WIDTH-1:0] rdata [0:LANES-1]
);

generate
    if (LANES < 1 || WIDTH < 1 || ADDR_W < 1) begin : g_param_check
        $error("lane_ram requires LANES, WIDTH and ADDR_W >= 1");
    end
endgenerate

localparam int DEPTH = 1 << ADDR_W;

logic [WIDTH-1:0] mem [0:DEPTH-1][0:LANES-1];

// write: only enabled lanes change
always_ff @(posedge clk) begin
    if (we) begin
        for (int l = 0; l < LANES; l++) begin
            if (wlane_en[l]) mem[waddr][l] <= wdata[l];
        end
    end
end

// read: samples mem before this edge's write lands (nonblocking), so a same-address
// read and write on one edge return the old word
always_ff @(posedge clk) begin
    if (re) begin
        for (int l = 0; l < LANES; l++) rdata[l] <= mem[raddr][l];
    end
end

`ifndef SYNTHESIS
// ---------------------------------------------------------------------------
// Assertions: contract §7.1-4 (Verilator --assert). No reset, so a checker-only
// flag arms them after the first edge. Data values against the written history
// are checked end to end by the cocotb ReadScoreboard.
// ---------------------------------------------------------------------------
logic sva_armed;                                    // simulation-only: there is no reset to arm from,
initial sva_armed = 1'b0;                           // so plain initial + always (always_ff forbids a
always @(posedge clk) sva_armed <= 1'b1;            // second driver, and this flop never reaches synthesis)

// checker-only record of what the last edge was asked to do, and the words as
// they were BEFORE that edge (nonblocking reads)
logic             sva_re_q, sva_we_q;
logic [ADDR_W-1:0] sva_waddr_q;
logic [LANES-1:0] sva_wlane_q;
logic [WIDTH-1:0] sva_rword_q [0:LANES-1];          // word at raddr before the edge
logic [WIDTH-1:0] sva_wold_q  [0:LANES-1];          // word at waddr before the edge
logic [WIDTH-1:0] sva_wdata_q [0:LANES-1];
always_ff @(posedge clk) begin
    sva_re_q    <= re;
    sva_we_q    <= we;
    sva_waddr_q <= waddr;
    sva_wlane_q <= wlane_en;
    for (int l = 0; l < LANES; l++) begin
        sva_rword_q[l] <= mem[raddr][l];
        sva_wold_q[l]  <= mem[waddr][l];
        sva_wdata_q[l] <= wdata[l];
    end
end

logic sva_read_ok, sva_write_ok;
logic [LANES*WIDTH-1:0] sva_rdata_flat;                 // packed copy: $stable needs a packed operand
always_comb begin
    sva_read_ok  = 1'b1;
    sva_write_ok = 1'b1;
    for (int l = 0; l < LANES; l++) begin
        sva_rdata_flat[l*WIDTH +: WIDTH] = rdata[l];
        sva_read_ok  = sva_read_ok && (rdata[l] == sva_rword_q[l]);
        sva_write_ok = sva_write_ok
                       && (mem[sva_waddr_q][l] == (sva_wlane_q[l] ? sva_wdata_q[l] : sva_wold_q[l]));
    end
end

// §7.4: rdata changes only on an edge with re high
a_rdata_held_without_read: assert property (@(posedge clk) disable iff (!sva_armed)
    !re |=> $stable(sva_rdata_flat));
// §7.1, 7.3: a read returns the word as it was before its edge (read-first)
a_read_returns_old_word: assert property (@(posedge clk) disable iff (!sva_armed)
    sva_re_q |-> sva_read_ok);
// §7.2: a write changes exactly the enabled lanes of its word
a_write_changes_enabled_lanes: assert property (@(posedge clk) disable iff (!sva_armed)
    sva_we_q |-> sva_write_ok);
`endif

endmodule
