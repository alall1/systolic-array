// Host interface: SRAM-style bus (always ready, read data the cycle after the
// read edge) in front of the matmul system. Address = {region, word, lane}:
// region 0 registers, 1 A, 2 B, 3 C; word/lane follow the tile-word layout, so
// all decode is field compares and concatenation, no arithmetic.
// Registers: CTRL (START), STATUS (BUSY, DONE, CMD_ERROR, HOST_ERROR), DIM_M/N/K.
// Contract: docs/top_contract.md §3-8
module host_if
import pe_pkg::*;
#(
    parameter  int MAX_M   = 64,                                    // must equal tile_ctrl's
    parameter  int MAX_N   = 64,                                    // must equal tile_ctrl's
    localparam int M_W     = $clog2(MAX_M + 1),
    localparam int N_W     = $clog2(MAX_N + 1),
    localparam int TILES_M = (MAX_M + ARRAY_DIM - 1) / ARRAY_DIM,
    localparam int TILES_N = (MAX_N + ARRAY_DIM - 1) / ARRAY_DIM,
    localparam int TM_W    = (TILES_M > 1) ? $clog2(TILES_M) : 1,
    localparam int TN_W    = (TILES_N > 1) ? $clog2(TILES_N) : 1,
    localparam int ROW_W   = (MAX_M > 1) ? $clog2(MAX_M) : 1,
    localparam int KI_W    = (MAX_K > 1) ? $clog2(MAX_K) : 1,
    localparam int A_AW    = TM_W + KI_W,                           // A word {tile_m, k}
    localparam int B_AW    = TN_W + KI_W,                           // B word {tile_n, k}
    localparam int C_AW    = ROW_W + TN_W,                          // C word {row, tile_n}
    localparam int LANE_W  = (ARRAY_DIM > 1) ? $clog2(ARRAY_DIM) : 1,
    localparam int WORD_W  = (A_AW > B_AW) ? ((A_AW > C_AW) ? A_AW : C_AW) : ((B_AW > C_AW) ? B_AW : C_AW),
    localparam int HOST_AW = 2 + WORD_W + LANE_W,                   // {region, word, lane}
    localparam int HOST_W  = ACC_WIDTH
)
(
    input  logic clk,
    input  logic rst_n,                                     // async, active low

    // host bus: SRAM-style, always ready, read data the cycle after the read edge
    input  logic host_we,
    input  logic host_re,
    input  logic [HOST_AW-1:0] host_addr,
    // one HOST_W bus word per transfer; registers and A/B elements use only its low bits
    /* verilator lint_off UNUSEDSIGNAL */
    input  logic [HOST_W-1:0] host_wdata,
    /* verilator lint_on UNUSEDSIGNAL */
    output logic [HOST_W-1:0] host_rdata,

    // tile_ctrl command interface
    output logic start,
    output logic [M_W-1:0] M,
    output logic [N_W-1:0] N,
    output logic [K_W-1:0] K,
    input  logic busy,
    input  logic done,
    input  logic error,

    // A / B memory write ports (lane_ram)
    output logic a_we,
    output logic [A_AW-1:0] a_waddr,
    output logic [ARRAY_DIM-1:0] a_wlane_en,
    output logic [DATA_WIDTH-1:0] a_wdata [0:ARRAY_DIM-1],
    output logic b_we,
    output logic [B_AW-1:0] b_waddr,
    output logic [ARRAY_DIM-1:0] b_wlane_en,
    output logic [DATA_WIDTH-1:0] b_wdata [0:ARRAY_DIM-1],

    // C memory read port (lane_ram)
    output logic c_re,
    output logic [C_AW-1:0] c_raddr,
    input  logic [ACC_WIDTH-1:0] c_rdata [0:ARRAY_DIM-1]
);

// same parameter rules as tile_ctrl / collector, which share MAX_M / MAX_N
generate
    if (ARRAY_DIM < 2) begin : g_dim_check
        $error("host_if requires ARRAY_DIM >= 2");
    end
endgenerate

localparam logic [1:0] REGION_REGS = 2'd0, REGION_A = 2'd1, REGION_B = 2'd2, REGION_C = 2'd3;
localparam int REG_CTRL = 0, REG_STATUS = 1, REG_DIM_M = 2, REG_DIM_N = 3, REG_DIM_K = 4, NUM_REGS = 5;

// ---------------------------------------------------------------------------
// address fields and decode (same cycle)
// ---------------------------------------------------------------------------
logic [1:0]        region;
logic [WORD_W-1:0] word;
logic [LANE_W-1:0] lane;
assign region = host_addr[HOST_AW-1 -: 2];
assign word   = host_addr[LANE_W +: WORD_W];
assign lane   = host_addr[LANE_W-1:0];

// a lane code >= ARRAY_DIM, word bits above the region's width, a nonzero lane in
// the register region, or an unused register index is unmapped: ignored / reads 0.
// Some compares are constant for some parameters (e.g. ARRAY_DIM a power of two,
// or a region as wide as WORD_W); they stay so every parameterization is checked.
logic lane_ok, mapped;
/* verilator lint_off CMPCONST */
assign lane_ok = (int'(lane) < ARRAY_DIM);
always_comb begin
    case (region)
        REGION_REGS: mapped = (lane == '0) && (int'(word) < NUM_REGS);
        REGION_A:    mapped = lane_ok && ((word >> A_AW) == '0);
        REGION_B:    mapped = lane_ok && ((word >> B_AW) == '0);
        default:     mapped = lane_ok && ((word >> C_AW) == '0);
    endcase
end
/* verilator lint_on CMPCONST */

logic start_q;                                  // a taken START, pulsed to tile_ctrl this cycle
logic busy_now;                                 // STATUS.BUSY: job running, or a start pending
assign busy_now = busy || start_q;

logic reg_we, start_taken, ab_write, ab_write_busy;
assign reg_we        = host_we && mapped && (region == REGION_REGS);
assign start_taken   = reg_we && (int'(word) == REG_CTRL) && host_wdata[0] && !busy_now;
assign ab_write      = host_we && mapped && ((region == REGION_A) || (region == REGION_B));
assign ab_write_busy = ab_write && busy_now;

// A / B write ports: dropped while BUSY (T6); one-hot lane enable, no read-modify-write
always_comb begin
    a_we       = host_we && mapped && (region == REGION_A) && !busy_now;
    b_we       = host_we && mapped && (region == REGION_B) && !busy_now;
    a_waddr    = a_we ? word[A_AW-1:0] : '0;
    b_waddr    = b_we ? word[B_AW-1:0] : '0;
    a_wlane_en = '0;
    b_wlane_en = '0;
    for (int l = 0; l < ARRAY_DIM; l++) begin
        a_wlane_en[l] = a_we && (int'(lane) == l);
        b_wlane_en[l] = b_we && (int'(lane) == l);
        a_wdata[l]    = a_we ? host_wdata[DATA_WIDTH-1:0] : '0;
        b_wdata[l]    = b_we ? host_wdata[DATA_WIDTH-1:0] : '0;
    end
end

// C read port: sampled by lane_ram on this edge, data back after it
assign c_re    = host_re && mapped && (region == REGION_C);
assign c_raddr = c_re ? word[C_AW-1:0] : '0;

// ---------------------------------------------------------------------------
// registers, sticky status, start pulse, read data
// ---------------------------------------------------------------------------
logic [M_W-1:0] dim_m;
logic [N_W-1:0] dim_n;
logic [K_W-1:0] dim_k;
logic done_s, cmd_error_s, host_error_s;

// STATUS as read now: BUSY live (incl. pending start); DONE / CMD_ERROR sticky OR live
logic [3:0] status;
assign status = {host_error_s, cmd_error_s || (done && error), done_s || done, busy_now};

logic [HOST_W-1:0] reg_rdata;                   // value of the addressed register before this edge
always_comb begin
    reg_rdata = '0;
    if (mapped && region == REGION_REGS) begin
        case (int'(word))
            REG_STATUS: reg_rdata = HOST_W'(status);
            REG_DIM_M:  reg_rdata = HOST_W'(dim_m);
            REG_DIM_N:  reg_rdata = HOST_W'(dim_n);
            REG_DIM_K:  reg_rdata = HOST_W'(dim_k);
            default:    reg_rdata = '0;             // CTRL reads 0
        endcase
    end
end

logic              rd_c_q;                      // last read edge was a mapped C read
logic [LANE_W-1:0] rd_lane_q;
logic [HOST_W-1:0] rd_val_q;                    // last read's value when not from C

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dim_m        <= '0;
        dim_n        <= '0;
        dim_k        <= '0;
        start_q      <= 1'b0;
        done_s       <= 1'b0;
        cmd_error_s  <= 1'b0;
        host_error_s <= 1'b0;
        rd_c_q       <= 1'b0;
        rd_lane_q    <= '0;
        rd_val_q     <= '0;
    end else begin
        start_q <= start_taken;

        // a taken START clears the sticky bits and wins over a same-edge set
        // (that done belongs to the previous job)
        if (start_taken) begin
            done_s       <= 1'b0;
            cmd_error_s  <= 1'b0;
            host_error_s <= 1'b0;
        end else begin
            if (done)          done_s       <= 1'b1;
            if (done && error) cmd_error_s  <= 1'b1;
            if (ab_write_busy) host_error_s <= 1'b1;
        end

        if (reg_we && int'(word) == REG_DIM_M) dim_m <= host_wdata[M_W-1:0];
        if (reg_we && int'(word) == REG_DIM_N) dim_n <= host_wdata[N_W-1:0];
        if (reg_we && int'(word) == REG_DIM_K) dim_k <= host_wdata[K_W-1:0];

        // read-first: registers are sampled before this edge's write
        if (host_re) begin
            rd_c_q    <= c_re;
            rd_lane_q <= lane;
            rd_val_q  <= reg_rdata;             // 0 for A/B, C-region writes and unmapped
        end
    end
end

assign start = start_q;
assign M     = dim_m;
assign N     = dim_n;
assign K     = dim_k;

// C data comes from lane_ram's registered read port; pick the lane read
always_comb begin
    host_rdata = rd_val_q;
    if (rd_c_q) begin
        host_rdata = '0;
        for (int l = 0; l < ARRAY_DIM; l++) begin
            if (int'(rd_lane_q) == l) host_rdata = HOST_W'(c_rdata[l]);
        end
    end
end

`ifndef SYNTHESIS
// ---------------------------------------------------------------------------
// Assertions: contract §8.1-6 (Verilator --assert). Disabled in reset, so the
// async-reset behaviour is checked by the cocotb monitors and Env.reset.
// ---------------------------------------------------------------------------

// armed one edge after reset release (rst_n stays purely asynchronous)
logic sva_armed;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) sva_armed <= 1'b0;
    else        sva_armed <= 1'b1;
end

// a START write as the host issued it, before the BUSY rule decides whether it is taken
logic sva_start_write;
assign sva_start_write = reg_we && (int'(word) == REG_CTRL) && host_wdata[0];

// the one-hot lane enable the address asks for, computed independently of the decode
logic [ARRAY_DIM-1:0] sva_lane_onehot;
always_comb begin
    sva_lane_onehot = '0;
    for (int l = 0; l < ARRAY_DIM; l++) if (lane == LANE_W'(l)) sva_lane_onehot[l] = 1'b1;
end

// the C lane the last read asked for, selected independently of the RTL mux
logic [ACC_WIDTH-1:0] sva_c_lane;
always_comb begin
    sva_c_lane = '0;
    for (int l = 0; l < ARRAY_DIM; l++) if (rd_lane_q == LANE_W'(l)) sva_c_lane = c_rdata[l];
end

// §8.1: start is a one-cycle pulse, only after a START write taken while not BUSY
a_start_after_taken_write: assert property (@(posedge clk) disable iff (!sva_armed)
    start |-> $past(sva_start_write && !(busy || start)));
a_start_one_cycle: assert property (@(posedge clk) disable iff (!sva_armed)
    start |=> !start);
a_start_ignored_while_busy: assert property (@(posedge clk) disable iff (!sva_armed)
    (sva_start_write && (busy || start)) |=> !start);

// §8.3: each sticky bit rises only from its cause and falls only on a taken START
a_done_rises_from_done: assert property (@(posedge clk) disable iff (!sva_armed)
    $rose(done_s) |-> $past(done));
a_cmd_error_rises_from_reject: assert property (@(posedge clk) disable iff (!sva_armed)
    $rose(cmd_error_s) |-> $past(done && error));
a_host_error_rises_from_busy_write: assert property (@(posedge clk) disable iff (!sva_armed)
    $rose(host_error_s) |-> $past(ab_write && (busy || start)));
a_sticky_falls_only_on_start: assert property (@(posedge clk) disable iff (!sva_armed)
    ($fell(done_s) || $fell(cmd_error_s) || $fell(host_error_s)) |-> start);
// a taken START leaves every sticky bit clear, even with a same-edge done (that done is stale)
a_start_clears_sticky: assert property (@(posedge clk) disable iff (!sva_armed)
    start_taken |=> !(done_s || cmd_error_s || host_error_s));

// §8.4, 6: memory strobes: A/B writes only while not BUSY, one lane; C reads only for host reads of C
a_ab_write_only_when_idle: assert property (@(posedge clk) disable iff (!sva_armed)
    (a_we || b_we) |-> !(busy || start));
a_ab_lane_one_hot: assert property (@(posedge clk) disable iff (!sva_armed)
    (a_we ? $onehot(a_wlane_en) : (a_wlane_en == '0)) && (b_we ? $onehot(b_wlane_en) : (b_wlane_en == '0)));
a_ab_lane_matches_address: assert property (@(posedge clk) disable iff (!sva_armed)
    (a_we |-> a_wlane_en == sva_lane_onehot) and (b_we |-> b_wlane_en == sva_lane_onehot));
a_c_read_from_host_read: assert property (@(posedge clk) disable iff (!sva_armed)
    c_re |-> host_re && (region == REGION_C));

// §8.5: a read returns the addressed value the cycle after its edge
a_reg_read_returns_value: assert property (@(posedge clk) disable iff (!sva_armed)
    (host_re && !c_re) |=> host_rdata == $past(reg_rdata));
a_c_read_returns_lane: assert property (@(posedge clk) disable iff (!sva_armed)
    c_re |=> host_rdata == HOST_W'(sva_c_lane));
`endif

endmodule
