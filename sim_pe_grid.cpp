#include "Vpe_grid.h"           // the C++ class Verilator generated from Verilog
#include "verilated.h"          // core Verilator runtime functions
#include "verilated_vcd_c.h"    // VCD tracing support, for writing waveform files for GTKWave
#include <cstdio>               // standard C++ library for printf()
#include <array>                // including array for matrices

int main(int argc, char** argv) {
    // handing command-line args to Verilator so flags work
    Verilated::commandArgs(argc, argv);

    // creating new instance of design; pointer "dut"
    Vpe_grid* dut = new Vpe_grid;

    // ===== tracing setup for waveform file =====
    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("pe_grid.vcd");    // waveform output file

    // ===== loop variables =====
    vluint64_t sim_time = 0;    // verilator's name for a 64-bit unsigned integer; used as a simulation timestamp that ticks upward
    int case_count = 0;     // test case count
    int errors = 0;         // a counter for how many test cases fail
    
    std::array<std::array<double, 3>, 3> A = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    /* ===== input & output pins =====
    input logic clk,
    input logic rst_n,
    input logic inp_valid,                                  // are the current a_in & b_in real values or filler
    input logic signed [DATA_WIDTH-1:0] a_in [0:N-1],       // the current column of matrix A entering the grid; a_in[0] is the first row, a_in[1] is the second, etc.
    input logic signed [DATA_WIDTH-1:0] b_in [0:N-1],       // the current row of matrix B entering the grid; b_in[0] is the first column, b_in[1] is the second, etc.
    output logic signed [ACC_WIDTH-1:0] out [0:N-1][0:N-1]  // the output of the grid, the resulting matrix; ready when done = 1
    output logic done,                                      // when the matmul is done
    */

    // ===== loop functions =====
    auto tick = [&]() {    // tick one clock cycle, one clk cycle = 2 waveform time units
        dut->clk = 0;
        dut->eval();
        tfp->dump(sim_time++);  // reading current values & incrementing sim_time

        dut->clk = 1;
        dut->eval();
        tfp->dump(sim_time++);  // reading current vals & incrementing sim_time
    };

    auto test_grid = [&](int A, int B) {
        // set inputs (& OxFF because 8-bit inputs for now)
        int short_A = A & 0xFF;
        int short_B = B & 0xFF;
        dut->in_a = short_A;
        dut->in_b = short_B;

        // calculating reference
        acc_sum = acc_sum + (A*B);

        tick();
        
        // comparing & writing
        if (dut->out_a != short_A || dut->out_b != short_B || dut->acc != acc_sum) {
            printf("ERROR: in_a=%d in_b=%d; got out_a=%d out_b=%d acc=%d; expected out_a=%d out_b=%d acc=%d\n",
                    A, B, dut->out_a, dut->out_b, dut->acc, short_A, short_B, acc_sum);
            errors++;
        } else {
            printf("OK: in_a=%d in_b=%d; got out_a=%d out_b=%d acc=%d\n",
                    A, B, dut->out_a, dut->out_b, dut->acc);
        }
        case_count++;
    };

    // ===== test loop =====
    // resetting, active low
    dut->rst_n = 0;
    tick();
    dut->rst_n = 1;


    // ===== cleanup =====
    tfp->close();   // close the waveform file so it is written to disk
    delete dut;     // free the memory allocated with "new"

    // ===== final report =====
    if (errors == 0) printf("\nAll %d test cases passed.\n", case_count);
    else printf("\n%d test case(s) failed of %d total\n", errors, case_count);

    return errors ? 1 : 0; // return 0 if no errors, or 1 if there were.
}