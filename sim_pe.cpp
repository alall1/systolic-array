#include "Vpe.h"                // the C++ class Verilator generated from Verilog
#include "verilated.h"          // core Verilator runtime functions
#include "verilated_vcd_c.h"    // VCD tracing support, for writing waveform files for GTKWave
#include <cstdio>               // standard C++ library for printf()

int main(int argc, char** argv) {
    // handing command-line args to Verilator so flags work
    Verilated::commandArgs(argc, argv);

    // creating new instance of design; pointer "dut"
    Vpe* dut = new Vpe;

    // ===== tracing setup for waveform file =====
    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    dut->trace(tfp, 99);
    tfp->open("pe.vcd");    // waveform output file

    // ===== loop variables =====
    vluint64_t sim_time = 0;    // verilator's name for a 64-bit unsigned integer; used as a simulation timestamp that ticks upward
    int case_count = 0;     // test case count
    int errors = 0;         // a counter for how many test cases fail
    
    int acc_sum = 0;        // accumulated sum of the 

    /* ===== input & output pins =====
    input clk,
    input rst_n,  // active low
    input [DATA_WIDTH-1:0] in_a,      // input from left neighbor
    input [DATA_WIDTH-1:0] in_b,      // input from top neighbor
    input in_valid,                   // high when inputs are real data (high if PE is being used this cycle)
    output [DATA_WIDTH-1:0] out_a,    // registered copy of left neighbor input to send to right neighbor (passing along operand)
    output [DATA_WIDTH-1:0] out_b,    // registered copy of top neighbor input to send to bottom neighbor
    output [ACC_WIDTH-1:0] acc 
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

    auto test_pe = [&](int A, int B) {
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

    // testing if in_valid works correctly
    dut->in_valid = 0;

    dut->in_a = 1;
    dut->in_b = 1;

    tick();

    if (dut->out_a != 0 || dut->out_b != 0 || dut->acc != 0) {
        printf("ERROR: in_a=%d in_b=%d; got out_a=%d out_b=%d acc=%d; expected out_a=%d out_b=%d acc=%d\n",
                0, 0, dut->out_a, dut->out_b, dut->acc, 0, 0, 0);
        errors++;
    } else {
        printf("OK: in_a=%d in_b=%d; got out_a=%d out_b=%d acc=%d\n",
                0, 0, dut->out_a, dut->out_b, dut->acc);
    }
    case_count++;

    dut->in_valid = 1;

    /*

    test_pe(5, 8);
    test_pe(-4, -24);
    test_pe(127, -128);

    */

    for (int i = -128; i < 128; i++) {
        for (int j = -128; j < 128; j++) {
            test_pe(i, j);
        }
    }

    // ===== cleanup =====
    tfp->close();   // close the waveform file so it is written to disk
    delete dut;     // free the memory allocated with "new"

    // ===== final report =====
    if (errors == 0) printf("\nAll %d test cases passed.\n", case_count);
    else printf("\n%d test case(s) failed of %d total\n", errors, case_count);

    return errors ? 1 : 0; // return 0 if no errors, or 1 if there were.
}