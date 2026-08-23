#include "Vpe.h"                // the C++ class Verilator generated from Verilog
#include "verilated.h"          // core Verilator runtime functions
#include "verilated_vcd_c.h"    // VCD tracing support, for writing waveform files for GTKWave
#include <cstdio>               // standard C++ library for printf()

int main(int argc, char** argv) {
    // handing command-line args to Verilator so flags work
    Verilated::commandArgs(argc, argv);

    // creating new instance of design; pointer "top"
    Vpe* top = new Vpe;

    // ===== tracing setup for waveform file =====
    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("pe.vcd");    // waveform output file

    // ===== loop variables =====
    vluint64_t time = 0;    // verilator's name for a 64-bit unsigned integer; used as a simulation timestamp that ticks upward
    int errors = 0;         // a counter for how many test cases fail

    // ===== loop functions =====

    // ===== test loop =====

    // ===== cleanup =====
    tfp->close();   // close the waveform file so it is written to disk
    delete top;     // free the memory allocated with "new"

    // ===== final report =====
    if (errors == 0) printf("\nAll test cases passed.\n");
    else printf("\n%d test case(s) failed.\n", errors);

    return errors ? 1 : 0; // return 0 if no errors, or 1 if there were.
}