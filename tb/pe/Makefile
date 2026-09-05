# cocotb Makefile for "pe" module
#
#   make                 # run with verilator (default here)
#   make WAVES=1         # run + dump waves for GTKWave
#   make clean
 
SIM ?= verilator
TOPLEVEL_LANG ?= verilog
 
RTL_DIR ?= ../../rtl
VERILOG_SOURCES += $(RTL_DIR)/pe_pkg.sv $(RTL_DIR)/pe.sv

TOPLEVEL = pe
COCOTB_TEST_MODULES = test_pe

# make the shared common/ importable
export PYTHONPATH := $(PWD)/../common:$(PYTHONPATH)

# parameters & passing them into Verilator
DATA_WIDTH ?= 8
ACC_WIDTH  ?= 32

COMPILE_ARGS += -DDATA_WIDTH=$(DATA_WIDTH) -DACC_WIDTH=$(ACC_WIDTH)

# exporting parameters to testbench
export DATA_WIDTH
export ACC_WIDTH

ifeq ($(WAVES), 1)
	EXTRA_ARGS   += --trace --trace-structs
    export COCOTB_TRACE = 1
endif
 
include $(shell cocotb-config --makefiles)/Makefile.sim
 
clean::
	rm -rf sim_build __pycache__ results.xml dump.fst dump.vcd
