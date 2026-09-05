# cocotb Makefile for "pe_grid" module
#   make
#   make WAVES=1         # + waveform dump for GTKWave
#   make clean

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

RTL_DIR ?= ../../rtl
# pe_grid instantiates pe, so BOTH sources are needed.
VERILOG_SOURCES += $(RTL_DIR)/pe_pkg.sv $(RTL_DIR)/pe.sv $(RTL_DIR)/pe_grid.sv

TOPLEVEL = pe_grid
COCOTB_TEST_MODULES = test_pe_grid

# make the shared common/ importable
export PYTHONPATH := $(PWD)/../common:$(PYTHONPATH)

# parameters & passing them into Verilator
DATA_WIDTH	?= 8
ACC_WIDTH  	?= 32
ARRAY_DIM  	?= 5

COMPILE_ARGS += -DDATA_WIDTH=$(DATA_WIDTH) -DACC_WIDTH=$(ACC_WIDTH) -DARRAY_DIM=$(ARRAY_DIM)

# exporting parameters to testbench
export DATA_WIDTH
export ACC_WIDTH
export ARRAY_DIM

ifeq ($(WAVES), 1)
	EXTRA_ARGS   += --trace --trace-structs
    export COCOTB_TRACE = 1
endif

include $(shell cocotb-config --makefiles)/Makefile.sim

clean::
	rm -rf sim_build __pycache__ results.xml dump.fst dump.vcd