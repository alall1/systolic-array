# cocotb Makefile for "pe_grid" module
#   make
#   make WAVES=1         # + waveform dump for GTKWave
#   make clean

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

RTL_DIR ?= ../../rtl
# pe_grid instantiates pe, so BOTH sources are needed.
VERILOG_SOURCES += $(RTL_DIR)/pe.sv
VERILOG_SOURCES += $(RTL_DIR)/pe_grid.sv

TOPLEVEL = pe_grid
COCOTB_TEST_MODULES = test_pe_grid

# make the shared common/ importable
export PYTHONPATH := $(PWD)/../common:$(PYTHONPATH)

# grid size + widths
GRID_N ?= 5
COMPILE_ARGS += -GDATA_WIDTH=8 -GACC_WIDTH=32 -GN=$(GRID_N)
EXTRA_ARGS   += --trace --trace-structs

ifeq ($(WAVES),1)
    export COCOTB_TRACE = 1
endif

include $(shell cocotb-config --makefiles)/Makefile.sim

clean::
	rm -rf sim_build __pycache__ results.xml dump.fst dump.vcd