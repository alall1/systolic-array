# cocotb Makefile for "skew_buffer" module
#
#   make
#   make WAVES=1         # run + dump waves for GTKWave
#   make clean
 
SIM ?= verilator
TOPLEVEL_LANG ?= verilog
 
RTL_DIR ?= ../../rtl
VERILOG_SOURCES += $(RTL_DIR)/pe_pkg.sv $(RTL_DIR)/skew_buffer.sv

TOPLEVEL = skew_buffer
COCOTB_TEST_MODULES = test_skew_buffer

# make the shared common/ importable
export PYTHONPATH := $(PWD)/../common:$(PYTHONPATH)

# parameters & passing them into Verilator
DATA_WIDTH ?= 8
ARRAY_DIM ?= 5

COMPILE_ARGS += -DARRAY_DIM=$(ARRAY_DIM) -DDATA_WIDTH=$(DATA_WIDTH)

# exporting parameters to testbench
export DATA_WIDTH
export ARRAY_DIM

ifeq ($(WAVES), 1)
	EXTRA_ARGS   += --trace --trace-structs
    export COCOTB_TRACE = 1
endif

include $(shell cocotb-config --makefiles)/Makefile.sim

clean::
	rm -rf sim_build __pycache__ results.xml dump.fst dump.vcd