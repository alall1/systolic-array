# cocotb Makefile for "pe" module
#
#   make                 # run with verilator (default here)
#   make WAVES=1         # run + dump waves for GTKWave
#   make clean
 
SIM ?= verilator
TOPLEVEL_LANG ?= verilog
 
RTL_DIR ?= ../../rtl
VERILOG_SOURCES += $(RTL_DIR)/pe.sv

TOPLEVEL = pe          		    # must match the module name in pe.sv
COCOTB_TEST_MODULES = test_pe	# python file (test_pe.py), no extension

COMPILE_ARGS += -GDATA_WIDTH=8 -GACC_WIDTH=32
EXTRA_ARGS   += --trace --trace-structs
 
ifeq ($(WAVES),1)
    export COCOTB_TRACE = 1
endif
 
include $(shell cocotb-config --makefiles)/Makefile.sim
 
clean::
	rm -rf sim_build __pycache__ results.xml dump.fst dump.vcd
