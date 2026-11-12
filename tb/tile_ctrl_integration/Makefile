# cocotb Makefile for the system smoke test: tile_ctrl + feeder + pe_grid + collector + lane_ram (A, B, C)
#
#   make
#   make WAVES=1         # run + dump waves for GTKWave
#   make SEED=<n>
#   make SVA=0           # build without the RTL assertions (on by default)
#   make clean

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

RTL_DIR ?= ../../rtl
VERILOG_SOURCES += $(RTL_DIR)/pe_pkg.sv $(RTL_DIR)/pe.sv $(RTL_DIR)/pe_grid.sv \
                   $(RTL_DIR)/skew_buffer.sv $(RTL_DIR)/read_sequencer.sv $(RTL_DIR)/feeder.sv \
                   $(RTL_DIR)/tile_ctrl.sv $(RTL_DIR)/collector.sv $(RTL_DIR)/lane_ram.sv \
                   $(PWD)/tile_ctrl_integration_top.sv

TOPLEVEL = tile_ctrl_integration_top
COCOTB_TEST_MODULES = test_tile_ctrl_integration

# make the shared common/ importable
export PYTHONPATH := $(PWD)/../common:$(PYTHONPATH)

# parameters & passing them into Verilator
DATA_WIDTH ?= 8
ACC_WIDTH  ?= 32
ARRAY_DIM  ?= 4
MAX_K      ?= 1024
MAX_M      ?= 64
MAX_N      ?= 64
SEED       ?= 12345
SVA        ?= 1

COMPILE_ARGS += -DDATA_WIDTH=$(DATA_WIDTH) -DACC_WIDTH=$(ACC_WIDTH) -DARRAY_DIM=$(ARRAY_DIM) \
                -DMAX_K=$(MAX_K) -GMAX_M=$(MAX_M) -GMAX_N=$(MAX_N)
# Verilator >= 5.0xx enables assertions by default; say it explicitly either way
ifeq ($(SVA), 1)
	COMPILE_ARGS += --assert
else
	COMPILE_ARGS += --no-assert
endif

# exporting parameters to testbench
export DATA_WIDTH
export ACC_WIDTH
export ARRAY_DIM
export MAX_K
export MAX_M
export MAX_N
export SEED

ifeq ($(WAVES), 1)
	EXTRA_ARGS   += --trace --trace-structs
    export COCOTB_TRACE = 1
endif

include $(shell cocotb-config --makefiles)/Makefile.sim

clean::
	rm -rf sim_build __pycache__ results.xml dump.fst dump.vcd
