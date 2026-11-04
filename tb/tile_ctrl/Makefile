# cocotb Makefile for "tile_ctrl" module (control FSM)
#
#   make                 # cocotb lockstep + monitors + coverage
#   make WAVES=1         # run + dump waves for GTKWave
#   make SEED=<n>        # reproduce / explore a random run
#   make SVA=0           # build without the RTL assertions (on by default)
#   make LOCKSTEP=0      # no DUT-vs-model compare: monitors (+ SVA) judge alone
#   make MONITORS=0      # no concurrent monitors (mutation attribution only)
#   make model           # standalone pytest of the golden model (no RTL)
#   make clean

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

RTL_DIR ?= ../../rtl
VERILOG_SOURCES += $(RTL_DIR)/pe_pkg.sv $(RTL_DIR)/tile_ctrl.sv

TOPLEVEL = tile_ctrl
COCOTB_TEST_MODULES = test_tile_ctrl

# make the shared common/ importable, plus FeederModel (closed-loop feed_done)
export PYTHONPATH := $(PWD)/../common:$(PWD)/../feeder:$(PYTHONPATH)

# parameters & passing them into Verilator
ARRAY_DIM ?= 5
MAX_K     ?= 1024
MAX_M     ?= 64
MAX_N     ?= 64
SEED      ?= 12345
SVA       ?= 1

COMPILE_ARGS += -DARRAY_DIM=$(ARRAY_DIM) -DMAX_K=$(MAX_K) -GMAX_M=$(MAX_M) -GMAX_N=$(MAX_N)
# Verilator >= 5.0xx enables assertions by default; say it explicitly either way
ifeq ($(SVA), 1)
	COMPILE_ARGS += --assert
else
	COMPILE_ARGS += --no-assert
endif

# exporting parameters to testbench
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

model:
	python -m pytest test_tile_ctrl_model.py -q

clean::
	rm -rf sim_build __pycache__ .pytest_cache results.xml dump.fst dump.vcd
