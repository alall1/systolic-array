# cocotb Makefile for "host_if" module (bus decode, registers, status, start, C read mux)
#
#   make                 # cocotb lockstep + monitors + coverage
#   make WAVES=1         # run + dump waves for GTKWave
#   make SEED=<n>        # reproduce / explore a random run
#   make SVA=0           # build without the RTL assertions (on by default)
#   make LOCKSTEP=0      # no DUT-vs-model compare: monitors (+ SVA) judge alone
#   make MONITORS=0      # no concurrent monitors (mutation attribution only)
#   make model           # standalone pytest of the golden model + closed loop (no RTL)
#   make clean

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

RTL_DIR ?= ../../rtl
VERILOG_SOURCES += $(RTL_DIR)/pe_pkg.sv $(RTL_DIR)/host_if.sv

TOPLEVEL = host_if
COCOTB_TEST_MODULES = test_host_if

# shared common/ helpers, plus the tile_ctrl + feeder models (busy/done/error from the real schedule)
export PYTHONPATH := $(PWD)/../common:$(PWD)/../feeder:$(PWD)/../tile_ctrl:$(PYTHONPATH)

# parameters & passing them into Verilator
ARRAY_DIM  ?= 5
DATA_WIDTH ?= 8
ACC_WIDTH  ?= 32
MAX_K      ?= 1024
MAX_M      ?= 64
MAX_N      ?= 64
SEED       ?= 12345
SVA        ?= 1

COMPILE_ARGS += -DARRAY_DIM=$(ARRAY_DIM) -DDATA_WIDTH=$(DATA_WIDTH) -DACC_WIDTH=$(ACC_WIDTH) \
                -DMAX_K=$(MAX_K) -GMAX_M=$(MAX_M) -GMAX_N=$(MAX_N)
# Verilator >= 5.0xx enables assertions by default; say it explicitly either way
ifeq ($(SVA), 1)
	COMPILE_ARGS += --assert
else
	COMPILE_ARGS += --no-assert
endif

# exporting parameters to testbench
export ARRAY_DIM
export DATA_WIDTH
export ACC_WIDTH
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
	python -m pytest test_host_if_model.py -q -p no:cacheprovider

clean::
	rm -rf sim_build __pycache__ .pytest_cache results.xml dump.fst dump.vcd
