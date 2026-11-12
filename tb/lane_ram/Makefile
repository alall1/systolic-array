# cocotb Makefile for "lane_ram" module (tile-word RAM: A, B and C memories)
#
#   make                 # cocotb lockstep + monitors + coverage
#   make WAVES=1         # run + dump waves for GTKWave
#   make SEED=<n>        # reproduce / explore a random run
#   make SVA=0           # build without the RTL assertions (on by default)
#   make LOCKSTEP=0      # no DUT-vs-model compare: monitors (+ SVA) judge alone
#   make MONITORS=0      # no concurrent monitors (mutation attribution only)
#   make model           # standalone pytest of the golden model + layout (no RTL)
#   make clean

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

RTL_DIR ?= ../../rtl
VERILOG_SOURCES += $(RTL_DIR)/lane_ram.sv

TOPLEVEL = lane_ram
COCOTB_TEST_MODULES = test_lane_ram

# make the shared common/ importable (tile_layout for the operand-stream test)
export PYTHONPATH := $(PWD)/../common:$(PYTHONPATH)

# parameters & passing them into Verilator (small ADDR_W: random traffic revisits words)
LANES  ?= 5
WIDTH  ?= 8
ADDR_W ?= 5
SEED   ?= 12345
SVA    ?= 1

COMPILE_ARGS += -GLANES=$(LANES) -GWIDTH=$(WIDTH) -GADDR_W=$(ADDR_W)
# Verilator >= 5.0xx enables assertions by default; say it explicitly either way
ifeq ($(SVA), 1)
	COMPILE_ARGS += --assert
else
	COMPILE_ARGS += --no-assert
endif

# exporting parameters to testbench
export LANES
export WIDTH
export ADDR_W
export SEED

ifeq ($(WAVES), 1)
	EXTRA_ARGS   += --trace --trace-structs
    export COCOTB_TRACE = 1
endif

include $(shell cocotb-config --makefiles)/Makefile.sim

model:
	python -m pytest test_lane_ram_model.py -q -p no:cacheprovider

clean::
	rm -rf sim_build __pycache__ .pytest_cache results.xml dump.fst dump.vcd
