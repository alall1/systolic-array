# pytest only (the cocotb run uses PYTHONPATH from the Makefile): make the
# shared common/ helpers and every block model importable for the closed loop.
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for d in ("common", "feeder", "pe", "tile_ctrl", "collector", "lane_ram"):
    sys.path.insert(0, os.path.join(_HERE, "..", d))
