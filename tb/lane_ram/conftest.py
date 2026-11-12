# pytest only (the cocotb run uses PYTHONPATH from the Makefile): make the
# shared common/ helpers and the feeder, PE, tile_ctrl and collector models
# importable for the closed-loop layout check.
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for d in ("common", "feeder", "pe", "tile_ctrl", "collector"):
    sys.path.insert(0, os.path.join(_HERE, "..", d))
