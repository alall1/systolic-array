# pytest only (the cocotb run uses PYTHONPATH from the Makefile): make the
# shared common/ models, FeederModel, PEModel and the tile_ctrl model + its
# closed-loop Harness (feeder + PE grid) importable.
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for d in ("common", "feeder", "pe", "tile_ctrl"):
    sys.path.insert(0, os.path.join(_HERE, "..", d))
