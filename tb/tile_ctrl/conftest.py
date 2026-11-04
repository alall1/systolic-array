# pytest only (the cocotb run uses PYTHONPATH from the Makefile): make the
# shared common/ models, FeederModel and PEModel importable.
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
for d in ("common", "feeder", "pe"):
    sys.path.insert(0, os.path.join(_HERE, "..", d))
