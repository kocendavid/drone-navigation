"""Execute this file from Isaac Sim's Script Editor while the timeline is stopped."""

import builtins
import runpy
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent
for directory in (PROJECT, PROJECT / ".vendor"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))

previous = getattr(builtins, "_px4_isaac_bridge", None)
if previous is not None:
    previous.close()

# Read both the bootstrap and package files directly, including on repeated
# executions in Script Editor. Kit's import cache can ignore importlib.reload.
bootstrap = runpy.run_path(str(PROJECT / "bridge_loader.py"))
bridge_module = bootstrap["load_bridge_modules"](PROJECT)
print(f"[PX4 bridge loader] Fresh source loaded from {PROJECT}", flush=True)
builtins._px4_isaac_bridge = bridge_module.IsaacPX4Bridge(PROJECT / "bridge_config.json")
