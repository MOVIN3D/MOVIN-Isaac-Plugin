"""Run a script and save viewport screenshots at chosen simulation steps.

Used by ``scripts/run_e2e.sh --gui`` to check what the windowed app renders.
Patches ``SimulationContext.step`` once ``AppLauncher`` has started the app, so the
target script runs unmodified.

Usage:
    CAPTURE_PREFIX=build/e2e/shots/bvh CAPTURE_STEPS=100,200 \
        python scripts/capture_viewport.py examples/mocap_to_isaaclab.py --mode bvh ...

Writes ``<CAPTURE_PREFIX>_step<N>.png`` for every N in ``CAPTURE_STEPS``.
"""

import os
import runpy
import sys

PREFIX = os.environ["CAPTURE_PREFIX"]
STEPS = {int(s) for s in os.environ.get("CAPTURE_STEPS", "150").split(",")}

from isaaclab.app import AppLauncher  # noqa: E402

_launcher_init = AppLauncher.__init__


def _init_and_hook_step(self, *args, **kwargs):
    _launcher_init(self, *args, **kwargs)
    # Only importable once the app is running.
    import isaaclab.sim as sim_utils
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    sim_step = sim_utils.SimulationContext.step
    count = 0

    def step(sim, *step_args, **step_kwargs):
        nonlocal count
        result = sim_step(sim, *step_args, **step_kwargs)
        count += 1
        if count in STEPS:
            path = f"{PREFIX}_step{count}.png"
            capture_viewport_to_file(get_active_viewport(), path)
            print(f"[CAPTURE] {path}", flush=True)
        return result

    sim_utils.SimulationContext.step = step


AppLauncher.__init__ = _init_and_hook_step

script = os.path.abspath(sys.argv[1])
sys.argv = sys.argv[1:]
sys.path.insert(0, os.path.dirname(script))
runpy.run_path(script, run_name="__main__")
