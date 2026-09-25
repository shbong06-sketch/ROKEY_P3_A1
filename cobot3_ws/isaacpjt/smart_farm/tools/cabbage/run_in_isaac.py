"""Run a plain pxr script with Isaac Sim 5.1's own USD (so binary layers stay readable by Isaac).

  D:\\isaacsim\\python.bat run_in_isaac.py SCRIPT.py [args...]
Exceptions are also written to <SCRIPT>.error.txt (Kit sometimes swallows stderr on shutdown).
"""
import sys, runpy, traceback
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
script = sys.argv[1]
sys.argv = sys.argv[1:]
try:
    runpy.run_path(script, run_name="__main__")
except BaseException:
    open(script + ".error.txt", "w", encoding="utf-8").write(traceback.format_exc())
    traceback.print_exc()
    raise
finally:
    app.close()
