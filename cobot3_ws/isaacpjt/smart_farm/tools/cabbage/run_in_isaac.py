"""Run a plain pxr script with Isaac Sim 5.1's own USD (so binary layers stay readable by Isaac).

  D:\\isaacsim\\python.bat run_in_isaac.py SCRIPT.py [args...]
"""
import sys, runpy
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
script = sys.argv[1]
sys.argv = sys.argv[1:]
try:
    runpy.run_path(script, run_name="__main__")
finally:
    app.close()
