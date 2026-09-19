from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_red_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 0.15]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),
)


world.scene.add_default_ground_plane()                  # 5. Scene
cube_red = world.scene.add(cube_red_prim)

world.reset()
world.stop()
teleported = False

n = 0
reset_required = True

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)

    if world.is_stopped():
                if not reset_required:
                    print("Simulation stopped")

                reset_required = True
                continue

    if world.is_playing():
        if reset_required:
            world.reset()

            n = 0
            teleported = False
            reset_required = False

            print(f"[리셋] Play 시작 -> Step_count: {n}")
            continue

    n += 1
    
    if n % 100 == 0:
        print(f"Step: {n}")
    
    if n % 300 == 0 and not teleported:
        cube_red.set_world_pose(
            position = np.array([0.0, 0.0, 1.0])
        )
        teleported = True

    if n % 500 == 0:
        world.stop()
        print('시뮬레이션 종료')
        break

simulation_app.close()