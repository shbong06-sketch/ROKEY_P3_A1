"""
내부 컨베이어 반송 Standalone 실행기

컨베이어는 Play 중 계속 돌아갑니다. 시작할 때 벨트는 비어 있습니다.
팔레트를 줄기(ㅜ 의 기둥)에 올려놓으면 — 손으로 끌어다 놓든, 로봇이 가져다
놓든 — 벨트가 그것을 감지해서 물고 갑니다.

    줄기 -y → 교차점에서 +x 전환 → 비전룸 카메라 앞 정지
    → 검사 완료 신호 → +x 로 배출

수정할 곳
  SCENE_PATH   : 어느 월드 USD 를 열지
  PALLET_PATHS : 어느 팔레트를 감시할지
  PARK_X, PARK_Y : 시작할 때 팔레트를 세워 둘 자리 (벨트 옆 바닥)

Play  : 시작 / 일시정지한 위치에서 재개
Stop  : 다음 Play에서 처음부터 재시작
"""

import os

from isaacsim import SimulationApp

# HEADLESS=1: 창 없이 자동으로 돌리고 배출까지 끝나면 종료합니다. 고칠 때마다 확인용.
#   GUI      : ~/isaacsim/python.sh conveyor_standalone.py
#   헤드리스 : HEADLESS=1 ~/isaacsim/python.sh conveyor_standalone.py
HEADLESS = os.environ.get("HEADLESS") == "1"
app = SimulationApp({"headless": HEADLESS})

from pathlib import Path

import omni.usd

from isaacsim.core.api import World
from pxr import Gf, UsdGeom

import carb

from conveyor import ConveyorController, Zone, prepare_world


SCENE_PATH = (
    Path(__file__).resolve().parent.parent
    / "scenes"
    / "Collected_smartfarm_v011"
    / "Collected_smartfarm_v011.usd"
)

# 감시할 팔레트. 벨트에 올라오면 자동으로 반송합니다.
# 로봇이 랙에서 가져온 팔레트를 쓰려면 그 경로를 여기에 더하면 됩니다.
PALLET_PATHS = (
    "/World/SmartFarm/Placed/Pallet_Inspect",
    "/World/SmartFarm/Placed/Pallet_Inspect_01",
    "/World/SmartFarm/Placed/Pallet_Inspect_02",
    "/World/SmartFarm/Placed/Pallet_Inspect_03",
)

# 시작할 때 팔레트를 세워 둘 자리. 줄기 서쪽 빈 바닥이고, 벨트 구역 밖이라
# 컨베이어가 건드리지 않습니다. 여기서 줄기 위로 끌어다 놓으면 반송이 시작됩니다.
PARK_X = -3.40
PARK_Y = -5.60                 # 여기서부터 PARK_GAP 간격으로 늘어세웁니다
PARK_GAP = 0.60
PARK_Z = 0.03                  # 바닥(z 0) 위. 팔레트 바닥이 원점-0.025 라 거의 딱 닿습니다

# 헤드리스로 돌릴 때만 팔레트 한 장을 줄기에 올려 두고 시작합니다.
# 손으로 올릴 수 없으니, 코드가 도는지 확인하려면 이게 필요합니다.
# GUI 에서도 그렇게 시작하고 싶으면 AUTO_DROP=1 을 주세요.
AUTO_DROP = os.environ.get("AUTO_DROP", "1" if HEADLESS else "0") == "1"
DROP_POSITION = (-2.19, -4.00, 0.796)

RENDER_EVERY = 0 if HEADLESS else 3
PHYSICS_DT = 1.0 / 60.0


def enable_mouse_grab():
    """Play 중에 마우스로 물체를 집어 옮길 수 있게 합니다 (PhysX 마우스 상호작용).

    시뮬레이션이 도는 동안에는 물리가 위치를 쥐고 있어서, 기즈모로 끌면
    제자리로 돌아갑니다. 이 기능을 켜면 잡아끄는 힘으로 옮길 수 있습니다.
    """
    settings = carb.settings.get_settings()
    settings.set("/physics/mouseInteractionEnabled", True)
    settings.set("/physics/mouseGrab", True)
    settings.set("/physics/forceGrab", True)


def open_scene():
    """파일과 프림을 확인하고, 수정 대상이 세션 레이어인 stage를 반환합니다."""
    if not SCENE_PATH.is_file():
        raise FileNotFoundError(SCENE_PATH)

    omni.usd.get_context().open_stage(str(SCENE_PATH))
    while omni.usd.get_context().get_stage_loading_status()[2] > 0:
        app.update()

    stage = omni.usd.get_context().get_stage()
    stage.SetEditTarget(stage.GetSessionLayer())

    for path in PALLET_PATHS:
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"Prim이 없습니다: {path}")
    return stage


def move_pallet(stage, path, position):
    """팔레트를 이 자리로 옮깁니다. world.reset() 전에만 뜻이 있습니다."""
    for op in UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            op.Set(Gf.Vec3d(*position))
            return
    raise RuntimeError(f"{path} 에 translate 가 없습니다.")


def place_all(stage):
    """벨트를 비우고 팔레트를 옆 바닥에 늘어세웁니다."""
    for index, path in enumerate(PALLET_PATHS):
        move_pallet(stage, path, (PARK_X, PARK_Y + index * PARK_GAP, PARK_Z))
    if AUTO_DROP:
        move_pallet(stage, PALLET_PATHS[0], DROP_POSITION)


def main():
    stage = open_scene()
    if not HEADLESS:
        enable_mouse_grab()
    prepare_world(stage)          # 옆가이드 · 통로 점검 (월드당 한 번)

    conveyor = ConveyorController(stage)
    conveyor.build()
    for path in PALLET_PATHS:
        conveyor.watch(path)
    place_all(stage)

    if AUTO_DROP:
        print(f"[준비] {PALLET_PATHS[0].rsplit('/', 1)[-1]} 를 줄기에 올려 두고 시작합니다")
    else:
        print("[준비] 벨트는 비어 있습니다. 팔레트는 줄기 서쪽 바닥에 세워 두었습니다.")
        print("       줄기(x -2.64~-1.74, y -6.28~-3.60) 위로 올리면 반송이 시작됩니다.")
        print("       Stop 중 기즈모로 옮긴 뒤 Play하거나, Play 중 PhysX 마우스 잡기를 사용하세요.")

    world = World(physics_dt=PHYSICS_DT, stage_units_in_meters=1.0)

    step_count = 0
    needs_reset = True

    print("Play: 시작/재개 | Pause: 대기 | Stop: 처음부터 재시작")
    if HEADLESS:
        world.play()

    while app.is_running():
        if world.is_stopped():
            needs_reset = True
            world.render()
            continue

        if not world.is_playing():
            world.render()
            continue

        if needs_reset:
            needs_reset = False
            conveyor.reset()       # Stop 후에는 감시 상태를 지우고 새로 시작합니다.
            world.reset()
            conveyor.attach()      # 롤러·팔레트 핸들은 reset 뒤에 다시 잡아야 합니다.
            # 여기서 팔레트를 다시 늘어세우지 않습니다. 그러면 직접 옮겨 놓은
            # 팔레트를 Play 누를 때마다 도로 가져가 버립니다.

        conveyor.update(PHYSICS_DT)

        # 비전 노드가 붙으면 여기서 추론·픽앤플레이스를 돌리고 끝나면 알립니다.
        #   if conveyor.inspecting:
        #       ...
        #       conveyor.inspection_done()
        # 지금은 conveyor.AUTO_RESUME_SECONDS 가 대신 내보냅니다.

        step_count += 1
        render = RENDER_EVERY > 0 and step_count % RENDER_EVERY == 0
        world.step(render=render)

        # 한 장이라도 끝까지 나가면 한 사이클이 끝난 것으로 봅니다.
        if HEADLESS and any(conveyor.zone_of(p) is Zone.GONE for p in PALLET_PATHS):
            print("[완료] 팔레트를 내보냈습니다.")
            return


try:
    main()
finally:
    app.close()
