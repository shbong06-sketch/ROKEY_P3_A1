"""
내부 컨베이어 반송 제어

컨베이어는 Play 중 계속 돌아갑니다. 로봇이 ㅜ 자 컨베이어의 세로줄기(아래쪽 짧은
벨트)에 팔레트를 내려놓으면, 벨트가 알아서 물고 갑니다.

    줄기 -y 주행 → 교차점에서 +x 로 전환 → 비전룸 카메라 앞 정지
    → 솎아내기 → 다시 +x 로 배출 (벽을 지나 월드 밖으로)

'시작' 명령이 따로 없습니다. 팔레트가 벨트에 올라온 것을 구역 감지로 알아채고
그때부터 실제 컨베이어처럼 반응합니다. 로봇 동작은 Play 로 시작하고, 컨베이어는
그 옆에서 계속 돌아가는 형태입니다.

쓰는 법 — 세 자리에서 부르면 됩니다. 순서가 곧 계약입니다.

    conveyor = install(stage, PALLET_PATHS, auto_resume=None)   # world.reset() 전
    world.reset()
    conveyor.attach()                                           # world.reset() 뒤 한 번
    while ...:
        conveyor.update(PHYSICS_DT)                             # 물리 스텝마다
        if conveyor.inspecting:              # 카메라 앞에 선 팔레트 경로
            ...비전 · 솎아내기 픽앤플레이스...
            conveyor.inspection_done()       # 끝나면 알려 줍니다 → 다시 내보냅니다
        if conveyor.fault:                   # (reason, detail) — 막혔을 때
            ...

    Stop → Play 로 다시 돌릴 때는 reset() → world.reset() → attach() 순입니다.

    install() 을 쓰지 않고 직접 조립해도 됩니다. 그때는 prepare_world(stage) →
    ConveyorController(stage) → build() → watch() 를 모두 world.reset() 전에
    끝내야 합니다. 순서를 틀리면 에러 없이 팔레트가 움직이지 않습니다.

구역 규칙 (매 스텝 다시 판단합니다)
    줄기      항상 -y. 포크가 내려놓으면 알아서 끌려 내려옵니다.
    교차점    팔레트가 들어오고 비전룸이 비어 있으면 휠을 솟구쳐 +x 로 넘깁니다.
              비전룸이 차 있으면 넘기지 않고 세워 둡니다 (뒤 팔레트가 대기).
    가로줄기  항상 +x. 단 검사 중인 팔레트가 있으면 멈춥니다.
    비전룸    팔레트가 정지선을 넘으면 세우고 고정합니다. 그리퍼가 상추를 뽑을 때
              팔레트가 밀리지 않게 하기 위해서입니다. inspection_done() 에 풀립니다.

비전 단계에서 알아 둘 것
  · 기본값은 검사 신호가 없어도 10초 후 자동 배출합니다.
    AUTO_RESUME_SECONDS = None 이면 inspection_done() 이 올 때까지 기다립니다.
  · 로봇에 넘길 좌표는 pallet_position() 으로 읽으세요. 정지 위치는 관성 때문에
    VISION_X 보다 10~20 mm 더 갑니다.

팔레트는 에셋의 '실제' 롤러가 밉니다. 구동 방식은 conveyor_rollers.py 를 보세요.

이 파일은 v011 월드의 컨베이어 치수를 상수로 갖고 있습니다.
월드가 바뀌면 아래 '구간 치수' 만 다시 재서 고치면 됩니다.
"""

from enum import Enum

import numpy as np
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from isaacsim.core.prims import RigidPrim

from conveyor_rollers import DEFAULT_SPEED, ROLLER_TOP, RollerDrive

PLACED_ROOT = "/World/SmartFarm/Placed"   # 유령 강체를 찾을 범위


# ══ 조절할 것 두 가지 ══════════════════════════════════
# 아래 둘은 기본값입니다. 부르는 쪽에서 바꾸려면 파일을 고치지 말고
# install(..., vision_x=..., auto_resume=...) 로 넘기세요.
#
# 비전룸에서 세우는 자리. 팔레트 원점이 이 x 를 넘으면 벨트를 멈추고 고정합니다.
# 관성 때문에 10~20 mm 더 가서 섭니다. 실제로 선 자리는 pallet_position() 으로
# 읽으세요. 카메라 화각에 맞춰 옮기려면 이 값만 바꾸면 됩니다.
VISION_X = -0.50

# 검사 완료 신호(inspection_done())가 없을 때 스스로 다시 출발하기까지의 시간(초).
#   None  : 신호가 올 때까지 무한정 기다립니다. 비전 노드를 붙이면 이쪽입니다.
#   숫자  : 그 시간이 지나면 알아서 내보냅니다. 비전 없이 돌려 볼 때 씁니다.
AUTO_RESUME_SECONDS = 10.0
# ═══════════════════════════════════════════════════════


# ── 구간 치수 (Collected_smartfarm_v011 실측) ───────────
# 옆가이드. 주행 중 팔레트가 y 로 흘러 벨트 밖으로 나가지 않게 잡아 줍니다.
GUIDE_X = (-1.700, 10.400)
GUIDE_N_Y = (-6.570, -6.550)
GUIDE_S_Y = (-6.950, -6.930)
GUIDE_HEIGHT = 0.130
GUIDE_ROOT = "/World/ConveyorGuide"

# 구역 경계
# 팔레트가 '벨트에 올라와 있는가' 는 x·y·z 를 모두 봅니다. y 만 보면 벨트 옆
# 바닥에 세워 둔 팔레트도 줄기에 있는 것으로 잡힙니다.
LINE_BAND_Y = (-7.25, -6.25)   # 가로줄기·교차점의 y 띠
BELT_Z = (0.74, 0.90)          # 적재면(0.769) 위에 얹혀 있는 높이대
BELT_WEST_X = -2.75            # 벨트 구역의 서쪽 끝 (줄기·교차점)

JUNCTION_Y = -6.72             # 줄기에서 여기까지 내려오면 교차점에 들어온 것입니다
# 교차점을 완전히 빠져나온 x. 줄기·교차점 띠와 가로줄기 띠의 경계이기도 합니다.
# 팔레트 뒤쪽이 아직 교차점 위에 있을 때 휠을 내리면 소터롤러가 다시 -y 로
# 끌어당겨 팔레트가 제자리에서 버팁니다. 교차점 휠은 x -1.734 에서 끝나고
# 팔레트 반길이가 0.276 이라 -1.458 이면 완전히 벗어납니다. 여유를 둬 -1.40.
FEEDER_EXIT_X = -1.40
EJECT_X = 10.20                # 여기를 넘으면 배출 완료로 봅니다
# 줄기 롤러 위쪽 끝(-3.60)보다 조금 너그럽게 잡습니다. 딱 맞추면 마지막 롤러
# 위에 놓인 팔레트를 놓칩니다 — 로봇이 어디에 내려놓을지 모르므로 여유를 둡니다.
# 높이(BELT_Z)와 x 도 함께 보므로 벨트 밖을 잘못 집지는 않습니다.
STEM_ENTRY_Y = -3.55

# 벨트가 도는데 팔레트가 이만큼 오래 제자리면 고장으로 신고합니다.
# '구역에 오래 있으면' 이 아닙니다 — 비전룸이 차서 교차점에서 정상 대기 중인
# 팔레트까지 고장으로 잡히기 때문입니다. 실제로 안 움직이는 것만 봅니다.
STUCK_SECONDS = 15.0
STALL_SPEED = 0.06             # 이보다 느리면 멈춘 것으로 봅니다 (m/s, 벨트는 0.30)
# fault 로 내보내는 reason 코드. Task Manager 인터페이스 문서의 코드와 같습니다.
STUCK_REASON = "CONVEYOR_FAILED"

# 컨베이어는 벽 세 장(비전룸 입·출구, 건물 외벽)을 지납니다. 맵에 문이 뚫려
# 있어야 하고, 시작할 때 _check_path() 가 실제로 뚫려 있는지 확인합니다.
# 아래는 그때 '비어 있어야 하는' 부피입니다.
CLEARANCE_Y = (-7.30, -6.20)
CLEARANCE_Z = (0.50, 1.10)
CORRIDOR_X = (-2.80, 10.60)    # 줄기 입구부터 배출 끝까지

LOCKED_ROT_Z = 4               # physxRigidBody:lockedRotAxis 의 Z 비트
PALLET_HALF_LEN = 0.276        # 팔레트 반길이. 배출선을 대조할 때 씁니다


class ConveyorError(RuntimeError):
    """컨베이어 때문에 작업을 멈춰야 할 때"""


class Zone(str, Enum):
    """팔레트가 지금 어느 구역에 있는가"""
    OFF = "OFF"                # 벨트 밖 (아직 안 올라왔거나 이미 나감)
    STEM = "STEM"              # 줄기를 -y 로 내려오는 중
    JUNCTION = "JUNCTION"      # 교차점 안
    LINE = "LINE"              # 가로줄기 주행 중
    VISION = "VISION"          # 카메라 앞 정지선을 넘음
    GONE = "GONE"              # 배출 완료


# ── 설치 (world.reset() 전에 한 번) ──────────────────────
def install(stage, pallet_paths, vision_x=VISION_X,
            auto_resume=AUTO_RESUME_SECONDS, speed=DEFAULT_SPEED):
    """컨베이어를 씬에 걸고 컨트롤러를 돌려줍니다. world.reset() 전에 부르세요.

    world.reset() 이전에 해야 하는 일을 전부 여기서 합니다. 부르는 쪽은
    순서를 외울 필요 없이 이것만 부르고, reset() 뒤에 attach() 한 번,
    이후 스텝마다 update(dt) 만 부르면 됩니다.

        conveyor = install(stage, PALLET_PATHS, auto_resume=None)
        world.reset()
        conveyor.attach()

    pallet_paths : 감시할 팔레트 프림 경로들. 로봇이 나중에 올려놓을 팔레트도
                   여기에 미리 넣어 둡니다 (벨트 밖에 있으면 가만히 있습니다).
    vision_x     : 비전룸에서 세울 x. 카메라 화각·로봇 도달거리에 맞춥니다.
    auto_resume  : 검사 신호 없이 스스로 내보내기까지의 초. 비전 노드를 붙일
                   때는 None 으로 두고 inspection_done() 을 기다립니다.
    """
    prepare_world(stage)
    controller = ConveyorController(stage, speed,
                                    vision_x=vision_x, auto_resume=auto_resume)
    controller.build()
    for path in pallet_paths:
        controller.watch(path)
    _check_layout(controller._drive)
    _check_belt_clear(stage, pallet_paths)
    return controller


def _check_layout(drive):
    """에셋을 실제로 재서 이 파일의 구간 치수와 맞는지 대조합니다.

    맵의 컨베이어가 옮겨지거나 바뀌면 상수가 틀어집니다. 그러면 팔레트를
    '벨트 위' 로 인식하지 못해 아무 일도 일어나지 않는데, 에러가 없어서
    원인을 찾기가 어렵습니다. 시작할 때 숫자를 맞춰 보고 알려 줍니다.
    """
    (sx0, sy0, sz0), (sx1, sy1, sz1) = drive.stem.extent
    (lx0, ly0, lz0), (lx1, ly1, lz1) = drive.line.extent
    top = max(sz1, lz1)

    print(f"[컨베이어] 실측 — 적재면 z {top:.3f} (상수 {ROLLER_TOP:.3f}) · "
          f"가로줄기 y {ly0:+.2f}~{ly1:+.2f} x {lx0:+.2f}~{lx1:+.2f} · "
          f"줄기 y {sy0:+.2f}~{sy1:+.2f}")

    # 치명적 — 이게 어긋나면 팔레트를 영영 감지하지 못합니다.
    if not BELT_Z[0] <= top <= BELT_Z[1]:
        raise ConveyorError(
            f"적재면 z {top:.3f} 가 BELT_Z {BELT_Z} 밖입니다. "
            f"이대로면 팔레트를 벨트 위로 인식하지 못합니다. BELT_Z 를 다시 재세요.")
    if not (LINE_BAND_Y[0] <= ly0 and ly1 <= LINE_BAND_Y[1]):
        raise ConveyorError(
            f"가로줄기 y {ly0:+.3f}~{ly1:+.3f} 가 LINE_BAND_Y {LINE_BAND_Y} 를 "
            f"벗어납니다. 구간 치수를 다시 재세요.")

    # 경고 — 돌기는 하지만 판정이 이상해집니다.
    # 배출선이 벨트 끝보다 조금 앞인 것은 정상입니다 — 원점이 배출선을 넘으면
    # 팔레트 앞쪽은 이미 벨트를 벗어납니다. 한 장 길이 넘게 앞이면 이릅니다.
    if EJECT_X + 2 * PALLET_HALF_LEN < lx1:
        print(f"[컨베이어] 주의 — EJECT_X {EJECT_X:+.2f} 가 벨트 끝 {lx1:+.2f} 보다 "
              f"많이 앞입니다. 아직 벨트 위인데 배출 완료로 봅니다.")
    if STEM_ENTRY_Y < sy1:
        print(f"[컨베이어] 주의 — STEM_ENTRY_Y {STEM_ENTRY_Y:+.2f} 가 줄기 끝 "
              f"{sy1:+.2f} 보다 아래입니다. 줄기 위쪽에 놓은 팔레트를 놓칩니다.")


def _check_belt_clear(stage, watched):
    """벨트 위에 감시 대상이 아닌 강체가 얹혀 있으면 알려 줍니다.

    표면속도 구동은 닿는 것을 가리지 않고 밉니다. 감시하지 않는 팔레트도
    제멋대로 밀려나가 길을 막는데, 컨베이어는 그 존재를 모르니 조용합니다.
    """
    root = stage.GetPrimAtPath(PLACED_ROOT)
    if not root or not root.IsValid():
        return
    cache = UsdGeom.XformCache()
    strays = []
    for prim in root.GetChildren():
        path = prim.GetPath().pathString
        if path in watched:
            continue
        if not any(child.HasAPI(UsdPhysics.RigidBodyAPI)
                   for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(
                       Usd.PrimAllPrimsPredicate))):
            continue
        x, y, z = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        if BELT_Z[0] <= z <= BELT_Z[1] and LINE_BAND_Y[0] <= y <= LINE_BAND_Y[1]:
            strays.append(f"{prim.GetName()} ({x:+.2f}, {y:+.2f}, {z:.2f})")
    if strays:
        print(f"[컨베이어] 주의 — 벨트 위에 감시 대상이 아닌 물체가 "
              f"{len(strays)}개 있습니다: {', '.join(strays)}")
        print( "           롤러가 이것들도 밀어냅니다. 치우거나 "
               "install() 의 pallet_paths 에 넣으세요.")


# ── 월드 준비 (월드당 한 번, world.reset() 전에) ─────────
def prepare_world(stage):
    """컨베이어가 돌아갈 수 있게 월드를 손봅니다.

    컨트롤러와 분리해 둡니다. 여기서 하는 일은 월드에 한 번만 하면 되는 것이고,
    컨트롤러를 여러 번 만들거나 팔레트를 여러 장 다뤄도 반복할 필요가 없습니다.
    """
    _make_guides(stage)
    _check_path(stage)


def _make_guides(stage):
    """보이지 않는 옆가이드 두 줄. 적재면 위에 세웁니다."""
    stage.DefinePrim(GUIDE_ROOT, "Scope")
    for name, ys in (("GuideN", GUIDE_N_Y), ("GuideS", GUIDE_S_Y)):
        cube = UsdGeom.Cube.Define(stage, f"{GUIDE_ROOT}/{name}")
        cube.GetSizeAttr().Set(1.0)
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOpOrder()      # 두 번 불러도 op 가 쌓이지 않게 합니다
        xform.AddTranslateOp().Set(Gf.Vec3d((GUIDE_X[0] + GUIDE_X[1]) / 2,
                                            (ys[0] + ys[1]) / 2,
                                            ROLLER_TOP + GUIDE_HEIGHT / 2))
        xform.AddScaleOp().Set(Gf.Vec3f(GUIDE_X[1] - GUIDE_X[0],
                                        ys[1] - ys[0], GUIDE_HEIGHT))
        prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(True)
        UsdGeom.Imageable(prim).MakeInvisible()


def _check_path(stage):
    """컨베이어가 지나는 벽에 문이 뚫려 있는지 봅니다. 가정하지 않고 확인합니다."""
    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    blocked = []
    for prim in stage.GetPseudoRoot().GetChildren():
        if not prim.GetName().startswith("BackWall"):
            continue
        for part in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if not part.HasAPI(UsdPhysics.CollisionAPI):
                continue
            rng = cache.ComputeWorldBound(part).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            lo, hi = rng.GetMin(), rng.GetMax()
            if hi[0] < CORRIDOR_X[0] or lo[0] > CORRIDOR_X[1]:
                continue
            if hi[1] <= CLEARANCE_Y[0] or lo[1] >= CLEARANCE_Y[1]:
                continue
            if hi[2] <= CLEARANCE_Z[0] or lo[2] >= CLEARANCE_Z[1]:
                continue
            blocked.append(part.GetPath().pathString)
    if blocked:
        print(f"[컨베이어] 주의 — 컨베이어 통로를 막는 벽이 있습니다: {', '.join(blocked)}. "
              f"맵에 컨베이어 개구부를 내야 합니다.")
    else:
        print("[컨베이어] 통로 확인됨 — 벽 개구부 이상 없음")


def _tray_body(bodies):
    """리지드바디 여럿 중 트레이 본체를 고릅니다.

    팔레트 위 상추도 각각 리지드바디라 그냥 첫 번째를 쓰면 순서에 기대게 됩니다.
    가장 얕은 강체를 트레이로 봅니다. 같은 깊이의 후보가 여러 개면 중단합니다.
    """
    if not bodies:
        raise ConveyorError("트레이를 고를 리지드바디가 없습니다.")
    depths = [len(prim.GetPath().pathString.split("/")) for prim in bodies]
    min_depth = min(depths)
    candidates = [prim for prim, depth in zip(bodies, depths) if depth == min_depth]
    if len(candidates) != 1:
        paths = ", ".join(str(prim.GetPath()) for prim in candidates)
        raise ConveyorError(f"트레이 후보가 여러 개입니다: {paths}")
    return candidates[0]


class _Pallet:
    """감시 중인 팔레트 한 장"""

    def __init__(self, path, tray, body_paths):
        self.path = path
        self.name = path.rsplit("/", 1)[-1]
        self.tray = tray
        self.body_paths = body_paths
        self.rigid = None          # RigidPrim (world.reset() 뒤에 잡습니다)
        self.reset_state()

    def reset_state(self):
        """물리 고정 해제 또는 핸들 연결 후 감시 상태를 초기화합니다."""
        self.zone = Zone.OFF
        self.inspected = False
        self.locked = False
        self.held_seconds = 0.0
        self.stalled_seconds = 0.0
        self.last_xy = None
        self.fault = None          # 막혔을 때 (reason, detail)

    def attach(self):
        """핸들을 잡고, 물리 상태를 깨끗한 동적 강체로 되돌립니다.

        GUI 에서 시뮬레이션 중에 기즈모로 끌면 물체가 잠시 키네마틱이 되는데,
        그게 풀리지 않고 남으면 중력을 안 받아 공중에 뜬 것처럼 보입니다.
        검사 고정을 풀지 못한 채 Stop 을 눌러도 같은 상태가 됩니다.
        Stop → Play 마다 여기를 지나므로, 그때 원상복구됩니다.
        """
        self.rigid = RigidPrim(self.body_paths)
        self.rigid.enable_rigid_body_physics()
        self.rigid.set_velocities(np.zeros((len(self.body_paths), 6), dtype=np.float32))

    def position(self):
        matrix = UsdGeom.Xformable(self.tray).ComputeLocalToWorldTransform(0)
        t = matrix.ExtractTranslation()
        return float(t[0]), float(t[1]), float(t[2])

    def lock(self):
        """그리퍼가 잡아당겨도 밀리지 않게 그 자리에 고정합니다."""
        if self.locked or self.rigid is None:
            return
        self.rigid.disable_rigid_body_physics()
        self.locked = True

    def release(self):
        """고정을 풉니다.

        물리를 켜는 것만으로는 다시 움직이지 않습니다. 속도를 한 번 써 줘야
        깨어납니다. (풀기만 하고 벨트를 돌렸더니 0.0 mm 로 서 있었습니다)
        """
        if not self.locked or self.rigid is None:
            return
        self.rigid.enable_rigid_body_physics()
        self.rigid.set_velocities(np.zeros((len(self.body_paths), 6), dtype=np.float32))
        self.locked = False


class ConveyorController:
    """
    컨베이어를 계속 돌리면서, 벨트에 올라온 팔레트를 구역 감지로 처리합니다.

    실제 컨베이어처럼 '지금 어느 구역에 무엇이 있는가' 만 보고 매 스텝 다시
    판단합니다. 팔레트마다 순서를 지시하지 않습니다.
    """

    def __init__(self, stage, speed=DEFAULT_SPEED,
                 vision_x=VISION_X, auto_resume=AUTO_RESUME_SECONDS):
        """vision_x  : 비전룸에서 세울 x. 카메라 화각·로봇 도달거리에 맞춥니다.
        auto_resume : 검사 신호 없이 스스로 내보내기까지의 초.
                      None 이면 inspection_done() 이 올 때까지 기다립니다.
        """
        self._stage = stage
        self._drive = RollerDrive(stage, speed)
        self._pallets = []
        self._ready = False
        self._vision_x = vision_x
        self._auto_resume_seconds = auto_resume

    # ── 읽기 ────────────────────────────────────────
    @property
    def inspecting(self):
        """카메라 앞에 서서 솎아내기를 기다리는 팔레트 경로. 없으면 None."""
        pallet = self._held()
        return pallet.path if pallet else None

    @property
    def fault(self):
        """컨베이어가 막혔으면 (reason, detail), 아니면 None.

        팔레트가 한 구역에서 STUCK_SECONDS 를 넘기면 여기에 실립니다. 다시
        움직여 구역이 바뀌면 저절로 없어집니다. 부르는 쪽은 이 값을 그대로
        실패 사유로 올리면 됩니다.
        """
        for pallet in self._pallets:
            if pallet.fault is not None:
                return pallet.fault
        return None

    def zone_of(self, pallet_path):
        return self._find(pallet_path).zone

    def pallet_position(self, pallet_path=None):
        """팔레트의 월드 좌표 (x, y, z). 로봇에 넘길 좌표는 이걸 쓰세요.

        경로를 생략하면 지금 카메라 앞에 선 팔레트를 씁니다.
        """
        if pallet_path is None:
            pallet = self._held()
            if pallet is None:
                raise ConveyorError("카메라 앞에 선 팔레트가 없습니다.")
        else:
            pallet = self._find(pallet_path)
        return pallet.position()

    # ── 준비 (world.reset() 전에) ────────────────────
    def build(self):
        """롤러를 구동 가능하게 바꿉니다."""
        self._drive.build()

    def watch(self, pallet_path):
        """이 팔레트를 감시 대상에 넣고, 컨베이어를 탈 수 있게 물성을 맞춥니다.

        팔레트 위 상추도 각각 리지드바디라 함께 처리합니다.
        """
        root = self._stage.GetPrimAtPath(pallet_path)
        if not root:
            raise ConveyorError(f"팔레트를 찾지 못했습니다: {pallet_path}")
        bodies = [prim for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
                  if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        if not bodies:
            raise ConveyorError(f"{pallet_path} 아래에 리지드바디가 없습니다.")
        tray = _tray_body(bodies)  # 물성을 바꾸기 전에 본체를 확정합니다.

        for prim in bodies:
            body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            # 멈춰 있는 동안 PhysX 슬립에 들어가면, 표면속도는 접촉으로만 힘을 주기
            # 때문에 다시 깨우지 못합니다. 벨트를 재가동해도 그대로 섭니다.
            body.CreateSleepThresholdAttr().Set(0.0)
            body.CreateStabilizationThresholdAttr().Set(0.0)

        # 트레이가 주행 중 요(yaw)로 돌면 옆가이드 사이에 대각선으로 물려 멈춥니다.
        # 가이드 달린 실제 롤러 컨베이어처럼 방향을 유지시킵니다.
        PhysxSchema.PhysxRigidBodyAPI.Apply(tray).CreateLockedRotAxisAttr().Set(LOCKED_ROT_Z)

        self._pallets.append(_Pallet(pallet_path, tray,
                                     [prim.GetPath().pathString for prim in bodies]))
        print(f"[컨베이어] {root.GetName()} 감시 시작: 리지드바디 {len(bodies)}개 · 요 고정")

    # ── 준비 (world.reset() 뒤에) ────────────────────
    def attach(self):
        """롤러와 팔레트 핸들을 잡습니다. world.reset() 뒤에 한 번 부릅니다."""
        self._drive.attach()
        for pallet in self._pallets:
            pallet.attach()
            pallet.reset_state()
        self._ready = True

    # ── 명령 ────────────────────────────────────────
    def inspection_done(self):
        """검사 완료를 알립니다. 카메라 앞에 선 팔레트를 다시 내보냅니다.

        세워 둔 팔레트가 없으면 아무 일도 하지 않고 False 를 돌려줍니다.
        물리 루프 안에서 불리므로 예외를 던지지 않습니다.
        """
        pallet = self._held()
        if pallet is None:
            print("[컨베이어] 검사 완료 신호를 받았지만 카메라 앞에 팔레트가 없습니다")
            return False
        pallet.inspected = True
        pallet.release()
        print(f"[컨베이어] {pallet.name} 검사 완료 — 배출 시작")
        return True

    def reset(self):
        """감시 상태를 초기화합니다. Stop → Play 사이에 부르세요."""
        for pallet in self._pallets:
            pallet.release()
            pallet.reset_state()
        self._drive.rest()

    # ── 진행 ────────────────────────────────────────
    def update(self, dt):
        """물리 스텝마다 한 번씩 부릅니다. 벨트는 이 안에서 계속 돌아갑니다."""
        self._drive.spin(dt)
        if not self._ready:
            return

        # 지난 스텝 동안 벨트가 돌고 있었는지. 검사 중이면 가로줄기·교차점이
        # 일부러 서 있으므로, 그때 안 움직이는 것은 고장이 아닙니다.
        driving = self._held() is None
        for pallet in self._pallets:
            self._update_zone(pallet, dt, driving)
        self._auto_resume(dt)
        self._apply_drive()

    def _auto_resume(self, dt):
        """검사 완료 신호가 없을 때, 정해 둔 시간이 지나면 스스로 내보냅니다."""
        pallet = self._held()
        if pallet is None:
            return
        pallet.held_seconds += dt
        if self._auto_resume_seconds is None:
            return
        if pallet.held_seconds >= self._auto_resume_seconds:
            print(f"[컨베이어] {pallet.name} 검사 신호 없이 "
                  f"{self._auto_resume_seconds:.0f}초 경과 — 스스로 내보냅니다")
            self.inspection_done()

    # ── 내부 ────────────────────────────────────────
    def _update_zone(self, pallet, dt, driving=True):
        """팔레트가 지금 어느 구역에 있는지 다시 보고, 막혔는지 살핍니다."""
        if pallet.zone is Zone.GONE:
            return
        x, y, z = pallet.position()
        zone = self._zone_at(x, y, z, pallet.inspected)
        self._check_stall(pallet, x, y, dt, driving)

        if zone is not pallet.zone:
            was_off = pallet.zone is Zone.OFF
            pallet.zone = zone
            if zone is Zone.VISION:
                pallet.held_seconds = 0.0
                pallet.lock()
                print(f"[컨베이어] {pallet.name} 카메라 앞 정지 ({x:+.3f}, {y:+.3f})")
            elif zone is Zone.GONE:
                print(f"[컨베이어] {pallet.name} 배출 완료 ({x:+.3f}, {y:+.3f})")
            elif was_off:
                print(f"[컨베이어] {pallet.name} 벨트 감지 — 반송 시작 ({x:+.3f}, {y:+.3f})")

    def _check_stall(self, pallet, x, y, dt, driving):
        """벨트가 도는데 안 움직이면 fault 에 싣습니다.

        벨트는 계속 돌립니다. 멈출지 말지는 부르는 쪽이 정합니다.
        """
        moving_zone = pallet.zone in (Zone.STEM, Zone.JUNCTION, Zone.LINE)
        if not driving or not moving_zone:
            # 검사 중이거나 벨트 밖이면 제자리인 게 정상입니다.
            pallet.stalled_seconds = 0.0
            pallet.last_xy = (x, y)
            return

        if pallet.last_xy is None:
            pallet.last_xy = (x, y)
            return
        moved = float(np.hypot(x - pallet.last_xy[0], y - pallet.last_xy[1]))
        pallet.last_xy = (x, y)

        if moved >= STALL_SPEED * dt:
            pallet.stalled_seconds = 0.0
            pallet.fault = None
            return

        pallet.stalled_seconds += dt
        if pallet.fault is None and pallet.stalled_seconds > STUCK_SECONDS:
            detail = (f"{pallet.name} 이 {pallet.zone.value} 에서 "
                      f"{STUCK_SECONDS:.0f}초째 움직이지 않습니다 "
                      f"(x={x:+.3f} y={y:+.3f})")
            pallet.fault = (STUCK_REASON, detail)
            print(f"[컨베이어] 고장 — {detail}")

    def _zone_at(self, x, y, z, inspected):
        """좌표 하나로 구역을 판정합니다. 벨트 밖이면 OFF 입니다."""
        if not BELT_Z[0] <= z <= BELT_Z[1]:
            return Zone.OFF
        if x >= EJECT_X:
            return Zone.GONE
        if x >= FEEDER_EXIT_X:                      # 가로줄기
            if not LINE_BAND_Y[0] <= y <= LINE_BAND_Y[1]:
                return Zone.OFF
            if x >= self._vision_x and not inspected:
                return Zone.VISION
            return Zone.LINE
        if x >= BELT_WEST_X:                        # 줄기와 교차점 (동쪽 끝은 위에서 걸렀습니다)
            if LINE_BAND_Y[0] <= y <= JUNCTION_Y:
                return Zone.JUNCTION
            if JUNCTION_Y < y <= STEM_ENTRY_Y:
                return Zone.STEM
        return Zone.OFF

    def _apply_drive(self):
        """구역 점유 상태만 보고 벨트를 다시 세팅합니다.

        구역을 하나 더 넣고 싶으면 여기에 조건 한 줄 더하면 됩니다.
        """
        vision_busy = self._held() is not None
        junction_busy = any(p.zone is Zone.JUNCTION for p in self._pallets)

        # 교차점: 팔레트가 들어왔고 비전룸이 비어 있을 때만 휠을 솟구쳐 넘깁니다.
        # 비전룸이 차 있으면 넘기지 않고 세워 둡니다 (뒤 팔레트가 교차점에서 대기).
        divert = junction_busy and not vision_busy
        if divert:
            self._drive.divert()
        elif junction_busy:
            self._drive.wait_junction()    # 비전룸이 빌 때까지 교차점에 세워 둡니다
        else:
            self._drive.feed()

        # 가로줄기: 검사 중인 팔레트가 있으면 멈춥니다.
        if vision_busy:
            self._drive.line_stop()
        else:
            self._drive.line_run()

    def _held(self):
        """카메라 앞에 서서 검사를 기다리는 팔레트 (없으면 None)"""
        for pallet in self._pallets:
            if pallet.zone is Zone.VISION and not pallet.inspected:
                return pallet
        return None

    def _find(self, pallet_path):
        for pallet in self._pallets:
            if pallet.path == pallet_path:
                return pallet
        raise ConveyorError(f"감시 중이 아닙니다: {pallet_path}")
