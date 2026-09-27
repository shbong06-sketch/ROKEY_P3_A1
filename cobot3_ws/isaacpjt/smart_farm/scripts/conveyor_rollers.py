"""
컨베이어 롤러 구동

에셋의 '실제' 롤러를 굴려서 팔레트를 나릅니다. 보이지 않는 판을 깔지 않습니다.

    롤러를 키네마틱 리지드바디로 바꾸고 PhysxSurfaceVelocityAPI 로 표면 속도를
    주면, 그 마찰이 팔레트를 밀어 냅니다. 같은 속도에 맞춰 롤러를 실제로
    회전시키므로 화면에서도 굴러가는 것이 보입니다.

    교차점(Feeder)은 실제 소터와 같은 구조입니다.
        SorterRoller  6개 : 축이 x, 윗면 0.7693  → ±y 로 나릅니다
        Wheel/Band   42개 : 축이 y, 윗면 0.7672  → ±x 로 나릅니다
    소터롤러가 2.1 mm 높아 평소에는 팔레트가 롤러 위에 있습니다. 방향을 바꿀 때
    실제 소터처럼 휠을 롤러 위로 솟구치게 해서 팔레트를 넘겨받습니다.

쓰는 법
    drive = RollerDrive(stage)
    drive.build()                       # world.reset() 전에
    world.reset()
    drive.attach()                      # world.reset() 뒤에 한 번
    drive.stem.set_speed(0, -0.30)      # 줄기가 -y 로 나릅니다
    ...
    drive.spin(PHYSICS_DT)              # 물리 스텝마다 (실제 회전)

알아 둘 것
  · 표면속도는 0 이 아닌 값으로 파싱되어야 합니다. 처음에 정확히 0 을 넣으면
    기능이 꺼진 채로 시작해, 이후 무엇을 넣어도 반영되지 않습니다.
    그래서 '정지' 는 0 대신 STOP_SPEED(0.1 mm/s) 로 표현합니다.
  · surfaceVelocityLocalSpace 를 False 로 못 박아야 합니다. 주지 않으면
    표면속도가 지시값과 다르게 해석됩니다.
"""

import math

import numpy as np
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from isaacsim.core.prims import RigidPrim


CONV = "/World/SmartFarm/Placed/Conveyor"

# ── 치수 (Collected_smartfarm_v011 실측) ────────────────
ROLLER_TOP = 0.769             # 적재면 (롤러 윗면)
ROLLER_RADIUS = 0.029          # 롤러 반지름 (지름 0.058)
WHEEL_RADIUS = 0.0535          # 교차점 휠 반지름 (지름 0.107)

DEFAULT_SPEED = 0.30           # m/s
STOP_SPEED = 1e-4              # '정지' 로 쓰는 값 (0.1 mm/s)

# 교차점에서 휠이 소터롤러 위로 솟는 높이. 높이차 2.1 mm 보다 넉넉히 잡습니다.
POP_UP = 0.006

LINE_SEGMENTS = range(2, 8)    # 가로줄기 Seg_2 ~ Seg_7

# 속이 꽉 찬 convexHull 로 들어와 팔레트를 막는 프레임들.
# Seg_N 프레임은 이미 meshSimplification 이라 손댈 필요가 없습니다.
SOLID_FRAMES = (
    f"{CONV}/TurnTable/Geometry/SM_ConveyorBelt_A08_01",
    f"{CONV}/Feeder/Geometry/SM_ConveyorBelt_A49_01",
)


def _quat_mul(one, many):
    """회전 하나를 여러 자세 앞에 곱합니다. (w, x, y, z)"""
    w1, x1, y1, z1 = one
    w2, x2, y2, z2 = many[:, 0], many[:, 1], many[:, 2], many[:, 3]
    return np.stack([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], axis=1)


class RollerError(RuntimeError):
    """롤러 에셋을 찾지 못했을 때.

    맵의 컨베이어 구조가 바뀌면 여기서 걸립니다. 조용히 안 도는 것보다
    시작할 때 멈추는 편이 낫습니다 — 안 돌면 원인을 찾기가 매우 어렵습니다.
    """


class RollerGroup:
    """
    같은 축으로 도는 롤러 묶음.

    표면속도로 팔레트를 밀고(실제 구동), 같은 속도로 눈에 보이게 회전합니다.
    axis 는 롤러의 회전축입니다. 'x' 면 x축 둘레로 돌아 ±y 로, 'y' 면 ±x 로 나릅니다.
    """

    def __init__(self, label, prims, axis, radius):
        """radius 가 None 이면 굴리지 않고 표면속도만 줍니다.

        교차점 밴드처럼 납작한 벨트는 돌리면 프로펠러가 되어 팔레트를
        튕겨 올립니다. 둥근 부품만 회전시킵니다.
        """
        self.label = label
        self._axis = axis
        self._radius = radius
        self._paths = [p.GetPath().pathString for p in prims]
        self._api = []
        self._handle = None
        self._angle = 0.0
        self._speed = 0.0
        self._lift = 0.0
        self._applied_lift = 0.0
        self._origin = None
        self._quat = None

        cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        pivots, corners = [], []
        for prim in prims:
            UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(True)
            api = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
            api.CreateSurfaceVelocityEnabledAttr().Set(True)
            api.CreateSurfaceVelocityLocalSpaceAttr().Set(False)
            api.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(STOP_SPEED, 0.0, 0.0))
            self._api.append(api)

            # 회전 중심은 롤러의 bbox 중심입니다. 바디 원점이 축 위에 없는
            # 부품(교차점 휠)도 있어서, 원점이 아니라 이 점을 축으로 돌립니다.
            rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            lo, hi = rng.GetMin(), rng.GetMax()
            pivots.append(((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2))
            corners.append((lo[0], lo[1], lo[2]))
            corners.append((hi[0], hi[1], hi[2]))
        self._pivot = np.array(pivots, dtype=np.float64)

        # 이 묶음이 실제로 차지하는 범위. 맵이 바뀌었는지 대조하는 데 씁니다.
        box = np.array(corners, dtype=np.float64) if corners else None
        self.extent = None if box is None else (box.min(axis=0), box.max(axis=0))

    def __len__(self):
        return len(self._paths)

    def attach(self):
        """world.reset() 뒤에 회전용 핸들을 잡습니다."""
        if not self._paths:
            return
        self._handle = RigidPrim(self._paths)
        origin, quat = self._handle.get_world_poses()
        self._origin = np.asarray(origin, dtype=np.float64)
        self._quat = np.asarray(quat, dtype=np.float64)

    def set_speed(self, vx, vy):
        """이 묶음이 나르는 속도 (m/s). 정확한 0 은 쓰지 않습니다."""
        if vx == 0.0 and vy == 0.0:
            vx = STOP_SPEED
        for api in self._api:
            api.GetSurfaceVelocityAttr().Set(Gf.Vec3f(vx, vy, 0.0))
        # 회전 방향은 나르는 방향에서 나옵니다.
        #   축 x : 윗면이 -y 로 흐르려면 +x 둘레로 돌아야 합니다.
        #   축 y : 윗면이 +x 로 흐르려면 +y 둘레로 돌아야 합니다.
        self._speed = -vy if self._axis == "x" else vx

    def lift(self, height):
        """묶음 전체를 이만큼 들어 올립니다 (교차점 휠 솟구치기)."""
        self._lift = height

    def spin(self, dt):
        """나르는 속도에 맞춰 실제로 돌립니다. 물리 스텝마다 부릅니다."""
        if self._handle is None:
            return
        turning = self._radius is not None and abs(self._speed) > STOP_SPEED
        if not turning and self._lift == self._applied_lift:
            return

        if turning:
            self._angle += (self._speed / self._radius) * dt
        angle = self._angle
        cos, sin = math.cos(angle), math.sin(angle)
        half = angle / 2.0

        rel = self._origin - self._pivot
        moved = np.empty_like(rel)
        if self._axis == "x":
            moved[:, 0] = rel[:, 0]
            moved[:, 1] = rel[:, 1] * cos - rel[:, 2] * sin
            moved[:, 2] = rel[:, 1] * sin + rel[:, 2] * cos
            turn = (math.cos(half), math.sin(half), 0.0, 0.0)
        else:
            moved[:, 0] = rel[:, 0] * cos + rel[:, 2] * sin
            moved[:, 1] = rel[:, 1]
            moved[:, 2] = -rel[:, 0] * sin + rel[:, 2] * cos
            turn = (math.cos(half), 0.0, math.sin(half), 0.0)

        position = self._pivot + moved
        position[:, 2] += self._lift
        self._handle.set_world_poses(position, _quat_mul(turn, self._quat))
        self._applied_lift = self._lift


class RollerDrive:
    """컨베이어의 롤러와 밴드를 다섯 묶음으로 나눠 다룹니다."""

    def __init__(self, stage, speed=DEFAULT_SPEED):
        self._stage = stage
        self.speed = speed
        self.stem = None        # ㅜ 의 기둥. 로봇이 여기에 내려놓습니다. ±y
        self.sorter = None      # 교차점 소터롤러. ±y
        self.cross = None       # 교차점 휠. ±x, 솟구침, 회전
        self.cross_band = None  # 교차점 밴드. ±x, 솟구침, 회전 없음(납작한 벨트)
        self.line = None        # 가로줄기 Seg_2~7. ±x

    @property
    def groups(self):
        return [g for g in (self.stem, self.sorter, self.cross, self.cross_band, self.line) if g]

    # ── world.reset() 전에 ──────────────────────────
    def build(self):
        for path in SOLID_FRAMES:
            prim = self._stage.GetPrimAtPath(path)
            if prim and prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Set("meshSimplification")

        self.stem = RollerGroup("줄기", self._find_stem(), "x", ROLLER_RADIUS)
        self.sorter = RollerGroup("교차점롤러", self._find_feeder("SorterRoller"), "x", ROLLER_RADIUS)
        self.cross = RollerGroup("교차점휠", self._find_feeder("_Wheel"), "y", WHEEL_RADIUS)
        self.cross_band = RollerGroup("교차점밴드", self._find_feeder("_Band"), "y", None)
        self.line = RollerGroup("가로줄기", self._prepare_line_rollers(), "y", ROLLER_RADIUS)

        # 빈 묶음은 조용히 사라집니다 (groups 가 걸러 냅니다). 그러면 그 구간만
        # 안 도는데 에러가 없어서, 원인을 찾는 데 한참 걸립니다. 여기서 막습니다.
        empty = [label for label, group in (
            ("줄기", self.stem), ("교차점롤러", self.sorter), ("교차점휠", self.cross),
            ("교차점밴드", self.cross_band), ("가로줄기", self.line)) if not len(group)]
        if empty:
            raise RollerError(
                f"롤러를 하나도 찾지 못한 묶음이 있습니다: {', '.join(empty)}\n"
                f"  에셋 이름이 바뀌었을 수 있습니다 ({CONV} 아래를 확인하세요).")

        print("[롤러] " + " · ".join(f"{g.label} {len(g)}개" for g in self.groups))

    # ── world.reset() 뒤에 ──────────────────────────
    def attach(self):
        for group in self.groups:
            group.attach()

    # ── 운전 ────────────────────────────────────────
    # 컨트롤러는 구역 점유만 보고 아래 네 가지를 조합해 부릅니다.
    # 어느 묶음이 무슨 일을 하는지는 여기에만 적혀 있습니다.
    def feed(self):
        """줄기와 교차점 소터롤러가 팔레트를 -y 로 받아 내립니다. 휠은 내려둡니다."""
        self.stem.set_speed(0.0, -self.speed)
        self.sorter.set_speed(0.0, -self.speed)
        self.cross.set_speed(0.0, 0.0)
        self.cross_band.set_speed(0.0, 0.0)
        self.cross.lift(0.0)
        self.cross_band.lift(0.0)

    def divert(self):
        """실제 소터처럼 휠을 솟구치게 해서 팔레트를 +x 로 넘깁니다.

        줄기는 계속 돌립니다. 컨베이어는 멈추지 않고, 넘기는 동안 소터롤러만
        멈춰서 -y 로 끌어당기지 않게 합니다.
        """
        self.stem.set_speed(0.0, -self.speed)
        self.sorter.set_speed(0.0, 0.0)
        self.cross.lift(POP_UP)
        self.cross_band.lift(POP_UP)
        self.cross.set_speed(self.speed, 0.0)
        self.cross_band.set_speed(self.speed, 0.0)

    def wait_junction(self):
        """교차점에 팔레트를 세워 둡니다. 비전룸이 비기를 기다리는 동안 쓰입니다.

        소터롤러를 멈춰 -y 로 더 밀지 않게 합니다. 줄기는 계속 돌려서
        뒤에서 오는 팔레트를 계속 받습니다.
        """
        self.stem.set_speed(0.0, -self.speed)
        self.sorter.set_speed(0.0, 0.0)
        self.cross.set_speed(0.0, 0.0)
        self.cross_band.set_speed(0.0, 0.0)
        self.cross.lift(0.0)
        self.cross_band.lift(0.0)

    def line_run(self):
        """가로줄기를 +x 로 돌립니다."""
        self.line.set_speed(self.speed, 0.0)

    def line_stop(self):
        """가로줄기를 멈춥니다. 검사 중인 팔레트가 있을 때 씁니다."""
        self.line.set_speed(0.0, 0.0)

    def rest(self):
        """모두 멈추고 휠도 내립니다."""
        for group in self.groups:
            group.set_speed(0.0, 0.0)
        self.cross.lift(0.0)
        self.cross_band.lift(0.0)

    def spin(self, dt):
        for group in self.groups:
            group.spin(dt)

    # ── 부품 찾기 ───────────────────────────────────
    def _parent(self, path):
        """부품이 들어 있는 프림. 없으면 맵 구조가 바뀐 것이라 멈춥니다."""
        prim = self._stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            raise RollerError(
                f"컨베이어 에셋을 찾지 못했습니다: {path}\n"
                f"  맵의 컨베이어 구조가 바뀌었다면 conveyor_rollers.py 의 "
                f"CONV · LINE_SEGMENTS 를 새 경로로 고쳐야 합니다.")
        return prim

    def _find_stem(self):
        parent = self._parent(f"{CONV}/TurnTable/Geometry")
        return [p for p in parent.GetChildren() if "_Roller" in p.GetName()]

    def _find_feeder(self, keyword):
        parent = self._parent(f"{CONV}/Feeder/Geometry")
        found = []
        for prim in Usd.PrimRange(parent, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if prim.GetTypeName() != "Mesh":
                continue
            name = prim.GetName()
            if keyword in name:
                found.append(prim)
        return found

    def _prepare_line_rollers(self):
        """가로줄기 롤러를 낱개로 떼어 냅니다.

        에셋은 한 세그먼트의 롤러 29개가 리지드바디 '하나' 로 묶여 있어서
        낱개로 돌릴 수 없습니다. 묶음을 풀고 롤러마다 바디를 답니다.

        강체를 분리할 때 충돌체와 스케일도 함께 수정합니다.
          · 충돌체가 부모 Rollers 에만 있습니다. 부모에서 리지드바디만 떼면
            정적 충돌체로 남아 팔레트가 그 위에 얹히고, 자식은 충돌체가 없어
            아무리 돌려도 밀지 못합니다. 부모 충돌은 끄고 자식마다 답니다.
          · 롤러 스케일의 y 가 음수(미러)라 그대로는 회전 핸들을 잡지 못합니다
            (RigidPrim 이 'Non-positive determinant' 로 거부합니다).
            원통이라 부호만 뒤집어도 모양은 그대로입니다 (bbox 변화 0.000 mm 확인).
        """
        rollers = []
        for index in LINE_SEGMENTS:
            parent = self._parent(f"{CONV}/Seg_{index}/Asset/Rollers")
            parent.RemoveAPI(UsdPhysics.RigidBodyAPI)
            UsdPhysics.CollisionAPI(parent).CreateCollisionEnabledAttr().Set(False)
            for child in parent.GetChildren():
                for op in UsdGeom.Xformable(child).GetOrderedXformOps():
                    if op.GetOpName() == "xformOp:scale":
                        scale = op.Get()
                        op.Set(Gf.Vec3f(abs(scale[0]), abs(scale[1]), abs(scale[2])))
                UsdPhysics.CollisionAPI.Apply(child)
                UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr().Set("convexHull")
                rollers.append(child)
        return rollers
