"""사람 돌발상황: 카터가 랙 통로를 후진으로 빠져나올 때 작업자 1명이 카터 경로(차선 중앙선) 위로 걸어 들어와 잠시 섰다가 비켜난다.

Nav2 쪽은 collision_monitor 의 HumanStop 정지 영역(scripts/wsl/make_human_params.py)이 라이다로 사람을 보고 멈추고,
사람이 비키면 그대로 다시 움직인다. 이 파일은 Isaac 쪽(사람 배치·걷기·기록)만 맡는다.

쓰는 법 (standalone_app.py --human-crossing):
  1) scene = prepare_scene(scene_path, work_dir, update)   # 장면을 열기 "전에" 사람을 넣은 감싼 장면(.usda)을 만든다
  2) open_stage(scene) ... world.reset()
  3) human = HumanCrossing(robot_position, out_dir)          # robot_position() -> (x, y, z) 월드 좌표
  4) 매 물리 스텝 human.update(dt)

사람을 장면을 연 "뒤에" 추가하면 Fabric 에 animationGraph 속성이 없어 걷기 애니메이션이 붙지 않는다(시험 14 에서 확인).
그래서 원래 장면을 sublayer 로 두고 사람만 더한 작은 감싼 장면을 따로 만들어 연다. 원래 장면 파일은 바뀌지 않는다.
"""
import json
import math
import time
from pathlib import Path

PEOPLE = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/People/Characters"
CHARACTER = "male_adult_construction_05_new"          # 작업복 차림 남성
ROOT = "/World/Characters"
WORKER = f"{ROOT}/Worker"
ANIM_EXTENSIONS = ("omni.anim.timeline", "omni.anim.graph.bundle", "omni.anim.graph.core",
                   "omni.anim.retarget.bundle", "omni.anim.retarget.core", "omni.anim.navigation.bundle")

# 카터 경로: 바닥 노란 차선 중앙선(-Lane). RACK_DOCK(-0.42, 1.01) 에서 x -0.42 을 따라 남쪽으로 후진 -> 모서리 (-0.42, -1.54)
# -> 제자리 회전 -> y -1.54 을 따라 서쪽으로 후진 -> FEEDER_APPROACH. 작업자는 남쪽 구간 모서리 바로 앞 중앙선에 선다.
# (-Lane 없이 Nav2 기본 경로로 가면 카터는 남서 대각선으로 가므로 이 점은 경로 밖이다. 그때는 (-1.75, -1.05) 를 쓴다)
CROSS_POINT = (-0.67, -1.45)   # 걷기는 목표 0.25 m 앞에서 멈춘다 -> 실제로 서는 곳은 남북 차선 중앙선(x -0.42) 근처
SPAWN = (0.25, -1.45)          # 대기 위치: 차선 동쪽 노란 테두리(x 0.07) 바로 밖. 카터가 출발하면 한 걸음(0.7 m)에 경로로 들어온다
SPAWN_YAW_DEG = 180.0          # 차선 쪽(서쪽)을 본다
EXIT = (1.30, -1.20)           # 비키는 곳: 모서리에서 1.75 m. 모서리 제자리 회전 판정 원(반경 1.25 m) 밖
TRIGGER_MOVED = 0.03           # 카터가 시작 위치에서 이만큼 움직이면(주행 시작) 출발. 수확 중 흔들림은 무시
STAND_S = 4.0                  # 경로 위에 서 있는 시간
WALK_SPEED = 1.0               # animation graph 'Walk' (1.0 = 약 1.1 m/s)
VIEW_CAMERA = f"{ROOT}/HumanViewCam"     # 사람 모드에서 GUI 뷰포트를 이 카메라로 바꾼다
VIEW_EYE = (1.90, -3.30, 2.90)          # 차선 남동쪽 위: 남북 차선·모서리·서쪽 차선·작업자가 한 화면에
VIEW_TARGET = (-0.90, -0.70, 0.30)


def prepare_scene(scene_path, work_dir, update=None):
    """scene_path 를 sublayer 로 두고 작업자를 더한 감싼 장면을 work_dir 에 만들고 그 경로를 돌려준다."""
    from isaacsim.core.utils.extensions import enable_extension

    for ext in ANIM_EXTENSIONS:
        enable_extension(ext)
        if update:
            update()
    for _ in range(20 if update else 0):     # 확장은 비동기로 켜진다: AnimationGraph 플러그인이 먼저 떠 있어야 한다
        update()
    from pxr import Gf, Sdf, Usd, UsdGeom
    from AnimGraphSchema import AnimationGraphAPI

    scene_path = Path(scene_path)
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    src = Sdf.Layer.FindOrOpen(str(scene_path))

    # 사람 부분만 메모리 장면에서 조립해 SkelRoot 를 찾고 AnimationGraphAPI 를 붙인다
    tmp = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(tmp, "/World")
    UsdGeom.Xform.Define(tmp, ROOT)
    bip = tmp.DefinePrim(f"{ROOT}/Biped_Setup", "Xform")
    bip.GetReferences().AddReference(f"{PEOPLE}/Biped_Setup.usd")
    UsdGeom.Imageable(bip).MakeInvisible()
    worker = tmp.DefinePrim(WORKER, "Xform")
    worker.GetReferences().AddReference(f"{PEOPLE}/{CHARACTER}/{CHARACTER}.usd")
    UsdGeom.XformCommonAPI(worker).SetTranslate(Gf.Vec3d(SPAWN[0], SPAWN[1], 0.0))
    rot = worker.GetAttribute("xformOp:rotateXYZ")            # 에셋이 double3 로 이미 갖고 있다
    if rot and rot.Get() is not None:
        rot.Set(type(rot.Get())(0.0, 0.0, SPAWN_YAW_DEG))
    else:
        UsdGeom.XformCommonAPI(worker).SetRotate(Gf.Vec3f(0.0, 0.0, SPAWN_YAW_DEG))
    graph = next((p for p in Usd.PrimRange(bip) if p.GetTypeName() == "AnimationGraph"), None)
    skel = next((p for p in Usd.PrimRange(worker) if p.GetTypeName() == "SkelRoot"), None)
    if graph is None or skel is None:
        raise RuntimeError(f"사람 에셋을 읽지 못했습니다 (graph={graph}, skelroot={skel}). 인터넷/에셋 경로 확인: {PEOPLE}")
    AnimationGraphAPI.Apply(skel).CreateAnimationGraphRel().SetTargets([graph.GetPath()])

    # 녹화용 카메라: 랙 통로 출구 ~ 사람이 서는 지점 ~ FEEDER 앞까지 비스듬히 내려다본다
    cam = UsdGeom.Camera.Define(tmp, VIEW_CAMERA)
    cam.CreateFocalLengthAttr(14.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*VIEW_EYE), Gf.Vec3d(*VIEW_TARGET), Gf.Vec3d(0, 0, 1))
    UsdGeom.Xformable(cam).AddTransformOp().Set(view.GetInverse())

    out = work_dir / f"{scene_path.stem}_human.usda"
    layer = Sdf.Layer.CreateAnonymous(".usda")
    layer.subLayerPaths.append(scene_path.resolve().as_posix())
    for key in ("upAxis", "metersPerUnit", "defaultPrim", "customLayerData"):
        if src.pseudoRoot.HasInfo(key):
            layer.pseudoRoot.SetInfo(key, src.pseudoRoot.GetInfo(key))
    Sdf.CreatePrimInLayer(layer, "/World")                     # over /World (원래 장면의 /World 에 얹힌다)
    Sdf.CopySpec(tmp.GetRootLayer(), ROOT, layer, ROOT)
    layer.Export(str(out))
    return out, str(skel.GetPath())


class HumanCrossing:
    """카터 위치를 보고 작업자를 경로 앞으로 걸어 들여보냈다가 비키게 한다. 사건은 out_dir/human_events.json."""

    def __init__(self, skel_path, robot_position, out_dir=None, clock=None):
        self.skel_path = skel_path
        self.robot_position = robot_position
        self.out = Path(out_dir) / "human_events.json" if out_dir else None
        self.t = 0.0
        self.state = "WAIT"
        self.character = None
        self.start = None
        self.history = []            # (t, x, y) 카터 위치
        self.target = None
        self.state_t = 0.0
        self.events = []
        self.robot_track = []
        self._next_sample = 0.0

    # ---- 기록 -------------------------------------------------------------------------------------------------
    def _event(self, name, **kw):
        e = {"t": round(self.t, 2), "event": name, **kw}
        self.events.append(e)
        print(f"[사람] {name} " + " ".join(f"{k}={v}" for k, v in kw.items()), flush=True)
        self._save()

    def _save(self):
        if self.out:
            self.out.parent.mkdir(parents=True, exist_ok=True)
            self.out.write_text(json.dumps({"events": self.events, "robot_track": self.robot_track},
                                           indent=1, ensure_ascii=False), encoding="utf-8")

    # ---- 캐릭터 ------------------------------------------------------------------------------------------------
    def _char(self):
        if not getattr(self, "_view_set", False):
            self._view_set = True
            try:
                from omni.kit.viewport.utility import get_active_viewport

                vp = get_active_viewport()
                self._prev_camera = str(vp.camera_path)
                vp.camera_path = VIEW_CAMERA
            except Exception as e:  # headless 등 뷰포트가 없으면 그냥 넘어간다
                print(f"[사람] 뷰포트 카메라 전환 생략: {e}", flush=True)
        if self.character is None:
            import omni.anim.graph.core as ag

            self.character = ag.get_character(self.skel_path)   # Play 중에만 잡힌다
        return self.character

    def _char_pos(self):
        import carb

        t, q = carb.Float3(0, 0, 0), carb.Float4(0, 0, 0, 1)
        self.character.get_world_transform(t, q)
        return (t[0], t[1])

    def _walk_to(self, xy):
        import carb

        c = self.character
        x, y = self._char_pos()
        c.set_variable("Action", "Walk")
        c.set_variable("PathPoints", [carb.Float3(x, y, 0.0), carb.Float3(xy[0], xy[1], 0.0)])
        c.set_variable("Walk", WALK_SPEED)

    def _halt(self):
        self.character.set_variable("Action", "None")
        self.character.set_variable("Walk", 0.0)

    # ---- 매 스텝 -----------------------------------------------------------------------------------------------
    def update(self, dt):
        self.t += dt
        if self.t < self._next_sample:
            return
        self._next_sample = self.t + 0.1
        if self._char() is None:
            return
        rx, ry, _ = (float(v) for v in self.robot_position())
        self.history = [h for h in self.history if self.t - h[0] < 1.0] + [(self.t, rx, ry)]
        if self.start is None:
            self.start = (rx, ry)
        hx, hy = self._char_pos()
        gap = math.hypot(rx - hx, ry - hy)
        v = 0.0
        if len(self.history) > 3:
            t0, x0, y0 = self.history[0]
            v = math.hypot(rx - x0, ry - y0) / max(self.t - t0, 1e-3)
        if int(self.t * 10) % 5 == 0:
            self.robot_track.append([round(self.t, 1), round(rx, 3), round(ry, 3), round(v, 3), self.state, round(gap, 2)])
            if self.state != "WAIT" and int(self.t * 10) % 20 == 0:
                self._save()

        if self.state == "WAIT":
            moved = math.hypot(rx - self.start[0], ry - self.start[1])
            if moved > TRIGGER_MOVED and v > 0.05:
                self.target = CROSS_POINT
                self._walk_to(self.target)
                self.state, self.state_t = "WALK_IN", self.t
                self._event("WALK_IN", robot=[round(rx, 2), round(ry, 2)], robot_speed=round(v, 2),
                            target=[round(self.target[0], 2), round(self.target[1], 2)])
        elif self.state == "WALK_IN":
            if math.hypot(hx - self.target[0], hy - self.target[1]) < 0.25 or self.t - self.state_t > 6.0:
                self._halt()
                self.state, self.state_t = "STAND", self.t
                self._event("STAND_IN_PATH", person=[round(hx, 2), round(hy, 2)], robot_gap=round(gap, 2), robot_speed=round(v, 2))
        elif self.state == "STAND":
            if self.t - self.state_t > STAND_S:
                self._walk_to(EXIT)
                self.state, self.state_t = "WALK_OUT", self.t
                self._event("WALK_OUT", robot_gap=round(gap, 2), robot_speed=round(v, 2))
        elif self.state == "WALK_OUT":
            if math.hypot(hx - EXIT[0], hy - EXIT[1]) < 0.25 or self.t - self.state_t > 8.0:
                self._halt()
                self.state = "DONE"
                self._event("CLEAR", robot_gap=round(gap, 2), robot_speed=round(v, 2))
        elif self.state == "DONE" and not getattr(self, "_resumed", False) and v > 0.05:
            self._resumed = True
            self._resumed_t = self.t
            self._event("ROBOT_RESUMED", robot=[round(rx, 2), round(ry, 2)], robot_speed=round(v, 2))
        elif self.state == "DONE" and getattr(self, "_resumed", False) and self.t - self._resumed_t > 40.0:
            self.state = "FINISHED"            # 도킹까지 보여 준 뒤 원래 뷰포트 카메라(비전룸 등)로 돌려놓는다
            try:
                from omni.kit.viewport.utility import get_active_viewport

                if getattr(self, "_prev_camera", None):
                    get_active_viewport().camera_path = self._prev_camera
            except Exception:
                pass
            self._save()
