"""[navigation 2026-09-28] 녹화용 시점 연출. 공정과 대상 위치를 보고 뷰포트 카메라를 고른다.

씬 파일을 고치지 않는다. 필요한 카메라는 실행 중에 만든다.
`SMARTFARM_VIEW_FOLLOW=1` 일 때만 동작하므로 평소 실행에는 영향이 없다.

카메라 자리는 눈대중이 아니라 2026-09-28 에 후보를 렌더해 비교하고 정했다
(tools/preview_new_cameras.py, results/log_media/camera_try_20260928_1643/).
"""

import math
import os
import time

CARTER = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/chassis_link"
FORK_LINK = "/World/SmartFarm/Placed/LiftRig/Asset/nova_carter_ROS/m0609_with_fork/link_6"
# 팔레트는 껍데기 Xform 이 아니라 그 아래 강체(Cube_011_001)가 움직인다.
# 껍데기 경로를 쓰면 팔레트가 벨트로 가도 좌표가 랙에 머물러 전환이 일어나지 않는다
# (2026-09-28 2차 녹화에서 실제로 겪음). 팀 코드의 PALLET_ASSET_NAME 과 같은 이름이다.
PALLET_ASSET = "Cube_011_001"
PALLET_ROOT = "/World/SmartFarm/Placed/Pallet_01"
PALLET = f"{PALLET_ROOT}/{PALLET_ASSET}/{PALLET_ASSET}"
PROCESS_CAMS = "/World/ProcessCameras"

# 씬에 이미 있는 카메라 (2026-09-28 렌더로 화각 확인함)
CAM_HARVEST = PROCESS_CAMS + "/Cam1_Harvest"          # 랙 + 카터 + 포크팔 풀샷
CAM_NAV_PLACE = PROCESS_CAMS + "/Cam2_Nav2Place"      # 주행 경로 + 턴테이블 롱샷
CAM_CULL = PROCESS_CAMS + "/Cam4_CullPickPlace"       # 비전룸 풀샷
CAM_PUSHER = PROCESS_CAMS + "/Cam5_Pusher"            # 컨베이어 베드 정면

# 실행 중에 만드는 카메라. (경로, 눈 위치, 바라볼 점, 초점거리)
NEW_CAMS = (
    # 턴테이블 벨트 ~ 피더 방향전환 ~ 비전룸 입구를 한 화면에. 3번이 비어 있어 그 번호를 쓴다.
    (PROCESS_CAMS + "/Cam3_FeederEntry", (-2.20, -2.30, 2.40), (-2.20, -7.00, 0.60), 14.0),
    # 비전룸 출구 ~ 반출 컨베이어 ~ 맵 밖까지.
    (PROCESS_CAMS + "/Cam6_Outfeed", (2.50, -3.80, 3.40), (7.00, -6.70, 0.60), 14.0),
    # 피더에서 본선으로 합류하는 지점과 비전룸 입구를 가까이에서.
    (PROCESS_CAMS + "/Cam7_FeederClose", (-4.20, -6.70, 1.90), (-1.80, -6.70, 0.70), 16.0),
    # 콘티(최종영상_스토리보드_260927.md §1)의 롱샷. 랙·통로·턴테이블·비전룸이 한 화면에.
    (PROCESS_CAMS + "/Cam_Long_Master", (7.50, -2.00, 6.50), (-1.00, -2.50, 0.60), 14.0),
)

# 움직이는 prim 에 붙이는 시점뷰. (부모 prim, 카메라 이름, 로컬 위치, 로컬 회전 XYZ, 초점거리)
# 이름은 콘티(최종영상_스토리보드_260927.md §1)의 prim 이름을 따른다.
WRIST_LINK = "/World/SmartFarm/Placed/M0609/Asset/link_6"
POV_CAMS = (
    # 오프셋은 2026-09-28 에 후보 8개를 렌더해 비교하고 정했다
    # (tools/preview_new_cameras.py --povcam, results/log_media/camera_try_pov_20260928_2252/).
    # 너무 가까우면 구조물에 파묻혀 흰 화면이 된다(Wrist 회전 180도 후보가 그랬다).
    (FORK_LINK, "Cam_Fork", (0.0, 0.0, 0.40), (90.0, 0.0, 180.0), 14.0),        # 행위자뷰 포크 끝
    (WRIST_LINK, "Cam_Wrist", (0.0, 0.0, 0.25), (90.0, 0.0, 0.0), 14.0),        # 행위자뷰 그리퍼 손목
    (CARTER, "Cam_Carter_Rear", (-0.75, 0.0, 0.85), (78.0, 0.0, 90.0), 14.0),   # 행위자뷰 카터 뒤(-x)
    (PALLET, "Cam_Pallet", (0.0, 0.0, 0.55), (60.0, 0.0, 0.0), 14.0),           # 대상뷰 팔레트(강체)
)

# 콘티 §1 의 풀샷·롱샷은 기존 카메라를 그대로 쓴다(중복 생성하지 않는다).
#   Cam_Full_Rack   = Cam1_Harvest
#   Cam_Full_Dock   = Cam2_Nav2Place
#   Cam_Full_Vision = Cam4_CullPickPlace
#   Cam_Long_Master = 위에서 새로 만든다

# 행위자뷰·대상뷰만으로 한 사이클을 담는 모드(SMARTFARM_VIEW_MODE=pov)
POV_BY_OPERATION = {
    "TRANSFER": FORK_LINK + "/Cam_Fork",
    "PICK_HARVEST": FORK_LINK + "/Cam_Fork",
    "PLACE_INSPECT": FORK_LINK + "/Cam_Fork",
    "CONVEY_TO_INSPECT": PALLET + "/Cam_Pallet",
    "PREPARE_INSPECT": WRIST_LINK + "/Cam_Wrist",
    "MOVE_TO_INSPECT": WRIST_LINK + "/Cam_Wrist",
    "CULL": WRIST_LINK + "/Cam_Wrist",
    "RELEASE_INSPECT": PALLET + "/Cam_Pallet",
    "CONVEYOR_OUT": PALLET + "/Cam_Pallet",
}

# 공정 단계별 기본 카메라
BY_OPERATION = {
    "TRANSFER": CAM_HARVEST,
    "PICK_HARVEST": CAM_HARVEST,
    "PLACE_INSPECT": CAM_NAV_PLACE,
    "CONVEY_TO_INSPECT": PROCESS_CAMS + "/Cam3_FeederEntry",
    "PREPARE_INSPECT": CAM_CULL,
    "MOVE_TO_INSPECT": CAM_CULL,
    "CULL": CAM_CULL,
    "RELEASE_INSPECT": CAM_PUSHER,
    "CONVEYOR_OUT": CAM_PUSHER,
}

# 랙 통로 남단. 카터가 이 선을 넘으면 주행 롱샷으로 바꾼다(stations.yaml 의 CORRIDOR_EXIT 기준).
CORRIDOR_EXIT_Y = -1.20
# 팔레트가 이 x 를 넘으면 반출 카메라로 바꾼다(비전룸 동쪽 끝 x 0.24 를 지난 뒤).
OUTFEED_X = 0.60
# 팔레트가 이 y 아래로 내려오면 피더를 지나 본선(Seg, y -7.33~-6.18)에 합류한 것으로 본다.
MAINLINE_JOIN_Y = -6.20
# 합류를 본 뒤 이만큼 있다가 가까운 시점으로 바꾼다(사용자 지시 2026-09-28).
# 영상 편집 기준이므로 시뮬 시각이 아니라 벽시계로 잰다.
FEEDER_CLOSE_DELAY_SEC = 0.5


_TITLES_LOGGED = False


def show_graph_window():
    """[navigation 2026-09-28] 그래프 편집기를 **뷰포트 아래 UI 칸에 도킹**한다.

    Isaac Sim 으로 작업했다는 것이 화면에 드러나도록 Content/Console 이 있는
    아래 칸에 탭으로 붙인다. 떠 있는 팝업으로 두면 뷰포트를 가려 못 쓴다
    (2026-09-28 사용자 지적).

    SMARTFARM_SHOW_GRAPH=1 일 때만 시도한다. 도킹이 확인될 때까지 True 를
    돌려주지 않으므로, 실패하면 호출자가 계속 다시 부른다.
    """

    if os.environ.get("SMARTFARM_SHOW_GRAPH", "").strip() not in ("1", "true", "True"):
        return True

    try:
        import omni.kit.app
        import omni.ui as ui
    except Exception:
        return True

    manager = omni.kit.app.get_app().get_extension_manager()
    for extension in ("omni.graph.window.action", "omni.graph.window.generic",
                      "omni.graph.window.core", "omni.kit.widget.graph"):
        try:
            manager.set_extension_enabled_immediate(extension, True)
        except Exception:
            pass

    titles = []
    try:
        titles = [w.title for w in ui.Workspace.get_windows()]
    except Exception:
        pass

    # 아래 칸의 기준 창. Content 가 그 자리에 있다.
    host = ui.Workspace.get_window("Content") or ui.Workspace.get_window("Console")
    if host is None:
        return False

    wanted = [t for t in titles if "Graph" in t or "Scripting" in t]
    for title in wanted + ["Action Graph", "Visual Scripting", "Generic Graph"]:
        window = ui.Workspace.get_window(title)
        if window is None:
            continue
        window.visible = True
        try:
            window.dock_in(host, ui.DockPosition.SAME, 0.5)
        except Exception as error:
            print(f"[화면] '{title}' 도킹 실패: {error}", flush=True)
            continue

        # 도킹이 실제로 됐는지 확인한다. 안 됐으면 성공이라고 말하지 않는다.
        if getattr(window, "docked", False):
            try:
                window.focus()
            except Exception:
                pass
            print(f"[화면] 아래 칸에 '{title}' 를 도킹했습니다.", flush=True)
            return True
        return False

    global _TITLES_LOGGED
    if not _TITLES_LOGGED and titles:
        _TITLES_LOGGED = True
        print("[화면] 사용 가능한 창 목록: " + ", ".join(sorted(titles)), flush=True)
    return False


def report_viewport_rect() -> None:
    """[navigation 2026-09-28] 뷰포트 창의 화면 좌표를 한 번 찍는다.

    녹화본에서 뷰포트만 잘라낼 때 이 값을 쓴다. 눈대중으로 자르지 않기 위함이다.
    """

    try:
        import omni.ui as ui

        window = ui.Workspace.get_window("Viewport")
        if window is None:
            return
        print(f"[화면] 뷰포트 사각형 x {int(window.position_x)} y {int(window.position_y)} "
              f"w {int(window.width)} h {int(window.height)}", flush=True)
    except Exception as error:
        print(f"[화면] 뷰포트 사각형 조회 실패 (무시): {error}", flush=True)


def enabled() -> bool:
    """녹화용 시점 연출을 켤지 여부."""

    return os.environ.get("SMARTFARM_VIEW_FOLLOW", "").strip() in ("1", "true", "True")


_MISSING_LOGGED = set()


def _world_xyz(stage, path):
    """prim 의 world 위치. 없으면 None. 없는 경로는 한 번만 알린다."""

    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        if path not in _MISSING_LOGGED:
            _MISSING_LOGGED.add(path)
            print(f"[화면] prim 을 찾지 못해 위치 판단을 건너뜁니다: {path}", flush=True)
        return None
    matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = matrix.ExtractTranslation()
    return (t[0], t[1], t[2])


def _look_at_rotation(eye, target):
    """eye 에서 target 을 보게 하는 (pitch, yaw). USD 카메라는 -z 를 본다."""

    dx, dy, dz = (target[i] - eye[i] for i in range(3))
    yaw = math.degrees(math.atan2(dy, dx)) - 90.0
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    return 90.0 + pitch, yaw


def ensure_cameras(stage) -> None:
    """녹화에 필요한 카메라를 만든다. 이미 있으면 그대로 둔다."""

    from pxr import Gf, UsdGeom

    for path, eye, target, focal in NEW_CAMS:
        if stage.GetPrimAtPath(path).IsValid():
            continue
        camera = UsdGeom.Camera.Define(stage, path)
        camera.GetFocalLengthAttr().Set(focal)
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*eye))
        pitch, yaw = _look_at_rotation(eye, target)
        xform.AddRotateXYZOp().Set(Gf.Vec3f(pitch, 0.0, yaw))
        print(f"[화면] 카메라 생성 {path}", flush=True)

    for parent, name, offset, rotation, focal in POV_CAMS:
        if not stage.GetPrimAtPath(parent).IsValid():
            print(f"[화면] 부모 prim 이 없어 {name} 을 건너뜀: {parent}", flush=True)
            continue
        path = f"{parent}/{name}"
        if stage.GetPrimAtPath(path).IsValid():
            continue
        camera = UsdGeom.Camera.Define(stage, path)
        camera.GetFocalLengthAttr().Set(focal)
        xform = UsdGeom.Xformable(camera.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*offset))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
        print(f"[화면] 시점뷰 카메라 생성 {path}", flush=True)


class ViewDirector:
    """지금 카메라를 기억해 두고 바뀔 때만 뷰포트를 옮긴다."""

    def __init__(self, set_camera, every: int = 6) -> None:
        """set_camera 는 카메라 경로를 받아 뷰포트를 바꾸는 함수다.

        every 를 작게 둔 이유는 0.5 초짜리 전환 신호를 놓치지 않기 위함이다.
        위치 조회만 하므로 부담이 없다.
        """

        self.set_camera = set_camera
        self.every = max(1, every)
        self.current = ""
        self.ready = False
        self.ticks = 0
        self.joined_at = None   # 팔레트가 본선에 합류한 벽시계 시각
        self.last_operation = ""
        self.graph_tries = 0
        self.rect_reported = False

    def _mode(self) -> str:
        """auto(기본) · pov(행위자뷰·대상뷰) · fixed:<카메라 경로>"""

        return os.environ.get("SMARTFARM_VIEW_MODE", "auto").strip() or "auto"

    def decide(self, stage, operation: str) -> str:
        """지금 보여 줄 카메라를 고른다.

        공정 이름만으로는 부족한 세 구간을 대상 위치로 보완한다.
          - NAVIGATION 은 Isaac 이 명령을 받지 않는다. 카터가 통로를 벗어나면 주행 롱샷.
          - CONVEY_TO_INSPECT 는 팔레트가 본선에 합류하고 0.5 초 뒤 가까운 시점으로.
          - CONVEYOR_OUT 은 팔레트가 비전룸을 나간 뒤 반출 카메라로.
        """

        mode = self._mode()
        if mode.startswith("fixed:"):
            return mode.split(":", 1)[1].strip()

        if mode == "pov":
            chosen = POV_BY_OPERATION.get(operation)
            if chosen:
                self.last_operation = operation
                return chosen
            # NAVIGATION 등 명령이 없는 구간은 카터 뒤 행위자뷰로 본다.
            if self.last_operation == "PICK_HARVEST":
                return CARTER + "/Cam_Carter_Rear"
            return self.current or FORK_LINK + "/Cam_Fork"

        if operation == "CONVEY_TO_INSPECT":
            pallet = _world_xyz(stage, PALLET)
            if pallet is not None and pallet[1] <= MAINLINE_JOIN_Y:
                if self.joined_at is None:
                    self.joined_at = time.monotonic()
                if time.monotonic() - self.joined_at >= FEEDER_CLOSE_DELAY_SEC:
                    return PROCESS_CAMS + "/Cam7_FeederClose"
            return PROCESS_CAMS + "/Cam3_FeederEntry"

        if operation == "CONVEYOR_OUT":
            pallet = _world_xyz(stage, PALLET)
            if pallet is not None and pallet[0] > OUTFEED_X:
                return PROCESS_CAMS + "/Cam6_Outfeed"
            return CAM_PUSHER

        chosen = BY_OPERATION.get(operation)
        if chosen:
            self.last_operation = operation
            return chosen

        # 여기부터는 Isaac 에 활성 명령이 없는 구간이다.
        # 주행(NAVIGATION) 때만 카터 위치로 판단하고, 그 밖에는 지금 화면을 유지한다.
        # 유지하지 않으면 INSPECT·RECHECK 처럼 명령이 잠깐 비는 사이에 화면이 왔다 갔다 한다
        # (2026-09-28 2차 녹화에서 실제로 겪음).
        if self.last_operation == "PICK_HARVEST":
            carter = _world_xyz(stage, CARTER)
            if carter is not None and carter[1] <= CORRIDOR_EXIT_Y:
                return CAM_NAV_PLACE
            return CAM_HARVEST
        return self.current or CAM_HARVEST

    def update(self, stage, operation: str) -> None:
        """매 프레임 불러도 되게 싸게 만들었다. every 프레임마다 한 번만 판단한다."""

        if not enabled() or stage is None:
            return
        if not self.ready:
            ensure_cameras(stage)
            self.ready = True

        # 그래프 창은 확장 기능이 올라온 뒤에야 잡히므로 몇 번 더 시도한다.
        if self.graph_tries < 200:
            self.graph_tries += 1
            if show_graph_window():
                self.graph_tries = 999
            elif self.graph_tries == 200:
                # 끝내 도킹이 안 되면 떠 있는 창을 닫는다. 뷰포트를 가리는 것보다 낫다.
                try:
                    import omni.ui as ui

                    for w in ui.Workspace.get_windows():
                        if ("Graph" in w.title or "Scripting" in w.title) and not getattr(w, "docked", False):
                            w.visible = False
                            print(f"[화면] 도킹 실패로 '{w.title}' 를 닫았습니다(뷰포트 가림 방지).", flush=True)
                except Exception:
                    pass
        if not self.rect_reported and self.ticks > 60:
            self.rect_reported = True
            report_viewport_rect()

        self.ticks += 1
        if self.ticks % self.every:
            return

        chosen = self.decide(stage, operation or "")
        if chosen and chosen != self.current:
            self.current = chosen
            print(f"[화면] 공정 {operation or '(명령없음)'} -> {chosen}", flush=True)
            self.set_camera(chosen)
