# 리프트 리그(Nova Carter + 리프트 + M0609) 안정화 값과 주행/작업 전환.
# 팔(M0609) 설정은 건드리지 않습니다. 원본 USD 파일도 저장하지 않습니다(메모리 안의 장면만 바뀜).
#
#   1) apply_rig_stability(stage, rig_root)   ... Play 전에 한 번. 후방 캐스터 로커 강성, 바퀴 잔류 목표속도 정리
#   2) RigModeSwitch(robot)                   ... Play 후. 주행(drive) / 작업(work) 전환
#        .work()   바퀴를 지금 각도에 붙잡아 둠(주차 브레이크). 팔·리프트가 움직여도 카터가 밀리지 않음
#        .drive()  브레이크 해제. /cmd_vel 주행 가능
#        .auto()   매 프레임 호출. 바퀴 속도 명령이 있으면 drive, 일정 시간 없으면 work 로 자동 전환
#
# 시험 근거: ~/smartfarm_world/rig_stability/cycle.txt (브레이크 없을 때 한 사이클에 87 mm 밀림 -> 8 mm)
import numpy as np
from pxr import UsdPhysics

ROCKER_JOINT = "joint_caster_base"
WHEEL_JOINTS = ["joint_wheel_left", "joint_wheel_right"]
DRIVE_GRAPH_OUTPUT = "differential_drive/differential_controller_01.outputs:velocityCommand"

ROCKER_STIFFNESS = 5000.0      # USD 단위(1/deg). 에셋 원래 값 100
ROCKER_DAMPING = 200.0         # 에셋 원래 값 10
BRAKE_STIFFNESS_DEG = 1.0e5    # 작업 모드에서 바퀴 드라이브 위치 강성 (N·m/deg)
AUTO_VELOCITY_EPS = 1.0e-3     # rad/s. 이보다 큰 바퀴 속도 명령이 있으면 '주행 중'
AUTO_BRAKE_DELAY = 1.0         # s.   속도 명령이 이만큼 계속 0 이면 브레이크


def apply_rig_stability(stage, rig_root):
    """Play 전에 호출. rig_root = '.../nova_carter_ROS' 프림 경로."""
    rocker = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{rig_root}/{ROCKER_JOINT}"), "angular")
    if not rocker:
        print(f"  [리그 안정화] {rig_root}/{ROCKER_JOINT} 드라이브를 찾지 못함 -> 건너뜀")
        return False
    rocker.GetStiffnessAttr().Set(ROCKER_STIFFNESS)
    rocker.GetDampingAttr().Set(ROCKER_DAMPING)
    for name in WHEEL_JOINTS:
        wheel = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(f"{rig_root}/{name}"), "angular")
        if wheel:
            wheel.GetTargetVelocityAttr().Set(0.0)      # 에셋에 남아 있던 2.14 deg/s 제거
    print(f"  [리그 안정화] 로커 강성 {ROCKER_STIFFNESS:g}/감쇠 {ROCKER_DAMPING:g}, 바퀴 잔류 목표속도 0")
    return True


class RigModeSwitch:
    """robot = 초기화된 SingleArticulation (아티큘레이션 루트 = chassis_link)."""

    def __init__(self, robot):
        self.robot = robot
        self.view = robot._articulation_view
        names = list(robot.dof_names)
        self.wheel_i = np.array([names.index(n) for n in WHEEL_JOINTS])
        kps, kds = self.view.get_gains(joint_indices=self.wheel_i)
        self.kds = np.array(kds, dtype=np.float32).reshape(1, -1)      # 감쇠(속도 제어용)는 그대로 둠
        self.brake_kp = np.full((1, len(self.wheel_i)), np.rad2deg(BRAKE_STIFFNESS_DEG), dtype=np.float32)
        self.mode = None
        self._idle = 0.0
        self.graph_attr = None
        try:      # 수업 코드에 없는 import: 주행 그래프(OmniGraph) 노드의 출력값을 읽기 위해 필요
            import omni.graph.core as og
            rig_root = str(robot.prim_path).rsplit("/", 1)[0]
            attr = og.Controller.attribute(f"{rig_root}/{DRIVE_GRAPH_OUTPUT}")
            if attr is not None and attr.is_valid():
                self.graph_attr = attr
        except Exception:
            self.graph_attr = None
        print(f"[리그 모드] 주행 명령 읽는 곳: {'주행 그래프 출력' if self.graph_attr is not None else '아티큘레이션 속도 목표'}")
        self.work()        # 시작은 작업 모드(브레이크). 주행 명령이 오면 auto() 가 바로 풉니다

    def work(self):
        """작업 모드: 바퀴를 '지금 각도'에 붙잡습니다. (0도로 잡으면 카터가 뒤로 끌려갑니다)"""
        if self.mode == "work":
            return
        now = np.array(self.view.get_joint_positions(joint_indices=self.wheel_i)).reshape(1, -1)
        self.view.set_joint_position_targets(now, joint_indices=self.wheel_i)
        self.view.set_joint_velocity_targets(np.zeros_like(now), joint_indices=self.wheel_i)
        self.view.set_gains(kps=self.brake_kp, kds=self.kds, joint_indices=self.wheel_i)
        self.mode = "work"
        print("[리그 모드] 작업 (바퀴 브레이크 ON)")

    def drive(self):
        """주행 모드: 위치 강성을 0으로. 속도 명령(/cmd_vel)만 따릅니다."""
        if self.mode == "drive":
            return
        self.view.set_gains(kps=np.zeros_like(self.brake_kp), kds=self.kds, joint_indices=self.wheel_i)
        self.mode = "drive"
        print("[리그 모드] 주행 (바퀴 브레이크 OFF)")

    def commanded_wheel_speed(self):
        """주행 명령의 크기(rad/s). Nova Carter 의 주행 그래프(/cmd_vel -> differential_controller) 출력을 읽고,
        그래프가 없으면 아티큘레이션에 걸린 속도 목표를 읽습니다."""
        if self.graph_attr is not None:
            try:
                value = self.graph_attr.get()
                if value is not None and len(value):
                    return float(np.max(np.abs(np.nan_to_num(np.array(value, dtype=float)))))
                return 0.0
            except Exception:
                self.graph_attr = None
        target = np.array(self.view.get_applied_actions().joint_velocities)[0, self.wheel_i]
        return float(np.max(np.abs(np.nan_to_num(target))))

    def auto(self, dt):
        """매 프레임 호출. 주행 명령이 있으면 drive, AUTO_BRAKE_DELAY 동안 없으면 work."""
        if self.commanded_wheel_speed() > AUTO_VELOCITY_EPS:
            self._idle = 0.0
            self.drive()
        else:
            self._idle += dt
            if self._idle >= AUTO_BRAKE_DELAY:
                self.work()

class RigModeWatcher:
    """World 없이 GUI 의 Play 버튼으로 도는 스크립트용. 메인 루프에서 매 프레임 update() 만 부르면 됩니다.
    Play 되면 리그를 잡아 자동 전환을 시작하고, Stop 되면 놓았다가 다음 Play 에 다시 잡습니다."""

    def __init__(self, rig_root):
        import omni.timeline
        self.rig_root = rig_root
        self.timeline = omni.timeline.get_timeline_interface()
        self.switch = None
        self._last_time = None
        self._failed = False

    @property
    def mode(self):
        return self.switch.mode if self.switch else None

    def update(self):
        if not self.timeline.is_playing():
            if self.timeline.is_stopped():
                self.switch, self._last_time, self._failed = None, None, False
            return
        now = self.timeline.get_current_time()
        if self.switch is None:
            if self._failed or now <= 0.0:
                return
            try:
                # 수업 코드에 없는 import: 주행 로봇(아티큘레이션 루트 = chassis_link)을 잡기 위해 필요
                from isaacsim.core.prims import SingleArticulation
                robot = SingleArticulation(f"{self.rig_root}/chassis_link")
                robot.initialize()
                self.switch = RigModeSwitch(robot)
            except Exception as error:
                self._failed = True
                print(f"[리그 모드] 시작 실패 -> 자동 전환 없이 계속합니다: {error!r}")
                return
        dt = 0.0 if self._last_time is None else max(0.0, now - self._last_time)
        self._last_time = now
        try:
            self.switch.auto(dt)
        except Exception as error:
            self.switch, self._failed = None, True
            print(f"[리그 모드] 실행 중 오류 -> 자동 전환 중단: {error!r}")
