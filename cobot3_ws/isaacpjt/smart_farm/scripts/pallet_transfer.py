"""리프트 정렬과 팔레트 Pick/Place 순서를 조정하는 비블로킹 상태 머신."""

from enum import Enum

from lift import check_base_level


class TransferState(str, Enum):
    IDLE = "IDLE"
    LIFT_ALIGN = "LIFT_ALIGN"
    WAIT_BASE_SETTLE = "WAIT_BASE_SETTLE"
    PICKING = "PICKING"
    PLACING = "PLACING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


_RUNNING_STATES = {
    TransferState.LIFT_ALIGN,
    TransferState.WAIT_BASE_SETTLE,
    TransferState.PICKING,
    TransferState.PLACING,
}


class PalletTransferController:
    """리프트와 로봇팔이 동시에 움직이지 않도록 한 작업을 순서대로 실행합니다."""

    def __init__(
        self,
        lift,
        motion,
        arm_base,
        base_watcher,
        base_below_shelf,
        fork_clear_check,
    ):
        self._lift = lift
        self._motion = motion
        self._arm_base = arm_base
        self._base_watcher = base_watcher
        self._base_below_shelf = base_below_shelf
        self._fork_clear_check = fork_clear_check

        self._state = TransferState.IDLE
        self._error = None
        self._task = None
        self._pallet = None
        self._completed_tasks = 0

    def start(self, task, pallet):
        """팔레트 높이로 리프트 정렬을 시작합니다. 이후 진행은 update()가 합니다.

        RuntimeError는 밖으로 전달하지 않고 FAILED 상태와 error에 기록합니다.
        호출 뒤 state/error로 시작 실패 여부를 확인하세요.
        """
        try:
            if self.is_running:
                raise RuntimeError(f"팔레트 이송이 이미 실행 중입니다: {self._state.value}")
            if self._state == TransferState.FAILED:
                raise RuntimeError("실패 상태입니다. cancel() 후 다시 시작하세요.")

            self._task = task
            self._pallet = pallet
            self._error = None
            self._fork_clear_check()

            pick_shelf_top = float(pallet.get_world_pose()[0][2])
            # 행정 밖이면 닿는 데까지 맞춥니다. 모자란 만큼은 팔이 뻗어서
            # 흡수합니다. 진짜로 못 닿는지는 IK 계획 단계가 판단합니다.
            self._lift.start_move(
                self._lift.clamp_height(pick_shelf_top - self._base_below_shelf)
            )
            self._state = TransferState.LIFT_ALIGN
        except RuntimeError as error:  # LiftError도 RuntimeError입니다.
            self._fail(error)

    def update(self, dt):
        """현재 상태를 한 스텝 진행합니다. RuntimeError는 FAILED/error에 기록합니다."""
        if not self.is_running:
            return

        try:
            if self._state == TransferState.LIFT_ALIGN:
                self._lift.update(dt)
                if self._lift.is_done:
                    self._base_watcher.reset()
                    self._state = TransferState.WAIT_BASE_SETTLE
                return

            if self._state == TransferState.WAIT_BASE_SETTLE:
                self._lift.hold()
                self._base_watcher.update(self._arm_base, dt)
                if self._base_watcher.settled:
                    check_base_level(self._arm_base.get_world_pose()[1])
                    self._motion.start_pick(
                        self._pallet,
                        start_from_home=(self._completed_tasks == 0),
                    )
                    self._state = TransferState.PICKING
                return

            if self._state == TransferState.PICKING:
                self._lift.hold()
                self._motion.update(dt)
                if self._motion.is_done:
                    if self._task.pick_only:
                        # 집기만 하는 작업. 팔레트를 든 채로 끝냅니다.
                        # cancel() 은 부르지 않습니다. 그걸 부르면 팔레트 추적이
                        # 지워져서, 실제로는 들고 있는데 '운반 중' 표시가 풀립니다.
                        # 그 상태로 다음 Pick 을 시작하면 막아주지 못합니다.
                        # 동작이 끝난 뒤에는 구동부가 마지막 명령 자세를 유지합니다.
                        self._completed_tasks += 1
                        self._state = TransferState.SUCCEEDED
                        return
                    self._motion.start_place(self._task.destination_shelf_top)
                    self._state = TransferState.PLACING
                return

            if self._state == TransferState.PLACING:
                self._lift.hold()
                self._motion.update(dt)
                if self._motion.is_done:
                    self._motion.cancel()
                    self._completed_tasks += 1
                    self._state = TransferState.SUCCEEDED
        except RuntimeError as error:  # LiftError도 RuntimeError입니다.
            self._fail(error)

    def cancel(self):
        """팔과 리프트를 멈추고 IDLE로 돌아갑니다. 정지 실패 시 FAILED/error를 유지합니다."""
        stop_error = self._safe_stop()
        self._base_watcher.reset()
        self._task = None
        self._pallet = None
        self._completed_tasks = 0
        self._error = stop_error
        self._state = (
            TransferState.FAILED if stop_error is not None else TransferState.IDLE
        )

    @property
    def state(self):
        return self._state

    @property
    def is_running(self):
        return self._state in _RUNNING_STATES

    @property
    def is_done(self):
        """성공·실패를 포함한 종료 여부. 성공은 state == TransferState.SUCCEEDED로 확인합니다."""
        return self._state in (TransferState.SUCCEEDED, TransferState.FAILED)

    @property
    def error(self):
        return self._error

    def _fail(self, error):
        self._error = error
        self._state = TransferState.FAILED
        stop_error = self._safe_stop()
        print(f"[중단] {self._error}")
        if stop_error is not None:
            print(f"[정지 오류] {stop_error}")
        print("원인을 확인하세요. Stop → Play로 처음부터 재시험합니다.")

    def _safe_stop(self):
        first_error = None
        try:
            self._motion.cancel()
        except RuntimeError as error:
            first_error = error

        try:
            self._lift.stop()
        except RuntimeError as error:
            if first_error is None:
                first_error = error
        return first_error
