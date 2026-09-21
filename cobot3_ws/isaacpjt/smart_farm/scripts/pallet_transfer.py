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
        """팔레트 높이로 리프트 정렬을 시작합니다. 이후 진행은 update()가 합니다."""
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
            self._lift.start_move(pick_shelf_top - self._base_below_shelf)
            self._state = TransferState.LIFT_ALIGN
        except RuntimeError as error:  # LiftError도 RuntimeError입니다.
            self._fail(error)

    def update(self, dt):
        """현재 상태에 해당하는 장치를 물리 한 스텝만큼 진행합니다."""
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
        """팔과 리프트를 현재 위치에 멈추고 IDLE로 돌아갑니다."""
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
        return self._state in (TransferState.SUCCEEDED, TransferState.FAILED)

    @property
    def error(self):
        return self._error

    def _fail(self, error):
        self._error = error
        self._state = TransferState.FAILED
        stop_error = self._safe_stop()
        if self._error is None:
            self._error = stop_error
        print(f"[중단] {self._error}")
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
