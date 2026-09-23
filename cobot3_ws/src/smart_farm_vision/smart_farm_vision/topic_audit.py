r"""Audit live ROS 2 topics against the inspection interface contract.

Run inside the vision container so the audit sees the same DDS view as
the inspection executor:

    docker compose -f compose.vision.yaml exec vision \
      /entrypoint.sh ros2 run smart_farm_vision topic_audit

Exit code 0 means every contract topic is present with the expected
type and the configured camera topic exists. Multiple camera
candidates are reported as a warning, not a failure.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node


# docs/02-interfaces.md 3절 기준. 이름이 확정된 Topic만 대조한다.
# 카메라 Topic은 같은 절에서 "최종 장면과 launch 설정에서 확정한다"고
# 명시하므로 고정 이름으로 대조하지 않고 후보를 찾아 보고한다.
CONTRACT_TOPICS = {
    '/inspection/command': 'smart_farm_interfaces/msg/TaskCommand',
    '/inspection/result': 'smart_farm_interfaces/msg/TaskResult',
    '/inspection/status': 'smart_farm_interfaces/msg/ExecutorStatus',
    '/inspection/detections_2d': 'std_msgs/msg/String',
}

CAMERA_HINTS = ('image_raw', 'image_rect', 'color/image', '/rgb')
SUPPORT_HINTS = ('camera_info', 'depth')


class TopicAudit(Node):
    """Compare the live topic graph with the inspection contract."""

    def __init__(self) -> None:
        """Read parameters and prepare the audit."""
        super().__init__('topic_audit')
        self.declare_parameter('settle_sec', 3.0)
        self.declare_parameter('camera_topic', '/rgb')
        self.declare_parameter('debug_image_topic', '/inspection/debug_image')

        self._settle_sec = float(
            self.get_parameter('settle_sec').value
        )
        self._camera_topic = str(
            self.get_parameter('camera_topic').value
        )
        self._debug_image_topic = str(
            self.get_parameter('debug_image_topic').value
        )

    def settle(self) -> None:
        """Spin until DDS discovery has converged.

        Endpoint counts read as zero right after start-up, so a fixed
        wait is required before any of the checks below are meaningful.
        """
        end = time.monotonic() + self._settle_sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def topics(self) -> dict[str, str]:
        """Return the live topic graph as a name to type mapping."""
        return {
            name: types[0] if types else ''
            for name, types in self.get_topic_names_and_types()
        }

    def describe_publisher_qos(self, topic: str) -> list[str]:
        """Return one readable QoS line per publisher of a topic."""
        lines = []
        for info in self.get_publishers_info_by_topic(topic):
            qos = info.qos_profile
            lines.append(
                f'{qos.reliability.name}/{qos.durability.name}'
                f'/{qos.history.name}(depth={qos.depth})'
            )
        return lines

    def check_contract(self, found: dict[str, str]) -> bool:
        """Report contract topics and whether their types match."""
        log = self.get_logger()
        ok = True
        log.info('[1] 계약 Topic (docs/02-interfaces.md 3절)')
        for topic, expected in CONTRACT_TOPICS.items():
            actual = found.get(topic)
            if actual is None:
                log.error(f'    MISSING  {topic}')
                ok = False
            elif actual and actual != expected:
                log.error(
                    f'    TYPE     {topic}: {actual} (기대 {expected})'
                )
                ok = False
            else:
                log.info(f'    OK       {topic}')
        return ok

    def check_clock(self) -> None:
        """Report /clock publishers without treating absence as failure.

        The inspection executor reaches READY on model load plus first
        image only, and every timeout inside it uses a monotonic clock.
        A missing /clock therefore blocks sim-time alignment with the
        other nodes, not inspection readiness.
        """
        log = self.get_logger()
        count = self.count_publishers('/clock')
        log.info(f'[2] /clock publishers={count}')
        if count:
            log.info('    use_sim_time 사용 가능')
        else:
            log.warning(
                '    /clock 없음: 장면에 ROS_Clock 그래프가 없습니다. '
                'Inspection READY 자체는 막지 않습니다.'
            )

    def check_camera(self, found: dict[str, str]) -> bool:
        """Report camera candidates and validate the configured one."""
        log = self.get_logger()
        ok = True
        log.info('[3] 카메라 Topic 후보 — camera_topic 에 넣을 값')
        candidates = sorted(
            name for name in found
            if any(hint in name for hint in CAMERA_HINTS)
        )
        for name in candidates:
            log.info(f'    {name}  [{found[name] or "?"}]')
            for line in self.describe_publisher_qos(name):
                log.info(f'        pub QoS: {line}')

        if not candidates:
            log.error(
                '    후보 없음: Isaac Sim 카메라 퍼블리셔가 꺼져 있습니다.'
            )
            ok = False
        elif len(candidates) > 1:
            log.warning(
                '    후보가 여러 개입니다. color 계열 하나로 고정하세요.'
            )

        support = sorted(
            name for name in found
            if any(hint in name for hint in SUPPORT_HINTS)
        )
        log.info(f'    depth/camera_info: {support or "없음"}')

        if self._camera_topic not in found:
            log.error(
                f'    설정된 camera_topic={self._camera_topic} 가 '
                '토픽 그래프에 없습니다. 노드는 IMAGE_TIMEOUT 으로 끝납니다.'
            )
            ok = False
        else:
            log.info(f'    설정값 OK: camera_topic={self._camera_topic}')
        return ok

    def check_status_qos(self) -> None:
        """Warn about the durability mismatch trap on /inspection/status."""
        log = self.get_logger()
        log.info('[4] /inspection/status QoS')
        lines = self.describe_publisher_qos('/inspection/status')
        if not lines:
            log.warning('    퍼블리셔 없음: 비전 노드가 기동하지 않았습니다.')
            return
        for line in lines:
            log.info(f'    pub QoS: {line}')
            if 'VOLATILE' in line:
                log.info(
                    '    echo 할 때 --qos-durability transient_local 을 '
                    '쓰지 마세요. durability 불일치로 한 건도 받지 못합니다.'
                )

    def check_debug_image(self, found: dict[str, str]) -> None:
        """Report the debug image topic and why it may be absent."""
        log = self.get_logger()
        log.info('[5] 디버그 영상')
        if self._debug_image_topic in found:
            log.info(f'    OK  {self._debug_image_topic}')
        else:
            log.warning(
                f'    없음  {self._debug_image_topic} '
                '(continuous_inference=false 면 명령 전까지 정상)'
            )

    def run(self) -> int:
        """Run every check and return a process exit code."""
        self.settle()
        found = self.topics()
        self.get_logger().info(f'발견한 Topic {len(found)}개')

        ok = self.check_contract(found)
        self.check_clock()
        ok = self.check_camera(found) and ok
        self.check_status_qos()
        self.check_debug_image(found)

        self.get_logger().info(f'RESULT: {"PASS" if ok else "FAIL"}')
        return 0 if ok else 1


def main(args: list[str] | None = None) -> None:
    """Run the topic audit once and exit with its result code."""
    rclpy.init(args=args)
    node = TopicAudit()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
