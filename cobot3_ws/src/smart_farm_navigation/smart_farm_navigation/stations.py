"""작업점 파일(config/stations.yaml) 읽기와 map 기준 목표 자세 생성.

navigation_node(비동기 액션)와 go_to_station(명령줄 도구)이 같은 값을 쓰도록 한 곳에 둔다.
"""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped


def default_path() -> str:
    return os.path.join(
        get_package_share_directory("smart_farm_navigation"), "config", "stations.yaml"
    )


def load(path: str = "") -> dict:
    with open(path or default_path(), encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def pose(x: float, y: float, yaw_deg: float, frame_id: str = "map") -> PoseStamped:
    """map 기준 목표 자세. stamp 는 0 으로 두어 '가장 최근 변환'을 쓰게 한다.

    벽시계 stamp 를 넣으면 Isaac 의 시뮬레이션 시각과 어긋나 Nav2 가 목표를 즉시 거부한다.
    """
    goal = PoseStamped()
    goal.header.frame_id = frame_id
    half = math.radians(float(yaw_deg)) / 2.0
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.orientation.z = math.sin(half)
    goal.pose.orientation.w = math.cos(half)
    return goal


def pose_of(stations: dict, name: str, frame_id: str = "map") -> PoseStamped:
    station = stations["stations"][name]
    return pose(station["x"], station["y"], station["yaw_deg"], frame_id)
