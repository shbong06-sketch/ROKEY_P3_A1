#!/usr/bin/env bash
# ROS 2 / Isaac Sim 환경 충돌 사전 점검 (고피 전용). 각 터미널에서 ros_set, isaac_ros 후 실행한다.
#   bash /home/rokey/ROKEY_P3_A1/cobot3_ws/src/smart_farm_navigation/scripts/env_check.sh
# 종료 코드: 0 = 충돌 없음, 1 = 경고, 2 = 충돌(진행 금지)
ISAAC=${ISAAC_ROOT:-$HOME/isaacsim}
BRIDGE=$ISAAC/exts/isaacsim.ros2.bridge
status=0
note() { echo "  $*"; }
warn() { echo "  [WARN] $*"; [[ $status -lt 1 ]] && status=1; }
fail() { echo "  [CONFLICT] $*"; status=2; }

echo "== 1. 셸 환경 변수 =="
note "ROS_DISTRO=${ROS_DISTRO:-<unset>}  ROS_VERSION=${ROS_VERSION:-<unset>}  ROS_PYTHON_VERSION=${ROS_PYTHON_VERSION:-<unset>}"
note "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
note "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
[[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]] && warn "FastDDS profile 파일이 없음: $FASTRTPS_DEFAULT_PROFILES_FILE"
[[ "${ROS_DOMAIN_ID:-}" == "101" ]] || warn "ROS_DOMAIN_ID 가 101 이 아님"

echo "== 2. 시스템 ROS 2 =="
installed=$(ls -d /opt/ros/*/ 2>/dev/null | xargs -n1 basename 2>/dev/null | tr '\n' ' ')
note "설치된 배포판: ${installed:-<none>}"
sourced=""
for d in $installed; do [[ ":${AMENT_PREFIX_PATH:-}:" == *":/opt/ros/$d:"* ]] && sourced="$sourced $d"; done
note "소싱된 배포판(AMENT_PREFIX_PATH):${sourced:-<none>}"
if [[ -z "$sourced" ]]; then
  warn "시스템 ROS 2 가 소싱되지 않음 (ros_set 필요)"
elif [[ $(echo $sourced | wc -w) -gt 1 ]]; then
  fail "배포판 두 개가 동시에 소싱됨:$sourced"
elif [[ -n "${ROS_DISTRO:-}" && "${sourced// /}" != "$ROS_DISTRO" ]]; then
  fail "ROS_DISTRO=$ROS_DISTRO 이지만 소싱된 배포판은${sourced}"
fi
note "ros2 실행 파일: $(command -v ros2 || echo '<none>')"
note "시스템 python3: $(python3 --version 2>&1)"
python3 -c "import rclpy" 2>/dev/null && note "rclpy import: OK" || warn "rclpy import 실패 (ros_set 이후에도 실패하면 설치 문제)"

echo "== 3. Isaac Sim =="
if [[ -d "$ISAAC" ]]; then
  note "경로: $ISAAC  버전: $(cat "$ISAAC/VERSION" 2>/dev/null || echo '<unknown>')"
  note "Isaac python: $("$ISAAC/kit/python/bin/python3" --version 2>&1 || echo '<unknown>')"
  shipped=$(ls -d "$BRIDGE"/*/ 2>/dev/null | xargs -n1 basename 2>/dev/null | grep -vE '^(config|docs|data|isaacsim|omni|PACKAGE|bin|lib)$' | tr '\n' ' ')
  note "bridge 내장 ROS 2 배포판: ${shipped:-<none>}"
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    if [[ -d "$BRIDGE/$ROS_DISTRO/lib" ]]; then note "bridge/$ROS_DISTRO/lib: 있음 (시스템 ROS 미소싱 시 내장 lib 사용 가능)"
    else warn "bridge 에 $ROS_DISTRO 내장 lib 이 없음 -> Isaac Sim 은 반드시 시스템 ROS($ROS_DISTRO) 를 소싱한 터미널에서만 실행한다. 다른 배포판 내장 lib(예: humble) 을 LD_LIBRARY_PATH 에 넣으면 안 된다"; fi
  fi
else
  fail "Isaac Sim 디렉터리가 없음: $ISAAC"
fi

echo "== 4. LD_LIBRARY_PATH 의 bridge 항목 =="
IFS=':' read -ra parts <<< "${LD_LIBRARY_PATH:-}"
found=0
for p in "${parts[@]}"; do
  [[ "$p" == *isaacsim.ros2.bridge* ]] || continue
  found=1
  d=$(basename "$(dirname "$p")")
  if [[ ! -d "$p" ]]; then warn "존재하지 않는 경로가 등록됨(무해하나 isaac_ros 가 실제 lib 을 못 잡음): $p"
  elif [[ -n "${ROS_DISTRO:-}" && "$d" != "$ROS_DISTRO" ]]; then fail "bridge lib 배포판($d) 이 ROS_DISTRO($ROS_DISTRO) 와 다름: $p"
  else note "bridge lib 등록됨: $p"; fi
done
[[ $found -eq 0 ]] && warn "LD_LIBRARY_PATH 에 bridge lib 없음 (isaac_ros 필요)"
for p in "${parts[@]}"; do
  [[ "$p" == /opt/ros/*/lib* ]] || continue
  d=${p#/opt/ros/}; d=${d%%/*}
  [[ -n "${ROS_DISTRO:-}" && "$d" != "$ROS_DISTRO" ]] && fail "LD_LIBRARY_PATH 에 다른 배포판 lib: $p"
done

echo "== 5. 실행 중인 ROS 2 그래프 =="
if command -v ros2 >/dev/null && [[ -n "$sourced" ]]; then
  n=$(timeout 8 ros2 topic list 2>/dev/null | wc -l)
  note "보이는 topic 수: $n (Isaac Sim Play 전에는 /parameter_events, /rosout 2개가 정상)"
fi

echo "== 판정 =="
case $status in
  0) echo "  OK: 충돌 없음";;
  1) echo "  WARN: 경고 항목 확인 후 진행 가능";;
  2) echo "  CONFLICT: 위 항목을 해결하기 전에는 Isaac Sim / 노드를 실행하지 않는다";;
esac
exit $status
