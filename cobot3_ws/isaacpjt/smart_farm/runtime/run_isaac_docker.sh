#!/usr/bin/env bash
# Isaac Sim 5.1.0 컨테이너에서 vision_functest.py 를 돌린다. 호스트에서 실행.
#
#   ./runtime/run_isaac_docker.sh --check              # 사전 점검만
#   ./runtime/run_isaac_docker.sh --keep-pose          # /rgb 발행
#   ./runtime/run_isaac_docker.sh --keep-pose --bridge # + 검출 -> base 좌표
#   ./runtime/run_isaac_docker.sh --shell              # 컨테이너 셸 (디버깅)
#
# 나머지 인자는 vision_functest.py 로 그대로 넘어간다.
#
# 호스트 쪽 ROS 터미널에도 같은 값을 넣는다:
#   export ROS_DOMAIN_ID=<같은 값>
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
#
# 덮어쓸 수 있는 환경변수 (괄호는 기본값):
#   IMAGE (nvcr.io/nvidia/isaac-sim:5.1.0)
#   PROJECT (이 스크립트의 상위 폴더) · M0609 (PROJECT 옆의 M0609)
#   LULA_DIR (~/Downloads/.../lula) · ISAAC_CACHE (~/docker/isaac-sim)
#   ISAAC_USER (비우면 이미지 기본 사용자) · ROS_DOMAIN_ID (0)
set -euo pipefail

IMAGE=${IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT=${PROJECT:-$(dirname "$SCRIPT_DIR")}
M0609=${M0609:-$(dirname "$PROJECT")/M0609}
LULA_DIR=${LULA_DIR:-$HOME/Downloads/smartfarm_v008_vision/Collected_smartfarm_v008/lula}
ISAAC_CACHE=${ISAAC_CACHE:-$HOME/docker/isaac-sim}
ISAAC_USER=${ISAAC_USER:-}
PROJECT_NAME=$(basename "$PROJECT")
WS=/workspace                      # 프로젝트와 M0609 를 나란히 둔다
ISAAC_ROOT=/isaac-sim

say() { printf '[docker] %s\n' "$*"; }
die() { printf '[docker][오류] %s\n' "$*" >&2; exit 1; }

command -v docker >/dev/null || die "docker 가 없습니다."
docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  say "이미지가 없어 받습니다: $IMAGE"; docker pull "$IMAGE"; }

# 이미지 안의 사용자·홈을 실제로 물어본다 (버전마다 root/비root 가 다를 수 있다).
USER_ARGS=()
[[ -n "$ISAAC_USER" ]] && USER_ARGS=(-u "$ISAAC_USER")
IMG_HOME=$(docker run --rm "${USER_ARGS[@]}" --entrypoint sh "$IMAGE" -c 'echo $HOME')
IMG_UID=$(docker run --rm "${USER_ARGS[@]}" --entrypoint sh "$IMAGE" -c 'id -u')

# 캐시 볼륨: 셰이더 컴파일을 두 번째 실행부터 건너뛰기 위함.
CACHE_DIRS=(".cache" ".nv/ComputeCache" ".nvidia-omniverse/logs"
            ".nvidia-omniverse/config" ".local/share/ov/data")
CACHE_ARGS=()
for d in "${CACHE_DIRS[@]}"; do
  host="$ISAAC_CACHE/${d//\//_}"
  mkdir -p "$host"
  # 컨테이너 사용자 uid 가 호스트와 다르면 쓰기가 막힌다. 캐시라서 열어 둔다.
  [[ "$IMG_UID" != "$(id -u)" ]] && chmod a+rwx "$host"
  CACHE_ARGS+=(-v "$host:$IMG_HOME/$d:rw")
done

MOUNT_ARGS=(-v "$PROJECT:$WS/$PROJECT_NAME:ro")
[[ -d "$M0609" ]] && MOUNT_ARGS+=(-v "$M0609:$WS/M0609:ro")
[[ -d "$LULA_DIR" ]] && MOUNT_ARGS+=(-v "$LULA_DIR:$WS/lula:ro")

ENV_ARGS=(
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  # 컨테이너와 호스트는 사용자가 달라 공유메모리 전송이 조용히 실패할 수 있다.
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  -e M0609_LULA_DESC="$WS/lula/m0609_robot_description.yaml"
)
[[ -n "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" ]] && \
  ENV_ARGS+=(-e ROS_AUTOMATIC_DISCOVERY_RANGE="$ROS_AUTOMATIC_DISCOVERY_RANGE")

BASE_ARGS=(--rm --gpus all --network=host "${USER_ARGS[@]}"
           "${ENV_ARGS[@]}" "${MOUNT_ARGS[@]}" "${CACHE_ARGS[@]}")
FUNCTEST="$WS/$PROJECT_NAME/runtime/vision_functest.py"
# 터미널이 아닐 때(-t 불가) 도 돌도록.
if [[ -t 0 && -t 1 ]]; then TTY=(-it); else TTY=(-i); fi

check() {
  local ok=1
  say "이미지 $IMAGE · 컨테이너 사용자 uid=$IMG_UID home=$IMG_HOME"
  say "프로젝트 $PROJECT -> $WS/$PROJECT_NAME"
  [[ -f "$PROJECT/runtime/vision_functest.py" ]] || { say "없음: runtime/vision_functest.py"; ok=0; }
  [[ -f "$PROJECT/runtime/vision_bridge.py" ]] || say "경고: vision_bridge.py 없음 (--bridge 불가)"
  [[ -f "$PROJECT/scenes/Collected_smartfarm_v013/Collected_smartfarm_v013.usd" ]] \
    || { say "없음: 기본 씬 USD (다른 씬이면 --scene 지정)"; ok=0; }
  [[ -d "$M0609" ]] && say "M0609 $M0609" || say "경고: M0609 폴더 없음 (IK 모드 불가, --keep-pose 는 가능)"
  [[ -f "$LULA_DIR/m0609_robot_description.yaml" ]] && say "Lula $LULA_DIR" \
    || say "경고: Lula YAML 없음 (IK 모드 불가, --keep-pose 는 가능)"
  docker run "${BASE_ARGS[@]}" --entrypoint bash "$IMAGE" -c "
    set -e
    nvidia-smi --query-gpu=name,driver_version --format=csv,noheader \
      || { echo '[docker][오류] 컨테이너에서 GPU 가 안 보입니다 (NVIDIA Container Toolkit 확인)'; exit 1; }
    test -x $ISAAC_ROOT/python.sh || { echo '[docker][오류] $ISAAC_ROOT/python.sh 없음'; exit 1; }
    test -d $ISAAC_ROOT/exts/isaacsim.ros2.bridge/jazzy/lib \
      && echo '[docker] 번들 ROS 2 (jazzy) 확인' \
      || { echo '[docker][오류] 번들 ROS 경로가 다릅니다:'; ls -d $ISAAC_ROOT/exts/*ros2* || true; exit 1; }
    test -r $FUNCTEST && echo '[docker] 마운트 확인: $FUNCTEST' \
      || { echo '[docker][오류] 컨테이너에서 스크립트를 못 읽습니다 (권한/마운트)'; exit 1; }
  " || ok=0
  (( ok )) && say "점검 통과" || die "점검 실패 — 위 메시지를 확인하세요."
}

case "${1:-}" in
  --check) check; exit 0 ;;
  --shell) shift; exec docker run "${TTY[@]}" "${BASE_ARGS[@]}" --entrypoint bash "$IMAGE" ;;
esac

say "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} · 첫 실행은 셰이더 컴파일로 수 분 걸립니다."
exec docker run "${TTY[@]}" "${BASE_ARGS[@]}" \
  --entrypoint "$ISAAC_ROOT/python.sh" "$IMAGE" \
  "$FUNCTEST" --headless "$@"
