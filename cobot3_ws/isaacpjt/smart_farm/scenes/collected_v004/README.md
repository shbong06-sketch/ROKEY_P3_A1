# v004 월드 전달 자료

`Collected_smartfarm_v004.tar.gz`는 Git LFS 파일입니다. 약 533 MiB이며, 전체 월드와 SubUSDs, friction.py 등 보조 스크립트를 포함합니다. 백업과 Python 캐시는 제외했습니다.

## 받기

Git LFS를 설치한 뒤 저장소 루트에서 실행합니다.

```bash
git lfs install
git switch feature/lift_robot_motion
git lfs pull --include="cobot3_ws/isaacpjt/smart_farm/scenes/collected_v004/Collected_smartfarm_v004.tar.gz"
cd cobot3_ws/isaacpjt/smart_farm/scenes/collected_v004
sha256sum -c SHA256SUMS
tar -xzf Collected_smartfarm_v004.tar.gz -C "$HOME"
```

## 실행

- Isaac Sim 5.1.0 기준입니다.
- `smart_farm/scripts/robot_motion.py`의 `SCENE_PATH`를 자신의 `Collected_smartfarm_v004/Collected_smartfarm_v004.usd` 절대경로로 수정하세요. 현재 값은 `/home/rokey/Collected_smartfarm_v004/Collected_smartfarm_v004.usd`입니다.
- `lift.py`는 `robot_motion.py`와 같은 폴더에 있어야 합니다.
- 저장소의 `cobot3_ws/isaacpjt/M0609` 폴더 구조를 유지하세요. URDF와 descriptor YAML을 이 위치에서 읽습니다. URDF 메시의 절대경로는 별도로 가져오기/시각화할 경우 사용자 환경에 맞게 조정해야 합니다.
- 자신의 Isaac Sim 설치 폴더의 `python.sh`로 `robot_motion.py`를 실행하고 Play를 누르세요.
- 현재 TASKS는 Pallet_1의 1단 → 2단 작업입니다.

## 주의할 기존 사항

포크-팔레트 마찰은 미적용 상태이며 `friction.py`가 포함되어 있습니다. 해당 스크립트의 적용 대상과 저장 절차를 확인하세요. 캐리지·마스트 간 겹침 및 RG2의 `/visuals/world` 참조 경고도 남아 있습니다. 단순 실행 완료가 충돌 여유나 다른 PC에서의 검증까지 의미하지는 않습니다.
