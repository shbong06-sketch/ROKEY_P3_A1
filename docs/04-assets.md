# v015 장면 자산 준비

이 문서는 [통합 시연 운영 가이드](03-operations.md)에서 사용하는 Isaac Sim v015 장면의 배포 위치와 설치 구조를 기록한다. 장면 파일은 Git에 없으므로 새 실행 PC마다 별도로 준비해야 한다.

## 다운로드 대상

| 항목 | 값 |
| --- | --- |
| 다운로드 | [Google Drive 공유 파일](https://drive.google.com/file/d/1yKIZRrcJuNADvLh1fkSNejRKs6LKzrXT/view?usp=drive_link) |
| 압축 파일명 | `Collected_smartfarm_v015.zip` |
| 압축 파일 크기 | 106.8 MB |
| 메인 USD | `Collected_smartfarm_v015.usd` |
| 설치 위치 | `cobot3_ws/isaacpjt/smart_farm/scenes/` |

압축 파일은 링크에서 수동으로 내려받는다. 압축본은 메인 USD가 들어 있는 **폴더 전체**를 아래 설치 위치에 배치한다. `SubUSDs`나 텍스처만 따로 옮기면 참조가 깨질 수 있다.

```bash
unzip -l ~/Downloads/Collected_smartfarm_v015.zip | head
unzip ~/Downloads/Collected_smartfarm_v015.zip
mv ~/Downloads/Collected_smartfarm_v015/* ROKEY_P3_A1/cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v015/*
```

압축 해제 후 저장소 루트 기준 메인 파일이 다음 경로에 있어야 한다.

```text
cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v015/Collected_smartfarm_v015.usd
```

## 설치 후 디렉터리 구조

v015 장면에서 확인한 주요 항목이다. 하위 파일 전체는 1,002개이며, 압축을 푼 디렉터리의 디스크 사용량은 약 327 MB다.

```text
cobot3_ws/isaacpjt/smart_farm/scenes/
└── Collected_smartfarm_v015/
    ├── Collected_smartfarm_v015.usd   # 메인 장면
    ├── integration_v1_physics.usda    # 물리 관련 레이어
    ├── env_dressing.usd
    ├── .collect.mapping.json
    ├── SubUSDs/
    │   ├── materials/
    │   ├── textures/
    │   └── *.usd
    ├── env_dressing_tex/
    ├── assets/
    │   └── cabbage_pallet_6/
    │       └── textures/
    └── omniverse-content-production.s3-us-west-2.amazonaws.com/
        └── Assets/
```

`.collect.mapping.json`은 수집 당시의 원본 위치와 배포 폴더 안의 대상 경로를 기록한다. 로컬 사본의 대상 경로 940개는 모두 존재한다. 이 매핑에는 이전 v014 작업 경로가 `source_url`로 남아 있지만, 설치 기준은 `target_url`의 v015 폴더 내 상대 경로다. 이것만으로 Isaac Sim에서 모든 USD 참조가 정상 로드된다는 뜻은 아니므로 장면 열기까지 확인한다.

## 함께 필요한 자산과 지도

| 구분 | 위치·역할 |
| --- | --- |
| 장면에 동봉된 자산 | `SubUSDs/`의 로봇·설비 USD, 재질·텍스처, `env_dressing.usd`와 `env_dressing_tex/`, `assets/cabbage_pallet_6/`, 수집된 Omniverse 자산 폴더를 메인 USD와 함께 유지한다. |
| 실행 코드의 M0609 파일 | `cobot3_ws/isaacpjt/M0609/doosan-robot2/urdf/m0609_isaac_sim.urdf`와 `M0609/descriptor/m0609_description.yaml`은 Standalone 런타임이 별도로 요구하며 Git에 포함되어 있다. |
| Nav2 지도 | Git에 포함된 `cobot3_ws/src/smart_farm_navigation/maps/Collected_smartfarm_v015.yaml`과 같은 폴더의 `Collected_smartfarm_v015.png`를 함께 사용한다. YAML의 `image` 항목이 PNG 파일을 가리킨다. |

[Standalone 진입점](../cobot3_ws/isaacpjt/smart_farm/runtime/standalone_app.py)은 v015 메인 USD가 있으면 이를 기본 장면으로 고른다. 파일이 없으면 다른 버전으로 대체될 수 있으므로 통합 실행에서는 [운영 가이드](03-operations.md)처럼 `--scene`에 위 경로를 명시한다. [Nav2 launch](../cobot3_ws/src/smart_farm_navigation/launch/nav2.launch.py)의 기본 지도도 v015다. v015 장면과 v015 지도는 한 쌍으로 사용하고, v014 지도나 다른 장면으로 바꾸는 경우 위치·축·도킹 좌표를 다시 검증해야 한다.

## 체크섬과 설치 확인

메인 USD와 지도 값은 현재 작업 PC에서 계산했고, 압축 파일 값은 공유자가 전달한 SHA-256이다. 메인 USD 값은 해제된 사본의 무결성 확인용이며, 압축본 전체의 체크섬을 대신하지 않는다.

| 파일 | SHA-256 |
| --- | --- |
| `Collected_smartfarm_v015.usd` | `26b0237cbb8991e9d488b81dc4ab1a7ba6341a60677388b969df916798ad7c92` |
| `Collected_smartfarm_v015.yaml` | `721ebbcd5bf5e07d8d960660b46ca6110418a16cd4b0144313b751f548af1b13` |
| `Collected_smartfarm_v015.zip` | `a98c668be6bb8d75cf2a7f251883296e920c31fef14841642cc67e63fece2c18` |

다운로드한 PC에서 ZIP의 정확한 바이트 수와 SHA-256을 계산해 위 값과 대조한다. 압축 해제 후에는 메인 파일과 지도 경로를 확인한다.

```bash
stat -c '%n %s bytes' ~/Downloads/Collected_smartfarm_v015.zip
sha256sum ~/Downloads/Collected_smartfarm_v015.zip

cd ~/ROKEY_P3_A1
test -f cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v015/Collected_smartfarm_v015.usd
test -f cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v015/env_dressing.usd
test -f cobot3_ws/src/smart_farm_navigation/maps/Collected_smartfarm_v015.png
sha256sum cobot3_ws/isaacpjt/smart_farm/scenes/Collected_smartfarm_v015/Collected_smartfarm_v015.usd
```

## Git에 포함하지 않은 이유

메인 USD는 많은 바이너리 USD·텍스처·재질에 의존하고, 현재 해제 폴더만 약 327 MB다. 이 자산을 일반 Git 이력에 넣으면 저장소와 변경 이력이 커져 코드·문서 배포에 불필요한 부담이 생긴다. 실제 [`.gitignore`](../cobot3_ws/isaacpjt/smart_farm/.gitignore)는 `scenes/Collected_smartfarm*/*`와 장면 ZIP을 제외한다. 따라서 코드는 Git으로, 장면 묶음은 위 공유 파일로 배포하고, 설치 경로와 체크섬으로 두 사본을 맞춘다.
