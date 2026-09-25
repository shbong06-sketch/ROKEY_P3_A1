# 올인원 한 줄 실행 (Windows PowerShell): Isaac Sim GUI(팀 standalone_app + 컨베이어 + 비전 검사·선별) + WSL2 의 실제 Nav2.
#   powershell -NoProfile -ExecutionPolicy Bypass -File run_allinone.ps1 [-OutDir DIR] [-Human] [-Lane] [-NoFlow] [-NoRviz]
#
# 흐름: PICK_HARVEST(랙 수확) -> NAVIGATION FEEDER_DOCK(Nav2 + feeder_dock) -> PLACE_INSPECT(벨트에 내려놓기)
#       -> 컨베이어 -> 비전룸 YOLO 검사 -> 노랑·갈색 솎아내기(SortBin_1/2) -> 재검사 -> 배출
# 옵션 (기본은 모두 꺼짐 = 팀 기본 동작 그대로)
#   -Human   사람 돌발상황: 작업자가 카터 경로에 들어왔다 비킨다. 카터는 트레이 끝 0.8 m 앞에서 멈췄다가 다시 간다.
#            (scripts/human_crossing.py + Nav2 설정 복사본의 HumanStop/HumanSlow. 사람 에셋을 NVIDIA 서버에서 받으므로 인터넷 필요)
#   -Lane    바닥 노란 차선 주행: 랙 통로 차선 중앙선을 따라 후진 -> 모서리 곡선 45 deg 까지 -> Nav2 기본 경로로 FEEDER
#   -NoFlow  Nav2·Isaac 만 띄우고 명령은 보내지 않음 (직접 보내려면 wsl/run_flow.py)
#   -Scene   씬 USD (기본 DEFAULT = standalone_app 의 기본 씬, scenes/Collected_smartfarm_v014/...cabbage.usd)
# 팀 파일은 바꾸지 않는다: Nav2 는 팀 nav2_params.yaml 을 읽어 만든 복사본(OUT/nav2_params_test.yaml)을 쓴다.
# 주의: 이 스크립트 출력을 | Out-Null, | Select-Object 로 넘기지 말 것 (Isaac·중계기 자식 프로세스가 출력을 잡고 있어 끝나지 않음).
param([string]$OutDir = "$HOME\smartfarm_runs\allinone_run",
      [switch]$Human, [switch]$Lane, [switch]$NoFlow, [switch]$NoRviz,
      [string]$Scene = "DEFAULT",
      [string]$Standoff = "0.92",
      [string]$IsaacDir = "D:\isaacsim",
      [string]$Distro = "Ubuntu-24.04",
      [string]$PyLib = "")          # 중계기(Windows 쪽)용 추가 라이브러리 폴더. 비우면 ~\smartfarm_runs\pylib 에 pyyaml·numpy 를 자동 설치
$ErrorActionPreference = "Continue"
$Here = $PSScriptRoot
$SmartFarm = (Resolve-Path "$Here\..\..").Path                     # .../isaacpjt/smart_farm
function To-Wsl([string]$p) { "/mnt/" + $p.Substring(0,1).ToLower() + $p.Substring(2).Replace('\','/') }
New-Item -ItemType Directory -Force $OutDir | Out-Null
$OutDir = (Resolve-Path $OutDir).Path
$wslOut = To-Wsl $OutDir
$wslHere = To-Wsl "$Here\wsl"
$env:WSL_UTF8 = "1"

"[1/5] 이전 실행 정리 (Isaac·중계기·WSL ROS)"
Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*run_with_monitor.py*' -or $_.CommandLine -like '*ros_tcp_relay*' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
wsl --terminate $Distro | Out-Null
Start-Sleep 2

"[2/5] WSL: 중계기 + Nav2(+RViz2) + navigation_node  (Human=$Human Lane=$Lane)"
$rviz = if ($NoRviz) { "false" } else { "true" }
Start-Process -FilePath wsl.exe -WindowStyle Hidden -RedirectStandardOutput "$OutDir\stack.txt" -RedirectStandardError "$OutDir\stack.err" `
    -ArgumentList "-d",$Distro,"--","env","STANDOFF=$Standoff","HUMAN=$(if ($Human) {'1'} else {'0'})","LANE=$(if ($Lane) {'1'} else {'0'})",`
                  "bash","$wslHere/start_nav2_stack.sh",$wslOut,$rviz | Out-Null

"[3/5] Windows: Isaac Sim GUI + standalone_app (+기록) + 중계기"
$j = "$IsaacDir\exts\isaacsim.ros2.bridge\jazzy"
$env:SMARTFARM_ROS_REEXEC = "1"; $env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"; $env:ROS_DISTRO = "jazzy"
$env:ROS_DOMAIN_ID = "101"
[Environment]::SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", $null)   # Windows 안 DDS. WSL 과는 ros_tcp_relay.py 로 잇는다
$env:PATH = "$j\lib;" + $env:PATH; $env:PYTHONUNBUFFERED = "1"
$env:CABBAGE_MONITOR_OUT = "$OutDir\monitor.json"
$env:SMARTFARM_STATION_OUT = "$OutDir\station"
$log = "$OutDir\isaac.log"
$appArgs = "`"$Here\run_with_monitor.py`" `"$SmartFarm\runtime\standalone_app.py`" --autoplay"
if ($Scene -ne "DEFAULT") { $appArgs += " --scene `"$Scene`"" }
if ($Human) { $appArgs += " --human-crossing" }
Start-Process -FilePath "$IsaacDir\python.bat" -NoNewWindow -RedirectStandardOutput $log -RedirectStandardError "$log.err" -ArgumentList $appArgs
$t0 = Get-Date
while (-not ((Get-Content $log -Raw -Encoding UTF8 -ErrorAction SilentlyContinue) -match "\[READY\]")) {
    Start-Sleep 5; if (((Get-Date) - $t0).TotalMinutes -gt 15) { "Isaac 이 15분 안에 READY 가 안 됨 - $log 확인"; exit 1 }
}
"Isaac READY ($([int]((Get-Date) - $t0).TotalSeconds) s)"
if (-not $PyLib) {                    # Isaac 내장 rclpy 는 yaml·numpy 가 필요한데 Isaac 파이썬에는 없다 -> 처음 한 번 설치 (인터넷 필요)
    $PyLib = "$HOME\smartfarm_runs\pylib"
    if (-not ((Test-Path "$PyLib\yaml") -and (Test-Path "$PyLib\numpy"))) {
        "중계기용 pyyaml·numpy 설치 -> $PyLib"
        & "$IsaacDir\kit\python\python.exe" -m pip install --quiet --disable-pip-version-check --target $PyLib pyyaml numpy
    }
}
$env:PYTHONPATH = "$j\rclpy;$PyLib"
Start-Process -FilePath "$IsaacDir\kit\python\python.exe" -NoNewWindow -RedirectStandardOutput "$OutDir\relay_win.log" -RedirectStandardError "$OutDir\relay_win.err" `
    -ArgumentList "`"$Here\wsl\ros_tcp_relay.py`"","win"

"[4/5] Nav2 + navigation_node 대기"
$t0 = Get-Date
while (-not ((Get-Content "$OutDir\stack.txt" -Raw -ErrorAction SilentlyContinue) -match "navigation_node ready")) {
    Start-Sleep 5
    if (((Get-Date) - $t0).TotalMinutes -gt 10) { "Nav2 가 10분 안에 안 뜸 - $OutDir\nav2.log 확인"; exit 1 }
}
if ((Get-Content "$OutDir\nav2.log" -Raw) -match "Aborting bringup") { "Nav2 bringup 중단 - $OutDir\nav2.log 확인"; exit 1 }
"Nav2 ready ($([int]((Get-Date) - $t0).TotalSeconds) s)"
if ($NoFlow) { "준비 완료 - 명령은 직접: wsl -d $Distro -- bash -c `"source ~/nav2_env.sh; python3 $wslHere/run_flow.py $wslOut/flow.json`""; exit 0 }

"[5/5] 흐름: PICK_HARVEST -> NAVIGATION FEEDER_DOCK -> PLACE_INSPECT"
wsl -d $Distro -- bash -c "source ~/nav2_env.sh; python3 $wslHere/run_flow.py $wslOut/flow.json 2>&1 | tee $wslOut/flow.log | grep '\[flow\]'"
"컨베이어 + 비전 검사·선별 (Pallet_01 완료까지 최대 30분)"
$t1 = Get-Date
while (-not ((Get-Content $log -Raw -Encoding UTF8 -ErrorAction SilentlyContinue) -match "Pallet_01 완료")) {
    Start-Sleep 5
    if (((Get-Date) - $t1).TotalMinutes -gt 30) { "30분 안에 끝나지 않음"; break }
}
Start-Sleep 20
Get-Content $log -Encoding UTF8 | Select-String "\[(사람|컨베이어|비전|솎아내기)\]" | ForEach-Object { "  " + $_.Line }
"로그: $OutDir"
