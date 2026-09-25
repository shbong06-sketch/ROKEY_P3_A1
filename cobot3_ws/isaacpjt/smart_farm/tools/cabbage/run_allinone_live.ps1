# All-in-one live run on this Windows PC: Isaac Sim GUI (team standalone_app + conveyor) + real Nav2 in WSL2.
#   powershell -File run_allinone_live.ps1 [-OutDir D:\smartfarm-sim\out\allinone_run] [-Standoff 0.92] [-NoFlow] [-NoRviz]
# Flow (team guidance2_25 order): PICK_HARVEST -> NAVIGATION FEEDER_DOCK (Nav2 + feeder_dock) -> PLACE_INSPECT
# -> the conveyor takes the tray down the stem to the vision booth, holds it, releases it (auto after 10 s).
param([string]$OutDir = "D:\smartfarm-sim\out\allinone_run", [string]$Standoff = "0.92", [switch]$NoFlow, [switch]$NoRviz,
      [string]$Scene = "D:\v013_extract\Collected_smartfarm_v013\Collected_smartfarm_v013_room_core_cabbage.usd",
      [string]$Root = "D:\smartfarm-sim\ROKEY_P3_A1\cobot3_ws\isaacpjt\smart_farm")
New-Item -ItemType Directory -Force $OutDir | Out-Null
$wslOut = "/mnt/" + $OutDir.Substring(0,1).ToLower() + $OutDir.Substring(2).Replace('\','/')
$env:WSL_UTF8 = "1"

"[1/5] clean start (old Isaac / relay / every WSL ROS process)"
Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*aio_wrapper.py*' -or $_.CommandLine -like '*ros_tcp_relay*' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
wsl --terminate Ubuntu-24.04 | Out-Null
Start-Sleep 2

"[2/5] WSL: relay server + Nav2 (+RViz2) + navigation_node (feeder_dock standoff_m=$Standoff)"
$rviz = if ($NoRviz) { "false" } else { "true" }
$stack = Start-Process -FilePath wsl.exe -PassThru -WindowStyle Hidden -RedirectStandardOutput "$OutDir\stack.txt" -RedirectStandardError "$OutDir\stack.err" `
    -ArgumentList "-d","Ubuntu-24.04","--","env","STANDOFF=$Standoff","bash","/mnt/d/smartfarm-sim/scripts/wsl/start_nav2_stack.sh",$wslOut,$rviz

$env:SMARTFARM_STATION_OUT = "$OutDir\station"     # vision_cull_station: YOLO images + station_results.json
"[3/5] Windows: Isaac Sim GUI + team standalone_app + Windows relay"
powershell -NoProfile -ExecutionPolicy Bypass -File D:\smartfarm-sim\scripts\cabbage\run_isaac_for_nav2.ps1 -Scene $Scene -OutDir $OutDir -Root $Root

"[4/5] waiting for Nav2 + navigation_node"
$t0 = Get-Date
while (-not ((Get-Content "$OutDir\stack.txt" -Raw -ErrorAction SilentlyContinue) -match "navigation_node ready")) {
    Start-Sleep 5
    if (((Get-Date) - $t0).TotalMinutes -gt 10) { "Nav2 stack did not come up - see $OutDir\nav2.log"; exit 1 }
}
if ((Get-Content "$OutDir\nav2.log" -Raw) -match "Aborting bringup") { "Nav2 bringup aborted - see $OutDir\nav2.log"; exit 1 }
"Nav2 ready after $([int]((Get-Date) - $t0).TotalSeconds) s"
if ($NoFlow) { "ready - send commands yourself (guidance2_25 section 8)"; exit 0 }

"[5/5] flow: PICK_HARVEST -> NAVIGATION FEEDER_DOCK -> PLACE_INSPECT"
wsl -d Ubuntu-24.04 -- bash -c "source ~/nav2_env.sh; python3 /mnt/d/smartfarm-sim/scripts/wsl/run_flow.py $wslOut/flow.json 2>&1 | tee $wslOut/flow.log | grep '\[flow\]'"
"conveyor + vision station (waiting for the station to finish Pallet_01, max 30 min):"
$t1 = Get-Date
while (-not ((Get-Content "$OutDir\isaac.log" -Raw -Encoding UTF8 -ErrorAction SilentlyContinue) -match "Pallet_01 완료")) {
    Start-Sleep 5
    if (((Get-Date) - $t1).TotalMinutes -gt 30) { "station did not finish in 30 min"; break }
}
Start-Sleep 20      # the tray leaving the vision room (recording)
Get-Content "$OutDir\isaac.log" -Encoding UTF8 | Select-String "\[(컨베이어|비전|솎아내기)\]" | ForEach-Object { "  " + $_.Line }
