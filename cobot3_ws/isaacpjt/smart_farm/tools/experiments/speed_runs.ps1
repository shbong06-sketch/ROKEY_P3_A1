param([string[]]$Conds = @(), [int]$Rep = 1, [string]$Root = "D:\smartfarm-sim\out\exp_20260927_speed")
# 주행 속도 실험: 조건 이름=Speed|Accel|DockArgs. 수확 -> 주행·도킹 -> 내려놓기까지만 돌리고 끊는다.
#   예: speed_runs.ps1 -Conds v06d1,v10 -Rep 1   (조건 뒤 _m<질량> = 포기 질량 실험). 일반 조건표는 ../allinone_live/run_conditions.ps1
$Tool = Join-Path $PSScriptRoot "..\allinone_live\run_allinone.ps1"
$MkVid = "/mnt/" + $PSScriptRoot.Substring(0, 1).ToLower() + $PSScriptRoot.Substring(2).Replace('\', '/') + "/mkvid.sh"
$Table = @{
  "v03"      = @("0.3", "", "")
  "v06"      = @("0.6", "", "")
  "v10"      = @("1.0", "", "")
  "v10a06"   = @("1.0", "0.6", "")
  "v10a10"   = @("1.0", "1.0", "")
  "v10a06d1" = @("1.0", "0.6", "reverse_speed_mps:=0.15,creep_speed_mps:=0.08,quiet_s:=1.0")
  "v08"      = @("0.8", "", "")
  "v06a06"   = @("0.6", "0.6", "")
  "v06d1"    = @("0.6", "", "reverse_speed_mps:=0.15,creep_speed_mps:=0.08,quiet_s:=1.0")
  "v06a06d1" = @("0.6", "0.6", "reverse_speed_mps:=0.15,creep_speed_mps:=0.08,quiet_s:=1.0")
  "v10a10d2" =@("1.0", "1.0", "reverse_speed_mps:=0.20,creep_speed_mps:=0.10,quiet_s:=0.5,turn_speed_radps:=0.5")
}
Remove-Item Env:SMARTFARM_VISION_ONLY -ErrorAction SilentlyContinue
# 녹화: 공정 카메라(수확·주행/놓기)를 0.5 s(시뮬레이션) 마다 캡처 -> 회차마다 mp4
$env:CABBAGE_CAPTURE_CAMS = "harvest=/World/ProcessCameras/Cam1_Harvest;navplace=/World/ProcessCameras/Cam2_Nav2Place"
$env:CABBAGE_CAPTURE_EVERY = "0.5"
foreach ($c in $Conds) {
  # 조건 뒤에 _m<질량> 을 붙이면 포기 질량 실험 (예: v10a10d2_m0.7)
  $base, $mass = $c -split "_m", 2
  $v = $Table[$base]
  if ($mass) { $env:CABBAGE_HEAD_MASS = $mass } else { Remove-Item Env:CABBAGE_HEAD_MASS -ErrorAction SilentlyContinue }
  $dir = "$Root\${c}_r$Rep"
  if (Test-Path "$dir\done.json") { "skip $dir"; continue }
  New-Item -ItemType Directory -Force $dir | Out-Null
  $args2 = @("-NoProfile","-ExecutionPolicy","Bypass","-File",$Tool,"-OutDir",$dir,"-NoRviz","-Speed",$v[0])
  if ($v[1]) { $args2 += @("-Accel",$v[1]) }
  if ($v[2]) { $args2 += @("-DockArgs",$v[2]) }
  $t0 = Get-Date
  $p = Start-Process -FilePath powershell.exe -ArgumentList $args2 -PassThru -WindowStyle Hidden -RedirectStandardOutput "$dir\runner.out" -RedirectStandardError "$dir\runner.err"
  $placed = $false
  while (((Get-Date) - $t0).TotalMinutes -lt 25) {
    if ($p.WaitForExit(10000)) { break }
    $fl = Get-Content "$dir\flow.log" -Raw -ErrorAction SilentlyContinue
    if ($fl -match "(?s)PLACE_INSPECT.*sim result") { $placed = $true; Start-Sleep 15; break }
    if ($fl -match "navigation result (FAILED|ABORTED|CANCELED)|\[flow\] error") { Start-Sleep 10; break }
  }
  if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue }
  Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*run_with_monitor.py*' -or $_.CommandLine -like '*ros_tcp_relay*' -or $_.CommandLine -like '*--yolo-worker*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
  Get-Process kit -ErrorAction SilentlyContinue | Where-Object { $_.Path -like 'D:\isaacsim*' } | Stop-Process -Force -ErrorAction SilentlyContinue
  wsl --terminate Ubuntu-24.04 | Out-Null
  @{ cond = $c; head_mass = $(if ($mass) { $mass } else { "0.3" }); speed = $v[0]; accel = $v[1]; dock = $v[2]; placed = $placed; wall_min = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1) } | ConvertTo-Json -Compress | Set-Content "$dir\done.json" -Encoding UTF8
  $wdir = "/mnt/d" + $dir.Substring(2).Replace('\','/')
  wsl -d Ubuntu-24.04 -- bash $MkVid $wdir
  "end $dir placed=$placed $([math]::Round(((Get-Date) - $t0).TotalMinutes,1)) min"
  Start-Sleep 5
}
