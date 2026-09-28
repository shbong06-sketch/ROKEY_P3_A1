# 조건표(CSV) 실행기: 행마다 run_allinone.ps1 을 한 번씩 돌리고 회차 폴더에 done.json 을 남긴다 (2026-09-28).
#   powershell -NoProfile -ExecutionPolicy Bypass -File run_conditions.ps1 -Table conditions.csv -Root D:\runs\exp1 [-Only name1,name2]
#
# CSV 열 (비운 칸 = 팀 기본값). 한 행 = 한 조건, reps 번 반복 -> <Root>\<name>_r<n>\
#   name        조건 이름 (폴더 이름)                 reps        반복 횟수 (기본 1)
#   stop        place = 내려놓기 결과까지만 / full = 스테이션 완료(Pallet_01 완료)까지
#   speed accel dock_args fast   run_allinone.ps1 의 -Speed -Accel -DockArgs -Fast (fast = 1 이면 켬)
#   grip_force  SMARTFARM_GRIP_FORCE (N·m)           vision_only  1 = SMARTFARM_VISION_ONLY
#   cull_retry  SMARTFARM_CULL_RETRY                 head_mass    CABBAGE_HEAD_MASS (kg)
#   min_move_s  CABBAGE_MIN_MOVE_S (팀 robot_motion 구간 최소 보간 시간, 팀 값 0.5)
#   capture     1 = 공정 카메라 Cam1·Cam2 0.5 s 캡처 (영상 만들기용)
#   aim_above_tray vision_deadband vision_refresh   비전 전용 모드 설정 (SMARTFARM_AIM_ABOVE_TRAY 등)
# 판정이 전부 UNKNOWN(카메라 검은 화면)이면 valid=false 로 남기고 한 번 더 돌린다(-MaxTries).
param([Parameter(Mandatory = $true)][string]$Table, [Parameter(Mandatory = $true)][string]$Root,
      [string[]]$Only = @(), [int]$MaxTries = 2, [int]$TimeoutMin = 35)
$Tool = Join-Path $PSScriptRoot "run_allinone.ps1"
$EnvMap = @{ grip_force = "SMARTFARM_GRIP_FORCE"; vision_only = "SMARTFARM_VISION_ONLY"; cull_retry = "SMARTFARM_CULL_RETRY";
             head_mass = "CABBAGE_HEAD_MASS"; min_move_s = "CABBAGE_MIN_MOVE_S"; aim_above_tray = "SMARTFARM_AIM_ABOVE_TRAY";
             vision_deadband = "SMARTFARM_VISION_DEADBAND"; vision_refresh = "SMARTFARM_VISION_REFRESH" }
New-Item -ItemType Directory -Force $Root | Out-Null
foreach ($row in (Import-Csv $Table -Encoding UTF8)) {
  if ($Only.Count -and $Only -notcontains $row.name) { continue }
  foreach ($k in $EnvMap.Keys) { if ($row.$k) { Set-Item "Env:$($EnvMap[$k])" $row.$k } else { Remove-Item "Env:$($EnvMap[$k])" -ErrorAction SilentlyContinue } }
  if ($row.capture -eq "1") { $env:CABBAGE_CAPTURE_CAMS = "harvest=/World/ProcessCameras/Cam1_Harvest;navplace=/World/ProcessCameras/Cam2_Nav2Place"; $env:CABBAGE_CAPTURE_EVERY = "0.5" }
  else { Remove-Item Env:CABBAGE_CAPTURE_CAMS -ErrorAction SilentlyContinue }
  $reps = if ($row.reps) { [int]$row.reps } else { 1 }
  for ($rep = 1; $rep -le $reps; $rep++) {
    for ($try = 1; $try -le $MaxTries; $try++) {
      $dir = Join-Path $Root ("{0}_r{1}" -f $row.name, $rep) ; if ($try -gt 1) { $dir += "_try$try" }
      if (Test-Path "$dir\done.json") { if ((Get-Content "$dir\done.json" -Raw) -match '"valid":\s*true') { "skip $dir"; break } else { continue } }
      New-Item -ItemType Directory -Force $dir | Out-Null
      $a = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $Tool, "-OutDir", $dir, "-NoRviz")
      if ($row.speed) { $a += @("-Speed", $row.speed) }
      if ($row.accel) { $a += @("-Accel", $row.accel) }
      if ($row.dock_args) { $a += @("-DockArgs", $row.dock_args) }
      if ($row.fast -eq "1") { $a += "-Fast" }
      $t0 = Get-Date
      $p = Start-Process -FilePath powershell.exe -ArgumentList $a -PassThru -WindowStyle Hidden -RedirectStandardOutput "$dir\runner.out" -RedirectStandardError "$dir\runner.err"
      $placed = $false
      while (((Get-Date) - $t0).TotalMinutes -lt $TimeoutMin) {
        if ($p.WaitForExit(10000)) { break }
        $fl = Get-Content "$dir\flow.log" -Raw -ErrorAction SilentlyContinue
        if ($fl -match "(?s)PLACE_INSPECT.*sim result") { $placed = $true; if ($row.stop -eq "place") { Start-Sleep 15; break } }
        if ($fl -match "navigation result (FAILED|ABORTED|CANCELED)|\[flow\] error|PLACE_INSPECT.*\n.*sim result FAILED") { Start-Sleep 20; break }
        if ((Get-Content "$dir\isaac.log" -Raw -Encoding UTF8 -ErrorAction SilentlyContinue) -match "Pallet_01 완료") { Start-Sleep 20; break }
      }
      if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue }
      Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*run_with_monitor.py*' -or $_.CommandLine -like '*ros_tcp_relay*' -or $_.CommandLine -like '*--yolo-worker*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
      Get-Process kit -ErrorAction SilentlyContinue | Where-Object { $_.Path -like 'D:\isaacsim*' } | Stop-Process -Force -ErrorAction SilentlyContinue
      $log = Get-Content "$dir\isaac.log" -Raw -Encoding UTF8 -ErrorAction SilentlyContinue
      $finished = $log -match "Pallet_01 완료"
      $judged = $log -match "Pallet_01 판정[^\r\n]*=(green|yellow|brown)"
      $valid = if ($row.stop -eq "place") { $true } else { [bool]($finished -and $judged) }
      $row | Select-Object * | Add-Member -PassThru -NotePropertyMembers @{ rep = $rep; try = $try; placed = $placed; finished = [bool]$finished;
          valid = $valid; wall_min = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1) } | ConvertTo-Json -Compress | Set-Content "$dir\done.json" -Encoding UTF8
      "end $dir placed=$placed finished=$finished valid=$valid $([math]::Round(((Get-Date) - $t0).TotalMinutes,1)) min"
      if ($valid) { break }
    }
  }
}
