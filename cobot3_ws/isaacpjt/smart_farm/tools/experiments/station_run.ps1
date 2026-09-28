param([string]$Root = "D:\smartfarm-sim\out\exp_20260928_station", [string[]]$Specs = @(), [int]$MaxTries = 3)
# 스테이션 단독 시험 묶음. Spec = "이름|인자|환경변수(K=V;K=V)". 비전 판정이 전부 UNKNOWN(검은 화면)이면 무효, 다시 돌림.
#   station_test.py (같은 폴더) 를 Isaac 파이썬으로 실행. 예: -Specs @("f12_m0.5|--pattern B,Y,B,Y,B,Y --truth-labels --head-mass 0.5|SMARTFARM_CULL_RETRY=0")
$Scene = Join-Path $PSScriptRoot "..\..\scenes\Collected_smartfarm_v014\Collected_smartfarm_v014_room_core_cabbage.usd"
New-Item -ItemType Directory -Force $Root | Out-Null
$EnvKeys = "SMARTFARM_VISION_ONLY","SMARTFARM_GRIP_FORCE","SMARTFARM_CULL_RETRY","SMARTFARM_AIM_ABOVE_TRAY","SMARTFARM_VISION_DEADBAND","SMARTFARM_VISION_REFRESH"
foreach ($spec in $Specs) {
  $name, $argStr, $envStr = $spec.Split('|')
  foreach ($k in $EnvKeys) { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
  if ($envStr) { foreach ($kv in $envStr.Split(';')) { $k, $v = $kv.Split('='); Set-Item "Env:$k" $v } }
  for ($try = 1; $try -le $MaxTries; $try++) {
    $dir = "$Root\$name" + $(if ($try -gt 1) { "_try$try" } else { "" })
    if (Test-Path "$dir\done.json") { if ((Get-Content "$dir\done.json" -Raw) -match '"valid":\s*true') { "skip $dir"; break } else { continue } }
    New-Item -ItemType Directory -Force $dir | Out-Null
    $t0 = Get-Date
    $al = @("`"$PSScriptRoot\station_test.py`"", "--scene", "`"$Scene`"", "--out", "`"$dir`"") + $argStr.Split(' ', [StringSplitOptions]::RemoveEmptyEntries)
    $p = Start-Process -FilePath "D:\isaacsim\python.bat" -PassThru -WindowStyle Hidden -WorkingDirectory $PSScriptRoot -ArgumentList $al -RedirectStandardOutput "$dir\test.out" -RedirectStandardError "$dir\test.err"
    if (-not $p.WaitForExit(20 * 60 * 1000)) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue; "TIMEOUT $dir" }
    Get-Process kit -ErrorAction SilentlyContinue | Where-Object { $_.Path -like 'D:\isaacsim*' } | Stop-Process -Force -ErrorAction SilentlyContinue
    Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*--yolo-worker*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    $out = Get-Content "$dir\test.out" -Raw -Encoding UTF8 -ErrorAction SilentlyContinue
    $judged = $out -match "Pallet_Test 판정[^\r\n]*=(green|yellow|brown)"
    $valid = (Test-Path "$dir\result.json") -and ($judged -or ($argStr -match "--truth-labels"))
    @{ name = $name; args = $argStr; env = $envStr; try = $try; valid = [bool]$valid; wall_min = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1) } |
      ConvertTo-Json -Compress | Set-Content "$dir\done.json" -Encoding UTF8
    "end $dir valid=$valid $([math]::Round(((Get-Date) - $t0).TotalMinutes,1)) min"
    if ($valid) { break }
  }
}
