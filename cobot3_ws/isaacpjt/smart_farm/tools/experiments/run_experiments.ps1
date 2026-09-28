# 사업용 시뮬레이션 실험 일괄 실행 (무인). 결과: $Root\<run>\..., 진행 기록: $Root\progress.log
#   powershell -NoProfile -ExecutionPolicy Bypass -File run_experiments.ps1 [-Root DIR] [-Only exp0,exp1,exp2,exp3]
# 실험 0: 속도 0.3/0.45/0.6 기본 경로 1회씩 -> 판정(analyze.py gate) 통과 속도만
# 실험 1: 통과 속도 x {기본 +1회, 차선 2회}
# 실험 2: 사람+차선 0.3 m/s 2회
# 실험 3: 스테이션 단독(09) 불량 패턴 17회
param([string]$Root = "D:\smartfarm-sim\out\exp_20260926", [string]$Only = "exp0,exp1,exp2,exp3",
      [string]$Speeds = "")          # 판정 대신 실험 1 속도를 직접 지정 (예: "0.45,0.6")
$ErrorActionPreference = "Continue"
$Tool = Join-Path $PSScriptRoot "..\allinone_live\run_allinone.ps1"
$Scene = Join-Path $PSScriptRoot "..\..\scenes\Collected_smartfarm_v014\Collected_smartfarm_v014_room_core_cabbage.usd"
$Py = "D:\isaacsim\kit\python\python.exe"
$Here = $PSScriptRoot
New-Item -ItemType Directory -Force $Root | Out-Null
function Log($m) { $line = "$(Get-Date -Format 'MM-dd HH:mm:ss') $m"; Add-Content -Path "$Root\progress.log" -Value $line -Encoding UTF8; $line }

function Kill-Sim {
    Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like '*run_with_monitor.py*' -or $_.CommandLine -like '*ros_tcp_relay*' -or $_.CommandLine -like '*station_test.py*' -or $_.CommandLine -like '*--yolo-worker*' } |
        ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    Get-Process kit -ErrorAction SilentlyContinue | Where-Object { $_.Path -like 'D:\isaacsim*' } | Stop-Process -Force -ErrorAction SilentlyContinue
}

function Run-AllInOne([string]$name, [hashtable]$meta, [string[]]$extra) {
    $dir = "$Root\$name"
    if (Test-Path "$dir\done.json") { Log "skip $name (done)"; return }
    New-Item -ItemType Directory -Force $dir | Out-Null
    ($meta | ConvertTo-Json -Compress) | Set-Content -Path "$dir\run_meta.json" -Encoding UTF8
    Log "start $name $($extra -join ' ')"
    $t0 = Get-Date
    $argList = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $Tool, "-OutDir", $dir) + $extra
    $p = Start-Process -FilePath powershell.exe -ArgumentList $argList -PassThru -WindowStyle Hidden `
         -RedirectStandardOutput "$dir\runner.out" -RedirectStandardError "$dir\runner.err"
    $ok = $false; $failed = $false
    while (((Get-Date) - $t0).TotalMinutes -lt 35) {
        if ($p.WaitForExit(20000)) { $ok = $true; break }
        if ((Get-Content "$dir\flow.log" -Raw -ErrorAction SilentlyContinue) -match "FAILED|\[flow\] error") {   # 흐름 실패면 선별 완료를 30분 기다리지 않는다
            $failed = $true; Start-Sleep 30; break
        }
    }
    if (-not $ok) { Log "STOP $name (flow_failed=$failed) -> kill"; Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue }
    $done = (Get-Content "$dir\isaac.log" -Raw -Encoding UTF8 -ErrorAction SilentlyContinue) -match "Pallet_01 완료"
    @{ finished = [bool]$done; timeout = (-not $ok); wall_min = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1) } |
        ConvertTo-Json -Compress | Set-Content -Path "$dir\done.json" -Encoding UTF8
    Log "end $name finished=$done timeout=$(-not $ok) $([math]::Round(((Get-Date) - $t0).TotalMinutes,1)) min"
    Kill-Sim
    Start-Sleep 5
}

$steps = $Only.Split(",")
if ($steps -contains "exp0") {
    foreach ($v in "0.3", "0.45", "0.6") {
        Run-AllInOne "e0_default_v$v" @{ exp = "exp0"; route = "default"; speed = [double]$v; human = $false; rep = 1 } @("-Speed", $v, "-NoRviz")
    }
}
$pass = @("0.3")
if ($steps -contains "exp1" -or $steps -contains "exp2") {
    $g = (& $Py "$Here\analyze.py" gate $Root) -join " "      # 여러 줄 -> 한 문자열 (배열에 -match 하면 $Matches 가 안 채워진다)
    Log "gate: $g"
    if ($g -match "PASS=([0-9.,]*)") { $pass = @($Matches[1].Split(",") | Where-Object { $_ }) }
    if ($Speeds) { $pass = @($Speeds.Split(",")) }
    if (-not ($pass -contains "0.3")) { $pass = @("0.3") + $pass }   # 기준 속도는 항상 포함
}
if ($steps -contains "exp1") {
    foreach ($v in $pass) {
        Run-AllInOne "e1_default_v${v}_r2" @{ exp = "exp1"; route = "default"; speed = [double]$v; human = $false; rep = 2 } @("-Speed", $v, "-NoRviz")
        foreach ($r in 1, 2) {
            Run-AllInOne "e1_lane_v${v}_r$r" @{ exp = "exp1"; route = "lane"; speed = [double]$v; human = $false; rep = $r } @("-Speed", $v, "-Lane", "-NoRviz")
        }
    }
}
if ($steps -contains "exp2") {
    foreach ($r in 1, 2) {
        Run-AllInOne "e2_human_lane_v0.3_r$r" @{ exp = "exp2"; route = "lane"; speed = 0.3; human = $true; rep = $r } @("-Speed", "0.3", "-Lane", "-Human", "-NoRviz")
    }
}
if ($steps -contains "exp3") {
    Kill-Sim; wsl --terminate Ubuntu-24.04 | Out-Null
    $patterns = [ordered]@{
        "d0_a" = "G,G,G,G,G,G"; "d0_b" = "G,G,G,G,G,G";
        "d1_s1" = "Y,G,G,G,G,G"; "d1_s2" = "G,B,G,G,G,G"; "d1_s3" = "G,G,Y,G,G,G"; "d1_s4" = "G,G,G,B,G,G"; "d1_s5" = "G,G,G,G,Y,G"; "d1_s6" = "G,G,G,G,G,B";
        "d2_s12" = "Y,B,G,G,G,G"; "d2_s34" = "G,G,Y,B,G,G"; "d2_s56" = "G,G,G,G,Y,B";
        "d3_s235" = "G,Y,B,G,Y,G"; "d3_s146" = "Y,G,G,B,G,Y"; "d3_s246" = "G,Y,G,B,G,Y"; "d3_s135" = "B,G,Y,G,B,G";
        "d6_a" = "Y,B,Y,B,Y,B"; "d6_b" = "B,Y,B,Y,B,Y"
    }
    foreach ($k in $patterns.Keys) {
        $dir = "$Root\e3_$k"
        if (Test-Path "$dir\result.json") { Log "skip e3_$k (done)"; continue }
        New-Item -ItemType Directory -Force $dir | Out-Null
        (@{ exp = "exp3"; pattern = $patterns[$k] } | ConvertTo-Json -Compress) | Set-Content -Path "$dir\run_meta.json" -Encoding UTF8
        Log "start e3_$k $($patterns[$k])"
        $t0 = Get-Date
        $p = Start-Process -FilePath "D:\isaacsim\python.bat" -PassThru -WindowStyle Hidden -WorkingDirectory $PSScriptRoot `
             -ArgumentList "`"$PSScriptRoot\station_test.py`"", "--scene", "`"$Scene`"", "--out", "`"$dir`"", "--pattern", $patterns[$k] `
             -RedirectStandardOutput "$dir\test.out" -RedirectStandardError "$dir\test.err"
        if (-not $p.WaitForExit(15 * 60 * 1000)) { Log "TIMEOUT e3_$k"; Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue }
        Kill-Sim
        Log "end e3_$k $([math]::Round(((Get-Date) - $t0).TotalMinutes,1)) min result=$(Test-Path "$dir\result.json")"
    }
}
Log "all done -> analyze"
& $Py "$Here\analyze.py" all $Root | ForEach-Object { Log $_ }
