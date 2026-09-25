# Windows ("고피") side of the real Nav2 test: Isaac Sim GUI + team standalone_app (+ cabbage monitor), DDS to WSL.
#   powershell -File run_isaac_for_nav2.ps1 -Scene <usd> -OutDir <dir>
# GUI is required: headless steps do not tick the ROS OmniGraphs (/clock, odom, lidar would be silent).
param([string]$Scene, [string]$OutDir,
      [string]$Root = "D:\smartfarm-sim\team_ref\allinone\cobot3_ws\isaacpjt\smart_farm")
New-Item -ItemType Directory -Force $OutDir | Out-Null
$j = "D:\isaacsim\exts\isaacsim.ros2.bridge\jazzy"
$env:SMARTFARM_ROS_REEXEC = "1"; $env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"; $env:ROS_DISTRO = "jazzy"
$env:ROS_DOMAIN_ID = "101"                    # team guidance2_25
[Environment]::SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", $null)   # Windows-local DDS; WSL is reached via ros_tcp_relay.py
$env:PATH = "$j\lib;" + $env:PATH; $env:PYTHONUNBUFFERED = "1"
$env:CABBAGE_MONITOR_OUT = "$OutDir\monitor.json"
$log = "$OutDir\isaac.log"
Start-Process -FilePath "D:\isaacsim\python.bat" -NoNewWindow -RedirectStandardOutput $log -RedirectStandardError "$log.err" `
  -ArgumentList "D:\smartfarm-sim\scripts\cabbage\aio_wrapper.py `"$Root\runtime\standalone_app.py`" --scene `"$Scene`" --autoplay"
$t0 = Get-Date
while (-not ((Get-Content $log -Raw -Encoding UTF8 -ErrorAction SilentlyContinue) -match "\[READY\]")) {
    Start-Sleep 5; if (((Get-Date) - $t0).TotalMinutes -gt 15) { "Isaac did not become READY"; exit 1 }
}
"Isaac READY after $([int]((Get-Date) - $t0).TotalSeconds) s (log $log)"
$env:PYTHONPATH = "$j\rclpy;D:\smartfarm-sim\team_ref\pylib"
Start-Process -FilePath "D:\isaacsim\kit\python\python.exe" -NoNewWindow -RedirectStandardOutput "$OutDir\relay_win.log" -RedirectStandardError "$OutDir\relay_win.err" `
  -ArgumentList "D:\smartfarm-sim\scripts\wsl\ros_tcp_relay.py","win"
"relay (Windows side) started -> connects to the WSL relay on 127.0.0.1:47100"
