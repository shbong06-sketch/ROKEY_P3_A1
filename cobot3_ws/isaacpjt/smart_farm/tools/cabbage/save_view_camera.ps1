# Save the Isaac Sim GUI viewport's current view as a scene camera (while run_allinone_live.ps1 is running).
#   powershell -File save_view_camera.ps1 [-Name Cam0_MyView]
# aio_wrapper picks the trigger up within ~1 s of sim time, adds /World/ProcessCameras/<Name> to the running stage and
# appends it to D:\smartfarm-sim\out\saved_view_cameras.json; 06_make_cabbage_scene.py bakes it into the scene copy.
param([string]$Name = "")
Set-Content -Path "D:\smartfarm-sim\out\SAVE_VIEW_CAMERA" -Value $Name -Encoding utf8
"trigger written - the view is saved on the next second of simulation (see isaac.log: [MONITOR] viewport view saved)"
