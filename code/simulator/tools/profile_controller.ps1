param([string]$Output = "$env:TEMP\autonex-profile.exe")
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$main = Join-Path $root 'LIDAR System/main'
$orch = Join-Path $root 'LIDAR System/orchestrator'
$shared = Join-Path $root 'shared'
$src = @('park_ackermann.c','trace_map.c','trace_parking.c','local_obstacle_map.c','camera_table_receiver.c','imu_yaw.c','park_planner.c','park_localize.c','odometry.c','arena_localizer.c','race_line.c') | ForEach-Object { Join-Path $main $_ }
$src += @((Join-Path $orch 'wro_controller.c'), (Join-Path $shared 'sensor_table_protocol.c'), (Join-Path $shared 'config_fields.c'), (Join-Path $PSScriptRoot 'profile_stages.c'), (Join-Path $PSScriptRoot 'profile_controller.c'))
$wrap = @('local_obstacle_map_update_cfg','local_obstacle_map_recommend_cfg','park_localize_track','park_ackermann_plan','arena_acquire','arena_update') | ForEach-Object { "-Wl,--wrap=$_" }
& gcc -O2 -std=c11 -I $main -I $orch -I $shared @src @wrap -o $Output -lm
if ($LASTEXITCODE -ne 0) { throw 'Fallo la compilacion del perfilador' }
& $Output
if ($LASTEXITCODE -ne 0) { throw 'Fallo el perfilador' }
