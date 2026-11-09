# Compila y ejecuta todas las pruebas: C nativo (vision, enlace, mapa, planificador,
# giroscopio), el simulador (Node) y las secuencias de imagenes (Python).
#   .\tools\simulate.ps1            todo (las rondas completas tardan unos minutos)
#   .\tools\simulate.ps1 -Quick     sin las rondas completas del simulador
#   .\tools\simulate.ps1 -Wasm      recompila el firmware a WebAssembly antes (necesita wasi-sdk)
param([switch]$Quick, [switch]$Wasm, [string]$WasiSdk = $env:WASI_SDK_PATH)
$ErrorActionPreference = 'Stop'
$visionRoot = Split-Path $PSScriptRoot -Parent
$repoRoot = Split-Path $visionRoot -Parent
$build = Join-Path $visionRoot 'build\simulation'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$lidar = Join-Path $repoRoot 'LIDAR System'
$shared = Join-Path $repoRoot 'shared'
$vm = Join-Path $visionRoot 'main'
$flags = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-Wpedantic', '-O2')

function Invoke-CTest([string]$name, [string[]]$arguments) {
    $exe = Join-Path $build "$name.exe"
    & gcc @flags @arguments -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "${name}: la compilacion fallo" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "${name}: las pruebas fallaron" }
}

Invoke-CTest 'vision' @('-I', $vm, '-I', $shared, "$vm\vision.c", "$shared\config_fields.c", "$shared\sensor_table_protocol.c", "$visionRoot\tests\test_vision.c")
Invoke-CTest 'vision_config_cmd' @('-I', $vm, '-I', $shared, "$vm\vision.c", "$vm\vision_config_cmd.c", "$shared\config_fields.c", "$shared\sensor_table_protocol.c", "$visionRoot\tests\test_vision_config_cmd.c")
Invoke-CTest 'vision_table' @('-I', $vm, '-I', $shared, "$vm\vision.c", "$vm\vision_table.c", "$shared\config_fields.c", "$shared\sensor_table_protocol.c", "$visionRoot\tests\test_vision_table.c")
Invoke-CTest 'lidar_map' @('-I', "$lidar\main", "$lidar\main\local_obstacle_map.c", "$lidar\tests\test_local_obstacle_map.c")
Invoke-CTest 'planner_fusion' @('-I', "$lidar\main", "$lidar\main\local_obstacle_map.c", "$lidar\tests\test_planner_fusion.c")
Invoke-CTest 'camera_link' @('-I', "$lidar\main", '-I', "$lidar\orchestrator", '-I', $shared, "$lidar\main\camera_table_receiver.c", "$lidar\tests\test_camera_table_receiver.c", "$shared\sensor_table_protocol.c", "$lidar\main\local_obstacle_map.c")
Invoke-CTest 'imu_yaw' @('-I', "$lidar\main", "$lidar\main\imu_yaw.c", "$lidar\tests\test_imu_yaw.c")
Invoke-CTest 'odometry' @('-I', "$lidar\main", "$lidar\main\odometry.c", "$lidar\tests\test_odometry.c")
Invoke-CTest 'axle_encoder' @('-I', $shared, "$shared\test_axle_encoder.c")
Invoke-CTest 'arena_localizer' @('-I', "$lidar/main", "$lidar/main/arena_localizer.c", "$lidar/tests/test_arena_localizer.c")
Invoke-CTest 'race_line' @('-I', "$lidar/main", "$lidar/main/race_line.c", "$lidar/tests/test_race_line.c")
Invoke-CTest 'park_ackermann' @('-I', "$lidar\main", "$lidar\main\park_ackermann.c", "$lidar\main\park_planner.c", "$lidar\tests\test_park_ackermann.c")
Invoke-CTest 'park_planner' @('-I', "$lidar\main", "$lidar\main\park_planner.c", "$lidar\tests\test_park_planner.c")
Invoke-CTest 'wro_controller' @('-I', "$lidar\main", '-I', "$lidar\orchestrator", '-I', $shared, "$lidar\orchestrator\wro_controller.c", "$lidar\main\park_ackermann.c", "$lidar\main\park_planner.c", "$lidar\main\park_localize.c", "$lidar\main\odometry.c", "$lidar\main\arena_localizer.c", "$lidar\main\race_line.c", "$lidar\main\imu_yaw.c", "$lidar\main\local_obstacle_map.c", "$lidar\main\camera_table_receiver.c", "$shared\sensor_table_protocol.c", "$shared\config_fields.c", "$lidar\tests\test_wro_controller.c")
Invoke-CTest 'p4_runtime' @('-I', "$lidar\main", '-I', "$lidar\orchestrator", '-I', $shared, "$lidar\orchestrator\p4_runtime.c", "$lidar\orchestrator\wro_controller.c", "$lidar\main\park_ackermann.c", "$lidar\main\park_planner.c", "$lidar\main\park_localize.c", "$lidar\main\odometry.c", "$lidar\main\arena_localizer.c", "$lidar\main\race_line.c", "$lidar\main\imu_yaw.c", "$lidar\main\local_obstacle_map.c", "$lidar\main\camera_table_receiver.c", "$shared\sensor_table_protocol.c", "$shared\config_fields.c", "$lidar\tests\test_p4_runtime.c")
# Arranque de los motores con el codigo real del P4 (driver LEDC sustituido): gnu11 porque hw_motors.c usa M_PI de newlib.
$motorSrc = @("$lidar\orchestrator\p4_runtime.c", "$lidar\orchestrator\wro_controller.c", "$lidar\main\park_ackermann.c", "$lidar\main\park_planner.c", "$lidar\main\park_localize.c", "$lidar\main\odometry.c", "$lidar\main\arena_localizer.c", "$lidar\main\race_line.c", "$lidar\main\imu_yaw.c", "$lidar\main\local_obstacle_map.c", "$lidar\main\camera_table_receiver.c", "$shared\sensor_table_protocol.c", "$shared\config_fields.c")
& gcc -std=gnu11 -Wall -Wextra -Wno-unused-variable -O2 -I"$lidar\tests\esp_stub" -I"$lidar\main" -I"$lidar\orchestrator" -I"$shared" -I"$lidar\p4_firmware\main" -o (Join-Path $build "motor_start.exe") "$lidar\tests\test_motor_start.c" "$lidar\tests\esp_stub\ledc_stub.c" "$lidar\p4_firmware\main\hw_motors.c" @motorSrc -lm
if ($LASTEXITCODE -ne 0) { throw 'motor_start: la compilacion fallo' }
& (Join-Path $build 'motor_start.exe')
if ($LASTEXITCODE -ne 0) { throw 'motor_start: las pruebas fallaron' }
# Prueba de motores desde la pagina (M1, M2, secuencia y parada).
& gcc -std=c11 -Wall -Wextra -Werror -I"$lidar\main" -o (Join-Path $build 'motor_test.exe') "$lidar\tests\test_motor_test.c" "$lidar\main\motor_test.c"
if ($LASTEXITCODE -ne 0) { throw 'motor_test: la compilacion fallo' }
& (Join-Path $build 'motor_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'motor_test: las pruebas fallaron' }

if ($Wasm) {
    & (Join-Path $repoRoot 'simulator\wasm\build.ps1') -WasiSdk $WasiSdk
}
& python (Join-Path $repoRoot 'simulator\wasm\manifest.py')
if ($LASTEXITCODE -ne 0) { throw 'El WASM no corresponde a las fuentes; ejecuta simulate.ps1 -Wasm.' }
# El firmware que corre en el simulador es el .wasm versionado: avisa si el C es mas nuevo.
$wasmFile = Join-Path $repoRoot 'simulator\web\wro_core.wasm'
$newest = Get-ChildItem -Path $lidar\main, $lidar\orchestrator, $vm, $shared -Include *.c, *.h -Recurse | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if ($newest.LastWriteTime -gt (Get-Item $wasmFile).LastWriteTime) {
    Write-Warning "wro_core.wasm es anterior a $($newest.Name): el simulador ejecuta firmware viejo. Ejecuta simulate.ps1 -Wasm."
}

& node (Join-Path $repoRoot 'simulator\tests\test_sim_basics.js')
if ($LASTEXITCODE -ne 0) { throw 'Pruebas del mundo simulado fallaron' }
& node (Join-Path $repoRoot 'simulator\tests\test_contrato.js')
if ($LASTEXITCODE -ne 0) { throw 'El contrato simulacion-firmware falló' }
& node (Join-Path $repoRoot 'simulator\tests\test_axle_encoder.js')
if ($LASTEXITCODE -ne 0) { throw 'Prueba del encoder simulado falló' }
& node (Join-Path $repoRoot 'simulator\tests\test_parking_exit.js')
if ($LASTEXITCODE -ne 0) { throw 'La salida del cajon falló' }
& node (Join-Path $repoRoot 'simulator\tests\test_parking_entry.js')
if ($LASTEXITCODE -ne 0) { throw 'La entrada del cajon falló' }
& node (Join-Path $repoRoot 'simulator\tests\test_ui_smoke.js')
if ($LASTEXITCODE -ne 0) { throw 'La interfaz no arranca' }
if (-not $Quick) {
    & node (Join-Path $repoRoot 'simulator\tests\test_missions.js')
    if ($LASTEXITCODE -ne 0) { throw 'Las rondas completas fallaron' }
}
# El depurador (python) sobre un plano que debe salir limpio: cualquier hallazgo hace fallar la prueba.
& python (Join-Path $repoRoot 'simulator\tools\depurar.py') (Join-Path $repoRoot 'simulator\tests\plano_corregido.json') --sin-mapas | Select-Object -Last 6
if ($LASTEXITCODE -ne 0) { throw 'El depurador encontro problemas en simulator\tests\plano_corregido.json' }
& python (Join-Path $repoRoot 'simulator\sequence_simulation.py')
if ($LASTEXITCODE -ne 0) { throw 'Las secuencias de vision fallaron; revisa simulator\build\sequence-simulation\report.html' }
& python (Join-Path $repoRoot 'simulator\build_simulator.py')
if ($LASTEXITCODE -ne 0) { throw 'No se pudo generar simulator\build\simulador.html' }
Write-Host 'TODO EN VERDE'
