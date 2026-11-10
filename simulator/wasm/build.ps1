<#
  Compila el firmware (vision del S3 + mapa/controlador del P4) a WebAssembly
  para que el simulador del navegador ejecute el codigo REAL.

  Necesita clang con destino wasm32-wasi. Se busca, en este orden:
    1. -WasiSdk <carpeta>
    2. la variable de entorno WASI_SDK_PATH
    3. simulator\.toolchain\wasi-sdk-*   (se descarga aqui con -Download)

  El resultado (simulator\web\wro_core.wasm) esta versionado: quien no cambia el
  firmware no necesita el SDK.
#>
param(
    [string]$WasiSdk = $env:WASI_SDK_PATH,
    [switch]$Download
)
$ErrorActionPreference = 'Stop'
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$toolchain = Join-Path (Split-Path $PSScriptRoot -Parent) '.toolchain'

if (-not $WasiSdk) {
    $found = Get-ChildItem -Path $toolchain -Directory -Filter 'wasi-sdk-*' -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($found) { $WasiSdk = $found.FullName }
}
if (-not $WasiSdk -and $Download) {
    $version = '25'
    $archive = Join-Path $toolchain 'wasi-sdk.tar.gz'
    New-Item -ItemType Directory -Force -Path $toolchain | Out-Null
    $url = "https://github.com/WebAssembly/wasi-sdk/releases/download/wasi-sdk-$version/wasi-sdk-$version.0-x86_64-windows.tar.gz"
    Write-Host "Descargando wasi-sdk $version (unos 500 MB) desde $url"
    & curl.exe -L -o $archive $url
    if ($LASTEXITCODE -ne 0) { throw 'La descarga fallo' }
    # No basta con `tar.exe`: si el PATH resuelve al tar de Git (MSYS), interpreta
    # "C:\..." como un host remoto y falla con "Cannot connect to C". Se usa el de Windows.
    $tar = Join-Path $env:SystemRoot 'System32\tar.exe'
    if (-not (Test-Path $tar)) { $tar = 'tar.exe' }
    & $tar -xzf $archive -C $toolchain
    if ($LASTEXITCODE -ne 0) { throw 'No se pudo extraer el SDK' }
    Remove-Item $archive
    $WasiSdk = (Get-ChildItem -Path $toolchain -Directory -Filter 'wasi-sdk-*' | Select-Object -First 1).FullName
}
if (-not $WasiSdk) {
    throw 'No hay wasi-sdk. Pasa -WasiSdk <carpeta>, define WASI_SDK_PATH o ejecuta con -Download.'
}
$clang = Join-Path $WasiSdk 'bin\clang.exe'
if (-not (Test-Path $clang)) { throw "No existe $clang" }

$lidar = Join-Path $root 'LIDAR System\main'
$orchestrator = Join-Path $root 'LIDAR System\orchestrator'
$vision = Join-Path $root 'Vision System\main'
$shared = Join-Path $root 'shared'
$mic = Join-Path $root 'Motor Control\MIC'
$sources = @(
    (Join-Path $PSScriptRoot 'core_api.c'),
    (Join-Path $orchestrator 'p4_runtime.c'),
    (Join-Path $lidar 'park_ackermann.c'), (Join-Path $mic 'mic.c'),
    (Join-Path $lidar 'trace_map.c'), (Join-Path $lidar 'trace_parking.c'), (Join-Path $lidar 'local_obstacle_map.c'), (Join-Path $lidar 'camera_table_receiver.c'),
    (Join-Path $lidar 'imu_yaw.c'), (Join-Path $orchestrator 'wro_controller.c'), (Join-Path $lidar 'park_planner.c'), (Join-Path $lidar 'park_localize.c'), (Join-Path $lidar 'odometry.c'), (Join-Path $lidar 'race_line.c'), (Join-Path $lidar 'it_plan.c'), (Join-Path $lidar 'arena_localizer.c'),
    (Join-Path $vision 'vision.c'), (Join-Path $vision 'vision_table.c'),
    (Join-Path $shared 'sensor_table_protocol.c'), (Join-Path $shared 'config_fields.c')
)
$output = Join-Path (Split-Path $PSScriptRoot -Parent) 'web\wro_core.wasm'
New-Item -ItemType Directory -Force -Path (Split-Path $output -Parent) | Out-Null
$flags = @('--target=wasm32-wasi', '-O2', '-std=c11', '-Wall', '-Wextra', '-Werror', '-nostartfiles',
    '-Wl,--no-entry', '-Wl,-z,stack-size=262144', '-Wl,--strip-all',
    '-I', $lidar, '-I', $orchestrator, '-I', $vision, '-I', $shared, '-I', $mic)
& $clang @flags @sources -o $output -lm
if ($LASTEXITCODE -ne 0) { throw 'La compilacion a WebAssembly fallo' }
& python (Join-Path $PSScriptRoot 'manifest.py') --write
if ($LASTEXITCODE -ne 0) { throw 'No se pudo registrar el manifiesto WASM' }
Write-Host ("OK: {0} ({1:N0} bytes)" -f $output, (Get-Item $output).Length)
