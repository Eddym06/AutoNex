$ErrorActionPreference = 'Stop'
$visionRoot = Split-Path $PSScriptRoot -Parent
if (-not $env:IDF_PATH) { $env:IDF_PATH = 'C:\Espressif\frameworks\esp-idf-v5.3.1' }
if (-not $env:IDF_TOOLS_PATH) { $env:IDF_TOOLS_PATH = 'C:\Espressif' }
$visionPython = Join-Path $env:IDF_TOOLS_PATH 'python_env\idf5.3_py3.11_env\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $visionPython)) { throw "ESP-IDF Python missing: $visionPython" }
$visionExports = & $visionPython (Join-Path $env:IDF_PATH 'tools\idf_tools.py') export --format key-value
if ($LASTEXITCODE -ne 0) { throw 'Cannot activate ESP-IDF tools' }
foreach ($visionLine in $visionExports) {
    if ($visionLine -match '^([A-Za-z_][A-Za-z_0-9]*)=(.*)$') {
        $visionName = $Matches[1]; $visionValue = $Matches[2]
        if ($visionName -eq 'PATH') {
            $visionValue = $visionValue.Replace('%PATH%', $env:PATH).Replace('$PATH', $env:PATH)
        }
        [Environment]::SetEnvironmentVariable($visionName, $visionValue, 'Process')
    }
}
$env:IDF_COMPONENT_CACHE_PATH = Join-Path $visionRoot 'build\component-cache'
$env:CCACHE_DIR = Join-Path $visionRoot 'build\ccache'
$env:CCACHE_TEMPDIR = Join-Path $visionRoot 'build\ccache-tmp'
function idf.py { & $visionPython (Join-Path $env:IDF_PATH 'tools\idf.py') @args }
Set-Location -LiteralPath $visionRoot
