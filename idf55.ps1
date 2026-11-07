param([Parameter(Mandatory = $true)][string]$Dir, [Parameter(ValueFromRemainingArguments = $true)][string[]]$IdfArgs)
# Ejecuta idf.py (ESP-IDF 5.5) dentro de una carpeta de proyecto. Ej.: .\idf55.ps1 "S3 Robot" build ; .\idf55.ps1 "LIDAR System\p4_firmware" -p COM9 flash
$ErrorActionPreference = 'Stop'
# Detecta ESP-IDF 5.5.x: usa IDF_PATH/IDF_TOOLS_PATH si ya estan definidos; si no, busca en las rutas habituales.
if (-not $env:IDF_PATH -or -not (Test-Path $env:IDF_PATH)) {
    $env:IDF_PATH = (Get-ChildItem 'C:\Espressif\frameworks\esp-idf-v5.5*' -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1).FullName
}
if (-not $env:IDF_PATH) { throw 'No se encontro ESP-IDF 5.5 en C:\Espressif\frameworks (define IDF_PATH).' }
if (-not $env:IDF_TOOLS_PATH -or -not (Get-ChildItem "$env:IDF_TOOLS_PATH\python_env\idf5.5_py*_env" -Directory -ErrorAction SilentlyContinue)) {
    $env:IDF_TOOLS_PATH = @('C:\Espressif\tools', "$env:USERPROFILE\.espressif") | Where-Object { Get-ChildItem "$_\python_env\idf5.5_py*_env" -Directory -ErrorAction SilentlyContinue } | Select-Object -First 1
}
if (-not $env:IDF_TOOLS_PATH) { throw 'No se encontro el entorno Python de ESP-IDF 5.5 (define IDF_TOOLS_PATH).' }
$py = (Get-ChildItem "$env:IDF_TOOLS_PATH\python_env\idf5.5_py*_env\Scripts\python.exe" | Select-Object -First 1).FullName
$ErrorActionPreference = 'Continue'   # Windows PowerShell 5.1 convierte los avisos por stderr en errores
$exports = & $py "$env:IDF_PATH\tools\idf_tools.py" export --format key-value 2>$null
$ErrorActionPreference = 'Stop'
foreach ($l in $exports) {
    if ($l -match '^([A-Za-z_][A-Za-z_0-9]*)=(.*)$') {
        $v = $Matches[2].Replace('%PATH%', $env:PATH).Replace('$PATH', $env:PATH)
        [Environment]::SetEnvironmentVariable($Matches[1], $v, 'Process')
    }
}
Set-Location (Join-Path $PSScriptRoot $Dir)
if (-not $IdfArgs) { $IdfArgs = @('build') }
# Perfil rapido de la camara OV3660 (~46 FPS en vez de ~18): se aplica al driver descargado antes de compilar.
if (Test-Path 'tools\patch_ov3660_pll.py') {
    $ErrorActionPreference = 'Continue'
    & $py 'tools\patch_ov3660_pll.py' 2>$null
    if ($LASTEXITCODE -eq 1) {   # primera vez: el driver aun no se descargo
        & $py "$env:IDF_PATH\tools\idf.py" reconfigure | Out-Null
        & $py 'tools\patch_ov3660_pll.py'
    }
    if ($LASTEXITCODE -gt 1 -or $LASTEXITCODE -eq 1) { throw 'No se pudo aplicar el parche del OV3660' }
    $ErrorActionPreference = 'Stop'
}
& $py "$env:IDF_PATH\tools\idf.py" @IdfArgs
exit $LASTEXITCODE
