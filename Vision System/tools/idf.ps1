param([Parameter(ValueFromRemainingArguments = $true)][string[]]$IdfArguments)
Push-Location
try {
    . (Join-Path $PSScriptRoot 'activate.ps1')
    & $visionPython (Join-Path $PSScriptRoot 'patch_ov3660_pll.py')
    if ($LASTEXITCODE -ne 0) { throw 'Could not apply the tested OV3660 high-FPS profile' }
    if (-not $IdfArguments) { $IdfArguments = @('build') }
    idf.py @IdfArguments
    $visionExitCode = $LASTEXITCODE
} finally { Pop-Location }
exit $visionExitCode
