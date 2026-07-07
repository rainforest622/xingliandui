param(
    [string]$Port = 'COM5',
    [string]$HiSparkStudioRoot = 'D:\HiSpark Studio 26.03.1',
    [string]$Package = ''
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
if ($Package -eq '') {
    $Package = Join-Path $repoRoot 'firmware\fwpkg\ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg'
}

$burnTool = Join-Path $HiSparkStudioRoot 'tools\BurnToolCmd\BurnToolCmd.exe'
if (-not (Test-Path -LiteralPath $burnTool)) {
    throw "BurnToolCmd not found: $burnTool"
}
if (-not (Test-Path -LiteralPath $Package)) {
    throw "Firmware package not found: $Package"
}

& $burnTool --burn -n ws63 -m serial $Port --baudRate 115200 -f $Package
