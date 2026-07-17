param(
    [string]$SdkRoot = 'D:\b\src',
    [string]$HiSparkStudioRoot = 'D:\HiSpark Studio 26.03.1'
)

$ErrorActionPreference = 'Stop'

$resolvedSdk = Resolve-Path -LiteralPath $SdkRoot
$ccachePath = Join-Path $HiSparkStudioRoot 'tools\cfbb\thirdparty\ccache'
if (Test-Path -LiteralPath $ccachePath) {
    $env:PATH = "$ccachePath;$env:PATH"
}

Push-Location $resolvedSdk
try {
    python build.py ws63-liteos-app
    python tools\pkg\packet_create.py -show output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg
}
finally {
    Pop-Location
}
