param(
    [string]$SdkRoot = 'D:\r\src',
    [string]$HiSparkStudioRoot = 'D:\HiSpark Studio 26.03.1'
)

$ErrorActionPreference = 'Stop'
$env:PYTHONWARNINGS = 'ignore::SyntaxWarning'

$resolvedSdk = Resolve-Path -LiteralPath $SdkRoot
$repoRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$overlayRoot = Join-Path $repoRoot 'firmware\deveco_ws63_overlay'
$builtPackage = Join-Path $resolvedSdk 'output\ws63\fwpkg\ws63-liteos-app\ws63-liteos-app_load_only.fwpkg'
$releasePackage = Join-Path $repoRoot 'firmware\fwpkg\ws63-liteos-app_ws63_sle_pi_bridge_load_only.fwpkg'
$ccachePath = Join-Path $HiSparkStudioRoot 'tools\cfbb\thirdparty\ccache'
if (Test-Path -LiteralPath $ccachePath) {
    $env:PATH = "$ccachePath;$env:PATH"
}

Push-Location $resolvedSdk
try {
    if (-not (Test-Path -LiteralPath $overlayRoot)) {
        throw "WS63 overlay not found: $overlayRoot"
    }
    # The bridge is selected in Kconfig, so applying only robot_mvp/*.c would
    # silently build the local motor backend instead of the Pi JSON bridge.
    Get-ChildItem -LiteralPath $overlayRoot -Force | ForEach-Object {
        $destination = Join-Path $resolvedSdk $_.Name
        if ($_.PSIsContainer) {
            New-Item -ItemType Directory -Force -Path $destination | Out-Null
            Get-ChildItem -LiteralPath $_.FullName -Force | ForEach-Object {
                Copy-Item -LiteralPath $_.FullName -Destination $destination -Recurse -Force
            }
        } else {
            Copy-Item -LiteralPath $_.FullName -Destination $destination -Force
        }
    }
    # build.py refreshes Kconfig headers before compiling. Keeping the existing
    # CMake directory avoids a Windows SDK clean-build cache defect that can
    # leave Ninja without its generated rules file.
    python build.py ws63-liteos-app
    if (-not (Test-Path -LiteralPath $builtPackage)) {
        throw "Expected WS63 package was not produced: $builtPackage"
    }
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $releasePackage) | Out-Null
    Copy-Item -LiteralPath $builtPackage -Destination $releasePackage -Force
    python tools\pkg\packet_create.py -show $builtPackage
    Get-FileHash -Algorithm SHA256 -LiteralPath $releasePackage
}
finally {
    Pop-Location
}
