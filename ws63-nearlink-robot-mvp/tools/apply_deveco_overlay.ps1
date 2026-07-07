param(
    [Parameter(Mandatory = $true)]
    [string]$SdkRoot
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$overlayRoot = Join-Path $repoRoot 'firmware\deveco_ws63_overlay'

if (-not (Test-Path -LiteralPath $overlayRoot)) {
    throw "Overlay not found: $overlayRoot"
}

$resolvedSdk = Resolve-Path -LiteralPath $SdkRoot
if (-not (Test-Path -LiteralPath (Join-Path $resolvedSdk 'application'))) {
    throw "SdkRoot does not look like fbb_bs2x SDK root: $resolvedSdk"
}

Write-Host "Applying overlay from $overlayRoot"
Write-Host "SDK root: $resolvedSdk"

Copy-Item -LiteralPath (Join-Path $overlayRoot 'application\*') -Destination (Join-Path $resolvedSdk 'application') -Recurse -Force
Copy-Item -LiteralPath (Join-Path $overlayRoot 'build\*') -Destination (Join-Path $resolvedSdk 'build') -Recurse -Force

if (Test-Path -LiteralPath (Join-Path $overlayRoot 'tools')) {
    Copy-Item -LiteralPath (Join-Path $overlayRoot 'tools\*') -Destination (Join-Path $resolvedSdk 'tools') -Recurse -Force
}

Write-Host 'Overlay applied.'
