param(
    [string]$Port = "COM5",
    [double]$Interval = 0.35,
    [string]$OutputDir = "artifacts"
)

$ErrorActionPreference = "Stop"
$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$output = Join-Path $OutputDir "mvp_demo_$timestamp.csv"

Write-Host "WS63 MVP demo on $Port"
$sequence = "O,E,D,T,I,T,F,S,T,B,S,T,L,S,T,R,S,E,D,T,S"
Write-Host "Keep wheels lifted. Sequence: $sequence"

python -m upper_client.robot_client `
    --transport serial-at `
    --serial-port $Port `
    --commands $sequence `
    --interval $Interval `
    --timeout 3 `
    --output $output

Write-Host "Demo CSV: $output"
