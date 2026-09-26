
#!/usr/bin/env pwsh
<#
.SYNOPSIS
Build and independently deploy rewrite/lq_path_follow.

.EXAMPLE
.\deploy_rewrite.ps1
.\deploy_rewrite.ps1 -RunMode Dry
.\deploy_rewrite.ps1 -RunMode Motors -ConfirmWheelsLifted
.\deploy_rewrite.ps1 -BuildOnly -Jobs 6
.\deploy_rewrite.ps1 -ForceRebuild
#>
[CmdletBinding()]
param(
    [string]$BoardIP = "192.168.43.220",
    [string]$BoardUser = "root",
    [ValidateSet("None", "Dry", "Motors")]
    [string]$RunMode = "None",
    [string]$WslDistribution = "Debian",
    [string]$EnvironmentRoot = (
        Join-Path $env:USERPROFILE "Downloads\lq环境配置 (2)"
    ),
    [ValidateRange(1, 32)]
    [int]$Jobs = [Math]::Min([Environment]::ProcessorCount, 6),
    [switch]$BuildOnly,
    [switch]$ForceRebuild,
    [switch]$SkipModels,
    [switch]$NoBackup,
    [switch]$ConfirmWheelsLifted,
    [string]$IdentityFile = ""
)

$ErrorActionPreference = "Stop"
$Utf8 = New-Object System.Text.UTF8Encoding($false)
if ($env:OS -eq "Windows_NT") {
    & "$env:SystemRoot\System32\chcp.com" 65001 | Out-Null
}
[Console]::InputEncoding = $Utf8
[Console]::OutputEncoding = $Utf8
$OutputEncoding = $Utf8
$env:LANG = "C.UTF-8"
$env:LC_ALL = "C.UTF-8"
$env:WSL_UTF8 = "1"

$RepoRoot = Split-Path -Parent $PSCommandPath
$Backend = Join-Path $RepoRoot "rewrite\deploy_rewrite.sh"

if ($RunMode -eq "Motors" -and -not $ConfirmWheelsLifted) {
    throw "Motor mode refused. Lift the wheels, then pass -ConfirmWheelsLifted."
}
if (-not (Test-Path -LiteralPath $Backend -PathType Leaf)) {
    throw "Missing deployment backend: $Backend"
}
if (-not (Test-Path -LiteralPath $EnvironmentRoot -PathType Container)) {
    throw "LoongArch environment root not found: $EnvironmentRoot"
}

function ConvertTo-WslPath {
    param([Parameter(Mandatory = $true)][string]$WindowsPath)
    $FullPath = [System.IO.Path]::GetFullPath($WindowsPath)
    if ($FullPath -notmatch '^([A-Za-z]):\\(.*)$') {
        throw "Cannot convert path to WSL: $FullPath"
    }
    $Drive = $Matches[1].ToLowerInvariant()
    $Tail = $Matches[2].Replace('\', '/')
    return "/mnt/$Drive/$Tail"
}

Write-Host "=== rewrite independent deployment ===" -ForegroundColor Cyan
Write-Host "Repository: $RepoRoot"
Write-Host "Board: ${BoardUser}@${BoardIP}"

Write-Host "`nWSL GCC 8.3 build/deployment" -ForegroundColor Yellow
$WslRepo = ConvertTo-WslPath $RepoRoot
$WslEnvironmentRoot = ConvertTo-WslPath $EnvironmentRoot
$WslArgs = @(
    "-d", $WslDistribution,
    "--cd", $WslRepo,
    "--", "env", "LQ_ENV_DIR=$WslEnvironmentRoot",
    "bash", "./rewrite/deploy_rewrite.sh",
    "--board-ip", $BoardIP,
    "--board-user", $BoardUser,
    "--run", $RunMode.ToLowerInvariant(),
    "--jobs", $Jobs
)
if ($BuildOnly) { $WslArgs += "--build-only" }
if ($ForceRebuild) { $WslArgs += "--force-rebuild" }
if ($SkipModels) { $WslArgs += "--skip-models" }
if ($NoBackup) { $WslArgs += "--no-backup" }
if ($ConfirmWheelsLifted) { $WslArgs += "--confirm-wheels-lifted" }
if ([string]::IsNullOrWhiteSpace($IdentityFile)) {
    $DefaultIdentity = Join-Path $env:USERPROFILE ".ssh\id_rsa"
    if (Test-Path -LiteralPath $DefaultIdentity -PathType Leaf) {
        $IdentityFile = $DefaultIdentity
    }
}
if (-not [string]::IsNullOrWhiteSpace($IdentityFile)) {
    if (-not (Test-Path -LiteralPath $IdentityFile -PathType Leaf)) {
        throw "SSH identity file not found: $IdentityFile"
    }
    $WslArgs += @("--identity-source", (ConvertTo-WslPath $IdentityFile))
}

& wsl.exe @WslArgs
if ($LASTEXITCODE -ne 0) {
    throw "rewrite build/deployment failed (WSL exit $LASTEXITCODE)."
}

Write-Host "`n=== Complete ===" -ForegroundColor Green
if ($BuildOnly) {
    Write-Host "Artifact: build\rewrite\lq_path_follow_rewrite"
} elseif ($RunMode -eq "None") {
    Write-Host "rewrite binary installed but not started."
} else {
    Write-Host "HTTP: http://${BoardIP}:8080"
    Write-Host "Telemetry: http://${BoardIP}:8080/telemetry"
}
