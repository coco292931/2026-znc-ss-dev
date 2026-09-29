
#!/usr/bin/env pwsh
<#
.SYNOPSIS
Build and deploy the reorganized lq_path_follow program.
原生 Windows 路径：构建走 mingw 宿主的 GCC 8.3 工具链（build_rewrite.ps1），
上传/安装走 Windows 自带 OpenSSH（ssh.exe / scp.exe）。不需要 WSL。

.DESCRIPTION
  构建: 调用 scripts\build\build_rewrite.ps1 target（自动解析工具链与依赖，
        内置工具链自愈与 ABI 校验）。需要 PowerShell 7+，脚本会自动探测
        PATH、便携版（Downloads\pwsh7\app\pwsh.exe）等位置。
  部署: 使用 ssh/scp 上传到板端 /home/root/，上传前后做 SHA-256 校验，
        旧程序默认备份为 .bak；同步标定数据、板端脚本与 NCNN 模型。

.PARAMETER WslDistribution
  历史参数（旧版 WSL 后端使用），保留以兼容 GUI 调用，本脚本不再使用。

.PARAMETER EnvironmentRoot
  历史参数（旧版 LoongArch Linux 环境目录），保留以兼容 GUI 调用，不再使用。

.EXAMPLE
.\scripts\deploy\deploy_rewrite.ps1
.\scripts\deploy\deploy_rewrite.ps1 -RunMode Dry
.\scripts\deploy\deploy_rewrite.ps1 -RunMode Motors -ConfirmWheelsLifted
.\scripts\deploy\deploy_rewrite.ps1 -BuildOnly -Jobs 6
.\scripts\deploy\deploy_rewrite.ps1 -ForceRebuild
#>
[CmdletBinding()]
param(
    [string]$BoardIP = "192.168.43.220",
    [string]$BoardUser = "root",
    [ValidateSet("None", "Dry", "Motors")]
    [string]$RunMode = "None",

    # 历史参数：旧版经由 WSL 调用 deploy_rewrite.sh 时使用。
    # 保留以兼容 GUI/旧调用方传参，本脚本不再使用这两个参数。
    [string]$WslDistribution = "",
    [string]$EnvironmentRoot = "",

    # 可选：显式指定原生构建的工具链/依赖（默认由 build_rewrite.ps1 自动解析）。
    [string]$ToolchainRoot = "",
    [string]$OpenCvDir = "",
    [string]$NcnnDir = "",

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

# deploy_rewrite.ps1 位于 <repo>\scripts\deploy\ 下，需要上溯三层到仓库根
# （原实现只用了两层 Parent，拼出 scripts\scripts\... 的错误路径）。
$RepoRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $PSCommandPath))
$BuildScript = Join-Path $RepoRoot "scripts\build\build_rewrite.ps1"
$Out = Join-Path $RepoRoot "build\rewrite\lq_path_follow_rewrite"

if ($RunMode -eq "Motors" -and -not $ConfirmWheelsLifted) {
    throw "Motor mode refused. Lift the wheels, then pass -ConfirmWheelsLifted."
}
if (-not (Test-Path -LiteralPath $BuildScript -PathType Leaf)) {
    throw "Missing build script: $BuildScript"
}

function Resolve-Pwsh7 {
    # build_rewrite.ps1 要求 PowerShell 7+。优先当前进程，其次 PATH，
    # 最后探测常见便携/系统安装位置。
    if ($PSVersionTable.PSVersion.Major -ge 7) {
        $self = Join-Path $PSHOME "pwsh.exe"
        if (Test-Path -LiteralPath $self -PathType Leaf) { return $self }
    }
    $cmd = Get-Command pwsh.exe -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    foreach ($candidate in @(
            (Join-Path $env:USERPROFILE "Downloads\pwsh7\app\pwsh.exe"),
            (Join-Path $env:ProgramFiles "PowerShell\7\pwsh.exe"))) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) { return $candidate }
    }
    return $null
}

function Get-Sha256FromOutput {
    param([string]$Text)
    $found = [regex]::Matches($Text, '[0-9a-fA-F]{64}')
    if ($found.Count -gt 0) {
        return $found[$found.Count - 1].Value.ToLowerInvariant()
    }
    return ""
}

Write-Host "=== rewrite native deployment (Windows, no WSL) ===" -ForegroundColor Cyan
Write-Host "Repository: $RepoRoot"
Write-Host "Board: ${BoardUser}@${BoardIP}"

# ---------------------------------------------------------------- 构建

Write-Host "`n=== 原生交叉编译（LoongArch GCC 8.3）===" -ForegroundColor Yellow
$Pwsh7 = Resolve-Pwsh7
if (-not $Pwsh7) {
    throw ("未找到 PowerShell 7（pwsh）：build_rewrite.ps1 需要 PS7+。" +
        "请安装 PowerShell 7，或把便携版放到 " +
        "$env:USERPROFILE\Downloads\pwsh7\app\pwsh.exe。")
}

$BuildArgs = @("-NoProfile", "-File", $BuildScript, "target", "-Jobs", $Jobs)
if ($ForceRebuild) { $BuildArgs += "-Force" }
if (-not [string]::IsNullOrWhiteSpace($ToolchainRoot)) { $BuildArgs += @("-ToolchainRoot", $ToolchainRoot) }
if (-not [string]::IsNullOrWhiteSpace($OpenCvDir)) { $BuildArgs += @("-OpenCvDir", $OpenCvDir) }
if (-not [string]::IsNullOrWhiteSpace($NcnnDir)) { $BuildArgs += @("-NcnnDir", $NcnnDir) }
& $Pwsh7 @BuildArgs
if ($LASTEXITCODE -ne 0) {
    throw "原生构建失败（build_rewrite.ps1 exit $LASTEXITCODE）。"
}
if (-not (Test-Path -LiteralPath $Out -PathType Leaf)) {
    throw "构建产物不存在: $Out"
}
$LocalHash = (Get-FileHash -LiteralPath $Out -Algorithm SHA256).Hash.ToLowerInvariant()
Write-Host "    产物   : $Out"
Write-Host "    SHA-256: $LocalHash"

if ($BuildOnly) {
    Write-Host "`n=== Complete (BuildOnly) ===" -ForegroundColor Green
    Write-Host "Artifact: build\rewrite\lq_path_follow_rewrite"
    exit 0
}

# ---------------------------------------------------------------- 上传与安装

$SshExe = (Get-Command ssh.exe -ErrorAction SilentlyContinue).Source
$ScpExe = (Get-Command scp.exe -ErrorAction SilentlyContinue).Source
if (-not $SshExe -or -not $ScpExe) {
    throw "未找到 Windows 自带 OpenSSH（ssh.exe / scp.exe）。"
}

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
}

$Remote = "${BoardUser}@${BoardIP}"
$RemotePath = "/home/root/lq_path_follow_rewrite"
$RemoteUpload = "${RemotePath}.upload"

$SshOpts = @(
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=8",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "HostKeyAlgorithms=+ssh-rsa",
    "-o", "PubkeyAcceptedAlgorithms=+ssh-rsa"
)
if (-not [string]::IsNullOrWhiteSpace($IdentityFile)) {
    $SshOpts += @("-i", $IdentityFile, "-o", "IdentitiesOnly=yes")
}

function Invoke-SshBoard {
    param([Parameter(Mandatory = $true)][string]$Command)
    $output = & $SshExe @SshOpts $Remote $Command | Out-String
    if ($LASTEXITCODE -ne 0) {
        throw "board command failed (exit $LASTEXITCODE): $Command"
    }
    return $output
}

function Invoke-UploadFile {
    param(
        [Parameter(Mandatory = $true)][string]$Source,
        [Parameter(Mandatory = $true)][string]$Destination
    )
    & $ScpExe -q @SshOpts $Source $Destination
    if ($LASTEXITCODE -eq 0) { return }
    # 老板端 sshd 可能没有 SFTP 子系统；OpenSSH 9+ 用 -O 退回传统 scp 协议。
    Write-Host "    scp(SFTP) 失败，改用传统 scp 协议（-O）重试..." -ForegroundColor DarkYellow
    & $ScpExe -q -O @SshOpts $Source $Destination
    if ($LASTEXITCODE -ne 0) {
        throw "上传失败: $Source -> $Destination"
    }
}

Write-Host "`n=== 检查板端连接 ${Remote} ===" -ForegroundColor Yellow
Invoke-SshBoard 'test "$(uname -m)" = loongarch64' | Out-Null

$InstalledHash = Get-Sha256FromOutput (Invoke-SshBoard "if [ -f '$RemotePath' ]; then sha256sum '$RemotePath'; fi")
$BinaryUpdated = 1
if ($InstalledHash -eq $LocalHash) {
    Write-Host "    binary unchanged; upload skipped" -ForegroundColor DarkGray
    $BinaryUpdated = 0
}
else {
    Invoke-UploadFile $Out "${Remote}:${RemoteUpload}"
    $RemoteHash = Get-Sha256FromOutput (Invoke-SshBoard "sha256sum '$RemoteUpload'")
    if ($RemoteHash -ne $LocalHash) {
        Invoke-SshBoard "rm -f '$RemoteUpload'" | Out-Null
        throw "Upload checksum mismatch."
    }
    $BackupCmd = "cp -f '$RemotePath' '${RemotePath}.bak'"
    if ($NoBackup) { $BackupCmd = ":" }
    Invoke-SshBoard ("set -e; killall lq_path_follow_rewrite 2>/dev/null || true; " +
        "if [ -f '$RemotePath' ]; then $BackupCmd; fi; " +
        "mv -f '$RemoteUpload' '$RemotePath'; chmod +x '$RemotePath'; sync") | Out-Null
}

Write-Host "`n=== 同步标定与辅助资产 ===" -ForegroundColor Yellow
Invoke-SshBoard "mkdir -p /home/root/rewrite /home/root/models" | Out-Null

function Sync-BoardAsset {
    param(
        [Parameter(Mandatory = $true)][string]$Source,
        [Parameter(Mandatory = $true)][string]$Destination
    )
    if (-not (Test-Path -LiteralPath $Source -PathType Leaf)) {
        throw "Missing file: $Source"
    }
    # 板端执行的 .sh 必须为 LF 行尾：Windows 工作区里 CRLF 的脚本上传后
    # shebang 会带 \r，板端报 "bad interpreter"。上传前做字节级规范化
    # （不做字符解码，任何编码都保真）。
    $uploadPath = $Source
    $tempPath = $null
    $sourceBytes = [System.IO.File]::ReadAllBytes($Source)
    $hasCrlf = $false
    for ($i = 0; $i -lt ($sourceBytes.Length - 1); $i++) {
        if ($sourceBytes[$i] -eq 13 -and $sourceBytes[$i + 1] -eq 10) {
            $hasCrlf = $true
            break
        }
    }
    if ($Source -like "*.sh" -and $hasCrlf) {
        $lfBytes = New-Object System.Collections.Generic.List[byte]
        for ($i = 0; $i -lt $sourceBytes.Length; $i++) {
            if ($sourceBytes[$i] -eq 13 -and
                ($i + 1) -lt $sourceBytes.Length -and
                $sourceBytes[$i + 1] -eq 10) {
                continue
            }
            $lfBytes.Add($sourceBytes[$i])
        }
        $tempPath = Join-Path ([System.IO.Path]::GetTempPath()) (
            "deploy_lf_" + [System.IO.Path]::GetFileName($Source)
        )
        [System.IO.File]::WriteAllBytes($tempPath, $lfBytes.ToArray())
        $uploadPath = $tempPath
        Write-Host "    行尾规范化: CRLF -> LF ($Source)" -ForegroundColor DarkGray
    }
    try {
        $localHash = (Get-FileHash -LiteralPath $uploadPath -Algorithm SHA256).Hash.ToLowerInvariant()
        $remoteHash = Get-Sha256FromOutput (Invoke-SshBoard "if [ -f '$Destination' ]; then sha256sum '$Destination'; fi")
        if ($localHash -eq $remoteHash) {
            Write-Host "    unchanged $Destination" -ForegroundColor DarkGray
            return
        }
        Invoke-UploadFile $uploadPath "${Remote}:${Destination}.upload"
        $verifyHash = Get-Sha256FromOutput (Invoke-SshBoard "sha256sum '${Destination}.upload'")
        if ($verifyHash -ne $localHash) {
            Invoke-SshBoard "rm -f '${Destination}.upload'" | Out-Null
            throw "Asset checksum mismatch: $Destination"
        }
        Invoke-SshBoard "mv -f '${Destination}.upload' '$Destination'" | Out-Null
        Write-Host "    updated $Destination"
    }
    finally {
        if ($tempPath) {
            Remove-Item -LiteralPath $tempPath -Force -ErrorAction SilentlyContinue
        }
    }
}

Sync-BoardAsset (Join-Path $RepoRoot "config\标定数据.txt") "/home/root/rewrite/标定数据.txt"
Sync-BoardAsset (Join-Path $RepoRoot "scripts\board\setup_st7735s_spi.sh") "/home/root/setup_st7735s_spi.sh"
Sync-BoardAsset (Join-Path $RepoRoot "scripts\board\record_imu_path.sh") "/home/root/record_imu_path.sh"
Sync-BoardAsset $Out "/home/root/lq_path_follow_imu"
Invoke-SshBoard "chmod +x /home/root/setup_st7735s_spi.sh /home/root/record_imu_path.sh /home/root/lq_path_follow_imu" | Out-Null

if (-not $SkipModels) {
    foreach ($asset in @("best.ncnn.param", "best.ncnn.bin", "model_metadata.json")) {
        Sync-BoardAsset (Join-Path $RepoRoot "models\target\$asset") "/home/root/models/$asset"
    }
}

$FinalHash = Get-Sha256FromOutput (Invoke-SshBoard "sha256sum '$RemotePath'")
if ($FinalHash -ne $LocalHash) {
    throw "Final checksum mismatch."
}

if ($RunMode -ne "None") {
    $ModeFlag = "--dry-run"
    if ($RunMode -eq "Motors") { $ModeFlag = "--enable-motors" }
    $RunArgs = "--camera /dev/video0 --http 8080 " +
        "--model-dir /home/root/models " +
        "--calibration /home/root/rewrite/标定数据.txt " +
        "--target-actions --target-input-size 32 --target-close-size 0.04 " +
        $ModeFlag
    $StartCommand = 'set -e; killall lq_path_follow_rewrite 2>/dev/null || true; ' +
        'cd /home/root; ' +
        'nohup env LD_LIBRARY_PATH=/home/root/LQ_Dep_libs/opencv-lib:/home/root/LQ_Dep_libs/ncnn-lib ' +
        './lq_path_follow_rewrite ' + $RunArgs + ' ' +
        '> /home/root/lq_path_follow_rewrite.log 2>&1 </dev/null & ' +
        'echo $! > /home/root/lq_path_follow_rewrite.pid; sleep 2; ' +
        'kill -0 $(cat /home/root/lq_path_follow_rewrite.pid)'
    Invoke-SshBoard $StartCommand | Out-Null
    Write-Host "==> Running ${RunMode}: http://${BoardIP}:8080" -ForegroundColor Green
    Write-Host "==> Telemetry: http://${BoardIP}:8080/telemetry" -ForegroundColor Green
}
else {
    if ($BinaryUpdated -eq 0) {
        Invoke-SshBoard "killall lq_path_follow_rewrite 2>/dev/null || true" | Out-Null
    }
    Write-Host "==> Installed independently; program not started"
}

Write-Host "`n=== Complete ===" -ForegroundColor Green
Write-Host "Artifact: build\rewrite\lq_path_follow_rewrite"
