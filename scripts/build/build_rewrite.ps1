#!/usr/bin/env pwsh
#Requires -Version 7.0
<#
.SYNOPSIS
    2026-znc-ss-dev 原生 Windows 交叉编译（旧世界 LoongArch / GCC 8.3）。

.DESCRIPTION
    直接调用 mingw 宿主的旧世界工具链（bin\loongarch64-linux-gnu-g++.exe）在本机完成
    编译 / 链接 / ABI 校验。不需要 WSL、Git-Bash 或任何 bash 环境。

    模式与 scripts/build/build_rewrite.sh 一一对应：
      selftest | target | replay | display-example | motor-test | motor-test-sim

    相较 .sh 版修掉的两个 replay 缺陷：
      1. SMARTCAR_SRC 未定义 —— .sh 的 replay 分支引用了它却从未赋值（Linux 上同样会挂）
      2. replay 链接漏了 -ldl -lm —— OpenCV 需要 dlopen/dlsym/dlclose@GLIBC_2.27

    链接后对产物做「真正决定能否上板」的校验，而不是只看架构名：
      · Machine == LoongArch
      · e_flags 含 OBJ-v0        （旧世界 psABI v0；新世界是 OBJ-v1，板子跑不了）
      · GLIBC 符号版本上限 <= 板端 glibc
      · 动态加载器 == /lib64/ld.so.1
      · 打印 NEEDED 清单，提醒板端必须具备这些 .so

.PARAMETER Mode
    selftest        主机自测程序（-DPATH_FOLLOW_NO_OPENCV -DPATH_FOLLOW_NO_HW）
    target          主程序 lq_path_follow_rewrite（需要 OpenCV；ncnn 可选）
    replay          离线视频回放（rewrite_video_replay）
    display-example ST7735S 屏幕示例
    motor-test      电机抖动测试（真硬件）
    motor-test-sim  电机抖动测试（SMARTCAR_SIM 仿真，不碰硬件）

.PARAMETER ToolchainRoot
    旧世界工具链根目录（其下应有 bin\loongarch64-linux-gnu-g++.exe）。
    解析顺序：本参数 > 环境变量 LQ_TC > 已知候选 > Downloads 下有限深度扫描。

.PARAMETER OpenCvDir
    opencv_install 目录（须含 include\opencv4\opencv2\core.hpp 与 lib）。
    解析顺序：本参数 > 环境变量 OPENCV_DIR > 已知候选 > Downloads 下扫描 LQ_Dep_libs。

.PARAMETER NcnnDir
    ncnn_install 目录（须含 include\ncnn\net.h）。不传则不加 -DREWRITE_WITH_NCNN。

.PARAMETER Jobs
    并行编译任务数，默认 min(CPU, 8)。仅在 PowerShell 7+ 生效，否则自动串行。

.PARAMETER Force
    忽略增量缓存，全量重编。

.PARAMETER TargetGlibcMax
    板端 glibc 上限，默认 2.28。产物 GLIBC 符号版本若高于它即判失败。

.PARAMETER NoVerify
    跳过链接后的 ABI 校验。

.PARAMETER RunSelftest
    selftest 模式下在本机尝试执行产物（仅当配了 qemu-loongarch64 之类的环境才有意义）。

.EXAMPLE
    .\scripts\build\build_rewrite.ps1 target
.EXAMPLE
    .\scripts\build\build_rewrite.ps1 target -Jobs 8 -Force
.EXAMPLE
    .\scripts\build\build_rewrite.ps1 motor-test-sim
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('selftest', 'target', 'replay', 'display-example', 'motor-test', 'motor-test-sim')]
    [string]$Mode = 'selftest',

    [string]$ToolchainRoot,
    [string]$OpenCvDir,
    [string]$NcnnDir,

    [ValidateRange(1, 64)]
    [int]$Jobs = [Math]::Min([Environment]::ProcessorCount, 8),

    [switch]$Force,
    [switch]$NoVerify,

    [ValidateRange(2.0, 9.9)]
    [double]$TargetGlibcMax = 2.28,

    [switch]$RunSelftest
)

# 注意：这里不能用 'Stop'。gcc 会把 warning 写到 stderr，而 `2>&1` 在
# $ErrorActionPreference='Stop' 下会把第一行 stderr 当成终止性错误（NativeCommandError）
# 直接中断编译。统一用 'Continue'，所有失败都靠显式检查 $LASTEXITCODE。
$ErrorActionPreference = 'Continue'
$ProgressPreference = 'SilentlyContinue'

# ---------------------------------------------------------------- 基础路径

$RepoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$SrcRoot = Join-Path $RepoRoot 'src'
$AppSrc = Join-Path $SrcRoot 'app'
$NavSrc = Join-Path $SrcRoot 'navigation'
$VisionSrc = Join-Path $SrcRoot 'vision'
$MotionSrc = Join-Path $SrcRoot 'motion'
$ImuSrc = Join-Path $SrcRoot 'sensors\imu'
$TofSrc = Join-Path $SrcRoot 'sensors\tof'
$TelemetrySrc = Join-Path $SrcRoot 'telemetry'
$DisplaySrc = Join-Path $SrcRoot 'display'
$PlatformSrc = Join-Path $SrcRoot 'platform'
$DriverSrc = Join-Path $SrcRoot 'drivers\loongson'
$BuildDir = Join-Path $RepoRoot 'build\rewrite'

# ---------------------------------------------------------------- 小工具

function ConvertTo-GccPath {
    param([Parameter(Mandatory)][string]$Path)
    # 统一成正斜杠，避免反斜杠在 gcc 参数里被当转义
    return ($Path -replace '\\', '/')
}

function Get-Sha256Hex {
    param([Parameter(Mandatory)][string]$Text)
    $sha = [System.Security.Cryptography.SHA256]::Create()
    try {
        $bytes = [System.Text.Encoding]::UTF8.GetBytes($Text)
        return ([System.BitConverter]::ToString($sha.ComputeHash($bytes)) -replace '-', '').ToLowerInvariant()
    }
    finally { $sha.Dispose() }
}

function Write-Section {
    param([string]$Title)
    Write-Host ''
    Write-Host "=== $Title ===" -ForegroundColor Cyan
}

function Resolve-ToolchainRoot {
    param([string]$Explicit)

    $exeRel = 'bin\loongarch64-linux-gnu-g++.exe'
    $probes = [System.Collections.Generic.List[string]]::new()
    foreach ($p in @($Explicit, $env:LQ_TC,
            (Join-Path $env:USERPROFILE 'Downloads\lstc83\loongson-gnu-toolchain-8.3-i686-mingw-loongarch64-linux-gnu-rc1.6'))) {
        if (-not [string]::IsNullOrWhiteSpace($p)) { $probes.Add($p) }
    }
    foreach ($p in $probes) {
        if (Test-Path -LiteralPath (Join-Path $p $exeRel)) { return (Resolve-Path -LiteralPath $p).Path }
    }

    # 兜底：在 Downloads 下有限深度扫描名字里带 loongarch 的目录
    $dl = Join-Path $env:USERPROFILE 'Downloads'
    if (Test-Path -LiteralPath $dl) {
        $hit = Get-ChildItem -LiteralPath $dl -Directory -Depth 3 -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -like '*loongarch*' } |
            Where-Object { Test-Path -LiteralPath (Join-Path $_.FullName $exeRel) } |
            Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    return $null
}

function Resolve-DepDir {
    <# 返回目录路径；找不到时返回 $null（由调用方决定是硬失败还是可选）。 #>
    param(
        [string]$Explicit,
        [string]$EnvValue,
        [string[]]$Candidates,
        [string]$MarkerRel,
        [string]$Label
    )

    foreach ($p in @($Explicit) + @($EnvValue) + @($Candidates)) {
        if ([string]::IsNullOrWhiteSpace($p)) { continue }
        if (Test-Path -LiteralPath (Join-Path $p $MarkerRel)) { return (Resolve-Path -LiteralPath $p).Path }
    }

    # 兜底：在 Downloads 下找 LQ_Dep_libs
    $dl = Join-Path $env:USERPROFILE 'Downloads'
    if (Test-Path -LiteralPath $dl) {
        $roots = Get-ChildItem -LiteralPath $dl -Directory -Recurse -Depth 4 -Filter 'LQ_Dep_libs' -ErrorAction SilentlyContinue
        foreach ($r in $roots) {
            $guess = Join-Path $r.FullName (Split-Path -Leaf $Candidates[0])
            if (Test-Path -LiteralPath (Join-Path $guess $MarkerRel)) { return (Resolve-Path -LiteralPath $guess).Path }
        }
    }

    Write-Host "找不到 $Label。" -ForegroundColor Red
    Write-Host "  期望目录含: $MarkerRel" -ForegroundColor DarkGray
    Write-Host "  请用 -$Label 参数或环境变量指定，例如:" -ForegroundColor DarkGray
    foreach ($c in $Candidates) { Write-Host "    $c" -ForegroundColor DarkGray }
    return $null
}

# ---------------------------------------------------------------- 编译驱动

function Invoke-BuildBatch {
    <#
        并行/串行执行一批编译命令。
        $Items: @{ Src = <显示名>; ArgList = <字符串数组（不含编译器本身）> }
        返回失败项列表。
    #>
    param(
        [Parameter(Mandatory)][string]$Cxx,
        [object[]]$Items = @(),
        [int]$Throttle = 1
    )

    if (@($Items).Count -eq 0) { return @() }

    $useParallel = ($PSVersionTable.PSVersion.Major -ge 7) -and ($Throttle -gt 1) -and ($Items.Count -gt 1)

    if ($useParallel) {
        $results = $Items | ForEach-Object -Parallel {
            $exe = $using:Cxx
            $argList = $_.ArgList
            $text = & $exe @argList 2>&1 | Out-String
            [pscustomobject]@{ Src = $_.Src; ExitCode = $LASTEXITCODE; Text = $text }
        } -ThrottleLimit $Throttle
    }
    else {
        $results = foreach ($item in $Items) {
            $argList = $item.ArgList
            $text = & $Cxx @argList 2>&1 | Out-String
            [pscustomobject]@{ Src = $item.Src; ExitCode = $LASTEXITCODE; Text = $text }
        }
    }

    $failed = [System.Collections.Generic.List[object]]::new()
    foreach ($r in $results) {
        if ($r.ExitCode -ne 0) {
            Write-Host "  失败: $($r.Src)" -ForegroundColor Red
            ($r.Text -split "`r?`n" | Where-Object { $_ -match 'error|错误' } | Select-Object -First 8) |
                ForEach-Object { Write-Host "        $_" -ForegroundColor Red }
            $failed.Add($r)
        }
        elseif ($r.Text.Trim()) {
            Write-Host "  警告: $($r.Src)" -ForegroundColor DarkYellow
            ($r.Text -split "`r?`n" | Where-Object { $_ -match 'warning' } | Select-Object -First 4) |
                ForEach-Object { Write-Host "        $_" -ForegroundColor DarkYellow }
        }
    }
    return $failed.ToArray()
}

function Test-ArtifactAbi {
    <#
        校验产物是不是「真能上旧世界板子的」二进制。
        这是本脚本相对 .sh 版最重要的补充：不再只看架构名。
    #>
    param(
        [Parameter(Mandatory)][string]$ReadElf,
        [Parameter(Mandatory)][string]$Path,
        [double]$GlibcMax = 2.28
    )

    $hdr = & $ReadElf -h $Path 2>&1 | Out-String
    $ver = & $ReadElf --version-info $Path 2>&1 | Out-String
    $prg = & $ReadElf -l $Path 2>&1 | Out-String
    $dyn = & $ReadElf -d $Path 2>&1 | Out-String

    $problems = [System.Collections.Generic.List[string]]::new()
    $info = [ordered]@{}

    Write-Section '产物 ABI 校验'
    Write-Host ("  文件   : {0}  ({1:N0} 字节)" -f $Path, (Get-Item -LiteralPath $Path).Length)

    # 1) 机器
    $isLoong = $hdr -match 'Machine:\s+LoongArch'
    $info['Machine'] = if ($isLoong) { 'LoongArch' } else { '非 LoongArch' }
    if ($isLoong) { Write-Host '  架构   : LoongArch                 [OK]' -ForegroundColor Green }
    else {
        Write-Host '  架构   : 不是 LoongArch            [失败]' -ForegroundColor Red
        $problems.Add('产物不是 LoongArch 二进制')
    }

    # 2) e_flags / world
    #    ⚠️ 绝不能匹配文本！同一个 e_flags 值 0x3：
    #       旧工具链自带 readelf(binutils 2.31.1) 印 "0x3, LP64"
    #       新世界 readelf(binutils 2.45)      印 "0x3, DOUBLE-FLOAT, OBJ-v0"
    #    所以必须按位判定：
    #       低 3 位 EF_LARCH_ABI_*   3 = double-float（即 lp64d）
    #       位   0x40 EF_LARCH_OBJABI_V1  置位 = 新世界 object ABI v1（板子跑不了）
    $flagsHit = ($hdr -split "`r?`n" | Select-String 'Flags:' | Select-Object -First 1)
    $flagsText = if ($flagsHit) { ($flagsHit.Line -replace '.*Flags:\s*', '').Trim() } else { '(未读到)' }
    $info['Flags'] = $flagsText

    $flagsNum = $null
    if ($flagsText -match '0x([0-9a-fA-F]+)') { $flagsNum = [Convert]::ToInt64($Matches[1], 16) }

    if ($null -eq $flagsNum) {
        Write-Host ("  e_flags: {0}   [无法解析]" -f $flagsText) -ForegroundColor Red
        $problems.Add("无法解析 e_flags: $flagsText")
    }
    else {
        $objAbiV1 = ($flagsNum -band 0x40) -ne 0
        $floatAbi = [int]($flagsNum -band 0x7)
        $worldName = if ($objAbiV1) { 'OBJ-v1 新世界' } else { 'OBJ-v0 旧世界' }
        $floatName = switch ($floatAbi) {
            0 { 'LP64 (未指定浮点)' }
            1 { 'LP64 软浮点' }
            2 { 'LP64F 单精度' }
            3 { 'LP64D 双精度' }
            default { "浮点ABI=$floatAbi" }
        }
        if ($objAbiV1) {
            Write-Host ("  e_flags: 0x{0:x}  {1} / {2}   [失败]" -f $flagsNum, $worldName, $floatName) -ForegroundColor Red
            $problems.Add('e_flags 置了 EF_LARCH_OBJABI_V1(0x40) —— 新世界产物，旧世界板子无法运行')
        }
        elseif ($floatAbi -eq 1) {
            Write-Host ("  e_flags: 0x{0:x}  {1} / {2}   [失败 板子需要 lp64d]" -f $flagsNum, $worldName, $floatName) -ForegroundColor Red
            $problems.Add('编译目标是软浮点，板子需要 lp64d')
        }
        else {
            Write-Host ("  e_flags: 0x{0:x}  {1} / {2}   [OK]" -f $flagsNum, $worldName, $floatName) -ForegroundColor Green
        }
    }

    # 3) 动态加载器（静态产物没有该行，属正常）
    $interp = $null
    if ($prg -match 'Requesting program interpreter:\s*([^\]]+)\]') { $interp = $Matches[1].Trim() }
    if ($interp) {
        $info['Interpreter'] = $interp
        if ($interp -eq '/lib64/ld.so.1') {
            Write-Host ("  加载器 : {0}    [OK]" -f $interp) -ForegroundColor Green
        }
        else {
            Write-Host ("  加载器 : {0}    [可疑 期望 /lib64/ld.so.1]" -f $interp) -ForegroundColor Yellow
        }
    }
    else {
        Write-Host '  加载器 : (静态链接，无解释器)' -ForegroundColor DarkGray
        $info['Interpreter'] = '(static)'
    }

    # 4) GLIBC 符号版本上限
    $glibcVers = [regex]::Matches($ver, 'GLIBC_(\d+)\.(\d+)') |
        ForEach-Object { [version]("{0}.{1}" -f $_.Groups[1].Value, $_.Groups[2].Value) }
    if ($glibcVers) {
        $maxGlibc = ($glibcVers | Sort-Object -Descending | Select-Object -First 1)
        $info['GlibcMax'] = $maxGlibc.ToString()
        if ($maxGlibc -le [version]$GlibcMax.ToString()) {
            Write-Host ("  GLIBC  : 上限 {0}  (板端 <= {1})  [OK]" -f $maxGlibc, $GlibcMax) -ForegroundColor Green
        }
        else {
            Write-Host ("  GLIBC  : 上限 {0}  (板端 <= {1})  [失败]" -f $maxGlibc, $GlibcMax) -ForegroundColor Red
            $problems.Add("需要 GLIBC_$maxGlibc，超出板端 glibc $GlibcMax")
        }
    }
    else {
        Write-Host '  GLIBC  : (无动态符号版本信息)' -ForegroundColor DarkGray
    }

    # 5) NEEDED
    $needed = [regex]::Matches($dyn, 'Shared library:\s*\[([^\]]+)\]') | ForEach-Object { $_.Groups[1].Value }
    if ($needed.Count) {
        $info['Needed'] = $needed
        Write-Host ("  NEEDED : ({0} 个) {1}" -f $needed.Count, ($needed -join ', ')) -ForegroundColor DarkGray
        $runtime = $needed | Where-Object { $_ -like 'libopencv_*' -or $_ -like 'libncnn*' -or $_ -like 'libgomp*' }
        if ($runtime) {
            Write-Host ("  板端需自备: {0}" -f ($runtime -join ', ')) -ForegroundColor Yellow
            Write-Host '          （产物无 rpath，运行前需 export LD_LIBRARY_PATH=...）' -ForegroundColor DarkGray
        }
    }

    if ($problems.Count) {
        Write-Host ''
        Write-Host '  ✗ ABI 校验未通过：' -ForegroundColor Red
        $problems | ForEach-Object { Write-Host "      · $_" -ForegroundColor Red }
        return $false
    }
    Write-Host ''
    Write-Host '  ✓ 产物通过 ABI 校验，可用于旧世界板卡' -ForegroundColor Green
    return $true
}

# ---------------------------------------------------------------- 解析路径

Write-Section '环境解析'

$tcRoot = Resolve-ToolchainRoot -Explicit $ToolchainRoot
if (-not $tcRoot) {
    Write-Host '找不到旧世界工具链（bin\loongarch64-linux-gnu-g++.exe）。' -ForegroundColor Red
    Write-Host '  请用 -ToolchainRoot 指定，或设置环境变量 LQ_TC。' -ForegroundColor DarkGray
    Write-Host '  例: .\scripts\build\build_rewrite.ps1 target -ToolchainRoot C:\path\to\loongson-gnu-toolchain-8.3-i686-mingw-loongarch64-linux-gnu-rc1.6' -ForegroundColor DarkGray
    exit 2
}

$Cxx = Join-Path $tcRoot 'bin\loongarch64-linux-gnu-g++.exe'
$ReadElf = Join-Path $tcRoot 'bin\loongarch64-linux-gnu-readelf.exe'

$cxxVersion = (& $Cxx -dumpversion 2>&1 | Out-String).Trim()
if ($cxxVersion -notlike '8.3*') {
    Write-Host "工具链不是 GCC 8.3：$Cxx  (dumpversion=$cxxVersion)" -ForegroundColor Red
    Write-Host '  旧世界必须是 GCC 8.3 —— 其它版本（尤其 gcc14/15）产出新世界 ABI，板子跑不了。' -ForegroundColor DarkGray
    exit 2
}
$cxxFull = (& $Cxx --version 2>&1 | Select-Object -First 1)

$depCandidatesRoot = Join-Path $env:USERPROFILE 'Downloads\longTech-Study'
$knownOcv = @(
    (Join-Path $depCandidatesRoot 'Loongson_2k300_301_Library-龙邱\Loongson_2K300_301_LIB\tools\LQ_Dep_libs\opencv_install'),
    (Join-Path $env:USERPROFILE 'Downloads\lq环境配置 (2)\LQ_Dep_libs\opencv_install')
)
$knownNcnn = @(
    (Join-Path $depCandidatesRoot 'Loongson_2k300_301_Library-龙邱\Loongson_2K300_301_LIB\tools\LQ_Dep_libs\ncnn_install'),
    (Join-Path $env:USERPROFILE 'Downloads\lq环境配置 (2)\LQ_Dep_libs\ncnn_install')
)

Write-Host ("  工具链 : {0}" -f $tcRoot)
Write-Host ("           {0}" -f $cxxFull) -ForegroundColor DarkGray
Write-Host ("  仓库   : {0}" -f $RepoRoot)
Write-Host ("  模式   : {0}   并行: {1}" -f $Mode, $Jobs)

$needOpenCv = $Mode -in @('target', 'replay')
$ocvRoot = $null
if ($needOpenCv) {
    $ocvRoot = Resolve-DepDir -Explicit $OpenCvDir -EnvValue $env:OPENCV_DIR -Candidates $knownOcv `
        -MarkerRel 'include\opencv4\opencv2\core.hpp' -Label 'OpenCvDir'
    if (-not $ocvRoot) {
        Write-Host "$Mode 模式需要 OpenCV，无法继续。" -ForegroundColor Red
        exit 2
    }
    Write-Host ("  OpenCV : {0}" -f $ocvRoot)
}

$ncnnRoot = $null
if ($Mode -eq 'target') {
    # ncnn 确实存在机器上时才启用，但不作为硬前提（源码里是 #ifdef REWRITE_WITH_NCNN 保护的）
    $ncnnRoot = Resolve-DepDir -Explicit $NcnnDir -EnvValue $env:NCNN_DIR -Candidates $knownNcnn `
        -MarkerRel 'include\ncnn\net.h' -Label 'NcnnDir'
    if ($ncnnRoot) { Write-Host ("  ncnn   : {0}" -f $ncnnRoot) }
    else {
        Write-Host '  ncnn   : 未启用（不加 -DREWRITE_WITH_NCNN）' -ForegroundColor DarkYellow
    }
}

New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null

# ---------------------------------------------------------------- 公共编译参数

$Common = @(
    '-std=c++17', '-O2', '-Wall', '-Wextra'
    "-I$(ConvertTo-GccPath $AppSrc)"
    "-I$(ConvertTo-GccPath $VisionSrc)"
    "-I$(ConvertTo-GccPath $NavSrc)"
    "-I$(ConvertTo-GccPath $MotionSrc)"
    "-I$(ConvertTo-GccPath $ImuSrc)"
    "-I$(ConvertTo-GccPath $TofSrc)"
    "-I$(ConvertTo-GccPath $TelemetrySrc)"
    "-I$(ConvertTo-GccPath $DisplaySrc)"
    "-I$(ConvertTo-GccPath $PlatformSrc)"
    "-I$(ConvertTo-GccPath (Join-Path $PlatformSrc 'safety'))"
    "-I$(ConvertTo-GccPath (Join-Path $DriverSrc 'inc'))"
)

$CoreSrc = @(
    (Join-Path $NavSrc 'path_params.cpp')
    (Join-Path $VisionSrc 'vision_pipeline.cpp')
    (Join-Path $NavSrc 'path_controller.cpp')
    (Join-Path $MotionSrc 'motion_control.cpp')
    (Join-Path $MotionSrc 'motor_adapter.cpp')
    (Join-Path $ImuSrc 'imu_feedback.cpp')
    (Join-Path $NavSrc 'inertial_navigation.cpp')
    (Join-Path $TofSrc 'tof_slope_sensor.cpp')
    (Join-Path $NavSrc 'odometry.cpp')
    (Join-Path $TelemetrySrc 'http_streamer.cpp')
    (Join-Path $VisionSrc 'target_recognizer.cpp')
)

$OpenCvIncludes = @()
$OpenCvLibs = @()
if ($ocvRoot) {
    $OpenCvIncludes = @("-I$(ConvertTo-GccPath (Join-Path $ocvRoot 'include\opencv4'))")
    $OpenCvLibs = @(
        "-L$(ConvertTo-GccPath (Join-Path $ocvRoot 'lib'))"
        "-Wl,-rpath-link,$(ConvertTo-GccPath (Join-Path $ocvRoot 'lib'))"
        '-lopencv_core', '-lopencv_imgproc', '-lopencv_videoio', '-lopencv_imgcodecs'
    )
}

# ---------------------------------------------------------------- 各模式

switch ($Mode) {

    'selftest' {
        Write-Section '构建 selftest'
        $out = Join-Path $BuildDir 'rewrite_path_selftest'
        $argList = $Common +
        @('-DPATH_FOLLOW_NO_OPENCV', '-DPATH_FOLLOW_NO_HW') +
        @((Join-Path $AppSrc 'path_selftest.cpp')) +
        $CoreSrc +
        @('-pthread', '-o', (ConvertTo-GccPath $out))

        $text = & $Cxx @argList 2>&1 | Out-String
        if ($LASTEXITCODE -ne 0) {
            ($text -split "`r?`n" | Where-Object { $_ -match 'error' } | Select-Object -First 15) |
                ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
            Write-Host 'selftest 编译失败' -ForegroundColor Red
            exit 1
        }
        Write-Host ("  built {0}" -f $out) -ForegroundColor Green

        if (-not $NoVerify) { [void](Test-ArtifactAbi -ReadElf $ReadElf -Path $out -GlibcMax $TargetGlibcMax) }

        Write-Host ''
        Write-Host '注意：selftest 的产物是 LoongArch 二进制，Windows 上无法直接执行。' -ForegroundColor Yellow
        Write-Host '      要跑它请用 qemu-loongarch64 或直接 scp 到板卡。' -ForegroundColor DarkGray
        if ($RunSelftest) {
            Write-Host '  尝试本机执行…' -ForegroundColor DarkGray
            try { & $out } catch { Write-Host ("  无法执行: {0}" -f $_.Exception.Message) -ForegroundColor DarkYellow }
        }
        exit 0
    }

    'display-example' {
        Write-Section '构建 display-example'
        $out = Join-Path $BuildDir 'rewrite_st7735s_example'
        $argList = $Common + @(
            (Join-Path $AppSrc 'st7735s_example.cpp')
            (Join-Path $DisplaySrc 'st7735s.cpp')
            (Join-Path $DisplaySrc 'spi1_shared.cpp')
            (Join-Path $DriverSrc 'LQ_HW_GPIO.cpp')
            (Join-Path $DriverSrc 'LQ_MAP_ADDR.cpp')
        ) + @('-pthread', '-o', (ConvertTo-GccPath $out))

        $text = & $Cxx @argList 2>&1 | Out-String
        if ($LASTEXITCODE -ne 0) {
            ($text -split "`r?`n" | Where-Object { $_ -match 'error' } | Select-Object -First 15) |
                ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
            Write-Host 'display-example 编译失败' -ForegroundColor Red
            exit 1
        }
        Write-Host ("  built {0}" -f $out) -ForegroundColor Green
        if (-not $NoVerify) { [void](Test-ArtifactAbi -ReadElf $ReadElf -Path $out -GlibcMax $TargetGlibcMax) }
        exit 0
    }

    { $_ -in 'motor-test', 'motor-test-sim' } {
        Write-Section "构建 $Mode"
        $isSim = ($Mode -eq 'motor-test-sim')
        $out = Join-Path $BuildDir 'rewrite_motor_stutter_test'

        # 注意：MOTOR_TEST_VARIANT 在源码里是当字符串用的（MOTOR_TEST_VARIANT + ".csv"），
        # 所以必须带引号传给 -D。PowerShell 7 会按 MSVCRT 规则转义，gcc 收到的 argv 是
        # -DMOTOR_TEST_VARIANT="rewrite"。
        $defines = @('-DMOTOR_TEST_VARIANT="rewrite"')
        $includes = @("-I$(ConvertTo-GccPath $PlatformSrc)")

        $sources = @(
            (Join-Path $AppSrc 'motor_stutter_test.cpp')
            (Join-Path $NavSrc 'path_params.cpp')
            (Join-Path $MotionSrc 'motion_control.cpp')
            (Join-Path $MotionSrc 'motor_adapter.cpp')
            (Join-Path $PlatformSrc 'hal.cpp')
            (Join-Path $PlatformSrc 'success_motor.cpp')
        )
        if ($isSim) {
            $defines += '-DSMARTCAR_SIM'
        }
        else {
            $includes += "-I$(ConvertTo-GccPath (Join-Path $DriverSrc 'inc'))"
            $sources += @(
                (Join-Path $DriverSrc 'LQ_HW_GPIO.cpp')
                (Join-Path $DriverSrc 'LQ_MAP_ADDR.cpp')
            )
        }

        $argList = $Common + $defines + $includes + $sources +
        @('-pthread', '-lm', '-o', (ConvertTo-GccPath $out))

        $text = & $Cxx @argList 2>&1 | Out-String
        if ($LASTEXITCODE -ne 0) {
            ($text -split "`r?`n" | Where-Object { $_ -match 'error' } | Select-Object -First 15) |
                ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
            Write-Host "$Mode 编译失败" -ForegroundColor Red
            exit 1
        }
        Write-Host ("  built {0}" -f $out) -ForegroundColor Green
        if (-not $NoVerify) { [void](Test-ArtifactAbi -ReadElf $ReadElf -Path $out -GlibcMax $TargetGlibcMax) }
        exit 0
    }

    'replay' {
        Write-Section '构建 replay'
        # 修复 .sh 版的两个缺陷：
        #   1) SMARTCAR_SRC 在 .sh 的 replay 分支从未定义（-u 下直接 unbound variable）
        #   2) .sh 的 replay LIBS 只有 -pthread，漏了 -ldl -lm（OpenCV 需要 dlopen@GLIBC_2.27）
        $out = Join-Path $BuildDir 'rewrite_video_replay'
        $includes = $Common + @("-I$(ConvertTo-GccPath $PlatformSrc)") + $OpenCvIncludes
        $libs = @('-pthread', '-ldl', '-lm') + $OpenCvLibs

        $argList = $includes + @(
            (Join-Path $AppSrc 'video_replay.cpp')
            (Join-Path $NavSrc 'path_params.cpp')
            (Join-Path $VisionSrc 'vision_pipeline.cpp')
            (Join-Path $NavSrc 'path_controller.cpp')
        ) + $libs + @('-o', (ConvertTo-GccPath $out))

        $text = & $Cxx @argList 2>&1 | Out-String
        if ($LASTEXITCODE -ne 0) {
            ($text -split "`r?`n" | Where-Object { $_ -match 'error' } | Select-Object -First 15) |
                ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
            Write-Host 'replay 编译失败' -ForegroundColor Red
            exit 1
        }
        Write-Host ("  built {0}" -f $out) -ForegroundColor Green
        if (-not $NoVerify) { [void](Test-ArtifactAbi -ReadElf $ReadElf -Path $out -GlibcMax $TargetGlibcMax) }
        exit 0
    }

    'target' {
        Write-Section '构建 target（增量）'

        $out = Join-Path $BuildDir 'lq_path_follow_rewrite'
        $objDir = Join-Path $BuildDir 'obj-target'
        New-Item -ItemType Directory -Force -Path $objDir | Out-Null

        $includes = $Common + $OpenCvIncludes
        $defines = @()
        $libs = @('-fopenmp', '-lgomp', '-pthread', '-ldl', '-lm')
        if ($ncnnRoot) {
            $defines += '-DREWRITE_WITH_NCNN'
            $includes += "-I$(ConvertTo-GccPath (Join-Path $ncnnRoot 'include'))"
            $libs += @("-L$(ConvertTo-GccPath (Join-Path $ncnnRoot 'lib'))", '-lncnn')
        }
        if ($ocvRoot) { $libs += $OpenCvLibs }

        $targetSrc = @(
            (Join-Path $AppSrc 'lq_path_follow.cpp')
            (Join-Path $TelemetrySrc 'status_display.cpp')
            (Join-Path $DisplaySrc 'st7735s.cpp')
            (Join-Path $DisplaySrc 'spi1_shared.cpp')
            (Join-Path $ImuSrc 'lsm6dsr_spi1.cpp')
        ) + $CoreSrc + @(
            (Join-Path $ImuSrc 'lq_lsm6dsr.cpp')
            (Join-Path $TofSrc 'lq_vl53l0x.cpp')
            (Join-Path $PlatformSrc 'hal.cpp')
            (Join-Path $PlatformSrc 'success_motor.cpp')
            (Join-Path $DriverSrc 'LQ_ATIM_PWM.cpp')
            (Join-Path $DriverSrc 'LQ_HW_ADC.cpp')
            (Join-Path $DriverSrc 'LQ_HW_GPIO.cpp')
            (Join-Path $DriverSrc 'LQ_MAP_ADDR.cpp')
        )

        # --- 参数指纹：变了就清空目标文件缓存（等价于 .sh 的 flags.sha256）
        $flagFile = Join-Path $objDir 'flags.sha256'
        $flagHash = Get-Sha256Hex ((@($cxxFull) + $defines + $includes + $libs + @('-fopenmp')) -join "`n")
        $cacheValid = (Test-Path -LiteralPath $flagFile) -and ((Get-Content -LiteralPath $flagFile -Raw).Trim() -eq $flagHash)
        if ($Force -or -not $cacheValid) {
            Get-ChildItem -LiteralPath $objDir -File -Filter '*.o' -ErrorAction SilentlyContinue | Remove-Item -Force
            Set-Content -LiteralPath $flagFile -Value $flagHash -NoNewline -Encoding ascii
        }

        # --- 头文件最新修改时间（一次性算好，比 .sh 每个文件 find 一遍快得多）
        $headerDirs = @($SrcRoot, $PlatformSrc, (Join-Path $DriverSrc 'inc'))
        $newestHeader = Get-ChildItem -LiteralPath $headerDirs -Recurse -File -ErrorAction SilentlyContinue |
            Where-Object { $_.Extension -in '.h', '.hpp' } |
            Sort-Object LastWriteTime -Descending | Select-Object -First 1
        $newestHeaderTime = if ($newestHeader) { $newestHeader.LastWriteTime } else { [datetime]::MinValue }

        # --- 决定哪些要重编
        $objects = [System.Collections.Generic.List[string]]::new()
        $todo = [System.Collections.Generic.List[object]]::new()

        foreach ($src in $targetSrc) {
            $rel = $src.Substring($RepoRoot.Length).TrimStart('\')
            $obj = Join-Path $objDir (($rel -replace '[\\/]', '__') -replace '\.cpp$', '.o')
            $objects.Add($obj)

            $need = $Force -or -not (Test-Path -LiteralPath $obj)
            if (-not $need) {
                $objTime = (Get-Item -LiteralPath $obj).LastWriteTime
                if ((Get-Item -LiteralPath $src).LastWriteTime -gt $objTime) { $need = $true }
                elseif ($newestHeaderTime -gt $objTime) { $need = $true }
            }
            if ($need) {
                $todo.Add(@{
                        Src     = $rel
                        ArgList = $includes + $defines + @(
                            '-fopenmp', '-c', (ConvertTo-GccPath $src), '-o', (ConvertTo-GccPath $obj)
                        )
                    })
            }
        }

        Write-Host ("  源文件 {0} 个，需重编 {1} 个" -f $targetSrc.Count, $todo.Count)
        if ($todo.Count) {
            $todo | ForEach-Object { Write-Host "    CXX $($_.Src)" -ForegroundColor DarkGray }
            $failed = @(Invoke-BuildBatch -Cxx $Cxx -Items $todo.ToArray() -Throttle $Jobs)
            if ($failed.Count) {
                Write-Host ("编译失败 {0} 个文件" -f $failed.Count) -ForegroundColor Red
                exit 1
            }
        }
        else {
            Write-Host '    全部命中缓存' -ForegroundColor DarkGray
        }

        # --- 链接
        $needLink = $Force -or -not (Test-Path -LiteralPath $out)
        if (-not $needLink) {
            $outTime = (Get-Item -LiteralPath $out).LastWriteTime
            $needLink = @($objects | Where-Object { (Get-Item -LiteralPath $_).LastWriteTime -gt $outTime }).Count -gt 0
        }
        if ($needLink) {
            Write-Host '    LINK build/rewrite/lq_path_follow_rewrite' -ForegroundColor DarkGray
            $linkArgs = @('-fopenmp') + ($objects | ForEach-Object { ConvertTo-GccPath $_ }) + $libs + @('-o', (ConvertTo-GccPath $out))
            $text = & $Cxx @linkArgs 2>&1 | Out-String
            if ($LASTEXITCODE -ne 0) {
                ($text -split "`r?`n" | Where-Object { $_ -match 'error|undefined' } | Select-Object -First 20) |
                    ForEach-Object { Write-Host "  $_" -ForegroundColor Red }
                Write-Host '链接失败' -ForegroundColor Red
                exit 1
            }
            Write-Host ("  built {0}" -f $out) -ForegroundColor Green
        }
        else {
            Write-Host ("  reused unchanged {0}" -f $out) -ForegroundColor DarkGray
        }

        if (-not $NoVerify) {
            if (-not (Test-ArtifactAbi -ReadElf $ReadElf -Path $out -GlibcMax $TargetGlibcMax)) { exit 1 }
        }
        exit 0
    }

    default {
        Write-Host "未知模式: $Mode" -ForegroundColor Red
        exit 2
    }
}
