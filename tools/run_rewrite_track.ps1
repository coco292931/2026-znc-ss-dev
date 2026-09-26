#!/usr/bin/env pwsh
<#
.SYNOPSIS
Start rewrite on the board, record telemetry automatically, and plot the run.

.EXAMPLE
.\tools\run_rewrite_track.ps1
.\tools\run_rewrite_track.ps1 -DryRun
.\tools\run_rewrite_track.ps1 -TargetPathDir /home/root -DryRun
#>
[CmdletBinding()]
param(
    [string]$BoardIP = "192.168.43.220",
    [string]$BoardUser = "root",
    [string]$IdentityFile = "",
    [string]$PythonExe = "",
    [string]$InertialFallbackPath = "",
    [string]$TargetPathDir = "",
    [ValidateSet("rewrite", "expt")]
    [string]$ControllerVariant = "rewrite",
    [ValidateSet("test", "track")]
    [string]$MachineProfile = "track",
    [ValidateSet("auto", "i2c", "spi")]
    [string]$ImuTransport = "auto",
    [ValidateRange(0.001, 1.0)]
    [double]$TargetCloseSize = 0.04,
    [ValidateRange(0.0, 1.0)]
    [double]$TargetPathTriggerY = 0.50,
    [string]$RemoteProgram = "",
    [switch]$DryRun,
    [ValidateRange(0, 86400)]
    [int]$AutoStopAfterSeconds = 0,
    [ValidateRange(5, 120)]
    [int]$StartupTimeoutSeconds = 20,
    [ValidateRange(20, 100)]
    [int]$ControlHz = 50,
    [ValidateRange(5, 180)]
    [double]$BaseSpeed = 70,
    [double]$MaxSpeed = 95,
    [ValidateRange(1.0, 2.5)]
    [double]$HardTurnOuterSpeedScale = 1.50,
    [ValidateRange(5, 100)]
    [double]$MaxPercent = 36,
    [ValidateRange(1, 200)]
    [double]$TargetAccel = 110,
    [ValidateRange(1, 300)]
    [double]$TargetDecel = 180,
    [ValidateRange(10, 1000)]
    [double]$PwmSlew = 180,
    [ValidateRange(100, 1000000)]
    [double]$PwmFrequencyHz = 1000,
    [ValidateRange(0, 3)]
    [double]$SpeedFf = 0.60,
    [ValidateRange(0, 5)]
    [double]$SpeedKp = 0.45,
    [ValidateRange(0, 5)]
    [double]$SpeedKi = 0.90,
    [ValidateRange(0, 36)]
    [double]$InnerBrake = 12,
    [ValidateRange(0, 100)]
    [double]$InnerBrakeMargin = 8,
    [ValidateRange(0.05, 2.0)]
    [double]$EncoderRatio = 0.40,
    [ValidateRange(0.05, 1.0)]
    [double]$EncoderFilterAlpha = 0.35,
    [double]$NearYawGain = 100,
    [double]$CenterYawWeight = 0.75,
    [double]$FarYawGain = 60,
    [double]$CurveYawBoost = 0.5,
    [ValidateRange(1, 5)]
    [double]$CurveYawShape = 2.0,
    [ValidateRange(0, 0.9)]
    [double]$CurvatureSlowdown = 0.0,
    [ValidateRange(0.1, 1.0)]
    [double]$MinFollowSpeed = 1.0,
    [ValidateRange(0.3, 1.0)]
    [double]$MinFollowYawScale = 0.80,
    [double]$MaxYawRate = 180,
    [ValidateRange(1, 5000)]
    [double]$TargetYawSlew = 600,
    [double]$YawRateKp = 0.30,
    [double]$YawRateLimit = 22,
    [ValidateRange(0, 1)]
    [double]$EncoderYawKpScale = 0.10,
    [ValidateRange(0, 0.1)]
    [double]$ImuAccelDeadband = 0.002,
    [ValidateSet(-1, 1)]
    [int]$ImuAccelForwardSign = 1,
    [ValidateSet(-1, 1)]
    [int]$ImuAccelRightSign = 1,
    [ValidateRange(0.001, 0.25)]
    [double]$ImuStationaryAccel = 0.02,
    [ValidateRange(0.1, 50)]
    [double]$ImuStationaryYaw = 2.5,
    [ValidateRange(0, 5)]
    [double]$ImuStationaryHold = 0.25,
    [switch]$ImuAccelSwapXY,
    [switch]$NoImu,
    [switch]$NoImuStationaryZero,
    [ValidateRange(0, 100)]
    [double]$VisionIGain = 2,
    [ValidateRange(0, 100)]
    [double]$VisionDGain = 2,
    [ValidateRange(0.5, 5.0)]
    [double]$VisionErrorStep = 2.0,
    [Alias("SteeringSlopeKp")]
    [ValidateRange(0, 500)]
    [double]$SteeringHeadingKp = 90.0,
    [ValidateRange(0, 500)]
    [double]$SteeringLateralKp = 110.0,
    [Alias("SteeringOffsetKi")]
    [ValidateRange(0, 500)]
    [double]$SteeringIntegralKi = 20.0,
    [Alias("SteeringAccelKd")]
    [ValidateRange(0, 10)]
    [double]$SteeringRateKd = 0.30,
    [ValidateRange(0.01, 2.0)]
    [double]$SteeringLateralScale = 0.25,
    [ValidateRange(0.01, 2.0)]
    [double]$SteeringLateralSoftening = 0.25,
    [ValidateRange(0.01, 10.0)]
    [double]$SteeringHeadingScale = 1.0,
    [ValidateRange(0, 2.0)]
    [double]$SteeringILimit = 0.08,
    [ValidateRange(0, 50)]
    [double]$SteeringIDecay = 1.5,
    [ValidateRange(0.001, 2.0)]
    [double]$SteeringDFilter = 0.06,
    [ValidateRange(0, 3)]
    [double]$SteeringCurvatureFf = 1.0,
    [ValidateRange(0, 1)]
    [double]$SteeringEncoderRateWeight = 0.25,
    [ValidateRange(0, 50)]
    [double]$RampBoost = 12,
    [ValidateRange(5, 180)]
    [double]$RampSpeed = 70,
    [ValidateRange(0.2, 10)]
    [double]$RampMaxSeconds = 2.5,
    [ValidateRange(20, 1000)]
    [double]$TofRampDelta = 400,
    [ValidateRange(1, 60)]
    [int]$CrossEnterFrames = 5,
    [ValidateRange(1, 30)]
    [int]$CrossExitFrames = 3,
    [ValidateRange(0, 200)]
    [double]$CrossMinDistance = 35,
    [ValidateRange(0.1, 10)]
    [double]$CrossMinTime = 0.8,
    [ValidateRange(0.2, 10)]
    [double]$CrossTimeout = 1.5,
    [ValidateRange(0.1, 1.0)]
    [double]$CrossSpeedScale = 0.70,
    [ValidateRange(0, 180)]
    [double]$CrossHeadingGrid = 90,
    [ValidateRange(0, 90)]
    [double]$CrossHeadingTolerance = 45,
    [ValidateRange(1, 360)]
    [double]$CrossEnterMaxYawRate = 45,
    [ValidateRange(0, 20)]
    [double]$HeadingHoldKp = 3.0,
    [ValidateRange(1, 360)]
    [double]$HeadingHoldMaxRate = 90,
    [ValidateRange(0, 20)]
    [int]$CrossCornerTolerance = 3,
    [ValidateRange(10, 300)]
    [double]$RoundEnterDistance = 35,
    [ValidateRange(0.05, 1.0)]
    [double]$RoundEnterMaxError = 0.25,
    [ValidateRange(0.05, 60.0)]
    [double]$TelemetryEchoInterval = 0.5,
    [switch]$NoTelemetryEcho,
    [switch]$NoSideRoad,
    [switch]$NoTofSlope,
    [switch]$NoStop,
    [switch]$KeepStraight,
    [switch]$SwapMotors,
    [switch]$SwapEncoders,
    [switch]$ValidateOnly
)

$ErrorActionPreference = "Stop"
$TurningParameters = [ordered]@{
    NearYawGain = $NearYawGain
    CenterYawWeight = $CenterYawWeight
    FarYawGain = $FarYawGain
    CurveYawBoost = $CurveYawBoost
    CurveYawShape = $CurveYawShape
    HardTurnOuterSpeedScale = $HardTurnOuterSpeedScale
    MinFollowYawScale = $MinFollowYawScale
    MaxYawRate = $MaxYawRate
    TargetYawSlew = $TargetYawSlew
    YawRateKp = $YawRateKp
    YawRateLimit = $YawRateLimit
    EncoderYawKpScale = $EncoderYawKpScale
}
foreach ($Entry in $TurningParameters.GetEnumerator()) {
    $Value = [double]$Entry.Value
    if ([double]::IsNaN($Value) -or [double]::IsInfinity($Value) -or
        $Value -lt 0) {
        throw "$($Entry.Key) must be a finite non-negative number."
    }
}
if ($MaxYawRate -le 0) {
    throw "MaxYawRate must be greater than zero."
}
if ($BaseSpeed -gt $MaxSpeed) {
    throw "BaseSpeed must not exceed MaxSpeed."
}
if ($CrossHeadingGrid -gt 0 -and
    $CrossHeadingTolerance -gt $CrossHeadingGrid / 2) {
    throw "CrossHeadingTolerance must not exceed half CrossHeadingGrid."
}
if (-not $NoTofSlope -and $RampSpeed -gt $MaxSpeed) {
    throw "RampSpeed must not exceed MaxSpeed."
}
$RepositoryRoot = Split-Path -Parent (Split-Path -Parent $PSCommandPath)
$DefaultInertialFallbackPath = Join-Path (
    Join-Path $RepositoryRoot "build\paths"
) "course.csv"
$UseTargetPaths = -not [string]::IsNullOrWhiteSpace($TargetPathDir)
$UseInertialFallback = -not $UseTargetPaths
$ExplicitInertialFallback = -not [string]::IsNullOrWhiteSpace(
    $InertialFallbackPath
)

function Test-InertialFallbackPath {
    param([Parameter(Mandatory = $true)][string]$Path)

    try {
        $CsvLines = @(
            Get-Content -LiteralPath $Path -Encoding UTF8 |
                Where-Object {
                    -not [string]::IsNullOrWhiteSpace($_) -and
                    -not $_.TrimStart().StartsWith("#")
                }
        )
        if ($CsvLines.Count -lt 3) {
            return $false
        }
        $Rows = @($CsvLines | ConvertFrom-Csv)
        if ($Rows.Count -lt 2 -or
            $Rows[0].PSObject.Properties.Name -notcontains "x_cm" -or
            $Rows[0].PSObject.Properties.Name -notcontains "y_cm") {
            return $false
        }
        $Culture = [Globalization.CultureInfo]::InvariantCulture
        $Style = [Globalization.NumberStyles]::Float
        $PreviousX = 0.0
        $PreviousY = 0.0
        $HavePrevious = $false
        $LengthCm = 0.0
        foreach ($Row in $Rows) {
            $X = 0.0
            $Y = 0.0
            if (-not [double]::TryParse(
                    [string]$Row.x_cm, $Style, $Culture, [ref]$X
                ) -or
                -not [double]::TryParse(
                    [string]$Row.y_cm, $Style, $Culture, [ref]$Y
                )) {
                return $false
            }
            if ($HavePrevious) {
                $LengthCm += [Math]::Sqrt(
                    [Math]::Pow($X - $PreviousX, 2) +
                    [Math]::Pow($Y - $PreviousY, 2)
                )
            }
            $PreviousX = $X
            $PreviousY = $Y
            $HavePrevious = $true
        }
        return $LengthCm -ge 1.0
    } catch {
        return $false
    }
}

if ($UseInertialFallback) {
    if ([string]::IsNullOrWhiteSpace($InertialFallbackPath)) {
        $InertialFallbackPath = $DefaultInertialFallbackPath
    }
    $FallbackExists = Test-Path `
        -LiteralPath $InertialFallbackPath -PathType Leaf
    $FallbackValid = $FallbackExists -and (
        Test-InertialFallbackPath -Path $InertialFallbackPath
    )
    if (-not $FallbackValid) {
        if ($ExplicitInertialFallback) {
            throw "Inertial fallback path is missing or has no measurable length: $InertialFallbackPath"
        }
        Write-Warning (
            "Default inertial fallback is missing or has no measurable " +
            "length; line loss will stop the vehicle: $InertialFallbackPath"
        )
        $UseInertialFallback = $false
    }
}
$BundledRecorder = Join-Path $PSScriptRoot "record_rewrite_trajectory.py"
$BundledPlotter = Join-Path $PSScriptRoot "plot_rewrite_trajectory.py"
$BundledDebugger = Join-Path $PSScriptRoot "debug_rewrite_telemetry.py"
$IsBundled = (
    (Test-Path -LiteralPath $BundledRecorder -PathType Leaf) -and
    (Test-Path -LiteralPath $BundledPlotter -PathType Leaf) -and
    (Test-Path -LiteralPath $BundledDebugger -PathType Leaf)
)
if ($IsBundled) {
    $WorkRoot = $PSScriptRoot
    $Recorder = $BundledRecorder
    $Plotter = $BundledPlotter
    $Debugger = $BundledDebugger
} else {
    $WorkRoot = $RepositoryRoot
    $Recorder = Join-Path $RepositoryRoot (
        "SmartCar\tools\record_rewrite_trajectory.py"
    )
    $Plotter = Join-Path $RepositoryRoot (
        "SmartCar\tools\plot_rewrite_trajectory.py"
    )
    $Debugger = Join-Path $RepositoryRoot (
        "tools\debug_rewrite_telemetry.py"
    )
}
$OutputDirectory = Join-Path $WorkRoot "build\trajectory"
$PythonCandidates = [System.Collections.Generic.List[string]]::new()
if (-not [string]::IsNullOrWhiteSpace($PythonExe)) {
    $PythonCandidates.Add($PythonExe)
}
foreach ($SystemPython in @(
    Get-Command python -All -ErrorAction SilentlyContinue
)) {
    $PythonCandidates.Add($SystemPython.Source)
}
foreach ($CommonPython in @(
    (Join-Path $env:USERPROFILE "Anaconda3\python.exe"),
    (Join-Path $env:USERPROFILE "Miniconda3\python.exe")
)) {
    if (Test-Path -LiteralPath $CommonPython -PathType Leaf) {
        $PythonCandidates.Add($CommonPython)
    }
}
$CodexPython = Join-Path $env:USERPROFILE (
    ".cache\codex-runtimes\codex-primary-runtime" +
    "\dependencies\python\python.exe"
)
if (Test-Path -LiteralPath $CodexPython -PathType Leaf) {
    $PythonCandidates.Add($CodexPython)
}
$Python = ""
foreach ($Candidate in $PythonCandidates | Select-Object -Unique) {
    if (-not (Test-Path -LiteralPath $Candidate -PathType Leaf)) {
        continue
    }
    # Windows PowerShell 5 can promote stderr from a failed native Python
    # probe to a terminating NativeCommandError while ErrorActionPreference
    # is Stop. Probe through System.Diagnostics.Process so an unusable
    # candidate is just skipped and later Python installations are tried.
    $ProbeInfo = New-Object System.Diagnostics.ProcessStartInfo
    $ProbeInfo.FileName = $Candidate
    $ProbeInfo.Arguments = '-c "import matplotlib"'
    $ProbeInfo.UseShellExecute = $false
    $ProbeInfo.CreateNoWindow = $true
    $ProbeInfo.RedirectStandardOutput = $true
    $ProbeInfo.RedirectStandardError = $true
    $Probe = New-Object System.Diagnostics.Process
    $Probe.StartInfo = $ProbeInfo
    $ProbeStarted = $false
    $ProbeExitCode = -1
    try {
        $ProbeStarted = $Probe.Start()
        if ($ProbeStarted) {
            $Probe.StandardOutput.ReadToEnd() | Out-Null
            $Probe.StandardError.ReadToEnd() | Out-Null
            $Probe.WaitForExit()
            $ProbeExitCode = $Probe.ExitCode
        }
    } catch {
        $ProbeStarted = $false
    } finally {
        $Probe.Dispose()
    }
    if ($ProbeStarted -and $ProbeExitCode -eq 0) {
        $Python = $Candidate
        break
    }
}
if ([string]::IsNullOrWhiteSpace($Python)) {
    throw (
        "No Python with matplotlib was found. Install it with " +
        "'python -m pip install matplotlib' or pass -PythonExe <path>."
    )
}

foreach ($Required in @($Recorder, $Plotter, $Debugger)) {
    if (-not (Test-Path -LiteralPath $Required -PathType Leaf)) {
        throw "Missing required tool: $Required"
    }
}

New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$Stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$Csv = Join-Path $OutputDirectory "${ControllerVariant}_$Stamp.csv"
$RecorderOut = Join-Path $OutputDirectory (
    "${ControllerVariant}_$Stamp.record.out.log"
)
$RecorderErr = Join-Path $OutputDirectory (
    "${ControllerVariant}_$Stamp.record.err.log"
)
$RecorderStop = Join-Path $OutputDirectory (
    "${ControllerVariant}_$Stamp.record.stop"
)
Remove-Item -LiteralPath $RecorderStop -Force -ErrorAction SilentlyContinue
$TelemetryUrl = "http://${BoardIP}:8080/telemetry"
$Remote = "${BoardUser}@${BoardIP}"

if ([string]::IsNullOrWhiteSpace($IdentityFile)) {
    $DefaultIdentity = Join-Path $env:USERPROFILE ".ssh\id_rsa"
    if (Test-Path -LiteralPath $DefaultIdentity -PathType Leaf) {
        $IdentityFile = $DefaultIdentity
    }
}

$SshArgs = @(
    "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=8",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "HostKeyAlgorithms=+ssh-rsa",
    "-o", "PubkeyAcceptedAlgorithms=+ssh-rsa"
)
if (-not [string]::IsNullOrWhiteSpace($IdentityFile)) {
    if (-not (Test-Path -LiteralPath $IdentityFile -PathType Leaf)) {
        throw "SSH identity file not found: $IdentityFile"
    }
    $SshArgs += @("-i", $IdentityFile, "-o", "IdentitiesOnly=yes")
}

if ($ControllerVariant -eq "expt") {
    if ([string]::IsNullOrWhiteSpace($RemoteProgram)) {
        $RemoteProgram = "/home/root/expt/lq_path_follow_expt"
    }
    $RemoteWorkDirectory = "/home/root/expt"
    $RemoteModelDirectory = "/home/root/expt/models"
    $CalibrationPath = "/home/root/expt/calibration.txt"
    $RemoteSetupScript = "/home/root/expt/setup_st7735s_spi.sh"
    $RemoteLogPath = "/home/root/expt/lq_path_follow_expt.log"
    $RemotePidPath = "/home/root/expt/lq_path_follow_expt.pid"
    $RemoteInertialFallbackPath = "/home/root/expt/inertial_fallback.csv"
} else {
    if ([string]::IsNullOrWhiteSpace($RemoteProgram)) {
        $RemoteProgram = "/home/root/lq_path_follow_imu"
    }
    $CalibrationName = -join @(
        [char]0x6807,
        [char]0x5B9A,
        [char]0x6570,
        [char]0x636E
    ) + ".txt"
    $RemoteWorkDirectory = "/home/root"
    $RemoteModelDirectory = "/home/root/models"
    $CalibrationPath = "/home/root/rewrite/$CalibrationName"
    $RemoteSetupScript = "/home/root/setup_st7735s_spi.sh"
    $RemoteLogPath = "/home/root/lq_path_follow_rewrite.log"
    $RemotePidPath = "/home/root/lq_path_follow_rewrite.pid"
    $RemoteInertialFallbackPath = "/home/root/rewrite_inertial_fallback.csv"
}
$ModeFlag = if ($DryRun) { "--dry-run" } else { "--enable-motors" }
if ($ControllerVariant -eq "expt") {
    if ($MachineProfile -eq "test") {
        $ImuTransport = "i2c"
        $NoTofSlope = $true
    } else {
        $ImuTransport = "spi"
    }
} else {
    $ImuTransport = "auto"
}
$SideRoadArguments = if ($NoSideRoad) {
    @()
} else {
    @(
        "--side-road",
        "--no-side-require-tof",
        "--side-enter-frames", "3",
        "--round-enter-distance", "$RoundEnterDistance",
        "--round-enter-max-error", "$RoundEnterMaxError",
        "--side-start-distance", "100"
    )
}
$TofArguments = if ($NoTofSlope) {
    @()
} else {
    @(
        "--tof-slope",
        "--tof-baseline-timeout", "2",
        "--tof-ramp-sign", "-1",
        "--tof-ramp-delta", "$TofRampDelta",
        "--tof-ramp-hyst", "35",
        "--tof-ramp-enter", "2",
        "--tof-ramp-exit", "5",
        "--ramp-boost", "$RampBoost",
        "--ramp-boost-speed", "$RampSpeed",
        "--ramp-boost-max", "$RampMaxSeconds"
    )
}
$NavigationArguments = if ($UseTargetPaths) {
    @(
        "--target-actions",
        "--target-input-size", "32",
        "--target-path-dir", $TargetPathDir,
        "--target-close-size", "$TargetCloseSize",
        "--target-path-trigger-y", "$TargetPathTriggerY"
    )
} elseif ($UseInertialFallback) {
    @(
        "--target-actions",
        "--target-input-size", "32",
        "--target-close-size", "$TargetCloseSize",
        "--inertial-path", $RemoteInertialFallbackPath,
        "--line-lost-inertial"
    )
} else {
    @(
        "--target-actions",
        "--target-input-size", "32",
        "--target-close-size", "$TargetCloseSize"
    )
}
$StopArguments = if ($NoStop) { @("--no-stop") } else { @() }
if ($KeepStraight) {
    if ($ControllerVariant -ne "expt") {
        throw "KeepStraight is only supported by the expt controller."
    }
    $StopArguments += "--keep-straight"
}
$ImuArguments = @(
    "--imu-transport", "$ImuTransport",
    "--imu-accel-forward-sign", "$ImuAccelForwardSign",
    "--imu-accel-right-sign", "$ImuAccelRightSign",
    "--imu-accel-deadband", "$ImuAccelDeadband",
    "--imu-stationary-accel", "$ImuStationaryAccel",
    "--imu-stationary-yaw", "$ImuStationaryYaw",
    "--imu-stationary-hold", "$ImuStationaryHold",
    $(if ($NoImuStationaryZero) {
        "--no-imu-stationary-zero"
    } else {
        "--imu-stationary-zero"
    })
)
if ($ImuAccelSwapXY) {
    $ImuArguments += "--imu-accel-swap-xy"
}
if ($NoImu) {
    $ImuArguments += "--no-imu"
}
$ControllerArguments = if ($ControllerVariant -eq "expt") {
    @(
        "--steering-heading-kp", "$SteeringHeadingKp",
        "--steering-lateral-kp", "$SteeringLateralKp",
        "--steering-integral-ki", "$SteeringIntegralKi",
        "--steering-rate-kd", "$SteeringRateKd",
        "--steering-lateral-scale", "$SteeringLateralScale",
        "--steering-lateral-softening", "$SteeringLateralSoftening",
        "--steering-heading-scale", "$SteeringHeadingScale",
        "--steering-i-limit", "$SteeringILimit",
        "--steering-i-decay", "$SteeringIDecay",
        "--steering-d-filter", "$SteeringDFilter",
        "--steering-curvature-ff", "$SteeringCurvatureFf",
        "--steering-encoder-rate-weight", "$SteeringEncoderRateWeight"
    )
} else {
    @()
}

# Cross entry requires paired corners, a symmetric widened opening, and
# consecutive controller confirmation.
$ProgramArguments = @(
    "--camera", "/dev/video0",
    "--control-hz", "$ControlHz",
    "--model-dir", $RemoteModelDirectory,
    "--calibration", $CalibrationPath,
    "--http", "8080",
    "--http-fps", "20",
    "--telemetry-hz", "20",
    "--display",
    "--display-spi", "/dev/spidev1.0",
    "--display-spi-speed", "8000000",
    "--display-hz", "4",
    $ModeFlag,
    "--base-speed", "$BaseSpeed",
    "--max-speed", "$MaxSpeed",
    "--hard-turn-outer-scale", "$HardTurnOuterSpeedScale",
    "--max-percent", "$MaxPercent",
    "--target-accel", "$TargetAccel",
    "--target-decel", "$TargetDecel",
    "--pwm-slew", "$PwmSlew",
    "--pwm-freq", "$PwmFrequencyHz",
    "--speed-ff", "$SpeedFf",
    "--speed-kp", "$SpeedKp",
    "--speed-ki", "$SpeedKi",
    "--inner-brake", "$InnerBrake",
    "--inner-brake-margin", "$InnerBrakeMargin",
    "--vision-yaw-sign", "-1",
    "--vision-i-gain", "$VisionIGain",
    "--vision-i-limit", "0.35",
    "--vision-i-max-error", "0.28",
    "--vision-i-curve-delta", "0.14",
    "--vision-d-gain", "$VisionDGain",
    "--vision-d-filter", "0.16",
    "--vision-d-max-rate", "0.90",
    "--vision-error-step", "$VisionErrorStep",
    "--near-yaw-gain", "$NearYawGain",
    "--center-yaw-weight", "$CenterYawWeight",
    "--far-yaw-gain", "$FarYawGain",
    "--curve-yaw-boost", "$CurveYawBoost",
    "--curve-yaw-shape", "$CurveYawShape",
    "--curvature-slowdown", "$CurvatureSlowdown",
    "--min-follow-speed", "$MinFollowSpeed",
    "--min-follow-yaw-scale", "$MinFollowYawScale",
    "--max-yaw-rate", "$MaxYawRate",
    "--target-yaw-slew", "$TargetYawSlew",
    "--yaw-rate-kp", "$YawRateKp",
    "--yaw-rate-limit", "$YawRateLimit",
    "--encoder-yaw-filter", "0.20",
    "--encoder-yaw-kp-scale", "$EncoderYawKpScale",
    "--encoder-yaw-limit", "5.0",
    "--imu-transport", "spi",
    "--imu-yaw-sign", "-1",
    "--wheel-diameter", "6.5",
    "--wheel-base", "15.3",
    "--encoder-lines", "1024",
    "--encoder-ratio", "$EncoderRatio",
    "--encoder-filter-alpha", "$EncoderFilterAlpha",
    $(if ($SwapMotors) { "--swap-motors" } else { "--no-swap-motors" }),
    $(if ($SwapEncoders) { "--swap-encoders" } else { "--no-swap-encoders" }),
    "--control-distance", "60",
    "--far-distance", "120",
    "--horizon-row", "11",
    "--threshold-floor", "70",
    "--wheel-box-center", "0.548",
    "--wheel-box-width", "0.190",
    "--wheel-box-top", "0.730",
    "--wheel-box-bottom", "1.000",
    "--edge-smooth", "1",
    "--bottom-fit-rows", "12",
    "--cross-enter", "$CrossEnterFrames",
    "--cross-exit", "$CrossExitFrames",
    "--cross-min-distance", "$CrossMinDistance",
    "--cross-min-time", "$CrossMinTime",
    "--cross-timeout", "$CrossTimeout",
    "--cross-speed-scale", "$CrossSpeedScale",
    "--cross-heading-grid", "$CrossHeadingGrid",
    "--cross-heading-tol", "$CrossHeadingTolerance",
    "--cross-enter-max-yaw", "$CrossEnterMaxYawRate",
    "--heading-hold-kp", "$HeadingHoldKp",
    "--heading-hold-max-rate", "$HeadingHoldMaxRate",
    "--cross-corner-tol", "$CrossCornerTolerance",
    "--branch-window", "3",
    "--branch-min-rows", "4",
    "--branch-return-rows", "3",
    "--branch-slope", "0.80",
    "--branch-dev", "8.0",
    "--branch-return", "3.0",
    "--lock-slope-tol", "0.20"
) + $ControllerArguments + $ImuArguments + $SideRoadArguments +
    $TofArguments + $StopArguments + $NavigationArguments
$RemoteArguments = $ProgramArguments -join " "
$CameraSelection = 'camera=/dev/video0'
if ($ControllerVariant -eq "expt") {
    $CameraSelector = Join-Path $RepositoryRoot "expt/select_camera.sh"
    $CameraSelection = (Get-Content -LiteralPath $CameraSelector -Raw -Encoding UTF8) + @'

camera=$(expt_select_camera /dev/video0) || exit $?
printf '[Camera] %s\n' "$camera"
'@
    $RemoteArguments = $RemoteArguments.Replace('--camera /dev/video0', '--camera "$camera"')
}
$RemoteScript = @'
set -e

# A previous program or camera diagnostic can keep /dev/video0 open while its
# threads are shutting down. Stop only known camera competitors, then wait for
# their teardown before checking the device.
camera_competitors="lq_path_follow_rewrite lq_path_follow_expt lq_path_follow_imu lq_path_follow lq_camera_yuv_test lq_camera_simple_test smartcar_camera_diag smartcar_vision_debug lq_marker_recognize"
for competitor in $camera_competitors; do
  killall "$competitor" 2>/dev/null || true
done
count=0
while true; do
  camera_busy=false
  for competitor in $camera_competitors; do
    if pidof "$competitor" >/dev/null 2>&1; then
      camera_busy=true
      break
    fi
  done
  if [ "$camera_busy" = false ]; then
    break
  fi
  count=$((count + 1))
  if [ "$count" -ge 60 ]; then
    for competitor in $camera_competitors; do
      killall -KILL "$competitor" 2>/dev/null || true
    done
    break
  fi
  sleep 0.1
done

__CAMERA_SELECTION__
camera_users=$(fuser "$camera" 2>/dev/null || true)
if [ -n "$camera_users" ]; then
  echo "Camera $camera is still busy; owner PIDs: $camera_users" >&2
  for owner_pid in $camera_users; do
    if [ -r "/proc/$owner_pid/cmdline" ]; then
      tr '\0' ' ' < "/proc/$owner_pid/cmdline" >&2
      echo >&2
    fi
  done
  exit 20
fi

if [ ! -x "__REMOTE_SETUP__" ]; then
  echo "Missing executable SPI1 setup: __REMOTE_SETUP__" >&2
  exit 23
fi
"__REMOTE_SETUP__"

program="__REMOTE_PROGRAM__"
if [ ! -x "$program" ]; then
  echo "Missing executable rewrite program: $program" >&2
  exit 22
fi
if [ "__REQUIRE_TARGET_PATHS__" = "1" ] &&
   ! env LD_LIBRARY_PATH=/home/root/LQ_Dep_libs/opencv-lib:/home/root/LQ_Dep_libs/ncnn-lib \
     "$program" --help | grep -q -- '--target-path-dir'; then
  echo "Rewrite program does not support --target-path-dir: $program" >&2
  exit 22
fi

cd "__REMOTE_WORK_DIR__"
: > "__REMOTE_LOG__"
nohup env LD_LIBRARY_PATH=/home/root/LQ_Dep_libs/opencv-lib:/home/root/LQ_Dep_libs/ncnn-lib \
  "$program" __PROGRAM_ARGUMENTS__ \
  > "__REMOTE_LOG__" 2>&1 </dev/null &
pid=$!
echo "$pid" > "__REMOTE_PID__"
sleep 0.5
if ! kill -0 "$pid" 2>/dev/null; then
  echo "Board program exited during startup. Program log:" >&2
  tail -n 120 "__REMOTE_LOG__" >&2 || true
  exit 21
fi
'@
$RemoteScript = $RemoteScript.Replace("__CAMERA_SELECTION__", $CameraSelection)
$RemoteScript = $RemoteScript.Replace(
    "__PROGRAM_ARGUMENTS__",
    $RemoteArguments
)
$RemoteScript = $RemoteScript.Replace("__REMOTE_PROGRAM__", $RemoteProgram)
$RemoteScript = $RemoteScript.Replace("__REMOTE_SETUP__", $RemoteSetupScript)
$RemoteScript = $RemoteScript.Replace(
    "__REMOTE_WORK_DIR__", $RemoteWorkDirectory
)
$RemoteScript = $RemoteScript.Replace("__REMOTE_LOG__", $RemoteLogPath)
$RemoteScript = $RemoteScript.Replace("__REMOTE_PID__", $RemotePidPath)
$RemoteScript = $RemoteScript.Replace(
    "__REQUIRE_TARGET_PATHS__",
    $(if ($UseTargetPaths) { "1" } else { "0" })
)
$RemoteScript = $RemoteScript.Replace("`r`n", "`n")
$RemoteScriptBase64 = [Convert]::ToBase64String(
    [System.Text.Encoding]::UTF8.GetBytes($RemoteScript)
)
# Passing a multiline shell program directly through Windows OpenSSH can
# consume its embedded quotes. Base64 keeps the POSIX script byte-for-byte.
$RemoteCommand = "echo $RemoteScriptBase64 | base64 -d | sh"

$RecorderArguments = @(
    $Recorder,
    "--url", $TelemetryUrl,
    "--output", $Csv,
    "--reconnect-delay", "0.25",
    "--timeout", "4",
    "--gap-threshold", "0.25",
    "--flush-every", "5",
    "--stop-file", $RecorderStop
)
$EchoArguments = @(
    $Debugger,
    "--url", $TelemetryUrl,
    "--interval", "$TelemetryEchoInterval",
    "--stop-file", $RecorderStop
)

if ($ValidateOnly) {
    Write-Host "Validation successful." -ForegroundColor Green
    Write-Host "Plot Python: $Python"
    Write-Host "Board: $Remote"
    Write-Host "Controller: $ControllerVariant"
    Write-Host "Remote program: $RemoteProgram"
    Write-Host "Mode: $ModeFlag"
    Write-Host "Telemetry: $TelemetryUrl"
    if ($UseInertialFallback) {
        Write-Host "Line loss: inertial takeover using $InertialFallbackPath"
    } elseif ($NoStop) {
        Write-Host "Line loss: visual recovery (STOP latch disabled)"
    } else {
        Write-Host "Line loss: visual stop"
    }
    Write-Host (
        "Yaw PID: vision_sign=-1 near=$NearYawGain " +
        "center=${CenterYawWeight}x far=$FarYawGain " +
        "curve=$CurveYawBoost shape=$CurveYawShape step=$VisionErrorStep px " +
        "slowdown=$CurvatureSlowdown " +
        "min_speed=$MinFollowSpeed min_yaw=$MinFollowYawScale " +
        "i=$VisionIGain/0.35 d=$VisionDGain tau=0.16 " +
        "max=$MaxYawRate slew=$TargetYawSlew deg/s2 rate_kp=$YawRateKp " +
        "rate_limit=$YawRateLimit cm/s; " +
        "encoder tau=0.20s gainx$EncoderYawKpScale limit=5cm/s"
    )
    Write-Host "IMU: SPI SCK=60 MOSI=62 MISO=61 CS=25 sign=-1"
    Write-Host (
        "Mapping: motors=$(if ($SwapMotors) { 'swapped' } else { 'normal' }) " +
        "encoders=$(if ($SwapEncoders) { 'swapped' } else { 'normal' }) " +
        "(PIN67 left, PIN65 right)"
    )
    if ($ControllerVariant -eq "expt") {
        Write-Host (
            "Geometry PID: heading=$SteeringHeadingKp " +
            "lateral=$SteeringLateralKp integral=$SteeringIntegralKi " +
            "rate=$SteeringRateKd curvature_ff=$SteeringCurvatureFf " +
            "encoder_rate_weight=$SteeringEncoderRateWeight " +
            "i_limit=$SteeringILimit i_decay=$SteeringIDecay " +
            "d_filter=$SteeringDFilter"
        )
        Write-Host (
            "Machine profile: $MachineProfile IMU=$ImuTransport " +
            "TOF=$(if ($NoTofSlope) { 'disabled' } else { 'enabled' })"
        )
    }
    Write-Host (
        "Wheel: speed=$BaseSpeed/$MaxSpeed cm/s " +
        "hard_outer=$HardTurnOuterSpeedScale" + "x/" +
        "$($MaxSpeed * $HardTurnOuterSpeedScale)cm/s " +
        "ff=$SpeedFf kp=$SpeedKp ki=$SpeedKi accel=$TargetAccel " +
        "control=$ControlHz Hz pwm=$PwmFrequencyHz Hz " +
        "pwm_slew=$PwmSlew%/s encoder_filter=$EncoderFilterAlpha " +
        "inner_brake=$InnerBrake% margin=$InnerBrakeMargin cm/s " +
        "max_pwm=$MaxPercent encoder_ratio=$EncoderRatio " +
        "reverse=forbidden"
    )
    Write-Host "Side road: $(-not $NoSideRoad)"
    Write-Host (
        "Line-loss STOP: " +
        $(if ($NoStop) { "disabled" } else { "enabled" })
    )
    Write-Host (
        "Cross: enter/exit=$CrossEnterFrames/$CrossExitFrames frames " +
        "corner_tol=$CrossCornerTolerance distance=$CrossMinDistance cm " +
        "min_time=$CrossMinTime s timeout=$CrossTimeout s speed=$CrossSpeedScale " +
        "heading_kp=$HeadingHoldKp rate<=$HeadingHoldMaxRate dps " +
        "grid=$CrossHeadingGrid+/-$CrossHeadingTolerance deg " +
        "enter_yaw<=$CrossEnterMaxYawRate dps"
    )
    Write-Host "TOF slope: $(-not $NoTofSlope)"
    Write-Host "SPI1 setup: $RemoteSetupScript"
    Write-Host "TFT state/recognition/action display: enabled"
    Write-Host "Local DATA/NAV echo: $(-not $NoTelemetryEcho)"
    if (-not $NoTofSlope) {
        Write-Host (
            "TOF ramp boost: distance<=$TofRampDelta mm " +
            "speed=$RampSpeed cm/s extra=$RampBoost% yaw<=25dps " +
            "pwm<=36% wheel_split<=10cm/s wheel_sync<=4% " +
            "max=$RampMaxSeconds s"
        )
    }
    if ($UseTargetPaths) {
        Write-Host (
            "Target inertial routes: dir=$TargetPathDir " +
            "close=$TargetCloseSize trigger_y=$TargetPathTriggerY"
        )
    } elseif ($UseInertialFallback) {
        Write-Host "Line-loss inertial fallback: $InertialFallbackPath"
    } else {
        Write-Host "Line-loss inertial fallback: disabled"
    }
    Write-Host "Startup timeout: $StartupTimeoutSeconds seconds"
    Write-Host "Packaged tools: $IsBundled"
    Write-Host "CSV: $Csv"
    exit 0
}

$RecorderProcess = $null
$EchoProcess = $null
$BoardStarted = $false
try {
    if ($UseInertialFallback) {
        Write-Host "Uploading inertial fallback path..." -ForegroundColor Cyan
        & scp.exe @SshArgs $InertialFallbackPath (
            "${Remote}:$RemoteInertialFallbackPath"
        )
        if ($LASTEXITCODE -ne 0) {
            throw "Inertial fallback path upload failed (SCP exit $LASTEXITCODE)."
        }
    }

    Write-Host "Starting automatic trajectory recorder..." -ForegroundColor Cyan
    $RecorderProcess = Start-Process `
        -FilePath $Python `
        -ArgumentList $RecorderArguments `
        -WorkingDirectory $WorkRoot `
        -RedirectStandardOutput $RecorderOut `
        -RedirectStandardError $RecorderErr `
        -WindowStyle Hidden `
        -PassThru

    Write-Host "Starting ${Remote} (${ModeFlag})..." -ForegroundColor Cyan
    & ssh.exe @SshArgs $Remote $RemoteCommand
    if ($LASTEXITCODE -ne 0) {
        throw "Board start failed (SSH exit $LASTEXITCODE)."
    }
    $BoardStarted = $true

    if (-not $NoTelemetryEcho) {
        Write-Host "Starting local DATA/NAV telemetry echo..." `
            -ForegroundColor Cyan
        $EchoProcess = Start-Process `
            -FilePath $Python `
            -ArgumentList $EchoArguments `
            -WorkingDirectory $WorkRoot `
            -NoNewWindow `
            -PassThru
    }

    Write-Host (
        "Waiting up to $StartupTimeoutSeconds seconds for first telemetry " +
        "(VL53L0X cold calibration may take over 10 seconds)..."
    )
    $Deadline = (Get-Date).AddSeconds($StartupTimeoutSeconds)
    while ((Get-Date) -lt $Deadline) {
        if ((Test-Path -LiteralPath $Csv) -and
            (Get-Item -LiteralPath $Csv).Length -gt 0) {
            break
        }
        if ($RecorderProcess.HasExited) {
            throw "Trajectory recorder exited before receiving telemetry."
        }
        Start-Sleep -Milliseconds 250
    }
    if (-not (Test-Path -LiteralPath $Csv) -or
        (Get-Item -LiteralPath $Csv).Length -eq 0) {
        Write-Warning "Board startup diagnostics:"
        $DiagnosticCommand = @'
echo "----- process -----"
if [ -f "__REMOTE_PID__" ]; then
  pid=$(cat "__REMOTE_PID__")
  echo "pid=$pid"
  kill -0 "$pid" 2>/dev/null && echo "running=yes" || echo "running=no"
else
  echo "pid file missing"
fi
echo "----- port 8080 -----"
netstat -lnt 2>/dev/null | grep ':8080' || echo "not listening"
echo "----- program log -----"
tail -n 100 "__REMOTE_LOG__" 2>/dev/null ||
  echo "program log missing"
'@
        $DiagnosticCommand = $DiagnosticCommand.Replace(
            "__REMOTE_PID__", $RemotePidPath
        ).Replace("__REMOTE_LOG__", $RemoteLogPath)
        & ssh.exe @SshArgs $Remote $DiagnosticCommand
        if (Test-Path -LiteralPath $RecorderErr -PathType Leaf) {
            Write-Warning "Recorder diagnostics:"
            Get-Content -LiteralPath $RecorderErr -Tail 40
        }
        throw (
            "No telemetry received from $TelemetryUrl within " +
            "$StartupTimeoutSeconds seconds. See diagnostics above."
        )
    }

    Write-Host ""
    Write-Host "Vehicle running; trajectory recording is active." `
        -ForegroundColor Green
    if ($AutoStopAfterSeconds -gt 0) {
        Write-Host "Automatic stop in $AutoStopAfterSeconds seconds."
        Start-Sleep -Seconds $AutoStopAfterSeconds
    } else {
        [void](Read-Host "Press Enter to stop the vehicle and finish the map")
    }
} finally {
    if ($BoardStarted) {
        Write-Host "Stopping board process..." -ForegroundColor Yellow
        $StopScript = @'
if [ -f "__REMOTE_PID__" ]; then
  pid=$(cat "__REMOTE_PID__")
  kill -INT "$pid" 2>/dev/null || true
  count=0
  while kill -0 "$pid" 2>/dev/null && [ "$count" -lt 50 ]; do
    sleep 0.1
    count=$((count + 1))
  done
  kill -TERM "$pid" 2>/dev/null || true
fi
'@
        $StopScript = $StopScript.Replace("__REMOTE_PID__", $RemotePidPath)
        $StopScript = $StopScript.Replace("`r`n", "`n")
        $StopScriptBase64 = [Convert]::ToBase64String(
            [System.Text.Encoding]::UTF8.GetBytes($StopScript)
        )
        $StopCommand = "echo $StopScriptBase64 | base64 -d | sh"
        & ssh.exe @SshArgs $Remote $StopCommand
    }

    if (($null -ne $RecorderProcess -and -not $RecorderProcess.HasExited) -or
        ($null -ne $EchoProcess -and -not $EchoProcess.HasExited)) {
        [System.IO.File]::WriteAllText($RecorderStop, "stop")
    }
    if ($null -ne $RecorderProcess -and -not $RecorderProcess.HasExited) {
        if (-not $RecorderProcess.WaitForExit(10000)) {
            Stop-Process -Id $RecorderProcess.Id -Force `
                -ErrorAction SilentlyContinue
        }
    }
    if ($null -ne $EchoProcess -and -not $EchoProcess.HasExited) {
        if (-not $EchoProcess.WaitForExit(5000)) {
            Stop-Process -Id $EchoProcess.Id -Force `
                -ErrorAction SilentlyContinue
        }
    }
    Remove-Item -LiteralPath $RecorderStop -Force -ErrorAction SilentlyContinue
}

if (-not (Test-Path -LiteralPath $Csv -PathType Leaf)) {
    throw "Trajectory CSV was not created. Recorder log: $RecorderErr"
}

Write-Host "Rendering trajectory..." -ForegroundColor Cyan
& $Python $Plotter $Csv --arrow-every 20
if ($LASTEXITCODE -ne 0) {
    throw "Trajectory plotting failed."
}

$Png = [System.IO.Path]::ChangeExtension($Csv, ".png")
$Summary = Join-Path (
    Split-Path -Parent $Csv
) "$([System.IO.Path]::GetFileNameWithoutExtension($Csv))_summary.txt"
$TofEvents = Join-Path (
    Split-Path -Parent $Csv
) "$([System.IO.Path]::GetFileNameWithoutExtension($Csv))_tof_events.csv"
Write-Host ""
Write-Host "CSV: $Csv" -ForegroundColor Green
Write-Host "PNG: $Png"
Write-Host "Summary: $Summary"
Write-Host "TOF events: $TofEvents"
