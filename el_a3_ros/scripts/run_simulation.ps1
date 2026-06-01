param(
    [switch]$DirectRviz,
    [switch]$NoBuild,
    [switch]$NoRviz,
    [switch]$NoAutoMotion,
    [switch]$Once,
    [switch]$ManualControl,
    [switch]$IKControl,
    [double]$SpeedScale = 1.0,
    [string]$WristMotorType = "EL05",
    [string]$RosSetup = "",
    [string]$CondaPrefix = ""
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RosWs = Resolve-Path (Join-Path $ScriptDir "..")
Set-Location $RosWs

function Invoke-SetupScript {
    param([string]$Path)
    if (-not (Test-Path $Path)) {
        throw "ROS setup script not found: $Path"
    }

    $cmd = "`"$Path`" && set"
    $lines = & cmd.exe /c $cmd
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to source setup script: $Path"
    }

    foreach ($line in $lines) {
        $idx = $line.IndexOf("=")
        if ($idx -gt 0) {
            $name = $line.Substring(0, $idx)
            $value = $line.Substring($idx + 1)
            [Environment]::SetEnvironmentVariable($name, $value, "Process")
        }
    }
}

function Add-CondaPrefixToPath {
    param([string]$Prefix)
    if (-not $Prefix) {
        return
    }
    $paths = @(
        $Prefix,
        (Join-Path $Prefix "Scripts"),
        (Join-Path $Prefix "Library\bin"),
        (Join-Path $Prefix "Library\usr\bin")
    ) | Where-Object { Test-Path $_ }
    $env:PATH = ($paths -join ";") + ";" + $env:PATH
    $env:CONDA_PREFIX = $Prefix
}

function Find-RosSetup {
    if ($RosSetup) {
        return $RosSetup
    }
    if ($CondaPrefix) {
        $candidate = Join-Path $CondaPrefix "Library\local_setup.bat"
        if (Test-Path $candidate) {
            return $candidate
        }
    }
    if ($env:CONDA_PREFIX) {
        $candidate = Join-Path $env:CONDA_PREFIX "Library\local_setup.bat"
        if (Test-Path $candidate) {
            return $candidate
        }
    }
    foreach ($candidate in @(
        "C:\dev\ros2_humble\local_setup.bat",
        "C:\opt\ros\humble\x64\local_setup.bat",
        "C:\ros2_humble\local_setup.bat"
    )) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }
    return ""
}

$detectedPrefix = ""
if ($CondaPrefix) {
    $detectedPrefix = $CondaPrefix
} elseif ($env:CONDA_PREFIX) {
    $detectedPrefix = $env:CONDA_PREFIX
}
Add-CondaPrefixToPath $detectedPrefix

$setup = Find-RosSetup
if (-not $setup) {
    throw @"
No native Windows ROS 2 setup script was found.

Pass one explicitly:
  powershell -ExecutionPolicy Bypass -File scripts\run_simulation.ps1 -RosSetup C:\path\to\local_setup.bat

For a RoboStack/conda environment, activate it first or pass:
  -CondaPrefix C:\path\to\env
"@
}

Write-Host "Using ROS setup: $setup"
$setupParent = Split-Path -Parent $setup
if ((Split-Path -Leaf $setupParent) -eq "Library") {
    Add-CondaPrefixToPath (Split-Path -Parent $setupParent)
}
Invoke-SetupScript $setup

if (-not (Get-Command ros2 -ErrorAction SilentlyContinue)) {
    throw "ros2 was not found after sourcing $setup"
}

if ($DirectRviz) {
    $prefixForDirect = if ($CondaPrefix) { $CondaPrefix } elseif ($env:CONDA_PREFIX) { $env:CONDA_PREFIX } else { Split-Path -Parent (Split-Path -Parent $setup) }
    $pythonForDirect = Join-Path $prefixForDirect "python.exe"
    if (-not (Test-Path $pythonForDirect)) {
        throw "Python was not found in ROS environment: $pythonForDirect"
    }
    $directArgs = @(
        (Join-Path $ScriptDir "windows_rviz_sim.py"),
        "--repo-root", (Resolve-Path (Join-Path $RosWs "..")),
        "--conda-prefix", $prefixForDirect,
        "--speed-scale", "$SpeedScale"
    )
    if ($NoRviz) {
        $directArgs += "--no-rviz"
    }
    if ($Once) {
        $directArgs += "--once"
    }
    if ($ManualControl) {
        $directArgs += "--manual-control"
    }
    if ($IKControl) {
        $directArgs += "--ik-control"
    }
    & $pythonForDirect @directArgs
    exit $LASTEXITCODE
}

if (-not (Get-Command colcon -ErrorAction SilentlyContinue)) {
    throw "colcon was not found. Install python3-colcon-common-extensions / colcon-common-extensions in this ROS environment."
}

if (-not $NoBuild) {
    colcon build --symlink-install --merge-install --packages-select el_a3_description el_a3_sim
    if ($LASTEXITCODE -ne 0) {
        throw "colcon build failed"
    }
}

$installSetup = Join-Path $RosWs "install\local_setup.bat"
Invoke-SetupScript $installSetup

$args = @(
    "launch", "el_a3_sim", "sim.launch.py",
    "use_rviz:=$(if ($NoRviz) { 'false' } else { 'true' })",
    "auto_motion:=$(if ($NoAutoMotion) { 'false' } else { 'true' })",
    "loop:=$(if ($Once) { 'false' } else { 'true' })",
    "speed_scale:=$SpeedScale",
    "wrist_motor_type:=$WristMotorType"
)

ros2 @args
