[CmdletBinding()]
param(
    # Full APK install. Use after changing libraries, build.gradle, FtcRobotController, assets or
    # @Pinned classes, and for the first deploy on a new robot. Default is a Sloth reload.
    [switch]$Full,
    # Stay on the robot Wi-Fi afterwards; go back later with .\tools\deploy.ps1 -Return.
    [switch]$Stay,
    # Never touch Wi-Fi; use whatever adb can already reach.
    [switch]$NoWifiSwitch,
    # Only rejoin the home network saved by an earlier -Stay deploy.
    [switch]$Return
)

# Command-line twin of the Android Studio "Robot - ..." run configurations (.run/). Both run the same
# Gradle tasks from tools/robot-deploy.gradle, which switch to the robot Wi-Fi (tools/robot-wifi.ps1)
# after building and switch back afterwards, even when the deploy fails.
#   .\tools\deploy.ps1          Sloth hot reload  (:TeamCode:robotSloth)
#   .\tools\deploy.ps1 -Full    full APK install  (:TeamCode:robotInstall)
# Robot Wi-Fi profile: 10100-RC, or set $env:BIOBUZZ_ROBOT_WIFI.

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent

$task = if ($Return) { ':TeamCode:robotWifiReturn' } elseif ($Full) { ':TeamCode:robotInstall' } else { ':TeamCode:robotSloth' }
$gradleArgs = @($task)
if ($Stay) { $gradleArgs += '-Probot.stay' }
if ($NoWifiSwitch) { $gradleArgs += '-Probot.noWifi' }

Push-Location $repoRoot
try {
    # Gradle prints harmless JVM warnings on stderr; under 'Stop', Windows PowerShell would treat a
    # redirected one as fatal. Success is judged by the exit code below instead.
    $ErrorActionPreference = 'Continue'
    & .\gradlew.bat @gradleArgs
    if ($LASTEXITCODE -ne 0) {
        if (-not $Full -and -not $Return) {
            Write-Host 'If you changed a library or a file outside TeamCode, use .\tools\deploy.ps1 -Full.'
        }
        throw "$task failed (exit $LASTEXITCODE)."
    }
} finally {
    Pop-Location
}
