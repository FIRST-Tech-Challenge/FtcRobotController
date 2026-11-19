[CmdletBinding()]
param(
    # Make the robot reachable over adb: USB if attached, else join the robot Wi-Fi and adb connect.
    [Parameter(ParameterSetName = 'Join', Mandatory)] [switch]$Join,
    # Undo -Join: adb disconnect and rejoin the network the laptop was on before.
    [Parameter(ParameterSetName = 'Leave', Mandatory)] [switch]$Leave,
    # With -Leave: stay on the robot Wi-Fi (the saved home network is kept for a later -Leave).
    [switch]$Stay,
    # With -Join: never touch Wi-Fi; only check adb can already reach the robot.
    [switch]$NoWifiSwitch,
    # Saved Windows Wi-Fi profile of the Control Hub. Override with $env:BIOBUZZ_ROBOT_WIFI.
    [string]$RobotWifi = $(if ($env:BIOBUZZ_ROBOT_WIFI) { $env:BIOBUZZ_ROBOT_WIFI } else { '10100-RC' })
)

# Wi-Fi switching for robot deploys. Called by the robot* Gradle tasks (tools/robot-deploy.gradle), so
# it works from Android Studio run configurations, tools/deploy.ps1 and the command line alike.
# The robot Wi-Fi must be a saved Windows profile: connect to it once by hand so Windows keeps the password.
# State between -Join and -Leave lives in build/robot-wifi.json (gitignored).

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$RobotAdbAddress = '192.168.43.1:5555'
$WifiTimeoutSeconds = 30
$stateFile = Join-Path $repoRoot 'build\robot-wifi.json'

function Find-Adb {
    $onPath = Get-Command adb -ErrorAction SilentlyContinue
    if ($onPath) { return $onPath.Source }
    $sdkDirs = @()
    $localProps = Join-Path $repoRoot 'local.properties'
    if (Test-Path $localProps) {
        $line = Select-String -LiteralPath $localProps -Pattern '^sdk\.dir=(.+)$' | Select-Object -First 1
        if ($line) { $sdkDirs += ($line.Matches[0].Groups[1].Value -replace '\\:', ':' -replace '\\\\', '\') }
    }
    if ($env:ANDROID_HOME) { $sdkDirs += $env:ANDROID_HOME }
    $sdkDirs += Join-Path $env:LOCALAPPDATA 'Android\Sdk'
    foreach ($sdk in $sdkDirs) {
        $candidate = Join-Path $sdk 'platform-tools\adb.exe'
        if (Test-Path $candidate) { return $candidate }
    }
    throw 'adb not found. Install Android SDK Platform-Tools (Android Studio > SDK Manager).'
}

function Get-AdbDevices {
    @(& $script:adb devices | Select-Object -Skip 1 | Where-Object { $_ -match "\tdevice$" } |
        ForEach-Object { ($_ -split "\t")[0] })
}

function Get-Wifi {
    # First connected wireless interface: @{ Interface; Ssid } or $null.
    $name = $null
    foreach ($line in (netsh wlan show interfaces)) {
        if ($line -match '^\s*Name\s*:\s*(.+?)\s*$') { $name = $Matches[1] }
        elseif ($line -match '^\s*SSID\s*:\s*(.+?)\s*$' -and $name) {
            return [pscustomobject]@{ Interface = $name; Ssid = $Matches[1] }
        }
    }
    return $null
}

function Test-SavedWifiProfile([string]$ssid) {
    foreach ($line in (netsh wlan show profiles)) {
        if ($line -match '^\s*[^:]*Profile\s*:\s*(.+?)\s*$' -and $Matches[1] -eq $ssid) { return $true }
    }
    return $false
}

function Request-WifiScan {
    # netsh only reads the list of nearby networks Windows last scanned, and has no scan command, so a
    # robot that was switched on after that scan looks absent. Ask the WLAN service for a fresh scan
    # (the same thing the taskbar Wi-Fi menu does). Results arrive over the next few seconds.
    if (-not ('BioBuzz.WifiScanner' -as [type])) {
        Add-Type -Namespace BioBuzz -Name WifiScanner -MemberDefinition @'
[DllImport("wlanapi.dll")] static extern int WlanOpenHandle(uint clientVersion, IntPtr reserved, out uint negotiatedVersion, out IntPtr handle);
[DllImport("wlanapi.dll")] static extern int WlanCloseHandle(IntPtr handle, IntPtr reserved);
[DllImport("wlanapi.dll")] static extern int WlanEnumInterfaces(IntPtr handle, IntPtr reserved, out IntPtr interfaceList);
[DllImport("wlanapi.dll")] static extern int WlanScan(IntPtr handle, ref Guid interfaceGuid, IntPtr ssid, IntPtr ieData, IntPtr reserved);
[DllImport("wlanapi.dll")] static extern void WlanFreeMemory(IntPtr memory);
// WLAN_INTERFACE_INFO_LIST: two DWORDs, then WLAN_INTERFACE_INFO { GUID; WCHAR[256]; DWORD } = 532 bytes each.
public static int ScanAll() {
    uint negotiated; IntPtr handle;
    int result = WlanOpenHandle(2, IntPtr.Zero, out negotiated, out handle);
    if (result != 0) return result;
    try {
        IntPtr list;
        result = WlanEnumInterfaces(handle, IntPtr.Zero, out list);
        if (result != 0) return result;
        try {
            int count = Marshal.ReadInt32(list, 0);
            for (int i = 0; i < count; i++) {
                Guid id = (Guid)Marshal.PtrToStructure(new IntPtr(list.ToInt64() + 8 + i * 532), typeof(Guid));
                int scan = WlanScan(handle, ref id, IntPtr.Zero, IntPtr.Zero, IntPtr.Zero);
                if (scan != 0) result = scan;
            }
        } finally { WlanFreeMemory(list); }
    } finally { WlanCloseHandle(handle, IntPtr.Zero); }
    return result;
}
'@
    }
    $result = [BioBuzz.WifiScanner]::ScanAll()
    if ($result -ne 0) { Write-Warning "Windows refused a Wi-Fi scan (error $result); using its last list." }
}

function Get-VisibleSsids {
    @(netsh wlan show networks | ForEach-Object { if ($_ -match '^SSID \d+ : (.*)$') { $Matches[1].Trim() } })
}

function Wait-WifiVisible([string]$ssid) {
    if ((Get-VisibleSsids) -contains $ssid) { return }
    Write-Host "[>] Scanning for '$ssid'"
    $deadline = (Get-Date).AddSeconds($WifiTimeoutSeconds)
    $nextScan = Get-Date
    while ((Get-Date) -lt $deadline) {
        if ((Get-Date) -ge $nextScan) { Request-WifiScan; $nextScan = (Get-Date).AddSeconds(4) }
        Start-Sleep -Milliseconds 500
        if ((Get-VisibleSsids) -contains $ssid) { Write-Host "[OK] Found '$ssid'"; return }
    }
    $seen = (Get-VisibleSsids) -join ', '
    throw "Windows cannot see Wi-Fi '$ssid' after $WifiTimeoutSeconds s of scanning (it sees: $seen). Is the robot on and in range?"
}

function Join-Wifi([string]$ssid, [string]$interface) {
    Wait-WifiVisible $ssid
    Write-Host "[>] Joining Wi-Fi '$ssid'"
    $deadline = (Get-Date).AddSeconds($WifiTimeoutSeconds)
    $nextConnect = Get-Date
    while ((Get-Date) -lt $deadline) {
        if ((Get-Date) -ge $nextConnect) {
            # Retried: right after a scan Windows sometimes reports the network unavailable for a moment.
            $out = netsh wlan connect name="$ssid" interface="$interface"
            if ($LASTEXITCODE -ne 0) { Write-Host "    netsh: $(($out | Out-String).Trim())" }
            $nextConnect = (Get-Date).AddSeconds(8)
        }
        Start-Sleep -Milliseconds 500
        $now = Get-Wifi
        if ($now -and $now.Ssid -eq $ssid) { Write-Host "[OK] On '$ssid'"; return }
    }
    throw "Timed out after $WifiTimeoutSeconds s joining '$ssid'. Is its saved password still right?"
}

function Connect-RobotAdb {
    # The hub's DHCP address and adbd can lag the Wi-Fi association by a few seconds.
    $deadline = (Get-Date).AddSeconds($WifiTimeoutSeconds)
    while ((Get-Date) -lt $deadline) {
        & $script:adb connect $RobotAdbAddress *> $null
        if ((Get-AdbDevices) -contains $RobotAdbAddress) { Write-Host "[OK] adb on $RobotAdbAddress"; return }
        Start-Sleep -Seconds 1
    }
    throw "On the robot Wi-Fi but adb could not reach $RobotAdbAddress. Is the Robot Controller app running?"
}

function Read-State {
    if (Test-Path $stateFile) { return Get-Content -LiteralPath $stateFile -Raw | ConvertFrom-Json }
    return $null
}

function Save-State($state) {
    New-Item -ItemType Directory -Force (Split-Path $stateFile -Parent) | Out-Null
    $state | ConvertTo-Json | Set-Content -LiteralPath $stateFile -Encoding utf8
}

$adb = Find-Adb
# Start the adb server detached. Started by a plain `adb devices` it inherits this script's output pipe,
# and Gradle's Exec then waits on that pipe for as long as the server lives.
$adbServer = Start-Process -FilePath $adb -ArgumentList 'start-server' -WindowStyle Hidden -PassThru
$adbServer.WaitForExit()

if ($Join) {
    $devices = Get-AdbDevices
    $usb = @($devices | Where-Object { $_ -notmatch ':' })
    if ($usb.Count -gt 0) { Write-Host "[OK] Hub on USB ($($usb[0])); Wi-Fi left alone"; exit 0 }
    if ($devices -contains $RobotAdbAddress) { Write-Host "[OK] Already connected to $RobotAdbAddress"; exit 0 }
    if ($NoWifiSwitch) { throw 'No robot on adb and Wi-Fi switching is off. Plug in USB-C or join the robot Wi-Fi.' }

    $current = Get-Wifi
    if (-not $current) { throw 'No connected Wi-Fi adapter found. Turn Wi-Fi on, or plug the hub in over USB-C.' }
    if ($current.Ssid -ne $RobotWifi) {
        if (-not (Test-SavedWifiProfile $RobotWifi)) {
            throw "Windows has no saved Wi-Fi profile '$RobotWifi'. Connect to it once by hand, or set BIOBUZZ_ROBOT_WIFI."
        }
        Write-Host "[*] Currently on '$($current.Ssid)'"
        # Saved before switching, so -Leave can still go home if anything below fails.
        Save-State ([pscustomobject]@{ Home = $current.Ssid; Interface = $current.Interface })
        Join-Wifi $RobotWifi $current.Interface
    }
    Connect-RobotAdb
    exit 0
}

# -Leave
$state = Read-State
if (-not $state) { Write-Host '[*] Wi-Fi was not switched by a deploy; nothing to undo'; exit 0 }
if ($Stay) { Write-Host "[*] Staying on '$RobotWifi'. Run 'Robot: Return Wi-Fi' to go back to '$($state.Home)'."; exit 0 }
& $adb disconnect $RobotAdbAddress *> $null
$now = Get-Wifi
if (-not $now -or $now.Ssid -ne $state.Home) {
    try { Join-Wifi $state.Home $state.Interface }
    catch { Write-Warning "Could not rejoin '$($state.Home)': $($_.Exception.Message) Switch back manually."; exit 0 }
}
Remove-Item -LiteralPath $stateFile -Force
exit 0
