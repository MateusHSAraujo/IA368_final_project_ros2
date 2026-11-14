<#
.SYNOPSIS
Forwards the ROS-TCP-Endpoint and scene streamer ports to a USB-connected Meta Quest 3.

.DESCRIPTION
Runs on the Windows host. Uses "adb reverse" so that the headset can reach
services running on the PC through its own localhost:
  tcp:10000  ROS-TCP-Endpoint (ROS 2 in WSL2)
  tcp:23005  CoppeliaSim scene streamer (REP)
  tcp:23006  CoppeliaSim scene streamer (PUB)
Requires adb (Android platform-tools or Meta Quest Developer Hub) on PATH and
exactly one device in state "device" (or pass -Serial).

.PARAMETER Serial
Device serial to use when more than one device is connected.

.PARAMETER List
Only list the current reverse rules.

.PARAMETER Remove
Remove all reverse rules (adb reverse --remove-all).

.PARAMETER PortProxy
Requires an elevated shell. Creates a netsh portproxy 127.0.0.1:10000 -> <WSL IP>:10000
and an inbound firewall rule for TCP 10000. Only needed if WSL2 localhost
forwarding does not make localhost:10000 reach the ROS-TCP-Endpoint in WSL2.

.PARAMETER RemovePortProxy
Requires an elevated shell. Removes the portproxy rule and the firewall rule.

.EXAMPLE
.\setup_adb_reverse.ps1

.EXAMPLE
.\setup_adb_reverse.ps1 -Serial 1WMHH123456789

.EXAMPLE
.\setup_adb_reverse.ps1 -List

.EXAMPLE
.\setup_adb_reverse.ps1 -PortProxy
#>
[CmdletBinding()]
param(
    [string]$Serial,
    [switch]$List,
    [switch]$Remove,
    [switch]$PortProxy,
    [switch]$RemovePortProxy
)

$ErrorActionPreference = 'Stop'

$RosPort = 10000
$ReversePorts = @(10000, 23005, 23006)
$FirewallRuleName = 'IA368 ROS-TCP-Endpoint 10000'

function Test-Admin {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Require-Admin {
    if (-not (Test-Admin)) {
        Write-Error 'This action requires an elevated PowerShell (Run as administrator).'
        exit 1
    }
}

function Remove-PortProxyRules {
    Require-Admin
    & netsh interface portproxy delete v4tov4 listenport=$RosPort listenaddress=127.0.0.1 2>$null | Out-Null
    $global:LASTEXITCODE = 0
    if (Get-NetFirewallRule -DisplayName $FirewallRuleName -ErrorAction SilentlyContinue) {
        Remove-NetFirewallRule -DisplayName $FirewallRuleName
    }
    Write-Host 'Portproxy and firewall rule removed.'
}

function Add-PortProxyRules {
    Require-Admin
    $wslOutput = (& wsl.exe hostname -I) -join ' '
    $wslIp = ($wslOutput.Trim() -split '\s+')[0]
    if (-not $wslIp -or $wslIp -notmatch '^\d{1,3}(\.\d{1,3}){3}$') {
        Write-Error "Could not determine the WSL IP (wsl.exe hostname -I returned '$wslOutput')."
        exit 1
    }
    & netsh interface portproxy delete v4tov4 listenport=$RosPort listenaddress=127.0.0.1 2>$null | Out-Null
    $global:LASTEXITCODE = 0
    & netsh interface portproxy add v4tov4 listenport=$RosPort listenaddress=127.0.0.1 connectport=$RosPort connectaddress=$wslIp
    if ($LASTEXITCODE -ne 0) {
        Write-Error 'netsh portproxy add failed.'
        exit 1
    }
    if (-not (Get-NetFirewallRule -DisplayName $FirewallRuleName -ErrorAction SilentlyContinue)) {
        New-NetFirewallRule -DisplayName $FirewallRuleName -Direction Inbound -Protocol TCP `
            -LocalPort $RosPort -Action Allow | Out-Null
    }
    Write-Host "Portproxy 127.0.0.1:$RosPort -> ${wslIp}:$RosPort configured."
}

if ($RemovePortProxy) {
    Remove-PortProxyRules
    exit 0
}
if ($PortProxy) {
    Add-PortProxyRules
    exit 0
}

if (-not (Get-Command adb -ErrorAction SilentlyContinue)) {
    Write-Error ('adb was not found on PATH. Install the Android platform-tools ' +
        '(https://developer.android.com/tools/releases/platform-tools) or Meta Quest Developer Hub, ' +
        'and add adb to PATH.')
    exit 1
}

if ($List) {
    & adb reverse --list
    exit $LASTEXITCODE
}

# Device selection: exactly one device in state "device", or the one given by -Serial.
$devices = @()
foreach ($line in (& adb devices)) {
    if ($line -match '^(\S+)\s+(\S+)$' -and $Matches[1] -ne 'List') {
        $devices += [pscustomobject]@{ Serial = $Matches[1]; State = $Matches[2] }
    }
}
if ($Serial) {
    $selected = @($devices | Where-Object { $_.Serial -eq $Serial })
    if ($selected.Count -eq 0) {
        Write-Error "Device '$Serial' not found in 'adb devices'."
        exit 1
    }
    if ($selected[0].State -ne 'device') {
        Write-Error "Device '$Serial' is in state '$($selected[0].State)' (expected 'device'). Check the USB debugging prompt in the headset."
        exit 1
    }
} else {
    $ready = @($devices | Where-Object { $_.State -eq 'device' })
    if ($ready.Count -eq 0) {
        $states = ($devices | ForEach-Object { "$($_.Serial)=$($_.State)" }) -join ', '
        Write-Error "No adb device in state 'device' (found: $states). Connect the Quest by USB and accept the debugging prompt."
        exit 1
    }
    if ($ready.Count -gt 1) {
        Write-Error 'More than one adb device is connected. Pick one with -Serial <id>.'
        exit 1
    }
    $Serial = $ready[0].Serial
}

if ($Remove) {
    & adb -s $Serial reverse --remove-all
    exit $LASTEXITCODE
}

foreach ($port in $ReversePorts) {
    & adb -s $Serial reverse "tcp:$port" "tcp:$port"
    if ($LASTEXITCODE -ne 0) {
        Write-Error "adb reverse tcp:$port failed."
        exit 1
    }
}
Write-Host "Reverse rules for device ${Serial}:"
& adb -s $Serial reverse --list

if (Test-NetConnection -ComputerName localhost -Port $RosPort -InformationLevel Quiet -WarningAction SilentlyContinue) {
    Write-Host "ROS-TCP-Endpoint is reachable on localhost:$RosPort."
} else {
    Write-Warning "localhost:$RosPort is not reachable. ROS-TCP-Endpoint may not be running yet (or try -PortProxy)."
}
exit 0
