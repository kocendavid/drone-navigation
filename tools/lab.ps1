<#
Agent/operator service entry point. Status is read-only. Start reuses existing
services and logs new hidden processes. Stop refuses armed modes outside Hold.
No arming, takeoff, flight-mode changes, scene edits or timeline changes.
#>
param(
    [ValidateSet('status', 'start', 'stop')][string]$Action = 'status',
    [ValidateSet('px4', 'agent', 'zenoh', 'mount', 'navigator', 'all')][string]$Service = 'all'
)
$ErrorActionPreference = 'Stop'
$labRoot = Split-Path $PSScriptRoot -Parent
$labLogs = Join-Path $labRoot 'logs'
$labManifest = 'C:\IsaacSim-ros_workspaces\jazzy_ws\pixi.toml'
$labPx4 = '/home/finel/PX4-Autopilot/build/px4_sitl_default/bin/px4'

function Get-LabProcesses {
    @(Get-CimInstance Win32_Process | Where-Object {
        $_.Name -in @('python.exe', 'MicroXRCEAgent.exe', 'rmw_zenohd.exe', 'static_transform_publisher.exe', 'wsl.exe')
    })
}
function Find-Service([string]$Name, $Processes) {
    @($Processes | Where-Object {
        $candidate = $_
        $line = [string]$_.CommandLine
        switch ($Name) {
            'agent' { $candidate.Name -eq 'MicroXRCEAgent.exe' -and $line.Contains('agent-v2.4.3') }
            'zenoh' { $candidate.Name -eq 'rmw_zenohd.exe' -and $line.Contains('jazzy_ws') }
            'mount' { $candidate.Name -eq 'static_transform_publisher.exe' -and $line -match '--frame-id\s+chassis' -and $line -match '--child-frame-id\s+point_cloud' }
            'navigator' { $candidate.Name -eq 'python.exe' -and $line.Contains($labRoot) -and $line.Contains('ros2_navigate.py') }
        }
    })
}
function Get-Px4Processes {
    $lines = @(& wsl -d Ubuntu --exec pgrep -a -x px4)
    if ($LASTEXITCODE -notin @(0, 1)) { throw 'Unable to inspect WSL PX4 processes' }
    @($lines | Where-Object { $_ })
}
function Assert-CanStop {
    $running = @(Get-Px4Processes)
    if ($running.Count -eq 0) { return }
    if ($running.Count -ne 1 -or -not $running[0].Contains('/px4_sitl_default/bin/px4')) {
        throw 'Expected one Isaac PX4 instance; inspect the unexpected instance first'
    }
    $state = (& wsl -d Ubuntu --exec "${labPx4}-commander" --instance 0 status) -join "`n"
    if ($LASTEXITCODE -ne 0) { throw 'PX4 status unavailable; refusing to stop flight services' }
    if ($state -notmatch '(?m)^INFO\s+\[commander\]\s+Disarmed\s*$' -and $state -notmatch '(?m)^INFO\s+\[commander\]\s+navigation mode:\s+Hold\s*$') {
        throw 'Select native PX4 Hold or disarm before stopping flight services'
    }
}

$labProcesses = @(Get-LabProcesses)
$labServices = if ($Service -eq 'all') { @('agent', 'zenoh', 'mount', 'px4', 'navigator') } else { @($Service) }
if ($Action -eq 'status') {
    foreach ($name in $labServices) {
        if ($name -eq 'px4') {
            $matches = @(Get-Px4Processes)
            [pscustomobject]@{ Service = $name; Instances = $matches.Count; Processes = $matches -join ', ' }
        } else {
            $matches = @(Find-Service $name $labProcesses)
            [pscustomobject]@{ Service = $name; Instances = $matches.Count; Processes = ($matches.ProcessId -join ', ') }
        }
    }
    return
}
if ($Action -eq 'stop') {
    Assert-CanStop
    if ($Service -eq 'all') { throw 'Stop one named service at a time; there is no blanket stop-all' }
    if ($Service -eq 'px4') {
        $target = @(Get-Px4Processes)
        if ($target.Count -eq 0) { Write-Output 'px4 already stopped'; return }
        # Lockstep shutdown can wait indefinitely after Isaac stops supplying sensors.
        $shutdown = Start-Process -FilePath 'wsl.exe' -ArgumentList "-d Ubuntu --exec ${labPx4}-shutdown --instance 0" -WindowStyle Hidden -PassThru
        if (-not $shutdown.WaitForExit(8000)) {
            $remaining = @(Get-Px4Processes)
            if ($remaining.Count -eq 1 -and $remaining[0] -eq $target[0]) {
                $labProcessId = ($target[0] -split ' ', 2)[0]
                & wsl -d Ubuntu --exec kill -TERM $labProcessId
                Start-Sleep -Milliseconds 500
            }
            if (-not $shutdown.WaitForExit(1000)) { Stop-Process -Id $shutdown.Id }
        }
        if (@(Get-Px4Processes).Count -ne 0) { throw 'PX4 did not stop' }
        Write-Output 'Stopped px4'
    } else {
        $matches = @(Find-Service $Service $labProcesses)
        if ($matches.Count -gt 1) { throw "Multiple $Service processes; inspect duplicates before stopping" }
        foreach ($process in $matches) {
            if ($Service -eq 'navigator') {
                foreach ($child in $labProcesses | Where-Object { $_.ParentProcessId -eq $process.ProcessId -and $_.CommandLine.Contains($labRoot) -and $_.CommandLine.Contains('navigation_map_worker.py') }) {
                    Stop-Process -Id $child.ProcessId
                }
            }
            Stop-Process -Id $process.ProcessId
            Write-Output "Stopped $Service (PID $($process.ProcessId))"
        }
    }
    return
}
New-Item -ItemType Directory -Path $labLogs -Force | Out-Null
foreach ($name in $labServices) {
    $existing = @(if ($name -eq 'px4') { Get-Px4Processes } else { Find-Service $name @(Get-LabProcesses) })
    if ($existing.Count -gt 1) { throw "Multiple $name instances; inspect before starting another" }
    if ($existing.Count -eq 1) {
        if ($name -eq 'px4' -and -not $existing[0].Contains('/px4_sitl_default/bin/px4')) { throw 'SIH is running; Isaac SITL cannot start' }
        Write-Output "$name already running; reused"
        continue
    }
    $stamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
    $out = Join-Path $labLogs "lab_${name}_${stamp}.out.log"
    $err = Join-Path $labLogs "lab_${name}_${stamp}.err.log"
    switch ($name) {
        'agent' { $exe = 'pixi.exe'; $arguments = "run --manifest-path `"$labManifest`" C:\px4-tools\agent-v2.4.3\bin\MicroXRCEAgent.exe udp4 -p 8888" }
        'zenoh' { $exe = 'pixi.exe'; $arguments = "run --manifest-path `"$labManifest`" ros2 run rmw_zenoh_cpp rmw_zenohd" }
        'mount' { $exe = 'pixi.exe'; $arguments = "run --manifest-path `"$labManifest`" cmd /d /c `"$PSScriptRoot\lidar_mount_tf.cmd`"" }
        'px4' { $exe = 'wsl.exe'; $arguments = '-d Ubuntu -- bash /mnt/c/Users/finel/Documents/ChatGPT/issaac/px4/start-isaac-sitl.sh --daemon' }
        'navigator' { $exe = 'pixi.exe'; $arguments = "run --manifest-path `"$labManifest`" cmd /d /c `"$PSScriptRoot\ros2_navigate.cmd`"" }
    }
    $started = Start-Process -FilePath $exe -ArgumentList $arguments -WorkingDirectory $labRoot -WindowStyle Hidden -RedirectStandardOutput $out -RedirectStandardError $err -PassThru
    Write-Output "Started $name (launcher PID $($started.Id)). Logs: $out and $err"
    # Wait briefly for daemon discovery so a second invocation does not race startup.
    Start-Sleep -Seconds 2
    $started.Refresh()
    if ($started.HasExited) { throw "$name launcher exited ($($started.ExitCode)); inspect $err" }
}
