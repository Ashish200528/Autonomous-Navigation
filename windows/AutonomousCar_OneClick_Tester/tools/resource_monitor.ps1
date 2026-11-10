param(
    [switch]$LibraryOnly,
    [string]$MonitorRunDirectory,
    [string]$Distro = "Ubuntu-24.04",
    [ValidateRange(1,60)][int]$IntervalSeconds = 5,
    [string]$StopFile
)

$ErrorActionPreference = "Stop"

if (-not ("StandaloneHostProbe" -as [type])) {
    $nativeSource = @"
using System;
using System.Runtime.InteropServices;
public static class StandaloneHostProbe {
  [StructLayout(LayoutKind.Sequential)]
  public struct MEMORYSTATUSEX {
    public uint dwLength;
    public uint dwMemoryLoad;
    public ulong ullTotalPhys;
    public ulong ullAvailPhys;
    public ulong ullTotalPageFile;
    public ulong ullAvailPageFile;
    public ulong ullTotalVirtual;
    public ulong ullAvailVirtual;
    public ulong ullAvailExtendedVirtual;
    public MEMORYSTATUSEX(bool init) {
      dwLength = (uint)Marshal.SizeOf(typeof(MEMORYSTATUSEX));
      dwMemoryLoad = 0;
      ullTotalPhys = ullAvailPhys = ullTotalPageFile = ullAvailPageFile = 0;
      ullTotalVirtual = ullAvailVirtual = ullAvailExtendedVirtual = 0;
    }
  }
  [StructLayout(LayoutKind.Sequential)]
  public struct FILETIME { public uint low; public uint high; }
  [DllImport("kernel32.dll", SetLastError=true)]
  public static extern bool GlobalMemoryStatusEx(ref MEMORYSTATUSEX value);
  [DllImport("kernel32.dll", SetLastError=true)]
  public static extern bool GetSystemTimes(out FILETIME idle, out FILETIME kernel, out FILETIME user);
  public static ulong ToUInt64(FILETIME value) {
    return ((ulong)value.high << 32) | value.low;
  }
}
"@
    Add-Type -TypeDefinition $nativeSource -ErrorAction Stop
}

$script:PreviousCpuTicks = $null
function Get-StandaloneSnapshot {
    param(
        [string]$Distribution = "Ubuntu-24.04",
        [string]$RunDirectory
    )
    $mem = New-Object -TypeName "StandaloneHostProbe+MEMORYSTATUSEX" -ArgumentList $true
    if (-not [StandaloneHostProbe]::GlobalMemoryStatusEx([ref]$mem)) {
        throw "GlobalMemoryStatusEx failed."
    }
    $idle = New-Object -TypeName "StandaloneHostProbe+FILETIME"
    $kernel = New-Object -TypeName "StandaloneHostProbe+FILETIME"
    $user = New-Object -TypeName "StandaloneHostProbe+FILETIME"
    if (-not [StandaloneHostProbe]::GetSystemTimes([ref]$idle,[ref]$kernel,[ref]$user)) {
        throw "GetSystemTimes failed."
    }
    $idleTicks = [StandaloneHostProbe]::ToUInt64($idle)
    $kernelTicks = [StandaloneHostProbe]::ToUInt64($kernel)
    $userTicks = [StandaloneHostProbe]::ToUInt64($user)
    $cpu = $null
    if ($null -ne $script:PreviousCpuTicks) {
        $idleDelta = [double]($idleTicks - $script:PreviousCpuTicks.idle)
        $totalDelta = [double](($kernelTicks + $userTicks) - $script:PreviousCpuTicks.total)
        if ($totalDelta -gt 0) {
            $cpu = [math]::Round([math]::Max(0,[math]::Min(100,100*(1-($idleDelta/$totalDelta)))),1)
        }
    }
    $script:PreviousCpuTicks = @{ idle=$idleTicks; total=($kernelTicks+$userTicks) }

    $wslMem = @{}
    $wslLines = @()
    try {
        $wslLines = @(& wsl.exe --distribution $Distribution --exec cat /proc/meminfo 2>$null)
        foreach ($line in $wslLines) {
            if ($line -match "^(MemTotal|MemAvailable|SwapTotal|SwapFree):\s+(\d+)") {
                $wslMem[$matches[1]] = [long]$matches[2]
            }
        }
    } catch { }

    $processRows = @()
    try {
        $procLines = @(& wsl.exe --distribution $Distribution --exec ps -eo pid,comm,rss,pcpu,args --no-headers 2>$null)
        foreach ($line in $procLines) {
            if ($line -match "^\s*(\d+)\s+(\S+)\s+(\d+)\s+([\d.]+)\s+(.*)$") {
                $pidValue = [int]$matches[1]
                $comm = [string]$matches[2]
                $rssKiB = [long]$matches[3]
                $cpuValue = [double]$matches[4]
                $command = [string]$matches[5]
                if (($comm -match "^(gz|gzserver|gzclient|ros|python3|component_container|robot_state_publisher|ekf_node)$") -or
                    ($command -match "(anc_lidar|anc_navigation|slam_toolbox|ros_gz_bridge|nav2_|gz sim|robot_state_publisher|robot_localization)")) {
                    $processRows += [pscustomobject]@{
                        timestampUtc = [DateTime]::UtcNow.ToString("o")
                        pid = $pidValue
                        command = $comm
                        rssMiB = [math]::Round($rssKiB/1024.0,2)
                        cpuPercent = $cpuValue
                        commandLine = $command
                    }
                }
            }
        }
    } catch { }

    $vmmemMiB = $null
    try {
        $vmmem = Get-Process -Name "vmmemWSL" -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($null -ne $vmmem) { $vmmemMiB = [math]::Round($vmmem.WorkingSet64/1MB,2) }
    } catch { }

    $rootPath = $null
    $diskFreeMiB = $null
    try {
        if ($RunDirectory) { $rootPath = [IO.Path]::GetPathRoot($RunDirectory) }
        if ($rootPath) {
            $drive = New-Object -TypeName System.IO.DriveInfo -ArgumentList $rootPath
            $diskFreeMiB = [math]::Floor($drive.AvailableFreeSpace/1MB)
        }
    } catch { }

    $toMiB = { param($key) if ($wslMem.ContainsKey($key)) { [math]::Floor($wslMem[$key]/1024) } else { $null } }
    return [pscustomobject]@{
        timestampUtc = [DateTime]::UtcNow.ToString("o")
        windowsTotalMiB = [math]::Floor($mem.ullTotalPhys/1MB)
        windowsAvailableMiB = [math]::Floor($mem.ullAvailPhys/1MB)
        windowsMemoryLoadPercent = [int]$mem.dwMemoryLoad
        windowsSystemCpuPercent = $cpu
        vmmemWslWorkingSetMiB = $vmmemMiB
        windowsRunVolumeAvailableMiB = $diskFreeMiB
        wslTotalMiB = & $toMiB "MemTotal"
        wslAvailableMiB = & $toMiB "MemAvailable"
        wslSwapTotalMiB = & $toMiB "SwapTotal"
        wslSwapUsedMiB = if ($wslMem.ContainsKey("SwapTotal") -and $wslMem.ContainsKey("SwapFree")) {
            [math]::Floor(($wslMem["SwapTotal"]-$wslMem["SwapFree"])/1024)
        } else { $null }
        wslProcesses = $processRows
    }
}

function Write-StandaloneResourceSample {
    param(
        [Parameter(Mandatory=$true)][string]$Directory,
        [string]$Distribution = "Ubuntu-24.04"
    )
    $snapshot = Get-StandaloneSnapshot -Distribution $Distribution -RunDirectory $Directory
    $resourceDir = Join-Path $Directory "resource_monitoring"
    New-Item -ItemType Directory -Path $resourceDir -Force | Out-Null
    $windowsCsv = Join-Path $resourceDir "windows_memory.csv"
    $wslCsv = Join-Path $resourceDir "wsl_memory.csv"
    $processCsv = Join-Path $resourceDir "process_usage.csv"
    $snapshot | Select-Object timestampUtc,windowsTotalMiB,windowsAvailableMiB,windowsMemoryLoadPercent,windowsSystemCpuPercent,vmmemWslWorkingSetMiB,windowsRunVolumeAvailableMiB |
        Export-Csv -LiteralPath $windowsCsv -Append -NoTypeInformation -Encoding UTF8
    $snapshot | Select-Object timestampUtc,wslTotalMiB,wslAvailableMiB,wslSwapTotalMiB,wslSwapUsedMiB |
        Export-Csv -LiteralPath $wslCsv -Append -NoTypeInformation -Encoding UTF8
    foreach ($process in $snapshot.wslProcesses) {
        $process | Export-Csv -LiteralPath $processCsv -Append -NoTypeInformation -Encoding UTF8
    }
    if (-not (Test-Path -LiteralPath $processCsv)) {
        "timestampUtc,pid,command,rssMiB,cpuPercent,commandLine" |
            Set-Content -LiteralPath $processCsv -Encoding UTF8
    }
    $jsonPath = Join-Path $Directory "environment\current_host_snapshot.json"
    $temporary = "$jsonPath.tmp"
    $snapshot | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $temporary -Encoding UTF8
    Move-Item -LiteralPath $temporary -Destination $jsonPath -Force
    return $snapshot
}

if (-not $LibraryOnly) {
    if (-not $MonitorRunDirectory) { throw "MonitorRunDirectory is required unless -LibraryOnly is used." }
    if (-not $StopFile) { $StopFile = Join-Path $MonitorRunDirectory "resource_monitoring\STOP_MONITOR" }
    while (-not (Test-Path -LiteralPath $StopFile)) {
        try { [void](Write-StandaloneResourceSample -Directory $MonitorRunDirectory -Distribution $Distro) }
        catch {
            $resourceDir = Join-Path $MonitorRunDirectory "resource_monitoring"
            New-Item -ItemType Directory -Path $resourceDir -Force | Out-Null
            $_ | Out-String | Add-Content -LiteralPath (Join-Path $resourceDir "monitor-errors.log")
        }
        Start-Sleep -Seconds $IntervalSeconds
    }
}
