param([switch]$PreflightOnly)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$PackageRoot = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$WslExe = Join-Path $env:WINDIR 'System32\wsl.exe'
$Distro = 'Ubuntu-24.04'
$ProjectRoot = '/home/ashish/AutonomousNavigationCar'
$SessionScript = "$ProjectRoot/scripts/windows_demo_session.sh"
$LinuxHelperWindows = Join-Path $PackageRoot 'scripts\run-wsl-session.sh'
$LogRoot = Join-Path $PackageRoot 'logs'
$ResultRoot = Join-Path $PackageRoot 'results'
$VideoRoot = Join-Path $PackageRoot 'videos'
$ScreenshotRoot = Join-Path $PackageRoot 'screenshots'
$ToolRoot = Join-Path $PackageRoot 'tools'
$script:RunTag = (Get-Date -Format 'yyyyMMdd_HHmmss') + '_' + $PID
$script:RunRoot = Join-Path $ResultRoot ('run_' + $script:RunTag)
$TranscriptPath = Join-Path $LogRoot ('launcher_' + $script:RunTag + '.log')
$script:TranscriptActive = $false
$script:Recorder = $null
$script:RecorderLogPath = $null
$script:Encoder = 'libx264'
$script:EncoderLabel = 'software H.264'

foreach ($folder in @($LogRoot,$ResultRoot,$VideoRoot,$ScreenshotRoot,$ToolRoot,(Join-Path $PackageRoot 'scripts'))) {
    New-Item -ItemType Directory -Force -Path $folder | Out-Null
}
New-Item -ItemType Directory -Force -Path $script:RunRoot | Out-Null
try { Start-Transcript -Path $TranscriptPath -Append | Out-Null; $script:TranscriptActive = $true } catch { }

function Write-Section([string]$Text) {
    Write-Host ''
    Write-Host ('=' * 68)
    Write-Host $Text
    Write-Host ('=' * 68)
}

function Get-HostMemoryMiB {
    if (-not ('ANCVisualDemoMemory' -as [type])) {
        Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class ANCVisualDemoMemory {
    [StructLayout(LayoutKind.Sequential, CharSet=CharSet.Auto)]
    public class MEMORYSTATUSEX {
        public uint dwLength = (uint)Marshal.SizeOf(typeof(MEMORYSTATUSEX));
        public uint dwMemoryLoad;
        public ulong ullTotalPhys, ullAvailPhys, ullTotalPageFile, ullAvailPageFile;
        public ulong ullTotalVirtual, ullAvailVirtual, ullAvailExtendedVirtual;
    }
    [DllImport("kernel32.dll", CharSet=CharSet.Auto, SetLastError=true)]
    public static extern bool GlobalMemoryStatusEx([In, Out] MEMORYSTATUSEX lpBuffer);
}
'@
    }
    $m = New-Object 'ANCVisualDemoMemory+MEMORYSTATUSEX'
    if (-not [ANCVisualDemoMemory]::GlobalMemoryStatusEx($m)) { throw 'GlobalMemoryStatusEx failed.' }
    return [math]::Floor($m.ullAvailPhys / 1MB)
}

function Get-WslResourceSnapshot {
    $old = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $mem = & $WslExe -d $Distro -- cat /proc/meminfo 2>&1
        $memCode = $LASTEXITCODE
        $disk = & $WslExe -d $Distro -- df -Pk '/home/ashish/AutonomousNavigationCar' 2>&1
        $diskCode = $LASTEXITCODE
    } finally { $ErrorActionPreference = $old }
    if ($memCode -ne 0 -or $diskCode -ne 0) { throw "WSL resource probe failed: $($mem -join ' ') $($disk -join ' ')" }
    $memLine = @($mem | Where-Object { [string]$_ -match '^MemAvailable:\s+\d+' } | Select-Object -Last 1)
    $diskLine = @($disk | Where-Object { [string]$_ -match '^/\S+\s+\d+\s+\d+\s+\d+\s+\d+%\s+\S+' } | Select-Object -Last 1)
    if (-not $memLine -or -not $diskLine) { throw "Could not parse WSL resource output: $($mem -join ' ') $($disk -join ' ')" }
    $memKiB = [int64]([regex]::Match([string]$memLine[0],'^MemAvailable:\s+(\d+)').Groups[1].Value)
    $diskKiB = [int64]([regex]::Match([string]$diskLine[0],'\s(\d+)\s+\d+%\s+\S+$').Groups[1].Value)
    return [pscustomobject]@{ WslAvailableMiB=[math]::Floor($memKiB/1024); ProjectDiskFreeMiB=[math]::Floor($diskKiB/1024) }
}

function Get-ResourceSnapshot {
    $hostMiB = Get-HostMemoryMiB
    $wsl = Get-WslResourceSnapshot
    $driveRoot = [System.IO.Path]::GetPathRoot($PackageRoot)
    $hostDiskMiB = [math]::Floor(([System.IO.DriveInfo]::new($driveRoot)).AvailableFreeSpace / 1MB)
    return [pscustomobject]@{
        WindowsAvailableMiB = $hostMiB
        WslAvailableMiB = $wsl.WslAvailableMiB
        ProjectDiskFreeMiB = $wsl.ProjectDiskFreeMiB
        PackageDiskFreeMiB = $hostDiskMiB
        CapturedAt = (Get-Date).ToString('o')
    }
}

function Invoke-WslText([string]$Command, [string]$Label) {
    Write-Host "WSL: $Label"
    $old = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $lines = & $WslExe -d $Distro -- bash -lc $Command 2>&1
        $code = $LASTEXITCODE
    } finally { $ErrorActionPreference = $old }
    foreach ($line in $lines) { Write-Host ([string]$line) }
    return [pscustomobject]@{ ExitCode=$code; Text=($lines -join [Environment]::NewLine) }
}

function Get-WslPath([string]$WindowsPath) {
    $normalized = $WindowsPath.Replace([string][char]92,'/')
    $value = & $WslExe -d $Distro -- wslpath -u -a $normalized 2>&1
    if ($LASTEXITCODE -ne 0) { throw "wslpath failed for $WindowsPath : $($value -join ' ')" }
    return ([string]($value | Select-Object -Last 1)).Trim()
}

function Save-Json([string]$Path, $Value) {
    $Value | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $Path -Encoding UTF8
}

function New-DemoResult([string]$Id, [string]$Mode, [string]$Scenario, [string]$State, [string]$Reason) {
    return [ordered]@{
        demo_id=$Id; mode=$Mode; scenario=$Scenario; status=$State; reason=$Reason
        gazebo_opened=$false; rviz_opened='not_required'; robot_spawned=$false
        sensors_published='not_assessed'; robot_moved=$false; navigation_reached_goal='not_run'
        safety_console_visible='not_required'
        video_saved=$false; clean_shutdown=$false; video_path=$null; evidence_folder=$null
        started_at=(Get-Date).ToString('o'); finished_at=$null
    }
}

function Write-BlockedResult([string]$Id, [string]$Mode, [string]$Scenario, [string]$Reason) {
    $r = New-DemoResult $Id $Mode $Scenario 'BLOCKED' $Reason
    $r.finished_at = (Get-Date).ToString('o')
    Save-Json (Join-Path $script:RunRoot ($Id + '.json')) $r
    Write-Host "BLOCKED $Id : $Reason" -ForegroundColor Yellow
    return $r
}

function Get-DesktopWindowApi {
    if (-not ('ANCVisualDemoWindows' -as [type])) {
        Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
public static class ANCVisualDemoWindows {
    public delegate bool EnumProc(IntPtr hWnd, IntPtr lParam);
    [DllImport("user32.dll")] static extern bool EnumWindows(EnumProc cb, IntPtr lp);
    [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr hWnd);
    [DllImport("user32.dll", CharSet=CharSet.Auto)] static extern int GetWindowText(IntPtr hWnd, StringBuilder text, int count);
    [DllImport("user32.dll")] static extern bool ShowWindow(IntPtr hWnd, int cmd);
    [DllImport("user32.dll")] static extern bool SetWindowPos(IntPtr hWnd, IntPtr after, int x, int y, int cx, int cy, uint flags);
    [DllImport("user32.dll")] static extern bool SetForegroundWindow(IntPtr hWnd);
    public static string[] Titles() {
        var result = new List<string>();
        EnumWindows((h, p) => { if (IsWindowVisible(h)) { var b = new StringBuilder(512); GetWindowText(h,b,b.Capacity); if (b.Length>0) result.Add(b.ToString()); } return true; }, IntPtr.Zero);
        return result.ToArray();
    }
    static IntPtr Find(string part) {
        IntPtr found=IntPtr.Zero;
        EnumWindows((h,p) => { if (IsWindowVisible(h)) { var b=new StringBuilder(512); GetWindowText(h,b,b.Capacity); if (b.ToString().IndexOf(part,StringComparison.OrdinalIgnoreCase)>=0) { found=h; return false; } } return true; },IntPtr.Zero);
        return found;
    }
    public static bool Place(string part,int x,int y,int w,int h,bool foreground) {
        var hwnd=Find(part); if(hwnd==IntPtr.Zero) return false;
        ShowWindow(hwnd,9);
        SetWindowPos(hwnd,IntPtr.Zero,x,y,w,h,0x0040);
        if(foreground) SetForegroundWindow(hwnd);
        return true;
    }
    public static bool Maximize(string part) {
        var hwnd=Find(part); if(hwnd==IntPtr.Zero) return false;
        ShowWindow(hwnd,3); SetForegroundWindow(hwnd); return true;
    }
}
'@
    }
}

function Get-WindowTitles {
    Get-DesktopWindowApi
    return @([ANCVisualDemoWindows]::Titles())
}

function Set-DemoWindowLayout([bool]$WithRviz) {
    Add-Type -AssemblyName System.Windows.Forms
    $area = [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea
    $gazebo = @(Get-WindowTitles | Where-Object { $_ -match '(?i)gazebo' } | Select-Object -First 1)
    $rvizTitle = @(Get-WindowTitles | Where-Object { $_ -match '(?i)rviz' } | Select-Object -First 1)
    $gazeboFound = [bool]$gazebo
    $rvizFound = if ($WithRviz) { [bool]$rvizTitle } else { $null }
    $consoleFound = if ($WithRviz) { $false } else { $null }
    if ($WithRviz) {
        $consoleHeight = [math]::Min(190,[int]($area.Height * 0.24))
        $displayHeight = $area.Height - $consoleHeight
        if ($gazeboFound -and $rvizFound) {
            $leftW = [int]($area.Width * 0.58)
            [void][ANCVisualDemoWindows]::Place('Gazebo',$area.X,$area.Y,$leftW,$displayHeight,$true)
            [void][ANCVisualDemoWindows]::Place('RViz',($area.X+$leftW),$area.Y,($area.Width-$leftW),$displayHeight,$false)
        } elseif ($gazeboFound) {
            [void][ANCVisualDemoWindows]::Place('Gazebo',$area.X,$area.Y,$area.Width,$displayHeight,$true)
        }
        $consoleFound = [ANCVisualDemoWindows]::Place('Autonomous Navigation Car',$area.X,($area.Y+$displayHeight),$area.Width,$consoleHeight,$false)
    } elseif ($gazeboFound) {
        [void][ANCVisualDemoWindows]::Maximize('Gazebo')
    }
    return [pscustomobject]@{ Gazebo=$gazeboFound; Rviz=$rvizFound; Console=$consoleFound; Titles=(Get-WindowTitles) }
}

function Quote-ProcessArg([string]$Value) {
    if ($Value -notmatch '[\s"]') { return $Value }
    return '"' + $Value.Replace('"','\"') + '"'
}

function Build-ArgumentLine([string[]]$Arguments) {
    return (($Arguments | ForEach-Object { Quote-ProcessArg ([string]$_) }) -join ' ')
}

function Get-FFmpeg {
    $ffmpeg = Get-ChildItem -LiteralPath $ToolRoot -Filter ffmpeg.exe -File -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($ffmpeg) { return $ffmpeg.FullName }

    Write-Host 'Fetching a package-local FFmpeg Windows build. No system-wide installation or PATH change will be made.'
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    $headers = @{ 'User-Agent'='AutonomousCar-Visual-Demo'; 'Accept'='application/vnd.github+json' }
    $release = Invoke-RestMethod -Uri 'https://api.github.com/repos/BtbN/FFmpeg-Builds/releases/latest' -Headers $headers -TimeoutSec 45
    $zipAsset = @($release.assets | Where-Object { $_.name -eq 'ffmpeg-master-latest-win64-gpl.zip' } | Select-Object -First 1)
    $sumAsset = @($release.assets | Where-Object { $_.name -eq 'checksums.sha256' } | Select-Object -First 1)
    if (-not $zipAsset -or -not $sumAsset) { throw 'The FFmpeg release did not provide the expected Windows archive and SHA-256 checksum file.' }

    $archivePath = Join-Path $ToolRoot 'ffmpeg-master-latest-win64-gpl.zip'
    $checksumPath = Join-Path $ToolRoot 'ffmpeg-checksums.sha256'
    $extractPath = Join-Path $ToolRoot ('ffmpeg-' + ([string]$release.tag_name -replace '[^A-Za-z0-9._-]','_'))
    if (-not (Test-Path -LiteralPath $archivePath)) {
        Invoke-WebRequest -Uri $zipAsset[0].browser_download_url -Headers @{ 'User-Agent'='AutonomousCar-Visual-Demo' } -OutFile $archivePath -UseBasicParsing -TimeoutSec 600
    }
    Invoke-WebRequest -Uri $sumAsset[0].browser_download_url -Headers @{ 'User-Agent'='AutonomousCar-Visual-Demo' } -OutFile $checksumPath -UseBasicParsing -TimeoutSec 60
    $checksumText = Get-Content -Raw -LiteralPath $checksumPath
    $sumLine = @($checksumText -split "`r?`n" | Where-Object { $_ -match [regex]::Escape($zipAsset[0].name) } | Select-Object -First 1)
    if (-not $sumLine) { throw 'The downloaded FFmpeg checksum file does not contain the selected archive name.' }
    $expectedHash = (($sumLine[0].Trim() -split '\s+')[0]).ToLowerInvariant()
    $actualHash = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actualHash -ne $expectedHash) {
        Remove-Item -LiteralPath $archivePath -Force
        throw "FFmpeg archive SHA-256 mismatch. Expected $expectedHash, got $actualHash. The mismatched archive was removed."
    }
    if (Test-Path -LiteralPath $extractPath) {
        throw "FFmpeg extraction folder already exists without a detected ffmpeg.exe: $extractPath. Inspect it before retrying."
    }
    New-Item -ItemType Directory -Path $extractPath | Out-Null
    Expand-Archive -LiteralPath $archivePath -DestinationPath $extractPath
    $ffmpeg = Get-ChildItem -LiteralPath $extractPath -Filter ffmpeg.exe -File -Recurse | Select-Object -First 1
    if (-not $ffmpeg) { throw 'FFmpeg archive was verified but contained no ffmpeg.exe.' }
    Set-Content -LiteralPath (Join-Path $ToolRoot 'ffmpeg-source.txt') -Encoding UTF8 -Value @("Repository: https://github.com/BtbN/FFmpeg-Builds", "Release tag: $($release.tag_name)", "Archive SHA-256: $actualHash", "Archive: $($zipAsset[0].browser_download_url)")
    return $ffmpeg.FullName
}

function Invoke-FFmpegWait([string]$Exe, [string[]]$Arguments, [string]$StdoutPath, [string]$StderrPath) {
    $proc = Start-Process -FilePath $Exe -ArgumentList (Build-ArgumentLine $Arguments) -WorkingDirectory $PackageRoot -WindowStyle Hidden -PassThru -Wait -RedirectStandardOutput $StdoutPath -RedirectStandardError $StderrPath
    return $proc.ExitCode
}

function Test-Encoders([string]$Ffmpeg) {
    $encLog = Join-Path $LogRoot 'ffmpeg_encoders.txt'
    $encErr = Join-Path $LogRoot 'ffmpeg_encoders_stderr.txt'
    $encExit = Invoke-FFmpegWait $Ffmpeg @('-hide_banner','-encoders') $encLog $encErr
    $encoderText = if (Test-Path -LiteralPath $encLog) { Get-Content -Raw -LiteralPath $encLog } else { '' }
    $probe = Join-Path $LogRoot 'encoder_probe.mp4'
    $probeErr = Join-Path $LogRoot 'encoder_probe.log'
    if ($encExit -eq 0 -and $encoderText -match '\bh264_nvenc\b') {
        $exit = Invoke-FFmpegWait $Ffmpeg @('-hide_banner','-loglevel','error','-f','lavfi','-i','color=c=black:s=640x360:r=15:d=1','-frames:v','15','-c:v','h264_nvenc','-preset','p1','-pix_fmt','yuv420p','-y',$probe) (Join-Path $LogRoot 'encoder_probe_stdout.txt') $probeErr
        if ($exit -eq 0 -and (Test-Path -LiteralPath $probe) -and (Get-Item -LiteralPath $probe).Length -gt 1000) {
            $script:Encoder = 'h264_nvenc'
            $script:EncoderLabel = 'GTX 1650 H.264 NVENC (probe passed)'
            Write-Host 'H.264 NVENC probe passed; recording will use the GPU encoder.'
            return
        }
        Write-Host 'H.264 NVENC probe failed; switching to software H.264. See logs\encoder_probe.log.' -ForegroundColor Yellow
    } else {
        Write-Host 'This FFmpeg build does not advertise h264_nvenc; using software H.264.'
    }
    $exit = Invoke-FFmpegWait $Ffmpeg @('-hide_banner','-loglevel','error','-f','lavfi','-i','color=c=black:s=640x360:r=15:d=1','-frames:v','15','-c:v','libx264','-preset','ultrafast','-crf','30','-pix_fmt','yuv420p','-y',$probe) (Join-Path $LogRoot 'encoder_probe_stdout.txt') (Join-Path $LogRoot 'encoder_probe.log')
    if ($exit -ne 0 -or -not (Test-Path -LiteralPath $probe) -or (Get-Item -LiteralPath $probe).Length -le 1000) {
        throw 'Both the NVENC and software H.264 encoder probes failed; no simulation will be started.'
    }
    $script:Encoder = 'libx264'
    $script:EncoderLabel = 'software H.264 (probe passed)'
    Write-Host 'Software H.264 probe passed.'
}

function Start-Recorder([string]$Ffmpeg, [string]$Title, [string]$VideoPath, [string]$RecorderLog) {
    $filter = 'scale=1280:720:force_original_aspect_ratio=decrease:force_divisible_by=2,pad=1280:720:(ow-iw)/2:(oh-ih)/2:black'
    if ($script:Encoder -eq 'h264_nvenc') {
        $codecArgs = @('-c:v','h264_nvenc','-preset','p1','-rc','vbr','-cq','29','-b:v','0')
    } else {
        $codecArgs = @('-c:v','libx264','-preset','ultrafast','-crf','29')
    }
    $ffArgs = @('-hide_banner','-loglevel','error','-nostats','-rtbufsize','64M','-f','gdigrab','-framerate','15','-draw_mouse','1','-i','desktop','-vf',$filter) + $codecArgs + @('-pix_fmt','yuv420p','-an','-metadata',('title=' + $Title),'-movflags','+faststart','-y',$VideoPath)
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $Ffmpeg
    $psi.Arguments = Build-ArgumentLine $ffArgs
    $psi.WorkingDirectory = $PackageRoot
    $psi.UseShellExecute = $false
    $psi.CreateNoWindow = $true
    $psi.RedirectStandardInput = $true
    $psi.RedirectStandardError = $true
    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $psi
    $script:RecorderLogPath = $RecorderLog
    Set-Content -LiteralPath $RecorderLog -Value ('Encoder=' + $script:EncoderLabel + '; Source=Windows gdigrab desktop; Resolution target=1280x720; FPS=15') -Encoding UTF8
    if (-not $process.Start()) { throw 'FFmpeg screen recorder did not start.' }
    $script:Recorder = $process
    Start-Sleep -Seconds 2
    Drain-RecorderLog
    if ($process.HasExited) { throw "FFmpeg screen recorder exited during capture startup (exit $($process.ExitCode))." }
    Write-Host "Recording: $VideoPath"
}

function Drain-RecorderLog {
    if ($null -eq $script:Recorder) { return }
    try {
        while ($script:Recorder.StandardError.Peek() -ge 0) {
            $line = $script:Recorder.StandardError.ReadLine()
            if ($null -ne $line) { Add-Content -LiteralPath $script:RecorderLogPath -Value $line -Encoding UTF8 }
        }
    } catch { }
}

function Stop-Recorder {
    if ($null -eq $script:Recorder) { return }
    $p = $script:Recorder
    try {
        if (-not $p.HasExited) {
            $p.StandardInput.WriteLine('q')
            $p.StandardInput.Flush()
            if (-not $p.WaitForExit(20000)) {
                Add-Content -LiteralPath $script:RecorderLogPath -Value 'FFmpeg did not stop after q within 20 seconds; terminating only the recorder started by this package.'
                $p.Kill()
                [void]$p.WaitForExit(5000)
            }
        }
    } catch { Add-Content -LiteralPath $script:RecorderLogPath -Value ('Recorder stop error: ' + $_.Exception.Message) }
    Drain-RecorderLog
    try { $p.Dispose() } catch { }
    $script:Recorder = $null
}

function Test-VideoFile([string]$Ffmpeg, [string]$VideoPath, [string]$Id) {
    $ffprobe = Join-Path (Split-Path -Parent $Ffmpeg) 'ffprobe.exe'
    if (-not (Test-Path -LiteralPath $ffprobe)) { return $false }
    $probeOut = Join-Path $LogRoot ($Id + '_ffprobe.txt')
    $probeErr = Join-Path $LogRoot ($Id + '_ffprobe_error.txt')
    $exit = Invoke-FFmpegWait $ffprobe @('-v','error','-show_entries','format=duration:stream=codec_name,width,height','-of','default=noprint_wrappers=1',$VideoPath) $probeOut $probeErr
    $text = if (Test-Path -LiteralPath $probeOut) { Get-Content -Raw -LiteralPath $probeOut } else { '' }
    $durationMatch = [regex]::Match($text,'(?m)^duration=([0-9.]+)$')
    $good = $exit -eq 0 -and $durationMatch.Success -and [double]::Parse($durationMatch.Groups[1].Value,[Globalization.CultureInfo]::InvariantCulture) -ge 4.0 -and (Test-Path -LiteralPath $VideoPath) -and (Get-Item -LiteralPath $VideoPath).Length -gt 20000
    if ($good) {
        $shot = Join-Path $ScreenshotRoot ($Id + '_' + $script:RunTag + '.png')
        $shotExit = Invoke-FFmpegWait $Ffmpeg @('-hide_banner','-loglevel','error','-ss','1','-i',$VideoPath,'-frames:v','1','-y',$shot) (Join-Path $LogRoot ($Id + '_screenshot_stdout.txt')) (Join-Path $LogRoot ($Id + '_screenshot_error.txt'))
        if ($shotExit -eq 0 -and (Test-Path -LiteralPath $shot) -and (Get-Item -LiteralPath $shot).Length -gt 10000) { return $true }
        return $false
    }
    return $false
}

function Copy-ProjectEvidence([string]$LinuxPath, [string]$Destination) {
    if ([string]::IsNullOrWhiteSpace($LinuxPath)) { return $false }
    $windowsSource = & $WslExe -d $Distro -- wslpath -w $LinuxPath 2>&1
    if ($LASTEXITCODE -ne 0) { Write-Host "Could not resolve evidence path: $LinuxPath"; return $false }
    $source = ([string]($windowsSource | Select-Object -Last 1)).Trim()
    if (-not (Test-Path -LiteralPath $source)) { Write-Host "Evidence folder not found: $source"; return $false }
    New-Item -ItemType Directory -Force -Path $Destination | Out-Null
    $names = if ($LinuxPath -match '/phase6/runs/') { @('result.json','launch.log','rosbag.log','truth_trajectory.csv','navigation_trace.png','global_costmap.png') } else { @('demo-status.txt','phase8c-demo-status.json','ros2-launch.log','scenario-generation.log') }
    $copied = 0
    foreach ($name in $names) {
        $candidate = Join-Path $source $name
        if (Test-Path -LiteralPath $candidate -PathType Leaf) { Copy-Item -LiteralPath $candidate -Destination $Destination -Force; $copied++ }
    }
    if ($LinuxPath -notmatch '/phase6/runs/') {
        $manifest = Join-Path $source 'scenario\manifest.json'
        if (Test-Path -LiteralPath $manifest -PathType Leaf) { Copy-Item -LiteralPath $manifest -Destination (Join-Path $Destination 'scenario-manifest.json') -Force; $copied++ }
    }
    return ($copied -gt 0)
}

function Invoke-TrackedStop {
    $result = Invoke-WslText "cd $ProjectRoot && bash scripts/windows_demo_session.sh stop" 'gracefully stop only the exact tracked project session'
    return $result.ExitCode -eq 0 -and $result.Text -notmatch 'BLOCKED:'
}

function Get-SessionState {
    $result = Invoke-WslText "cd $ProjectRoot && bash scripts/windows_demo_session.sh status" 'check tracked project session state'
    return $result.Text.Trim()
}

function Write-NewSessionLogLines([string]$Path, [ref]$SeenCharacters) {
    if (-not (Test-Path -LiteralPath $Path)) { return }
    try {
        $content = Get-Content -Raw -LiteralPath $Path
        if ($null -eq $content) { return }
        if ($content.Length -lt $SeenCharacters.Value) { $SeenCharacters.Value = 0 }
        if ($content.Length -le $SeenCharacters.Value) { return }
        $newText = $content.Substring($SeenCharacters.Value)
        $SeenCharacters.Value = $content.Length
        foreach ($line in ($newText -split "`r?`n")) {
            if (-not [string]::IsNullOrWhiteSpace($line)) { Write-Host ('SIM> ' + $line) -ForegroundColor DarkGray }
        }
    } catch { }
}

function Get-WslSimulationProcesses {
    $old = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $lines = & $WslExe -d $Distro -- ps -eo 'pid=,pgid=,comm=,args=' 2>&1
        $code = $LASTEXITCODE
    } finally { $ErrorActionPreference = $old }
    if ($code -ne 0) { throw "Could not inspect WSL process list: $($lines -join ' ')" }
    $found = New-Object System.Collections.Generic.List[string]
    foreach ($line in $lines) {
        $parts = ([string]$line).Trim() -split '\s+',4
        if ($parts.Count -lt 4) { continue }
        $comm = $parts[2]
        $args = $parts[3]
        $isSimulator = $comm -in @('gz','gzserver','gzclient','rviz2')
        $isRosLaunch = ($comm -eq 'ros2' -and $args -match '\blaunch\b') -or ($comm -eq 'python3' -and $args -match 'ros2 launch')
        $isLidarLauncher = $comm -eq 'bash' -and $args -match 'run_lidar_demo\.sh'
        if ($isSimulator -or $isRosLaunch -or $isLidarLauncher) {
            $found.Add(([string]$line).Trim())
        }
    }
    return $found.ToArray()
}

function Start-TrackedSession([string]$Mode, [string]$Scenario, [string]$Id, [string]$LinuxLog, [string]$WindowsStdout, [string]$WindowsStderr) {
    $linuxHelper = Get-WslPath $LinuxHelperWindows
    $bashCommand = "exec bash '$linuxHelper' '$Mode' '$Scenario' 101 '$(if ($Mode -eq 'lidar') {'true'} else {'false'})' '$LinuxLog'"
    $argLine = '-d ' + $Distro + ' -- bash -lc "' + $bashCommand + '"'
    $process = Start-Process -FilePath $WslExe -ArgumentList $argLine -WindowStyle Hidden -PassThru -RedirectStandardOutput $WindowsStdout -RedirectStandardError $WindowsStderr
    return $process
}

function Get-Phase6EvidencePath([string]$SessionLog) {
    if (-not (Test-Path -LiteralPath $SessionLog)) { return $null }
    $text = Get-Content -Raw -LiteralPath $SessionLog
    $m = [regex]::Match($text,'(?m)^Session ID: (windows-phase6-[^\s]+)')
    if (-not $m.Success) { return $null }
    return "$ProjectRoot/docs/evidence/phase6/runs/$($m.Groups[1].Value)"
}

function Get-LidarEvidencePath([string]$SessionLog) {
    if (-not (Test-Path -LiteralPath $SessionLog)) { return $null }
    $text = Get-Content -Raw -LiteralPath $SessionLog
    $m = [regex]::Match($text,'(?m)^Evidence saved: (.+)$')
    if ($m.Success) { return $m.Groups[1].Value.Trim() }
    return $null
}

function Invoke-Demo([string]$Id, [string]$Mode, [string]$Scenario, [int]$MinHostMiB, [bool]$WithRviz, [int]$DurationSeconds, [string]$Ffmpeg) {
    Write-Section "$Id : $Mode / $Scenario"
    $record = New-DemoResult $Id $Mode $Scenario 'PENDING' ''
    $memory = Get-ResourceSnapshot
    $record.resources_before = $memory
    Write-Host "Windows available: $($memory.WindowsAvailableMiB) MiB; WSL available: $($memory.WslAvailableMiB) MiB; WSL project free: $($memory.ProjectDiskFreeMiB) MiB; package drive free: $($memory.PackageDiskFreeMiB) MiB"
    if ($memory.WindowsAvailableMiB -lt $MinHostMiB -or $memory.WslAvailableMiB -lt 1536 -or $memory.ProjectDiskFreeMiB -lt 3072 -or $memory.PackageDiskFreeMiB -lt 4096) {
        $reason = "Resource gate: require Windows >= $MinHostMiB MiB, WSL >= 1536 MiB, WSL project disk >= 3072 MiB, package drive >= 4096 MiB; observed Windows $($memory.WindowsAvailableMiB), WSL $($memory.WslAvailableMiB), WSL disk $($memory.ProjectDiskFreeMiB), package disk $($memory.PackageDiskFreeMiB) MiB."
        $blocked = Write-BlockedResult $Id $Mode $Scenario $reason
        return [pscustomobject]$blocked
    }

    $preflight = Invoke-WslText "cd $ProjectRoot && bash scripts/windows_demo_launcher.sh diagnostics" 'run project read-only preflight'
    if ($preflight.ExitCode -ne 0 -or $preflight.Text -match '(?m)^BLOCKED:|untracked project simulation process|These project processes') {
        $blocked = Write-BlockedResult $Id $Mode $Scenario 'Project diagnostics reported a blocked environment or an untracked project simulation process. Inspect the launcher transcript; no process was killed.'
        return [pscustomobject]$blocked
    }
    $state = Get-SessionState
    if ($state -notmatch 'NO_ACTIVE_PROJECT_SESSION') {
        if (-not (Invoke-TrackedStop)) {
            $blocked = Write-BlockedResult $Id $Mode $Scenario 'The exact tracked project session did not stop cleanly; no new simulation was started.'
            return [pscustomobject]$blocked
        }
    }
    $state = Get-SessionState
    if ($state -notmatch 'NO_ACTIVE_PROJECT_SESSION') {
        $blocked = Write-BlockedResult $Id $Mode $Scenario 'A project session remains active after the graceful stop attempt. No competing simulation was started.'
        return [pscustomobject]$blocked
    }
    $running = @(Get-WslSimulationProcesses)
    if ($running.Count -gt 0) {
        Write-Host 'Untracked WSL simulator/ROS launch process found; leaving it untouched:' -ForegroundColor Yellow
        $running | ForEach-Object { Write-Host $_ }
        $blocked = Write-BlockedResult $Id $Mode $Scenario 'An untracked WSL Gazebo/RViz/ROS launch is active. It was not stopped; no new simulation was started.'
        return [pscustomobject]$blocked
    }

    $modeLabel = if ($Mode -eq 'phase6') { 'Reference-Map Nav2 Demonstration' } else { 'Realistic RGB + LiDAR Perception Demonstration' }
    $scenarioLabel = if ($Scenario -eq 'S00') { 'S00 Empty arena' } elseif ($Scenario -eq 'S01') { 'S01 Box obstacle' } else { 'S02 Sensing only' }
    $title = "Autonomous Navigation Car - $modeLabel - $scenarioLabel"
    $filePrefix = if ($Mode -eq 'phase6') { 'ANC_ReferenceMap_Nav2' } else { 'ANC_Realistic_RGB_LiDAR_Perception' }
    $video = Join-Path $VideoRoot ($filePrefix + '_' + $Scenario + '_' + $script:RunTag + '.mp4')
    $recLog = Join-Path $LogRoot ($Id + '_' + $script:RunTag + '_ffmpeg.log')
    $sessionLogWindows = Join-Path $LogRoot ($Id + '_' + $script:RunTag + '_wsl_session.log')
    $sessionLog = Get-WslPath $sessionLogWindows
    $stdout = Join-Path $LogRoot ($Id + '_' + $script:RunTag + '_wsl_stdout.log')
    $stderr = Join-Path $LogRoot ($Id + '_' + $script:RunTag + '_wsl_stderr.log')
    $resultPath = Join-Path $script:RunRoot ($Id + '.json')
    $projectEvidence = $null
    $navResult = $null
    $process = $null
    $gazeboSeen = $false
    $rvizSeen = if ($WithRviz) { $false } else { 'not_required' }
    $statusConsoleSeen = if ($WithRviz) { $false } else { 'not_required' }
    $simExit = $false
    $stopClean = $false
    $goalLine = $null
    $seenSessionCharacters = 0
    $reason = ''
    $videoSaved = $false

    try {
        Start-Recorder $Ffmpeg $title $video $recLog
        $process = Start-TrackedSession $Mode $Scenario $Id $sessionLog $stdout $stderr
        Write-Host "Started tracked WSL session PID $($process.Id); awaiting Gazebo and real simulation results."
        $windowWait = [Diagnostics.Stopwatch]::StartNew()
        while ($windowWait.Elapsed.TotalSeconds -lt 55 -and -not $process.HasExited) {
            $layout = Set-DemoWindowLayout $WithRviz
            if ($layout.Gazebo) { $gazeboSeen = $true }
            if ($WithRviz -and $layout.Rviz) { $rvizSeen = $true }
            if ($WithRviz -and $layout.Console) { $statusConsoleSeen = $true }
            if ($gazeboSeen -and (-not $WithRviz -or $rvizSeen)) { break }
            Write-NewSessionLogLines $sessionLogWindows ([ref]$seenSessionCharacters)
            Start-Sleep -Seconds 2
            Drain-RecorderLog
        }
        if (-not $gazeboSeen) { Write-Host 'Gazebo window title was not detected yet; continuing to observe the session and recording. This will not count as an opened GUI.' -ForegroundColor Yellow }
        if ($WithRviz -and -not $rvizSeen) { Write-Host 'RViz window title was not detected yet; continuing to observe. LiDAR video will report RViz as unverified.' -ForegroundColor Yellow }

        $clock = [Diagnostics.Stopwatch]::StartNew()
        $goalWait = if ($Mode -eq 'phase6') { 220 } else { $DurationSeconds }
        $statusSample = $null
        while ($clock.Elapsed.TotalSeconds -lt $goalWait) {
            if ($process.HasExited) { $simExit = $true; break }
            $memoryNow = Get-HostMemoryMiB
            if ($memoryNow -lt 1024) { $reason = "Safety stop: Windows available RAM fell to $memoryNow MiB."; break }
            $layout = Set-DemoWindowLayout $WithRviz
            if ($layout.Gazebo) { $gazeboSeen = $true }
            if ($WithRviz -and $layout.Rviz) { $rvizSeen = $true }
            if ($WithRviz -and $layout.Console) { $statusConsoleSeen = $true }
            Write-NewSessionLogLines $sessionLogWindows ([ref]$seenSessionCharacters)
            if (Test-Path -LiteralPath $sessionLogWindows) {
                $liveText = Get-Content -Raw -LiteralPath $sessionLogWindows
                if ($Mode -eq 'phase6' -and $liveText -match '(?m)^Goal status:') {
                    $statusSample = [regex]::Matches($liveText,'(?m)^Goal status:.*$') | Select-Object -Last 1
                    if ($statusSample) { $goalLine = $statusSample.Value }
                    break
                }
            }
            Drain-RecorderLog
            Start-Sleep -Seconds 2
        }
        if ($Mode -eq 'phase6' -and $goalLine) {
            Write-Host $goalLine
            Start-Sleep -Seconds 7
        } elseif ($Mode -eq 'phase6' -and -not $reason) {
            $reason = "No Phase 6 goal result appeared within $goalWait seconds; video will be retained if valid."
        } elseif ($Mode -eq 'lidar') {
            if ($reason) { Write-Host "LiDAR sensing demonstration ended early: $reason" -ForegroundColor Yellow }
            else { Write-Host "LiDAR sensing demonstration recorded for $DurationSeconds seconds; Nav2 goal requests were not enabled." }
        }

        $stopClean = Invoke-TrackedStop
        if ($process) { [void]$process.WaitForExit(35000) }
        $simExit = if ($process) { $process.HasExited } else { $false }
        $sessionText = if (Test-Path -LiteralPath $sessionLogWindows) { Get-Content -Raw -LiteralPath $sessionLogWindows } else { '' }
        if ($Mode -eq 'phase6') { $projectEvidence = Get-Phase6EvidencePath $sessionLogWindows } else { $projectEvidence = Get-LidarEvidencePath $sessionLogWindows }
        if ($projectEvidence) {
            $evidenceDest = Join-Path $script:RunRoot ($Id + '_evidence')
            if (Copy-ProjectEvidence $projectEvidence $evidenceDest) { $record.evidence_folder = $evidenceDest }
        }
        Stop-Recorder
        $videoSaved = Test-VideoFile $Ffmpeg $video $Id

        if ($Mode -eq 'phase6' -and $record.evidence_folder -and (Test-Path -LiteralPath (Join-Path $record.evidence_folder 'result.json'))) {
            $navResult = Get-Content -Raw -LiteralPath (Join-Path $record.evidence_folder 'result.json') | ConvertFrom-Json
            $record.robot_spawned = $null -ne $navResult.metrics -and $null -ne $navResult.metrics.truth_path_length_m
            $record.robot_moved = $record.robot_spawned -and [double]$navResult.metrics.truth_path_length_m -gt 0.1
            $record.navigation_reached_goal = if ($navResult.action.status -eq 'SUCCEEDED') { $true } else { $false }
            $record.phase6_classification = $navResult.classification
            $record.path_length_m = $navResult.metrics.truth_path_length_m
            $record.final_goal_error_m = $navResult.metrics.final_xy_goal_error_m
            $record.collision_events = $navResult.metrics.contact_collision_events
            $record.final_stop_verified = $navResult.metrics.post_action_robot_stopped
            $record.clean_shutdown = [bool]$navResult.shutdown.gazebo_child_shutdown_clean -and [bool]$navResult.shutdown.group_gone
        } elseif ($Mode -eq 'lidar' -and $record.evidence_folder -and (Test-Path -LiteralPath (Join-Path $record.evidence_folder 'phase8c-demo-status.json'))) {
            $sensor = Get-Content -Raw -LiteralPath (Join-Path $record.evidence_folder 'phase8c-demo-status.json') | ConvertFrom-Json
            $counts = $sensor.sensor_counts
            $record.robot_spawned = [int]$counts.'/realistic/scan' -gt 0 -or [int]$counts.'/camera/image_raw' -gt 0
            $record.sensors_published = [ordered]@{
                scan=([int]$counts.'/realistic/scan' -gt 0); camera=([int]$counts.'/camera/image_raw' -gt 0)
                slam_map=([int]$counts.'/map' -gt 0); lidar_local_map=([int]$counts.'/realistic/local_map/lidar' -gt 0)
                counts=$counts
            }
            $record.robot_moved = $null -ne $sensor.odometry_displacement_since_monitor_start_m -and [double]$sensor.odometry_displacement_since_monitor_start_m -gt 0.01
            $record.navigation_reached_goal = 'not_run_by_design'
            $record.safety_state = $sensor.safety_state
            $record.guarded_forward_demo = $sensor.guarded_forward_demo
            $record.nav2_goal_navigation = $sensor.nav2_goal_navigation
            $record.clean_shutdown = $stopClean -and $simExit
        }
        $record.gazebo_opened = $gazeboSeen
        $record.rviz_opened = $rvizSeen
        $record.safety_console_visible = $statusConsoleSeen
        $record.video_path = $video
        $record.video_saved = $videoSaved
        $record.clean_shutdown = [bool]$record.clean_shutdown -and $stopClean -and $simExit
        $record.goal_log_line = $goalLine
        $record.session_process_exited = $simExit
        $record.tracked_session_stopped_cleanly = $stopClean
        $record.encoder = $script:EncoderLabel
        $record.reason = $reason
        $record.finished_at = (Get-Date).ToString('o')
        $validMovement = if ($Mode -eq 'phase6') { [bool]$record.robot_moved -and $record.navigation_reached_goal -eq $true } else { [bool]$record.robot_moved -or ($Scenario -eq 'S02' -and $record.sensors_published -is [System.Collections.IDictionary] -and $record.sensors_published.scan) }
        $record.status = if ($record.gazebo_opened -and $record.robot_spawned -and $record.video_saved -and $record.clean_shutdown -and $validMovement) { 'PASS' } else { 'FAIL_OR_PARTIAL' }
        if ($Mode -eq 'phase6' -and $record.navigation_reached_goal -ne $true) { $record.status='FAIL_OR_PARTIAL' }
        if ($Mode -eq 'lidar' -and -not $statusConsoleSeen) { $record.status='FAIL_OR_PARTIAL' }
        if ($Mode -eq 'lidar' -and $record.sensors_published -is [System.Collections.IDictionary] -and (-not $record.sensors_published.scan -or -not $record.sensors_published.camera -or -not $record.sensors_published.slam_map -or -not $record.sensors_published.lidar_local_map)) { $record.status='FAIL_OR_PARTIAL' }
        Save-Json $resultPath $record
        Write-Host "$($record.status) $Id | Gazebo=$($record.gazebo_opened) robot=$($record.robot_spawned) moved=$($record.robot_moved) goal=$($record.navigation_reached_goal) video=$($record.video_saved) shutdown=$($record.clean_shutdown)"
        if ($videoSaved) { Write-Host "Video: $video" }
        return [pscustomobject]$record
    } catch {
        $reason = $_.Exception.Message
        Write-Host "Demo exception: $reason" -ForegroundColor Red
        try { $stopClean = Invoke-TrackedStop } catch { }
        try { Stop-Recorder } catch { }
        $record.status = 'FAIL_OR_PARTIAL'
        $record.reason = $reason
        $record.video_path = $video
        $record.video_saved = Test-VideoFile $Ffmpeg $video $Id
        $record.gazebo_opened = $gazeboSeen
        $record.rviz_opened = $rvizSeen
        $record.safety_console_visible = $statusConsoleSeen
        $record.tracked_session_stopped_cleanly = $stopClean
        $record.finished_at = (Get-Date).ToString('o')
        Save-Json $resultPath $record
        return [pscustomobject]$record
    } finally {
        try { Stop-Recorder } catch { }
        if ($process) { try { $process.Dispose() } catch { } }
    }
}

function Write-RunSummary([object[]]$Results, [string]$Extra) {
    $lines = New-Object System.Collections.Generic.List[string]
    $lines.Add('AUTONOMOUS NAVIGATION CAR - VISUAL DEMONSTRATION SUMMARY')
    $lines.Add('Generated: ' + (Get-Date).ToString('o'))
    $lines.Add('')
    $lines.Add('This package reports only observed GUI, simulation, result JSON, sensor counters and video files.')
    $lines.Add('Phase 6 is predefined-map ORACLE_REFERENCE Nav2. LiDAR runs do not enable Nav2 goals or production turning.')
    $lines.Add('')
    foreach ($r in $Results) {
        if ($null -eq $r) { continue }
        $lines.Add("$($r.demo_id): $($r.status) | $($r.mode) $($r.scenario)")
        $lines.Add("  Gazebo opened=$($r.gazebo_opened); RViz=$($r.rviz_opened); safety console visible=$($r.safety_console_visible); robot spawned=$($r.robot_spawned); sensors=$($r.sensors_published); moved=$($r.robot_moved); goal=$($r.navigation_reached_goal)")
        $lines.Add("  video saved=$($r.video_saved); clean shutdown=$($r.clean_shutdown); reason=$($r.reason)")
        if ($r.video_path) { $lines.Add('  video=' + $r.video_path) }
        if ($r.evidence_folder) { $lines.Add('  evidence=' + $r.evidence_folder) }
    }
    if ($Extra) { $lines.Add(''); $lines.Add($Extra) }
    $lines.Add('')
    $lines.Add('No live autonomous parking or RGB+LiDAR Nav2 goal performance is claimed.')
    $lines.Add('Detailed results folder: ' + $script:RunRoot)
    $summaryPath = Join-Path $ResultRoot 'RUN_SUMMARY.txt'
    $lines | Set-Content -LiteralPath $summaryPath -Encoding UTF8
    $lines | Set-Content -LiteralPath (Join-Path $script:RunRoot 'RUN_SUMMARY.txt') -Encoding UTF8
    Write-Host ''
    Write-Host ($lines -join [Environment]::NewLine)
    Write-Host "Summary saved: $summaryPath"
}

try {
    Write-Section 'AUTONOMOUS NAVIGATION CAR - VISUAL DEMONSTRATION'
    Write-Host "Package: $PackageRoot"
    Write-Host "Date: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss zzz')"
    Write-Host 'The project branch and ROS installations will not be changed.'
    Write-Host 'No unrelated Windows process will be closed.'

    $results = New-Object System.Collections.Generic.List[object]
    $memory = Get-ResourceSnapshot
    Write-Host "Windows available RAM: $($memory.WindowsAvailableMiB) MiB"
    Write-Host "WSL available RAM: $($memory.WslAvailableMiB) MiB"
    Write-Host "WSL project filesystem free: $($memory.ProjectDiskFreeMiB) MiB"
    Write-Host "Package drive free: $($memory.PackageDiskFreeMiB) MiB"
    $diag = Invoke-WslText "cd $ProjectRoot && bash scripts/windows_demo_launcher.sh diagnostics" 'read-only project, branch, WSLg, package and process diagnostics'
    $diagBlocked = $diag.ExitCode -ne 0 -or $diag.Text -match '(?m)^BLOCKED:|untracked project simulation process|These project processes'
    $lowForPhase6 = $memory.WindowsAvailableMiB -lt 2560 -or $memory.WslAvailableMiB -lt 1536 -or $memory.ProjectDiskFreeMiB -lt 3072 -or $memory.PackageDiskFreeMiB -lt 4096
    if ($PreflightOnly -or $diagBlocked -or $lowForPhase6) {
        foreach ($item in @(@('A1','phase6','S00'),@('A2','phase6','S01'),@('B1','lidar','S00'),@('B2','lidar','S02'))) {
            $minimum = if ($item[1] -eq 'lidar') { 3328 } else { 2560 }
            $reason = if ($diagBlocked) { 'The existing project diagnostics reported a blocked environment/process state.' } else { "Requires Windows >= $minimum MiB, WSL >= 1536 MiB, WSL project disk >= 3072 MiB and package drive >= 4096 MiB. Observed Windows $($memory.WindowsAvailableMiB), WSL $($memory.WslAvailableMiB), WSL disk $($memory.ProjectDiskFreeMiB), package disk $($memory.PackageDiskFreeMiB) MiB." }
            $blocked = Write-BlockedResult $item[0] $item[1] $item[2] $reason
            $blocked | Add-Member -NotePropertyName resources_observed -NotePropertyValue $memory -Force
            Save-Json (Join-Path $script:RunRoot ($item[0] + '.json')) $blocked
            $results.Add($blocked)
        }
        Write-RunSummary $results.ToArray() $(if ($PreflightOnly) { 'Preflight only: no simulation, FFmpeg download, or video capture was started.' } else { 'Stopped before launch: free memory and double-click RUN_VISUAL_DEMO.bat again.' })
        return
    }

    $ffmpeg = Get-FFmpeg
    Test-Encoders $ffmpeg

    $a1 = Invoke-Demo 'A1_Phase6_S00' 'phase6' 'S00' 2560 $false 0 $ffmpeg
    $results.Add($a1)
    $a2 = Invoke-Demo 'A2_Phase6_S01' 'phase6' 'S01' 2560 $false 0 $ffmpeg
    $results.Add($a2)
    $b1 = Invoke-Demo 'B1_LiDAR_S00' 'lidar' 'S00' 3328 $true 45 $ffmpeg
    $results.Add($b1)
    $b2 = Invoke-Demo 'B2_LiDAR_S02' 'lidar' 'S02' 3328 $true 35 $ffmpeg
    $results.Add($b2)
    Write-RunSummary $results.ToArray() 'The four scenarios were attempted sequentially with separate video files.'
} catch {
    $fatalText = $_.Exception.Message
    Write-Host ('FATAL LAUNCHER ERROR: ' + $fatalText) -ForegroundColor Red
    $errorPath = Join-Path $script:RunRoot 'LAUNCHER_ERROR.txt'
    Set-Content -LiteralPath $errorPath -Value ($_ | Out-String) -Encoding UTF8
    Write-Host "Error detail saved: $errorPath"
    if ($null -eq $results) { $results = New-Object System.Collections.Generic.List[object] }
    foreach ($item in @(@('A1_Phase6_S00','phase6','S00'),@('A2_Phase6_S01','phase6','S01'),@('B1_LiDAR_S00','lidar','S00'),@('B2_LiDAR_S02','lidar','S02'))) {
        if (-not (@($results | Where-Object { $_.demo_id -eq $item[0] }).Count)) {
            $results.Add((Write-BlockedResult $item[0] $item[1] $item[2] "Not run because launcher setup stopped: $fatalText"))
        }
    }
    try { Write-RunSummary $results.ToArray() ('Launcher stopped before completing the requested sequence: ' + $fatalText) } catch { }
} finally {
    try { Stop-Recorder } catch { }
    if ($script:TranscriptActive) { try { Stop-Transcript | Out-Null } catch { } }
    Write-Host ''
    Write-Host 'The final status and media paths are recorded in results\RUN_SUMMARY.txt when a run reached summary creation.'
    Write-Host 'Review the summary above. Close this window when you are finished.'
}
