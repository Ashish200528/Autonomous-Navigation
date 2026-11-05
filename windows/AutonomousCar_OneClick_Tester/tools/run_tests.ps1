param([string]$LauncherPath,[switch]$DryRun,[string]$Distro="Ubuntu-24.04")
$ErrorActionPreference="Stop"
$tools=$PSScriptRoot; $root=Split-Path -Parent $tools; $runs=Join-Path $root "runs"
$latest=Join-Path $root "latest"; $settingsPath=Join-Path $root "config\test_settings.json"
$runnerWin=Join-Path $tools "wsl_test_runner.sh"; $collectorWin=Join-Path $tools "collect_results.py"
$monitorWin=Join-Path $tools "resource_monitor.ps1"; $projectWsl="/home/ashish/AutonomousNavigationCar"
$wslExe=(Get-Command "wsl.exe" -ErrorAction Stop).Source
$controller=$null; $runWsl=$null; $runnerWsl=$null; $collectorWsl=$null; $runDirectory=$null
$watchdogAction=$null; $script:MonitoringErrors=New-Object System.Collections.Generic.List[string]

function ConvertTo-WslPath([string]$Value) {
    $r=@(& $wslExe --distribution $Distro --exec wslpath -a -u $Value 2>$null)
    if($LASTEXITCODE -ne 0 -or $r.Count -eq 0){throw "Could not convert path to WSL: $Value"}
    return [string]$r[-1]
}
function Quote-WindowsArgument([string]$Value) {
    $v=[string]$Value
    if($v -notmatch '[\s"]'){return $v}
    $v=$v -replace '(\\*)"', '$1$1\"'
    $v=$v -replace '(\\+)$', '$1$1'
    return '"' + $v + '"'
}
function Invoke-WslCollector([string[]]$Arguments) {
    $o=@(& $wslExe --distribution $Distro --exec python3 $collectorWsl @Arguments 2>&1)
    $c=$LASTEXITCODE; if($o.Count){$o|ForEach-Object{Write-Host ([string]$_)}}; return $c
}
function Request-GracefulStop([string]$Reason) {
    if($null -eq $controller -or $controller.HasExited){return}
    Write-Host "Requesting graceful WSL SIGINT: $Reason"
    try {
        $o=@(& $wslExe --distribution $Distro --exec bash $runnerWsl --stop $runWsl 2>&1)
        $c=$LASTEXITCODE
        $o|Out-String|Add-Content -LiteralPath (Join-Path $runDirectory "resource_monitoring\watchdog-stop.log") -Encoding UTF8
        if($c -ne 0){$script:MonitoringErrors.Add("Graceful stop returned $c.")}
    } catch {$script:MonitoringErrors.Add("Graceful stop failed: $($_.Exception.Message)")}
    $controller.Refresh(); if(-not $controller.HasExited){try{$controller.WaitForExit(35000)}catch{}}
}

try {
    if(-not (Test-Path -LiteralPath $settingsPath -PathType Leaf)){throw "Missing settings: $settingsPath"}
    foreach($p in @($runnerWin,$collectorWin,$monitorWin)){if(-not(Test-Path -LiteralPath $p -PathType Leaf)){throw "Missing helper: $p"}}
    $settings=Get-Content -Raw -LiteralPath $settingsPath|ConvertFrom-Json
    $distros=@(& $wslExe --list --quiet 2>&1)
    if($LASTEXITCODE -ne 0){throw "WSL distribution listing failed: $($distros -join ' ')"}
    $names=@($distros|ForEach-Object{([string]$_).Trim().TrimStart('*').Trim()})
    if($names -notcontains $Distro){throw "WSL distro '$Distro' is not registered. Available: $($names -join ', ')"}
    $mode=if($DryRun){"DRY_RUN"}else{"FULL"}
    $runId=(Get-Date).ToUniversalTime().ToString("yyyyMMddTHHmmssZ")+"_S00_"+[Guid]::NewGuid().ToString("N").Substring(0,8)
    $runDirectory=Join-Path $runs $runId; New-Item -ItemType Directory -Path $runDirectory -ErrorAction Stop|Out-Null
    foreach($d in @("environment","tests","ros_logs","gazebo_logs","maps","trajectories","resource_monitoring","config")) {
        New-Item -ItemType Directory -Path (Join-Path $runDirectory $d) -Force|Out-Null
    }
    Copy-Item -LiteralPath $settingsPath -Destination (Join-Path $runDirectory "config\test_settings.json")
    $wslStatus=New-Object System.Collections.Generic.List[string]
    foreach($label in @("version","status","list --verbose")) {
        $wslStatus.Add("=== wsl --$label ===")
        if($label -eq "list --verbose"){$o=@(& $wslExe --list --verbose 2>&1)}
        elseif($label -eq "status"){$o=@(& $wslExe --status 2>&1)}
        else{$o=@(& $wslExe --version 2>&1)}
        $wslStatus.AddRange([string[]]@($o|ForEach-Object{[regex]::Replace([string]$_,'\x00','')}))
        $wslStatus.Add("exit_code=$LASTEXITCODE")
    }
    $wslStatus|Set-Content -LiteralPath (Join-Path $runDirectory "environment\wsl_status.txt") -Encoding UTF8
    $gpuTool=Get-Command "nvidia-smi.exe" -ErrorAction SilentlyContinue
    if($gpuTool){
        $gpuExe=$gpuTool.Source
        $gpuLines=@(& $gpuExe --query-gpu=name,memory.total,memory.free --format=csv,noheader 2>&1)
        $gpuCode=$LASTEXITCODE
        @("command=nvidia-smi --query-gpu=name,memory.total,memory.free --format=csv,noheader",
          "exit_code=$gpuCode")+@($gpuLines|ForEach-Object{[string]$_}) |
            Set-Content -LiteralPath (Join-Path $runDirectory "environment\windows_gpu.txt") -Encoding UTF8
    } else {
        "nvidia-smi.exe not found; GPU model/VRAM could not be queried." |
            Set-Content -LiteralPath (Join-Path $runDirectory "environment\windows_gpu.txt") -Encoding UTF8
    }
    . $monitorWin -LibraryOnly
    try{[void](Write-StandaloneResourceSample -Directory $runDirectory -Distribution $Distro)}
    catch{$script:MonitoringErrors.Add("Initial resource sample failed: $($_.Exception.Message)")}
    $runnerWsl=ConvertTo-WslPath $runnerWin
    $collectorWsl=ConvertTo-WslPath $collectorWin
    $runWsl=ConvertTo-WslPath $runDirectory
    $settingsWsl=ConvertTo-WslPath (Join-Path $runDirectory "config\test_settings.json")
    $latestWsl=ConvertTo-WslPath $latest
    $init=@(& $wslExe --distribution $Distro --exec python3 $collectorWsl init --run-dir $runWsl --run-id $runId --mode $mode --scenario S00 --seed ([string]$settings.seed) 2>&1)
    if($LASTEXITCODE -ne 0){throw "Could not initialize manifest: $($init -join ' ')"}
    $controllerStart=(Get-Process -Id $PID).StartTime.ToUniversalTime().ToString("o")
    $dry=if($DryRun){"true"}else{"false"}
    $wslArgs=@("--distribution",$Distro,"--exec","setsid","--wait","bash",$runnerWsl,"--run",
        $runId,$runWsl,$settingsWsl,$projectWsl,[string]$PID,$dry,$controllerStart,$latestWsl)
    $argText=($wslArgs|ForEach-Object{Quote-WindowsArgument $_})-join " "
    $stdout=Join-Path $runDirectory "resource_monitoring\wsl-stdout.log"
    $stderr=Join-Path $runDirectory "resource_monitoring\wsl-stderr.log"
    Write-Host "Run ID: $runId"; Write-Host "Run folder: $runDirectory"
    Write-Host "Mode: $mode | headless S00 | Nav2 gated by current readiness"
    $controller=Start-Process -FilePath $wslExe -ArgumentList $argText -WorkingDirectory $root -PassThru -WindowStyle Hidden -RedirectStandardOutput $stdout -RedirectStandardError $stderr
    $started=Get-Date; $max=[int]$settings.resource.maximumTotalRuntimeSeconds
    while($true) {
        $controller.Refresh(); if($controller.HasExited){break}
        try{$sample=Write-StandaloneResourceSample -Directory $runDirectory -Distribution $Distro}
        catch{$sample=$null;$script:MonitoringErrors.Add("Resource sample failed: $($_.Exception.Message)")}
        if((Get-Date)-$started -gt [TimeSpan]::FromSeconds($max)){
            $watchdogAction="Maximum total runtime ($max seconds) reached."; Request-GracefulStop $watchdogAction; break
        }
        $statePath=Join-Path $runDirectory "environment\simulation_state.json"; $active=$false
        if(Test-Path -LiteralPath $statePath){try{$s=Get-Content -Raw -LiteralPath $statePath|ConvertFrom-Json;$active=$s.state -eq "RUNNING"}catch{}}
        if($active -and $null -ne $sample) {
            $abort=($sample.windowsAvailableMiB -lt [int]$settings.resource.abortWindowsAvailableMiB) -or
                ($sample.windowsMemoryLoadPercent -ge [int]$settings.resource.abortWindowsMemoryLoadPercent) -or
                ($null -ne $sample.wslAvailableMiB -and $sample.wslAvailableMiB -lt 1024)
            if($abort){$watchdogAction="Host/WSL RAM or load crossed an in-run abort threshold.";Request-GracefulStop $watchdogAction;break}
        }
        Start-Sleep -Seconds ([int]$settings.resource.monitorIntervalSeconds)
    }
    $controller.Refresh()
    if(-not $controller.HasExited){Request-GracefulStop "PowerShell monitoring loop ended"}
    if($controller.HasExited){
        $controller.WaitForExit()
        $exitFile=Join-Path $runDirectory "environment\worker_exit_code.txt"
        if(Test-Path -LiteralPath $exitFile){Write-Host "WSL worker exit code: $((Get-Content -Raw $exitFile).Trim())"}
        else{Write-Host "WSL process exited; worker exit code was not recorded."}
    }
    else{Write-Host "BLOCKED: worker remains after SIGINT; run state and logs were preserved."}
} catch {
    $watchdogAction=$_.Exception.Message; Write-Host "BLOCKED: $watchdogAction"
    if($runDirectory){$_|Out-String|Add-Content -LiteralPath (Join-Path $runDirectory "resource_monitoring\controller-errors.log") -Encoding UTF8}
} finally {
    if($null -ne $controller -and -not $controller.HasExited -and $runWsl -and $runnerWsl){Request-GracefulStop "PowerShell cleanup after interruption"}
}

if($runDirectory -and $runWsl -and $collectorWsl -and ($null -eq $controller -or $controller.HasExited)) {
    $manifestPath=Join-Path $runDirectory "RUN_MANIFEST.json"
    if(Test-Path -LiteralPath $manifestPath){
        try{
            $m=Get-Content -Raw -LiteralPath $manifestPath|ConvertFrom-Json
            if($m.overall_status -eq "RUNNING"){
                $reason=if($watchdogAction){$watchdogAction}else{"Worker exited before report finalization."}
                [void](Invoke-WslCollector @("mark-interrupted","--run-dir",$runWsl,"--reason",$reason))
                [void](Invoke-WslCollector @("finalize","--run-dir",$runWsl,"--latest-dir",$latestWsl))
            }
        }catch{Write-Host "Report finalization needs review: $($_.Exception.Message)"}
    }
}
if($runDirectory){
    foreach($e in $script:MonitoringErrors){$e|Add-Content -LiteralPath (Join-Path $runDirectory "resource_monitoring\monitor-errors.log") -Encoding UTF8}
    $summary=Join-Path $runDirectory "RUN_SUMMARY.txt"
    if(Test-Path -LiteralPath $summary){Write-Host "";Get-Content -LiteralPath $summary}
    else{Write-Host "Run evidence folder: $runDirectory";Write-Host "No finalized summary; inspect resource_monitoring and WSL logs."}
    Write-Host "Latest report: $latest\RUN_SUMMARY.txt"
}
$runnerExit=0
if($runDirectory){
    $summaryJson=Join-Path $runDirectory "RUN_SUMMARY.json"
    if(-not(Test-Path -LiteralPath $summaryJson)){$runnerExit=2}
    else{
        try{
            $report=Get-Content -Raw -LiteralPath $summaryJson|ConvertFrom-Json
            if($report.run.overall_status -in @("FAIL","TIMEOUT")){$runnerExit=1}
        }catch{$runnerExit=2}
    }
    if((Test-Path -LiteralPath (Join-Path $runDirectory "environment\orchestrator-error.log"))){$runnerExit=2}
}
exit $runnerExit
