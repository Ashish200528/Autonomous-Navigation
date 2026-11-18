# Copies this backend's code to the board over Wi-Fi (SSH) and restarts it under PM2.
# Keeps the board's own config/local.json, config/secret and node_modules.
#   powershell -ExecutionPolicy Bypass -File backend\tools\deploy-unoq.ps1 -Board 10.112.10.83
# The first run asks for the board's password twice (upload, then update) and installs this
# laptop's SSH key on the board, so later runs need no password.
param(
  [string]$Board = "10.112.10.83",
  [string]$User = "arduino",
  [string]$Dir = "anc/backend"   # PM2 app "anc-backend" runs ~/anc/backend
)
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)  # repo root (holds backend\)
$tar = Join-Path $env:TEMP "backend-update.tar"
$sh = Join-Path $env:TEMP "backend-update.sh"
$keyFile = Join-Path $env:USERPROFILE ".ssh\id_ed25519.pub"
# Windows' own tools: Git/MSYS tar and scp read "C:..." as a remote host "C".
$win = Join-Path $env:SystemRoot "System32"
$tarExe = Join-Path $win "tar.exe"; $scpExe = Join-Path $win "OpenSSH\scp.exe"; $sshExe = Join-Path $win "OpenSSH\ssh.exe"
$pub = if (Test-Path $keyFile) { (Get-Content $keyFile -Raw).Trim() } else { "" }

Push-Location $root
& $tarExe --exclude=node_modules -cf $tar backend/src backend/public backend/tools backend/test backend/config/default.json backend/package.json
Pop-Location
if ($LASTEXITCODE -ne 0) { Write-Host "Packing failed."; exit 1 }

$script = @'
set -e
KEY='__PUB__'
mkdir -p ~/.ssh && touch ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys
if [ -n "$KEY" ] && ! grep -qF "$KEY" ~/.ssh/authorized_keys; then echo "$KEY" >> ~/.ssh/authorized_keys; fi
rm -rf ~/upd && mkdir ~/upd && tar xf ~/backend-update.tar -C ~/upd && rm ~/backend-update.tar
cd ~/__DIR__
for d in src public tools test; do rm -rf "$d"; cp -r ~/upd/backend/"$d" "$d"; done
cp ~/upd/backend/config/default.json config/default.json; cp ~/upd/backend/package.json package.json; rm -rf ~/upd
export PATH=$HOME/node/bin:$PATH
node --test 2>&1 | grep -E '^# (pass|fail)' || true
pm2 restart anc-backend > /dev/null && echo "restarted anc-backend"
rm -f ~/backend-update.sh
'@
$script = $script.Replace('__PUB__', $pub).Replace('__DIR__', $Dir) -replace "`r", ""
[IO.File]::WriteAllText($sh, $script, (New-Object Text.UTF8Encoding $false))

Write-Host "Uploading to $User@$Board ..."
& $scpExe $tar $sh "${User}@${Board}:"
if ($LASTEXITCODE -ne 0) { Write-Host "Upload failed (address / password?)."; exit 1 }
Write-Host "Updating and restarting ..."
& $sshExe "${User}@${Board}" "bash ~/backend-update.sh"
if ($LASTEXITCODE -ne 0) { Write-Host "Update failed - see the messages above."; exit 1 }
Write-Host "Done: open http://${Board}:8080/ and refresh."
