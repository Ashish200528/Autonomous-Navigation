# Sends this Windows laptop's webcam to the robot's backend as the robot camera (for a board
# without a USB webcam, e.g. the Arduino UNO Q). The backend waits on TCP port 5000 when it finds
# no USB webcam (camera.fallback_input); this script connects to it, so Windows Firewall needs no rule.
#   powershell -ExecutionPolicy Bypass -File tools\laptop-camera.ps1 -Robot 10.112.10.83
# Keep the window open while driving; it reconnects by itself. Needs ffmpeg on the laptop.
param(
  [string]$Robot = "10.112.10.83",  # the board's address (dashboard host)
  [string]$Camera = ""              # DirectShow camera name; empty = the first one found
)

if (-not $Camera) {
  $list = & ffmpeg -hide_banner -list_devices true -f dshow -i dummy 2>&1 | Out-String
  $m = [regex]::Match($list, '"([^"]+)" \(video\)')
  if (-not $m.Success) { Write-Host "No webcam found on this laptop."; exit 1 }
  $Camera = $m.Groups[1].Value
}
Write-Host "Sending '$Camera' to ${Robot}:5000 - Ctrl+C to stop."

while ($true) {
  # Laptop cameras run 16:9 at 30 fps: crop to 4:3, scale to the backend's 640x480, 15 fps, MJPEG.
  & ffmpeg -hide_banner -loglevel error -f dshow -rtbufsize 64M -video_size 1280x720 -framerate 30 -i "video=$Camera" `
    -vf "crop=960:720,scale=640:480,fps=15" -c:v mjpeg -q:v 5 -f mjpeg "tcp://${Robot}:5000"
  Write-Host "$(Get-Date -Format HH:mm:ss) not connected to ${Robot}:5000 (backend restarting or unreachable), retrying..."
  Start-Sleep 2
}
