# One-shot dev loop for this machine: build, optionally flash, optionally watch serial,
# printing only the lines that matter (keeps the output short).
#
#   tools\dev.ps1                      build only
#   tools\dev.ps1 -Flash               build + flash COM4
#   tools\dev.ps1 -Flash -Log 60       build + flash + 60 s of serial, filtered
#   tools\dev.ps1 -Log 60              just watch serial
#   tools\dev.ps1 -Log 10 -Cmd "mode sweep","wait 2000","tap 120 170","status"
#                                      drive the board: commands go out in order, "wait N"
#                                      pauses N ms (see serialCommands() in main.cpp)
#
# Uses the arduino-cli inside the Arduino IDE with this project's own data folder
# (%USERPROFILE%\.arduino-han433), so the IDE's own ESP32 core is left alone.
param(
  [switch]$Flash,
  [int]$Log = 0,
  [string[]]$Cmd = @(),
  [string]$Port = 'COM4',
  [string]$Match = 'CMD|STATUS|SCAN|ROW|PROBE|TOUCH|CAL|SWEEP|METER|TX|Radio|saved|loaded|calibration|TIME|DET|LOG|rror|Guru|panic|rst:'
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent

if ($Flash -or $Log -eq 0) {
  $env:ACLI = 'C:/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe'
  $env:ACLI_CONFIG = "$env:USERPROFILE/.arduino-han433/arduino-cli.yaml"
  # Git Bash, not WSL's bash.exe
  $bash = Join-Path (Split-Path (Split-Path (Get-Command git).Source)) 'bin\bash.exe'
  $a = @("$repo/tools/build.sh")
  if ($Flash) { $a += 'flash', $Port }
  $t = [Diagnostics.Stopwatch]::StartNew()
  & $bash @a 2>&1 | ForEach-Object { "$_" } |
    Select-String -Pattern 'Sketch uses|rror|FAILED|Hash of data verified|^== ' | ForEach-Object Line
  if ($LASTEXITCODE) { Write-Output "BUILD/FLASH FAILED"; exit 1 }
  Write-Output ("done in {0:n0} s" -f $t.Elapsed.TotalSeconds)
}

if ($Log -gt 0) {
  $p = New-Object System.IO.Ports.SerialPort $Port, 115200
  $p.DtrEnable = $false; $p.RtsEnable = $false
  $p.Open()
  # The board has no clock: send it ours ("T<unix ms>") now, and again when it boots.
  $sendTime = { $p.Write("T$([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds())`n") }
  & $sendTime
  # Commands with their send times: "wait N" moves the next ones N ms later.
  $at = 300; $queue = @()
  foreach ($c in $Cmd) {
    if ($c -match '^wait (\d+)$') { $at += [int]$Matches[1] } else { $queue += ,@($at, $c); $at += 100 }
  }
  $t0 = Get-Date; $qi = 0
  $buf = ''; $rssi = @(); $end = (Get-Date).AddSeconds($Log)
  while ((Get-Date) -lt $end) {
    while ($qi -lt $queue.Count -and ((Get-Date) - $t0).TotalMilliseconds -ge $queue[$qi][0]) {
      $p.Write($queue[$qi][1] + "`n"); $qi++
    }
    $buf += $p.ReadExisting()
    while (($i = $buf.IndexOf("`n")) -ge 0) {
      $line = $buf.Substring(0, $i).Trim(); $buf = $buf.Substring($i + 1)
      if ($line -match '^H@N 433MHz') { & $sendTime }
      if ($line -match '^RSSI: (-?[\d.]+)') { $rssi += [double]$Matches[1] }
      elseif ($line -match $Match) { $line }
    }
    Start-Sleep -Milliseconds 100
  }
  $p.Close()
  if ($rssi.Count) {
    $s = $rssi | Measure-Object -Minimum -Maximum -Average
    "METER readings: {0}  min {1}  avg {2:n1}  max {3} dBm" -f $s.Count, $s.Minimum, $s.Average, $s.Maximum
  }
}
