@echo off
chcp 65001 >nul
powershell.exe -NoProfile -Command "$node = (Get-Command node.exe -ErrorAction SilentlyContinue).Source; if (-not $node) { foreach ($pkg in (Get-AppxPackage -Name '*CodexPrimaryRuntime*' | Sort-Object Version -Descending)) { $candidate = Join-Path $pkg.InstallLocation 'dependencies\node\bin\node.exe'; if (Test-Path -LiteralPath $candidate) { $node = $candidate; break } } }; if (-not $node) { Write-Host 'Node.js not found. Install Node.js or Codex first.'; exit 1 }; & $node '%~dp0server.mjs' --open %*; exit $LASTEXITCODE"
if "%~1"=="--check" exit /b %errorlevel%
pause
