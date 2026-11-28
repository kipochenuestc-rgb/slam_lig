@echo off
cd /d "%~dp0web_viewer"
for %%A in (%*) do if /i "%%~A"=="--web" goto console
start "" pythonw.exe desktop.py %*
exit /b %errorlevel%
:console
python desktop.py %*
if errorlevel 1 pause
