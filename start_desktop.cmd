@echo off
cd /d "%~dp0web_viewer"
python desktop.py %*
if errorlevel 1 pause
