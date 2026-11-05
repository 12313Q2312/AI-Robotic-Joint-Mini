@echo off
cd /d "%~dp0"
python launch_demo.py
if errorlevel 1 pause
