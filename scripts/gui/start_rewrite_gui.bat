@echo off
setlocal
cd /d "%~dp0..\.."
if exist "D:\anaconda\pythonw.exe" (
    start "" "D:\anaconda\pythonw.exe" "%~dp0rewrite_control_gui.py" %*
    exit /b 0
)
where pythonw.exe >nul 2>nul
if errorlevel 1 (
    python "%~dp0rewrite_control_gui.py" %*
) else (
    start "" pythonw.exe "%~dp0rewrite_control_gui.py" %*
)
