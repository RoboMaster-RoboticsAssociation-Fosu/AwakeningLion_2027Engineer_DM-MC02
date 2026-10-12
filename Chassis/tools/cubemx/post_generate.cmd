@echo off
setlocal
rem CubeMX User Actions execute batch files reliably on Windows. The PowerShell
rem script resolves the repository root from this wrapper's own location.
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0post_generate.ps1"
exit /b %ERRORLEVEL%
