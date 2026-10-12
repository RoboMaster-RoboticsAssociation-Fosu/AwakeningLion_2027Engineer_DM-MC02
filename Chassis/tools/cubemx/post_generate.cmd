@echo off
setlocal
rem CubeMX invokes this local entry; the common implementation lives in Shared.
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0..\..\..\Shared\tools\cubemx\post_generate.ps1" -ProjectRoot "%~dp0..\.."
exit /b %ERRORLEVEL%
