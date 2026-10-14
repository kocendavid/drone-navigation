@echo off
setlocal
title PX4 - Isaac External Simulator
wsl -d Ubuntu -- bash /mnt/c/Users/finel/Documents/ChatGPT/issaac/px4/start-isaac-sitl.sh
endlocal & exit /b %ERRORLEVEL%
