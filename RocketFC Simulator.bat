@echo off
REM ===========================================================================
REM  RocketFC simulator + 3D flight view -- double-click launcher.
REM
REM  The simulator itself lives in tools\sim and has to stay there:
REM  bridge.cpp includes "..\..\src\core\flight_core.h", the Makefile depends
REM  on ..\..\src\config.h, and core.py locates the repo root by walking two
REM  directories up. This file just starts it so the path never has to be
REM  typed.
REM
REM  Works from either place, so the two copies stay byte-identical:
REM    - the repo root  (RocketFC\)            -> tools\sim
REM    - one level above (the project folder)  -> RocketFC\tools\sim
REM ===========================================================================
setlocal

set "SIMDIR="
if exist "%~dp0tools\sim\dashboard.py" set "SIMDIR=%~dp0tools\sim"
if exist "%~dp0RocketFC\tools\sim\dashboard.py" set "SIMDIR=%~dp0RocketFC\tools\sim"

if not defined SIMDIR (
    echo.
    echo Could not find the simulator next to this launcher.
    echo Expected one of:
    echo     %~dp0tools\sim\dashboard.py
    echo     %~dp0RocketFC\tools\sim\dashboard.py
    echo.
    echo Keep this file either in the RocketFC folder or directly above it.
    echo.
    pause
    exit /b 1
)

cd /d "%SIMDIR%"

REM Prefer the py launcher. Plain "python" can resolve to the Microsoft Store
REM stub, which has no matplotlib and fails with a confusing error.
set PY=python
where py >nul 2>&1
if not errorlevel 1 set PY=py

echo Starting the RocketFC simulator...
echo (first run compiles rocketfc_core.dll, ~1 s, then it is cached)
echo.
%PY% dashboard.py
if errorlevel 1 (
    echo.
    echo The simulator exited with an error - scroll up for details.
    echo.
    echo If it mentions a missing package:
    echo     %PY% -m pip install matplotlib pygame PyOpenGL
    echo.
    pause
    exit /b 1
)

exit /b 0
