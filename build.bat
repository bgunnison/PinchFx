@echo off
setlocal

set ROOT=%~dp0

if "%VST3_SDK_ROOT%"=="" (
  echo VST3_SDK_ROOT is not set. Example:
  echo   set VST3_SDK_ROOT=C:\projects\ableplugs\vst3sdk
  goto :done
)

if not exist "%ROOT%build" mkdir "%ROOT%build"

cmake -S "%ROOT%vst3" -B "%ROOT%build"
if errorlevel 1 goto :done

cmake --build "%ROOT%build" --config Debug
if errorlevel 1 goto :done

cmake --build "%ROOT%build" --config Release
if errorlevel 1 goto :done

:done
pause
endlocal
