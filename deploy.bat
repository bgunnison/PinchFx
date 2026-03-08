@echo off
setlocal

set ROOT=%~dp0

set SRC_DEBUG_BIN=%ROOT%build\VST3\Debug\PinchFX.vst3
set SRC_DEBUG_RES=%ROOT%build\VST3\Debug\DebugPinchFX.vst3
set SRC_RELEASE=%ROOT%build\VST3\Release\PinchFX.vst3
set DST_DIR=C:\ProgramData\vstplugins
set DST_DEBUG=%DST_DIR%\DebugPinchFX.vst3
set DST_RELEASE=%DST_DIR%\PinchFX.vst3

if not exist "%SRC_DEBUG_BIN%" (
  echo Missing Debug binary bundle at: %SRC_DEBUG_BIN%
  echo Run build.bat first.
  goto :done
)

if not exist "%SRC_DEBUG_RES%" (
  echo Missing Debug resource bundle at: %SRC_DEBUG_RES%
  echo Run build.bat first.
  goto :done
)

if not exist "%SRC_RELEASE%" (
  echo Missing Release build at: %SRC_RELEASE%
  echo Run build.bat first.
  goto :done
)

if not exist "%DST_DIR%" mkdir "%DST_DIR%"

if exist "%DST_DEBUG%" (
  rmdir /S /Q "%DST_DEBUG%"
)
if exist "%DST_DEBUG%" (
  echo Debug deploy failed: could not clear %DST_DEBUG%
  echo Close Ableton Live and try again.
  goto :done
)

if exist "%DST_RELEASE%" (
  rmdir /S /Q "%DST_RELEASE%"
)
if exist "%DST_RELEASE%" (
  echo Release deploy failed: could not clear %DST_RELEASE%
  echo Close Ableton Live and try again.
  goto :done
)

robocopy "%SRC_DEBUG_BIN%" "%DST_DEBUG%" /E /NFL /NDL /NJH /NJS /NC /NS
set RC=%ERRORLEVEL%
if %RC% GEQ 8 goto :done

robocopy "%SRC_DEBUG_RES%\Contents\Resources" "%DST_DEBUG%\Contents\Resources" /E /NFL /NDL /NJH /NJS /NC /NS
set RC=%ERRORLEVEL%
if %RC% GEQ 8 goto :done

robocopy "%SRC_RELEASE%" "%DST_RELEASE%" /E /NFL /NDL /NJH /NJS /NC /NS
set RC=%ERRORLEVEL%
if %RC% GEQ 8 goto :done

if not exist "%DST_DEBUG%\Contents\x86_64-win\DebugPinchFX.vst3" (
  echo Debug deploy failed: missing %DST_DEBUG%\Contents\x86_64-win\DebugPinchFX.vst3
  goto :done
)

if not exist "%DST_DEBUG%\Contents\Resources\pinchfx.uidesc" (
  echo Debug deploy failed: missing %DST_DEBUG%\Contents\Resources\pinchfx.uidesc
  goto :done
)

if not exist "%DST_RELEASE%\Contents\x86_64-win\PinchFX.vst3" (
  echo Release deploy failed: missing %DST_RELEASE%\Contents\x86_64-win\PinchFX.vst3
  goto :done
)

if not exist "%DST_RELEASE%\Contents\Resources\pinchfx.uidesc" (
  echo Release deploy failed: missing %DST_RELEASE%\Contents\Resources\pinchfx.uidesc
  goto :done
)

echo Deploy OK: %DST_DEBUG% and %DST_RELEASE%

:done
pause
endlocal
