@echo off
setlocal

set ROOT=%~dp0
set SRC_RELEASE=%ROOT%build\VST3\Release\PinchFX.vst3
set ZIP_RELEASE=%ROOT%PinchFX.vst3.zip

if not exist "%SRC_RELEASE%" (
  echo Missing Release build at: %SRC_RELEASE%
  echo Run build.bat first.
  goto :done
)

if not exist "%SRC_RELEASE%\Contents\x86_64-win\PinchFX.vst3" (
  echo Release package incomplete: missing %SRC_RELEASE%\Contents\x86_64-win\PinchFX.vst3
  goto :done
)

if not exist "%SRC_RELEASE%\Contents\Resources\pinchfx.uidesc" (
  echo Release package incomplete: missing %SRC_RELEASE%\Contents\Resources\pinchfx.uidesc
  goto :done
)

if not exist "%SRC_RELEASE%\Contents\Resources\logo_strip.png" (
  echo Release package incomplete: missing %SRC_RELEASE%\Contents\Resources\logo_strip.png
  goto :done
)

if exist "%ZIP_RELEASE%" del /F /Q "%ZIP_RELEASE%"
if exist "%ZIP_RELEASE%" (
  echo Release zip failed: could not clear %ZIP_RELEASE%
  goto :done
)

powershell -NoProfile -Command "Compress-Archive -Path $env:SRC_RELEASE -DestinationPath $env:ZIP_RELEASE -Force"
if errorlevel 1 (
  echo Release zip failed: %ZIP_RELEASE%
  goto :done
)

if not exist "%ZIP_RELEASE%" (
  echo Release zip failed: missing %ZIP_RELEASE%
  goto :done
)

echo Release OK: %ZIP_RELEASE%

:done
pause
endlocal
