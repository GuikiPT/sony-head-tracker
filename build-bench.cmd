@echo off
rem Build and run the local performance benchmark (run-bench.exe). The packet
rem source is simulated, so no headset is required. Compiles the hot-path
rem sources with the same /O2 optimisation level as the release build.
setlocal

where cl >nul 2>nul && goto build

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo Could not find Visual Studio. Install "Desktop development with C++",
  echo or open a "x64 Native Tools Command Prompt for VS" and run this script there.
  exit /b 1
)
set "PATH=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer;%PATH%"
set "VSPATH="
for /f "usebackq tokens=*" %%i in (`vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%i"
if not defined VSPATH (
  echo Visual Studio with the C++ toolset was not found.
  exit /b 1
)
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul

:build
if not exist build md build
cl /nologo /std:c++latest /EHsc /permissive- /utf-8 /O2 /W4 /DUNICODE /D_UNICODE /I include /Fobuild\ ^
   src\math.cpp src\hid_descriptor.cpp src\orientation.cpp src\protocol.cpp src\output_udp.cpp ^
   bench\bench_main.cpp ^
   /Fe:run-bench.exe
if not %errorlevel%==0 (
  echo.
  echo Benchmark build failed.
  exit /b 1
)
echo.
"%~dp0run-bench.exe"
endlocal
