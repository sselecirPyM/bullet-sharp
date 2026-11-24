@echo off
rem Build C++ benchmark against bullet3, Release, single-threaded.
rem 1) Build Bullet static libs with CMake (official default flags, /fp:precise for parity with .NET)
rem 2) Compile benchmark.cpp and link against them

setlocal EnableDelayedExpansion
cd /d "%~dp0"

if not "%~1"=="" set "BULLET3=%~1"
if not defined BULLET3 set "BULLET3=%~dp0..\..\..\bullet3"
if not exist "%BULLET3%\src" (
  echo Error: bullet3 source not found at "%BULLET3%". Set BULLET3 env var or pass its path as the first argument.
  exit /b 1
)
set "BUILD=bullet3-build"

if not exist "%BUILD%\lib\Release\BulletDynamics.lib" (
  cmake -S "%BULLET3%" -B "%BUILD%" -A x64 ^
    -DBUILD_BULLET3=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_CPU_DEMOS=OFF ^
    -DBUILD_OPENGL3_DEMOS=OFF -DBUILD_EXTRAS=OFF -DBUILD_UNIT_TESTS=OFF ^
    -DBUILD_ENET=OFF -DBUILD_CLSOCKET=OFF -DUSE_GRAPHICAL_BENCHMARK=OFF ^
    -DBUILD_PYBULLET=OFF -DUSE_MSVC_FAST_FLOATINGPOINT=OFF || exit /b 1
  cmake --build "%BUILD%" --config Release --target BulletDynamics -- /m /v:m /nologo || exit /b 1
  cmake --build "%BUILD%" --config Release --target BulletCollision -- /m /v:m /nologo || exit /b 1
  cmake --build "%BUILD%" --config Release --target LinearMath -- /m /v:m /nologo || exit /b 1
)

where cl >nul 2>&1
if errorlevel 1 (
  set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
  if exist "!VSWHERE!" (
    for /f "usebackq tokens=*" %%i in (`"!VSWHERE!" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%i"
  )
  if not defined VSINSTALL (
    echo Error: Visual Studio C++ tools not found. Run from an x64 Native Tools Command Prompt.
    exit /b 1
  )
  call "!VSINSTALL!\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
)
cl /nologo /utf-8 /O2 /Ob2 /MT /EHsc /std:c++17 /DNDEBUG /I"%BULLET3%\src" benchmark.cpp ^
  /link /LIBPATH:"%BUILD%\lib\Release" BulletDynamics.lib BulletCollision.lib LinearMath.lib /OUT:benchmark.exe
