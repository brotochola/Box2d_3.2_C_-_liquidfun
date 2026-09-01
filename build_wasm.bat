@echo off
setlocal EnableExtensions

set "ROOT=%~dp0"
set "ROOT=%ROOT:~0,-1%"

rem Portable cmake/ninja from local tools\ only
if exist "%ROOT%\tools\cmake" (
    for /d %%D in ("%ROOT%\tools\cmake\*") do set "CMAKE_BIN=%%D\bin"
)
if defined CMAKE_BIN set "PATH=%CMAKE_BIN%;%PATH%"
if exist "%ROOT%\tools\ninja" set "PATH=%ROOT%\tools\ninja;%PATH%"

rem Emscripten: local emsdk\ then %EMSDK%
if exist "%ROOT%\emsdk\emsdk_env.bat" (
    call "%ROOT%\emsdk\emsdk_env.bat" >nul 2>&1
) else if defined EMSDK (
    if exist "%EMSDK%\emsdk_env.bat" call "%EMSDK%\emsdk_env.bat" >nul 2>&1
)

where emcc >nul 2>&1
if errorlevel 1 (
    echo ERROR: emcc not found.
    echo Install emsdk in "%ROOT%\emsdk" or set %%EMSDK%%.
    exit /b 1
)

if /i "%~1"=="clean" (
    if exist "%ROOT%\build_wasm" rmdir /s /q "%ROOT%\build_wasm"
    echo Cleaned build_wasm
    exit /b 0
)

if not exist "%ROOT%\build_wasm" mkdir "%ROOT%\build_wasm"
pushd "%ROOT%\build_wasm"

if /i "%~1"=="configure" goto configure
if not exist CMakeCache.txt goto configure
goto build

:configure
echo Configuring CMake for WASM build...
emcmake cmake ..\wasm -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 goto fail
if /i "%~1"=="configure" goto ok

:build
echo Building WASM binary...
cmake --build . --config Release
if errorlevel 1 goto fail

:ok
popd
echo.
echo ========================================================
echo Build complete:
echo   %ROOT%\demo\box2d_wasm.js
echo   %ROOT%\demo\box2d_wasm.wasm
echo.
echo Test locally:  node demo/node_server.js
echo Open browser:  http://localhost:8000/
echo ========================================================
exit /b 0

:fail
popd
echo ERROR: WASM build failed.
exit /b 1
