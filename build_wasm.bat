@echo off
setlocal EnableExtensions

set "ROOT=%~dp0"
set "ROOT=%ROOT:~0,-1%"

rem 1. Portable cmake/ninja from local tools\ or sibling tools\
if exist "%ROOT%\tools\cmake" (
    for /d %%D in ("%ROOT%\tools\cmake\*") do set "CMAKE_BIN=%%D\bin"
) else if exist "%ROOT%\..\box2d_3.0_wasm_sab\tools\cmake" (
    for /d %%D in ("%ROOT%\..\box2d_3.0_wasm_sab\tools\cmake\*") do set "CMAKE_BIN=%%D\bin"
)

if defined CMAKE_BIN set "PATH=%CMAKE_BIN%;%PATH%"

if exist "%ROOT%\tools\ninja" (
    set "PATH=%ROOT%\tools\ninja;%PATH%"
) else if exist "%ROOT%\..\box2d_3.0_wasm_sab\tools\ninja" (
    set "PATH=%ROOT%\..\box2d_3.0_wasm_sab\tools\ninja;%PATH%"
)

rem 2. Emscripten SDK discovery (local -> sibling -> %EMSDK% -> PATH)
if exist "%ROOT%\emsdk\emsdk_env.bat" (
    call "%ROOT%\emsdk\emsdk_env.bat" >nul 2>&1
) else if exist "%ROOT%\..\box2d_3.0_wasm_sab\emsdk\emsdk_env.bat" (
    call "%ROOT%\..\box2d_3.0_wasm_sab\emsdk\emsdk_env.bat" >nul 2>&1
) else if defined EMSDK (
    if exist "%EMSDK%\emsdk_env.bat" call "%EMSDK%\emsdk_env.bat" >nul 2>&1
) else (
    rem Fallback manual path injection if emsdk folder structure exists without activation script
    if exist "%ROOT%\..\box2d_3.0_wasm_sab\emsdk" (
        set "PATH=%ROOT%\..\box2d_3.0_wasm_sab\emsdk;%ROOT%\..\box2d_3.0_wasm_sab\emsdk\upstream\emscripten;%PATH%"
        set "EMSDK=%ROOT%\..\box2d_3.0_wasm_sab\emsdk"
    )
)

where emcc >nul 2>&1
if errorlevel 1 (
    echo ERROR: emcc not found.
    echo Install emsdk in "%ROOT%\emsdk", configure %%EMSDK%%, or keep it in "..\box2d_3.0_wasm_sab\emsdk".
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
