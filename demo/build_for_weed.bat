@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem Build Box2D WASM with Weed post-js (importScripts weedjs_post.js) and copy
rem into sibling multithreadad-game-engine\src\box2d\
rem
rem Usage:
rem   build_for_weed.bat                      rebuild + copy (THREADS=4, LTO=full default = -flto=full)
rem   build_for_weed.bat clean                wipe default + variant weed build dirs
rem   build_for_weed.bat copy                 copy existing root artifacts only
rem   build_for_weed.bat 4 1                  THREADS=4, -flto (thin/default LTO)
rem   build_for_weed.bat 2 0                  THREADS=2, no LTO
rem
rem Override destination:
rem   set WEED_BOX2D_DIR=D:\path\to\multithreadad-game-engine\src\box2d

set "ROOT=%~dp0"
set "ROOT=%ROOT:~0,-1%"

if not defined WEED_BOX2D_DIR set "WEED_BOX2D_DIR=%ROOT%\..\multithreadad-game-engine\src\box2d"

rem Positional overrides: build_for_weed.bat [THREADS] [LTO]
rem LTO: 0 | 1 | full   (1 = -flto, full = -flto=full; default full)
rem (skipped when first arg is clean/copy/configure)
if /i not "%~1"=="clean" if /i not "%~1"=="copy" if /i not "%~1"=="configure" (
	if not "%~1"=="" set "THREADS=%~1"
	if not "%~2"=="" set "LTO=%~2"
)

if not defined THREADS set "THREADS=4"
if not defined LTO set "LTO=full"

if not "%THREADS%"=="2" if not "%THREADS%"=="4" (
	echo ERROR: THREADS must be 2 or 4 ^(got %THREADS%^)
	exit /b 1
)

if /i "%LTO%"=="full" (
	set "LTO_CMAKE=FULL"
	set "LTO_TAG=ltofull"
) else if "%LTO%"=="1" (
	set "LTO_CMAKE=ON"
	set "LTO_TAG=lto"
) else if "%LTO%"=="0" (
	set "LTO_CMAKE=OFF"
	set "LTO_TAG=nolto"
) else (
	echo ERROR: LTO must be 0, 1, or full ^(got %LTO%^)
	exit /b 1
)

set "BUILD_DIR=%ROOT%\build_wasm_weed_t%THREADS%_%LTO_TAG%"

rem Portable cmake/ninja from tools\ (if present)
for /d %%D in ("%ROOT%\tools\cmake\*") do set "CMAKE_BIN=%%D\bin"
if defined CMAKE_BIN set "PATH=%CMAKE_BIN%;%PATH%"
if exist "%ROOT%\tools\ninja" set "PATH=%ROOT%\tools\ninja;%PATH%"

rem Emscripten
if exist "%ROOT%\emsdk\emsdk_env.bat" (
	call "%ROOT%\emsdk\emsdk_env.bat" >nul 2>&1
) else (
	set "PATH=%ROOT%\emsdk;%ROOT%\emsdk\upstream\emscripten;%PATH%"
	set "EMSDK=%ROOT%\emsdk"
	if exist "%ROOT%\emsdk\python\3.13.3_64bit\python.exe" set "EMSDK_PYTHON=%ROOT%\emsdk\python\3.13.3_64bit\python.exe"
	if exist "%ROOT%\emsdk\node\22.16.0_64bit\bin\node.exe" set "EMSDK_NODE=%ROOT%\emsdk\node\22.16.0_64bit\bin\node.exe"
)

where emcc >nul 2>&1
if errorlevel 1 (
	echo ERROR: emcc not found.
	echo Install emsdk in "%ROOT%\emsdk" or activate it globally.
	exit /b 1
)

if /i "%~1"=="clean" (
	if exist "%ROOT%\build_wasm_weed" rmdir /s /q "%ROOT%\build_wasm_weed"
	for /d %%D in ("%ROOT%\build_wasm_weed_t*") do rmdir /s /q "%%D"
	echo Cleaned build_wasm_weed and build_wasm_weed_t*
	exit /b 0
)

if /i "%~1"=="copy" goto copy_only

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
pushd "%BUILD_DIR%"

if /i "%~1"=="configure" goto configure
if not exist CMakeCache.txt goto configure
rem Force reconfigure if knobs mismatch
findstr /C:"BOX2D_WEED_INTEGRATION:BOOL=ON" CMakeCache.txt >nul 2>&1
if errorlevel 1 goto configure
findstr /C:"BOX2D_PTHREAD_POOL_SIZE:STRING=%THREADS%" CMakeCache.txt >nul 2>&1
if errorlevel 1 goto configure
findstr /C:"BOX2D_LTO_MODE:STRING=%LTO_CMAKE%" CMakeCache.txt >nul 2>&1
if errorlevel 1 goto configure
goto build

:configure
echo Configuring Weed WASM ^(THREADS=%THREADS% LTO_MODE=%LTO_CMAKE% dir=%BUILD_DIR%^)...
emcmake cmake ..\wasm -DCMAKE_BUILD_TYPE=Release -DBOX2D_WEED_INTEGRATION=ON -DBOX2D_PTHREAD_POOL_SIZE=%THREADS% -DBOX2D_LTO_MODE=%LTO_CMAKE%
if errorlevel 1 goto fail
if /i "%~1"=="configure" (
	popd
	exit /b 0
)

:build
echo Building Weed WASM ^(THREADS=%THREADS% LTO_MODE=%LTO_CMAKE%^)...
cmake --build . --config Release
if errorlevel 1 goto fail
popd

:copy_only
if not exist "%WEED_BOX2D_DIR%" (
	echo ERROR: Weed box2d dir not found:
	echo   %WEED_BOX2D_DIR%
	echo Set WEED_BOX2D_DIR to multithreadad-game-engine\src\box2d
	exit /b 1
)

if not exist "%ROOT%\box2d_wasm.js" (
	echo ERROR: missing %ROOT%\box2d_wasm.js — build first
	exit /b 1
)
if not exist "%ROOT%\box2d_wasm.wasm" (
	echo ERROR: missing %ROOT%\box2d_wasm.wasm — build first
	exit /b 1
)

rem Extra Binaryen size pass (emcc -O3 already ran wasm-opt; this can still trim a bit).
rem Must enable threads + SIMD to match the Weed build.
set "WASM_OPT="
if exist "%ROOT%\emsdk\upstream\bin\wasm-opt.exe" set "WASM_OPT=%ROOT%\emsdk\upstream\bin\wasm-opt.exe"
if not defined WASM_OPT (
	where wasm-opt >nul 2>&1
	if not errorlevel 1 for /f "delims=" %%P in ('where wasm-opt') do (
		if not defined WASM_OPT set "WASM_OPT=%%P"
	)
)
if defined WASM_OPT (
	echo Running wasm-opt size pass...
	"%WASM_OPT%" -O --enable-threads --enable-simd --enable-bulk-memory --enable-mutable-globals --enable-nontrapping-float-to-int --enable-sign-ext "%ROOT%\box2d_wasm.wasm" -o "%ROOT%\box2d_wasm.opt.wasm"
	if errorlevel 1 (
		echo WARN: wasm-opt failed — keeping unoptimized box2d_wasm.wasm
		if exist "%ROOT%\box2d_wasm.opt.wasm" del /q "%ROOT%\box2d_wasm.opt.wasm"
	) else (
		move /Y "%ROOT%\box2d_wasm.opt.wasm" "%ROOT%\box2d_wasm.wasm" >nul
		echo wasm-opt done.
	)
) else (
	echo WARN: wasm-opt not found — skipping post-link size pass
)

rem Sanity: must load weedjs_post, not lab game-constants
findstr /C:"weedjs_post.js" "%ROOT%\box2d_wasm.js" >nul 2>&1
if errorlevel 1 (
	echo ERROR: box2d_wasm.js does not reference weedjs_post.js
	echo Rebuild with build_for_weed.bat ^(not plain build_wasm.bat^)
	exit /b 1
)
findstr /C:"game-constants.js" "%ROOT%\box2d_wasm.js" >nul 2>&1
if not errorlevel 1 (
	echo ERROR: box2d_wasm.js still references lab game-constants.js
	echo Run: build_for_weed.bat clean ^& build_for_weed.bat
	exit /b 1
)

echo Copying to Weed:
echo   %WEED_BOX2D_DIR%
copy /Y "%ROOT%\box2d_wasm.js" "%WEED_BOX2D_DIR%\box2d_wasm.js" >nul
if errorlevel 1 goto copy_fail
copy /Y "%ROOT%\box2d_wasm.wasm" "%WEED_BOX2D_DIR%\box2d_wasm.wasm" >nul
if errorlevel 1 goto copy_fail
if exist "%ROOT%\box2d_wasm.worker.js" (
	copy /Y "%ROOT%\box2d_wasm.worker.js" "%WEED_BOX2D_DIR%\box2d_wasm.worker.js" >nul
)

echo.
echo Weed Box2D ready ^(THREADS=%THREADS% LTO=%LTO%^):
echo   %WEED_BOX2D_DIR%\box2d_wasm.js
echo   %WEED_BOX2D_DIR%\box2d_wasm.wasm
echo.
echo Hard-refresh the browser ^(or bust cache^). Lab demo: use build_wasm.bat instead.
exit /b 0

:copy_fail
echo ERROR: copy failed
exit /b 1

:fail
popd
exit /b 1
