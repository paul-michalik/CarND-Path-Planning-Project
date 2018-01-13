@echo off

set "VcPkgPlatform=x64"
set "VcPkgToolset=v141"
set "VcPkgBuildType=Release"

if NOT "%~1"=="" set "VcPkgPlatform=%~1"
if NOT "%~2"=="" set "VcPkgToolset=%~2"
if NOT "%~3"=="" set "VcPkgBuildType=%~3" 

rem ----------------------------------
rem Locate vcpkg using environment variables
rem ----------------------------------
set "VcPkgDir=%~d0\Software\vcpkg\vcpkg"
set "VcpkgTriplet=%VcPkgPlatform%-windows"
if defined VCPKG_ROOT_DIR if /i not "%VCPKG_ROOT_DIR%"=="" set "VcPkgDir=%VCPKG_ROOT_DIR%"
if defined VCPKG_DEFAULT_TRIPLET if /i not "%VCPKG_DEFAULT_TRIPLET%"=="" set "VcpkgTriplet=%VCPKG_DEFAULT_TRIPLET%"

rem ----------------------------------
rem Try to look for vcpkg at default locations
rem ----------------------------------
if not exist "%VcPkgDir%" set "VcPkgDir=%~d0\Software\vcpkg\vcpkg"
if not exist "%VcPkgDir%" set "VcPkgDir=%~d0\.vcpkg\vcpkg"
if not exist "%VcPkgDir%" set "VcPkgDir=C:\Software\vcpkg\vcpkg"
if not exist "%VcPkgDir%" set "VcPkgDir=C:\.vcpkg\vcpkg"
if not exist "%VcPkgDir%" set "VcPkgDir=%USERPROFILE%\.vcpkg\vcpkg"
if not exist "%VcPkgDir%" (
    echo vcpkg not found, installing at %VcPkgDir%...
    git clone --recursive https://github.com/Microsoft/vcpkg.git "%VcPkgDir%"
    call "%VcPkgDir%\bootstrap-vcpkg.bat"
) else (
    echo vcpkg found at %VcPkgDir%...
    pushd "%VcPkgDir%"
    git pull --all --prune
    call "%VcPkgDir%\bootstrap-vcpkg.bat"
    popd 
)

if not exist "%VcPkgDir%" echo vcpkg path is not set correctly, bailing out & exit /b 1
set "VcPkgPath=%VcPkgDir%\vcpkg.exe"
if not exist "%VcPkgPath%" echo vcpkg path is not set correctly, bailing out & exit /b 1

echo. & echo Bootstrapping dependencies for triplet: %VcPkgTriplet% & echo.

rem ==============================
rem Update and Install packages.
rem ==============================
call "%VcPkgPath%" update
call "%VcPkgPath%" remove --outdated --recurse
call "%VcPkgPath%" install uwebsockets --triplet %VcPkgTriplet%
call "%VcPkgPath%" integrate project --triplet %VcPkgTriplet%

set "VcPkgTripletDir=%VcPkgDir%\installed\%VcPkgTriplet%"

if not exist "%VcPkgTripletDir%" echo %VcPkgTripletDir% does not exist, bailing out & exit /b 1

set "CMAKE_PREFIX_PATH=%VcPkgTripletDir%;%CMAKE_PREFIX_PATH%"

echo. & echo Bootstrapping successful for triplet: %VcPkgTriplet% & echo CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH% & echo.