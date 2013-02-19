@echo OFF
cls

if "%1" == ""  goto execute
if "%1" == "clean" goto clean_up
if not "%1%" == "?"  goto execute
@echo valid targets are:
@echo.
@echo   meeblip-se-v2_original                    build original firmware for meeblip-se hardware    (for reference only)
@echo   meeblip_micro_original                    build original firmware for meeblip micro hardware (for reference only)
@echo   meeblip-se-v2_optimized                   build optimized firmware for meeblip-se hardware
@echo   meeblip_micro_optimized                   build optimized firmware for meeblip micro hardware
@echo   meeblip-se-v2_aweMeeblipSE-v2             build aweMeeblipSE-v2 firmware for meeblip-se hardware
@echo   meeblip_micro_aweMeeblipSE-v2             build aweMeeblipSE-v2 firmware for meeblip micro hardware
@echo.
@echo   meeblip-se-v2_aweMeeblipSE-v3             build aweMeeblipSE-v3 firmware for meeblip hardware
@echo   meeblip_micro_aweMeeblipSE-v3             build aweMeeblipSE-v3 firmware for meeblip_micro hardware
goto exit

:execute
set CURDIR=%CD%

set ASM.EXE="C:\Program Files\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe"
if exist %ASM.EXE% goto do_it
set ASM.EXE="C:\Programme\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe"
if exist %ASM.EXE% goto do_it
goto exit

:do_it
: ---------------------------------------------------------------------------
:@echo on

if "%1" == "meeblip-se-v2_original"                set SRC=aweMeeblipSE-v2
if "%1" == "meeblip_micro_original"                set SRC=aweMeeblipSE-v2
if "%1" == "meeblip-se-v2_optimized"               set SRC=aweMeeblipSE-v2
if "%1" == "meeblip_micro_optimized"               set SRC=aweMeeblipSE-v2
if "%1" == "meeblip-se-v2_aweMeeblipSE-v2"         set SRC=aweMeeblipSE-v2
if "%1" == "meeblip_micro_aweMeeblipSE-v2"         set SRC=aweMeeblipSE-v2

if "%1" == "meeblip-se-v2_aweMeeblipSE-v3"         set SRC=aweMeeblipSE-v3
if "%1" == "meeblip_micro_aweMeeblipSE-v3"         set SRC=aweMeeblipSE-v3

if "%1" == ""  goto build_all
if "%1" == "%1" goto %1

:build_all

set SRC=aweMeeblipSE-v2
@echo.
@echo -------------------------------------------------------------------------------
@echo  build from %SRC%
@echo -------------------------------------------------------------------------------
@echo.

:meeblip-se-v2_original
@echo.
@echo build original meeblip-se-v2
@echo ----------------------------
set TARGET=meeblip-se-v2_original
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=1 -D _MEEBLIP_OPIMIZED_CODE=0 -D _MEEBLIP_V1_31_HW=1   -S "labels.tmp" -f I -W+ie -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:meeblip_micro_original
@echo.
@echo build original meeblip micro
@echo ----------------------------
set TARGET=meeblip_micro_original
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=1 -D _MEEBLIP_OPIMIZED_CODE=0 -D _MEEBLIP_MICRO_V2_04_HW=1   -S "labels.tmp" -f I -W+ie -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:meeblip-se-v2_optimized
@echo.
@echo build optimized meeblip-se-v2
@echo -----------------------------
set TARGET=meeblip-se-v2_optimized
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE% -D _MEEBLIP_ORIGINAL_CODE=1 -D _MEEBLIP_OPIMIZED_CODE=1 -D _MEEBLIP_V1_31_HW=1   -S "labels.tmp" -f I -W+ie -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:meeblip-micro_optimized
@echo.
@echo build optimized meeblip-micro
@echo -----------------------------
set TARGET=meeblip-micro_optimized
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE% -D _MEEBLIP_ORIGINAL_CODE=1 -D _MEEBLIP_OPIMIZED_CODE=1 -D _MEEBLIP_MICRO_V2_04_HW=1   -S "labels.tmp" -f I -W+ie -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit


:meeblip-se-v2_aweMeeblipSE-v2
@echo.
@echo build aweMeeblipSE-v2 for meeblip-se hardware
@echo ---------------------------------------------
set TARGET=meeblip-se-v2_aweMeeblipSE-v2
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=0  -D _MEEBLIP_V1_31_HW=1 -S "labels.tmp" -f I -W+ie -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:meeblip_micro_aweMeeblipSE-v2
@echo.
@echo build aweMeeblipSE-v2 for meeblip micro hardware
@echo ------------------------------------------------
set TARGET=meeblip_micro_aweMeeblipSE-v2
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=0  -D _MEEBLIP_MICRO_V2_04_HW=1 -S "labels.tmp" -f I -W+ie -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit


set SRC=aweMeeblipSE-v3
@echo.
@echo -------------------------------------------------------------------------------
@echo  build from %SRC%
@echo -------------------------------------------------------------------------------
@echo.

:meeblip-se-v2_aweMeeblipSE-v3
@echo.
@echo build aweMeeblipSE-v3 for meeblip hardware
@echo ---------------------------------------------
set TARGET=meeblip-se-v2_aweMeeblipSE-v3
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_V1_31_HW=1  -S "labels.tmp" -f I -W+ie -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:meeblip_micro_aweMeeblipSE-v3
@echo.
@echo build aweMeeblipSE-v3 for meeblip_micro hardware
@echo ------------------------------------------------
set TARGET=meeblip_micro_aweMeeblipSE-v3
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%   -D _MEEBLIP_MICRO_V2_04_HW=1  -S "labels.tmp" -f I -W+ie -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:clean_up
del /s *.obj
del /s labels.tmp
:del /s *.lst
:del /s *.map

:exit
