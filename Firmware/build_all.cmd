@echo OFF
cls

if "%1" == ""  goto execute
if not "%1%" == "TARGETS"  goto execute
@echo valid targets are:
@echo.
@echo   meeblip_v2.04_original
@echo   meeblip_micro_v2.04_original
@echo   meeblip_v2.04_optimized
@echo   meeblip_v2.04_original_for_avrSynth
@echo   meeblip_v2.04_optimized_for_avrSynth
@echo   aweMeeblipSE_V2.04_for_meeblip
@echo   aweMeeblipSE_V2.04
@echo.
@echo   aweMeeblipSE_V3.01
@echo   aweMeeblipSE_V3.01_for_meeblip
goto exit

:execute
@echo computer %COMPUTERNAME%
set CURDIR=%CD%

if not %COMPUTERNAME%==OEKAN27B goto next_1
set ASM.EXE="C:\Program Files\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe"
goto do_it

:next_1
if not %COMPUTERNAME%==AWE-ATHLON64 goto next_2
set ASM.EXE="C:\Programme\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe"
goto do_it:

:next_2
if not %COMPUTERNAME%==AXEL-NETBOOK goto next_3
set ASM.EXE="C:\Program Files\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe"
goto do_it

:next_3
if not %COMPUTERNAME%==AWE-LAPTOP-Z61M goto next_4
goto do_it

:next_4
@echo Unknown computer %COMPUTERNAME%
goto exit

:do_it
: ---------------------------------------------------------------------------
:@echo on

if "%1" == "aweMeeblipSE_V3"  set SRC=aweMeeblipSE_V3.01
if "%1" == "aweMeeblipSE_V3_for_meeblip"  set SRC=aweMeeblipSE_V3.01

if "%1" == ""  goto build_all
if "%1" == "%TARGET%" goto %TARGET%

:build_all

set SRC=aweMeeblipSE_V2.04
@echo.
@echo -------------------------------------------------------------------------------
@echo  build from %SRC%
@echo -------------------------------------------------------------------------------
@echo.

:meeblip_v2.04_original
@echo.
@echo build original meeblip v2.04
@echo ----------------------------
set TARGET=meeblip_v2.04_original
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=1 -D _MEEBLIP_OPIMIZED_CODE=0 -D _MEEBLIP_V1_31_HW=1   -S "labels.tmp" -f I -W+ie -C V2E -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:meeblip_micro_v2.04_original
@echo.
@echo build original meeblip micro v2.04
@echo ----------------------------------
set TARGET=meeblip_micro_v2.04_original
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=1 -D _MEEBLIP_MICRO_V2_04_HW=1   -S "labels.tmp" -f I -W+ie -C V2E -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:meeblip_v2.04_optimized
@echo.
@echo build optimized meeblip v2.04
@echo -----------------------------
set TARGET=meeblip_v2.04_optimized
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE% -D _MEEBLIP_ORIGINAL_CODE=1 -D _MEEBLIP_OPIMIZED_CODE=1 -D _MEEBLIP_V1_31_HW=1   -S "labels.tmp" -f I -W+ie -C V2E -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:meeblip_v2.04_original_for_avrSynth
@echo.
@echo build original meeblip v2.04 for my hardware without uart support
@echo -----------------------------------------------------------------
set TARGET=meeblip_v2.04_original_for_avrSynth
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=1 -D _USE_UART=0   -S "labels.tmp" -f I -W+ie -C V2E -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:meeblip_v2.04_optimized_for_avrSynth
@echo.
@echo build optimized meeblip v2.04 for my hardware without uart support
@echo ------------------------------------------------------------------
set TARGET=meeblip_v2.04_optimized_for_avrSynth
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=1 -D _MEEBLIP_OPIMIZED_CODE=1 -D _USE_UART=0   -S "labels.tmp" -f I -W+ie -C V2E -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:aweMeeblipSE_V2.04_for_meeblip
@echo.
@echo build aweMeeblipSE_V2.04 for meeblip hardware
@echo ---------------------------------------------
set TARGET=aweMeeblipSE_V2.04_for_meeblip
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=0  -D _MEEBLIP_V1_31_HW=1 -S "labels.tmp" -f I -W+ie -C V2E -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:aweMeeblipSE_V2.04
@echo.
@echo build aweMeeblipSE_V2.04 for my hardware (uart support)
@echo -------------------------------------------------------
set TARGET=aweMeeblipSE_V2.04
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_ORIGINAL_CODE=0   -S "labels.tmp" -f I -W+ie -C V2E -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

set SRC=aweMeeblipSE_V3.01
@echo.
@echo -------------------------------------------------------------------------------
@echo  build from %SRC%
@echo -------------------------------------------------------------------------------
@echo.

:aweMeeblipSE_V3.01
@echo.
@echo build aweMeeblipSE_V3.01 (uart support)
@echo --------------------------------------
set TARGET=aweMeeblipSE_V3.01
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_V1_31_HW=0 -D _USE_UART=1  -S "labels.tmp" -f I -W+ie -C V2E -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:aweMeeblipSE_V3.01_for_meeblip
@echo.
@echo build aweMeeblipSE_V3.01 for meeblip hardware
@echo ---------------------------------------------
set TARGET=aweMeeblipSE_V3.01_for_meeblip
@echo TARGET = %TARGET%
set BUILD_DIR=_build\%TARGET%
if not exist %BUILD_DIR% mkdir %BUILD_DIR%
pushd %BUILD_DIR%
%ASM.EXE%  -D _MEEBLIP_V1_31_HW=1  -S "labels.tmp" -f I -W+ie -C V2E -o "%TARGET%.hex" -d "%TARGET%.obj" -e "%TARGET%.eep" -m "%TARGET%.map" -l "%TARGET%.lst" "..\..\%SRC%.asm"
popd
if "%1" == "%TARGET%" goto exit

:exit
