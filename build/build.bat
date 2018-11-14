@echo off
SETLOCAL EnableDelayedExpansion
REM IoT.Keystone firmware build script for Windows
REM Copyright (c) 2018 This. Is. IoT. 
REM BSD licensed.

REM Usage:
REM   %1 - Contiki TARGET - directory under arch/platform  e.g. simplelink
REM   %2 - Contiki BOARD - directory under the TARGET e.g. launchpad\cc1352r1 (note the backslash)
REM   %3 - path to application folder containing makefile to build.
REM        e.g. examples/hello-world (note the forward slash here)
REM   %4 - optional argument to append e.g. clean
REM   %5 - optional argument to append

REM We need to use Git's bash command line interpreter
REM to handle some advanced makefile features that TI has
REM added to their recent simplelink platform addition.
REM Namely the 'find' command.
REM Otherwise gmake could be run from the ordinary Windows
REM command-line.

REM gmake.exe and srec_cat.exe must be available to Git bash.

set BUILD_DIR=%cd%

REM Set the location of your Git installation
set BASH_PATH=%ProgramFiles%\Git\bin\bash.exe

REM Name of output directory
set BUILD_OUTPUT_DIR=output

REM Set the build output path to copy build products to
set BUILD_OUTPUT_PATH=%BUILD_DIR%\%BUILD_OUTPUT_DIR%

REM Set the Contiki root path
pushd ..
set CONTIKI_ROOT=%cd%
popd

if [%1]==[] ( 
    goto :sub_help
) else if [%2]==[] (
    goto :sub_help
) else if [%3]==[] (
    goto :sub_help
)

set TARGET=%1
set BOARD=%2

set BUILD_CMD="%BASH_PATH%" -c "gmake TARGET=%TARGET% BOARD=%BOARD% %4 %5"

set APP_PATH=%CONTIKI_ROOT%\%3

REM set the location of the Contiki build's output
set BUILD_PRODUCTS_PATH=%APP_PATH%\BUILD\%TARGET%\%BOARD:/=\%

REM try to switch to the application directory
echo Building in %3...
cd %APP_PATH%
if %errorlevel% equ 1 (
    echo Error: Directory not found: %APP_PATH%
    goto END
)
if not exist "makefile" (
    echo Error: Makefile not found in %APP_PATH%
    goto END
)

echo %BUILD_CMD%
%BUILD_CMD%

if %ERRORLEVEL% NEQ 0 goto END

REM ----------------------------------------------------------------
REM Copy and rename the build products into a unique name dependent
REM on the target, board and .bin file hash.
REM
REM New file names are of the form:
REM    <app>_<target>_<board>_<hash>.bin|elf|hex|map
REM
echo Copying the build products into the output dir...
if not exist %BUILD_OUTPUT_PATH% mkdir %BUILD_OUTPUT_PATH%

echo Working in %BUILD_PRODUCTS_PATH%...
cd %BUILD_PRODUCTS_PATH%

for %%G in (*.bin) do (
    echo Processing built application '%%~nG'...
    set FILE_HASH=
    for /f "skip=3 tokens=2" %%T in ('powershell get-filehash %%G') do set FILE_HASH=%%T
    echo hash [%%G]=!FILE_HASH!
    set FW_NAME_STR=%%~nG_%TARGET%_%BOARD:/=-%_!FILE_HASH:~0,8!
    REM if there is a bin, there are also elf, hex and map files too.
    REM copy them all to the build output dir.
    echo Copying %%~nG.bin to %BUILD_OUTPUT_DIR%\!FW_NAME_STR!.bin...
    copy /y %%~nG.bin %BUILD_OUTPUT_PATH%\!FW_NAME_STR!.bin >NUL
    echo Copying %%~nG.map to %BUILD_OUTPUT_DIR%\!FW_NAME_STR!.map...
    copy /y %%~nG.map %BUILD_OUTPUT_PATH%\!FW_NAME_STR!.map >NUL
    echo Copying %%~nG.elf to %BUILD_OUTPUT_DIR%\!FW_NAME_STR!.elf...
    copy /y %%~nG.elf %BUILD_OUTPUT_PATH%\!FW_NAME_STR!.elf >NUL
    echo Copying %%~nG.hex to %BUILD_OUTPUT_DIR%\!FW_NAME_STR!.hex...
    copy /y %%~nG.hex %BUILD_OUTPUT_PATH%\!FW_NAME_STR!.hex >NUL
)


:END
cd %BUILD_DIR%
goto :eof


:sub_help
echo Missing arguments: please specify Contiki build [platform] [board] [app].
echo Example: "build keystone r1 examples\hello-world" builds the r1 board of the keystone platform for the hello-world app
goto END



