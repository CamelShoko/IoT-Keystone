@echo off
SETLOCAL EnableDelayedExpansion
REM Keystone-Prod firmware build file
REM Copyright (c) 2018 This. Is. IoT. 
REM Not licensed for distribution.

REM Usage:
REM   %1 - Contiki TARGET e.g. simplelink
REM   %2 - Contiki BOARD e.g. launchpad/cc1352r1
REM   %3 - path to application folder containing makefile to build.
REM        e.g. build examples/hello-world
REM   %4 - optional argument to append e.g. clean
REM   %5 - optional argument to append

REM We need to use Git's bash command line interpreter
REM to handle some advanced makefile features that TI has
REM added to their recent simplelink platform addition.
REM namely the 'find' command.
REM Otherwise gmake could be run from the ordinary Windows
REM command-line.

REM gmake.exe and srec_cat.exe must be available.
REM Look at tools\build\README.md for details.

set BUILD_DIR=%cd%

set BASH_PATH=%ProgramFiles%\Git\bin\bash.exe


REM Set the Contik root path
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

REM try to switch to the application directory
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

:END
cd %BUILD_DIR%
goto :eof


:sub_help
echo Missing arguments: please specify Contiki build [TARGET] [BOARD] [app].
goto END



