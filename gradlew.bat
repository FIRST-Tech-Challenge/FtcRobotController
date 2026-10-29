@if "%DEBUG%" == "" @echo off
@rem ##########################################################################
@rem
@rem  Gradle startup script for Windows
@rem
@rem ##########################################################################

@rem Set local scope for the variables with windows NT shell
if "%OS%"=="Windows_NT" setlocal

@rem Add default JVM options here. You can also use JAVA_OPTS and GRADLE_OPTS to pass JVM options to this script.
set DEFAULT_JVM_OPTS=

set DIRNAME=%~dp0
if "%DIRNAME%" == "" set DIRNAME=.
set APP_BASE_NAME=%~n0
set APP_HOME=%DIRNAME%

@rem Find java.exe
if defined JAVA_HOME goto findJavaFromJavaHome

set JAVA_EXE=java.exe
%JAVA_EXE% -version >NUL 2>&1
if "%ERRORLEVEL%" == "0" goto init

echo.
echo ERROR: JAVA_HOME is not set and no 'java' command could be found in your PATH.
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.

goto fail

:findJavaFromJavaHome
set JAVA_HOME=%JAVA_HOME:"=%
set JAVA_EXE=%JAVA_HOME%/bin/java.exe

if exist "%JAVA_EXE%" goto init

echo.
echo ERROR: JAVA_HOME is set to an invalid directory: %JAVA_HOME%
echo.
echo Please set the JAVA_HOME variable in your environment to match the
echo location of your Java installation.

goto fail

:init
@rem Get command-line arguments, handling Windowz variants

if not "%OS%" == "Windows_NT" goto win9xME_args
if "%@eval[2+2]" == "4" goto 4NT_args

:win9xME_args
@rem Slurp the command line arguments.
set CMD_LINE_ARGS=
set _SKIP=2

:win9xME_args_slurp
if "x%~1" == "x" goto execute

set CMD_LINE_ARGS=%*
goto execute

:4NT_args
@rem Get arguments from the 4NT Shell from JP Software
set CMD_LINE_ARGS=%$

:execute
@rem Setup the command line

set CLASSPATH=%APP_HOME%\gradle\wrapper\gradle-wrapper.jar

@rem Execute Gradle
"%JAVA_EXE%" %DEFAULT_JVM_OPTS% %JAVA_OPTS% %GRADLE_OPTS% "-Dorg.gradle.appname=%APP_BASE_NAME%" -classpath "%CLASSPATH%" org.gradle.wrapper.GradleWrapperMain %CMD_LINE_ARGS%

:end
@rem End local scope for the variables with windows NT shell
if "%ERRORLEVEL%"=="0" goto mainEnd

:fail
rem Set variable GRADLE_EXIT_CONSOLE if you need the _script_ return code instead of
rem the _cmd.exe /c_ return code!
if  not "" == "%GRADLE_EXIT_CONSOLE%" exit 1
exit /b 1

:mainEnd
if "%OS%"=="Windows_NT" endlocal

:omega

@rem FTC: Gradle 9.1 needs Java 17-25; stop early with a clear fix instead of a stack trace
if defined JAVA_HOME (set "_FTC_JAVA=%JAVA_HOME%\bin\java.exe") else (set "_FTC_JAVA=java.exe")
set "_FTC_VER=" & set "_FTC_MIN="
"%_FTC_JAVA%" -fullversion 2> "%TEMP%\ftc_java_ver.txt"
for /f usebackq^ tokens^=2-3^ delims^=.^" %%a in ("%TEMP%\ftc_java_ver.txt") do (set "_FTC_VER=%%a" & set "_FTC_MIN=%%b")
del "%TEMP%\ftc_java_ver.txt" 2>NUL
if "%_FTC_VER%"=="1" set "_FTC_VER=%_FTC_MIN%"
if not defined _FTC_VER goto ftcJavaOk
if %_FTC_VER% LSS 17 goto ftcBadJava
if %_FTC_VER% LEQ 25 goto ftcJavaOk
:ftcBadJava
echo. 1>&2
echo BUILD STOPPED: found Java %_FTC_VER%, but this project needs Java 17-25. 1>&2
echo Run these, then try again: 1>&2
echo. 1>&2
echo   set "JAVA_HOME=C:\Program Files\Android\Android Studio\jbr" 1>&2
echo   gradlew --stop 1>&2
echo. 1>&2
echo In PowerShell, use instead: $env:JAVA_HOME="C:\Program Files\Android\Android Studio\jbr" 1>&2
echo To make it permanent: setx JAVA_HOME "C:\Program Files\Android\Android Studio\jbr" 1>&2
echo. 1>&2
goto fail
:ftcJavaOk