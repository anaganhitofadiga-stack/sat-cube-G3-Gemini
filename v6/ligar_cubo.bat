@echo off
setlocal enabledelayedexpansion
rem ===================================================================
rem  ligar_cubo.bat - compila, grava e liga-se ao cubo num duplo clique.
rem
rem  Uso:  duplo clique, ou   ligar_cubo.bat COM7   para forcar a porta.
rem ===================================================================
title Cubo V5 - ligar
cd /d "%~dp0"

echo.
echo === 1/5  Procurar o ESP-IDF ===
where idf.py >nul 2>&1
if %errorlevel%==0 (
    echo     idf.py ja esta no PATH.
    goto :tem_idf
)
set "IDF_EXPORT="
for %%R in ("%USERPROFILE%\esp" "%USERPROFILE%\.espressif\frameworks" "C:\Espressif\frameworks" "C:\esp") do (
    if exist "%%~R\esp-idf\export.bat" set "IDF_EXPORT=%%~R\esp-idf\export.bat"
    for /d %%D in ("%%~R\*") do (
        if exist "%%~D\export.bat" set "IDF_EXPORT=%%~D\export.bat"
        if exist "%%~D\esp-idf\export.bat" set "IDF_EXPORT=%%~D\esp-idf\export.bat"
    )
)
if not defined IDF_EXPORT (
    echo     NAO ENCONTREI o ESP-IDF.
    echo     Abre o VS Code, Ctrl+Shift+P, "ESP-IDF: Open ESP-IDF Terminal",
    echo     e corre este ficheiro desse terminal:  ligar_cubo.bat
    goto :fim
)
echo     encontrado: !IDF_EXPORT!
call "!IDF_EXPORT!" >nul
where idf.py >nul 2>&1
if not %errorlevel%==0 (
    echo     o export.bat correu mas o idf.py continua fora do PATH.
    goto :fim
)
:tem_idf

echo.
echo === 2/5  Descobrir a porta do XIAO ===
set "PORTA=%~1"
if defined PORTA (
    echo     porta dada na linha de comando: !PORTA!
) else (
    for /f "usebackq delims=" %%P in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0ferramentas\detetar_porta.ps1"`) do set "PORTA=%%P"
)
if not defined PORTA (
    echo     Nenhuma porta COM. Verifica:
    echo       - cabo USB-C de DADOS ligado ao XIAO
    echo       - se nada aparecer: BOOT premido + toque no RESET + largar BOOT
    goto :fim
)
echo     vou usar !PORTA!

echo.
echo === 3/5  Compilar ===
cd /d "%~dp0firmware"
if not exist sdkconfig (
    echo     primeira vez: idf.py set-target esp32s3
    call idf.py set-target esp32s3
    if not !errorlevel!==0 goto :erro
)
call idf.py build
if not !errorlevel!==0 goto :erro

echo.
echo === 4/5  Gravar na placa ===
call idf.py -p !PORTA! flash
if not !errorlevel!==0 (
    echo     A gravacao falhou. Tenta: BOOT premido + toque no RESET + largar BOOT,
    echo     e corre outra vez.
    goto :erro
)

echo.
echo === 5/5  Ligar ao cubo ===
cd /d "%~dp0"
python -c "import serial" >nul 2>&1
if not %errorlevel%==0 (
    echo     Falta a biblioteca pyserial.
    set /p RESP="     Instalar agora com pip? [S/N] "
    if /i "!RESP!"=="S" python -m pip install pyserial matplotlib
)
for /f "usebackq delims=" %%T in (`powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmm"`) do set "STAMP=%%T"
set "CSV=ensaio_!STAMP!.csv"
echo.
echo     MANTEM O CUBO PARADO nos proximos 2 segundos (calibracao do giroscopio).
echo     A gravar em !CSV!  -  escreve ? e Enter para a lista de comandos.
echo.
python ferramentas\gs_cubo.py --porta !PORTA! --saida "!CSV!"
goto :fim

:erro
echo.
echo *** Parei por causa do erro acima. ***

:fim
echo.
pause
endlocal
