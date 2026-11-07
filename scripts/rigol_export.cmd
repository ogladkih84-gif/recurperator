@echo off
chcp 65001 >nul
setlocal
set PYTHONUTF8=1
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 (
  echo Не найден Python Launcher. Установите Python 3 для Windows с компонентом py launcher.
  pause
  exit /b 1
)
py -3 -c "import pyvisa" >nul 2>nul
if errorlevel 1 (
  echo Устанавливаю библиотеку PyVISA через pip в выбранный Python 3.
  py -3 -m pip install "pyvisa>=1.14,<2"
  if errorlevel 1 (
    echo Установка не завершилась. Пришлите сообщение об ошибке.
    pause
    exit /b 1
  )
)
echo Осциллограф должен хранить нужную запись в STOP. Закройте Ultra Sigma перед выгрузкой.
py -3 rigol_export.py %*
pause
