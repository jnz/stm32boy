@echo off
IF %1.==. GOTO NoFirmware
set Firmware=%1
GOTO CheckId
:NoFirmware
set Firmware="firmware.hex"

:CheckId
IF %2.==. GOTO NoID
set DevID=%2
"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe" -List
GOTO Flash
:NoID
set DevID=0

:Flash
echo ST-LINK device index: %DevID%
"C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe" -c ID=%DevID% SWD UR -p %Firmware% -V "while_programming" -Rst -NoPrompt -OB IWDG_SW=1

if errorlevel 1 exit /b 1
