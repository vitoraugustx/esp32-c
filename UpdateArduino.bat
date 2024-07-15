@echo off

:: Variáveis que precisam ser definidas de acordo com o ambiente
set filepath=C:\Users\Vitor\Desktop\led_blink
set port=COM6

echo Listing Arduino Boards:
arduino-cli board list

echo Compiling sketch:
arduino-cli compile --fqbn esp32:esp32:esp32 %filepath%

echo Uploading sketch:
arduino-cli upload --fqbn esp32:esp32:esp32 -p %port% %filepath%

pause