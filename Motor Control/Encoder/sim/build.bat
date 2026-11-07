@echo off
rem Compila el simulador con MinGW-w64 / MSYS2 (gcc en el PATH)
gcc -O2 -std=c99 -Wall -Wextra -o encoder_sim.exe encoder_sim.c ..\src\encoder_core.c -lm
if errorlevel 1 (echo ERROR de compilacion & exit /b 1)
echo Compilado. Ejecuta:  encoder_sim.exe quick   ^(~40 s^)   o   encoder_sim.exe full   ^(varios minutos^)
