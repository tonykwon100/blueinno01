@echo off

if not exist _build\nul mkdir _build
if exist _build\* del /q _build\*

set package=%appdata%\Arduino15\packages\BlueInno2

set base=%package%\hardware\BlueInno2
dir /b %base% >%temp%\ver.txt
set /p ver= <%temp%\ver.txt
set base=%base%\%ver%

set toolchain=%package%\tools\arm-none-eabi-gcc\4.8.3-2014q1

set gcc=%toolchain%\bin\arm-none-eabi-gcc
set ar=%toolchain%\bin\arm-none-eabi-ar
set nm=%toolchain%\bin\arm-none-eabi-nm

set includes=-I../include
set includes=%includes% -I../../CMSIS/CMSIS/Include

set cflags=-Os -w -mcpu=cortex-m0 -mthumb -DNRF51 -DBOARD_PCA10001 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fno-builtin %includes%

echo compiling...

echo startup_nrf51822.c
%gcc% %cflags% -c -o _build\startup_nrf51822.o startup_nrf51822.c

echo system_nrf51.c
%gcc% %cflags% -c -o _build\system_nrf51.o system_nrf51.c

echo creating library...

set objs=_build\startup_nrf51822.o
set objs=%objs% _build\system_nrf51.o

set output=libRFduinoSystem.a

for %%f in (%objs%) do %ar% rcs _build\%output% %%f

%nm% _build\%output% >_build\%output%.txt

echo copying libray and txt to variants...
copy _build\%output% %base%\variants\BlueInno2\%output%
copy _build\%output%.txt %base%\variants\BlueInno2\%output%.txt

:end
pause
