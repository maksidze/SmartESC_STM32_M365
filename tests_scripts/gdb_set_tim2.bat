@echo off
arm-none-eabi-gdb.exe -ex "target remote localhost:3333" -ex "set confirm off" --ex "set cpt=0"  --ex "set cpt=0"  --ex "set tim2_ccr2=%1" -ex "monitor resume" --batch C:\VSARM\Workspace\SmartESC\Debug\SmartESC.elf
