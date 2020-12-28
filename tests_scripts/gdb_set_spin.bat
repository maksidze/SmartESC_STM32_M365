@echo off
arm-none-eabi-gdb.exe -ex "target remote localhost:3333" -ex "set confirm off" --ex "set spinValue=%1"  --ex "set captureSpeed=%2" -ex "monitor resume" --batch C:\VSARM\Workspace\SmartESC\Debug\SmartESC.elf
