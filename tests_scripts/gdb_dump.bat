@echo off

arm-none-eabi-gdb.exe -ex "target remote localhost:3333" -ex "set confirm off" -ex "dump binary value adb_buffer_storage_phA.hex adb_buffer_storage_phA"  -ex "dump binary value adb_buffer_storage_phB.hex adb_buffer_storage_phB"  -ex "dump binary value adb_buffer_storage_phC.hex adb_buffer_storage_phC"  -ex "print *adb_buffer_storage_phA@200"  -ex "print *adb_buffer_storage_phB@200"   -ex "print *adb_buffer_storage_phC@200"  -ex "monitor resume" --batch C:\VSARM\Workspace\SmartESC\Debug\SmartESC.elf
