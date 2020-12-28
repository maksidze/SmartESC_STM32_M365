@echo off

C:\VSARM\gnuplot\bin\gnuplot.exe  -p -e "set terminal png font arial 14 size 1600,800; set output 'file_%1.png'; set yrange [-4000:4000]; plot 'C:\VSARM\openocd\adb_buffer_storage_phA.hex' binary format='%%int16' u 0:1 every ::0::1000 with lines, 'C:\VSARM\openocd\adb_buffer_storage_phB.hex' binary format='%%int16' u 0:1 every ::0::1000 with lines"
