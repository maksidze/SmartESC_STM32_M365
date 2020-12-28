@echo off

set /a MIN = %1
set /a MAX = %3
set /a INC = %2
set /a SPEED = %4

echo MIN = %MIN%
echo MAX = %MAX%
echo INC = %INC%

for /l %%v in (%MIN%, %INC%, %MAX%) do (

	echo ##########################################################################
	echo ##########################################################################
	echo    Testing %%v - min %MIN% / max %MAX% / increment %INC% / speed %SPEED%
	echo ##########################################################################
	echo ##########################################################################

	call gdb_set_tim2.bat %%v

	call gdb_set_spin.bat 3000 %SPEED%

	timeout /t 10

	call gdb_dump.bat %%v
	
	call plot_no_display.bat %%v

)