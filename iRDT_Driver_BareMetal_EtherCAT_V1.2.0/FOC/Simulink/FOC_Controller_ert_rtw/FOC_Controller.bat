@echo off

set MATLAB=D:\Program Files\R2021a

cd .

if "%1"=="" ("D:\PROGRA~2\R2021a\bin\win64\gmake"  -f FOC_Controller.mk all) else ("D:\PROGRA~2\R2021a\bin\win64\gmake"  -f FOC_Controller.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1