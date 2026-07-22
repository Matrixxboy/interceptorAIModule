@echo off
setlocal
echo ===================================================
echo   Building Arjuna GCS Graphical Executable (.exe)
echo ===================================================
echo.

python build_exe.py

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Build failed! Check the output above for errors.
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo [SUCCESS] Full graphical Arjuna GCS executable built successfully!
echo The output folder is located at dist\Arjuna\Arjuna.exe
echo.
pause
