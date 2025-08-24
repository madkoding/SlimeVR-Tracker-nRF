@echo off
echo ====================================
echo Cleaning build directory
echo ====================================
echo.

if exist build (
    echo Removing build directory...
    rmdir /s /q build
    echo Build directory removed.
) else (
    echo Build directory does not exist.
)

echo.
echo ====================================
echo CLEAN COMPLETE!
echo ====================================
echo.

