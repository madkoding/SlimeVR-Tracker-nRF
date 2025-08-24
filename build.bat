@echo off
echo ====================================
echo Building SlimeVR-Tracker-nRF 
echo ====================================
echo.

REM Command from the GitHub workflow for building with custom board
west build ^
    --board promicro_uf2/nrf52840/spi ^
    --pristine=always ^
    --build-dir build ^
    -- ^
    -DBOARD_ROOT=.

if %errorlevel% equ 0 (
    echo.
    echo ====================================
    echo BUILD SUCCESSFUL!
    echo ====================================
    echo Output file: build\SlimeVR-Tracker-nRF\zephyr\zephyr.uf2
    echo.
) else (
    echo.
    echo ====================================
    echo BUILD FAILED!
    echo ====================================
    echo Check the error messages above.
    echo.
)

