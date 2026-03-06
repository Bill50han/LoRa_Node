@echo off
echo ========================================
echo  LoRa Node Host - Build Script
echo ========================================
echo.

:: Check for MinGW g++
where g++ >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] g++ not found in PATH
    echo Please install MinGW-w64 and add to PATH
    pause
    exit /b 1
)

echo [INFO] Compiling with MinGW...
g++ -o lora_host.exe main.cpp -mwindows -lcomctl32 -static -O2

if %errorlevel% equ 0 (
    echo.
    echo [SUCCESS] Build complete: lora_host.exe
    echo.
) else (
    echo.
    echo [ERROR] Build failed
)

pause
