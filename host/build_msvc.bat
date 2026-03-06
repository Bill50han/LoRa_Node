@echo off
echo ========================================
echo  LoRa Node Host - MSVC Build Script
echo ========================================
echo.

:: Check for cl.exe
where cl >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] cl.exe not found in PATH
    echo Please run from Visual Studio Developer Command Prompt
    echo Or run: "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
    pause
    exit /b 1
)

echo [INFO] Compiling with MSVC...
cl /EHsc /O2 /W3 /Fe:lora_host.exe main.cpp user32.lib gdi32.lib comctl32.lib /link /SUBSYSTEM:WINDOWS

if %errorlevel% equ 0 (
    echo.
    echo [SUCCESS] Build complete: lora_host.exe
    echo.
    :: Clean up intermediate files
    del *.obj >nul 2>&1
) else (
    echo.
    echo [ERROR] Build failed
)

pause
