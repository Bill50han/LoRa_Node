cmake_minimum_required(VERSION 3.15)
project(lora_host LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Windows only
if(NOT WIN32)
    message(FATAL_ERROR "This project is Windows only")
endif()

# Add executable
add_executable(lora_host WIN32 main.cpp)

# Link libraries
target_link_libraries(lora_host PRIVATE
    comctl32
)

# Enable Unicode
target_compile_definitions(lora_host PRIVATE
    UNICODE
    _UNICODE
    WIN32_LEAN_AND_MEAN
)

# MSVC specific
if(MSVC)
    target_compile_options(lora_host PRIVATE /W3)
endif()

# MinGW specific - static link runtime
if(MINGW)
    target_link_options(lora_host PRIVATE -static)
endif()
