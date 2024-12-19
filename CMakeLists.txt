cmake_minimum_required(VERSION 3.28)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME devcontainer-stm32-cubemx-invertedsway)

include("cmake/gcc-arm-none-eabi.cmake")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

enable_language(C CXX ASM)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    "Core/Src/system_clock.c"
    "Core/Src/main.cpp"
    "Core/Src/mpu6050.cpp"
    "Core/Src/l298n.cpp"
    "Core/Src/motor.cpp"
    "Core/Src/system.cpp"
    "Core/Inc/regulators.hpp"
    "Core/Inc/kalman.hpp"
    "Core/Src/encoder.cpp"
    "Core/Src/tests.cpp"
    "Core/Src/gpio.c"
    "Core/Src/i2c.c"
    "Core/Src/tim.c"
    "Core/Src/usart.c"
    "Core/Src/stm32l4xx_it.c"
    "Core/Src/stm32l4xx_hal_msp.c"
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "Inc"
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC
    -std=c++23
    -fconcepts
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Wimplicit
    -Waddress
    -Wnon-virtual-dtor
    -pedantic
    -Wpessimizing-move
    -Wsign-conversion
    -Wmisleading-indentation
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wregister
    -Wnull-dereference
    -Wuseless-cast
    -Wdouble-promotion
    -Wformat=2
    -Wimplicit-fallthrough
    -Wcast-align
)