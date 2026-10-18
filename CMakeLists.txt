cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Documents) 
# Yeah I know "Documents" is a wierd name for a project, wasn't intentional
# STMCubeMX took that name for some reason and I didn't bother to change 

# Select exactly one application entry point and its explicit dependencies.
set(FIRMWARE_APP "flight" CACHE STRING "Firmware application to build")
set_property(CACHE FIRMWARE_APP PROPERTY STRINGS flight test_template test_sd test_sensors test_gnss test_xbee test_camera)

if(FIRMWARE_APP STREQUAL "flight")
    set(FIRMWARE_SOURCES
        Core/Src/apps/FlightApp.c
        Core/Src/apps/Cansat.c
        Core/Src/apps/SensorAcquisition.c
        Core/Src/apps/Communication.c
        Core/Src/apps/Telemetry.c
        Core/Src/apps/Logging.c
        Core/Src/apps/StateMachine.c
        Core/Src/apps/BarometricAltitude.c
        Core/Src/apps/CameraControl.c
        Core/Src/drivers/MissionStore.c
        Core/Src/drivers/MissionStore_HAL.c
    )
elseif(FIRMWARE_APP STREQUAL "test_template")
    set(FIRMWARE_SOURCES
        Tests/test_template.c
    )
elseif(FIRMWARE_APP STREQUAL "test_sd")
    set(FIRMWARE_SOURCES Tests/test_sd.c)
elseif(FIRMWARE_APP STREQUAL "test_sensors")
    set(FIRMWARE_SOURCES Tests/test_sensors.c)
elseif(FIRMWARE_APP STREQUAL "test_gnss")
    set(FIRMWARE_SOURCES Tests/test_gnss.c)
elseif(FIRMWARE_APP STREQUAL "test_xbee")
    set(FIRMWARE_SOURCES Tests/test_xbee.c)
elseif(FIRMWARE_APP STREQUAL "test_camera")
    set(FIRMWARE_SOURCES Tests/test_camera.c Core/Src/apps/CameraControl.c)
else()
    message(FATAL_ERROR
        "Unknown FIRMWARE_APP '${FIRMWARE_APP}'. Choose flight, test_template, test_sd, test_sensors, test_gnss or test_xbee.")
endif()

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "${FIRMWARE_APP}")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Console code, headers and float formatting belong to test firmware only.
if(NOT FIRMWARE_APP STREQUAL "flight")
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE Tests/Console.c)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE Tests)
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
endif()

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${FIRMWARE_SOURCES}
    Core/Src/sd_spi.c
    Core/Src/sd_diskio_spi.c
    Core/Src/sd_functions.c
    Core/Src/sd_benchmark.c
    Core/Src/drivers/SDCard.c
    Core/Src/drivers/Switches.c
    Core/Src/drivers/Servo.c
    Core/Src/drivers/Battery.c
    Core/Src/drivers/SensorI2C.c
    Core/Src/drivers/BMP.c
    Core/Src/drivers/AHT20.c
    Core/Src/drivers/bmi270.c
    Core/Src/drivers/bmi270_config.c
    Core/Src/drivers/IIS2MDC.c
    Core/Src/drivers/GNSS.c
    Core/Src/drivers/GNSS_HAL.c
    Core/Src/drivers/XBee.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
