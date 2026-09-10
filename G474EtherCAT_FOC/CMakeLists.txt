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
set(CMAKE_PROJECT_NAME G474EtherCAT_FOC)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/src/bsp_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/src/bsp_spi_lan9252.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware/src/mt6701.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FOC/src/SVPWM.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FOC/src/VF.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FOC/src/VF_App.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FOC/src/FOC_Param.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FOC/src/Encoder_Map.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/port/el9800hw.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/aoeappl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/bootmode.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/cia402appl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/coeappl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/diag.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/ecataoe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/ecatcoe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/ecatappl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/ecateoe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/ecatfoe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/ecatslv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/ecatsoe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/emcy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/eoeappl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/foeappl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/mailbox.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/objdef.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/src/sdoserv.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Hardware/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/FOC/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/EtherCAT/port
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
