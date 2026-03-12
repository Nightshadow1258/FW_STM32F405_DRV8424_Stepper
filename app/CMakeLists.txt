#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(BOARD stm32f405_stepper)

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

# Add custom driver directory
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../drivers/sensor/as5047p build)
# list(APPEND EXTRA_EXTERNAL_MODULE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../drivers)
# list(APPEND EXTRA_EXTERNAL_MODULE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../lib)

target_sources(app PRIVATE 
    src/main.c
    # add additional source files here
    )
