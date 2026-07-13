# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)


find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(blinky)

# If board is nRF9160 add the source files 
if (CONFIG_BOARD_CIRCUITDOJO_FEATHER_NRF9160)
    target_sources(app PRIVATE ${ZEPHYR_BASE}/samples/basic/blinky/src/main.c)
elseif(CONFIG_BOARD_CIRCUITDOJO_FEATHER_NRF9161 OR CONFIG_BOARD_CIRCUITDOJO_FEATHER_NRF9151)
    target_sources(app PRIVATE src/main.c)
else()
    message(FATAL_ERROR "Unsupported board")
endif()



