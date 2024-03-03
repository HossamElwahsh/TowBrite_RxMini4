cmake_minimum_required(VERSION 3.27)
project(RX_2024)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(RX_2024
        flash_asm.s51
        hal_defs.h
        hal_types.h
        ioCCxx10_bitdef.h
        main.c
        RADIO_CC1110_GFSK.h
        RxBoardDef.h

        MCAL/wdt/wdt_program.c
        MCAL/wdt/wdt_config.h
        MCAL/wdt/wdt_interface.h
        MCAL/wdt/wdt_private.h

)
