# Read by ESP-IDF (and so esp-forge) as a component, and by any other CMake
# project that add_subdirectory()s this repo, such as a CubeMX-generated STM32
# one. stm-forge never reads it: it compiles every .c under src/, chip folders
# included, puts src/ on the include path of this library's sources and adds
# include/ -- which is why each port file, and the TWAI driver, is empty on
# the other platform. ESP_PLATFORM tells the two apart; IDF sets it in its
# early requirements pass as well as in the build.

if(NOT ESP_PLATFORM)
    # A static library `can`. Every .c under src/, as stm-forge does: a chip
    # file compiles to nothing unless its CONFIG_CAN_<CHIP> is 1 (the TWAI
    # driver always does here), so there is no source list here to keep in
    # step with the IDF one below.
    file(GLOB_RECURSE can_sources CONFIGURE_DEPENDS src/*.c)
    add_library(can STATIC ${can_sources})
    # include/ for the app; src/ only for this library's own sources, so a
    # chip folder reaches can_priv.h by plain name and no private header is
    # ever on the app's include path.
    target_include_directories(can PUBLIC include PRIVATE src)

    # The options are the Kconfig bools, read from Kconfig so a new chip needs
    # no line here: a true CMake variable CONFIG_CAN_<CHIP> (or
    # CONFIG_CAN_DEBUG) becomes CONFIG_CAN_<CHIP>=1, what stm-forge passes.
    # Only bools: an int option cannot be 1-or-absent. Each `config` line in
    # Kconfig is followed directly by its type line, which is what the match
    # relies on. PUBLIC, because can_log.h reads CONFIG_CAN_DEBUG in the
    # app's sources. CONFIG_CAN_TWAI is ESP-IDF's controller, so it does not
    # count as a chip picked here.
    file(READ Kconfig can_kconfig)
    string(REGEX MATCHALL "config CAN_[A-Z0-9_]+[ \t\r\n]+bool" can_options
           "${can_kconfig}")
    list(TRANSFORM can_options REPLACE "config (CAN_[A-Z0-9_]+)[ \t\r\n]+bool"
         "CONFIG_\\1")
    set(can_chips_on "")
    foreach(option IN LISTS can_options)
        if(${option})
            target_compile_definitions(can PUBLIC ${option}=1)
            if(NOT option STREQUAL "CONFIG_CAN_DEBUG" AND
               NOT option STREQUAL "CONFIG_CAN_TWAI")
                list(APPEND can_chips_on ${option})
            endif()
        endif()
    endforeach()
    if(NOT can_chips_on)
        message(WARNING "can: no chip driver is selected; "
                        "set(CONFIG_CAN_<CHIP> ON) before add_subdirectory()")
    endif()

    # The HAL is the app's. MCU_HAL_TARGET names a target carrying its include
    # dirs and defines (USE_HAL_DRIVER, the device, the hal_conf.h directory)
    # -- stm32cubemx in a CubeMX-generated project; MCU_DEVICE_HEADER is the
    # family header the public headers include as CMSIS_device_header, which
    # CubeMX does not define. Both are shared by every mcu-* library, so an
    # app sets them once. Left unset, the app provides them its own way.
    #
    # Only the target's include dirs and defines are copied, the target is
    # never linked: CubeMX up to 6.13 also hangs main.c, the HAL and the
    # startup file on stm32cubemx as INTERFACE sources, which linking would
    # compile into libcan.a. A misspelt name fails at generate time.
    if(MCU_HAL_TARGET)
        target_include_directories(can PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_INCLUDE_DIRECTORIES>)
        target_compile_definitions(can PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_COMPILE_DEFINITIONS>)
    endif()
    if(MCU_DEVICE_HEADER)
        target_compile_definitions(can PUBLIC
            "CMSIS_device_header=\"${MCU_DEVICE_HEADER}\"")
    endif()
    return()
endif()

# ESP-IDF: the chip files are listed one by one, and PRIV_INCLUDE_DIRS "src"
# lets a chip folder reach can_priv.h by plain name.
#
# The core is always built, even with CONFIG_CAN off; each chip driver only
# when its Kconfig option is on. Kconfig values are available here during
# the build phase, and only SRCS depends on them -- never REQUIRES, which IDF
# resolves before sdkconfig. That is also why CAN off still builds the core:
# an empty SRCS would need a second, differently-shaped register call, and an
# unreferenced core costs nothing once --gc-sections has run.
#
# No file name here is one of esp-sn65hvd230-can's (core.c, io.c, health.c,
# internal.h), and no global symbol either: an app links both.
#
# REQUIRES driver, not esp_driver_spi/gpio: the split components do not
# exist on IDF 5.2, and driver still pulls them in on later versions; it is
# also where the legacy TWAI driver lives. esp_timer is can_time_us()'s
# clock.
set(srcs "src/can.c" "src/can_log.c" "src/can_port_esp.c")

if(CONFIG_CAN AND NOT (CONFIG_CAN_MCP2515 OR CONFIG_CAN_TWAI))
    message(WARNING "can: CONFIG_CAN is on but no chip driver is selected; "
                    "set CONFIG_CAN_<CHIP>=y (esp-forge: CAN_<CHIP> = y)")
endif()

if(CONFIG_CAN_MCP2515)
    list(APPEND srcs "src/mcp2515/mcp2515.c")
endif()
if(CONFIG_CAN_TWAI)
    list(APPEND srcs "src/twai/twai_can.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES driver esp_timer)
