# ESP-IDF only. stm-forge never reads this file: it compiles every .c under
# src/, chip folders included, puts src/ on the include path of this
# library's sources and adds include/ -- which is why each port file, and
# the TWAI driver, is empty on the other platform. Here the chip files are
# listed one by one, and PRIV_INCLUDE_DIRS "src" lets a chip folder reach
# can_priv.h by plain name.
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
