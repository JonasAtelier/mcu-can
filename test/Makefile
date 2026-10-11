CC ?= cc
# gnu11: the can_* log macros use `, ##__VA_ARGS__`.
CFLAGS += -std=gnu11 -Wall -Wextra -Werror -pedantic
# Signed overflow usually wraps anyway, so a test of wrapping arithmetic
# passes on the broken code too. UBSan makes it abort instead.
CFLAGS += -fsanitize=undefined -fno-sanitize-recover=all
# src/ for the common private header, and every chip folder, because the
# tests include a chip's private headers by plain name.
CPPFLAGS += -I../include -I../src $(addprefix -I,$(wildcard ../src/*/))

# Two suites, one per platform: the same chip driver through
# src/can_port_esp.c against idf_stubs/ (esp/), and through
# src/can_port_stm.c against hal_stubs/ (stm/). The cases live once, in
# *_cases.h, and both platforms run them against their own fakes. The TWAI
# is ESP-IDF's only and bxCAN the HAL's only, so each has one test.
# test_mcp2515_math.c touches no bus or platform API and runs once;
# test_names.c is the ESP-IDF header coexistence check against a copy of
# esp-sn65hvd230-can's hvd230can.h in fixtures/.

# What ESP-IDF passes: ESP_PLATFORM, and every chip on.
ESP_CPPFLAGS := -Iidf_stubs -DESP_PLATFORM -DCONFIG_CAN_MCP2515=1 \
		-DCONFIG_CAN_TWAI=1
# What stm-forge passes: the family header the code includes, the HAL modules
# an app's hal_conf would enable, and every chip option on (CAN_<CHIP> = y).
STM_CPPFLAGS := -Ihal_stubs \
		-DCMSIS_device_header='"stm32_hal_stub.h"' \
		-DHAL_I2C_MODULE_ENABLED -DHAL_SPI_MODULE_ENABLED \
		-DHAL_UART_MODULE_ENABLED -DHAL_CAN_MODULE_ENABLED \
		-DCONFIG_CAN_MCP2515=1 -DCONFIG_CAN_BXCAN=1 \
		-DCONFIG_CAN_DEBUG=1

# The core and the stubs every binary of a suite links.
CORE     := ../src/can.c ../src/can_log.c
ESP_CORE := $(CORE) ../src/can_port_esp.c idf_stubs/idf_stubs.c
STM_CORE := $(CORE) ../src/can_port_stm.c hal_stubs/hal_stubs.c

MCP2515 := ../src/mcp2515/mcp2515.c
TWAI    := ../src/twai/twai_can.c
BXCAN   := ../src/bxcan/bxcan.c

MATH_TESTS := test_mcp2515_math test_names
# Plus one port test each, for the bus ops.
ESP_TESTS  := $(addprefix esp/,test_core test_mcp2515 test_twai test_port_esp)
STM_TESTS  := $(addprefix stm/,test_core test_mcp2515 test_bxcan test_port_stm)
TESTS      := $(MATH_TESTS) $(ESP_TESTS) $(STM_TESTS)

.PHONY: all clean test

all: $(TESTS)

test_mcp2515_math: test_mcp2515_math.c
test_names: test_names.c
test_names: CPPFLAGS += -Ifixtures $(ESP_CPPFLAGS)

esp/test_core: esp/test_core.c $(ESP_CORE)
esp/test_mcp2515: esp/test_mcp2515.c $(MCP2515) $(ESP_CORE)
esp/test_twai: esp/test_twai.c $(TWAI) $(ESP_CORE)
esp/test_port_esp: esp/test_port_esp.c $(ESP_CORE)

stm/test_core: stm/test_core.c $(STM_CORE)
stm/test_mcp2515: stm/test_mcp2515.c $(MCP2515) $(STM_CORE)
stm/test_bxcan: stm/test_bxcan.c $(BXCAN) $(STM_CORE)
stm/test_port_stm: stm/test_port_stm.c $(STM_CORE)

$(ESP_TESTS): CPPFLAGS += $(ESP_CPPFLAGS)
$(STM_TESTS): CPPFLAGS += $(STM_CPPFLAGS)

# Lets the driver tests count calloc() and free(), to prove a failed init
# leaks nothing. GNU ld; only the driver's own references are wrapped.
DRIVER_TESTS := esp/test_mcp2515 esp/test_twai stm/test_mcp2515 stm/test_bxcan
$(DRIVER_TESTS): LDFLAGS += -Wl,--wrap=calloc -Wl,--wrap=free
# Every printf() in a binary that links the HAL stubs also lands in
# stub_log_last, so a test can check what was logged.
$(STM_TESTS): LDFLAGS += -Wl,--wrap=printf

$(TESTS):
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) $(filter %.c,$^) -o $@ -lm

test: $(TESTS)
	@set -e; for t in $(TESTS); do echo "./$$t"; ./$$t; done

clean:
	rm -f $(TESTS)

# Headers are prerequisites so that editing one rebuilds. Without this, `make
# test` reruns a stale binary and reports green for code it never compiled.
# Recipes use $(filter %.c,$^) so the headers never reach the compiler.
HDRS  := $(wildcard ../include/*.h ../src/*.h ../src/*/*.h)
STUBS := $(wildcard idf_stubs/*.h idf_stubs/*/*.h hal_stubs/*.h fixtures/*.h)
$(TESTS): $(HDRS) $(STUBS) $(wildcard *_cases.h)
