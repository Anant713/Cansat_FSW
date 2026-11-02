SOURCES := \
	dcs_attitude.c \
	dcs_circle_fit.c \
	dcs_control.c \
	dcs_dubins.c \
	dcs_flight_config.c \
	dcs_guidance.c \
	dcs_nav.c \
	dcs_planner.c \
	dcs_runtime.c \
	dcs_sensor_transform.c \
	dcs_wind.c \
	third_party/Fusion/Fusion/FusionAhrs.c \
	third_party/Fusion/Fusion/FusionConvention.c

COMMON_FLAGS := -std=c11 -O2 -Wall -Wextra -Werror -pedantic \
	-DFUSION_USE_NORMAL_SQRT -I.
HOST_BUILD := .build/host/all
ARM_BUILD := .build/arm/all
HOST_OBJECTS := $(patsubst %.c,$(HOST_BUILD)/%.o,$(SOURCES))
ARM_OBJECTS := $(patsubst %.c,$(ARM_BUILD)/%.o,$(SOURCES))
ARM_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 \
	-mfloat-abi=hard -ffunction-sections -fdata-sections

.PHONY: all host arm tests clean

all: host

host: $(HOST_BUILD)/libdcs.a

arm: $(ARM_BUILD)/libdcs.a

$(HOST_BUILD)/libdcs.a: $(HOST_OBJECTS)
	$(AR) rcs $@ $^

$(ARM_BUILD)/libdcs.a: $(ARM_OBJECTS)
	$(ARM_AR) rcs $@ $^

$(HOST_BUILD)/%.o: %.c
	@mkdir -p $(@D)
	$(CC) $(COMMON_FLAGS) -c $< -o $@

$(ARM_BUILD)/%.o: %.c
	@mkdir -p $(@D)
	$(ARM_CC) $(COMMON_FLAGS) $(ARM_FLAGS) -c $< -o $@

tests:
	powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1

clean:
	powershell -NoProfile -Command "Remove-Item -LiteralPath '.build/host/all','.build/arm/all' -Recurse -Force -ErrorAction SilentlyContinue"

CC := gcc
AR := ar
ARM_CC := arm-none-eabi-gcc
ARM_AR := arm-none-eabi-ar
