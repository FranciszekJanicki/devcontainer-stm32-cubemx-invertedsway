PROJECT_DIR := $(shell pwd)
BUILD_DIR := ${PROJECT_DIR}/build
SOURCE_DIR := ${PROJECT_DIR}/Core
DRIVERS_DIR := ${PROJECT_DIR}/Drivers
REQUIREMENTS_DIR := ${PROJECT_DIR}/requirements
STM32CUBEMX_DIR := ${PROJECT_DIR}/cmake/stm32cubemx

.PHONY: build
build: 
	cd ${BUILD_DIR} && make

.PHONY: clean
clean: 
	rm -rf ${BUILD_DIR}


.PHONY: flash
flash: 
	arm-none-eabi-objcopy -O binary ${BUILD_DIR}/devcontainer-stm32-cubemx-invertedsway.elf devcontainer-stm32-cubemx-invertedsway.bin
	st-flash --connect-under-reset write devcontainer-stm32-cubemx-invertedsway.bin 0x8000000
	rm devcontainer-stm32-cubemx-invertedsway.bin
.PHONY: serial
serial:
	minicom -D /dev/ttyACM0 -b 115200


.PHONY: clang-format
clang-format:
	for ext in h c cpp hpp; do \
		find $(SOURCE_DIR) -iname "*.$$ext" -print0 | xargs -0 -r clang-format -i; \
	done

.PHONY: all
all:
	make clang-format && make build && make flash && make serial
