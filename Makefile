PROJECT_DIR := $(shell pwd)
BUILD_DIR := ${PROJECT_DIR}/build
SOURCE_DIR := ${PROJECT_DIR}/Core
DRIVERS_DIR := ${PROJECT_DIR}/Drivers
REQUIREMENTS_DIR := ${PROJECT_DIR}/requirements
STM32CUBEMX_DIR := ${PROJECT_DIR}/cmake/stm32cubemx


.PHONY: build
build: 
	cd ${BUILD_DIR} && make


.PHONY: flash
flash: 
	cd ${BUILD_DIR} && STM32_Programmer_CLI -c port=SWD -w devcontainer-stm32-cubemx-invertedsway.elf -rst 0x08000000

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
