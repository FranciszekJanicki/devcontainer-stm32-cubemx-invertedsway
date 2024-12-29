PROJECT_DIR := $(shell pwd)
BUILD_DIR := ${PROJECT_DIR}/build
SOURCE_DIR := ${PROJECT_DIR}/Core
DRIVERS_DIR := ${PROJECT_DIR}/Drivers
REQUIREMENTS_DIR := ${PROJECT_DIR}/requirements
STM32CUBEMX_DIR := ${PROJECT_DIR}/cmake/stm32cubemx
export PATH := $(PATH)

.PHONY: build
build: 
	cd ${BUILD_DIR} && make

.PHONY: clean
clean: 
	rm -rf ${BUILD_DIR}

.PHONY: cmake
cmake:
	cd ${PROJECT_DIR} && make clean && mkdir build && cmake -S . -B build

.PHONY: flash
flash: 
	STM32_Programmer_CLI -c port=swd -d ${BUILD_DIR}/devcontainer-stm32-cubemx-invertedsway.elf -rst

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
