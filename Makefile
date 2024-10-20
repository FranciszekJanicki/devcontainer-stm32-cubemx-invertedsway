include make/common.mk
include make/linters.mk

.PHONY: build
build: 
	cd ${BUILD_DIR} && make


.PHONY: flash
flash: 
	cd ${BUILD_DIR} && STM32_Programmer_CLI -c port=SWD -w devcontainer-stm32-cubemx-invertedsway.elf -rst 0x08000000