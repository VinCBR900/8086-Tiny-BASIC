CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
BUILD_DIR := build

TINYASM_SRC := tools/tinyasm.c tools/ins.c
TINYASM_BIN := $(BUILD_DIR)/tinyasm
ROM_ASM := uBASIC8088.asm
ROM_BIN := $(BUILD_DIR)/uBASIC_rom.bin

SIM_SRC := tools/sim_rom.c tools/cpu.c tools/ins.c
SIM_HDR := tools/cpu.h tools/cpuconf.h tools/config.h tools/i8259.h tools/debuglog.h
SIM_BIN := $(BUILD_DIR)/sim_rom

.PHONY: all rom sim rom-run clean help

all: rom sim

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TINYASM_BIN): $(TINYASM_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(TINYASM_SRC)

rom: $(ROM_BIN)

$(ROM_BIN): $(ROM_ASM) $(TINYASM_BIN)
	$(TINYASM_BIN) -f bin $(ROM_ASM) -o $@

sim: $(SIM_BIN)

$(SIM_BIN): $(SIM_SRC) $(SIM_HDR) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(SIM_SRC)

rom-run: sim
	$(SIM_BIN) $(ROM_ASM)

clean:
	rm -rf $(BUILD_DIR)

help:
	@echo "Targets:"
	@echo "  make all      Build ROM assembler, ROM image, and simulator"
	@echo "  make rom      Build ROM image ($(ROM_BIN))"
	@echo "  make sim      Build simulator ($(SIM_BIN); includes tools/cpu.c)"
	@echo "  make rom-run  Run ROM in simulator from ASM source"
	@echo "  make clean    Remove build artifacts"
