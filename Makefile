CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra

BUILD_DIR := build
TOOLS_DIR := tools

TINYASM := $(BUILD_DIR)/tinyasm
ROM_SIM := $(BUILD_DIR)/sim_rom

BASIC_ASM := uBASIC8088.asm
BASIC_ROM_BIN := $(BUILD_DIR)/uBASIC_rom.bin

CHIPSET_DIR := chipset
I8259_STUB := $(CHIPSET_DIR)/i8259.h
CONFIG_STUB := config.h
DEBUGLOG_STUB := debuglog.h

.PHONY: all help tools rom sim rom-run clean

all: rom

help:
	@echo "  make rom      - Build standalone ROM image (via tinyasm)"
	@echo "  make sim      - Build ROM simulator (sim_rom)"
	@echo "  make rom-run  - Build ROM + simulator and run"
	@echo "  make clean    - Remove build artifacts"

tools: $(TINYASM)

$(BUILD_DIR):
	mkdir -p $@

$(TINYASM): $(TOOLS_DIR)/tinyasm.c $(TOOLS_DIR)/ins.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(TOOLS_DIR)/tinyasm.c $(TOOLS_DIR)/ins.c

$(I8259_STUB): $(TOOLS_DIR)/i8259.h
	mkdir -p $(CHIPSET_DIR)
	cp -f $< $@

$(CONFIG_STUB): $(TOOLS_DIR)/config.h
	cp -f $< $@

$(DEBUGLOG_STUB): $(TOOLS_DIR)/debuglog.h
	cp -f $< $@

$(ROM_SIM): $(TOOLS_DIR)/sim_rom.c $(TOOLS_DIR)/cpu.c $(I8259_STUB) $(CONFIG_STUB) $(DEBUGLOG_STUB) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(TOOLS_DIR)/sim_rom.c $(TOOLS_DIR)/cpu.c

$(BASIC_ROM_BIN): $(BASIC_ASM) $(TINYASM) | $(BUILD_DIR)
	$(TINYASM) -f bin -dROM=1 $(BASIC_ASM) -o $@

rom: $(BASIC_ROM_BIN)

sim: $(ROM_SIM)

rom-run: rom sim
	$(ROM_SIM) $(BASIC_ROM_BIN)

clean:
	rm -rf $(BUILD_DIR) $(CHIPSET_DIR) $(CONFIG_STUB) $(DEBUGLOG_STUB)
