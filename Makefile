CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra
ASM ?= nasm
ASMFLAGS ?= -f bin

BUILD_DIR := build
TOOLS_DIR := tools

TINYASM := $(BUILD_DIR)/tinyasm
EMULATOR := $(BUILD_DIR)/8086tiny
ROM_SIM := $(BUILD_DIR)/sim_rom

BASIC_ASM := uBASIC8088.asm
BOOT_ASM := $(TOOLS_DIR)/bootsect.asm
BIOS_ASM := $(TOOLS_DIR)/biosblob.asm

BASIC_BIN := $(BUILD_DIR)/uBASIC8088.bin
BASIC_ROM_BIN := $(BUILD_DIR)/uBASIC_rom.bin
BASIC_8BW_BIN := $(BUILD_DIR)/uBASIC_8bitworkshop.bin
BOOT_BIN := $(BUILD_DIR)/boot.bin
BIOS_BIN := $(BUILD_DIR)/bios.bin
FLOPPY_IMG := $(BUILD_DIR)/floppy.img

CHIPSET_DIR := chipset
I8259_STUB := $(CHIPSET_DIR)/i8259.h
CONFIG_STUB := config.h
DEBUGLOG_STUB := debuglog.h

.PHONY: all help tools image run rom rom-run sim clean

all: image

help:
	@echo "Build options from uBASIC8088.asm header:"
	@echo "  make image    - Variant 1 (8086tiny batch-test) floppy + BIOS images"
	@echo "  make run      - Variant 1 run under 8086tiny"
	@echo "  make rom      - Variant 3 standalone ROM image (-dROM=1 via tinyasm)"
	@echo "  make rom-run  - Variant 3 run ROM image in sim_rom"
	@echo "  make sim      - Build only Variant 3 simulator (sim_rom)"
	@echo "  make clean    - Remove build artifacts"
	@echo ""
	@echo "Variant 2 (8bitworkshop) is selected automatically by yasm (__YASM_MAJOR__)."
	@echo "Optional local artifact: make $(BASIC_8BW_BIN)"

tools: $(TINYASM) $(EMULATOR)

$(BUILD_DIR):
	mkdir -p $@

$(TINYASM): $(TOOLS_DIR)/tinyasm.c $(TOOLS_DIR)/ins.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(TOOLS_DIR)/tinyasm.c $(TOOLS_DIR)/ins.c

$(EMULATOR): $(TOOLS_DIR)/8086tiny.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DNO_GRAPHICS -o $@ $(TOOLS_DIR)/8086tiny.c

$(I8259_STUB): $(TOOLS_DIR)/i8259.h
	mkdir -p $(CHIPSET_DIR)
	cp -f $< $@

$(CONFIG_STUB): $(TOOLS_DIR)/config.h
	cp -f $< $@

$(DEBUGLOG_STUB): $(TOOLS_DIR)/debuglog.h
	cp -f $< $@

$(ROM_SIM): $(TOOLS_DIR)/sim_rom.c $(TOOLS_DIR)/cpu.c $(I8259_STUB) $(CONFIG_STUB) $(DEBUGLOG_STUB) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(TOOLS_DIR)/sim_rom.c $(TOOLS_DIR)/cpu.c

$(BASIC_BIN): $(BASIC_ASM) | $(BUILD_DIR)
	$(ASM) $(ASMFLAGS) $(BASIC_ASM) -o $@

$(BASIC_ROM_BIN): $(BASIC_ASM) $(TINYASM) | $(BUILD_DIR)
	$(TINYASM) -f bin -dROM=1 $(BASIC_ASM) -o $@

$(BASIC_8BW_BIN): $(BASIC_ASM) | $(BUILD_DIR)
	yasm -f bin $(BASIC_ASM) -o $@

$(BOOT_BIN): $(BOOT_ASM) | $(BUILD_DIR)
	$(ASM) $(ASMFLAGS) $(BOOT_ASM) -o $@

$(BIOS_BIN): $(BIOS_ASM) | $(BUILD_DIR)
	$(ASM) $(ASMFLAGS) $(BIOS_ASM) -o $@

$(FLOPPY_IMG): $(BOOT_BIN) $(BASIC_BIN) | $(BUILD_DIR)
	python3 -c 'from pathlib import Path; boot=Path("$(BOOT_BIN)").read_bytes(); basic=Path("$(BASIC_BIN)").read_bytes(); limit=5*512; assert len(basic)<=limit, f"BASIC binary too large ({len(basic)} bytes), max is {limit} bytes"; out=Path("$@"); out.write_bytes(boot+basic+bytes(limit-len(basic))); print(f"Wrote {out} ({out.stat().st_size} bytes)")'

image: $(FLOPPY_IMG) $(BIOS_BIN)

run: image $(EMULATOR)
	$(EMULATOR) $(BIOS_BIN) $(FLOPPY_IMG)

rom: $(BASIC_ROM_BIN)

sim: $(ROM_SIM)

rom-run: rom sim
	$(ROM_SIM) $(BASIC_ROM_BIN)

clean:
	rm -rf $(BUILD_DIR) $(CHIPSET_DIR) $(CONFIG_STUB) $(DEBUGLOG_STUB)
