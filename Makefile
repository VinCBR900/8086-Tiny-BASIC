CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra
ASM ?= nasm
ASMFLAGS ?= -f bin

BUILD_DIR := build
TOOLS_DIR := tools

TINYASM := $(BUILD_DIR)/tinyasm
EMULATOR := $(BUILD_DIR)/8086tiny

BASIC_ASM := uBASIC8088.asm
BOOT_ASM := $(TOOLS_DIR)/bootsect.asm
BIOS_ASM := $(TOOLS_DIR)/biosblob.asm

BASIC_BIN := $(BUILD_DIR)/uBASIC8088.bin
BOOT_BIN := $(BUILD_DIR)/boot.bin
BIOS_BIN := $(BUILD_DIR)/bios.bin
FLOPPY_IMG := $(BUILD_DIR)/floppy.img

.PHONY: all tools image run clean

all: image

tools: $(TINYASM) $(EMULATOR)

$(BUILD_DIR):
	mkdir -p $@

$(TINYASM): $(TOOLS_DIR)/tinyasm.c $(TOOLS_DIR)/ins.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $(TOOLS_DIR)/tinyasm.c $(TOOLS_DIR)/ins.c

$(EMULATOR): $(TOOLS_DIR)/8086tiny.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -DNO_GRAPHICS -o $@ $(TOOLS_DIR)/8086tiny.c

$(BASIC_BIN): $(BASIC_ASM) | $(BUILD_DIR)
	$(ASM) $(ASMFLAGS) $(BASIC_ASM) -o $@

$(BOOT_BIN): $(BOOT_ASM) | $(BUILD_DIR)
	$(ASM) $(ASMFLAGS) $(BOOT_ASM) -o $@

$(BIOS_BIN): $(BIOS_ASM) | $(BUILD_DIR)
	$(ASM) $(ASMFLAGS) $(BIOS_ASM) -o $@

$(FLOPPY_IMG): $(BOOT_BIN) $(BASIC_BIN) | $(BUILD_DIR)
	python3 -c 'from pathlib import Path; boot=Path("$(BOOT_BIN)").read_bytes(); basic=Path("$(BASIC_BIN)").read_bytes(); limit=5*512; assert len(basic)<=limit, f"BASIC binary too large ({len(basic)} bytes), max is {limit} bytes"; out=Path("$@"); out.write_bytes(boot+basic+bytes(limit-len(basic))); print(f"Wrote {out} ({out.stat().st_size} bytes)")'

image: $(FLOPPY_IMG) $(BIOS_BIN)

run: image $(EMULATOR)
	$(EMULATOR) $(BIOS_BIN) $(FLOPPY_IMG)

clean:
	rm -rf $(BUILD_DIR)
