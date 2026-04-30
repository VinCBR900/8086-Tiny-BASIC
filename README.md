# 8086-Tiny-BASIC
**uBASIC 8088** - Tiny BASIC for Toy 8086/88 Embedded systems

Target: <=2048 bytes code ROM, 2048/4096 bytes RAM.

Copyright (c) 2026 Vincent Crabtree, MIT License

Originally inspired by the 1980s BYTE magazine article "Ease into 16 bit Computing" by Steve Ciarcia.

Credit to [Oscar Toledo's bootBASIC](https://github.com/VinCBR900/bootBASIC), which is leveraged here.

## Functionality
**Statements**: `PRINT` `IF` .. `THEN` `GOTO` `GOSUB` `RETURN` `FOR` .. `TO` .. `STEP` `NEXT` `LET` `INPUT` `REM` `END` `RUN` `LIST` `NEW` `POKE` `FREE` `HELP`

**Expressions**: `+ - * / % = < > <= >= <>` unary- `CHR$(n)` `PEEK(addr)` `USR(addr)` A-Z

**Numbers**: signed 16-bit (`-32768..32767`)

**Multi-Statement**: colon separator `:`

**Errors**: `?0` syntax, `?1` undef line, `?2` div/zero, `?3` out of memory, `?4` bad variable, `?5` `RETURN` without `GOSUB`, `?6` `NEXT` without `FOR`, `?B` break into program (ROM version)

## Build instructions (Variant 3 only)

This repository now supports **only Variant 3** (standalone ROM target).

### Prerequisites
- C compiler (`cc`/`gcc`/`clang`)
- `make`

The build uses the bundled `tools/tinyasm.c` assembler and no longer depends on NASM/YASM.

### Build ROM image
```bash
make rom
```
Output: `build/uBASIC_rom.bin` (2KB ROM image).

### Build simulator
```bash
make sim
```
Output: `build/sim_rom`.

### Run ROM in simulator
```bash
make rom-run
```

### Clean artifacts
```bash
make clean
```

## Variant 3 memory/hardware notes
- ORIGIN   = `0xF800` (ROM occupies `0xF800-0xFFFF`, reset stub at `0xFFF0`)
- RAM_BASE = `0x0000` (RAM `0x0000-0x07FF`)
- RAM_SIZE = `2048` (2KB)
- STACK    = `0x0800` (top of RAM)
- I/O      = bitbang UART via 8755 Port A

Simulator memory model (`tools/sim_rom.c`):
- `CS=DS=ES=SS=0x0000` (flat single-segment)
- `addr >= 0xF800 -> ROM[addr & 0x7FF]`
- `addr <  0xF800 -> RAM[addr & 0x7FF]`

## License

Copyright (c) 2026 Vincent Crabtree

**MIT License**
