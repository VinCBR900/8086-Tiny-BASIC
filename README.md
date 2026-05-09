# 8086-Tiny-BASIC
**uBASIC 8088** - Tiny BASIC for Toy 8086/88 Embedded systems

Target: <=2048 bytes code ROM, 2048/4096 bytes RAM.

Copyright (c) 2026 Vincent Crabtree, MIT License

Originally inspired by the 1980s BYTE magazine article "Ease into 16 bit Computing" by Steve Ciarcia.

Credit to [Oscar Toledo's bootBASIC](https://github.com/VinCBR900/bootBASIC), which is leveraged here.

## Functionality
**Statements**: `PRINT [TAB(spaces)] [;] [CHR$(n)]`, `IF` .. `THEN`, `GOTO`, `GOSUB` `RETURN`, `FOR` .. `TO` .. `STEP` `NEXT`, `LET`, `INPUT`, `REM`, `END`, `RUN`, `LIST [start,end]`, `NEW`, `POKE`, `FREE`, `HELP`, `OUT`

**Expressions**:  
  - Arithmetic `+` `-` `*` `/` `% (Mod)`; Boolean `&` `|`; Relational `<` `>` `<=` `>=` `<>`, unary`-` and `= (assignment)` 
  - functions `PEEK(addr)`, `USR(addr)`, `IN(io)`, `ABS(val)`, `RND(limit)` 
  - variables `A`..`Z`

**Numbers**: signed 16-bit (`-32768..32767`)

**Multi-Statement**: colon separator `:` (Does not support `GOSUB`/`RETURN` or `FOR`/`NEXT` on same line)

**Errors**: `?0` syntax, `?1` undef line, `?2` div/zero, `?3` out of memory, `?4` bad variable, `?5` `RETURN` without `GOSUB`, `?6` `NEXT` without `FOR`, `?B` break into program (ROM version)

## Build instructions 

Standalone ROM target is built with makefile, otherwise copy/paste into 8bitworkshop to run the embedded demo program.

http://8bitworkshop.com/v3.12.1/?redir.html?platform=x86&githubURL=https%3A%2F%2Fgithub.com%2FVinCBR900%2F8086-Tiny-BASIC&file=uBASIC8088.asm

### Prerequisites
- C compiler (`cc`/`gcc`/`clang`)
- `make`

The build uses the bundled `tools/tinyasm.c` assembler which is a subset clone of NASM.

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

## Memory/hardware notes
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
