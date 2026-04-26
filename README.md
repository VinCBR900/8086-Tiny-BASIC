# 8086-Tiny-BASIC
**uBASIC 8088** - Tiny BASIC for Toy 8086/88 Embedded systems

Target: <=2048 bytes code ROM, 2048/4096 bytes RAM.

Copyright (c) 2026 Vincent Crabtree, MIT License

Originally Inspired by the 1980's BYTE magazine article "Ease into 16 bit Computing" by Steve Ciarcia. 

Credit to [Oscar Toledo's bootBASIC](https://github.com/VinCBR900/bootBASIC) which is leveraged here. 

### Functionality
**Statements**: `PRINT` `IF` .. `THEN` `GOTO` `GOSUB` `RETURN` `FOR` .. `TO` .. `STEP` `NEXT` `LET` `INPUT` `REM` `END` `RUN` `LIST` `NEW` `POKE` `FREE` `HELP`

**Expressions**: + - * / %  = < > <= >= <>  unary-  `CHR$(n)` `PEEK(addr)` `USR(addr)` A-Z

**Numbers**: signed 16-bit (-32768..32767)

**Multi-Statement**: colon separator ':' **NOTE**: Does not support `FOR` `NEXT` or `GOSUB` `RETURN` on same line. 

**Errors**: `?0` syntax,  `?1` undef line,  `?2` div/zero,  `?3` out of memory,  `?4` bad variable,  `?5` `RETURN` without `GOSUB`, `?6` `NEXT` without `FOR`, `?B` Break into program (ROM version) 

## BUILD INSTRUCTIONS

You can play with the Interpreter by copying and pasting into 8bitworkshop with the target simulation set as x86 FREEDOS.
https://8bitworkshop.com/v3.12.1/?platform=x86&file=uBASIC8088.asm

The source has assembly time conditions to include a demo program. Type `LIST` or view it and `RUN` to execute.

Two simulators were used to develop - **[8086Tiny](https://github.com/alblue/8086tiny)** for initial testing and **[XTulator](https://github.com/mikechambers84/XTulator)** CPU core used for Embedded ROM target testing.

**Prep**
Assembler: Oscar Toledo's **[tinyasm](https://github.com/nanochess/tinyasm/)** or NASM (both produce identical output for this file).

### Variant 1: 8086tiny batch-test (boot sector, BIOS I/O) 

**Assemble**:
```   
     tinyasm -f bin uBASIC8088.asm -o uBASIC8088.bin
   or:
     nasm -f bin uBASIC8088.asm -o uBASIC8088.bin
```

 **Create floppy image & Simulate**
 
 `bootsect.asm` loads 5 sectors to 0x0000:0x7E00:
```
     nasm -f bin bootsect.asm -o boot.bin
     python3 -c "
       boot  = open('boot.bin','rb').read()
       basic = open('uBASIC8088.bin','rb').read()
       img   = boot + basic + bytes(2560 - len(basic))
       open('floppy.img','wb').write(img)"
```
   Run under 8086tiny (compile with `-DNO_GRAPHICS` for stdin/stdout I/O):
  ```
     gcc -O2 -DNO_GRAPHICS -o 8086tiny 8086tiny.c
     ./8086tiny bios.bin floppy.img
```
**Memory map**:
- ORIGIN   = 0x7E00  (code loaded here by boot sector)
- RAM_BASE = 0x1000  (variables, program store)
- RAM_SIZE = 4096    (4KB)
- I/O      = BIOS INT 10h (display), INT 16h (keyboard)

###  Variant 2: 8bitworkshop online IDE (yasm assembler) 

   Open the file directly in https://8bitworkshop.com (8086 mode).
   
   The 8bitworkshop assembler **yasm** defines __YASM_MAJOR__ which selects this variant automatically.
   A pre-loaded Mandelbrot showcase program is embedded in the image.

   **Memory map**:
- ORIGIN   = 0xF800  (forced origin)
- RAM_BASE = 0x0000 
- RAM_SIZE = 4096    (4KB)
- I/O      = BIOS INT 10h / INT 16h (emulated by 8bitworkshop)

### Variant 3: Standalone ROM target (for real hardware) 

**Assemble**:
```
     tinyasm -f bin -dROM=1 uBASIC8088.asm -o uBASIC_rom.bin
```
he output is exactly 2048 bytes, ready to burn to a 2KB EPROM/EEPROM.

**Hardware design**:
- CPU    : Intel 8088 @ 5 MHz (or compatible)
- ROM    : 2KB at physical 0xF800-0xFFFF  (A12=1 selects ROM)
- RAM    : 2KB at physical 0x0000-0x07FF  (A12=0 selects RAM)
- Serial : Intel 8755 MMIO
  - Port A (0x00) bit 0 = TX (output), bit 1 = RX (input)
  - DDR A (0x02) configured in init: 0xFD = all outputs except RX
  - Baud rate: 4800 baud @ 5 MHz (BAUD=60 loop constant)
- Reset  : 8086 reset vector at 0xFFFF0 -> FAR JMP to 0xF800:0x0000 (start)
- INT 0  : Divide-by-zero -> prints ?2 and re-enters interpreter
- INT 2  : NMI (break key) -> prints ?B and re-enters interpreter

**Simulate**
Uses **XTulator* project **cpu.c** by Mike Chambers + stubs:
```
    gcc -O2 -o sim_rom sim_rom.c cpu.c   # cpu.c, cpu.h, cpuconf.h from XTulator
    ./sim_rom uBASIC_rom.bin              # run ROM image
    ./sim_rom uBASIC_rom.bin --trace      # trace every instruction
    echo "PRINT 2+2" | ./sim_rom uBASIC_rom.bin --cycles 5000000
```
Simulator memory model (sim_rom.c):
- CS=DS=ES=SS=0x0000 (flat single-segment)
- addr >= 0xF800 -> ROM[addr & 0x7FF]   (top 2KB of address space)
- addr <  0xF800 -> RAM[addr & 0x7FF]   (bottom of address space)
- I/O: output/input_key intercepted at entry points; **bitbang bypassed** i.e. not emulated so YMMV

**Memory map**:
- ORIGIN   = 0xF800  (ROM occupies 0xF800-0xFFFF, reset stub at 0xFFF0)
- RAM_BASE = 0x0000  (RAM 0x0000-0x07FF)
- RAM_SIZE = 2048    (2KB)
- STACK    = 0x0800  (top of RAM)
- I/O      = bitbang UART via 8755 Port A;

---

## Licence

Copyright (c) 2026 Vincent Crabtree

**MIT License**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

