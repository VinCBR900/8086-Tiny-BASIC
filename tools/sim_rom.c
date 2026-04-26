/*
 * sim_rom.c  --  uBASIC 8088 ROM simulator
 *
 * Wraps the XTulator 8086 CPU core (cpu.c, Mike Chambers) with the
 * memory/IO model of the target hardware.
 *
 * Memory decode (A12 selects bank):
 *   A12=0  2KB SRAM  phys 0x00000-0x00FFF  (uses 0x0000-0x07FF, mirrored)
 *   A12=1  2KB ROM   phys 0x0F000-0xFFFFF  (uses 0xF800-0xFFFF, mirrored)
 *
 * Reset: CS=0xFFFF IP=0x0000 -> phys 0xFFFF0 -> ROM[0x07F0] -> JMP start
 *
 * I/O (Intel 8755 stub):
 *   0x00 PORT_A: read=getchar(), write=putchar()
 *   0x02 DDR_A:  ignored
 *
 * NMI: send SIGINT (Ctrl-C) to inject INT 2 into the running CPU.
 *
 * Build:  gcc -O2 -Wall -o sim_rom sim_rom.c cpu.c
 * Usage:  sim_rom <rom.bin> [--trace] [--cycles N]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include "cpu.h"

/* ── Memory ──────────────────────────────────────────────────────────────── */
#define RAM_SIZE    2048
#define ROM_SIZE    2048

static uint8_t ram[RAM_SIZE];
static uint8_t rom[ROM_SIZE];

/* Memory decode:
 *   ROM when physical addr >= 0xF800 — covers both:
 *     CS=0xF800 code/data access (phys 0xF8000+)
 *     DS=0 access to high offsets >= 0xF800 (phys 0x0F800-0x0FFFF)
 *   RAM otherwise (phys 0x0000-0x0F7FF)
 * ROM index = addr & (ROM_SIZE-1) = addr & 0x7FF.
 * This matches hardware where the top 2KB of the 16-bit address space
 * wraps to the ROM regardless of segment. */
uint8_t cpu_read(CPU_t *cpu, uint32_t addr) {
    (void)cpu;
    addr &= 0xFFFFFu;
    if (addr >= 0xF800u)
        return rom[addr & (ROM_SIZE - 1)];
    return ram[addr & (RAM_SIZE - 1)];
}

/* cpu_readw provided by cpu.c (calls our cpu_read) */

void cpu_write(CPU_t *cpu, uint32_t addr, uint8_t value) {
    (void)cpu;
    addr &= 0xFFFFFu;
    if (addr < 0xF800u)             /* ROM writes silently dropped */
        ram[addr & (RAM_SIZE - 1)] = value;
}

/* cpu_writew provided by cpu.c (calls our cpu_write) */

/* ── I/O: 8755 Port A stub ───────────────────────────────────────────────── */
/* port_read: intercept at input_key handles real reads.
 * Return idle state (RX=1 = bit1 high) for any stray IN instructions. */
uint8_t port_read(CPU_t *cpu, uint16_t port) {
    (void)cpu; (void)port;
    return 0xFF;   /* all lines idle/high */
}

uint16_t port_readw(CPU_t *cpu, uint16_t port) {
    return (uint16_t)port_read(cpu, port);
}

/* port_write: all I/O is intercepted at output/input_key entry points.
 * Raw OUT instructions (serial init, DDR setup) are silently discarded. */
void port_write(CPU_t *cpu, uint16_t port, uint8_t value) {
    (void)cpu; (void)port; (void)value;
}

void port_writew(CPU_t *cpu, uint16_t port, uint16_t value) {
    port_write(cpu, port, (uint8_t)value);
}

/* ── NMI via SIGINT ──────────────────────────────────────────────────────── */
static CPU_t          *g_cpu        = NULL;
static volatile int    nmi_pending  = 0;

static void sigint_handler(int sig) {
    (void)sig;
    nmi_pending = 1;
    /* Reinstall so repeated Ctrl-C keeps working */
    signal(SIGINT, sigint_handler);
}

/* ── Options ─────────────────────────────────────────────────────────────── */
static int opt_trace     = 0;
static int opt_maxcycles = 200000000;   /* ~2 seconds of emulated 8088 time */

/* ── Main ────────────────────────────────────────────────────────────────── */
int main(int argc, char **argv) {
    const char *romfile = NULL;

    for (int i = 1; i < argc; i++) {
        if      (strcmp(argv[i], "--trace") == 0)
            opt_trace = 1;
        else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc)
            opt_maxcycles = atoi(argv[++i]);
        else
            romfile = argv[i];
    }

    if (!romfile) {
        fprintf(stderr,
            "uBASIC 8088 ROM Simulator (XTulator CPU core by Mike Chambers)\n"
            "\n"
            "Usage: %s <rom.bin> [--trace] [--cycles N]\n"
            "\n"
            "  rom.bin    2048-byte ROM image built with:\n"
            "               tinyasm -f bin -dROM=1 uBASIC8088.asm -o rom.bin\n"
            "  --trace    print CS:IP + registers before each instruction\n"
            "  --cycles N stop after N instructions (default %d)\n"
            "\n"
            "Memory map (A12 decode):\n"
            "  A12=0  2KB SRAM   0x0000-0x07FF\n"
            "  A12=1  2KB ROM    0xF800-0xFFFF (mirrored to all A12=1 addresses)\n"
            "\n"
            "Reset: CS=0xFFFF IP=0x0000 -> phys 0xFFFF0 -> ROM[0x07F0] -> JMP start\n"
            "NMI:   Ctrl-C injects INT 2 (break) into the interpreter\n",
            argv[0], opt_maxcycles);
        return 1;
    }

    /* Load ROM */
    FILE *f = fopen(romfile, "rb");
    if (!f) { perror(romfile); return 1; }
    size_t n = fread(rom, 1, ROM_SIZE, f);
    fclose(f);
    if (n != ROM_SIZE) {
        fprintf(stderr,
            "Error: ROM must be exactly %d bytes (got %zu).\n"
            "  Build with: tinyasm -f bin -dROM=1 uBASIC8088.asm -o rom.bin\n",
            ROM_SIZE, (size_t)n);
        return 1;
    }

    /* Initialise */
    CPU_t cpu;
    memset(&cpu, 0, sizeof(cpu));
    memset(ram,  0, sizeof(ram));
    g_cpu = &cpu;
    signal(SIGINT, sigint_handler);

    /* Override reset state: use flat single-segment model (CS=DS=ES=SS=0).
     * The ROM is mapped at 0xF800-0xFFFF. All listing addresses are in
     * the range 0xF800-0xFFFF and are used directly as IP offsets.
     * PHYS(CS=0, IP=0xF9E6) = 0xF9E6 → ROM[0x01E6]. Correct.
     * The real 8086 reset (CS=0xFFFF) would need offsets, not abs addresses. */
    cpu_reset(&cpu);
    cpu.segregs[regcs] = 0x0000;   /* flat: all segments = 0 */
    cpu.segregs[regds] = 0x0000;
    cpu.segregs[reges] = 0x0000;
    cpu.segregs[regss] = 0x0000;
    cpu.ip = 0xF800;               /* jump straight to start label */

    /* Diagnostic banner */
    uint32_t reset_phys = (uint32_t)cpu.ip;   /* CS=0, phys=IP */
    uint8_t  first_op   = cpu_read(&cpu, reset_phys);
    fprintf(stderr,
        "[sim_rom] ROM: %s (%zu bytes)\n"
        "[sim_rom] Start: CS=%04X IP=%04X -> phys 0x%05X -> opcode 0x%02X (%s)\n",
        romfile, n,
        cpu.segregs[regcs], cpu.ip,
        reset_phys, first_op,
        first_op == 0xFA ? "CLI - OK" :
        first_op == 0x31 ? "XOR - OK" : "unexpected - check ROM");

    /* ── Execution loop ────────────────────────────────────────────────── */
    int cycles = 0;
    while (cycles < opt_maxcycles) {

        /* NMI from SIGINT */
        if (nmi_pending) {
            nmi_pending = 0;
            fprintf(stderr, "\n[sim_rom] NMI injected (Ctrl-C) at CS:IP=%04X:%04X\n",
                    cpu.segregs[regcs], cpu.ip);
            cpu_intcall(&cpu, 2);
        }

        /* Trace */
        if (opt_trace) {
            uint32_t phys = ((uint32_t)cpu.segregs[regcs] << 4) + cpu.ip;
            fprintf(stderr,
                "[%05X] CS:%04X IP:%04X  "
                "AX:%04X BX:%04X CX:%04X DX:%04X  "
                "SP:%04X BP:%04X SI:%04X DI:%04X\n",
                phys, cpu.segregs[regcs], cpu.ip,
                cpu.regs.wordregs[regax], cpu.regs.wordregs[regbx],
                cpu.regs.wordregs[regcx], cpu.regs.wordregs[regdx],
                cpu.regs.wordregs[regsp], cpu.regs.wordregs[regbp],
                cpu.regs.wordregs[regsi], cpu.regs.wordregs[regdi]);
        }

        if (cpu.hltstate)
            break;

        /* Intercept ROM serial I/O at the routine entry points.
         * output (0xF800:0xFC32): AL holds the char to transmit.
         *   Putchar AL, then RET (pop return address, skip bitbang body).
         * input_key (0xF800:0xFC9E): getchar into AL, then RET.
         * These intercepts make simulation fast and correct without
         * needing to reconstruct characters from the bitbang bit stream.
         */
        /* Intercept at full listing addresses (CS=0, so IP=listing_addr) */
        #define OUTPUT_CS    0x0000u
        #define OUTPUT_IP    0xFC35u   /* listing addr of output: */
        #define INPUT_KEY_IP 0xFCA1u   /* listing addr of input_key: */
        if (cpu.segregs[regcs] == OUTPUT_CS) {
            if (cpu.ip == OUTPUT_IP) {
                /* output: putchar(AL), then RET */
                uint8_t ch = cpu.regs.byteregs[regal];
                putchar((int)ch);
                fflush(stdout);
                uint16_t sp = cpu.regs.wordregs[regsp];
                cpu.ip = cpu_readw(&cpu, ((uint32_t)cpu.segregs[regss]<<4)+sp);
                cpu.regs.wordregs[regsp] = sp + 2;
                cycles++; continue;
            }
            if (cpu.ip == INPUT_KEY_IP) {
                /* input_key: getchar into AL, then RET.
                 * On EOF halt the simulator cleanly. */
                int c = getchar();
                if (c == EOF) { cpu.hltstate = 1; break; }
                cpu.regs.byteregs[regal] = (uint8_t)c;
                uint16_t sp = cpu.regs.wordregs[regsp];
                cpu.ip = cpu_readw(&cpu, ((uint32_t)cpu.segregs[regss]<<4)+sp);
                cpu.regs.wordregs[regsp] = sp + 2;
                cycles++; continue;
            }
        }

        cpu_exec(&cpu, 1);
        cycles++;
    }

    fprintf(stderr,
        "\n[sim_rom] Stopped: %d instructions, CS:IP=%04X:%04X, hlt=%d\n",
        cycles, cpu.segregs[regcs], cpu.ip, cpu.hltstate);
    return 0;
}
