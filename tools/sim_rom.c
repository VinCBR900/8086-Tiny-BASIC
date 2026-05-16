#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <ctype.h>
#include <limits.h>
#include "./cpu.h"

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

/*
 * sim_rom.c  --  Minimal 8088 CPU simulator wrapper for uBASIC
 * Leverages Mike Chamber's XTulator project cpu core.
 * Compright Vincent Crabtree 2026, MIT License
 *
 * Supports two input modes:
 *   1) ASM source
 *      Invokes tinyasm from the same directory as sim_wrap
 *      (tinyasm on Linux, tinyasm.exe on Windows), then parses
 *      the generated listing to locate getchar and putchar labels.
 *
 *   2) Raw binary image
 *      Requires explicit --getchar and --putchar addresses.
 *
 * Runtime behaviour:
 *
 *   * getchar is blocking and returns the received byte in AL.
 *   * putchar outputs the byte in AL.
 *   * Both are interecepted by simulator and not actually executed.
 *
 * Default load address:
 *   If --load is omitted:
 *
 *       load_address = 0x10000 - image_size
 *
 *   Examples:
 *       2 KiB image  -> 0xF800
 *       64 KiB image -> 0x0000
 *
 * Usage:
 *   sim_rom <program.asm>
 *            [--load ADDR]
 *            [--trace]
 *            [--cycles N]
 *
 *   sim_rom <program.bin>
 *            --getchar ADDR
 *            --putchar ADDR
 *            [--load ADDR]
 *            [--trace]
 *            [--cycles N]
 *
 * ASM mode notes:
 *   * Requires tinyasm/tinyasm.exe in same directory beside sim_rom.
 *   * Automatically extracts getchar/putchar addresses from listing output.
 *   * Fails if assembly errors occur or labels are missing.
 *
 * Binary mode notes:
 *   * Both --getchar and --putchar are mandatory.
 *   * Intended for pre-assembled ROM/RAM binary images.
 */

#define MEM_SIZE 65536u
static uint8_t mem[MEM_SIZE];
static uint16_t g_putchar_addr = 0;
static uint16_t g_getchar_addr = 0;

/*
 * cpu_read
 * What it does:
 *   Returns one byte from simulated 64 KiB memory.
 * Inputs:
 *   cpu  - CPU instance pointer (unused).
 *   addr - Address to read; masked to 16 bits.
 * Outputs:
 *   Returns byte value at memory[addr & 0xFFFF].
 */
uint8_t cpu_read(CPU_t *cpu, uint32_t addr) { (void)cpu; return mem[addr & 0xFFFFu]; }
/*
 * cpu_write
 * What it does:
 *   Stores one byte into simulated 64 KiB memory.
 * Inputs:
 *   cpu   - CPU instance pointer (unused).
 *   addr  - Address to write; masked to 16 bits.
 *   value - Byte value to store.
 * Outputs:
 *   None; writes value into memory[addr & 0xFFFF].
 */
void cpu_write(CPU_t *cpu, uint32_t addr, uint8_t value) { (void)cpu; mem[addr & 0xFFFFu] = value; }
/*
 * port_read
 * What it does:
 *   Handles 8-bit I/O port reads for the simulator stub.
 * Inputs:
 *   cpu  - CPU instance pointer (unused).
 *   port - Port number (unused).
 * Outputs:
 *   Returns 0xFF for all ports.
 */
uint8_t port_read(CPU_t *cpu, uint16_t port) { (void)cpu; (void)port; return 0xFF; }
/*
 * port_readw
 * What it does:
 *   Handles 16-bit I/O port reads using the 8-bit stub reader.
 * Inputs:
 *   cpu  - CPU instance pointer.
 *   port - Port number.
 * Outputs:
 *   Returns 16-bit value derived from port_read (currently 0x00FF).
 */
uint16_t port_readw(CPU_t *cpu, uint16_t port) { return (uint16_t)port_read(cpu, port); }
/*
 * port_write
 * What it does:
 *   Handles 8-bit I/O port writes for the simulator stub.
 * Inputs:
 *   cpu   - CPU instance pointer (unused).
 *   port  - Port number (unused).
 *   value - Value to write (unused).
 * Outputs:
 *   None; write is ignored.
 */
void port_write(CPU_t *cpu, uint16_t port, uint8_t value) { (void)cpu; (void)port; (void)value; }
/*
 * port_writew
 * What it does:
 *   Handles 16-bit I/O port writes by forwarding low byte to port_write.
 * Inputs:
 *   cpu   - CPU instance pointer.
 *   port  - Port number.
 *   value - 16-bit value; low 8 bits forwarded.
 * Outputs:
 *   None; write is ignored by underlying port_write stub.
 */
void port_writew(CPU_t *cpu, uint16_t port, uint16_t value) { port_write(cpu, port, (uint8_t)value); }

static volatile int nmi_pending = 0;
/*
 * sigint_handler
 * What it does:
 *   Handles Ctrl-C (SIGINT) by requesting an NMI on the next CPU cycle.
 * Inputs:
 *   sig - Signal number (unused).
 * Outputs:
 *   None; sets nmi_pending flag and re-registers handler.
 */
static void sigint_handler(int sig) { (void)sig; nmi_pending = 1; signal(SIGINT, sigint_handler); }

static int opt_trace = 0;
static int opt_maxcycles = 200000000;

/*
 * parse_u16
 * What it does:
 *   Parses a string as an unsigned 16-bit integer (base auto-detected).
 * Inputs:
 *   s   - String to parse.
 *   out - Destination for parsed value on success.
 * Outputs:
 *   Returns 0 on success and stores value in *out; returns -1 on error.
 */
static int parse_u16(const char *s, uint16_t *out) {
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (!s || !s[0] || (end && *end) || v > 0xFFFFul) return -1;
    *out = (uint16_t)v;
    return 0;
}

/*
 * is_asm_file
 * What it does:
 *   Checks whether a file path ends with ".asm" (case-insensitive).
 * Inputs:
 *   path - File path to inspect.
 * Outputs:
 *   Returns non-zero if path has .asm extension, otherwise 0.
 */
static int is_asm_file(const char *path) {
    const char *dot = strrchr(path, '.');
    return dot && strcasecmp(dot, ".asm") == 0;
}

/*
 * lower_copy
 * What it does:
 *   Copies source string into destination while lowercasing characters.
 * Inputs:
 *   dst    - Destination buffer.
 *   dst_sz - Destination buffer size.
 *   src    - Source C-string.
 * Outputs:
 *   None; writes a null-terminated lowercase copy into dst.
 */
static void lower_copy(char *dst, size_t dst_sz, const char *src) {
    size_t i = 0;
    if (!dst_sz) return;
    for (; src[i] && i + 1 < dst_sz; ++i) dst[i] = (char)tolower((unsigned char)src[i]);
    dst[i] = '\0';
}

/*
 * find_label_addr_in_lst
 * What it does:
 *   Scans a tinyasm listing file to locate a symbol address.
 * Inputs:
 *   lst_file - Path to listing file.
 *   label    - Target symbol name (case-insensitive).
 *   addr     - Destination for resolved 16-bit address.
 * Outputs:
 *   Returns 0 when found, 1 when not found, -1 on file-open failure.
 */
static int find_label_addr_in_lst(const char *lst_file, const char *label, uint16_t *addr) {
    FILE *f = fopen(lst_file, "r");
    if (!f) return -1;
    char needle[128];
    lower_copy(needle, sizeof(needle), label);

    char line[1024];
    while (fgets(line, sizeof(line), f)) {
        unsigned a = 0;
        char symbol[256] = {0}, symbol_l[256] = {0};
        /* Format 1: "ADDR  ... LABEL:" inline listing line */
        if (sscanf(line, "%x %255[^: ]", &a, symbol) == 2) {
            lower_copy(symbol_l, sizeof(symbol_l), symbol);
            if (strcmp(symbol_l, needle) == 0) {
                fclose(f);
                *addr = (uint16_t)(a & 0xFFFFu);
                return 0;
            }
        }
        /* Format 2: "LABEL  addr" symbol table line (tinyasm listing footer) */
        a = 0;
        memset(symbol, 0, sizeof(symbol));
        if (sscanf(line, "%255s %x", symbol, &a) == 2) {
            lower_copy(symbol_l, sizeof(symbol_l), symbol);
            if (strcmp(symbol_l, needle) == 0) {
                fclose(f);
                *addr = (uint16_t)(a & 0xFFFFu);
                return 0;
            }
        }
    }
    fclose(f);
    return 1;
}

/*
 * get_dir_from_argv0
 * What it does:
 *   Extracts the directory portion of argv[0].
 * Inputs:
 *   argv0  - Program path string.
 *   out    - Destination buffer for directory path.
 *   out_sz - Destination buffer size.
 * Outputs:
 *   Returns 0 on success; returns -1 on failure/truncation.
 */
static int get_dir_from_argv0(const char *argv0, char *out, size_t out_sz) {
    const char *slash1 = strrchr(argv0, '/');
    const char *slash2 = strrchr(argv0, '\\');
    const char *slash = slash1 > slash2 ? slash1 : slash2;
    if (!slash) return snprintf(out, out_sz, ".") > 0 ? 0 : -1;
    size_t n = (size_t)(slash - argv0);
    if (n + 1 > out_sz) return -1;
    memcpy(out, argv0, n);
    out[n] = '\0';
    return 0;
}

/*
 * run_tinyasm
 * What it does:
 *   Locates tinyasm beside the running executable and assembles ASM input.
 * Inputs:
 *   argv0      - Program path used to locate tinyasm/tinyasm.exe.
 *   asm_file   - ASM source file path.
 *   out_bin    - Destination buffer for generated .bin path.
 *   out_bin_sz - Size of out_bin buffer.
 *   out_lst    - Destination buffer for generated .lst path.
 *   out_lst_sz - Size of out_lst buffer.
 * Outputs:
 *   Returns 0 on success, -1 on path errors, -2 if tinyasm not found,
 *   -3 if assembler command fails.
 */
static int run_tinyasm(const char *argv0, const char *asm_file, char *out_bin, size_t out_bin_sz, char *out_lst, size_t out_lst_sz) {
    char dir[PATH_MAX];
    if (get_dir_from_argv0(argv0, dir, sizeof(dir)) != 0) return -1;

    char tinyasm[PATH_MAX], tinyasm_exe[PATH_MAX];
    snprintf(tinyasm, sizeof(tinyasm), "%s/tinyasm", dir);
    snprintf(tinyasm_exe, sizeof(tinyasm_exe), "%s/tinyasm.exe", dir);

    const char *tool = NULL;
    FILE *t = fopen(tinyasm, "rb");
    if (t) { tool = tinyasm; fclose(t); }
    if (!tool) {
        t = fopen(tinyasm_exe, "rb");
        if (t) { tool = tinyasm_exe; fclose(t); }
    }
    if (!tool) return -2;

    snprintf(out_bin, out_bin_sz, "%s.bin", asm_file);
    snprintf(out_lst, out_lst_sz, "%s.lst", asm_file);

    char cmd[4096];
    snprintf(cmd, sizeof(cmd), "\"%s\" -f bin \"%s\" -l \"%s\" -o \"%s\"", tool, asm_file, out_lst, out_bin);
    return system(cmd) == 0 ? 0 : -3;
}

/*
 * main
 * What it does:
 *   Entry point for the ROM simulator wrapper; parses CLI options, loads
 *   binary image (or assembles ASM), configures CPU state, and runs cycles
 *   while intercepting getchar/putchar labels.
 * Inputs:
 *   argc - Argument count.
 *   argv - Argument vector containing input file and options.
 * Outputs:
 *   Returns 0 on normal completion; non-zero on usage or runtime errors.
 */
int main(int argc, char **argv) {
    const char *input_file = NULL;
    int load_addr_set = 0;
    int getchar_set = 0;
    int putchar_set = 0;
    uint16_t load_addr = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--trace") == 0) opt_trace = 1;
        else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) opt_maxcycles = atoi(argv[++i]);
        else if (strcmp(argv[i], "--load") == 0 && i + 1 < argc) {
            if (parse_u16(argv[++i], &load_addr) != 0) { fprintf(stderr, "Invalid --load value\n"); return 1; }
            load_addr_set = 1;
        } else if (strcmp(argv[i], "--getchar") == 0 && i + 1 < argc) {
            if (parse_u16(argv[++i], &g_getchar_addr) != 0) { fprintf(stderr, "Invalid --getchar value\n"); return 1; }
            getchar_set = 1;
        } else if (strcmp(argv[i], "--putchar") == 0 && i + 1 < argc) {
            if (parse_u16(argv[++i], &g_putchar_addr) != 0) { fprintf(stderr, "Invalid --putchar value\n"); return 1; }
            putchar_set = 1;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            input_file = NULL;
        } else input_file = argv[i];
    }

    if (!input_file) {
        fprintf(stderr,
            "Usage:\n"
            "  %s <program.asm> [--load ADDR] [--trace] [--cycles N]\n"
            "  %s <program.bin> --getchar ADDR --putchar ADDR [--load ADDR] [--trace] [--cycles N]\n\n"
            "ASM mode:\n"
            "  * Requires tinyasm/tinyasm.exe beside sim_rom.\n"
            "  * Assembles source and parses listing for getchar/putchar addresses.\n"
            "  * Errors if assembler or labels are missing.\n\n"
            "Binary mode:\n"
            "  * Requires both --getchar and --putchar addresses.\n"
            "  * getchar is blocking and returns AL; putchar outputs AL.\n"
            "  * Default load address is 0x10000 - file_size (2 KiB -> 0xF800, 64 KiB -> 0x0000).\n",
            argv[0], argv[0]);
        return 1;
    }

    char bin_path[PATH_MAX], lst_path[PATH_MAX];
    const char *bin_file = input_file;
    if (is_asm_file(input_file)) {
        int asm_rc = run_tinyasm(argv[0], input_file, bin_path, sizeof(bin_path), lst_path, sizeof(lst_path));
        if (asm_rc == -2) { fprintf(stderr, "Error: tinyasm not found beside sim_rom.\n"); return 1; }
        if (asm_rc != 0) { fprintf(stderr, "Error: tinyasm failed assembling '%s'.\n", input_file); return 1; }

        if (find_label_addr_in_lst(lst_path, "getchar", &g_getchar_addr) != 0 ||
            find_label_addr_in_lst(lst_path, "putchar", &g_putchar_addr) != 0) {
            fprintf(stderr, "Error: could not find getchar/putchar in listing '%s'.\n", lst_path);
            return 1;
        }
        bin_file = bin_path;
    } else if (!getchar_set || !putchar_set) {
        fprintf(stderr, "Error: binary mode requires --getchar and --putchar addresses.\n");
        return 1;
    }

    FILE *f = fopen(bin_file, "rb");
    if (!f) { perror(bin_file); return 1; }
    if (fseek(f, 0, SEEK_END) != 0) { perror("fseek"); fclose(f); return 1; }
    long sz = ftell(f);
    if (sz <= 0 || sz > (long)MEM_SIZE) {
        fprintf(stderr, "Error: invalid program size %ld bytes (must be 1..65536).\n", sz);
        fclose(f);
        return 1;
    }
    rewind(f);

    size_t n = (size_t)sz;
    uint8_t *img = (uint8_t *)malloc(n);
    if (!img) { fprintf(stderr, "Error: out of memory.\n"); fclose(f); return 1; }
    if (fread(img, 1, n, f) != n) { perror("fread"); free(img); fclose(f); return 1; }
    fclose(f);

    if (!load_addr_set) load_addr = (uint16_t)(0x10000u - (unsigned)n);
    if ((unsigned)load_addr + (unsigned)n > MEM_SIZE) {
        fprintf(stderr, "Error: image (%zu bytes) does not fit at load address 0x%04X.\n", n, load_addr);
        free(img);
        return 1;
    }

    memset(mem, 0, sizeof(mem));
    memcpy(&mem[load_addr], img, n);
    free(img);

    CPU_t cpu;
    memset(&cpu, 0, sizeof(cpu));
    signal(SIGINT, sigint_handler);
    cpu_reset(&cpu);
    cpu.segregs[regcs] = cpu.segregs[regds] = cpu.segregs[reges] = cpu.segregs[regss] = 0;
    cpu.ip = load_addr;

    for (int cycles = 0; cycles < opt_maxcycles; cycles++) {
        if (nmi_pending) { nmi_pending = 0; cpu_intcall(&cpu, 2); }
        if (opt_trace) fprintf(stderr, "CS:IP=%04X:%04X AX=%04X\n", cpu.segregs[regcs], cpu.ip, cpu.regs.wordregs[regax]);
        if (cpu.hltstate) break;

        if (cpu.segregs[regcs] == 0) {
            if (cpu.ip == g_putchar_addr) {
                putchar((int)cpu.regs.byteregs[regal]); fflush(stdout);
                uint16_t sp = cpu.regs.wordregs[regsp];
                cpu.ip = cpu_readw(&cpu, ((uint32_t)cpu.segregs[regss] << 4) + sp);
                cpu.regs.wordregs[regsp] = sp + 2;
                continue;
            }
            if (cpu.ip == g_getchar_addr) {
                int c = getchar();
                if (c == EOF) { cpu.hltstate = 1; break; }
                cpu.regs.byteregs[regal] = (uint8_t)c;
                uint16_t sp = cpu.regs.wordregs[regsp];
                cpu.ip = cpu_readw(&cpu, ((uint32_t)cpu.segregs[regss] << 4) + sp);
                cpu.regs.wordregs[regsp] = sp + 2;
                continue;
            }
        }
        cpu_exec(&cpu, 1);
    }

    return 0;
}
