#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <ctype.h>
#include <limits.h>
#include "cpu.h"

/*
 * sim_rom.c  --  uBASIC 8088 simulator with ASM/bin loaders
 *
 * Input modes:
 *   1) ASM source: invokes tinyasm from the same directory as sim_rom
 *      (supports tinyasm on Linux and tinyasm.exe on Windows), then parses
 *      the generated listing to find getchar and putchar routine addresses.
 *   2) Binary image: requires --getchar and --putchar addresses.
 *
 * In both modes getchar is blocking and returns byte in AL, putchar writes AL.
 *
 * Default load address when --load is omitted:
 *   load = 0x10000 - image_size
 * Examples: 2 KiB -> 0xF800, 64 KiB -> 0x0000.
 */

#define MEM_SIZE 65536u
static uint8_t mem[MEM_SIZE];
static uint16_t g_putchar_addr = 0;
static uint16_t g_getchar_addr = 0;

uint8_t cpu_read(CPU_t *cpu, uint32_t addr) { (void)cpu; return mem[addr & 0xFFFFu]; }
void cpu_write(CPU_t *cpu, uint32_t addr, uint8_t value) { (void)cpu; mem[addr & 0xFFFFu] = value; }
uint8_t port_read(CPU_t *cpu, uint16_t port) { (void)cpu; (void)port; return 0xFF; }
uint16_t port_readw(CPU_t *cpu, uint16_t port) { return (uint16_t)port_read(cpu, port); }
void port_write(CPU_t *cpu, uint16_t port, uint8_t value) { (void)cpu; (void)port; (void)value; }
void port_writew(CPU_t *cpu, uint16_t port, uint16_t value) { port_write(cpu, port, (uint8_t)value); }

static volatile int nmi_pending = 0;
static void sigint_handler(int sig) { (void)sig; nmi_pending = 1; signal(SIGINT, sigint_handler); }

static int opt_trace = 0;
static int opt_maxcycles = 200000000;

static int parse_u16(const char *s, uint16_t *out) {
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (!s || !s[0] || (end && *end) || v > 0xFFFFul) return -1;
    *out = (uint16_t)v;
    return 0;
}

static int is_asm_file(const char *path) {
    const char *dot = strrchr(path, '.');
    return dot && strcasecmp(dot, ".asm") == 0;
}

static void lower_copy(char *dst, size_t dst_sz, const char *src) {
    size_t i = 0;
    if (!dst_sz) return;
    for (; src[i] && i + 1 < dst_sz; ++i) dst[i] = (char)tolower((unsigned char)src[i]);
    dst[i] = '\0';
}

static int find_label_addr_in_lst(const char *lst_file, const char *label, uint16_t *addr) {
    FILE *f = fopen(lst_file, "r");
    if (!f) return -1;
    char needle[128];
    lower_copy(needle, sizeof(needle), label);

    char line[1024];
    while (fgets(line, sizeof(line), f)) {
        unsigned a = 0;
        char symbol[256] = {0}, symbol_l[256] = {0};
        if (sscanf(line, "%x %255[^: ]", &a, symbol) == 2) {
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
