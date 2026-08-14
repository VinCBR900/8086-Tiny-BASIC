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
 * sim_rom.c  --  Minimal 8088 CPU simulator wrapper for uBASIC (v1.4)
 * Leverages Mike Chamber's XTulator project cpu core.
 * Compright Vincent Crabtree 2026, MIT License
 *
 * ---------------------------------------------------------------------------
 * VERSION HISTORY
 * ---------------------------------------------------------------------------
 * v1.4 : Fixed getchar/putchar hostcall trap silently never firing for ROMs
 *          whose reset stub far-jumps to a non-zero CS that is nonetheless
 *          address-equivalent to CS=0 under this sim's flat/16-bit-masked
 *          memory model (e.g. CS=0xF000:IP=0x0000, as used by a real-
 *          hardware-style reset vector) -- the trap's gate literally
 *          checked `cpu.segregs[regcs] == 0`, which only matched a ROM
 *          whose reset stub happened to jump within CS=0x0000 (flat
 *          model; e.g. uBASIC8088.asm's `dw ORIGIN / dw 0x0000` far jump)
 *          and missed the equally-valid `dw 0x0000 / dw 0xF000` style
 *          (e.g. miniBASIC8088.asm), leaving GETCH/PUTCH un-intercepted:
 *          the ROM fell through into its real, port-stubbed bitbang UART
 *          code instead, producing no output and hanging on input.
 *          Replaced with cs_is_flat_zero(), which accepts any CS whose
 *          segment base (CS<<4) wraps to 0 mod 0x10000 -- i.e. any of the
 *          16 segments (0x0000, 0x1000, ... 0xF000) that are physically
 *          equivalent to CS=0 in this memory model, not just CS==0
 *          itself. No change to --load/--entry semantics or ASM/binary
 *          mode handling.
 * v1.3 : Fixed reset entry point. cpu.ip was hardcoded to load_addr, i.e.
 *          "wherever the image was placed" rather than "where an 8088
 *          actually starts fetching after reset" -- correct only by
 *          coincidence for a 2 KiB image (load_addr happens to equal the
 *          ROM's start label there) and wrong for a full 64 KiB image
 *          (load_addr = 0x0000, landing in the VARS/pad area instead of
 *          the reset stub). Added --entry ADDR, defaulting to 0xFFF0 --
 *          an 8088 resets to phys 0xFFFF0, which this sim's flat 16-bit
 *          model (CS forced to 0 at start, see below) truncates to
 *          0xFFF0, matching where the ROM source's own "org 0xFFF0"
 *          reset stub actually sits. --load is unaffected: it still only
 *          controls where the image bytes are placed in memory.
 * v1.2 : Fixed ASM-mode invocation, which was completely non-functional:
 *          - Makefile built tinyasm from a nonexistent "tools/ins.c" (the
 *            file is just "ins.c" at the project root) and hardcoded a ROM
 *            source filename ("uBASIC8088.asm") that doesn't exist in this
 *            project -- so `make` could not even produce a tinyasm binary
 *            for sim_rom to find beside itself, regardless of anything in
 *            this file. That was the actual root cause of "ASM mode not
 *            working"; both are fixed in the Makefile, not here.
 *        Portability fixes for Win11/TCC (64-bit) as well as Linux/gcc:
 *          - is_asm_file() used strcasecmp(), which is POSIX-only (not
 *            declared under -std=c11 without _POSIX_C_SOURCE on glibc, and
 *            entirely absent from the Windows/MSVCRT headers TCC targets
 *            on Windows). Replaced with a local, portable case-insensitive
 *            compare.
 *          - run_tinyasm()'s system() call quoted the tool path and each
 *            argument individually ("tool" "arg1" "arg2" ...). On Windows,
 *            cmd.exe (which system() shells out to) applies a special rule
 *            when a command line both starts and ends with a double quote,
 *            and can misparse a path containing spaces (e.g. anything
 *            under "C:\Users\<name>\...") as a result. Fixed by wrapping
 *            the entire command line in one more pair of quotes on
 *            _WIN32 builds, which is the standard workaround; no change
 *            on non-Windows builds.
 * v1.1 : Added three debug features for inspecting MBF4 float-library bugs
 *        without rebuilding instrumented test files each time:
 *          --trace LO:HI / --trace-range LO:HI : scope tracing to an
 *            address range; --trace alone still traces everything.
 *          Widened per-line trace dump from AX-only to full register/flag
 *            set (AX,BX,CX,DX,SI,DI,BP,SP,CS,DS,ES,SS,CF,ZF,SF,OF,AF,PF).
 *          --break-at ADDR[:N] / --break-continue : halt (or just log and
 *            continue) on the Nth time IP reaches ADDR; ADDR may be a
 *            numeric address or a label resolved from the .lst file
 *            (ASM mode only); :0 means "every hit". Repeatable, max 8.
 *          --watch ADDR:LEN : dump LEN bytes at ADDR alongside every trace
 *            line and every breakpoint hit; ADDR may be numeric or a label.
 *            Repeatable, max 8.
 *        Label resolution for --trace-range/--break-at/--watch is deferred
 *        until after tinyasm produces the .lst file, so labels work in ASM
 *        mode; binary mode only accepts numeric addresses for these flags.
 * v1.0 : Initial release. ASM/binary input modes, getchar/putchar
 *        interception, --load/--trace/--cycles.
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
 * Default entry point (cpu.ip at power-on):
 *   If --entry is omitted, cpu.ip = 0xFFF0 always, regardless of image
 *   size or --load -- this models the 8088 reset vector (phys 0xFFFF0)
 *   under this simulator's flat, CS-forced-to-0 memory model. --load only
 *   controls where the image is placed; --entry only controls where
 *   execution starts. They are independent.
 *
 * Usage:
 *   sim_rom <program.asm>
 *            [--load ADDR] [--entry ADDR]
 *            [--trace [LO:HI]]
 *            [--trace-range LO:HI]
 *            [--break-at ADDR[:N]] [--break-continue]
 *            [--watch ADDR:LEN]
 *            [--cycles N]
 *
 *   sim_rom <program.bin>
 *            --getchar ADDR
 *            --putchar ADDR
 *            [--load ADDR] [--entry ADDR]
 *            [--trace [LO:HI]]
 *            [--trace-range LO:HI]
 *            [--break-at ADDR[:N]] [--break-continue]
 *            [--watch ADDR:LEN]
 *            [--cycles N]
 *
 * ASM mode notes:
 *   * Requires tinyasm/tinyasm.exe in same directory beside sim_rom.
 *   * Automatically extracts getchar/putchar addresses from listing output.
 *   * Fails if assembly errors occur or labels are missing.
 *   * --break-at/--watch/--trace-range accept label names (resolved via the
 *     generated .lst file) as well as numeric addresses.
 *
 * Binary mode notes:
 *   * Both --getchar and --putchar are mandatory.
 *   * Intended for pre-assembled ROM/RAM binary images.
 *   * --break-at/--watch/--trace-range accept numeric addresses only (no
 *     .lst file exists to resolve labels against).
 *
 * Debug flag notes:
 *   * --trace                : trace every instruction (full reg/flag dump).
 *   * --trace LO:HI          : trace only while IP is within [LO,HI].
 *   * --trace-range LO:HI    : same as above, as a standalone flag.
 *   * --break-at ADDR[:N]    : dump regs/flags/watches on the Nth hit of
 *                              ADDR (default N=1), then halt; N=0 dumps on
 *                              every hit and implies --break-continue.
 *   * --break-continue       : with --break-at, keep running after the dump
 *                              instead of halting.
 *   * --watch ADDR:LEN       : dump LEN bytes at ADDR on every --trace line
 *                              and every --break-at hit. Repeatable.
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

/* --- new debug-feature state --- */
static int opt_trace_range_set = 0;
static uint16_t opt_trace_lo = 0, opt_trace_hi = 0xFFFF;
static char opt_trace_range_raw[64] = {0}; /* deferred until after assembly (labels need .lst) */

#define MAX_BREAKPOINTS 8
static int opt_break_count = 0;
static char opt_break_raw[MAX_BREAKPOINTS][64];
static uint16_t opt_break_addr[MAX_BREAKPOINTS];
static uint32_t opt_break_hitcount[MAX_BREAKPOINTS];      /* target hit number (1 = first hit) */
static uint32_t opt_break_hitcount_seen[MAX_BREAKPOINTS]; /* hits observed so far, per breakpoint */
static int opt_break_continue = 0;                         /* 0 = halt after dump, 1 = keep running */

#define MAX_WATCHES 8
static int opt_watch_count = 0;
static char opt_watch_raw[MAX_WATCHES][64];
static uint16_t opt_watch_addr[MAX_WATCHES];
static uint16_t opt_watch_len[MAX_WATCHES];

static void dump_regs(FILE *out, CPU_t *cpu) {
    uint16_t flags = makeflagsword(cpu);
    fprintf(out,
        "  AX=%04X BX=%04X CX=%04X DX=%04X SI=%04X DI=%04X BP=%04X SP=%04X\n"
        "  CS=%04X DS=%04X ES=%04X SS=%04X  CF=%d ZF=%d SF=%d OF=%d AF=%d PF=%d  FLAGS=%04X\n",
        cpu->regs.wordregs[regax], cpu->regs.wordregs[regbx],
        cpu->regs.wordregs[regcx], cpu->regs.wordregs[regdx],
        cpu->regs.wordregs[regsi], cpu->regs.wordregs[regdi],
        cpu->regs.wordregs[regbp], cpu->regs.wordregs[regsp],
        cpu->segregs[regcs], cpu->segregs[regds], cpu->segregs[reges], cpu->segregs[regss],
        cpu->cf, cpu->zf, cpu->sf, cpu->of, cpu->af, cpu->pf, flags);
}

static void dump_watch(FILE *out, uint16_t addr, uint16_t len) {
    fprintf(out, "  [%04X:%u] =", addr, (unsigned)len);
    for (uint16_t i = 0; i < len; i++) fprintf(out, " %02X", mem[(uint16_t)(addr + i)]);
    fprintf(out, "\n");
}

static void dump_all_watches(FILE *out) {
    for (int i = 0; i < opt_watch_count; i++) dump_watch(out, opt_watch_addr[i], opt_watch_len[i]);
}

static int parse_u16(const char *s, uint16_t *out) {
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (!s || !s[0] || (end && *end) || v > 0xFFFFul) return -1;
    *out = (uint16_t)v;
    return 0;
}

/* Portable case-insensitive compare: strcasecmp is POSIX-only (absent on
 * Windows/MSVCRT, which TCC targets on Windows) -- avoid it entirely
 * rather than relying on a platform-specific alternative. */
static int ci_str_equal(const char *a, const char *b) {
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++; b++;
    }
    return *a == '\0' && *b == '\0';
}

static int is_asm_file(const char *path) {
    const char *dot = strrchr(path, '.');
    return dot && ci_str_equal(dot, ".asm");
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

/* Resolve a token to an address: tries a plain numeric literal first
 * (0x.., decimal, etc.); if that fails and lst_file is non-NULL, tries it
 * as a label name via find_label_addr_in_lst. */
static int resolve_addr_token(const char *tok, const char *lst_file, uint16_t *addr) {
    if (parse_u16(tok, addr) == 0) return 0;
    if (lst_file && find_label_addr_in_lst(lst_file, tok, addr) == 0) return 0;
    return -1;
}

/* Same idea, but for "TOKEN" or "TOKEN:N" pairs where TOKEN may be a label
 * or a numeric address, and N is a plain decimal/hex count. */
static int resolve_addr_pair_token(const char *s, const char *lst_file, uint16_t *addr, uint32_t *second, uint32_t def_second) {
    char buf[64];
    size_t i = 0;
    for (; s[i] && s[i] != ':' && i + 1 < sizeof(buf); i++) buf[i] = s[i];
    buf[i] = '\0';
    if (resolve_addr_token(buf, lst_file, addr) != 0) return -1;
    if (s[i] == ':') {
        char *end = NULL;
        unsigned long v = strtoul(s + i + 1, &end, 0);
        if (!end || *end) return -1;
        *second = (uint32_t)v;
    } else {
        *second = def_second;
    }
    return 0;
}

/* =============================================================================
 * CS_IS_FLAT_ZERO  true if CS's segment base wraps to 0 in this sim's
 *   16-bit-masked flat memory model (segbase(CS) = CS<<4; mem[] is indexed
 *   mod 0x10000 -- see cpu_read()/cpu_write() above). Any CS that is a
 *   multiple of 0x1000 (0x0000, 0x1000, ... 0xF000) satisfies this, since
 *   CS<<4 is then an exact multiple of 0x10000 and contributes no offset,
 *   so code/data addressing behaves identically to CS=0x0000.
 *   Used to gate the getchar/putchar hostcall trap in main()'s run loop:
 *   some ROMs reset with a flat CS=0x0000, others with a real-hardware-
 *   style far jump to e.g. CS=0xF000:IP=0x0000 -- both address-equivalent
 *   here, but v1.3 and earlier only recognised the literal CS==0 case,
 *   silently disabling GETCH/PUTCH interception for any ROM using the
 *   latter convention (the ROM would fall through into its real, port-
 *   stubbed I/O code instead, producing no output and hanging on input).
 * Inputs  : cs -- CPU's current CS segment register value
 * Outputs : 1 if CS's segment base is address-equivalent to CS=0, else 0
 * Clobbers: none
 * ============================================================================= */
static int cs_is_flat_zero(uint16_t cs) {
    return (((uint32_t)cs << 4) & 0xFFFFu) == 0;
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
#ifdef _WIN32
    /* cmd.exe (which system() invokes on Windows) special-cases a command
     * line that both starts and ends with a double quote, and can mis-
     * parse it when the path contains spaces. Wrapping the whole line in
     * one extra pair of quotes is the standard workaround. Not needed
     * (and not applied) on POSIX, where the shell has no such quirk. */
    snprintf(cmd, sizeof(cmd), "\"\"%s\" -f bin \"%s\" -l \"%s\" -o \"%s\"\"", tool, asm_file, out_lst, out_bin);
#else
    snprintf(cmd, sizeof(cmd), "\"%s\" -f bin \"%s\" -l \"%s\" -o \"%s\"", tool, asm_file, out_lst, out_bin);
#endif
    return system(cmd) == 0 ? 0 : -3;
}

int main(int argc, char **argv) {
    const char *input_file = NULL;
    int load_addr_set = 0;
    int entry_set = 0;
    int getchar_set = 0;
    int putchar_set = 0;
    uint16_t load_addr = 0;
    uint16_t entry_addr = 0xFFF0;   /* 8088 reset: phys 0xFFFF0, truncated to
                                        this sim's flat 16-bit model -> 0xFFF0.
                                        Overridable via --entry for debugging. */

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--trace") == 0) {
            opt_trace = 1;
            /* Optional inline range: --trace LO:HI (only if next arg contains ':' --
             * otherwise leave full-range tracing as before). */
            if (i + 1 < argc && strchr(argv[i + 1], ':') && argv[i + 1][0] != '-') {
                snprintf(opt_trace_range_raw, sizeof(opt_trace_range_raw), "%s", argv[++i]);
                opt_trace_range_set = 1;
            }
        }
        else if (strcmp(argv[i], "--trace-range") == 0 && i + 1 < argc) {
            snprintf(opt_trace_range_raw, sizeof(opt_trace_range_raw), "%s", argv[++i]);
            opt_trace_range_set = 1; opt_trace = 1;
        }
        else if (strcmp(argv[i], "--break-at") == 0 && i + 1 < argc) {
            if (opt_break_count >= MAX_BREAKPOINTS) { fprintf(stderr, "Too many --break-at (max %d)\n", MAX_BREAKPOINTS); return 1; }
            snprintf(opt_break_raw[opt_break_count], sizeof(opt_break_raw[0]), "%s", argv[++i]);
            opt_break_count++;
        }
        else if (strcmp(argv[i], "--break-continue") == 0) opt_break_continue = 1;
        else if (strcmp(argv[i], "--watch") == 0 && i + 1 < argc) {
            if (opt_watch_count >= MAX_WATCHES) { fprintf(stderr, "Too many --watch (max %d)\n", MAX_WATCHES); return 1; }
            snprintf(opt_watch_raw[opt_watch_count], sizeof(opt_watch_raw[0]), "%s", argv[++i]);
            opt_watch_count++;
        }
        else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) opt_maxcycles = atoi(argv[++i]);
        else if (strcmp(argv[i], "--load") == 0 && i + 1 < argc) {
            if (parse_u16(argv[++i], &load_addr) != 0) { fprintf(stderr, "Invalid --load value\n"); return 1; }
            load_addr_set = 1;
        } else if (strcmp(argv[i], "--entry") == 0 && i + 1 < argc) {
            if (parse_u16(argv[++i], &entry_addr) != 0) { fprintf(stderr, "Invalid --entry value\n"); return 1; }
            entry_set = 1;
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
            "  %s <program.asm> [--load ADDR] [--entry ADDR] [--trace[ LO:HI]] [--cycles N]\n"
            "                    [--trace-range LO:HI] [--break-at ADDR[:N]] [--break-continue]\n"
            "                    [--watch ADDR:LEN]\n"
            "  %s <program.bin> --getchar ADDR --putchar ADDR [above flags also apply]\n\n"
            "ASM mode:\n"
            "  * Requires tinyasm/tinyasm.exe beside sim_rom.\n"
            "  * Assembles source and parses listing for getchar/putchar addresses.\n"
            "  * Errors if assembler or labels are missing.\n\n"
            "Binary mode:\n"
            "  * Requires both --getchar and --putchar addresses.\n"
            "  * getchar is blocking and returns AL; putchar outputs AL.\n"
            "  * Default load address is 0x10000 - file_size (2 KiB -> 0xF800, 64 KiB -> 0x0000).\n"
            "  * Default entry point (cpu.ip) is 0xFFF0 (the 8088 reset vector, phys 0xFFFF0,\n"
            "    truncated to this sim's flat/CS=0 model), independent of --load. Override\n"
            "    with --entry ADDR.\n\n"
            "Debug flags:\n"
            "  --trace                  Trace every instruction: CS:IP and full register/flag dump.\n"
            "  --trace LO:HI            Same, but only while IP is within [LO,HI].\n"
            "  --trace-range LO:HI      Equivalent to '--trace LO:HI' as a separate flag.\n"
            "  --break-at ADDR[:N]      On the Nth time IP==ADDR (default N=1), dump full\n"
            "                           registers/flags (and any --watch ranges), then halt.\n"
            "                           N=0 means dump on every hit (implies --break-continue).\n"
            "                           Repeatable (max %d).\n"
            "  --break-continue         With --break-at, dump but keep running instead of halting.\n"
            "  --watch ADDR:LEN         Dump LEN bytes at ADDR alongside every --trace line and\n"
            "                           every --break-at hit. Repeatable (max %d).\n",
            argv[0], argv[0], MAX_BREAKPOINTS, MAX_WATCHES);
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

    /* Resolve any deferred debug-flag tokens now that lst_path (if any) exists,
     * so labels like '--break-at fa_addorsub' work in ASM mode. Binary mode
     * has no .lst, so only numeric addresses resolve (lst_file = NULL). */
    const char *lst_for_resolve = is_asm_file(input_file) ? lst_path : NULL;

    if (opt_trace_range_set) {
        uint32_t hi;
        if (resolve_addr_pair_token(opt_trace_range_raw, lst_for_resolve, &opt_trace_lo, &hi, 0xFFFF) != 0 || hi > 0xFFFF) {
            fprintf(stderr, "Invalid/unresolved --trace range '%s' (use LO:HI, numeric or label)\n", opt_trace_range_raw);
            return 1;
        }
        opt_trace_hi = (uint16_t)hi;
    }
    for (int b = 0; b < opt_break_count; b++) {
        uint32_t hitn;
        if (resolve_addr_pair_token(opt_break_raw[b], lst_for_resolve, &opt_break_addr[b], &hitn, 1) != 0) {
            fprintf(stderr, "Invalid/unresolved --break-at '%s' (use ADDR[:N] or LABEL[:N])\n", opt_break_raw[b]);
            return 1;
        }
        opt_break_hitcount[b] = hitn;
    }
    for (int w = 0; w < opt_watch_count; w++) {
        uint32_t len;
        if (resolve_addr_pair_token(opt_watch_raw[w], lst_for_resolve, &opt_watch_addr[w], &len, 1) != 0) {
            fprintf(stderr, "Invalid/unresolved --watch '%s' (use ADDR:LEN or LABEL:LEN)\n", opt_watch_raw[w]);
            return 1;
        }
        opt_watch_len[w] = (uint16_t)len;
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
    cpu.ip = entry_addr;   /* defaults to the 8088 reset vector (0xFFF0); see entry_addr init above */
    (void)entry_set;

    for (int cycles = 0; cycles < opt_maxcycles; cycles++) {
        if (nmi_pending) { nmi_pending = 0; cpu_intcall(&cpu, 2); }

        if (opt_trace && (!opt_trace_range_set || (cpu.ip >= opt_trace_lo && cpu.ip <= opt_trace_hi))) {
            fprintf(stderr, "CS:IP=%04X:%04X\n", cpu.segregs[regcs], cpu.ip);
            dump_regs(stderr, &cpu);
            dump_all_watches(stderr);
        }

        for (int b = 0; b < opt_break_count; b++) {
            if (cpu.ip == opt_break_addr[b]) {
                uint32_t hit = ++opt_break_hitcount_seen[b];
                /* hitcount 0 means "every hit"; otherwise stop only on the Nth */
                if (opt_break_hitcount[b] == 0 || hit == opt_break_hitcount[b]) {
                    fprintf(stderr, "--- break-at 0x%04X (hit #%u) ---\n", opt_break_addr[b], (unsigned)hit);
                    fprintf(stderr, "CS:IP=%04X:%04X\n", cpu.segregs[regcs], cpu.ip);
                    dump_regs(stderr, &cpu);
                    dump_all_watches(stderr);
                    if (!opt_break_continue && opt_break_hitcount[b] != 0) { cpu.hltstate = 1; }
                }
            }
        }

        if (cpu.hltstate) break;

        if (cs_is_flat_zero(cpu.segregs[regcs])) {
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
