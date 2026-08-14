# =============================================================================
# Makefile  --  8088 Tiny BASIC: native tools/ROMs + browser (Wasm) build (v2)
#
# Layout assumed:
#   tools/            sim_rom.c, cpu.c, cpu.h, tinyasm.c, ins.c, ...
#   web/              index8088.html (browser front end)
#   <root>            uBASIC8088.asm, miniBASIC8088.asm, this Makefile
#
# Targets:
#   make tools          build native tinyasm (+ sim_rom, for local testing)
#   make roms           assemble uBASIC8088.bin / miniBASIC8088.bin (native)
#   make native-smoke   plain gcc sanity build of sim_rom.c + cpu.c
#   make web-assets     assemble the same two ROMs into web/assets/, plus
#                        web/assets/rom-io.json (getchar/putchar addresses
#                        extracted from each ROM's .lst, consumed by
#                        web/index8088.html so it never hardcodes them)
#   make web-dist       web-assets + emcc build -> dist/ (sim_rom.js/.wasm,
#                        index.html, assets/) ready for GitHub Pages
#   make clean          remove all build output
#
# History:
#   v2 (Aug 2026) -- web-dist now also `cp -R`s web/assets/ into dist/assets/
#     as literal static files, alongside the existing --preload-file step.
#     --preload-file only bakes web/assets/ into the WASM's virtual MEMFS
#     (for sim8088_select()'s own fopen() calls at runtime); it does not
#     produce real files under dist/, so index8088.html's plain
#     fetch('assets/rom-io.json') was 404ing on GitHub Pages (served the
#     Pages 404 HTML page back where JSON was expected -- reported after
#     the first live deploy).
#   v1 (Aug 2026) -- initial Makefile for this project. Mirrors the sibling
#     65C02 project's Makefile/emscripten-pages.yml shape (native-smoke +
#     web-assets + web-dist targets, same emcc flag set), adapted for this
#     project's binary-only browser build (no in-browser assembler -- see
#     tools/sim_rom.c v1.5's header) and its hostcall-based (not memory-
#     mapped-port) GETCH/PUTCH, whose addresses are per-ROM function
#     addresses rather than a fixed hardware register -- hence the
#     rom-io.json generation step, which the 65C02 project doesn't need.
# =============================================================================

CC      ?= gcc
CFLAGS  ?= -std=c11
EMCC    ?= emcc

TOOLS_DIR      := tools
WEB_DIR        := web
WEB_ASSETS_DIR := $(WEB_DIR)/assets
DIST_DIR       := dist
BUILD_DIR      := build

TINYASM   := $(TOOLS_DIR)/tinyasm
SIM_ROM   := $(TOOLS_DIR)/sim_rom
SIM_SRC   := $(TOOLS_DIR)/sim_rom.c
CPU_SRC   := $(TOOLS_DIR)/cpu.c

ROM_NAMES := uBASIC8088 miniBASIC8088
ROMS      := $(addsuffix .bin,$(ROM_NAMES))
WEB_ROMS  := $(addprefix $(WEB_ASSETS_DIR)/,$(ROMS))
ROM_IO_JSON := $(WEB_ASSETS_DIR)/rom-io.json

WASM_EXPORTS := "['_sim8088_select','_sim8088_input','_sim8088_run_chunk','_sim8088_cycles','_sim8088_set_io_addrs','_sim8088_set_load_addr','_sim8088_set_entry_addr','_sim8088_set_maxcycles']"

.PHONY: all tools roms native-smoke web-assets web-dist clean clean-web-assets clean-web-dist

all: roms

# --- native tools -----------------------------------------------------------

$(TINYASM): $(TOOLS_DIR)/tinyasm.c $(TOOLS_DIR)/ins.c
	$(CC) $(CFLAGS) -O2 -o $@ $(TOOLS_DIR)/tinyasm.c $(TOOLS_DIR)/ins.c -lm

$(SIM_ROM): $(SIM_SRC) $(CPU_SRC) $(TOOLS_DIR)/cpu.h $(TOOLS_DIR)/cpuconf.h
	$(CC) $(CFLAGS) -O2 -o $@ $(SIM_SRC) $(CPU_SRC) -lm

tools: $(TINYASM) $(SIM_ROM)

native-smoke:
	$(CC) $(CFLAGS) -O2 -Wall -Wextra -o /tmp/sim_rom-native-smoke $(SIM_SRC) $(CPU_SRC) -lm

# --- native ROM images (full assembled span; see tools/sim_rom.c's
#     load-address auto-detection -- ROM-only or full-image, whichever
#     each source naturally assembles to; no range flag needed/available) --

roms: $(ROMS)

uBASIC8088.bin: uBASIC8088.asm $(TINYASM)
	$(TINYASM) -f bin $< -l $(BUILD_DIR)/uBASIC8088.lst -o $@

miniBASIC8088.bin: miniBASIC8088.asm $(TINYASM)
	$(TINYASM) -f bin $< -l $(BUILD_DIR)/miniBASIC8088.lst -o $@

$(BUILD_DIR) $(WEB_ASSETS_DIR):
	mkdir -p $@

# both native ROM recipes drop their .lst in $(BUILD_DIR); declare the order-
# only dependency so `make roms` works from a clean tree
uBASIC8088.bin miniBASIC8088.bin: | $(BUILD_DIR)

# --- web assets: same two ROMs, into web/assets/, plus their I/O addresses -

$(WEB_ASSETS_DIR)/uBASIC8088.bin: uBASIC8088.asm $(TINYASM) | $(WEB_ASSETS_DIR) $(BUILD_DIR)
	$(TINYASM) -f bin $< -l $(BUILD_DIR)/uBASIC8088-web.lst -o $@

$(WEB_ASSETS_DIR)/miniBASIC8088.bin: miniBASIC8088.asm $(TINYASM) | $(WEB_ASSETS_DIR) $(BUILD_DIR)
	$(TINYASM) -f bin $< -l $(BUILD_DIR)/miniBASIC8088-web.lst -o $@

# Extracts GETCHAR/PUTCHAR label addresses from each ROM's .lst symbol
# table (format: "<line#>  LABEL  <hex addr>") so web/index8088.html never
# hardcodes them -- they move whenever the .asm source does.
$(ROM_IO_JSON): $(WEB_ROMS)
	@{ \
		echo "{"; \
		first=1; \
		for name in $(ROM_NAMES); do \
			lst="$(BUILD_DIR)/$$name-web.lst"; \
			gc=$$(awk 'toupper($$1)=="GETCHAR"{a=$$2} END{print a}' "$$lst"); \
			pc=$$(awk 'toupper($$1)=="PUTCHAR"{a=$$2} END{print a}' "$$lst"); \
			if [ -z "$$gc" ] || [ -z "$$pc" ]; then \
				echo "error: could not find GETCHAR/PUTCHAR in $$lst" >&2; exit 1; \
			fi; \
			[ $$first -eq 0 ] && echo ","; \
			first=0; \
			printf '  "%s.bin": { "getchar": "0x%s", "putchar": "0x%s" }' "$$name" "$$gc" "$$pc"; \
		done; \
		echo ""; \
		echo "}"; \
	} > $@

web-assets: $(WEB_ROMS) $(ROM_IO_JSON)

# --- browser (Wasm) build ----------------------------------------------------

web-dist: web-assets
	mkdir -p $(DIST_DIR)
	cp $(WEB_DIR)/index8088.html $(DIST_DIR)/index.html
	# Real static copy for the browser's own fetch('assets/rom-io.json') --
	# --preload-file below only bakes web/assets/ into the WASM's virtual
	# MEMFS (for sim8088_select()'s fopen() calls at runtime), it does NOT
	# put literal files under dist/, so without this GitHub Pages 404s on
	# a plain fetch() and the JS gets an HTML error page back where it
	# expected JSON.
	rm -rf $(DIST_DIR)/assets
	cp -R $(WEB_ASSETS_DIR) $(DIST_DIR)/assets
	$(EMCC) $(SIM_SRC) $(CPU_SRC) \
		-O2 \
		-s MODULARIZE=0 \
		-s EXPORTED_RUNTIME_METHODS="['cwrap']" \
		-s EXPORTED_FUNCTIONS=$(WASM_EXPORTS) \
		-s ALLOW_MEMORY_GROWTH=1 \
		-s FORCE_FILESYSTEM=1 \
		-s INVOKE_RUN=0 \
		-s EXIT_RUNTIME=0 \
		--preload-file $(WEB_ASSETS_DIR)@assets \
		-o $(DIST_DIR)/sim_rom.js

# --- cleanup ------------------------------------------------------------------

clean-web-assets:
	rm -rf $(WEB_ASSETS_DIR)

clean-web-dist:
	rm -rf $(DIST_DIR)

clean: clean-web-assets clean-web-dist
	rm -f $(ROMS) $(TINYASM) $(SIM_ROM)
	rm -rf $(BUILD_DIR)
