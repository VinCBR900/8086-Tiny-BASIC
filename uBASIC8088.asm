; =============================================================================
; uBASIC 8088  v1.6.0
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Tiny BASIC for single-segment 8088/8086 systems.
; Target: <=2048 bytes code ROM, 4096 bytes RAM.
;
; Credit to Oscar Toledo for his bootBASIC inspiration
;
; Statements : PRINT IF..THEN GOTO GOSUB RETURN FOR..TO..STEP NEXT LET INPUT REM 
; 		OUT END RUN LIST NEW POKE FREE HELP
; Expressions: + - * / %  = < > <= >= <>  unary-  CHR$(n) PEEK(addr) USR(addr) IN(I/O) A-Z
; Numbers    : signed 16-bit (-32768..32767)
; Multi-stmt : colon separator ':' (dont use for/next or gosub/return on same line)
; Errors     : ?0 syntax  ?1 undef line  ?2 div/zero  ?3 out of mem  
;		?4 bad variable  ?5 return without gosub ?B Break into Program (ROM only)
;
; =============================================================================
; BUILD INSTRUCTIONS
; =============================================================================
;
; Assembler: Oscar Toledo's tinyasm  (https://github.com/nanochess/tinyasm)
;   or NASM (both produce identical output for this file).
;
; --- Variant 1: 8086tiny batch-test (boot sector, BIOS I/O) -----------------
;
;   Assemble:
;     tinyasm -f bin uBASIC8088.asm -o uBASIC8088.bin
;   or:
;     nasm -f bin uBASIC8088.asm -o uBASIC8088.bin
;
;   Create floppy image (bootsect.asm loads 5 sectors to 0x0000:0x7E00):
;     nasm -f bin bootsect.asm -o boot.bin
;     python3 -c "
;       boot  = open('boot.bin','rb').read()
;       basic = open('uBASIC8088.bin','rb').read()
;       img   = boot + basic + bytes(2560 - len(basic))
;       open('floppy.img','wb').write(img)"
;
;   Run under 8086tiny (compile with -DNO_GRAPHICS for stdio):
;     gcc -O2 -DNO_GRAPHICS -o 8086tiny 8086tiny.c
;     ./8086tiny bios.bin floppy.img
;
;   Memory map:
;     ORIGIN   = 0x7E00  (code loaded here by boot sector)
;     RAM_BASE = 0x1000  (variables, program store)
;     RAM_SIZE = 4096    (4KB)
;     I/O      = BIOS INT 10h (display), INT 16h (keyboard)
;
; --- Variant 2: 8bitworkshop online IDE (YASM/yasm assembler) ----------------
;
;   Open the file directly in https://8bitworkshop.com (8086 mode).
;   yasm defines __YASM_MAJOR__ which selects this variant automatically.
;   A pre-loaded Mandelbrot showcase program is embedded in the image.
;
;   Memory map:
;     ORIGIN   = 0xF800  (8bitworkshop segment base)
;     RAM_BASE = 0x0000
;     RAM_SIZE = 4096    (4KB)
;     I/O      = BIOS INT 10h / INT 16h (emulated by 8bitworkshop)
;
; --- Variant 3: Standalone ROM target (real hardware) -----------------------
;
;   Assemble:
;     tinyasm -f bin -dROM=1 uBASIC8088.asm -o uBASIC_rom.bin
;
;   The output is exactly 2048 bytes, ready to burn to a 2KB EPROM/EEPROM.
;
;   Hardware design:
;     CPU    : Intel 8088 @ 5 MHz (or compatible)
;     ROM    : 2KB at physical 0xF800-0xFFFF  (A12=1 selects ROM)
;     RAM    : 2KB at physical 0x0000-0x07FF  (A12=0 selects RAM)
;     Serial : Intel 8755 MMIO
;                Port A (0x00) bit 0 = TX (output), bit 1 = RX (input)
;                DDR A (0x02) configured in init: 0xFD = all outputs except RX
;              Baud rate: 4800 baud @ 5 MHz (BAUD=60 loop constant)
;     Reset  : 8086 reset vector at 0xFFFF0 -> FAR JMP to 0xF800:0x0000 (start)
;     INT 0  : Divide-by-zero -> prints ?2 and re-enters interpreter
;     INT 2  : NMI (break key) -> prints ?B and re-enters interpreter
;
;   Memory map:
;     ORIGIN   = 0xF800  (ROM occupies 0xF800-0xFFFF, reset stub at 0xFFF0)
;     RAM_BASE = 0x0000  (RAM 0x0000-0x07FF)
;     RAM_SIZE = 2048    (2KB)
;     STACK    = 0x0800  (top of RAM)
;     I/O      = bitbang UART via 8755 Port A
;
;   Simulate (requires XTulator cpu.c by Mike Chambers + stubs):
;     gcc -O2 -o sim_rom sim_rom.c cpu.c   # cpu.c, cpu.h, cpuconf.h from XTulator
;     ./sim_rom uBASIC_rom.bin              # run ROM image
;     ./sim_rom uBASIC_rom.bin --trace      # trace every instruction
;     echo "PRINT 2+2" | ./sim_rom uBASIC_rom.bin --cycles 5000000
;
;   Simulator memory model (sim_rom.c):
;     CS=DS=ES=SS=0x0000 (flat single-segment)
;     addr >= 0xF800 -> ROM[addr & 0x7FF]   (top 2KB of address space)
;     addr <  0xF800 -> RAM[addr & 0x7FF]   (bottom of address space)
;     I/O: output/input_key intercepted at entry points; bitbang bypassed.
;
; =============================================================================
;
; Segment model: CS=DS=ES=SS (single segment, flat).
;   Boot sector (bootsect.asm) loads 5 sectors to 0x7E00 and jumps there.
;   All absolute addresses in RAM (VARS, PROGRAM, etc.) are segment-0 offsets.
;
; Line store: <linenum_lo> <linenum_hi> <tokenized body> <CR>
;   Token bytes 0x80..0x93 replace keywords; printable ASCII and CR pass through.
;   Stmt tokens 0x80..0x90 (NUM_TOKENS=17): PRINT IF GOTO LIST RUN NEW INPUT
;     REM END LET POKE FREE HELP GOSUB RETURN FOR NEXT
;   Sub-keyword tokens 0x91..0x93: THEN TO STEP
;   String literals (between quotes) and REM bodies stored verbatim.
;
; History:
;   v1.6.0 (2026-04-26)  Added missing IN/OUT commands, refactored to make space.
;   v1.5.0 (2026-04-23)  ROM target integration + showcase fixes:
;     - %ifdef ordering: ROM first so -dROM=1 wins over __YASM_MAJOR__
;     - RAM_SIZE conditional: 2KB for ROM (2KB RAM), 4KB for others
;     - ROM serial init: 8755 DDR setup + TX idle before interpreter start
;     - ROM interrupt vectors: CS=0xF800 written at init
;     - divide_error: prints ?2 then re-enters interpreter (do_error_hw)
;     - nmi_handler: prints ?B then re-enters interpreter (do_error_hw)
;     - do_error_hw: resets SP then calls do_error (safe from any context)
;     - bdly: bit-period delay routine for bitbang serial (BAUD=60@5MHz)
;     - output/input_key: conditional on %ifdef ROM (bitbang vs BIOS)
;     - Reset vector stub at 0xFFF0: near JMP to start, padded 0xFF
;     - Showcase: rep stosw CX fixed to (PROGRAM-RAM_BASE)/2 -- stops
;       exactly before PROGRAM so first showcase byte is not zeroed
;     - 8bitworkshop PROG_END init points AT sentinel (consistent with do_new)
;   v1.4.1 (2026-04-23) No new features, Reorg prepping for ROM version
;   v1.4.0 (2026-04-19)  FOR/NEXT with optional STEP:
;     - FOR <var>=<start> TO <end> [STEP <step>]
;     - NEXT <var>: increments var, loops if condition holds, unwinds nesting
;     - Dedicated 4-entry FOR stack at FOR_SP(0x1090)/FOR_STK(0x1092)
;     - PROGRAM moved 0x1090->0x10B2; costs 34 bytes of program store
;     - Tokens: stmt TK_FOR=0x8F TK_NEXT=0x90; sub-kw TK_THEN=0x91 TK_TO=0x92 TK_STEP=0x93
;     - Bugs fixed: kw_match clobbers DI->save var_ptr in INS_TMP; kw_match
;       clobbers AX(AH)->set default step AFTER kw_match; input_number clobbers
;       CX->push/pop CX around STEP expr; kw_next had wrong terminator T_X->T_T
;     - do_poke: push/pop DI around value expr (same kw_match/DI issue)
;     - Error ?6 = NEXT without FOR
;     - Incorporates uploaded tweaks: dp_str in dl_kw_lp, new_line tail-call
;   v1.3.1 (2026-04-18)  Bugfix release:
;     - LIST: dl_eol now outputs CR+LF (was LF only)
;     - Showcase (8bitworkshop): db data converted to tokenised form;
;       saves 195 bytes of program store and makes LIST correct
;     - Version string updated to v1.3.1
;   v1.3.0 (2026-04-17)  Tokenizer + line-editor refactor:
;     - Lines stored tokenized: keywords -> 0x80..0x8F (saves ~19% program store)
;     - LIST detokenizes on output (keywords printed uppercase)
;     - stmt: token fast-path (0x80+ -> direct dispatch, no kw_match loop)
;     - tokenize(): in-place scan of IBUF, string/REM passthrough
;     - insline refactored: drop second find_line call and BP stack frame
;     - deline refactored: use PROG_END directly, not find_program_end
;     - editln: tokenize IBUF before insline
;   v1.2.0 (2026-04-17)  Add GOSUB/RETURN:
;     - Dedicated 8-entry GOSUB stack at 0x107E (GOSUB_SP) + 0x1080 (GOSUB_STK)
;     - PROGRAM store moved 0x107E -> 0x1090 (costs 18 bytes program RAM)
;     - New error ?5 = RETURN without GOSUB
;     - GOSUB: saves RUN_NEXT on GOSUB_STK, then behaves like GOTO
;     - RETURN: restores RUN_NEXT from GOSUB_STK, resumes after GOSUB line
;   v1.1.0 (2026-04-17)  Bug fixes and do_new improvement:
;     - expr: add xor ax,ax before test ah,dl so rel_t (dec ax) yields
;       0xFFFF; AX held right operand, giving wrong relational results.
;     - do_new: rep stosw to clear entire program store, not just sentinel;
;       prevents stale data confusing walk_lines after future LOAD/SAVE.
;     - equ "0".."4" changed to equ 0x30..0x34 (tinyasm compatibility).
;     - dp_str_eol dead label removed.
;   v1.0.9 (2026-04-12)  Expand IBUF 34->64 bytes; input limit 32->62 chars.
;                         Fixes silent truncation of long lines (e.g. nested
;                         IF with multi-digit numbers). Embed Mandelbrot
;                         showcase as pre-loaded ROM program (COM build).
;   v1.0.8 (2026-04-10)  Fix editln: push/pop BX around find_line+deline;
;                         walk_lines clobbered BX (body pointer), causing
;                         ins_copy to read from 0x0000 (IVT) instead of IBUF.
;   v1.0.7 (2026-04-10)  Fix modulo: IDIV clobbers DX (DL=operator) with
;                         remainder; save DL to BL before IDIV, compare BL.
;   v1.0.6 (2026-04-10)  Fix rel_lt and rel_gt plain-op paths: pop ax before
;                         call rel_setup was missing (stack imbalance + wrong BX).
;   v1.0.5 (2026-04-10)  Fix expression register preservation:
;     expr: push/pop AX (not BX) to save left operand across peek;
;     rel_xx: pop AX (restore left) before call rel_setup;
;     rel_setup: reverted to push ax/call expr_add/pop bx;
;     ea_do: push/pop DX to preserve operator DL across call expr1.
;   v1.0.4 (2026-04-10)  Fix expr: save expr_add result in BX before relational
;                         peek; mov al,[si] was overwriting AL (low byte of AX),
;                         corrupting all expression results to 0x0D (CR).
;   v1.0.3 (2026-04-10)  Fix expr2: reload AL from [si] after kw_match chain
;                         (kw_match clobbers AL; stale value used for digit/var test)
;   v1.0.2 (2026-04-10)  Fix for 8086tiny / boot sector execution:
;     - ORG changed from 0x0100 to 0x7E00 to match boot sector load address.
;       All dw pointers in st_tab and kw tables now resolve correctly.
;     - Init: CX increased from 26 to 48 words so rep stosw zeroes all RAM
;       control variables (RUNNING, CURLN, IBUF, PROG_END, RUN_NEXT, INS_TMP).
;     - Segment setup: push cs / pop ds/es/ss added to ensure CS=DS=ES=SS=0.
;       Boot sector leaves DS=ES=SS=0 already; this is defensive.
;     - expr: fixed dead 'je rel_gt' (was 'cmp al,<' / 'je rel_lt' / 'je rel_gt').
;       Now correctly: 'cmp al,>' / 'je rel_gt' so '>' operators work.
;     - do_input: added variable-letter validation before get_var_addr (was absent).
;   v1.0.1 (2026-04-09)  Added push cs/pop ds; changed string routine to dp_str.
;   v1.0.0 (2026-04-09)  First release. Clean 8088 port from uBASIC 65c02 v17.0.
; =============================================================================

        	cpu 8086
                
	; Configure origin and RAM base for target platform.
	; ROM must be first: -dROM=1 must override yasm's __YASM_MAJOR__.
%ifdef ROM                      ; Standalone 2KB ROM at 0xF800, 2KB RAM at 0x0000
ORIGIN:         equ 0xF800
RAM_BASE:       equ 0x0000
RAM_SIZE:       equ 2048        ; 2KB RAM (A12=0 selects RAM, A12=1 selects ROM)
%else
%ifdef __YASM_MAJOR__           ; 8bitworkshop: yasm defines __YASM_MAJOR__
        SECTION .text
ORIGIN:         equ 0xf800      ; showcase placed at PROGRAM offset from seg 0
RAM_BASE:       equ 0x0
%else                           ; 8086tiny: boot sector loads to 0x7E00
ORIGIN:         equ 0x7E00
RAM_BASE:       equ 0x1000
%endif
%endif
        
; --- RAM layout (all relative to RAM_BASE) ----------------------------------
%ifndef RAM_SIZE
RAM_SIZE:       equ 4096        ; 8bitworkshop / 8086tiny: 4KB RAM
%endif
DIV0:           equ RAM_BASE + 0x000    ; 4 bytes: divide-by-zero vector (ROM)
CURLN:          equ RAM_BASE + 0x004    ; word:  current line number (error reports)
RUN_NEXT:       equ RAM_BASE + 0x006    ; word:  next-line pointer for run loop
NMI:            equ RAM_BASE + 0x008    ; 4 bytes: NMI vector (ROM)
IBUF:           equ RAM_BASE + 0x00C    ; 64 bytes: input line buffer
INS_TMP:        equ RAM_BASE + 0x04C    ; word:  insline / do_for var_ptr scratch
GOSUB_SP:       equ RAM_BASE + 0x04E    ; word: gosub stack depth (0..7)
GOSUB_STK:      equ RAM_BASE + 0x050    ; 16 bytes: 8 gosub return addresses
FOR_SP:         equ RAM_BASE + 0x060    ; word: FOR stack depth (0..3)
FOR_STK:        equ RAM_BASE + 0x062    ; 32 bytes: 4 x 8-byte FOR frames
VARS:           equ RAM_BASE + 0x082    ; 52 bytes: A-Z variables (word each)
RUNNING:        equ RAM_BASE + 0x0B6    ; byte:  0=immediate, 1=running
PROG_END:       equ RAM_BASE + 0x0B7    ; word:  one past last program byte
PROGRAM:        equ RAM_BASE + 0x0B9    ; program store start
STACK_TOP:      equ RAM_BASE + RAM_SIZE ; initial SP
PROGRAM_TOP:    equ STACK_TOP - 0x100   ; 256-byte stack reserve

; --- Error codes -------------------------------------------------------------
ERR_SN:         equ 0x30  
ERR_UL:         equ 0x31  
ERR_OV:         equ 0x32   
ERR_OM:         equ 0x33  
ERR_UK:         equ 0x34
ERR_RT:         equ 0x35  ; RETURN without GOSUB
ERR_NF:         equ 0x36  ; NEXT without FOR
ERR_BRK:        equ 0x42        ; NMI break: prints "?B"

; --- Keyword last-byte constants: ASCII | 0x80 -------------------------------
T_T:            equ 0xD4        ; 'T'  PRINT LIST INPUT LET
T_F:            equ 0xC6        ; 'F'  IF
T_O:            equ 0xCF        ; 'O'  GOTO
T_N:            equ 0xCE        ; 'N'  RUN THEN
T_W:            equ 0xD7        ; 'W'  NEW
T_M:            equ 0xCD        ; 'M'  REM
T_D:            equ 0xC4        ; 'D'  END
T_E:            equ 0xC5        ; 'E'  POKE FREE
T_K:            equ 0xCB        ; 'K'  PEEK
T_R:            equ 0xD2        ; 'R'  USR
T_P:            equ 0xD0        ; 'P'  HELP
T_DS:           equ 0xA4        ; '$'  CHR$
T_B:            equ 0xC2        ; 'B'  GOSUB

; --- Token bytes (0x80.. = keyword in stored program) ---
; Order matches st_tab
TK_PRINT:       equ 0x80
TK_IF:          equ 0x81
TK_GOTO:        equ 0x82
TK_LIST:        equ 0x83
TK_RUN:         equ 0x84
TK_NEW:         equ 0x85
TK_INPUT:       equ 0x86
TK_REM:         equ 0x87
TK_END:         equ 0x88
TK_LET:         equ 0x89
TK_POKE:        equ 0x8A
TK_FREE:        equ 0x8B
TK_HELP:        equ 0x8C
TK_GOSUB:       equ 0x8D
TK_RETURN:      equ 0x8E
TK_FOR:         equ 0x8F        ; st_tab index 15
TK_NEXT:        equ 0x90        ; st_tab index 16
TK_OUT:		equ 0x91
NUM_TOKENS:     equ 18          ; stmt-dispatch tokens: TK_PRINT..TK_OUT
; sub-keywords TK_THEN/TK_TO/TK_STEP are >= 0x91 (not dispatched by stmt)
TK_THEN:        equ 0x92        ; sub-keyword (not in st_tab, outside dispatch)
TK_TO:          equ 0x93        ; sub-keyword (FOR..TO)
TK_STEP:        equ 0x94        ; sub-keyword (FOR..STEP)

; --- ROM bitbang serial: Intel 8755, 4800 baud @ 5MHz ----------------------
PORT_A:         equ 0x00        ; 8755 Port A data register
DDR_A:          equ 0x02        ; 8755 Port A direction register
TX:             equ 0x01        ; Port A bit 0 = TX (output)
RX:             equ 0x02        ; Port A bit 1 = RX (input)
BAUD:           equ 60          ; bit-period loop count: 17cy/iter @5MHz ~4800baud

; =============================================================================
; Pre-loaded showcase (8BitWorkshop only).  Type RUN to execute, NEW to clear.
; Feature demos   lines 10-190 : arithmetic, comparisons, FOR/NEXT, GOSUB
; Mandelbrot      lines 200-340: FOR loops for rows/cols, GOSUB 600 for escape
; Subroutines     500=sum1..10, 550=factorial5, 600=record-escape
;
; Fixed-point scale 1/64.  16 Mandelbrot iterations.  ASCII density chars.
; Tokens: PRINT=0x80 IF=0x81 GOSUB=0x8D RETURN=0x8E END=0x88
;         FOR=0x8F NEXT=0x90 THEN=0x91 TO=0x92 STEP=0x93 REM=0x87
; =============================================================================

; 8bitworkshop default org 0
%ifdef __YASM_MAJOR__
	mov ax, reset_vec; Trampoline for 8bitworkshop, overwritten when running
	jmp ax          ; One way to do a Near jump greater than 32768
	times (PROGRAM - 5) nop  ;  Pad over program VARS/Equates (3byte mov, 2byte jump)

SHOWCASE_DATA:
        ; ── Feature demos ────────────────────────────────────────────────────
        db 0x0A,0x00,0x87,"uBASIC 8088 v1.5.0 showcase",0x0D         ; 10  REM ...
        db 0x14,0x00,0x80,0x22,"--- ARITHMETIC ---",0x22,0x0D         ; 20  PRINT
        db 0x1E,0x00,0x80,0x22,"2+3=",0x22,";2+3;",0x22,"  6*7=",0x22,";6*7",0x0D   ; 30
        db 0x28,0x00,0x80,0x22,"20/4=",0x22,";20/4;",0x22,"  17%5=",0x22,";17%5",0x0D ; 40
        db 0x32,0x00,0x80,0x22,"--- COMPARISONS ---",0x22,0x0D        ; 50
        db 0x3C,0x00,0x81,"5>3 ",0x91,0x80,0x22,"5>3 ok",0x22,0x0D   ; 60  IF THEN PRINT
        db 0x46,0x00,0x81,"3<5 ",0x91,0x80,0x22,"3<5 ok",0x22,0x0D   ; 70
        db 0x50,0x00,0x81,"3>=3 ",0x91,0x80,0x22,"3>=3 ok",0x22,0x0D ; 80
        db 0x5A,0x00,0x81,"4<>3 ",0x91,0x80,0x22,"4<>3 ok",0x22,0x0D ; 90
        db 0x64,0x00,0x80,0x22,"--- FOR/NEXT ---",0x22,0x0D           ; 100
        db 0x6E,0x00,0x8F,"I=1 ",0x92,"5",0x0D                       ; 110 FOR I=1 TO 5
        db 0x78,0x00,0x80,"I;",0x0D                                   ; 120 PRINT I;
        db 0x82,0x00,0x90,"I",0x0D                                    ; 130 NEXT I
        db 0x8C,0x00,0x80,0x22,0x22,0x0D                              ; 140 PRINT ""
        db 0x96,0x00,0x80,0x22,"--- GOSUB ---",0x22,0x0D              ; 150
        db 0xA0,0x00,0x8D,"500",0x0D                                  ; 160 GOSUB 500 (sum)
        db 0xA5,0x00,0x80,0x22,"sum 1..10=",0x22,";S",0x0D           ; 165 PRINT result
        db 0xAA,0x00,0x8D,"550",0x0D                                  ; 170 GOSUB 550 (fact)
        db 0xAF,0x00,0x80,0x22,"5!=",0x22,";F",0x0D                  ; 175 PRINT result
        db 0xB4,0x00,0x80,0x22,0x22,0x0D                              ; 180 PRINT ""
        ; ── Mandelbrot ───────────────────────────────────────────────────────
        db 0xBE,0x00,0x80,0x22,"--- MANDELBROT ---",0x22,0x0D         ; 190
        db 0xC8,0x00,0x8F,"I=-64 ",0x92,"56 ",0x93,"6",0x0D           ; 200 FOR I=-64 TO 56 STEP 6
        db 0xD2,0x00,0x8F,"C=-128 ",0x92,"16 ",0x93,"4",0x0D          ; 210 FOR C=-128 TO 16 STEP 4
        db 0xDC,0x00,"D=I:A=C:B=D:E=0",0x0D                           ; 220 init row
        db 0xE6,0x00,0x8F,"N=1 ",0x92,"16",0x0D                       ; 230 FOR N=1 TO 16
        db 0xF0,0x00,"T=A*A/64-B*B/64+C",0x0D                         ; 240 iterate
        db 0xFA,0x00,"B=2*A*B/64+D:A=T",0x0D                          ; 250
        db 0x04,0x01,0x81,"A*A/64+B*B/64>256 ",0x91,0x8D,"600",0x0D  ; 260 IF escaped GOSUB 600
        db 0x0E,0x01,0x90,"N",0x0D                                     ; 270 NEXT N
        db 0x18,0x01,0x81,"E>0 ",0x91,0x80,"CHR$(E+32);",0x0D         ; 280 print density
        db 0x22,0x01,0x81,"E=0 ",0x91,0x80,"CHR$(32);",0x0D           ; 290 print space
        db 0x2C,0x01,0x90,"C",0x0D                                     ; 300 NEXT C
        db 0x36,0x01,0x80,0x22,0x22,0x0D                               ; 310 PRINT "" (newline)
        db 0x40,0x01,0x90,"I",0x0D                                     ; 320 NEXT I
        db 0x4A,0x01,0x88,0x0D                                         ; 330 END
        ; ── Subroutine 500: sum 1..10 ─────────────────────────────────────────
        db 0xF4,0x01,"S=0",0x0D                                        ; 500
        db 0xFE,0x01,0x8F,"J=1 ",0x92,"10",0x0D                       ; 510 FOR J=1 TO 10
        db 0x08,0x02,"S=S+J",0x0D                                      ; 520
        db 0x12,0x02,0x90,"J",0x0D                                     ; 530 NEXT J
        db 0x1C,0x02,0x8E,0x0D                                         ; 540 RETURN
        ; ── Subroutine 550: factorial 5 ───────────────────────────────────────
        db 0x26,0x02,"F=1",0x0D                                        ; 550
        db 0x30,0x02,0x8F,"K=1 ",0x92,"5",0x0D                        ; 560 FOR K=1 TO 5
        db 0x3A,0x02,"F=F*K",0x0D                                      ; 570
        db 0x44,0x02,0x90,"K",0x0D                                     ; 580 NEXT K
        db 0x4E,0x02,0x8E,0x0D                                         ; 590 RETURN
        ; ── Subroutine 600: record escape iteration ───────────────────────────
        db 0x58,0x02,0x81,"E=0 ",0x91,"E=N",0x0D                      ; 600 IF E=0 THEN E=N
        db 0x62,0x02,0x8E,0x0D                                         ; 610 RETURN
        dw 0                                                            ; end sentinel
SHOWCASE_END:
	TIMES 0x7cce nop	; times cannot use anything more than 0x8000
	TIMES 0x7800 nop	; had to do this manually (V annoying)
%else
	org ORIGIN			; ROM & bootsector takes origin
%endif

; =============================================================================
; INIT  cold start
; Clobbers: everything
; =============================================================================
start:
        cld
%ifdef ROM
        ; 8755 serial: bit1=RX(in), rest=out; TX idle high
        mov al, 0xFD
        out DDR_A, al
        mov al, TX
        out PORT_A, al
        sti
%endif

%ifdef __YASM_MAJOR__
        ; Zero VARs area: DIV0 through PROG_END (stops before PROGRAM)
        ; Zero RAM: covers DIV0..RUNNING, stops before PROGRAM
        mov di, RAM_BASE
        mov cx, (PROGRAM - RAM_BASE) / 2
        call clr_mem
        ; PROG_END points AT the sentinel (= PROGRAM + body bytes, excl sentinel)
        mov word [PROG_END], PROGRAM+(SHOWCASE_END-SHOWCASE_DATA)-2
%else
        ; Zero RAM inline (no CALL - avoids stack at top of RAM being clobbered).
        ; CX = words to zero = (STACK_TOP-2)/2, leaving return-addr slot intact.
        ; Then separately zero stack slot and set PROG_END.
        xor ax, ax
        mov di, RAM_BASE
        mov cx, RAM_SIZE / 2
        rep stosw
	mov word [PROG_END], PROGRAM
%endif

%ifdef ROM
        ; Install interrupt vectors (IP only; CS was zeroed above)
        mov ax, 0xf800
        mov word [DIV0],   divide_error
        mov word [DIV0+2], ax
        mov word [NMI],    nmi_handler
        mov word [NMI+2],  ax
%endif
        ; signon
	mov si, str_banner
        call dp_str
        call do_free
        ; fall through into main_loop

; =============================================================================
; MAIN_LOOP  prompt / dispatch loop
; =============================================================================
main_loop:
        mov sp, STACK_TOP
	call do_end		; not running
        
	mov al, '>'
        call output
        
        call input_line         ; read line; SI -> IBUF

        call peek_line     	; blank line?
        je main_loop

        call input_number       ; parse optional line number -> AX
        or ax, ax
        jne ml_numbered
        call stmt_line          ; CALL - correct return address
        jmp short main_loop
ml_numbered:
        call editln             ; numbered line: store/edit in program
        jmp short main_loop

; =============================================================================
; STMT_LINE  execute ':'-separated statements on line at SI
; =============================================================================
stmt_line:
        call stmt
sl_chk:
        call spaces
        lodsb               ; AL = [SI], SI = SI + 1
        cmp al, ':'         ; Was it the multi-statement separator?
        je stmt_line        ; Yes: Jump to execute next statement
        
        dec si              ; No: Back up SI so it points to the non-':' char
sl_ret:
        ret

; =============================================================================
; PEEK_LINE  Advance and check for CR
; =============================================================================
peek_line:
        call spaces
        cmp byte [si], 0x0d
stmt_ret:
do_if_false:
	ret
        
; =============================================================================
; DO_IF  IF <expr> [THEN] <stmt>
; Handles both tokenized THEN (0x8F) and plain-text THEN.
; =============================================================================
do_if:
        call expr
        or ax, ax
        je do_if_false
        call spaces
        cmp byte [si], TK_THEN  ; tokenized THEN?
        jne di_kw_then
        inc si                  ; consume token
        jmp stmt                ; drop through into stmt
di_kw_then:
        mov bx, then_tab
        call kw_match           ; try plain-text THEN (direct mode)
        ; drop through
        
; =============================================================================
; STMT  execute one statement from SI
; Token fast-path: stored programs have keyword tokens (0x80..0x8F);
; direct-mode input falls through to the kw_match loop as before.
; =============================================================================
stmt:
        call peek_line
        je stmt_ret
        ; --- Token fast-path (stored programs) ---
        mov al, [si]
        cmp al, TK_PRINT        ; first token?
        jb  stmt_text           ; < 0x80: plain text -> kw_match loop
        cmp al, TK_PRINT + NUM_TOKENS
        jnb stmt_text           ; >= 0x90: not a token
        inc si                  ; consume token byte
        sub al, TK_PRINT        ; AL = 0..15 (index into st_tab)
        xor ah, ah
        add ax, ax              ; AX = index * 2
        add ax, ax              ; AX = index * 4 (each st_tab entry = 4 bytes)
        mov bx, st_tab
        add bx, ax              ; BX -> correct st_tab entry
        jmp [bx+2]              ; dispatch directly (saves kw_match loop)
        ; --- Text fall-through (direct mode) ---
stmt_text:
        mov bx, st_tab
stmt_lp:
        mov ax, [bx]
        or ax, ax
        je do_let               ; sentinel -> implicit LET/assignment
        call kw_match
        jnc stmt_call
        add bx, 4
        jmp short stmt_lp
stmt_call:
        jmp [bx+2]              ; indirect call to handler
; =============================================================================
; DO_INPUT  INPUT <var>
; =============================================================================
do_input:
        ; Validate variable letter before proceeding
	call let_input_hlpr            
        mov al, '?'
        call output
        mov al, ' '
        call output
        call input_line
        call expr
        pop di
        mov [di], ax
        ret
        
; =============================================================================
; DO_LET  [LET] <var> = <expr>
; =============================================================================
do_let:
	call let_input_hlpr     
        call spaces
        cmp byte [si], '='
        jne dl_err2
        inc si
        call expr
        pop di
        mov [di], ax
        ret
dl_err2:
        pop di
JERRUK:
        mov al, ERR_UK
        jmp do_error

; -- helper
let_input_hlpr:
        call spaces
        mov al, [si]
        call uc_al
        cmp al, 'A'
        jb JERRUK
        cmp al, 'Z'
        ja JERRUK
        call get_var_addr
        push di
        ret

; =============================================================================
; KW_MATCH  case-insensitive keyword match at [SI]
;
; Inputs  : BX -> table entry word 0 = keyword string ptr
;           SI -> input text
; Outputs : CF=0 matched, SI advanced past keyword
;           CF=1 no match, SI unchanged
; Clobbers: AX, DI, DL
; =============================================================================
kw_match:
        push    si              ; Save SI for failure restoration
        call    spaces          ; Skip whitespace
        mov     di, [bx]        ; DI = pointer to keyword string
.match_lp:
        mov     al, [di]        ; Get keyword char
        inc     di              ; Move to next keyword char
        mov     dl, al          ; DL holds char + "end-of-word" flag (bit 7)
        and     al, 0x7F        ; Strip flag for comparison
        call    uc_al           ; Uppercase keyword char
        mov     ah, al          ; Store uppercased keyword char in AH
        
        lodsb                   ; Load input char into AL, SI++
        call    uc_al           ; Uppercase input char
        
        cmp     al, ah          ; Compare input (AL) with keyword (AH)
        jne     .fail           ; Mismatch? Restore SI and exit
        
        test    dl, 0x80        ; Was that keyword char the last one?
        jz      .match_lp       ; No? Keep matching characters
        
        ; --- Boundary Check: Ensure we didn't match a prefix (e.g., IF vs IFFY) ---
        mov     al, [si]        ; Peek at next char in input
        call    uc_al           ; Standardize to uppercase
        cmp     al, '_'         ; Underscore is part of a word
        je      .fail
        cmp     al, 'A'         ; If < 'A'...
        jb      .check_num      ; ...it might be a number
        cmp     al, 'Z'         ; If 'A'-'Z'...
        jbe     .fail           ; ...it's still a word (no boundary)
.check_num:
        cmp     al, '0'         ; If < '0'...
        jb      .ok             ; ...it's a valid boundary (space, etc.)
        cmp     al, '9'         ; If '0'-'9'...
        jbe     .fail           ; ...it's a word (no boundary)
.ok:
        pop     ax              ; Match success! Discard saved SI
        clc                     ; Clear Carry Flag (Matched)
        ret
.fail:
        pop     si              ; Match failed: Restore SI to start
        stc                     ; Set Carry Flag (No Match)
        ret

; =============================================================================
; UC_AL uppercase 
; =============================================================================
uc_al:
        cmp al, 'a'
        jb uc_al_r
        cmp al, 'z'
        ja uc_al_r
        and al, 0xdf
uc_al_r:
dl_done:
        ret

; =============================================================================
; DO_LIST
; =============================================================================
do_list:
        mov di, PROGRAM
dl_lp:
        mov ax, [di]        ; Load word at DI
        test ax, ax         ; Shortest way to check for NULL sentinel
        jz dl_done          ; Exit if 0

        call output_number
        mov al, ' '
        call output

        mov si, di          ; SI = DI
        inc si              ; SI = DI + 2 (using two INCs is 2 bytes, 
        inc si              ; same as ADD SI, 2, but often cleaner)

dl_body:
        lodsb
        cmp al, 0x0d            ; CR = end of line
        je  dl_eol
        cmp al, TK_PRINT        ; token? (0x80..0x8F)
        jb  dl_raw              ; < 0x80: plain char
        cmp al, TK_PRINT + NUM_TOKENS + 3  ; cover TK_FOR..TK_STEP (0x8F..0x93)
        jnb dl_raw              ; >= 0x94: not a token
        ; detokenize: look up keyword string and print it
        sub al, TK_PRINT        ; index 0..15
        xor ah, ah
        add ax, ax              ; word offset
        mov bx, tk_kw_tab
        add bx, ax
        mov bx, [bx]            ; BX -> keyword string
        ; print keyword chars (bit-7 terminated), followed by space
        push si
        mov si, bx
dl_kw_lp:
        call dp_str
        mov al, ' '
        call output
        pop si
        jmp dl_body
dl_raw:
        call output
        jmp dl_body
dl_eol:
        call new_line
        call next_line_ptr
        jmp dl_lp

; =============================================================================
; DO_REM  skip rest of line
; =============================================================================
do_rem:
        lodsb           ; AL = [SI], then SI++
        cmp al, 0x0d    ; Was it the CR?
        jne do_rem      ; If not, keep going
dp_ret:
        ret

; =============================================================================
; DO_PRINT  PRINT [item [; item] ...]
; Items: "string", CHR$(n), expression.
; ';' between items or trailing ';' suppresses CR+LF.
; Also used to print strings - terminator top bit set
; =============================================================================
do_print:
dp_top:
        call peek_line	; empty line PRINT
        je dp_nl
        cmp byte [si], '"'
        jne dp_expr
        inc si		; skip over "
dp_str:
        lodsb
    	cmp al, 0x22	; check for PRINT terminator
    	je dp_after
    
    	test al, 0x80	; check for top bit terminator
    	jz loop_print
    
    	and al, 0x7f
    	jmp output        ; Tail-call optimization: output will RET for us

loop_print:
        call output
        jmp short dp_str		
        
dp_nl:
        jmp new_line	; tail call   
dp_expr:
        mov bx, chrs_tab
        call kw_match
        jc dp_num
        call eat_paren_expr
        call output
        jmp short dp_after
dp_num:
        call expr
        call output_number
dp_after:
        call spaces
        cmp byte [si], ';' ; no newline
        jne dp_nl
        inc si
        call peek_line
        je dp_ret
        jmp short dp_top

; =============================================================================
; DO_POKE  POKE <addr>, <val> - need to finagle addresses above 32768
; DO_OUT  OUT <addr>, <val> 
; =============================================================================
do_poke:
	call poke_out_hlpr
	mov [di], al
	ret

do_out:
	call poke_out_hlpr
	mov dx,di
        out dx, al
	ret

poke_out_hlpr:
        call expr		; destination address
        push ax			; save: second expr's kw_match clobbers DI
        call spaces
        cmp byte [si], ','
        jne JERRSN
        inc si
        call expr               ; AX = value to poke
        pop di
        ret
JERRSN:
        mov al, ERR_SN
        jmp do_error

; =============================================================================
; EXPR  evaluate expression including relational operators
;
; Inputs  : SI -> expression text
; Outputs : AX = signed 16-bit result; true=0xFFFF false=0x0000
; Clobbers: AX, BX, CX, DX, SI
expr:
        call    expr_add        ; Left operand -> AX
        push    ax              ; Save left
        call    spaces
        
        ; Map relational operators to a bitmask (LT=1, EQ=2, GT=4)
        xor     dx, dx          ; DX = 0 (Our mask)
.op_loop:
        lodsb                   ; Peek/get char
        cmp     al, '<'
        jne     .not_lt
        or      dl, 1
        jmp short     .op_loop
.not_lt:
        cmp     al, '='
        jne     .not_eq
        or      dl, 2
        jmp short     .op_loop
.not_eq:
        cmp     al, '>'
        jne     .not_gt
        or      dl, 4
        jmp short     .op_loop
.not_gt:
        dec     si              ; Backtrack SI (non-relational char)
        test    dl, dl          ; Did we find any relational operators?
        jnz     .do_rel         ; Yes, go handle them
        pop     ax              ; No, restore left operand
        ret

.do_rel:
        push    dx              ; Save operator mask
        call    expr_add        ; Right operand -> AX
        pop     dx              ; Restore mask (DL)
        pop     bx              ; BX = left operand

        cmp     bx, ax          ; Compare left vs right
        
        ; 1. Generate 'Equal' bit (Bit 1 / value 2)
        mov     ax, 2           ; Assume equal (2 bytes)
        jz      .check          ; If ZF=1, we are done with AL=2 (2 bytes)

        ; 2. Generate 'LT' (1) or 'GT' (4) bit
        ; If we are here, ZF=0.
        sbb     ax, ax          ; If BX < AX (signed), CF is often not enough for signed... 
                                ; Better: use the actual signed flags.
        jl      .set_lt
        mov     al, 4           ; GT bit
        jmp     short .check
.set_lt:
        mov     al, 1           ; LT bit

.check:
        test    al, dl
        mov     ax, 0           ; Clear AX (shorter than xor/dec combo for logic)
        jz      .done           ; If test fails, return 0
        dec     ax              ; Result is -1 (0xFFFF)
.done:
e1_ret:
ea_ret:
        ret

; =============================================================================
; EXPR_ADD  additive level: + and -
; =============================================================================
expr_add:
        call expr1
ea_lp:
        call spaces
        mov dl, [si]
        cmp dl, '+'
        je ea_do
        cmp dl, '-'
        jne ea_ret
ea_do:
        push dx                 ; save operator DL (clobbered by expr1)
        push ax                 ; save left
        inc si
        call expr1
        pop bx                  ; BX = left
        pop dx                  ; DL = operator
        cmp dl, '-'
        jne ea_add
        neg ax
ea_add:
        add ax, bx
        jmp ea_lp

; =============================================================================
; EXPR1  multiplicative level: * / %
; =============================================================================
expr1:
        call expr2
e1_lp:
        call spaces
        mov dl, [si]
        cmp dl, '*'
        je e1_do
        cmp dl, '/'
        je e1_do
        cmp dl, '%'
        jne e1_ret
e1_do:
        inc si
        push dx
        push ax
        call expr2
        pop cx                  ; CX = left
        pop dx                  ; DL = operator
        cmp dl, '*'
        jne e1_div
        xchg ax, cx             ; AX = left, CX = right
        imul cx
        jmp e1_lp
e1_div:
        or ax, ax               ; AX = right (divisor)
        je e1_zero
        xchg ax, cx             ; AX = left, CX = right
        mov bl, dl              ; save operator before IDIV clobbers DX
        cwd
        idiv cx                 ; AX=quotient DX=remainder (DL overwritten!)
        cmp bl, '%'
        jne e1_lp
        mov ax, dx              ; modulo: return remainder
        jmp e1_lp
e1_zero:
        mov al, ERR_OV
        jmp do_error

; =============================================================================
; EXPR2  atom level
; =============================================================================
e2_pos:
        inc si
	; drop through
expr2:
        call spaces
        mov al, [si]
        cmp al, '('
        je e2_par
        cmp al, '-'
        je e2_neg
        cmp al, '+'
        je e2_pos
        ; CHR$
        mov bx, chrs_tab
        call kw_match
        jc e2_nchrs
	; drop through
        
; eat_paren_expr: '(' expr ')' -> AX
eat_paren_expr:
        call spaces
        cmp byte [si], '('
        jne epe_err
e2_par:
	inc si
        call expr
        call spaces
        cmp byte [si], ')'
        jne epe_err
        inc si
        ret 

peek_in_hlp:
        call eat_paren_expr
        mov bx, ax
        xor ah, ah
        ret
        
e2_nchrs:	; not chr$ is it PEEK 
        ; PEEK
        mov bx, peek_tab
        call kw_match
        jc e2_npeek	; not peek
        call peek_in_hlp
        mov al, [bx]
        ret

e2_npeek:	; not PEEK is it IN 
        ; IN
        mov bx, in_tab
        call kw_match
        jc e2_nin	; not peek
        call peek_in_hlp
        mov dx, bx
        in al,dx
        ret

e2_nin:
        ; USR
        mov bx, usr_tab
        call kw_match
        jc e2_nusr
        call eat_paren_expr
        jmp ax ; tail call
        
e2_var:
        ; variable A-Z?
        call uc_al
        cmp al, 'A'
        jb e2_bad
        cmp al, 'Z'
        ja e2_bad
        call get_var_addr
        mov ax, [di]
        ret
e2_bad:
        xor ax, ax
        ret      
        
e2_neg:
        inc si
        call expr2
        neg ax
        ret

epe_err:
        jmp JERRSN

; =============================================================================
; GET_VAR_ADDR  letter at [SI] -> DI=&var, SI advanced
; =============================================================================
get_var_addr:
        lodsb
        call uc_al
        sub al, 'A'
        xor ah, ah
        add ax, ax
        add ax, VARS
        mov di, ax
sp_r:   ret

; =============================================================================
; SPACES  skip spaces; preserves AX, BX, CX, DX
; =============================================================================
spaces:
        cmp byte [si], ' '
        jne sp_r	; return
        inc si
        jmp short spaces

e2_nusr:
        ; Reload AL from [si] - kw_match may have clobbered it
        mov al, [si]
        ; decimal literal?
        cmp al, '0'
        jb e2_var
        cmp al, '9'
        ja e2_var
	; drop through
        
; =============================================================================
; INPUT_NUMBER  parse unsigned decimal from [SI] -> AX; SI past digits
; =============================================================================
input_number:
        xor bx, bx
inm_lp:
        mov al, [si]
        sub al, '0'
        jb inm_done
        cmp al, 9
        ja inm_done
        inc si
        cbw
        xchg ax, bx
        mov cx, 10
        mul cx
        add bx, ax
        jmp short inm_lp
inm_done:
        mov ax, bx
        ret

; =============================================================================
; OUTPUT_NUMBER  signed 16-bit AX -> terminal
; =============================================================================
output_number:
        or ax, ax
        jns on_pos
        push ax
        mov al, '-'
        call output
        pop ax
        neg ax
on_pos:
        xor dx, dx
        mov cx, 10
        div cx
        push dx
        or ax, ax
        je on_digit
        call output_number
on_digit:
        pop ax
        add al, '0'
        jmp output
       
; =============================================================================
; INPUT_LINE  read edited line into IBUF; SI -> IBUF
; =============================================================================
input_line:
        mov di, IBUF
        xor cx, cx
ipl_lp:
        call input_key
        cmp al, 0x08
        jne ipl_nbs
        or cx, cx
        je ipl_lp
        dec di
        dec cx
        call backsp
        mov al, ' '
        call output
        call backsp
        jmp ipl_lp
backsp:
        mov al, 0x08
        call output
	ret
        
ipl_nbs:
        cmp al, 0x0d
        je ipl_cr
        cmp cx, 62	; max line length - need a EQU really
        jnb ipl_lp
        call output
        stosb
        inc cx
        jmp ipl_lp
ipl_cr:
        stosb
        mov si, IBUF
        ; drop through
                
; =============================================================================
; NEW_LINE  CR + LF - does not touch SI or DI
; =============================================================================
new_line:
        mov al, 0x0d
        call output
        mov al, 0x0a
	; drop through

; =============================================================================
; OUTPUT  AL -> terminal
; ROM: bitbang 8N1 via 8755 Port A.  Others: BIOS INT 10h.
; =============================================================================
output:
%ifdef ROM
        ; bitbang TX: start bit, 8 data bits LSB-first, stop bit
        mov ah, al              ; stash char
        xor al, al
        out PORT_A, al          ; start bit (TX=0)
        call bdly
        mov cx, 8
.out_bit:
        mov al, ah
        and al, TX              ; LSB into bit0
        out PORT_A, al
        call bdly
        shr ah, 1
        loop .out_bit
        mov al, TX
        out PORT_A, al          ; stop bit (TX=1, line idles high)
        ret                     ; no stop-bit delay: next call handles gap
%else
        push bx
        mov ah, 0x0e
        mov bx, 0x0007
        int 0x10
        pop bx
        ret
%endif
        
; =============================================================================
; DO_FREE  print free program-store bytes
; =============================================================================
do_free:
        mov ax, PROGRAM_TOP
        sub ax, [PROG_END]
        call output_number
        mov al, ' '
        call output
        mov si, kw_free
        call dp_str
	jmp new_line     ; Tail-call: new_line will RET for us

; =============================================================================
; DO_HELP  print all keywords
; =============================================================================
do_help:
    mov si, kw_tab_start
dh_lp:
    call dp_str
    mov al, ' '
    call output
    cmp byte [si], 0 ; Check for sentinel
    jne dh_lp        ; Loop back if not zero

    jmp new_line     ; Tail-call: new_line will RET for us
    
; =============================================================================
; DO_ERROR  print error; never returns to caller
; Inputs  : AX = error code
; =============================================================================
do_error:
        push ax
        call new_line
        
        mov al, '?'
        call output
        pop ax
        ; add al, '0'
        call output             ; print "?N"
        ; direct mode?
        cmp byte [RUNNING], 0
        je do_error_nl
        
        mov al, '@'
        call output
        mov ax, [CURLN]
        call output_number
do_error_nl:
        call new_line
        jmp main_loop    
        
; =============================================================================
; INPUT_KEY  -> AL
; ROM: bitbang UART RX from 8755 Port A bit1.  Others: BIOS INT 16h.
; =============================================================================
input_key:
%ifdef ROM
; Wait for start bit (RX goes low), then sample 8 data bits LSB-first.
.ik_wait:
        in al, PORT_A
        test al, RX
        jnz .ik_wait
        call bdly               ; skip start bit, land mid-first-data-bit
        mov cx, 8
        xor ah, ah
.ik_bit:
        in al, PORT_A
        shr al, 1               ; RX bit1 -> CF
        rcr ah, 1               ; accumulate LSB-first into AH
        call bdly
        loop .ik_bit
        mov al, ah              ; stop-bit delay omitted: .ik_wait handles it
        ret
%else
        mov ah, 0x00
        int 0x16
        ret
%endif

; =============================================================================
; ROM-only: bdly (bit-delay), divide_error, nmi_handler, do_error_hw
; Placed here so they land well before the 0xFFF0 reset vector area.
; =============================================================================
%ifdef ROM
bdly:
; BDLY  one bit-period delay: BAUD iterations of LOOP @ 17cy = ~4800 baud @5MHz
; Clobbers: nothing (saves/restores CX)
        push cx
        mov cx, BAUD
        loop $
        pop cx
        ret

; DIVIDE_ERROR  INT 0 handler: reset stack and show ?2
divide_error:
        mov al, ERR_OV
        jmp do_error_hw

; NMI_HANDLER  INT 2 handler: reset stack and show ?B
nmi_handler:
        mov al, ERR_BRK
        ; fall through to do_error_hw

; DO_ERROR_HW: abandon corrupt interrupt stack, re-enter interpreter
do_error_hw:
        mov sp, STACK_TOP
        jmp do_error
%endif

; =============================================================================
; LINE EDITOR  
; =============================================================================
find_program_end:
        mov ax, 0xffff
find_line:
;         
walk_lines:
        mov di, PROGRAM
wl_lp:
        mov bx, [di]
        or bx, bx
        je wl_done
        cmp bx, ax
        jnb wl_done
        call next_line_ptr
        jmp wl_lp

next_line_ptr:
        add di, 2
nlp_lp:
        cmp byte [di], 0x0d
        je nlp_done
        inc di
        jmp nlp_lp
nlp_done:
        inc di
wl_done:
el_done:
editln_done:
	ret

; EDITLN  AX=linenum, SI->body text (raw, from IBUF)
; Tokenizes the body in-place, finds insertion point, deletes existing
; line if present, inserts new line (unless body is empty = delete only).
editln:
        push ax                 ; save line number
        call spaces             ; skip any leading spaces
        ; tokenize body in-place in IBUF (writes back tokenized form)
        call tokenize           ; SI->body unchanged; tokenized in-place
        pop dx                  ; DX = line number
        ; measure tokenized body length (including CR)
        mov bx, si              ; BX = body start
        mov cx, 0
el_len:
        inc cx
        cmp byte [si], 0x0d
        je el_ldone
        inc si
        jmp el_len
        
el_ldone:
        ; CX = length including CR; SI points to the CR
        ; Find insertion point (push BX: find_line/walk_lines clobbers BX)
        push bx
        mov ax, dx
        call find_line          ; DI = insertion point (first line >= AX)
        cmp [di], dx            ; exact match = line already exists?
        jne el_noex
        call deline             ; delete it; DI unchanged (still insert point)
el_noex:
        pop bx                  ; restore body pointer
        cmp byte [bx], 0x0d     ; empty body = delete-only
        je editln_done
        mov si, bx              ; SI = body start
        mov ax, dx              ; AX = line number
        ; CX = body+CR len; insline will add 2 for linenum
        jmp insline
        
; INSLINE  insert a line into the program store.
; Inputs:  AX = line number (word)
;          DI = insertion point (from editln's find_line - no second walk)
;          SI -> tokenized body including CR
;          CX = body+CR length
; Clobbers: AX, BX, CX, DX, SI, DI
insline:
        ; Overflow check: need CX + 2 (linenum) + 2 (sentinel) more bytes
        mov bx, [PROG_END]
        add bx, cx
        add bx, 4               ; +2 linenum +2 sentinel
        cmp bx, PROGRAM_TOP
        jnb ins_oom

        ; DX = gap size = body+CR + linenum word
        mov dx, cx
        add dx, 2

        ; Shift existing data [DI..PROG_END+1] up by DX bytes (backward copy)
        ; bytes_to_shift = PROG_END + 2 - DI
        push di                 ; save insertion point
        push si                 ; save body pointer
        push cx                 ; save body+CR length

        mov bx, [PROG_END]
        add bx, 2               ; BX = one past last sentinel byte
        sub bx, di              ; BX = bytes to shift up
        jz  ins_shift_done      ; inserting at end: no shift needed

        ; SI = last source byte, DI = last destination byte
        mov si, [PROG_END]
        inc si                  ; last src = PROG_END + 1
        mov di, si
        add di, dx              ; last dst = last_src + gap
        mov cx, bx              ; CX = bytes to shift
        std
        rep movsb
        cld
ins_shift_done:
        pop cx                  ; body+CR length
        pop si                  ; body pointer
        pop di                  ; insertion point

        ; Write line number, then body+CR
        mov [di], ax
        add di, 2
ins_copy:
        rep movsb                ; copy tokenized body+CR (CX bytes)

        ; PROG_END += linenum_word + body+CR = DX
        add [PROG_END], dx
        ret

ins_oom:
        mov al, ERR_OM
        jmp do_error

; DELINE  delete line at DI
; Inputs:  DI -> line to delete
; Outputs: DI = unchanged insertion point (rep movsb advances DI; we restore)
; Uses PROG_END directly (no find_program_end walk needed).
deline:
        push di                 ; save insertion point
        call next_line_ptr      ; DI -> first byte of next line (src of slide)
        mov si, di              ; SI = source (data to move down)
        mov cx, [PROG_END]
        add cx, 2               ; CX = one past end (+2 for sentinel word)
        pop di                  ; DI = insertion point (dest of slide)
        push di                 ; save again: rep movsb will advance DI
        sub cx, si              ; CX = bytes to move
        mov bx, si
        sub bx, di              ; BX = bytes deleted (to subtract from PROG_END)
        rep movsb               ; slide data down (DI advances by CX)
        sub [PROG_END], bx
        pop di                  ; restore DI to original insertion point
dg_ret:
	ret
; =============================================================================
; DO_NEW  clear program store (PROGRAM..PROGRAM_TOP) and reset PROG_END
; Uses rep stosw so the sentinel at PROG_END is always 0x0000.
; Also eliminates stale data that could confuse walk_lines after LOAD/SAVE.
; =============================================================================
do_new:
        mov word [PROG_END], PROGRAM
        mov di, PROGRAM         ; start of program store
        mov cx, (PROGRAM_TOP - PROGRAM) / 2   ; words to clear
clr_mem:
        xor ax, ax
	rep stosw               ; zero entire program store
        ; fall through to do_end (clears RUNNING via run_end)

; =============================================================================
; DO_END  END statement - stops program execution
; =============================================================================
do_end:
        mov ax, [PROG_END]      ; Point RUN_NEXT at the sentinel (0x0000 word)
        mov [RUN_NEXT], ax      ; run_loop will read 0 and exit cleanly
        xor al, al              ; AL=0 for RUNNING clear below
run_end:
        mov byte [RUNNING], al  ; Clear running flag
        ret
; =============================================================================
; DO_GOTO / DO_RUN
; =============================================================================
do_goto:
        call    expr            ; AX = target line
        call    find_line       ; DI = pointer to line >= AX
        cmp     [di], ax        ; Does it exist exactly?
        je      dg_common       ; Yes: proceed to run/resume
JERRUL:
        mov     al, ERR_UL      ; No: Undefined Line error
        jmp     do_error

do_run:
        mov     di, PROGRAM     ; RUN always starts at the beginning
dg_common:
        mov     [RUN_NEXT], di  ; Update the program counter
        cmp     byte [RUNNING], 0
        jne     dg_ret          ; If already running (from a GOTO), just return
        inc     byte [RUNNING]  ; Set RUNNING to 1
        ; Fall through to run_loop

; =============================================================================
; RUN_LOOP
; =============================================================================
run_loop:
        mov     di, [RUN_NEXT]  ; Get pointer to current line
        mov     si, di
        lodsw                   ; AX = Line Number, SI = points to body
        test    ax, ax          ; Is it 0000 (End of Program)?
        jz      run_end
        
        mov     [CURLN], ax     ; Update current line number for error reporting
        call    next_line_ptr   ; This uses DI to find the START of the NEXT line
        mov     [RUN_NEXT], di  ; Store it for the next iteration
        
        ; SI is already pointing to the body because of LODSW!
        call    stmt_line       ; Execute the statement
        jmp     short run_loop

; =============================================================================
; DO_GOSUB  GOSUB <linenum>
;   Saves current RUN_NEXT on the GOSUB stack, then jumps to target line.
;   Uses a dedicated 8-entry stack at GOSUB_STK to avoid hardware stack
;   conflicts with the interpreter's own call chain.
; Inputs : SI -> expression (line number)
; Clobbers: AX, BX, DI
; =============================================================================
do_gosub:
        call    expr            ; AX = target line number
        call    find_line       ; DI -> line entry >= AX
        cmp     [di], ax        ; exact match?
        jne     gs_noline       ; no -> undefined line error
        mov     bx, [GOSUB_SP]  ; BX = current stack depth
        cmp     bx, 8           ; stack full? (max 8 levels)
        jb      gs_push         ; room: go push
        jmp     JERRSN		; stack overflow -> syntax error
gs_noline:
        jmp     JERRUL
gs_push:
        inc     word [GOSUB_SP] ; bump depth first (BX still = old depth)
        add     bx, bx          ; BX = byte offset = old_depth * 2
        mov     si, GOSUB_STK
        add     si, bx          ; SI -> save slot
        mov     ax, [RUN_NEXT]
        mov     [si], ax        ; push RUN_NEXT onto gosub stack
        mov     [RUN_NEXT], di  ; set PC to GOSUB target line
        ret

; =============================================================================
; DO_RETURN  RETURN
;   Pops a return address from the GOSUB stack and resumes execution there.
; Clobbers: AX, BX
; =============================================================================
do_return:
        mov     bx, [GOSUB_SP]  ; BX = current stack depth
        or      bx, bx          ; stack empty?
        jz      gs_underflow    ; yes -> error ?5
        dec     bx              ; pre-decrement
        mov     [GOSUB_SP], bx  ; store new depth
        add     bx, bx          ; BX = byte offset (depth * 2)
        mov     si, GOSUB_STK   ; SI -> base of gosub stack
        add     si, bx          ; SI -> this slot
        mov     ax, [si]        ; pop return address
        mov     [RUN_NEXT], ax  ; restore PC to line after GOSUB
        ret

gs_underflow:
        mov     al, ERR_RT      ; ?5 return without gosub
        jmp     do_error


; =============================================================================
; TOKENIZE  Convert keyword text -> token bytes in-place in IBUF.
; Input:  SI -> start of body text in IBUF (after line number was parsed)
; Output: body in IBUF replaced with tokenized form; SI unchanged.
; String literals (between quotes) and REM bodies passed through verbatim.
; Token bytes 0x80..0x8F replace keywords; all other chars copied as-is.
; Since tokenized form is always <= original, in-place is safe.
; Clobbers: AX, BX, CX, DX, DI (not SI - caller needs it)
; =============================================================================
tokenize:
        push si                 ; preserve SI for caller
        mov di, si              ; DI = write ptr (same as read ptr initially)
tk_lp:
        mov al, [si]
        cmp al, 0x0d            ; end of line?
        je  tk_done
        cmp al, '"'             ; start of string literal?
        jne tk_kw_scan
        ; copy opening quote, then fall into string verbatim copy
        lodsb                   ; consume the '"' (al still = '"')
        stosb
        jmp tk_str_body         ; enter string loop past the quote
tk_kw_scan:
        ; Only try keyword match if current char is a letter (A-Z/a-z).
        ; Non-letters (spaces, digits, operators) cannot start a keyword:
        ; copy verbatim without kw_match attempts. Also preserves spaces exactly.
        cmp al, 'A'
        jb  tk_char
        cmp al, 'Z'
        jbe tk_kw_try
        cmp al, 'a'
        jb  tk_char
        cmp al, 'z'
        ja  tk_char
tk_kw_try:
        ; try each keyword in token order
        ; BX walks tk_kw_tab one word at a time; index computed from BX offset on match
        mov bx, tk_kw_tab
tk_try:
        cmp word [bx], 0        ; end of table?
        je  tk_char             ; no keyword matched: copy char
        push di                 ; kw_match clobbers DI (keyword scan ptr)
        push bx                 ; kw_match may use BX indirectly; save for index calc
        call kw_match           ; CF=0 if matched (SI advanced past keyword)
        pop bx                  ; restore table pointer
        pop di                  ; restore write pointer
        jnc tk_emit             ; matched! BX still points to matched entry
        add bx, 2               ; next table entry
        jmp tk_try
tk_emit:
        ; compute token byte from table offset: index = (BX - tk_kw_tab) / 2
        mov ax, bx
        sub ax, tk_kw_tab       ; AX = byte offset into table
        shr ax, 1               ; AX = token index (0..15)
        add al, TK_PRINT        ; token byte = 0x80 + index
        stosb                   ; write to DI
        push ax                 ; save token byte
        call spaces             ; consume trailing whitespace after keyword
        pop ax                  ; restore token byte
        cmp al, TK_REM          ; was it REM? (REM body must pass through verbatim)
        jne tk_lp
        ; REM: copy rest of line verbatim
tk_rem_lp:
        lodsb                   ; read from SI
        stosb                   ; write to DI
        cmp al, 0x0d
        jne tk_rem_lp
        jmp tk_finish
tk_char:
        ; no keyword matched: copy one char literally and advance both ptrs
        lodsb                   ; al = [si], si++
        stosb                   ; [di] = al, di++
        jmp tk_lp
tk_str:
        ; inside string: copy verbatim until closing quote or CR
        lodsb                   ; read opening quote (first entry)
        stosb
tk_str_body:
        lodsb
        stosb
        cmp al, '"'
        je  tk_lp               ; closing quote: resume keyword scanning
        cmp al, 0x0d
        jne tk_str_body
        jmp tk_finish           ; CR inside string (malformed)
tk_done:
        stosb                   ; write the CR
tk_finish:
        pop si                  ; restore SI to body start
        ret

; =============================================================================
; DO_FOR  FOR <var> = <start> TO <end> [STEP <step>]
; Frame layout in FOR_STK: var_ptr(w), limit(w), step(w), loop_ptr(w)
; Inputs : SI -> line body after FOR token
; Clobbers: AX, BX, CX, DX, DI
; =============================================================================
do_for:
        call    spaces
        call    get_var_addr    ; DI = &var (address in VARS array)
        mov     [INS_TMP], di   ; save var_ptr: kw_match inside expr clobbers DI
        ; parse '='
        call    spaces
        cmp     byte [si], '='
        jne     df_syn
        inc     si
        call    expr            ; AX = start value (DI clobbered by kw_match)
        mov     di, [INS_TMP]   ; restore &var
        mov     [di], ax        ; initialise loop variable

        ; parse TO (token or plain keyword)
        call    spaces
        cmp     byte [si], TK_TO
        jne     df_to_kw
        inc     si
        jmp     df_parse_limit
df_to_kw:
        mov     bx, to_tab
        call    kw_match
        jc      df_syn          ; TO is mandatory
df_parse_limit:
        call    expr            ; AX = limit
        mov     cx, ax          ; CX = limit (save while parsing STEP)

        ; parse optional STEP (token or plain keyword); default = 1
        call    spaces
        push    cx              ; save limit: input_number (inside expr) clobbers CX
        cmp     byte [si], TK_STEP
        jne     df_step_kw
        inc     si
        jmp     df_parse_step
df_step_kw:
        mov     bx, step_tab
        call    kw_match        ; CF=1 if no STEP keyword present
        jc      df_default_step ; no STEP: use default 1
df_parse_step:
        call    expr            ; AX = explicit step value
        jmp     df_have_step
df_default_step:
        mov     ax, 1           ; default step (set AFTER kw_match so AH is safe)
df_have_step:
        ; DI=var_ptr  CX=limit  AX=step
        ; Check stack depth
        mov     dx, [FOR_SP]
        cmp     dx, 4
        jnb     df_syn          ; FOR stack overflow -> syntax error

        ; frame offset = FOR_SP * 8
        mov     bx, dx
	mov 	cl, 3
	shl 	bx,cl      
        add     bx, FOR_STK     ; BX -> frame slot

	pop     cx              ; restore limit (was clobbered by input_number in expr)

        ; write frame: var_ptr, limit, step, loop_ptr
        mov     di, [INS_TMP]   ; reload var_ptr (kw_match clobbered DI)
        mov     [bx],   di      ; var_ptr
        mov     [bx+2], cx      ; limit
        mov     [bx+4], ax      ; step
        mov     ax, [RUN_NEXT]
        mov     [bx+6], ax      ; loop_ptr = address of line AFTER FOR

        inc     dx
        mov     [FOR_SP], dx    ; bump depth
        ret

df_syn:
        mov     al, ERR_SN
        jmp     do_error

; =============================================================================
; DO_NEXT  NEXT <var>
; Increments loop var, tests condition, loops or unwinds.
; Inputs : SI -> line body after NEXT token
; Clobbers: AX, BX, CX, DX, DI
; =============================================================================
do_next:
        call    spaces
        call    get_var_addr    ; DI = &var

        ; search FOR_STK for frame matching this var_ptr (scan top-down)
        mov     cx, [FOR_SP]
        or      cx, cx
        jz      dn_no_for       ; stack empty -> NEXT without FOR
dn_search:
        dec     cx
        mov     bx, cx
        add     bx, bx          ; *2
        add     bx, bx          ; *4
        add     bx, bx          ; *8
        add     bx, FOR_STK     ; BX -> candidate frame
        cmp     [bx], di        ; var_ptr match?
        je      dn_found
        or      cx, cx
        jnz     dn_search
        ; fell through without match
dn_no_for:
        mov     al, ERR_NF
        jmp     do_error

dn_found:
        ; BX -> matched frame. CX = frame index.
        ; step var
        mov     ax, [bx+4]      ; AX = step
        add     [di], ax        ; var += step
        mov     ax, [di]        ; AX = new var value
        mov     dx, [bx+2]      ; DX = limit

        ; test: if step >= 0 then continue while var <= limit
        ;       if step <  0 then continue while var >= limit
        cmp     word [bx+4], 0
        jl      dn_neg_step
        ; positive step: var <= limit ?
        cmp     ax, dx
        jle     dn_loop
        jmp     dn_done
dn_neg_step:
        ; negative step: var >= limit ?
        cmp     ax, dx
        jge     dn_loop
        ; fall through to dn_done

dn_done:
        ; loop ended: unwind stack down to and including this frame
        mov     [FOR_SP], cx    ; FOR_SP = matched frame index (pops this + any nested)
        ret                     ; continue past NEXT line

dn_loop:
        ; loop continues: jump back to saved loop_ptr
        mov     ax, [bx+6]      ; loop_ptr = address of line after FOR
        mov     [RUN_NEXT], ax
        ret

; =============================================================================
; Keyword strings (bit-7 terminated; 0x00 sentinel ends do_help table)
; =============================================================================
kw_tab_start:
kw_print:   db 0x50,0x52,0x49,0x4e,T_T
kw_if:      db 0x49,T_F
kw_goto:    db 0x47,0x4f,0x54,T_O
kw_list:    db 0x4c,0x49,0x53,T_T
kw_run:     db 0x52,0x55,T_N
kw_new:     db 0x4e,0x45,T_W
kw_input:   db 0x49,0x4e,0x50,0x55,T_T
kw_rem:     db 0x52,0x45,T_M
kw_end:     db 0x45,0x4e,T_D
kw_let:     db 0x4c,0x45,T_T
kw_poke:    db 0x50,0x4f,0x4b,T_E
kw_free:    db 0x46,0x52,0x45,T_E
kw_help:    db 0x48,0x45,0x4c,T_P
kw_gosub:   db 0x47,0x4f,0x53,0x55,T_B   ; GOSUB
kw_return:  db 0x52,0x45,0x54,0x55,0x52,T_N  ; RETURN
kw_for:     db 0x46,0x4F,T_R                  ; FOR  (F,O,R+0x80)
kw_next:    db 0x4E,0x45,0x58,T_T             ; NEXT (N,E,X,T+0x80)
kw_out:	    db 0xf4,0x55, T_T	
kw_to:      db 0x54,T_O                       ; TO   (T,O+0x80)
kw_step:    db 0x53,0x54,0x45,T_P             ; STEP (S,T,E,P+0x80)
; not commands but still want to print in help
kw_then:    db 0x54,0x48,0x45,T_N
kw_chrs:    db 0x43,0x48,0x52,T_DS
kw_peek:    db 0x50,0x45,0x45,T_K
kw_usr:     db 0x55,0x53,T_R
kw_in:	    db 0x49, T_N

            db 0

; --- Token -> keyword string pointer table (same order as st_tab / TK_xx) ---
; 17 stmt entries + 3 sub-keyword entries
; Stmt (0x80-0x90): PRINT IF GOTO LIST RUN NEW INPUT REM END LET POKE FREE HELP GOSUB RETURN FOR NEXT
; Sub-kw (0x91-0x93): THEN TO STEP (not dispatched by stmt)
tk_kw_tab:
        dw kw_print, kw_if, kw_goto, kw_list, kw_run, kw_new
        dw kw_input, kw_rem, kw_end, kw_let, kw_poke, kw_free
        dw kw_help, kw_gosub, kw_return
        dw kw_for, kw_next, kw_out      ; indices 15,16 -> tokens TK_FOR=0x8F, TK_NEXT=0x90
        dw kw_then, kw_to, kw_step  ; sub-keywords: TK_THEN=0x91, TK_TO=0x92, TK_STEP=0x93
        dw 0                    ; sentinel


; =============================================================================
; Strings (bit 7 terminated)
; =============================================================================
str_banner: db "uBASIC 8088 v1.6.0"
CRLF:	    db 0x0d, 0x0a + 0x80	

; --- Dispatch table: dw kw_ptr, dw handler; sentinel dw 0 -------------------
st_tab:
        dw kw_print,    do_print
        dw kw_if,       do_if
        dw kw_goto,     do_goto
        dw kw_list,     do_list
        dw kw_run,      do_run
        dw kw_new,      do_new
        dw kw_input,    do_input
        dw kw_rem,      do_rem
        dw kw_end,      do_end
        dw kw_let,      do_let
        dw kw_poke,     do_poke
        dw kw_free,     do_free
        dw kw_help,     do_help
        dw kw_gosub,    do_gosub
        dw kw_return,   do_return
        dw kw_for,      do_for
        dw kw_next,     do_next
        dw kw_out,	do_out
        dw 0
        ; non commands but still match
peek_tab:       dw kw_peek
usr_tab:        dw kw_usr
then_tab:       dw kw_then
chrs_tab:       dw kw_chrs
to_tab:         dw kw_to
step_tab:       dw kw_step
in_tab:		dw kw_in

; --- Reset vector at 0xFFF0 -------------------------------------------
; 8086 resets CS=0xFFFF IP=0x0000 -> phys 0xFFFF0.
; We need a FAR JMP to set CS=0xF800 and IP=0x0000.
; JMP FAR 0xF800:0x0000 = EA 00 00 00 F8 (5 bytes)
%ifdef ROM
	org 0xFFF0
%else	; 8bit workshop       
        ; times 94 nop ; bytes free still?
%endif
reset_vec:
%ifdef ROM
        ; ROM cold start: segments, stack, serial, vectors
        xor ax, ax
%else
        ; 8bitworkshop / 8086tiny: CS=DS=ES=SS
        mov ax, cs
%endif
        ; setup segments
        mov ds, ax
        mov es, ax
        mov ss, ax
        mov sp, STACK_TOP
%ifdef ROM
	db 0xEA                 ; far JMP opcode
        dw 0x0000               ; IP = 0x0000
        dw 0xF800               ; CS = 0xF800 -> start
%else	
	jmp start
%endif
