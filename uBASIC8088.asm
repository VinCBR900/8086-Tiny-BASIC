; =============================================================================
; uBASIC 8088  v1.8.2
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Tiny BASIC interpreter for the 8088/8086.  Single-segment, integer-only.
; Targets a 2 KB code ROM + 2 KB RAM embedded system; also runs in
; 8bitworkshop (x86 mode) with a pre-loaded Mandelbrot showcase.
;
; Credit: Oscar Toledo G. for bootBASIC inspiration and TinyASM 8086 assembler.
;         XTulator CPU core by Mike Chambers.
;
; ---------------------------------------------------------------------------
; LANGUAGE REFERENCE
; ---------------------------------------------------------------------------
;
; Statements  : END, FOR..TO [..STEP], NEXT, GOTO, GOSUB, RETURN, IF..THEN,
;               INPUT, LET, PRINT [CHR$(val)] [TAB(n)] [;], POKE, OUT, REM,
;               FREE, HELP, LIST [start,end], NEW, RUN
; Expressions - Arithmetic : + - * / % (Mod)  ^ (power)  unary-
;               Relational  : < > <= >= <>
;               Bitwise     : & (and)  | (or)  ~ (xor)
;               Functions   : ABS(n) IN(port) NOT(n) PEEK(addr) RND(n) USR(addr)
;               Variables   : A..Z (signed 16-bit)
; Numbers     : signed 16-bit  (-32768 .. 32767)
; Multi-stmt  : NOT supported. One statement per line. 
; Errors      : ?0 syntax   ?1 undefined line   ?2 divide/zero   ?3 out of memory
;               ?4 bad variable   ?5 RETURN without GOSUB   ?6 NEXT without FOR
;
; Line store  : <lo> <hi> <tokenised body> <CR>
;   Line numbers 1-32767.
;   PRINT string literals and REM bodies stored verbatim.
;
; ---------------------------------------------------------------------------
; BUILD INSTRUCTIONS
; ---------------------------------------------------------------------------
;
; Assembler: Oscar Toledo's tinyasm  (https://github.com/nanochess/tinyasm)
;   tinyasm -f bin uBASIC8088.asm -o uBASIC_rom.bin
;
; Variant 1: Standalone 2 KB ROM  (real hardware or sim_rom.c simulator)
; ---------------------------------------------------------------------------
;   Hardware:
;     CPU    : Intel 8088 @ 5 MHz (or compatible)
;     ROM    : 2 KB  phys 0xF800-0xFFFF  (address line A12=1 selects ROM)
;     RAM    : 2 KB  phys 0x0000-0x07FF  (address line A12=0 selects RAM)
;     Serial : Intel 8755 MMIO
;                Port A (0x00)  bit 0 = TX (output)   bit 1 = RX (input)
;                DDR A  (0x02)  init: 0xFD  (all outputs except RX)
;              Baud rate: 4800 baud @ 5 MHz  (BAUD = 57 loop constant)
;     Reset  : 8086 reset -> phys 0xFFFF0 -> FAR JMP to 0xF800:0x0000
;
;   Memory map:
;     ORIGIN   = 0xF800       ROM: 0xF800-0xFFFF, reset stub at 0xFFF0
;     RAM_BASE = 0x0000       RAM: 0x0000-0x07FF
;     STACK    = 0x0800       top of RAM, grows downward
;     I/O      = bitbang UART via 8755 Port A
;
;   Simulate (XTulator CPU core by Mike Chambers):
;     gcc -O2 -o sim_rom sim_rom.c cpu.c
;     ./sim_rom uBASIC_rom.bin
;     ./sim_rom uBASIC_rom.bin --trace
;     ./sim_rom uBASIC_rom.bin --cycles 5000000
;
;   Simulator memory model (sim_rom.c):
;     CS=DS=ES=SS = 0x0000  (flat single-segment)
;     addr >= 0xF800  ->  ROM[addr & 0x7FF]
;     addr <  0xF800  ->  RAM[addr & 0x7FF]
;     I/O: output/input_key intercepted at assembled entry points
;
; Variant 2: 8bitworkshop online IDE  (YASM assembler, FREEDOS EXE)
; ---------------------------------------------------------------------------
;   Open directly at https://8bitworkshop.com in 8086 mode.
;   YASM defines __YASM_MAJOR__ which selects this variant automatically.
;   Assembled as a FREEDOS EXE; auto-executes the Mandelbrot showcase.
;
;     I/O      = BIOS INT 10h / INT 16h
;
; =============================================================================
; CHANGE HISTORY
; =============================================================================
;   v1.8.2 (2026-08-14)  28 bytes free
;     - INPUT_LINE: buffer-full keystrokes >62 chars were silently dropped.
;       Added a BELL primitive and sound it once per rejected keystroke when at
;       capacity.
;   v1.8.1 (2026-08-14)  20 bytes free
;     - Added separate demo only mem clear to START instead of calling DO_NEW.
;     - EDITLN: NEW then line entry hang - fixed by trusting raw memory compare 
;       when DI is still within DI < PROG_END.
;     - EDITLN: replacing an existing line (LIST/GOTO-visible number already
;       present) calls DELINE to close the gap first. DELINE's own header
;       documents "Clobbers: ... DX" (DX holds the byte-shift amount for
;       SLIDE_DATA), but EDITLN still read DX as the line number immediately
;       afterward for the INSLINE call, storing the leftover shift amount
;       (old line length, negated) as the line number instead. 
;   v1.8.0 (2026-08-13)  43 bytes free
;     - REMOVED partial multi-statement ':' support to matc 65C02/2650 variants
;     - SHOWCASE: line 220 (D=I:A=C:B=D:E=0) split into 220-223; line 250
;       (B=2*A*B/64+D:A=T) split into 250-251.
;   v1.7.9 (2026-08-13)  24 bytes free  
;     - input_line/ipl_nbs: v1.7.8 made EVERY LF byte silently discarded,
;       Fixed by only discarding first LF of a new line (cx==0).
;   v1.7.8 (2026-08-13)  32 bytes free
;     - input_line: stray LF (0x0A) left over from a CRLF line terminator was
;       falling through to the "ordinary character" path in ipl_nbs -- echoed,
;       stored, and left as the leading byte of the NEXT line read. That byte
;       then failed GET_VAR_ADDR's 'A'..'Z' check on the following command,
;       raising ?4 (bad variable) on any LIST/RUN/numbered-line entry after a
;       prior line was stored via a terminal sending CRLF. Fixed by ignoring
;       0x0A silently in ipl_nbs (not stored, not echoed, not counted).
;     - RAM layout: IBUF declared 64 bytes but RND_SEED located 62 bytes
;       later (0x00C -> 0x04A), overwriting.
;     - tk_kw_tab had no terminating `dw 0` before then_tab/to_tab/step_tab/
;       chrs_tab/tab_tab. TOKENIZE stops on a zero word, so it overan, matching
;       TO/STEP/THEN/CHR$/TAB as statement keywords, corrupting any stored line
;       that used them e.g. `FOR I=1 TO 5` tokenized "TO" to garbage raising 
;       ?3 out-of-memory on RUN. 
;   v1.7.7 (2026-08-13)  29 bytes free
;     - fixed bug where grouping parens used as a factor (e.g. `(-2)^2`,
;       vs ABS(...) func jumped to e2_par instead of eat_paren_expr, skipping
;       consuming the opening '(', SI never advanced, stack smashed.
;     - do_for/do_next: redundant `call spaces` before `call get_var_addr`
;     - updated SHOWCASE for power operator and bitwise ops
;   v1.7.6 (2026-08-12)  [archived as uBASIC8088-v1.7.5.asm]
;     - Added ^ power operator: new expr_pow precedence level, sits above
;       * / % and below factors (correct BODMAS).
;     - Added expr_unary precedence level (between expr1 and expr_pow) so a
;       leading unary minus wraps the whole power expression: -2^2 = -4,
;       not (-2)^2. Existing unary handling inside expr2 is unchanged, so
;       2^-3 (sign on the exponent) still parses as before.
;     - Bitwise XOR moved from ^ to ~ (freed ^ for power; & and | unchanged).
;     - Removed DELAY statement to make space for power.
;   v1.7.5 (2026-05-12)  Bug fixes and size optimisations:
;     - Updated Showcase tokens and rmeoved spaces
;     - LIST: clean up before/after token spaces
;     - Removed dead dw 0 sentinel after step_tab (2 bytes saved).
;     - kw_match: removed redundant '_' boundary check (3 bytes saved).
;     - do_for: stack overflow now reports ERR_OM (?3) not ERR_SN (?0).
;     - get_var_addr: removed redundant double-read of [SI].
;     - Subroutine headers normalised; formatting pass throughout.
;   v1.7.4 (2026-05-09)  [archived as uBASIC8088-v1.7.4.asm]
;     eat_paren_expr bugfix.  Size optimisations: STMT dispatcher,
;     DO_LET/DO_INPUT shared sections.  Added ^ XOR bitwise, NOT()/RND().
;     Removed INT 0/2h vectors for space.
;   v1.7.3 (2026-05-09)  eat_paren_expr bugfix, size optimisations.
;   v1.7.2 (2026-05-02)  Refactored operator table, added & | bitwise ops.
;     Added PRINT TAB() and ABS(); fixed NEXT error; multi-statement : fixes.
;   v1.7.1 (2026-05-02)  Size optimisation (11 bytes saved, 67->78 bytes free).
;   v1.7.0 (2026-05-01)  LIST range, EXPR2 function dispatch table refactor.
;   v1.6.1 (2026-04-30)  Fix sbb ax,ax clobbering flags before jl (all signed
;     comparisons were wrong).  Showcase tokens updated for TK_OUT insertion.
;   v1.6.0 (2026-04-26)  Added IN / OUT; statement dispatch refactor.
;   v1.5.0 (2026-04-23)  ROM target: bitbang UART, IVT, reset stub, bdly.
;   v1.4.0 (2026-04-19)  FOR/NEXT/STEP; 4-entry FOR stack; error ?6.
;   v1.3.1 (2026-04-17)  LIST CR+LF fix; showcase in tokenised form.
;   v1.3.0 (2026-04-17)  Tokeniser + line-editor refactor.
;   v1.2.0 (2026-04-17)  GOSUB / RETURN; 8-entry stack; error ?5.
;   v1.1.0 (2026-04-17)  Bug fixes (relational xor, do_new, tinyasm compat).
;   v1.0.0 (2026-04-09)  First release: 8088 port of uBASIC 65c02 v17.0.
; =============================================================================

        cpu 8086

; =============================================================================
; PLATFORM CONFIGURATION
; =============================================================================

ORIGIN:         equ 0xF800              ; ROM base (also YASM segment)
RAM_BASE:       equ 0x0000
RAM_SIZE:       equ 2048                ; 2 KB RAM (A12=0 selects RAM)
MAX_BUF:        equ 62                  ; maximum input length

; =============================================================================
; RAM LAYOUT  (all offsets relative to RAM_BASE)
; =============================================================================

DIV0:           equ RAM_BASE + 0x000    ; 4 bytes : divide-by-zero IVT entry
CURLN:          equ RAM_BASE + 0x004    ; word    : current line# for error reports
RUN_NEXT:       equ RAM_BASE + 0x006    ; word    : next-line pointer for run loop
NMI:            equ RAM_BASE + 0x008    ; 4 bytes : NMI IVT entry
IBUF:           equ RAM_BASE + 0x00C    ; 64 bytes: input line buffer
RND_SEED:       equ RAM_BASE + 0x04C    ; word    : LFSR random seed
INS_TMP:        equ RAM_BASE + 0x04E    ; word    : insline / do_for var_ptr scratch
GOSUB_SP:       equ RAM_BASE + 0x050    ; word    : GOSUB stack depth (0..7)
GOSUB_STK:      equ RAM_BASE + 0x052    ; 16 bytes: 8-entry GOSUB return-address stack
FOR_SP:         equ RAM_BASE + 0x062    ; word    : FOR stack depth (0..3)
FOR_STK:        equ RAM_BASE + 0x064    ; 32 bytes: 4 x 8-byte FOR frames
VARS:           equ RAM_BASE + 0x084    ; 52 bytes: variables A-Z (word each)
RUNNING:        equ RAM_BASE + 0x0B8    ; byte    : 0=immediate mode, 1=running
PROG_END:       equ RAM_BASE + 0x0B9    ; word    : one past last program byte
PROGRAM:        equ RAM_BASE + 0x0BB    ; program store start
STACK_TOP:      equ RAM_BASE + RAM_SIZE ; initial SP (grows downward)
PROGRAM_TOP:    equ STACK_TOP - 0x100   ; 256-byte stack reserve

; =============================================================================
; ERROR CODES  (printed as "?N")
; =============================================================================

ERR_SN:         equ 0x30        ; ?0 Syntax error
ERR_UL:         equ 0x31        ; ?1 Undefined line
ERR_OV:         equ 0x32        ; ?2 Overflow / divide by zero
ERR_OM:         equ 0x33        ; ?3 Out of memory
ERR_UK:         equ 0x34        ; ?4 Bad variable
ERR_RT:         equ 0x35        ; ?5 RETURN without GOSUB
ERR_NF:         equ 0x36        ; ?6 NEXT without FOR
ERR_BRK:        equ 0x42        ; ?B NMI break (ROM version, no room) 

; =============================================================================
; KEYWORD TERMINATOR CONSTANTS  (last byte of keyword string = ASCII | 0x80)
; =============================================================================

T_B:            equ 0xC2        ; 'B'+0x80  used by: GOSUB, TAB
T_D:            equ 0xC4        ; 'D'+0x80  used by: END
T_E:            equ 0xC5        ; 'E'+0x80  used by: POKE, FREE
T_F:            equ 0xC6        ; 'F'+0x80  used by: IF
T_K:            equ 0xCB        ; 'K'+0x80  used by: PEEK
T_M:            equ 0xCD        ; 'M'+0x80  used by: REM
T_N:            equ 0xCE        ; 'N'+0x80  used by: RUN, THEN
T_O:            equ 0xCF        ; 'O'+0x80  used by: GOTO
T_P:            equ 0xD0        ; 'P'+0x80  used by: HELP
T_R:            equ 0xD2        ; 'R'+0x80  used by: USR
T_S:            equ 0xD3        ; 'S'+0x80  used by: ABS
T_T:            equ 0xD4        ; 'T'+0x80  used by: PRINT, LIST, INPUT, LET, NEXT, NOT
T_W:            equ 0xD7        ; 'W'+0x80  used by: NEW
T_DS:           equ 0xA4        ; '$'+0x80  used by: CHR$

; =============================================================================
; TOKEN BYTES  (0x80+ stored in program lines; order matches st_tab)
; =============================================================================

TK_PRINT:       equ 0x80        ; --- statement tokens (dispatched by stmt) ---
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
TK_FOR:         equ 0x8F
TK_NEXT:        equ 0x90
TK_OUT:         equ 0x91
NUM_TOKENS:     equ 18          ; count: TK_PRINT (0x80) .. TK_OUT (0x91)

TK_THEN:        equ 0x93        ; --- sub-keywords (not in st_tab, not dispatched) ---
TK_TO:          equ 0x94
TK_STEP:        equ 0x95

; =============================================================================
; ROM BITBANG SERIAL  (Intel 8755 Port A, 4800 baud @ 5 MHz)
; =============================================================================

PORT_A:         equ 0x00        ; 8755 Port A data register
DDR_A:          equ 0x02        ; 8755 Port A direction register
TX:             equ 0x01        ; Port A bit 0 = TX (output)
RX:             equ 0x02        ; Port A bit 1 = RX (input)
BAUD:           equ 57          ; bit-period loop count: 17 cy/iter @5MHz ~4800 baud

; =============================================================================
; SHOWCASE DATA  (8bitworkshop / YASM build only)
;
; Pre-loaded program.  Type RUN to execute, NEW to clear.
;   Lines  10-190 : feature demos (arithmetic, comparisons, FOR/NEXT, GOSUB)
;   Lines 200-330 : Mandelbrot (fixed-point 1/64, 16 iterations, ASCII density)
;   Lines 500-540 : subroutine: sum 1..10
;   Lines 550-590 : subroutine: factorial 5
;   Lines 600-610 : subroutine: Mandelbrot escape recorder
;
; Token map (v1.7.6):
;   PRINT=0x80  IF=0x81  GOSUB=0x8D  RETURN=0x8E  END=0x88
;   FOR=0x8F    NEXT=0x90  OUT=0x91
;   THEN=0x93   TO=0x94    STEP=0x95  REM=0x87
; =============================================================================
	ORG 0
%ifdef __YASM_MAJOR__
        ; Trampoline: 8bitworkshop needs a near jump it can overwrite
        mov  ax, start
        jmp  ax
%endif

        times PROGRAM - ($-$$) db 0     ; pad over VARS / equate area

SHOWCASE_DATA:
        ; ── Feature demos ──────────────────────────────────────────────────────
        db 0x0A,0x00, 0x87,"uBASIC 8088 v1.8.0 showcase",0x0D            ; 10  REM
        db 0x14,0x00, 0x80,0x22,"--- ARITHMETIC ---",0x22,0x0D            ; 20  PRINT
        db 0x1E,0x00, 0x80,0x22,"2+3=",0x22,";2+3;",0x22,"  6*7=",0x22,";6*7",0x0D      ; 30
        db 0x28,0x00, 0x80,0x22,"20/4=",0x22,";20/4;",0x22,"  17%5=",0x22,";17%5",0x0D  ; 40
        db 0x29,0x00, 0x80,0x22,"3^4=",0x22,";3^4;",0x22,"  2^10=",0x22,";2^10",0x0D    ; 41  power
        db 0x2A,0x00, 0x80,0x22,"--- BITWISE ---",0x22,0x0D                             ; 42
        db 0x2B,0x00, 0x80,0x22,"12 & 10=",0x22,";12&10;",0x22,"  12 | 10=",0x22,";12|10",0x0D ; 43  AND/OR
        db 0x2C,0x00, 0x80,0x22,"12 XOR 10=",0x22,";12~10",0x0D                             ; 44  XOR
        db 0x32,0x00, 0x80,0x22,"--- COMPARISONS ---",0x22,0x0D           ; 50
        db 0x3C,0x00, 0x81,"5>3",0x93,0x80,0x22,"5>3 ok",0x22,0x0D      ; 60  IF THEN(0x93) PRINT
        db 0x46,0x00, 0x81,"3<5",0x93,0x80,0x22,"3<5 ok",0x22,0x0D      ; 70
        db 0x50,0x00, 0x81,"3>=3",0x93,0x80,0x22,"3>=3 ok",0x22,0x0D    ; 80
        db 0x5A,0x00, 0x81,"4<>3",0x93,0x80,0x22,"4<>3 ok",0x22,0x0D    ; 90
        db 0x64,0x00, 0x80,0x22,"--- FOR/NEXT ---",0x22,0x0D              ; 100
        db 0x6E,0x00, 0x8F,"I=1",0x94,"5",0x0D                           ; 110 FOR I=1 TO(0x94) 5
        db 0x78,0x00, 0x80,"I;",0x0D                                       ; 120 PRINT I;
        db 0x82,0x00, 0x90,"I",0x0D                                        ; 130 NEXT I
        db 0x8C,0x00, 0x80,0x22,0x22,0x0D                                  ; 140 PRINT ""
        db 0x96,0x00, 0x80,0x22,"--- GOSUB ---",0x22,0x0D                 ; 150
        db 0xA0,0x00, 0x8D,"500",0x0D                                      ; 160 GOSUB 500
        db 0xA5,0x00, 0x80,0x22,"sum 1..10=",0x22,";S",0x0D               ; 165 PRINT result
        db 0xAA,0x00, 0x8D,"550",0x0D                                      ; 170 GOSUB 550
        db 0xAF,0x00, 0x80,0x22,"5!=",0x22,";F",0x0D                      ; 175 PRINT result
        db 0xB4,0x00, 0x80,0x22,0x22,0x0D                                  ; 180 PRINT ""
        ; ── Mandelbrot ─────────────────────────────────────────────────────────
        db 0xBE,0x00, 0x80,0x22,"--- MANDELBROT ---",0x22,0x0D            ; 190
        db 0xC8,0x00, 0x8F,"I=-64",0x94,"56 ",0x95,"6",0x0D              ; 200 FOR I=-64 TO(0x94) 56 STEP(0x95) 6
        db 0xD2,0x00, 0x8F,"C=-128 ",0x94,"16 ",0x95,"4",0x0D             ; 210 FOR C=-128 TO(0x94) 16 STEP(0x95) 4
        db 0xDC,0x00, "D=I",0x0D                                           ; 220 init row
        db 0xDD,0x00, "A=C",0x0D                                           ; 221
        db 0xDE,0x00, "B=D",0x0D                                           ; 222
        db 0xDF,0x00, "E=0",0x0D                                           ; 223
        db 0xE6,0x00, 0x8F,"N=1",0x94,"16",0x0D                          ; 230 FOR N=1 TO(0x94) 16
        db 0xF0,0x00, "T=A^2/64-B^2/64+C",0x0D                            ; 240 iterate
        db 0xFA,0x00, "B=2*A*B/64+D",0x0D                                  ; 250
        db 0xFB,0x00, "A=T",0x0D                                           ; 251
        db 0x04,0x01, 0x81,"A^2/64+B^2/64>256",0x93,0x8D,"600",0x0D        ; 260 IF THEN(0x93) GOSUB 600
        db 0x0E,0x01, 0x90,"N",0x0D                                        ; 270 NEXT N
        db 0x18,0x01, 0x81,"E>0",0x93,0x80,"CHR$(E+32);",0x0D              ; 280 IF E>0 THEN(0x93) PRINT
        db 0x22,0x01, 0x81,"E=0",0x93,0x80,"CHR$(32);",0x0D                ; 290 IF E=0 THEN(0x93) PRINT
        db 0x2C,0x01, 0x90,"C",0x0D                                        ; 300 NEXT C
        db 0x36,0x01, 0x80,0x0D                                        	   ; 310 PRINT (newline)
        db 0x40,0x01, 0x90,"I",0x0D                                        ; 320 NEXT I
        db 0x4A,0x01, 0x88,0x0D                                            ; 330 END
        ; ── Subroutine 500: sum 1..10 ──────────────────────────────────────────
        db 0xF4,0x01, "S=0",0x0D                                           ; 500
        db 0xFE,0x01, 0x8F,"J=1",0x94,"10",0x0D                            ; 510 FOR J=1 TO(0x94) 10
        db 0x08,0x02, "S=S+J",0x0D                                         ; 520
        db 0x12,0x02, 0x90,"J",0x0D                                        ; 530 NEXT J
        db 0x1C,0x02, 0x8E,0x0D                                            ; 540 RETURN
        ; ── Subroutine 550: factorial 5 ────────────────────────────────────────
        db 0x26,0x02, "F=1",0x0D                                           ; 550
        db 0x30,0x02, 0x8F,"K=1",0x94,"5",0x0D                             ; 560 FOR K=1 TO(0x94) 5
        db 0x3A,0x02, "F=F*K",0x0D                                         ; 570
        db 0x44,0x02, 0x90,"K",0x0D                                        ; 580 NEXT K
        db 0x4E,0x02, 0x8E,0x0D                                            ; 590 RETURN
        ; ── Subroutine 600: record Mandelbrot escape iteration ─────────────────
        db 0x58,0x02, 0x81,"E=0 ",0x93,"E=N",0x0D                         ; 600 IF E=0 THEN(0x93) E=N
        db 0x62,0x02, 0x8E,0x0D                                            ; 610 RETURN
        dw 0                                                                ; end sentinel
SHOWCASE_END:

        ; YASM does not like multiple ORGs 
;        org ORIGIN
	times ORIGIN-($-$$) db 0

; =============================================================================
; INIT  cold start
; Inputs  : (reset state)
; Outputs : (falls through to main_loop)
; Clobbers: everything
; =============================================================================
start:
%ifdef __YASM_MAJOR__
        cld
        mov  ax, cs             ; EXE: normalise DS/ES/SS to CS (FREEDOS leaves at PSP)
%else
        ; ROM: CS=0xF800 after far JMP.  RAM at segment 0.
        xor  ax, ax
%endif
        cli
        ; setup 8088 segments for tiny system
        mov  ds, ax
        mov  es, ax
        mov  ss, ax
        mov  sp, STACK_TOP
        ; Clear variable mem
        call clr_vars

        ; Protect showcase - Delete next lines and instead call DO_NEW for real ROM 
        ; PROG_END: just past last showcase byte (excluding sentinel)
        mov  word [PROG_END], SHOWCASE_END - 2

        ; Signon banner; fall through to main_loop
        mov  si, str_banner
        call dp_str
        call do_free
        ; drop through
; =============================================================================
; MAIN_LOOP  prompt / read / dispatch
; Inputs  : (none — top-level loop)
; Clobbers: everything
; =============================================================================
main_loop:
        mov  sp, STACK_TOP
        call do_end             ; clear RUNNING

        mov  al, '>'
        call output

        call input_line         ; read line; SI -> IBUF
        call peek_line
        je   main_loop          ; blank line: re-prompt

        call input_number       ; parse optional line number -> AX
        or   ax, ax
        jne  ml_numbered
        call stmt               ; no line number: execute immediately
        jmp  short main_loop
ml_numbered:
        call editln             ; numbered line: store/edit in program
        jmp  short main_loop

; =============================================================================
; DO_ERROR  print "?N[@line]", then restart main loop — never returns
; Inputs  : AL = error code character ('0'..'6', 'B', ...)
; Clobbers: everything
; =============================================================================
do_error:
        push ax
        call new_line
        call question
        pop  ax
        call output             ; print "?N"
        cmp  byte [RUNNING], 0
        je   do_error_nl
        mov  al, '@'
        call output
        mov  ax, [CURLN]
        call output_number
do_error_nl:
        call new_line
        jmp  main_loop

; =============================================================================
; DO_IF_FALSE  skip remainder of line (IF condition was false)
; Inputs  : SI -> chars after condition
; Outputs : SI -> CR (not consumed)
; Clobbers: AX, SI
; =============================================================================
do_if_false:
        lodsb
        cmp  al, 0x0D
        jne  do_if_false
        dec  si                 ; leave CR for caller
        ; fall through to peek_line

; =============================================================================
; PEEK_LINE  test whether SI is at end-of-statement (CR)
; No multi-statement lines: CR is the only end-of-statement marker.
; Inputs  : SI -> current position
; Outputs : ZF=1 at CR, ZF=0 otherwise
; Clobbers: (none)
; =============================================================================
peek_line:
        call spaces
        cmp  byte [si], 0x0D
sl_ret:
        ret

; =============================================================================
; DO_IF  IF <expr> [THEN] <stmt>
; Handles tokenised THEN (TK_THEN = 0x93) and plain-text THEN.
; Inputs  : SI -> expression text
; Clobbers: AX, BX, SI (via expr / stmt)
; =============================================================================
do_if:
        call expr
        or   ax, ax
        je   do_if_false
        call spaces
        cmp  byte [si], TK_THEN
        jne  di_kw_then
        inc  si                 ; consume token
        jmp  short stmt

di_kw_then:
        mov  bx, then_tab       ; THEN is optional in direct mode
        call kw_match
        ; fall through to stmt

; =============================================================================
; STMT  execute one statement from SI
; Token fast-path: stored programs use keyword tokens (0x80+).
; Direct-mode input falls through to the kw_match loop.
; Inputs  : SI -> statement (token byte or raw text)
; Clobbers: AX, BX, CX, DX, SI, DI
; =============================================================================
stmt:
        call peek_line
        je   sl_ret
        mov  al, [si]
        cmp  al, TK_PRINT               ; below token range?
        jb   stmt_text
        cmp  al, TK_PRINT + NUM_TOKENS  ; above dispatchable range?
        jnb  stmt_text

        ; Token fast-path (stored programs)
        inc  si                         ; consume token byte
        mov  bx, st_tab
        call get_token_ptr              ; BX -> st_tab entry
        jmp  word [bx]

        ; Text fall-through (direct mode)
stmt_text:
        mov  bx, tk_kw_tab
        mov  cx, NUM_TOKENS
stmt_lp:
        call kw_match
        jnc  stmt_call
        add  bx, 2
        loop stmt_lp
        jmp  do_let                     ; no keyword -> implicit LET
stmt_call:
        sub  bx, tk_kw_tab              ; BX = index * 2
        jmp  word [bx + st_tab]

; =============================================================================
; GET_TOKEN_PTR  map token byte to table-entry address
; Inputs  : AL = token byte (>= TK_PRINT),  BX = table base
; Outputs : BX = &table[token - TK_PRINT]
; Clobbers: AX
; =============================================================================
get_token_ptr:
        sub  al, TK_PRINT       ; 0-based index
        cbw
        add  ax, ax             ; * 2 (word table)
        add  bx, ax
        ret

; =============================================================================
; DO_LET  [LET] <var> = <expr>
; Inputs  : SI -> variable name
; Clobbers: AX, DI, SI
; =============================================================================
do_let:
        call get_var_addr       ; DI = &var, SI advanced
        push di
        call expect_equals      ; consume '='
        call expr               ; AX = result
        jmp  short var_store

; =============================================================================
; DO_INPUT  INPUT <var>
; Inputs  : SI -> variable name
; Clobbers: AX, DI, SI
; =============================================================================
do_input:
        call get_var_addr       ; DI = &var, SI advanced
        push di
        call question
        push si                 ; save program pointer
        call input_line         ; resets SI -> IBUF
        call expr               ; parse number -> AX
        pop  si                 ; restore program pointer
        ; fall through to var_store

; =============================================================================
; VAR_STORE  shared assignment tail for DO_LET and DO_INPUT
; Inputs  : AX = value to store, DI = &var on stack
; Clobbers: DI
; =============================================================================
var_store:
        pop  di
        stosw
        ret

; =============================================================================
; GET_VAR_ADDR  validate and address a single-letter variable A-Z
; Inputs  : SI -> variable letter (leading spaces are skipped)
; Outputs : DI = &VARS[var], SI advanced past letter
; Clobbers: AX, DI
; =============================================================================
JERRUK:
        mov  al, ERR_UK
        jmp  do_error
get_var_addr:
        call spaces
        lodsb                   ; read and advance in one step
        call uc_al
        cmp  al, 'A'
        jb   JERRUK
        cmp  al, 'Z'
        ja   JERRUK
        sub  al, 'A'
        cbw
        add  ax, ax             ; word offset
        add  ax, VARS
        xchg di, ax
        ret

; =============================================================================
; EXPECT_TOKEN_OR_KW  match a sub-keyword by token byte or plain text
; Inputs  : AL = token value (e.g. TK_TO), BX -> keyword table entry (text match)
; Outputs : CF=0 matched (SI advanced), CF=1 no match
; Clobbers: AX, DI, DL
; =============================================================================
expect_token_or_kw:
        call spaces
        cmp  byte [si], al
        je   etk_match
;        jmp  kw_match   ; tail call
        ; ret
        ; drop through
; =============================================================================
; KW_MATCH  case-insensitive keyword match at [SI]
; Inputs  : BX -> table entry (word = pointer to bit-7-terminated keyword string)
;           SI -> input text
; Outputs : CF=0 matched (SI advanced past keyword)
;           CF=1 no match (SI unchanged)
; Clobbers: AX, DI, DL
; =============================================================================
kw_match:
        push si
        call spaces
        mov  di, [bx]           ; DI -> keyword string
.match_lp:
        mov  al, [di]
        inc  di
        mov  dl, al             ; DL: char + bit-7 end-of-word flag
        and  al, 0x7F
        call uc_al
        mov  ah, al             ; AH = uppercased keyword char
        lodsb                   ; AL = input char, SI++
        call uc_al
        cmp  al, ah
        jne  .fail
        test dl, 0x80           ; last keyword char?
        jz   .match_lp

        ; Boundary check: reject prefix match (e.g. "IF" vs "IFFY")
        ; '_' check removed — not a valid BASIC identifier char (saves 3 bytes)
        mov  al, [si]
        call uc_al
        cmp  al, 'A'
        jb   .check_num
        cmp  al, 'Z'
        jbe  .fail              ; A-Z: still a word
.check_num:
        cmp  al, '0'
        jb   .ok
        cmp  al, '9'
        jbe  .fail              ; 0-9: still a word
.ok:
        pop  ax                 ; discard saved SI
        ;clc
        ;ret
        DB 0xB0 ; swallow next byte
etk_match:
        inc  si
        clc
        ret
kw_match.fail:
        pop  si
        stc
        ret

; =============================================================================
; UC_AL  convert AL to uppercase if it is a lowercase letter
; Inputs  : AL = character
; Outputs : AL = uppercase equivalent (unchanged if not a-z)
; Clobbers: (none)
; =============================================================================
uc_al:
        cmp  al, 'a'
        jb   uc_al_r
        cmp  al, 'z'
        ja   uc_al_r
        and  al, 0xDF
uc_al_r:
dl_done:
        ret

; =============================================================================
; DO_LIST  LIST [<start>,<end>]
; Both arguments must be supplied together if used.
; Inputs  : SI -> optional range arguments
; Clobbers: AX, BX, CX, DX, SI, DI, BP
; =============================================================================
do_list:
        mov  di, PROGRAM
        mov  bp, 0x7FFF         ; default: all lines
        call peek_line
        je   dl_lp              ; bare LIST
        call poke_out_hlpr      ; DI = start addr, AX = end line#
        xchg bp, ax             ; BP = end line#
        mov  ax, di             ; start line# (left in DI by poke_out_hlpr)
        call find_line          ; DI -> first line >= start
dl_lp:
        mov  ax, [di]
        test ax, ax
        jz   dl_done
        cmp  bp, ax
        jl   dl_done
	call num_space
        lea  si, [di+2]
        xor  dx, dx             ; DL = 0: nothing printed yet for this line
dl_body:
        lodsb
        cmp  al, 0x0D
        je   dl_eol
        cmp  al, TK_PRINT       ; Is it a token?
        jb   dl_raw

        ; --- TOKEN HANDLING ---
        push si
        push ax                 ; Save current token

        ; Check for leading space - Don't print if:  
        test dl, dl		; 1. Start of line (DL=0)
        jz   .skip_leading
        cmp  dl, ' '		; 2. Prev was space (DL=' ')
        je   .skip_leading
        cmp  dl, 1		; 3. Prev was a token (DL=1)
        je   .skip_leading
        call output_space

.skip_leading:
        pop  ax                 ; Restore current token
        mov  bx, tk_kw_tab
        call get_token_ptr
        mov  si, [bx]
        call dp_str             ; Print the keyword
        call output_space

        pop  si
        mov  dl, 1              ; Set state: "Last thing was a token"
        jmp  dl_body

dl_raw:
        call output
        mov  dl, al             ; Store the actual char printed (e.g., ' ', '=', etc.)
        jmp  dl_body
dl_eol:
        call new_line
        call next_line_ptr
        jmp  dl_lp

; =============================================================================
; DO_PRINT  PRINT [item [; item] ...]
; Items: "string literal", CHR$(n), TAB(n), expression.
; Trailing ';' suppresses CR+LF.
; Also used by dp_str to output bit-7-terminated ROM strings.
; Inputs  : SI -> print list
; Clobbers: AX, BX, CX, SI
; =============================================================================
do_print:
        call peek_line
        je   dp_nl              ; bare PRINT -> newline
dp_top:
        cmp  byte [si], '"'
        jne  dp_chrs
        inc  si                 ; skip opening quote
dp_str:
        lodsb
        cmp  al, 0x22           ; closing '"'?
        je   dp_after
        test al, 0x80           ; bit-7 terminator (ROM string)?
        pushf                   ; Save flags (ZF=1 if bit 7 is clear)
        and  al, 0x7F           ; Mask off terminator bit
        call output
        popf                    ; Restore flags
        jz   dp_str             ; Loop if it wasn't the terminator
        ret                     ; Return to caller (do_help/do_free)

dp_chrs:
        mov  bx, chrs_tab
        call kw_match
        jc   dp_tab
        call eat_paren_expr
        call output
        jmp  short dp_after

dp_tab:
        mov  bx, tab_tab
        call kw_match
        jc   dp_num
        call eat_paren_expr
        xchg ax, cx
tab_loop:
        call output_space
        loop tab_loop
        jmp  short dp_after

dp_num:
        call expr
        call output_number
        ; Falls through to dp_after
        
dp_after:
        call spaces
        cmp  byte [si], ';'
        jne  dp_nl
        inc  si
        call peek_line          ; Check if anything follows ';'
        jne  dp_top             ; If not end of line, loop back up
dp_ret:
        ret                     ; End of line after ';', return without newline

; =============================================================================
; DO_FREE  print free program-store bytes 
; (Moved BEFORE do_help to allow do_help to fall through to dp_nl)
; Inputs  : (none)
; Clobbers: AX, SI
; =============================================================================
do_free:
        mov  ax, PROGRAM_TOP
        sub  ax, [PROG_END]
        call num_space
        mov  si, kw_free
        call dp_str
        jmp  short dp_nl        ; Jump to the shared newline tail-call

; =============================================================================
; DO_HELP  print all keywords
; Inputs  : (none)
; Clobbers: AX, SI
; =============================================================================
do_help:
        mov  si, kw_tab_start
dh_lp:
        call dp_str
        call output_space
        cmp  byte [si], 0       ; sentinel?
        jne  dh_lp
dp_nl:
        jmp new_line           ; tail-call

; =============================================================================
; POKE_OUT_HLPR  parse "<addr>, <val>" pair shared by DO_POKE and DO_OUT
; Inputs  : SI -> argument text
; Outputs : DI = address, AL = value (low byte)
; Clobbers: AX, BX, CX, DX, SI, DI
; =============================================================================
poke_out_hlpr:
        call expr               ; AX = address
        push ax
        mov  al, ','
        call expect
        call expr               ; AX = value
        pop  di                 ; DI = address
        ret

; =============================================================================
; DO_POKE  POKE <addr>, <val>
; Inputs  : SI -> argument text
; Clobbers: AX, BX, CX, DX, SI, DI
; =============================================================================
do_poke:
        call poke_out_hlpr
        stosb
        ret

; =============================================================================
; DO_OUT  OUT <port>, <val>
; Inputs  : SI -> argument text
; Clobbers: AX, BX, CX, DX, SI, DI, DX
; =============================================================================
do_out:
        call poke_out_hlpr
        mov  dx, di
        out  dx, al
        ret

; =============================================================================
; EXPECT_EQUALS  consume '=' at [SI], else syntax error
; Inputs  : SI -> current position
; Outputs : SI advanced past '='
; Clobbers: AX
; =============================================================================
expect_equals:
        mov  al, '='
        ; fall through to expect

; =============================================================================
; EXPECT  consume expected character at [SI], else syntax error
; Inputs  : AL = expected character, SI -> current position
; Outputs : SI advanced past the character
; Clobbers: (none beyond AL)
; =============================================================================
expect:
        call spaces
        cmp  [si], al
        jne  JERRSN
        inc  si
sp_r:
        ret

; =============================================================================
; SPACES  skip space characters (need to preserve AX, BX, CX, DX)
; Inputs  : SI -> current position
; Outputs : SI advanced past spaces
; Clobbers: (none) - lodsb breaks AL
; =============================================================================
spaces:
        cmp  byte [si], ' '
        jne  sp_r
        inc  si
        jmp  short spaces

; =============================================================================
; EXPR  evaluate expression including relational operators
; Returns true = 0xFFFF, false = 0x0000.
; Inputs  : SI -> expression text
; Outputs : AX = signed 16-bit result
; Clobbers: AX, BX, CX, DX, SI
; =============================================================================
expr:
        call expr_bitwise       ; left operand -> AX
        push ax
        call spaces

        ; Accumulate relational operator bitmask: LT=1 EQ=2 GT=4
        xor  dx, dx
.op_loop:
        lodsb
        cmp  al, '<'
        jne  .not_lt
        or   dl, 1
        jmp  short .op_loop
.not_lt:
        cmp  al, '='
        jne  .not_eq
        or   dl, 2
        jmp  short .op_loop
.not_eq:
        cmp  al, '>'
        jne  .not_gt
        or   dl, 4
        jmp  short .op_loop
.not_gt:
        dec  si                 ; back up: non-relational char
        test dl, dl
        jnz  .do_rel
        pop  ax                 ; no relational operator: return left
        ret

.do_rel:
        push dx
        call expr_bitwise       ; right operand -> AX
        pop  dx
        pop  bx                 ; BX = left operand
        cmp  bx, ax
        mov  ax, 2              ; assume equal
        jz   .check
        jl   .set_lt
        mov  al, 4              ; GT
        jmp  short .check
.set_lt:
        mov  al, 1              ; LT
.check:
        test al, dl
        mov  ax, 0
        jz   .done
        dec  ax                 ; -1 = 0xFFFF = true
.done:
e1_ret:
ea_ret:
        ret

; =============================================================================
; CENTRAL ERROR ENTRIES
; =============================================================================
JERRSN:
        mov  al, ERR_SN
        jmp  do_error
        db   0xBB               ; opcode prefix: absorbs next 2 bytes as "mov bx,imm"
div_err:
        mov  al, ERR_OV
        jmp  do_error

; =============================================================================
; MATH PRIMITIVES  (invoked via prec_engine dispatch table)
; Inputs  : AX = left operand, CX = right operand
; Outputs : AX = result
; Clobbers: DX (math_div, math_mod only)
; =============================================================================
math_add:
        add  ax, cx
        ret

math_sub:
        sub  ax, cx
        ret

math_mul:
        imul cx
        ret

; =============================================================================
; DO_RND_FUNC  RND(n) -> pseudo-random value in [0, n)
; Inputs  : AX = limit n (from eat_paren_expr)
; Outputs : AX = value in range [0, n)
; Clobbers: BX, CX, DX
; =============================================================================
do_rnd_func:
        push ax                 ; save limit
        call rnd_shuffle        ; advance LFSR -> AX
        pop  cx                 ; CX = limit
        ; drop through          ; returns AX % CX
math_mod:
        call math_div
        xchg ax, dx             ; return remainder
        ret

math_div:
        or   cx, cx
        je   div_err
        cwd
        idiv cx
        ret

; =============================================================================
; MATH_POW  integer exponentiation (repeated signed multiply)
; Inputs  : AX = base, CX = exponent
; Outputs : AX = base^exponent (16-bit signed, wraps silently on overflow
;           like the other math primitives)
; Clobbers: BX
; Errors  : negative exponent -> ERR_OV (?2), integer BASIC has no fractional
;           result to return
; =============================================================================
math_pow:
        or   cx, cx
        js   div_err             ; negative exponent: undefined for int BASIC
        xchg bx, ax              ; BX = base
        mov  ax, 1               ; accumulator (x^0 = 1)
        jcxz .done
.lp:
        imul bx
        loop .lp
.done:
        ret

expr_bitwise:                   ; lowest precedence
        mov  bx, tab_bitwise
        mov  di, expr_add
        jmp  short prec_engine

expr_add:
        mov  bx, tab_add
        mov  di, expr1
        jmp  short prec_engine

expr1:
        mov  bx, tab_mul
        mov  di, expr_unary
        jmp  short prec_engine

bitwise_or:
        or   ax, cx
        ret

bitwise_xor:
        xor  ax, cx
        ret

bitwise_and:
        and  ax, cx
        ret

; =============================================================================
; EXPR_UNARY  unary +/- wrapping a whole power-expression (correct BODMAS:
; binds looser than ^, so -2^2 = -4, not (-2)^2)
; Inputs  : SI -> expression text
; Outputs : AX = value
; Clobbers: AX, BX, CX, DX, SI
; =============================================================================
eu_neg:
        inc  si
        call expr_unary
        jmp short jnegax

eu_pos:
        inc  si
expr_unary:
        call spaces
        mov  al, [si]
        cmp  al, '-'
        je   eu_neg
        cmp  al, '+'
        je   eu_pos
expr_pow:
        mov  bx, tab_pow
        mov  di, expr2          ; functions are highest precedence
        ; fall through to prec_engine

; =============================================================================
; PREC_ENGINE  generic left-associative binary operator evaluator
; Inputs  : BX -> operator table {char(1), handler_ptr(2), ...}, 0x00 sentinel
;           DI = pointer to next-lower-precedence function
; Clobbers: AX, BX, CX, DX, SI (via recursive sub-calls)
; =============================================================================
prec_engine:
        push bx                 ; save operator table pointer
        push di                 ; save next-level function pointer
        call di                 ; get initial LHS
.lp:
        mov  bp, sp
        mov  di, [bp]           ; DI = next-level func
        mov  bx, [bp+2]         ; BX = operator table
        call spaces
        mov  dl, [si]           ; peek operator char
.search:
        cmp  byte [bx], 0
        je   .done
        cmp  [bx], dl
        je   .found
        add  bx, 3              ; next entry: char(1) + handler_ptr(2)
        jmp  .search
.found:
        inc  si                 ; consume operator char
        push ax                 ; save LHS
        push word [bx+1]        ; save handler address
        call di                 ; get RHS -> AX
        xchg cx, ax             ; CX = RHS
        pop  bx                 ; BX = handler
        pop  ax                 ; AX = LHS
        call bx                 ; AX = AX op CX
        jmp  .lp
.done:
        add  sp, 4              ; discard saved BX and DI
        ret

; =============================================================================
; DO_ABS_FUNC  ABS(n) -> absolute value
; Inputs  : AX = value (from eat_paren_expr)
; Outputs : AX = |value|
; Clobbers: (none)
; =============================================================================
do_abs_func:
        or   ax, ax     ; already +ve
        jns  dafd
jnegax:        
        neg  ax
dafd:
        ret

; =============================================================================
; E2_NEG  unary negation factor
; =============================================================================
e2_neg:
        inc  si
        call expr2
        jmp short jnegax

; =============================================================================
; EXPR2  factor level: unary operators, built-in functions, literals, variables
; Inputs  : SI -> factor text
; Outputs : AX = value
; Clobbers: AX, BX, DX, SI
; =============================================================================
e2_pos:
        inc  si
        ; fall through to expr2
expr2:
        call spaces
        mov  al, [si]
        cmp  al, '('
        je   eat_paren_expr
        cmp  al, '-'
        je   e2_neg
        cmp  al, '+'
        je   e2_pos

        ; Scan function dispatch table
        mov  bx, func_tab
e2_func_lp:
        cmp  word [bx], 0       ; sentinel?
        je   e2_nusr
        push bx
        call kw_match
        pop  bx
        jnc  e2_func_call
        add  bx, 4              ; next entry: kw_ptr(2) + handler_ptr(2)
        jmp  e2_func_lp

e2_func_call:
        push bx
        call eat_paren_expr
        pop  bx
        jmp  [bx+2]             ; indirect jump to handler

; =============================================================================
; EAT_PAREN_EXPR  parse '(' <expr> ')' -> AX
; Inputs  : SI -> '('
; Outputs : AX = expression value, SI advanced past ')'
; Clobbers: AX, BX, CX, DX, SI
; =============================================================================
eat_paren_expr:
        mov  al, '('
        call expect
e2_par:
        call expr
        push ax
        mov  al, ')'
        call expect
        pop  ax
        ret

; =============================================================================
; DO_PEEK_FUNC  PEEK(addr) -> byte at memory address
; Inputs  : AX = address (from eat_paren_expr)
; Outputs : AX = zero-extended byte value
; Clobbers: AX, BX
; Note: falls through into in_tail via 0xBB prefix trick
; =============================================================================
do_peek_func:
        xchg bx, ax
        mov  al, [bx]
        db   0xBB            ; "mov bx, imm16": swallows do_in_func's xchg+in
        ; fall through to in_tail

; =============================================================================
; DO_IN_FUNC  IN(port) -> byte from I/O port
; Inputs  : AX = port number (from eat_paren_expr)
; Outputs : AX = zero-extended byte value
; Clobbers: AX, DX
; =============================================================================
do_in_func:
        xchg dx, ax
        in   al, dx
in_tail:
        xor  ah, ah             ; zero-extend to 16-bit
        ret

; =============================================================================
; RND_SHUFFLE  advance 16-bit Galois LFSR and return new seed value
; Inputs  : (none; reads RND_SEED)
; Outputs : AX = new seed
; Clobbers: AX
; =============================================================================
rnd_shuffle:
        mov  ax, [RND_SEED]
        shr  ax, 1
        jnc  .skip
        xor  ax, 0xA001
.skip:
        mov  [RND_SEED], ax
        ; drop through
; =============================================================================
; DO_NOT_FUNC  NOT(n) -> bitwise complement
; Inputs  : AX = value (from eat_paren_expr)
; Outputs : AX = ~value
; Clobbers: (none)
; =============================================================================
do_not_func:
        not  ax
        ret

; =============================================================================
; E2_VAR  load variable value at factor level
; Inputs  : SI -> variable letter
; Outputs : AX = variable value
; Clobbers: AX, DI
; =============================================================================
e2_var:
        call get_var_addr
        mov  ax, [di]
        ret

; =============================================================================
; E2_NUSR  number-or-variable dispatch (after function table miss)
; Routes to input_number (decimal literal) or e2_var (letter).
; =============================================================================
e2_nusr:
        mov  al, [si]           ; reload: kw_match may have clobbered AL
        cmp  al, '0'
        jb   e2_var
        cmp  al, '9'
        ja   e2_var
        ; fall through to input_number

; =============================================================================
; INPUT_NUMBER  parse unsigned decimal integer from [SI]
; Inputs  : SI -> digit string
; Outputs : AX = parsed value, SI advanced past digits
; Clobbers: AX, BX, CX
; =============================================================================
input_number:
        xor  bx, bx
inm_lp:
        lodsb               ; AL = [SI], SI++
        sub  al, '0'
        jb   inm_stop
        cmp  al, 9
        ja   inm_stop      
        cbw                 ; AX = digit
        xchg ax, bx         ; BX = digit, AX = running total
        mov  cx, 10
        mul  cx             ; DX:AX = AX * 10
        add  bx, ax         ; BX = total * 10 + digit
        jmp  short inm_lp
inm_stop:
        dec  si             ; BACK UP: LODSB advanced SI past the non-digit
inm_done:
        xchg ax, bx         ; Result into AX
        ret

; =============================================================================
; INPUT_LINE  read an edited line into IBUF; returns SI -> IBUF
; Supports backspace editing.  Maximum 62 characters.
; Inputs  : (none)
; Outputs : SI -> IBUF (terminated with CR)
; Clobbers: AX, CX, DI
; =============================================================================
input_line:
        mov  di, IBUF
        xor  cx, cx             ; Zeroing
ipl_lp:
        call input_key

        cmp  al, 0x08           ; Backspace?
        je   ipl_bs
        cmp  al, 0x0D           ; CR?
        je   ipl_cr
        cmp  al, 0x0A           ; LF?
        jne  ipl_chkfull

        ; --- LF Handler ---
        jcxz ipl_lp             ; Buffer empty? Ignore stray LF
        mov  al, 0x0D           ; Convert LF to CR
                                ; FALLTHROUGH directly into ipl_cr!

ipl_cr: ; --- CR / End of Line ---
        stosb                   ; Store the CR (1 byte)
        mov  si, IBUF
        jmp  short new_line

ipl_chkfull: ; --- Normal Character ---
        cmp  cx, MAX_BUF             ; Buffer full?
        jb   ipl_store          ; No: go store it
        call bell               ; Yes: audible feedback
        jmp  short ipl_lp

ipl_store:
        call output             ; Echo char
        stosb                   ; Store char in [di], di++ (1 byte)
        inc  cx                 ; (1 byte)
        jmp  short ipl_lp

ipl_bs: ; --- Backspace Handler ---
        jcxz ipl_lp             ; Buffer empty? Ignore backspace
        dec  di                 ; (1 byte)
        dec  cx                 ; (1 byte)
        call backsp
        call output_space
        call backsp
        jmp  short ipl_lp

; =============================================================================
; NEW_LINE  emit CR + LF
; OUTPUT_SPACE  emit a single space character
; BACKSP  emits BASCKSPACE
; QUESTION  emits '?'
; Inputs  : Num_Space AX is num
; Clobbers: AX
; =============================================================================
num_space:
	call output_number
        jmp output_space
new_line:
        mov  al, 0x0D
        call output
        mov  al, 0x0A
	db 0x3d
question:
	mov al, '?'
	db 0x3d
backsp:
	mov al, 0x08
        db 0x3d
bell:
        mov  al, 0x07            ; BEL -- audible buffer-full feedback (v1.8.2)
        db   0x3d
output_space:
        mov  al, ' '
        jmp  short output             ; tail-call 

; =============================================================================
; OUTPUT_NUMBER  print signed 16-bit integer to terminal
; Inputs  : AX = signed 16-bit value
; Clobbers: AX, CX, DX
; =============================================================================
output_number:
        or   ax, ax
        jns  on_pos
        push ax
        mov  al, '-'
        call output
        pop  ax
        neg  ax
on_pos:
        xor  dx, dx
        mov  cx, 10
        div  cx
        push dx
        or   ax, ax
        je   on_digit
        call output_number      ; recurse for higher-order digits
on_digit:
        pop  ax
        add  al, '0'
        ; drop through
        
; =============================================================================
; OUTPUT / PUTCHAR  send character in AL to terminal
; ROM variant  : bitbang 8N1 via Intel 8755 Port A
; YASM variant : BIOS INT 10h teletype (AH=0Eh)
; Inputs  : AL = character to send
; Outputs : (none)
; Clobbers: AX  (ROM variant also: BL)
; =============================================================================
putchar:
output:
%ifdef __YASM_MAJOR__
        push bx
        mov  ah, 0x0E
        mov  bx, 0x0007
        int  0x10
        pop  bx
        ret
%else
        mov  ah, al             ; AH = char to send
        mov  al, 0              ; start bit (TX line low)
        out  PORT_A, al
        call bdly
        mov  bl, 9              ; 8 data bits + 1 stop bit
        stc                     ; CF=1 pre-loads the stop bit
.out_bit:
        rcr  ah, 1              ; LSB -> CF; old CF -> AH bit 7
        sbb  al, al             ; CF=1 -> AL=0xFF, CF=0 -> AL=0x00
        and  al, TX
        out  PORT_A, al
        call bdly
        stc
        dec  bx                 ; BX (1-byte opcode), not BL (2-byte)
        jnz  .out_bit
        ret
%endif
   
; =============================================================================
; INPUT_KEY  read one character from terminal into AL
; ROM variant  : bitbang UART RX from Intel 8755 Port A bit 1
; YASM variant : BIOS INT 16h keyboard read
; Also advances the LFSR on each poll iteration.
; Inputs  : (none)
; Outputs : AL = character
; Clobbers: AX  (ROM variant also: AH, CX)
; =============================================================================
getchar:
input_key:
        call rnd_shuffle        ; advance PRNG while idle
%ifdef __YASM_MAJOR__
        mov  ah, 0x01           ; peek keyboard buffer
        int  0x16
        jz   input_key          ; ZF=1: no key yet
        mov  ah, 0x00           ; read and remove key
        int  0x16
        ret
%else
        in   al, PORT_A
        test al, RX             ; wait for start bit (RX goes low)
        jnz  input_key
        call bdly               ; centre of start bit
        mov  ah, 0x80           ; marker: shifts out when byte complete
.ik_bit:
        in   al, PORT_A
        shr  al, 1              ; bit 1 -> bit 0 -> CF
        shr  al, 1
        rcr  ah, 1              ; CF -> AH MSB
        call bdly
        jnc  .ik_bit
        mov  al, ah
        ret
%endif

; =============================================================================
; BDLY  bit-period delay (~1 bit-time at 4800 baud / 5 MHz)
; Inputs  : (none)
; Outputs : (none)
; Clobbers: CX
; =============================================================================
bdly:
        mov  cx, BAUD
        loop $                  ; 17 cy/iter on 8088
        ret

; =============================================================================
; FIND_LINE / WALK_LINES  scan program for first line >= AX
; Inputs  : AX = target line number
; Outputs : DI -> first line entry with line# >= AX, or sentinel if not found
; Clobbers: BX, DI
; =============================================================================
find_line:
walk_lines:
        mov  di, PROGRAM
wl_lp:
        mov  bx, [di]
        or   bx, bx
        je   wl_done
        cmp  bx, ax
        jnb  wl_done
        call next_line_ptr
        jmp  short wl_lp

; =============================================================================
; NEXT_LINE_PTR  advance DI from current line start to next line start
; Inputs  : DI -> current line (at line number word)
; Outputs : DI -> start of next line (or sentinel)
; Clobbers: DI
; =============================================================================
next_line_ptr:
        add  di, 2              ; skip line number word
nlp_lp:
        cmp  byte [di], 0x0D
        je   nlp_done
        inc  di
        jmp  nlp_lp
nlp_done:
        inc  di                 ; skip CR
wl_done:
        ret

; =============================================================================
; EDITLN  tokenise body then store, replace, or delete a numbered line
; Inputs  : AX = line number, SI -> raw body text in IBUF (spaces already skipped)
; Clobbers: AX, BX, CX, DX, SI, DI
; =============================================================================
editln:
        push ax
        call spaces
        call tokenize           ; tokenise in-place; SI preserved
        pop  dx                 ; DX = line number
        ; measure tokenised body + CR
        mov  bx, si
        mov  cx, 0
el_len:
        inc  cx
        cmp  byte [si], 0x0D
        je   el_ldone
        inc  si
        jmp  el_len
el_ldone:
        push bx
        mov  ax, dx
        call find_line          ; DI = insertion point
        cmp  di, [PROG_END]     ; DI past the logical program end? NEW resets
        jnb  el_noex            ; PROG_END but leaves old bytes physically
                                 ; intact (cold boot needs them undisturbed
                                 ; for the showcase); without this check a
                                 ; leftover line number here that happens to
                                 ; match DX looks like a real existing line,
                                 ; and deleting/re-inserting it walks PROG_END
                                 ; below PROGRAM, wrapping SLIDE_DATA's byte
                                 ; count to ~64K on the next insert -- the
                                 ; reported NEW-then-line-entry hang. (v1.8.1)
        cmp  [di], dx
        jne  el_noex
        push cx
        push dx                 ; DELINE clobbers DX -- save line number (v1.8.1)
        call deline             ; delete existing line
        pop  dx                 ; restore line number (else INSLINE below
        pop  cx                 ;  would store DELINE's leftover shift amount)
el_noex:
        pop  bx
        cmp  byte [bx], 0x0D   ; empty body = delete only
        je   editln_done
        mov  si, bx
        mov  ax, dx
        ; fall through to insline

; =============================================================================
; INSLINE  insert a tokenised line into program store
; Inputs  : AX = line number, SI -> tokenised body + CR, CX = body+CR length
; Clobbers: AX, BX, CX, DX, SI, DI
; =============================================================================
insline:
        mov  bx, [PROG_END]
        add  bx, cx
        add  bx, 4              ; +2 line# word + 2 new sentinel
        cmp  bx, PROGRAM_TOP
        jnb  ins_oom
        push ax
        push si
        push cx
        mov  dx, cx
        add  dx, 2              ; gap = body + line number word
        call slide_data
        pop  cx
        pop  si
        pop  ax
        stosw                   ; write line number word
        rep  movsb              ; copy body + CR
el_done:
editln_done:
        ret

ins_oom:
        mov  al, ERR_OM
        jmp  do_error

; =============================================================================
; DELINE  delete the line at DI from program store
; Inputs  : DI -> line to delete (at line number word)
; Outputs : DI = original value (restored)
; Clobbers: AX, BX, CX, DX, SI
; =============================================================================
deline:
        push di
        call next_line_ptr      ; DI -> next line
        mov  dx, di
        pop  di
        sub  dx, di             ; DX = byte count of line
        neg  dx                 ; negative = close gap
        ; fall through to slide_data

; =============================================================================
; SLIDE_DATA  shift program memory to open or close a gap at DI
; Inputs  : DI = target address
;           DX = shift (positive = open gap / insert; negative = close / delete)
; Clobbers: AX, BX, CX, SI, DI
; =============================================================================
slide_data:
        push di
        mov  si, [PROG_END]
        add  si, 2              ; SI = one past sentinel
        mov  cx, si
        sub  cx, di             ; CX = bytes from DI to end
        cmp  dx, 0
        jl   slide_down

        ; Slide UP (insert): copy backwards to avoid overlap
        add  di, cx
        mov  si, di
        dec  si
        add  di, dx
        dec  di
        std
        rep  movsb
        cld
        jmp  slide_done

slide_down:
        ; Slide DOWN (delete): copy forwards
        mov  si, di
        sub  si, dx             ; DX negative -> SI = DI + abs(DX)
        rep  movsb

slide_done:
        add  [PROG_END], dx
        pop  di
        ret

; =============================================================================
; DO_NEW  clear program store and reset PROG_END
; Inputs  : (none)
; Clobbers: AX, CX, DI
; =============================================================================
clr_vars:
        xor  ax, ax
        xor  di, di                     ; clear ram starting from zero
        mov  cx, PROGRAM                ; 0xB9 
        rep  stosb                      ; DI increments and stops at PROGRAM
        mov  word [RND_SEED], 0xACE1    ; seed LFSR
        ret 

do_new:
        call clr_vars
        mov  word [PROG_END], DI   
        mov  word [DI], ax               ; empty-program sentinel.
        ; fall through to do_end
; =============================================================================
; DO_END  END statement — stops program execution
; Inputs  : (none)
; Clobbers: AX
; =============================================================================
do_end:
        mov  ax, [PROG_END]
        mov  [RUN_NEXT], ax     ; RUN_NEXT -> sentinel -> run_loop will exit
        xor  al, al
run_end:
        mov  byte [RUNNING], al
        ret
        
; =============================================================================
; Shared GOTO / GOSUB helper
; Clobbers: AX, DI
; =============================================================================
get_line_or_err:
        call expr
        call find_line
        cmp  [di], ax
        jne  JERRUL
dg_ret:
        ret                     ; Shared return used by get_line_or_err and dg_common

JERRUL:
        mov  al, ERR_UL
        jmp  do_error

; =============================================================================
; DO_GOTO  GOTO <linenum>
; DO_RUN   RUN
; Inputs  : SI -> line number expression (GOTO) or program start (RUN)
; Clobbers: AX, BX, DI
; =============================================================================
do_goto:
        call get_line_or_err
        jmp  short dg_common

do_run:
        mov  di, PROGRAM
        jmp  short dg_common

; =============================================================================
; DO_GOSUB  GOSUB <linenum>
; Saves RUN_NEXT on dedicated GOSUB stack then jumps to target.
; Inputs  : SI -> line number expression
; Clobbers: AX, BX, DI
; =============================================================================
do_gosub:
        call get_line_or_err
        mov  bx, [GOSUB_SP]
        cmp  bx, 8
        jb   gs_push
        jmp  JERRSN             ; overflow -> syntax error

gs_push:
        inc  word [GOSUB_SP]
        add  bx, bx             ; BX is now byte offset (0, 2, 4...)
        mov  ax, [RUN_NEXT]
        mov  [bx + GOSUB_STK], ax ; Direct base-indexed addressing! (Replaces LEA bodge)
        ; fall through to dg_common!

dg_common:
        mov  [RUN_NEXT], di
        cmp  byte [RUNNING], 0
        jne  dg_ret             ; already running (mid-GOTO/GOSUB): jump up to shared 'ret'
        inc  byte [RUNNING]
        ; fall through to run_loop

; =============================================================================
; RUN_LOOP  fetch and execute lines until sentinel or DO_END
; Inputs  : (none; reads RUN_NEXT)
; Clobbers: everything (one statement per iteration)
; =============================================================================
run_loop:
        mov  di, [RUN_NEXT]
        mov  si, di
        lodsw                   ; AX = line#; SI -> body
        test ax, ax
        jz   run_end
        mov  [CURLN], ax
        call next_line_ptr      ; DI -> start of next line
        mov  [RUN_NEXT], di
        call stmt
        jmp  short run_loop

; =============================================================================
; DO_RETURN  RETURN
; Pops return address from GOSUB stack and resumes execution.
; Inputs  : (none)
; Clobbers: AX, BX, SI
; =============================================================================
gs_underflow: 
        mov  al, ERR_RT
        jmp  do_error
do_return:
	mov  si, GOSUB_SP		; 3
        mov  ax, [si]			; 2
        or   ax, ax			; 2
        jz   gs_underflow		; 2
        dec  ax				; 1
        mov  [si], ax			; 2
        add  ax, ax                     ; byte offset = depth * 2	;2
        xchg ax, bx			; 1
	lea  si, [bx + GOSUB_STK]
        lodsw				; 1
        mov  [RUN_NEXT], ax		; 2
        ret				; 1
        
; =============================================================================
; DO_REM  REM — skip remainder of line during program execution
; Inputs  : SI -> REM body
; Clobbers: AH, SI, DI
; =============================================================================
do_rem:
        mov  di, si             ; DI = SI: copy_si_di becomes a pure skip
        mov  ah, 0x0D
        ; fall through to copy_si_di

; =============================================================================
; COPY_SI_DI  copy (or skip) bytes from SI to DI until AH or CR
; If DI = SI on entry the copy is a no-op (used by DO_REM at runtime).
; Used by TOKENIZE to pass string literals and REM bodies verbatim.
; Inputs  : SI = read ptr, DI = write ptr, AH = secondary terminator
; Outputs : SI and DI advanced to char after terminator
; Clobbers: AX, SI, DI
; =============================================================================
copy_si_di:
        lodsb
        stosb
        cmp  al, 0x0D
        je   .done
        cmp  al, ah
        jne  copy_si_di
.done:
        ret

; =============================================================================
; TOKENIZE  convert keyword text to single-byte tokens in-place in IBUF
; String literals and REM bodies are preserved verbatim.
; Tokenised form <= original length, so in-place rewrite is safe.
; Inputs  : SI -> start of body text in IBUF
; Outputs : IBUF rewritten with token bytes; SI unchanged (restored)
; Clobbers: AX, BX, CX, DX, DI
; =============================================================================
tokenize:
        push si
        mov  di, si             ; write pointer starts at read pointer

tk_lp:
        lodsb
        cmp  al, 0x0D
        je   tk_done

        ; String literal: pass verbatim
        cmp  al, '"'
        jne  tk_not_str
        stosb                   ; write opening quote
        mov  ah, '"'
        call copy_si_di
        jmp  tk_lp

tk_not_str:
        dec  si                 ; back up to re-include current char
        mov  bx, tk_kw_tab
tk_try:
        cmp  word [bx], 0
        je   tk_char
        push di
        push bx
        call kw_match
        pop  bx
        pop  di
        jc   tk_next_kw

        ; Keyword matched: emit token byte
        mov  ax, bx
        sub  ax, tk_kw_tab      ; byte offset into table
        shr  ax, 1              ; -> 0-based index
        add  al, TK_PRINT       ; + base
        stosb
        call spaces             ; consume trailing spaces
        cmp  al, TK_REM
        jne  tk_lp
        mov  ah, 0x0D           ; REM: copy rest verbatim
        call copy_si_di
        jmp  tk_finish

tk_next_kw:
        add  bx, 2
        jmp  tk_try

tk_char:
        lodsb                   ; no match: emit literal char
        stosb
        jmp  tk_lp

tk_done:
        stosb                   ; write final CR
tk_finish:
        pop  si                 ; restore body start pointer
        ret

; =============================================================================
; DO_FOR  FOR <var> = <start> TO <end> [STEP <step>]
; Frame layout (8 bytes per slot in FOR_STK):
;   [bx+0] var_ptr  [bx+2] limit  [bx+4] step  [bx+6] loop_ptr
; Inputs  : SI -> line body after FOR token
; Clobbers: AX, BX, CX, DX, DI
; =============================================================================
df_syn:
        mov  al, ERR_OM
        jmp  do_error
do_for:
        call get_var_addr       ; get_var_addr already skips spaces itself
        mov  [INS_TMP], di      ; save &var
        call expect_equals
        call expr               ; AX = start value
        mov  di, [INS_TMP]
	stosw			; initialise loop variable
        ; TO is mandatory
        mov  al, TK_TO
        mov  bx, to_tab
        call expect_token_or_kw
        jc   df_syn

        call expr               ; AX = limit
        push ax

        ; STEP is optional (default = 1)
        mov  al, TK_STEP
        mov  bx, step_tab
        call expect_token_or_kw
        mov  ax, 1
        jc   df_no_step
        call expr               ; AX = explicit step value
df_no_step:
        ; Push frame
        mov  cx, [FOR_SP]
        cmp  cl, 4
        jnb  df_syn             ; FOR stack full
        inc  word [FOR_SP]
        call for_ptr_hlp        ; BX -> frame slot
        ; setup frame
        xchg bx, di             ; DI = base address of frame
	xchg ax, bx		; save 'step'
	mov  ax, [INS_TMP]      ; Get var_ptr
        stosw                   ; Store [INS_TMP] at [di], then di += 2        
        pop ax	                ; Get limit
        stosw                   ; Store limit at [di+2], then di += 2
        xchg ax, bx             ; get 'step' back
        stosw                   ; Store 'step' at [di+4], then di += 2
        mov  ax, [RUN_NEXT]     ; Get loop_ptr
        stosw                   ; Store loop_ptr at [di+6], then di += 2      
        ret
	
; =============================================================================
; FOR_PTR_HLP  convert FOR stack depth to frame pointer
; Inputs  : CX = depth index (0-based)
; Outputs : BX -> FOR_STK[CX]  (8 bytes per frame)
; Clobbers: BX
; =============================================================================
for_ptr_hlp:
        mov  bx, cx
        shl  bx, 1              ; * 2	no `shl RX, n` on 8086
        shl  bx, 1              ; * 4
        shl  bx, 1              ; * 8
        add  bx, FOR_STK
        ret

; =============================================================================
; DO_NEXT  NEXT <var>
; Increments loop variable, tests exit condition, loops or pops frame.
; Inputs  : SI -> line body after NEXT token
; Clobbers: AX, BX, CX, DX, DI
; =============================================================================
dn_no_for:
        mov  al, ERR_NF
        jmp  do_error
do_next:
        call get_var_addr       ; DI = &var (already skips spaces itself)
        mov  cx, [FOR_SP]
dn_search:
        jcxz dn_no_for          ; stack empty: no matching FOR
        dec  cx
        call for_ptr_hlp        ; BX -> frame
        cmp  [bx], di           ; var_ptr match?
        jne  dn_search

        ; Update variable, then test exit condition:
        ;   positive step: exit when var > limit
        ;   negative step: exit when var < limit
        mov  ax, [bx+4]         ; step
        add  [di], ax           ; var += step
        mov  ax, [di]           ; new var value
        mov  dx, [bx+2]         ; limit
        cmp  word [bx+4], 0
        jl   dn_neg
        cmp  ax, dx
        jg  dn_done
dn_loop:
        mov  ax, [bx+6]
        mov  [RUN_NEXT], ax     ; jump back to top of loop
        ret

dn_neg:
        cmp  ax, dx
        jge  dn_loop
dn_done:
        mov  [FOR_SP], cx       ; pop frame (CX = correct new depth)
        ret

; =============================================================================
; KEYWORD STRINGS  (bit-7 terminated; table ends with 0x00 sentinel)
; =============================================================================
kw_tab_start:
kw_print:   db 0x50,0x52,0x49,0x4E,T_T         ; PRINT
kw_if:      db 0x49,T_F                         ; IF
kw_goto:    db 0x47,0x4F,0x54,T_O              ; GOTO
kw_list:    db 0x4C,0x49,0x53,T_T              ; LIST
kw_run:     db 0x52,0x55,T_N                   ; RUN
kw_new:     db 0x4E,0x45,T_W                   ; NEW
kw_input:   db 0x49,0x4E,0x50,0x55,T_T         ; INPUT
kw_rem:     db 0x52,0x45,T_M                   ; REM
kw_end:     db 0x45,0x4E,T_D                   ; END
kw_let:     db 0x4C,0x45,T_T                   ; LET
kw_poke:    db 0x50,0x4F,0x4B,T_E             ; POKE
kw_free:    db 0x46,0x52,0x45,T_E             ; FREE
kw_help:    db 0x48,0x45,0x4C,T_P             ; HELP
kw_gosub:   db 0x47,0x4F,0x53,0x55,T_B        ; GOSUB
kw_return:  db 0x52,0x45,0x54,0x55,0x52,T_N   ; RETURN
kw_for:     db 0x46,0x4F,T_R                  ; FOR
kw_next:    db 0x4E,0x45,0x58,T_T             ; NEXT
kw_out:     db 0x4F,0x55,T_T                  ; OUT
kw_to:      db 0x54,T_O                        ; TO
kw_step:    db 0x53,0x54,0x45,T_P             ; STEP
; --- not statements; included for HELP output ---
kw_then:    db 0x54,0x48,0x45,T_N             ; THEN
kw_chrs:    db 0x43,0x48,0x52,T_DS            ; CHR$
kw_peek:    db 0x50,0x45,0x45,T_K             ; PEEK
kw_usr:     db 0x55,0x53,T_R                  ; USR
kw_in:      db 0x49,T_N                        ; IN
kw_tab:     db 0x54,0x41,T_B                  ; TAB
kw_abs:     db 0x41,0x42,T_S                  ; ABS
kw_rnd:     db 0x52,0x4E,T_D                  ; RND
kw_not:     db 0x4E,0x4F,T_T                  ; NOT
            db 0                               ; sentinel

; =============================================================================
; TOKEN -> KEYWORD STRING POINTER TABLE  (same order as st_tab / TK_xx)
; =============================================================================
tk_kw_tab:
        dw kw_print, kw_if, kw_goto, kw_list, kw_run, kw_new
        dw kw_input, kw_rem, kw_end, kw_let, kw_poke, kw_free
        dw kw_help, kw_gosub, kw_return
        dw kw_for, kw_next, kw_out
        dw 0                     ; terminate tk_kw_tab scan (else TOKENIZE
                                  ; walks into then_tab/to_tab/step_tab/... below)

; Sub-keyword pointer entries (matched individually; not iterated)
then_tab:   dw kw_then
to_tab:     dw kw_to
step_tab:   dw kw_step
; PRINT-only functions (single entry each; matched individually, not iterated)
chrs_tab:   dw kw_chrs
tab_tab:    dw kw_tab

; =============================================================================
; STRINGS  (bit-7 terminated)
; =============================================================================
str_banner: db "uBASIC 8088 v1.8.2"
CRLF:       db 0x0D, 0x0A + 0x80

; =============================================================================
; STATEMENT HANDLER TABLE  (indexed by token - TK_PRINT, one word per entry)
; =============================================================================
st_tab:
        dw do_print,  do_if,     do_goto,   do_list,  do_run,   do_new
        dw do_input,  do_rem,    do_end,    do_let,   do_poke,  do_free
        dw do_help,   do_gosub,  do_return, do_for,   do_next,  do_out

; =============================================================================
; OPERATOR TABLES  {char(1), handler_ptr(2), ...}, 0x00 sentinel
; =============================================================================

tab_add:                        ; additive level
        db '+'
        dw math_add
        db '-'
        dw math_sub
        db 0

tab_mul:                        ; multiplicative level
        db '*'
        dw math_mul
        db '/'
        dw math_div
        db '%'
        dw math_mod
        db 0

tab_bitwise:                    ; bitwise level (lowest among binary operators)
        db '&'
        dw bitwise_and
        db '|'
        dw bitwise_or
        db '~'
        dw bitwise_xor
        db 0

tab_pow:                        ; power level (higher precedence than * / %)
        db '^'
        dw math_pow
        db 0

; =============================================================================
; FUNCTION DISPATCH TABLE  {kw_ptr(2), handler_ptr(2), ...}, dw 0 sentinel
; =============================================================================
func_tab:
        dw kw_rnd,  do_rnd_func
        dw kw_peek, do_peek_func
        dw kw_in,   do_in_func
        dw kw_usr,  do_usr_func
        dw kw_abs,  do_abs_func
        dw kw_not,  do_not_func
        dw 0

ROM_END:

; =============================================================================
; RESET VECTOR  at 0xFFF0
; 8086 resets to CS=0xFFFF IP=0x0000 -> phys 0xFFFF0.
; =============================================================================
%ifndef __YASM_MAJOR__
        org 0xFFF0
%endif

reset_vec:
        cld
        ; Configure 8755 Port A: bit1=RX(input), all others output; TX idles high
        mov  al, 0xFD
        out  DDR_A, al
        mov  al, TX
        out  PORT_A, al

        ; FAR JMP (opcode: EA)
        db   0xEA
        dw   ORIGIN             ; IP
        dw   0x0000             ; CS

; =============================================================================
; DO_USR_FUNC  USR(addr) — call arbitrary machine-code address
; Placed here as a space-filler between reset vector and end of memory
; Inputs  : AX = call address (from eat_paren_expr)
; Outputs : AX = return value from called routine
; Clobbers: whatever the called routine clobbers
; =============================================================================
do_usr_func:
        jmp  ax                 ; tail-call
