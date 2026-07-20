; =============================================================================
; miniBASIC 8088  v3.5  (was: uBASIC 8088 v1.7.5)
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Tiny-BASIC-derived interpreter for the 8088/8086 with MBF4 32-bit
; floating-point support, a rational/polynomial trig library (SIN, COS,
; TAN, ATN, ASIN, ACOS, SQRT).
;
; Credit: Oscar Toledo G. for bootBASIC inspiration and TinyASM 8086 assembler.
;         XTulator CPU core by Mike Chambers.
;
; KNOWN LIMITATIONS
;   - Multi-statement lines (':'-separated statements) removed in v2.7 to
;     reclaim ROM bytes and eliminate REM/PRINT interaction bugs that
;     arose from colon-splitting. One statement per line only.
;
;   - TAB(n) prints n literal space characters relative to the current
;     cursor position -- it does NOT move to absolute column n like
;     classic BASIC's TAB. Code ported from a dialect with absolute TAB
;     semantics (e.g. "PRINT TAB(C);..." inside a loop where C is the
;     running column) will produce cumulative, ever-widening spacing
;     instead of column alignment; use a fixed per-iteration TAB(k)
;     (e.g. TAB(1) to advance one column per printed item) instead.
;
;   - FLT_ADD's internal "put the larger-magnitude operand first" swap does
;     not restore FLT_B's original identity afterward.  If |FLT_B|>|FLT_A| on
;     entry, FLT_B ends up holding mangled remnants of the swap rather than
;     its own original value.
;
; =============================================================================
; CHANGE HISTORY
; =============================================================================
;
; v3.5 (2026-07-19)
;   - BUGFIX: EDITLN corrupted the stored line number when replacing an
;     existing line (DELINE clobbers DX internally; EDITLN read it again
;     afterward without saving/restoring it). Caused permanently orphaned,
;     unreachable duplicate lines on re-edit. Fixed with a push/pop.
;   - BUGFIX: FLT_PI_B/FLT_2PI_B's byte-overlap self-modifying-code trick
;     (DB 0x3B) crashed/reset the interpreter for SIN/COS near PI and 2*PI.
;     Replaced with an equally compact, verified shared-tail merge.
;   - BUGFIX: showcase VORTEX demo used PRINT TAB(C) assuming classic
;     absolute-column TAB semantics; this interpreter's TAB(n) prints n
;     literal spaces relative to the current cursor (see KNOWN LIMITATIONS
;     history), so C growing 0..60 every row produced cumulative,
;     ever-widening space runs instead of column alignment. Changed to
;     TAB(1) for correct single-space-per-column advancement.
;   - FEATURE: INPUT_LINE now sounds BELL (0x07) on each keystroke once the
;     62-char line limit is hit, instead of silently dropping the input.
;   - Code golf: FLT_MUL, FLT_DIV, FLT_ADD, FOR_PTR_HLP, EXPECT_TOKEN_OR_KW,
;     DO_NEXT, FLT_PRINT micro-optimized (word-load+xchg tricks, 3-op
;     two's-complement negate, disp8-vs-disp16 pointer reuse, dead-code
;     removal, 8-bit IMUL, inc/dec+jz/jnz idioms -- see individual routine
;     headers for details). New shared ONE_MINUS_MUL helper deduped an
;     identical 5-instruction sequence in FLT_ASIN/FLT_SIN.
;   - Net effect: ~104 bytes free (ROM variant), up from ~25, despite
;     adding the BELL feature. All changes verified via assemble + trace +
;     full regression pass (showcase, arithmetic, trig incl. domain edges,
;     FOR/NEXT, line insert/replace/delete/out-of-order, 60+ line program).
;
; v3.4 (2026-07-16)
;   - Added TAN() and its underlying floating-point implementation.
;   - Replaced the showcase program with a "vortex" demo exercising all trig functions.
;   - Cleaned up residual CORDIC RAM definitions and documentation.
;   - Verified trigonometric functions consistently use radians.
;
; v3.3 (2026-07-15)
;   - Fixed an infinite recursion bug when parsing bare parenthesized expressions.
;
; v3.2 (2026-07-15)
;   - Fixed a divide-by-zero error in ASIN() and ACOS() when evaluating 1 or -1.
;
; v3.1 (2026-07-15)
;   - Exposed SQRT() as a standard BASIC keyword.
;   - Updated the version banner and showcase intro strings.
;
; v3.0 (2026-07-14)
;   - Refactored SIN() and COS() CORDIC implementations with a polynomial approximation.
;   - Added float-truncation support and new math constants.
;   - Fixed a bug in float-to-integer conversion where fractional values failed to truncate to zero.
;
; v2.9 (2026-07-14)
;   - Added ASIN() and ACOS() functions and their underlying floating-point implementations.
;   - Micro-optimised ATAN() and extracted common PI/2 loading code to save space.
;   - Identified a known bug with ASIN() and ACOS() boundary values.
;
; v2.8 (2026-07-14)
;   - Replaced the CORDIC-based ATN() implementation with a rational approximation.
;   - Noted a minor precision reduction in ATN() in exchange for significant byte savings.
;   - Identified a pre-existing bug where COS() incorrectly mirrored SIN().
;
; v2.7 (2026-07-14)
;   - Removed partial multi-statement line support (colon separator) to reclaim ROM space.
;   - Updated the showcase program to use single-statement lines.
;
; v2.6 (2026-07-14)
;   - Added a floating-point square root routine using Newton-Raphson approximation.
;   - Fixed a variable-copying bug in the ported square root source.
;   - Removed bitwise operators (&, |, ^) to reclaim ROM space.
;
; v2.5 (2026-07-13)
;   - Fixed a bug where relational operators always evaluated to false due to register clobbering.
;
; v2.4 (2026-07-13)
;   - Added ATN() using the CORDIC engine's vectoring mode.
;   - Extracted duplicate operator-table scans and line-target parsing into shared routines.
;
; v2.3 (2026-07-12)
;   - Fixed a jump-encoding bug that caused SIN() to silently compute COS().
;   - Fixed expression-evaluator register corruption.
;   - Fixed stack-corruption bugs in FOR loop initialization and STEP handling.
;   - Optimised ROM size by removing redundant instructions and merging duplicate sequences.
;
; v2.2 (2026-06-30)
;   - Floating-point relational operators now compare native MBF4 values.
;   - Fixed LIST token handling for THEN, TO and STEP.
;   - Fixed GOSUB stack overflow reporting.
;   - Updated banner and documentation.
;
; v2.1 (2026-06-25)
;   - Added SIN() and COS() using a shared fixed-point CORDIC engine.
;   - Added CORDIC RAM workspace and angle-reduction support.
;   - Fixed nested-expression reentrancy in the floating-point evaluator.
;   - Fixed FOR/NEXT stack corruption.
;   - Fixed nested function-call parsing.
;   - Fixed TAB(0) and negative TAB() behaviour.
;   - Replaced the showcase program with floating-point demonstrations.
;   - Miscellaneous size optimisations.
;
; v2.0 (2026-06-23)
;   - Integrated the MBF4 floating-point library.
;   - Expanded ROM and RAM from 2 KB to 4 KB.
;   - Converted variables, arithmetic and FOR/NEXT to floating point.
;   - Updated BASIC functions and I/O to use MBF4 where appropriate.
;   - Unified floating-point error handling with BASIC runtime errors.
;   - Fixed merge-related parser and expression-evaluation issues.
;
; v1.7.5 (2026-05-12) 2kByte Tiny BasicSigned 16bit
; =============================================================================


        cpu 8086

; =============================================================================
; PLATFORM CONFIGURATION
; =============================================================================

ORIGIN:         equ 0xF000              ; ROM base (also YASM segment)
RAM_BASE:       equ 0x0000

RAM_SIZE:       equ 4096                ; 4 KB address space

; =============================================================================
; RAM LAYOUT  (all offsets relative to RAM_BASE)
; =============================================================================

DIV0:           equ RAM_BASE + 0x000    ; 4 bytes : divide-by-zero IVT entry
CURLN:          equ RAM_BASE + 0x004    ; word    : current line# for error reports
RUN_NEXT:       equ RAM_BASE + 0x006    ; word    : next-line pointer for run loop
NMI:            equ RAM_BASE + 0x008    ; 4 bytes : NMI IVT entry
IBUF:           equ RAM_BASE + 0x00C    ; 64 bytes: input line buffer (also
                                         ;   reused as flt_print's digit scratch)
RND_SEED:       equ RAM_BASE + 0x04A    ; word    : LFSR random seed
INS_TMP:        equ RAM_BASE + 0x04C    ; word    : insline / do_for var_ptr scratch
GOSUB_SP:       equ RAM_BASE + 0x04E    ; word    : GOSUB stack depth (0..7)
GOSUB_STK:      equ RAM_BASE + 0x050    ; 16 bytes: 8-entry GOSUB return-address stack
FOR_SP:         equ RAM_BASE + 0x060    ; word    : FOR stack depth (0..3)
FOR_STK:        equ RAM_BASE + 0x062    ; 48 bytes: 4 x 12-byte FOR frames
                                         ;   [+0] var_ptr(2) [+2] limit(4)
                                         ;   [+6] step(4)    [+10] loop_ptr(2)
VARS:           equ RAM_BASE + 0x092    ; 104 bytes: variables A-Z (4-byte
                                         ;   MBF4 float each)
FLT_SA:         equ RAM_BASE + 0x0FA    ; byte    : float result sign scratch
FLT_SB:         equ RAM_BASE + 0x0FB    ; byte    : sign of B scratch (flt_add)
FLT_ER:         equ RAM_BASE + 0x0FC    ; byte    : result exponent scratch (mul/div)
FLT_DE:         equ RAM_BASE + 0x0FD    ; byte    : flt_print decimal exponent /
                                         ;   flt_parse sign stash
FLT_DB:         equ RAM_BASE + 0x0FE    ; 3 bytes : flt_div B-mantissa spill
FLT_A:          equ RAM_BASE + 0x101    ; 4 bytes : primary float operand/result
FLT_B:          equ RAM_BASE + 0x105    ; 4 bytes : secondary float operand
FLT_C:          equ RAM_BASE + 0x109    ; 4 bytes : LHS park for
                                         ;   prec_engine_f's float dispatch.
                                         ;   GOTCHA: reserved exclusively for
                                         ;   that use -- was also once
                                         ;   accidentally shared with a
                                         ;   CORDIC scratch of the same
                                         ;   role, which was a bug (see the
                                         ;   v2.1 change history); that
                                         ;   scratch (and the rest of
                                         ;   CORDIC's RAM workspace) is gone
                                         ;   now anyway -- CORDIC removed
                                         ;   entirely in v3.0.
ATN_FLAGS:      equ RAM_BASE + 0x10D    ; byte    : FLT_ATAN scratch
                                         ;   (was DO_ATN_FUNC's before
                                         ;   v2.8); 0x80=original sign,
                                         ;   0x01=was range-reduced via 1/x
SQRT_S:         equ RAM_BASE + 0x10E    ; 4 bytes : FLT_SQRT scratch: original S
SQRT_X:         equ RAM_BASE + 0x112    ; 4 bytes : FLT_SQRT scratch: current x_n
TAN_C:          equ RAM_BASE + 0x116    ; 4 bytes : FLT_TAN scratch: cos(x), parked
                                         ;   across the FLT_SIN call (which clobbers
                                         ;   FLT_B, so cos(x) can't be parked there)

RUNNING:        equ RAM_BASE + 0x11A    ; byte    : 0=immediate mode, 1=running
PROG_END:       equ RAM_BASE + 0x11B    ; word    : one past last program byte
PROGRAM:        equ RAM_BASE + 0x11D    ; program store start
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
T_Y:            equ 0xD9        ; 'Y'+0x80  used by: DELAY
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
TK_DELAY:       equ 0x92
NUM_TOKENS:     equ 19          ; count: TK_PRINT (0x80) .. TK_DELAY (0x92)

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
; Token map (v1.7.5):
;   PRINT=0x80  IF=0x81  GOSUB=0x8D  RETURN=0x8E  END=0x88
;   FOR=0x8F    NEXT=0x90  OUT=0x91  DELAY=0x92
;   THEN=0x93   TO=0x94    STEP=0x95  REM=0x87
; =============================================================================

%ifdef __YASM_MAJOR__
        ; Trampoline: 8bitworkshop needs a near jump it can overwrite
        mov  ax, reset_vec
        jmp  ax
        times PROGRAM - ($-$$) db 0     ; pad over VARS / equate area

SHOWCASE_DATA:
        ; ── Feature demos: arithmetic, comparisons, FOR/NEXT, GOSUB ───────────────
        db 0x0A,0x00, 0x87, "miniBASIC 8088 v3.5 showcase", 0x0D ; 10  REM
        db 0x14,0x00, 0x80, 0x22, "--- ARITHMETIC ---", 0x22, 0x0D ; 20  PRINT
        db 0x1E,0x00, 0x80, 0x22, "2+3=", 0x22, ";2+3;", 0x22, "  6*7=", 0x22, ";6*7", 0x0D ; 30  PRINT
        db 0x28,0x00, 0x80, 0x22, "20/4=", 0x22, ";20/4;", 0x22, "  17%5=", 0x22, ";17%5", 0x0D ; 40  PRINT
        db 0x32,0x00, 0x80, 0x22, "1.5+2.25=", 0x22, ";1.5+2.25", 0x0D ; 50  PRINT
        db 0x3C,0x00, 0x80, 0x22, "--- COMPARISONS ---", 0x22, 0x0D ; 60  PRINT
        db 0x46,0x00, 0x81, "5>3", 0x93, 0x80, 0x22, "5>3 ok", 0x22, 0x0D ; 70  IF THEN(0x93) PRINT
        db 0x50,0x00, 0x81, "3<5", 0x93, 0x80, 0x22, "3<5 ok", 0x22, 0x0D ; 80  IF THEN(0x93) PRINT
        db 0x5A,0x00, 0x81, "3>=3", 0x93, 0x80, 0x22, "3>=3 ok", 0x22, 0x0D ; 90  IF THEN(0x93) PRINT
        db 0x64,0x00, 0x81, "4<>3", 0x93, 0x80, 0x22, "4<>3 ok", 0x22, 0x0D ; 100 IF THEN(0x93) PRINT
        db 0x6E,0x00, 0x80, 0x22, "--- FOR/NEXT ---", 0x22, 0x0D ; 110 PRINT
        db 0x78,0x00, 0x8F, "I=1", 0x94, "5", 0x0D ; 120 FOR I=1 TO(0x94) 5
        db 0x82,0x00, 0x80, "I;", 0x0D             ; 130 PRINT I;
        db 0x8C,0x00, 0x90, "I", 0x0D              ; 140 NEXT I
        db 0x96,0x00, 0x80, 0x22, 0x22, 0x0D       ; 150 PRINT ""
        db 0xA0,0x00, 0x80, 0x22, "--- GOSUB ---", 0x22, 0x0D ; 160 PRINT
        db 0xAA,0x00, 0x8D, "500", 0x0D            ; 170 GOSUB 500
        db 0xB4,0x00, 0x80, 0x22, "sum 1..10=", 0x22, ";S", 0x0D ; 180 PRINT
        db 0xBE,0x00, 0x8D, "550", 0x0D            ; 190 GOSUB 550
        db 0xC8,0x00, 0x80, 0x22, "5!=", 0x22, ";F", 0x0D ; 200 PRINT
        db 0xD2,0x00, 0x80, 0x22, 0x22, 0x0D       ; 210 PRINT ""
        ; ── Vortex: trig-library stress test (SIN, COS, TAN, ASIN, ACOS, ATN, ─
        ; SQRT), replaced the old CORDIC sine-wave demo in v3.4.
        db 0xDC,0x00, 0x87, "============================================", 0x0D ; 220  REM
        db 0xDD,0x00, 0x87, "VORTEX.BAS V1.0 - TRIG LIBRARY STRESS TEST", 0x0D ; 221  REM
        db 0xDE,0x00, 0x87, "RENDERS A WARPED 3D SPIRAL VORTEX TO TEST:", 0x0D ; 222  REM
        db 0xDF,0x00, 0x87, "SIN, COS, TAN, ASIN, ACOS, ATN, SQRT", 0x0D ; 223  REM
        db 0xE0,0x00, 0x87, "============================================", 0x0D ; 224  REM
        db 0xE1,0x00, "H=27", 0x0D ; 225
        db 0xE2,0x00, "V=13", 0x0D ; 226
        db 0xE3,0x00, 0x8F, "R=0", 0x94, "26", 0x0D ; 227 FOR R=0 TO(0x94) 26
        db 0xE4,0x00, 0x8F, "C=0", 0x94, "60", 0x0D ; 228 FOR C=0 TO(0x94) 60
        db 0xE5,0x00, "X=(C-30)/H", 0x0D ; 229
        db 0xE6,0x00, "Y=(R-13)/V", 0x0D ; 230
        db 0xE7,0x00, "D=SQRT(X*X+Y*Y)", 0x0D ; 231
        db 0xE8,0x00, 0x81, "D>1.2", 0x93, 0x82, "255", 0x0D ; 232 IF THEN(0x93) GOTO(0x82) 255
        db 0xE9,0x00, 0x81, "X=0", 0x93, 0x82, "236", 0x0D ; 233 IF THEN(0x93) GOTO(0x82) 236
        db 0xEA,0x00, "T=ATN(Y/X)", 0x0D ; 234
        db 0xEB,0x00, 0x82, "237", 0x0D ; 235 GOTO 237
        db 0xEC,0x00, "T=1.5708", 0x0D ; 236
        db 0xED,0x00, 0x87, "--- TEST SIN/COS ---", 0x0D ; 237  REM
        db 0xEE,0x00, "W=SIN(6*D-3*T)", 0x0D ; 238
        db 0xEF,0x00, 0x87, "--- TEST TAN ---", 0x0D ; 239  REM
        db 0xF0,0x00, "U=TAN(W*0.5)", 0x0D ; 240
        db 0xF1,0x00, 0x87, "--- BOUND VALUE TO [-0.99, 0.99] ---", 0x0D ; 241  REM
        db 0xF2,0x00, "P=COS(U)*0.99", 0x0D ; 242
        db 0xF3,0x00, 0x87, "--- TEST ASIN/ACOS ---", 0x0D ; 243  REM
        db 0xF4,0x00, "A=ACOS(P)", 0x0D ; 244
        db 0xF5,0x00, "B=ASIN(P)", 0x0D ; 245
        db 0xF6,0x00, 0x87, "--- MATH SHADE VALUE ---", 0x0D ; 246  REM
        db 0xF7,0x00, "Z=(A-B)/3.1416", 0x0D ; 247
        db 0xF8,0x00, 0x87, "--- MAP TO ASCII CHARS (recalibrated -- see change history) ---", 0x0D ; 248  REM
        db 0xF9,0x00, "S=32", 0x0D ; 249
        db 0xFA,0x00, 0x81, "Z>-0.36", 0x93, "S=46", 0x0D ; 250 IF THEN(0x93) S=46
        db 0xFB,0x00, 0x81, "Z>-0.3", 0x93, "S=43", 0x0D ; 251 IF THEN(0x93) S=43
        db 0xFC,0x00, 0x81, "Z>-0.24", 0x93, "S=79", 0x0D ; 252 IF THEN(0x93) S=79
        db 0xFD,0x00, 0x81, "Z>-0.18", 0x93, "S=64", 0x0D ; 253 IF THEN(0x93) S=64
        db 0xFE,0x00, 0x80, "CHR$(S);", 0x0D ; 254 PRINT
        db 0xFF,0x00, 0x90, "C", 0x0D ; 255 NEXT C
        db 0x00,0x01, 0x80, 0x0D ; 256 PRINT
        db 0x01,0x01, 0x90, "R", 0x0D ; 257 NEXT R
        db 0x02,0x01, 0x88, 0x0D ; 258 END
        ; ── Mandelbrot: native MBF4 float, no fixed-point scaling needed ─────
        db 0x18,0x01, 0x80, 0x22, "--- MANDELBROT (FLOAT) ---", 0x22, 0x0D ; 280 PRINT
        db 0x22,0x01, 0x8F, "I=-1", 0x94, "1 ", 0x95, "0.18", 0x0D ; 290 FOR I=-1 TO(0x94) 1 STEP(0x95) 0.18
        db 0x2C,0x01, 0x8F, "C=-2", 0x94, "0.5 ", 0x95, "0.045", 0x0D ; 300 FOR C=-2 TO(0x94) 0.5 STEP(0x95) 0.045
        db 0x36,0x01, "A=C", 0x0D                  ; 310
        db 0x37,0x01, "B=I", 0x0D                  ; 311
        db 0x38,0x01, "E=0", 0x0D                  ; 312
        db 0x40,0x01, 0x8F, "N=1", 0x94, "16", 0x0D ; 320 FOR N=1 TO(0x94) 16
        db 0x4A,0x01, "T=A*A-B*B+C", 0x0D          ; 330
        db 0x54,0x01, "B=2*A*B+I", 0x0D            ; 340
        db 0x5E,0x01, "A=T", 0x0D                  ; 350
        db 0x68,0x01, 0x81, "A*A+B*B>4", 0x93, 0x8D, "600", 0x0D ; 360 IF THEN(0x93) GOSUB 600
        db 0x72,0x01, 0x90, "N", 0x0D              ; 370 NEXT N
        db 0x7C,0x01, 0x81, "E>0", 0x93, 0x80, "CHR$(E+32);", 0x0D ; 380 IF THEN(0x93) PRINT
        db 0x86,0x01, 0x81, "E=0", 0x93, 0x80, "CHR$(32);", 0x0D ; 390 IF THEN(0x93) PRINT
        db 0x90,0x01, 0x90, "C", 0x0D              ; 400 NEXT C
        db 0x9A,0x01, 0x80, 0x22, 0x22, 0x0D       ; 410 PRINT ""
        db 0xA4,0x01, 0x90, "I", 0x0D              ; 420 NEXT I
        db 0xAE,0x01, 0x88, 0x0D                   ; 430 END
        ; ── Subroutine 500: sum 1..10 ──────────────────────────────────────
        db 0xF4,0x01, "S=0", 0x0D                  ; 500
        db 0xFE,0x01, 0x8F, "J=1", 0x94, "10", 0x0D ; 510 FOR J=1 TO(0x94) 10
        db 0x08,0x02, "S=S+J", 0x0D                ; 520
        db 0x12,0x02, 0x90, "J", 0x0D              ; 530 NEXT J
        db 0x1C,0x02, 0x8E, 0x0D                   ; 540 RETURN
        ; ── Subroutine 550: factorial 5 ───────────────────────────────────
        db 0x26,0x02, "F=1", 0x0D                  ; 550
        db 0x30,0x02, 0x8F, "K=1", 0x94, "5", 0x0D ; 560 FOR K=1 TO(0x94) 5
        db 0x3A,0x02, "F=F*K", 0x0D                ; 570
        db 0x44,0x02, 0x90, "K", 0x0D              ; 580 NEXT K
        db 0x4E,0x02, 0x8E, 0x0D                   ; 590 RETURN
        ; ── Subroutine 600: record Mandelbrot escape iteration ───────────
        db 0x58,0x02, 0x81, "E=0 ", 0x93, "E=N", 0x0D ; 600 IF THEN(0x93)
        db 0x62,0x02, 0x8E, 0x0D                   ; 610 RETURN
        dw 0                                                                ; end sentinel
SHOWCASE_END:
        times ORIGIN-($-$$) db 0
%else
        org ORIGIN
%endif

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
        cli
        ; ROM: CS=0xF000 after far JMP.  RAM at segment 0.
        xor  ax, ax
%endif
        mov  ds, ax
        mov  es, ax
        mov  ss, ax
        mov  sp, STACK_TOP
        mov  di, RAM_BASE

%ifndef __YASM_MAJOR__
        ; Zero ALL RAM first (variables, FOR stack, program store, IVT area).
        ; Must happen BEFORE setting PROG_END.
        mov  cx, RAM_SIZE / 2
%else
        ; Zero only the vars area (program store holds showcase).
        mov  cx, PROGRAM / 2    ; PROGRAM/2 words (symbolic; see RAM LAYOUT)
        xor  ax, ax
%endif
        rep  stosw

%ifndef __YASM_MAJOR__
        ; PROG_END: empty program (set after rep stosw so it isn't wiped)
        mov  word [PROG_END], PROGRAM
%else
        ; PROG_END: just past last showcase byte (excluding sentinel)
        mov  word [PROG_END], PROGRAM + (SHOWCASE_END - SHOWCASE_DATA) - 2
%endif
        mov  word [RND_SEED], 0xACE1    ; seed LFSR

        ; Signon banner; fall through to main_loop
        mov  si, str_banner
        call dp_str
        call do_free

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
        call stmt                ; no line number: execute immediately
                                  ; (was stmt_line -- multi-statement colon
                                  ; support removed v2.7, see change history)
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
; PEEK_LINE  test whether SI is at end-of-line (CR)
; v2.7: was "end-of-statement (CR or ':')" -- ':' dropped along with
; multi-statement colon support, see change history. Label kept as
; 'sl_ret' for its single RET (was also STMT_LINE's return point before
; that routine was removed in the same change; name is now just legacy).
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
; v2.0: expr returns float in FLT_A; truncated to int16 here for the
; branch test (the only place in this file that needs expr's result as a
; plain boolean rather than a value to store/print/use further).
; Inputs  : SI -> expression text
; Clobbers: AX, BX, CX, DX, SI, FLT_A, FLT_B, FLT_C (via expr / stmt)
; =============================================================================
do_if:
        call expr
        call flt_to_int          ; AX = int16(FLT_A)
        or   ax, ax
        je   do_if_false
        call spaces
        cmp  byte [si], TK_THEN
        jne  di_kw_then
        inc  si                 ; consume token
        jmp  stmt

di_kw_then:
        mov  bx, then_tab       ; THEN is optional in direct mode
        call kw_match
        ; fall through to stmt

; =============================================================================
; STMT  execute one statement from SI
; Token fast-path: stored programs use keyword tokens (0x80+).
; Direct-mode input falls through to the kw_match loop.
; Inputs  : SI -> statement (token byte or raw text)
; Outputs : (none)
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
; v2.0: expr now always returns float in FLT_A directly (see expr's own
; header for the precision-loss bug this fixes) -- no promotion needed
; here.
; Inputs  : SI -> variable name
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, FLT_C
; =============================================================================
do_let:
        call get_var_addr       ; DI = &var, SI advanced
        push di
        call expect_equals      ; consume '='
        call expr               ; FLT_A = result
        jmp  short var_store

; =============================================================================
; DO_INPUT  INPUT <var>
; Inputs  : SI -> variable name
; Clobbers: AX, DI, SI, FLT_A (v2.0: expr's result is float, in FLT_A)
; =============================================================================
do_input:
        call get_var_addr       ; DI = &var, SI advanced
        push di
        call question
        push si                 ; save program pointer
        call input_line         ; resets SI -> IBUF
        call expr               ; parse expression -> FLT_A
        pop  si                 ; restore program pointer
        ; fall through to var_store

; =============================================================================
; VAR_STORE  shared assignment tail for DO_LET and DO_INPUT
; Inputs  : FLT_A = value to store (expr's result), DI = &var on stack
; Outputs : VARS[var] = FLT_A
; Clobbers: AX, SI, DI (v2.0: widened from 2-byte stosw to 4-byte float copy)
; v2.3: tail-shared with CP4 (see FLT_A_TO_B) instead of inlining its own
; copy -- DF is already clear here (nothing between here and program start
; sets it), so CP4's own copy handles it.
; =============================================================================
var_store:
        pop  di
        push si
        mov  si, FLT_A
        jmp  cp4

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
        add  ax, ax             ; x2
        add  ax, ax             ; x4 : 4-byte float stride (v2.0)
        add  ax, VARS
        xchg di, ax
        ret

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
        clc
        ret
.fail:
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
        ; Determine keyword table base: sub-keywords (TK_THEN/TO/STEP)
        ; use then_tab as base; statement tokens use tk_kw_tab.
        mov  bx, tk_kw_tab
        cmp  al, TK_THEN
        jb   .dl_have_base      ; below TK_THEN: statement token, tk_kw_tab ok
        cmp  al, TK_STEP
        ja   dl_raw             ; above TK_STEP: unknown, print raw
        ; It is TK_THEN/TK_TO/TK_STEP: rebase to then_tab and re-bias AL
        mov  bx, then_tab
        sub  al, TK_THEN        ; 0-based index within sub-keyword block
        add  al, TK_PRINT       ; get_token_ptr will subtract TK_PRINT back

.dl_have_base:
        ; --- TOKEN HANDLING ---
        push si
        push ax                 ; Save (possibly adjusted) token index

        ; Check for leading space - Don't print if:
        test dl, dl             ; 1. Start of line (DL=0)
        jz   .skip_leading
        cmp  dl, ' '            ; 2. Prev was space (DL=' ')
        je   .skip_leading
        cmp  dl, 1              ; 3. Prev was a token (DL=1)
        je   .skip_leading
        call output_space

.skip_leading:
        pop  ax                 ; Restore token index
        call get_token_ptr      ; BX -> table entry (BX was set above)
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
; Inputs  : SI -> print list
; Outputs : (none)
; Clobbers: AX, BX, CX, SI
; =============================================================================
do_print:
dp_top:
        call peek_line
        je   dp_nl              ; bare PRINT -> newline
        cmp  byte [si], '"'
        jne  dp_chrs
        inc  si                 ; skip opening quote

; DP_STR  print a quoted string body up to the closing '"', OR a
; bit-7-terminated ROM string up to its terminator -- also called
; directly (not just reached via fallthrough) by DO_FREE and by the
; boot banner to print a ROM string standalone.
; Clobbers: AX, SI
dp_str:
        lodsb
        cmp  al, 0x22           ; closing '"'?
        je   dp_after
        test al, 0x80           ; bit-7 terminator (ROM string)?
        jz   loop_print
        and  al, 0x7F
        jmp  output             ; tail-call
loop_print:
        call output
        jmp  short dp_str

dp_chrs:
        mov  bx, chrs_tab
        call kw_match
        jc   dp_tab
        call eat_paren_expr      ; FLT_A = arg
        call flt_to_int          ; AX = int16(FLT_A) = char code
        call output
        jmp  short dp_after
dp_tab:
        mov  bx, tab_tab
        call kw_match
        jc   dp_num
        call eat_paren_expr      ; FLT_A = arg
        call flt_to_int          ; AX = int16(FLT_A) = column count
        or   ax, ax              ; GOTCHA: LOOP decrements before testing
                                  ; CX, so CX<=0 here would wrap to a huge
                                  ; count instead of looping zero times
        jle  dp_after
        xchg ax, cx
tab_loop:
        call output_space
        loop tab_loop
        jmp  short dp_after
dp_num:
        ; v2.0: expr now always returns float in FLT_A directly (full
        ; precision preserved for plain arithmetic; relational/bitwise
        ; results, e.g. PRINT A&15 or PRINT X<Y, are promoted to float by
        ; expr itself before returning) -- no extra promotion needed here.
        call expr                ; FLT_A = result
        ; GOTCHA: flt_print clobbers SI (reuses IBUF as its own digit
        ; scratch) -- IBUF is also where the statement being parsed
        ; lives, so SI must be saved/restored here or DP_AFTER's ';'
        ; check reads garbage. See v2.1 change history.
        push si
        call flt_print           ; print FLT_A as decimal (clobbers SI)
        pop  si                  ; restore the real parse position
dp_after:
        call spaces
        cmp  byte [si], ';'
        jne  dp_nl
        inc  si
        call peek_line
        je   dp_ret
        jmp  short dp_top

; =============================================================================
; DO_FREE  print free program-store bytes (also provides dp_nl / newline)
; Inputs  : (none)
; Outputs : (none)
; Clobbers: AX, SI
; =============================================================================
do_free:
        mov  ax, PROGRAM_TOP
        sub  ax, [PROG_END]
        call num_space
        mov  si, kw_free
        call dp_str
dp_nl:
        jmp  new_line           ; tail-call
dp_ret:
        ret

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
        jmp  new_line           ; tail-call

; =============================================================================
; POKE_OUT_HLPR  parse "<addr>, <val>" pair shared by DO_POKE, DO_OUT,
; and DO_LIST's range syntax.
; v2.0: expr returns float; both operands truncated to int16 here (POKE/
; OUT addresses+values and LIST line numbers are always integer).
; Inputs  : SI -> argument text
; Outputs : DI = address, AL = value (low byte)
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_A
; =============================================================================
poke_out_hlpr:
        call expr               ; FLT_A = address
        call flt_to_int          ; AX = int16(FLT_A)
        push ax
        mov  al, ','
        call expect
        call expr               ; FLT_A = value
        call flt_to_int          ; AX = int16(FLT_A)
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
; v2.0: ALWAYS returns float in FLT_A 
; Inputs  : SI -> expression text
; Outputs : FLT_A = result
; Clobbers: AX, BX, CX, DX, SI, FLT_A, FLT_B
; =============================================================================
expr:
        call expr_add            ; FLT_A = left operand (full float precision)
        call spaces
        mov  al, [si]
        cmp  al, '<'
        je   .has_rel
        cmp  al, '='
        je   .has_rel
        cmp  al, '>'
        je   .has_rel
        ret                      ; no relational operator: FLT_A already
                                  ; holds the correct (float) result
.has_rel:
        call flt_a_push

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
        push dx                 ; save bitmask across RHS eval
        call expr_add            ; FLT_A = right operand (float)
        call flt_a_to_b         ; FLT_B = RHS
        pop  dx                 ; DL = operator bitmask
        call flt_a_pop           ; restore LHS float into FLT_A
        push dx                  ; DX (operator bitmask) must survive
                                  ; across FLT_CMP -- FLT_CMP documents DX
                                  ; as clobbered (see its own header), and
                                  ; this was previously left to chance:
                                  ; DL ended up 0 here every time, so the
                                  ; "test al,dl" below always saw a zero
                                  ; mask and every comparison evaluated
                                  ; false. See change history.
        call flt_cmp            ; AX = -1 (LT), 0 (EQ), +1 (GT)
        pop  dx                  ; restore operator bitmask (POP doesn't
                                  ; touch flags, so the comparison result
                                  ; tested just below survives intact)
        ; Map flt_cmp result to LT=1 / EQ=2 / GT=4 bitmask in AL
        or   ax, ax
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
        jmp  flt_from_int        ; tail-call: FLT_A = float(AX)

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
; TAB_SEARCH  shared linear scan for a char(1)+handler_ptr(2) operator
; table, terminated by a 0x00 sentinel char. Used by PREC_ENGINE_F (was
; also used by EXPR_BITWISE before bitwise operators were removed -- see
; v2.6 change history).
; Inputs  : BX -> table, DL = char to find
; Outputs : found:     BX -> matching entry, CF=0
;           not found: BX -> sentinel entry, CF=1
; Clobbers: none else
; =============================================================================
tab_search:
        cmp  byte [bx], 0
        je   .notfound
        cmp  [bx], dl
        je   .found
        add  bx, 3
        jmp  tab_search
.notfound:
        stc
        ret
.found:
        clc
        ret

; =============================================================================
; PREC_ENGINE_F  generic left-associative FLOAT binary operator evaluator.
; Used by expr_add/expr1 (the float-typed levels). Operands are 4-byte
; floats in FLT_A/FLT_B; the LHS is parked on the real CPU stack across
; the RHS sub-call.
; GOTCHA: this routine recurses into ITSELF across precedence levels
; (expr_add's RHS sub-call is expr1, itself prec_engine_f-based) -- the
; LHS park MUST live on the real stack, not a fixed scratch location,
; or a nested call's own park will clobber the outer one. See v2.1
; change history for the concrete failure this caused before the fix.
; Inputs  : BX -> operator table {char(1), handler_ptr(2), ...}, 0x00 sentinel
;           DI = pointer to next-lower-precedence function (float-returning,
;           result left in FLT_A)
; Outputs : FLT_A = result
; Clobbers: AX, BX, CX, DX, SI, FLT_A, FLT_B
; =============================================================================
prec_engine_f:
        push bx                 ; save operator table pointer
        push di                 ; save next-level function pointer
        call di                 ; get initial LHS -> FLT_A
.lp:
        mov  bp, sp
        mov  di, [bp]           ; DI = next-level func
        mov  bx, [bp+2]         ; BX = operator table
        call spaces
        mov  dl, [si]           ; peek operator char
        call tab_search
        jc   .done
        inc  si                 ; consume operator char
        mov  dx, [bx+1]          ; DX = handler address -- fetched BEFORE
                                  ; FLT_A_PUSH runs, because FLT_A_PUSH (like
                                  ; FLT_A_POP) clobbers BX internally; see
                                  ; their shared header. Grabbing this first
                                  ; means nothing needs BX preserved across
                                  ; either helper call below.
        call flt_a_push          ; park LHS on the real stack
        mov  di, [bp]           ; DI = next-level func (BP still valid:
                                 ; nothing between the top of .lp and here
                                 ; touches it)
        push dx                 ; save handler address
        call di                 ; get RHS -> FLT_A
        call flt_a_to_b         ; FLT_B = RHS
        pop  di                 ; DI = handler
        call flt_a_pop           ; restore LHS from the real stack
        call di                 ; FLT_A = FLT_A op FLT_B
        jmp  .lp
.done:
        add  sp, 4              ; discard saved BX and DI
        ret

; =============================================================================
; EXPR_ADD  additive level (+ -).  v2.0: float-typed, via prec_engine_f.
; Inputs  : SI -> expression text
; Outputs : FLT_A = result
; Clobbers: AX, BX, CX, DX, SI, FLT_A, FLT_B
; =============================================================================
expr_add:
        mov  bx, tab_add
        mov  di, expr1
        jmp  short prec_engine_f

; =============================================================================
; EXPR1  multiplicative level (* / %).  v2.0: float-typed, via prec_engine_f.
; Inputs  : SI -> expression text
; Outputs : FLT_A = result
; Clobbers: AX, BX, CX, DX, SI, FLT_A, FLT_B, FLT_C
; =============================================================================
expr1:
        mov  bx, tab_mul
        mov  di, expr2          ; functions are highest precedence
        jmp  short prec_engine_f

; =============================================================================
; FLT_MOD  FLT_A = FLT_A modulo FLT_B (truncating, like int16 % did)
; Both operands truncated to int16, divided with math_mod's int semantics,
; result promoted back to float. (BASIC's % was always integer-only; kept
; that way rather than implementing a true float remainder.)
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A = int16(FLT_A) mod int16(FLT_B), as a float
; Clobbers: AX, BX, CX, DX, FLT_A, FLT_B
; =============================================================================
flt_mod:
        call flt_to_int         ; AX = int16(FLT_A)
        push ax
        call flt_b_to_a
        call flt_to_int         ; AX = int16(FLT_B)
        pop  cx                 ; CX = int16(FLT_A) (the dividend)
        xchg ax, cx              ; AX = dividend, CX = divisor
        call math_mod            ; AX = AX mod CX (int16, may raise div_err)
        jmp  flt_from_int        ; tail-call: FLT_A = float(AX)

; =============================================================================
; MATH_MOD / MATH_DIV  int16 primitives, used only by FLT_MOD and RND(n).
; Inputs  : AX = left operand, CX = right operand
; Outputs : AX = result (math_mod), or AX=quotient/DX=remainder (math_div)
; Clobbers: DX
; =============================================================================
math_mod:
        call math_div
        xchg ax, dx             ; return remainder
        ret

math_div:
        or   cx, cx
        jne  .ok
        jmp  div_err             ; out of short-jump range; long jmp instead
.ok:
        cwd
        idiv cx
        ret

; =============================================================================
; EXPR2  factor level: unary operators, built-in functions, literals, variables
; v2.0: float-returning (result in FLT_A), matching expr_add/expr1 above.
; Inputs  : SI -> factor text
; Outputs : FLT_A = value
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_A
; =============================================================================
e2_pos:
        inc  si
        ; fall through to expr2
expr2:
        call spaces
        mov  al, [si]
        cmp  al, '('
        jne  .not_paren
        ; v3.3 BUG FIX: this used to be a plain 'je e2_par', jumping
        ; straight into E2_PAR without ever consuming the '(' -- E2_PAR
        ; is also reached from EAT_PAREN_EXPR, which DOES consume it
        ; first via its own 'mov al,"(" / call expect' (that's why
        ; function calls like SIN(x) always worked: they go through
        ; EAT_PAREN_EXPR, not this path). A bare '(expr)' used outside
        ; a function call landed here instead, called EXPR with SI still
        ; pointing at the same unconsumed '(', which recursed straight
        ; back into this same branch -- genuine infinite recursion,
        ; confirmed via a live trace (SI stuck, SP decreasing ~16
        ; bytes/level) rather than a deliberate "not supported" syntax
        ; error. Fixed by consuming '(' here too before jumping in.
        inc  si
        jmp  e2_par
.not_paren:
        cmp  al, '-'
        jne  .not_neg
        jmp  e2_neg              ; out of short-jump range
.not_neg:
        cmp  al, '+'
        je   e2_pos

        ; Scan function dispatch table
        mov  bx, func_tab
e2_func_lp:
        cmp  word [bx], 0       ; sentinel?
        jne  .have_entry
        jmp  e2_nusr             ; out of short-jump range
.have_entry:
        push bx
        call kw_match
        pop  bx
        jnc  e2_func_call
        add  bx, 4              ; next entry: kw_ptr(2) + handler_ptr(2)
        jmp  e2_func_lp

e2_func_call:
        push bx
        call eat_paren_expr     ; FLT_A = argument value
        pop  bx
        jmp  [bx+2]             ; indirect jump to handler

; =============================================================================
; EAT_PAREN_EXPR  parse '(' <expr> ')' -> FLT_A
; Inputs  : SI -> '('
; Outputs : FLT_A = expression value, SI advanced past ')'
; Clobbers: AX, BX, CX, DX, SI, FLT_A, FLT_B, FLT_C
; =============================================================================
eat_paren_expr:
        mov  al, '('
        call expect
e2_par:
        call expr               ; FLT_A = result
        call flt_a_push
        mov  al, ')'
        call expect
        call flt_a_pop
        ret

; =============================================================================
; DO_PEEK_FUNC  PEEK(addr) -> byte at memory address
; v2.0: argument truncated to int16 (address), result promoted back to
; float so it composes with the float expression chain above expr2.
; Inputs  : FLT_A = address (from eat_paren_expr)
; Outputs : FLT_A = zero-extended byte value, as a float
; Clobbers: AX, BX, FLT_A
; =============================================================================
do_peek_func:
        call flt_to_int         ; AX = int16(FLT_A) = address
        xchg bx, ax
        mov  al, [bx]
        xor  ah, ah             ; zero-extend to 16-bit
        jmp  flt_from_int        ; tail-call: FLT_A = float(AX)

; =============================================================================
; DO_IN_FUNC  IN(port) -> byte from I/O port
; v2.0: argument truncated to int16 (port), result promoted back to float.
; Inputs  : FLT_A = port number (from eat_paren_expr)
; Outputs : FLT_A = zero-extended byte value, as a float
; Clobbers: AX, DX, FLT_A
; =============================================================================
do_in_func:
        call flt_to_int         ; AX = int16(FLT_A) = port
        xchg dx, ax
        in   al, dx
        xor  ah, ah             ; zero-extend to 16-bit
        jmp  flt_from_int        ; tail-call: FLT_A = float(AX)

; do_usr_func is placed near the reset vector (acts as space filler); see below.

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
        ret

; =============================================================================
; E2_VAR  load variable value at factor level
; v2.0: 4-byte float load (was 2-byte int).
; Inputs  : SI -> variable letter
; Outputs : FLT_A = variable value
; Clobbers: AX, SI, DI
; =============================================================================
e2_var:
        call get_var_addr        ; DI = &VARS[var]
        push si
        mov  si, di
        mov  di, FLT_A
        jmp  cp4

; =============================================================================
; E2_NEG  unary negation factor.  v2.0: float-native (was int16 neg ax).
; Inputs  : SI -> '-' followed by factor text
; Outputs : FLT_A = -factor
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_A, FLT_B, FLT_C
; =============================================================================
e2_neg:
        inc  si
        call expr2
        jmp  flt_negate          ; tail-call: FLT_A = -FLT_A

; =============================================================================
; E2_NUSR  number-or-variable dispatch (after function table miss)
; Routes to flt_parse (decimal literal, v2.0: was input_number) or e2_var
; (letter).
; =============================================================================
e2_nusr:
        mov  al, [si]           ; reload: kw_match may have clobbered AL
        cmp  al, '0'
        jb   e2_var
        cmp  al, '9'
        ja   e2_var
        jmp  flt_parse           ; tail-call: FLT_A = parsed literal

; =============================================================================
; INPUT_NUMBER  parse unsigned decimal integer from [SI]
; v2.0: kept as a plain int16 scanner -- still used by main_loop to read a
; leading line number (always integer; never a float). No longer called
; from expr2 (see e2_nusr above, which now calls flt_parse instead).
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
        xor  cx, cx
ipl_lp:
        call input_key
        cmp  al, 0x08           ; backspace?
        jne  ipl_nbs
        or   cx, cx
        je   ipl_lp             ; buffer empty: ignore
        dec  di
        dec  cx
        call backsp
        call output_space
        call backsp
        jmp  ipl_lp

ipl_nbs:
        cmp  al, 0x0D           ; CR?
        je   ipl_cr
        cmp  cx, 62             ; buffer full? (62 chars + CR + guard byte)
        jb   ipl_store
        mov  al, 0x07           ; BELL -- audible "line full" feedback
        call output
        jmp  ipl_lp
ipl_store:
        call output
        stosb
        inc  cx
        jmp  ipl_lp
ipl_cr:
        stosb
        mov  si, IBUF
        jmp  new_line

; =============================================================================
; OUTPUT_NUMBER  print signed 16-bit integer to terminal
; Inputs  : AX = signed 16-bit value
; Outputs : (none)
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
; NUM_SPACE  print AX as a signed decimal number, then a trailing space
; Inputs  : AX = value to print
; Outputs : (none)
; Clobbers: AX, CX, DX
; =============================================================================
num_space:
        call output_number
        jmp  output_space        ; tail-call

; =============================================================================
; NEW_LINE / QUESTION / BACKSP / OUTPUT_SPACE
;
; Four single-character output routines sharing one tail. Each may be
; called directly by name for its own character; the fallthrough chain
; between them is an internal size optimisation
; =============================================================================
; NEW_LINE
; Inputs  : (none)
; Outputs : CR (0x0D) then LF (0x0A)
; Clobbers: AX
new_line:
        mov  al, 0x0D
        call output
        mov  al, 0x0A
        db   0x3D	; consume next2 bytes
; QUESTION
; Inputs  : (none)
; Outputs : '?'
; Clobbers: AX
question:
        mov  al, '?'
        db   0x3D ; consume next2 bytes
; BACKSP
; Inputs  : (none)
; Outputs : backspace (0x08)
; Clobbers: AX
backsp:
        mov  al, 0x08
        db   0x3D
; OUTPUT_SPACE
; Inputs  : (none)
; Outputs : ' '
; Clobbers: AX
output_space:
        mov  al, ' '
        jmp  output             ; tail-call

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
        jmp  wl_lp

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
; Outputs : (none)
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
        cmp  [di], dx
        jne  el_noex
        ; BUGFIX (ported from prior audit): DELINE documents DX as
        ; clobbered (reused internally as SLIDE_DATA's byte-shift amount)
        ; but EDITLN still needs DX (the real line number) below at
        ; 'mov ax,dx'. Without this save/restore, replacing an existing
        ; line stores a garbage line number (the negated byte-length of
        ; the just-deleted old entry) instead of the real one -- e.g.
        ; editing line 20 a second time shows up as e.g. "-7" in LIST and
        ; becomes a permanently orphaned, unreachable duplicate entry.
        push dx
        push cx
        call deline             ; delete existing line
        pop  cx
        pop  dx
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
; Outputs : (none)
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
do_new:
        mov  word [PROG_END], PROGRAM
        mov  di, PROGRAM
        mov  cx, (PROGRAM_TOP - PROGRAM) / 2
clr_mem:
        xor  ax, ax
        rep  stosw              ; zeroes sentinel too
        ; fall through to do_end

; =============================================================================
; DO_END  END statement — stops program execution
; Inputs  : (none)
; Outputs : (none)
; Clobbers: AX
; =============================================================================
do_end:
        mov  ax, [PROG_END]
        mov  [RUN_NEXT], ax     ; RUN_NEXT -> sentinel -> run_loop will exit
        xor  al, al
run_end:
        mov  byte [RUNNING], al
dg_ret:
        ret

; =============================================================================
; DO_GOTO  GOTO <linenum>
; DO_RUN   RUN
; v2.0: expr returns float; truncated to int16 here (line numbers are
; always integer).
; Inputs  : SI -> line number expression (GOTO) or program start (RUN)
; Clobbers: AX, BX, CX, DX, DI, FLT_A
; =============================================================================
; =============================================================================
; PARSE_LINE_TARGET  shared "parse a line-number expression, look it up"
; tail for DO_GOTO and DO_GOSUB, which otherwise both inlined an identical
; "call expr / call flt_to_int / call find_line" sequence.
; Outputs : AX = target line# (int16), DI -> line >= AX (via FIND_LINE)
; Clobbers: AX, BX, CX, DX, DI, FLT_A
; =============================================================================
parse_line_target:
        call expr                ; FLT_A = target line#
        call flt_to_int          ; AX = int16(FLT_A)
        jmp  find_line           ; DI -> line >= AX; tail-call, ret goes to caller

do_goto:
        call parse_line_target
        cmp  [di], ax
        je   dg_common
JERRUL:
        mov  al, ERR_UL
        jmp  do_error

do_run:
        mov  di, PROGRAM
dg_common:
        mov  [RUN_NEXT], di
        cmp  byte [RUNNING], 0
        jne  dg_ret             ; already running (e.g. mid-GOTO): just return
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
        call stmt                ; (was stmt_line -- multi-statement colon
                                  ; support removed v2.7)
        jmp  short run_loop

; =============================================================================
; DO_GOSUB  GOSUB <linenum>
; Saves RUN_NEXT on dedicated GOSUB stack then jumps to target.
; v2.0: expr returns float; truncated to int16 here (line numbers are
; always integer).
; Inputs  : SI -> line number expression
; Clobbers: AX, BX, CX, DX, DI, FLT_A
; =============================================================================
do_gosub:
        call parse_line_target
        cmp  [di], ax
        jne  JERRUL
        mov  bx, [GOSUB_SP]
        cmp  bx, 8
        jb   gs_push
        jmp  ins_oom            ; overflow -> out of memory (?3)

gs_push:
        inc  word [GOSUB_SP]
        add  bx, bx              ; BX is already loaded, use it
; tinyasm has no LEA; encode "lea si,[GOSUB_STK+bx]" by hand (opcode
; 0x8D /r with mod=01 reg=SI(110) rm=BX(111), disp8=0x50=GOSUB_STK lo byte)
%ifdef __YASM_MAJOR__
        lea  si, [GOSUB_STK + bx]
%else
        db   0x8D, 0x77, 0x50
%endif
        mov  ax, [RUN_NEXT]
        mov  [si], ax
        mov  [RUN_NEXT], di      ; DI is target from find_line
        ret

gs_underflow:
        mov  al, ERR_RT
        jmp  do_error

; =============================================================================
; DO_RETURN  RETURN
; Pops return address from GOSUB stack and resumes execution.
; Inputs  : (none)
; Outputs : (none)
; Clobbers: AX, BX, SI
; =============================================================================
do_return:
        mov  si, GOSUB_SP
        mov  ax, [si]
        or   ax, ax
        jz   gs_underflow
        dec  ax
        mov  [si], ax
        add  ax, ax              ; byte offset = depth * 2
        xchg ax, bx
; tinyasm has no LEA; encode "lea si,[GOSUB_STK+bx]" by hand (see gs_push)
%ifdef __YASM_MAJOR__
        lea  si, [GOSUB_STK + bx]
%else
        db   0x8D, 0x77, 0x50
%endif
        lodsw
        mov  [RUN_NEXT], ax
        ret

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
; v2.0: start/limit/step are now float (4 bytes each); frame widened from
; 8 to 12 bytes/slot.
; Frame layout (12 bytes per slot in FOR_STK):
;   [bx+0] var_ptr(2)  [bx+2] limit(4)  [bx+6] step(4)  [bx+10] loop_ptr(2)
; Inputs  : SI -> line body after FOR token
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_A, FLT_B, FLT_C
; =============================================================================
df_syn:
        mov  al, ERR_OM
        jmp  do_error
do_for:
        call spaces
        call get_var_addr
        mov  [INS_TMP], di      ; save &var
        call expect_equals
        call expr               ; FLT_A = start value
        push si
        mov  si, FLT_A
        mov  di, [INS_TMP]
        movsw
        movsw                    ; initialise loop variable (4 bytes)
        pop  si
        ; TO is mandatory
        mov  al, TK_TO
        mov  bx, to_tab
        call expect_token_or_kw
        jc   df_syn

        call expr               ; FLT_A = limit
        ; Stash the limit in FLT_C across the STEP expression below
        ; (which will itself overwrite FLT_A while being parsed). This
        ; is a single, non-nested use of FLT_C -- unlike prec_engine_f's
        ; old design (see its own header), nothing here recurses, so
        ; sharing FLT_C for this one stash is fine.
        push si
        mov  si, FLT_A
        mov  di, FLT_C
        movsw
        movsw
        pop  si

        ; STEP is optional (default = 1.0)
        mov  al, TK_STEP
        mov  bx, step_tab
        call expect_token_or_kw
        jc   df_one_step
        call expr               ; FLT_A = explicit step value
        jmp  short df_have_step
df_one_step:
        mov  ax, 1
        call flt_from_int        ; FLT_A = 1.0 (default step)
df_have_step:
        ; Push frame.  FLT_A = step, FLT_C = limit at this point.
        mov  cx, [FOR_SP]
        cmp  cl, 4
        jnb  df_syn             ; FOR stack full
        inc  word [FOR_SP]
        call for_ptr_hlp        ; BX -> frame slot
        mov  di, bx             ; DI = base address of frame
        mov  ax, [INS_TMP]      ; var_ptr
        stosw                   ; [di+0] = var_ptr, di += 2
        push si
        mov  si, FLT_C          ; limit
        movsw
        movsw                   ; [di+2..5] = limit, di += 4
        mov  si, FLT_A          ; step
        movsw
        movsw                   ; [di+6..9] = step, di += 4
        pop  si
        mov  ax, [RUN_NEXT]     ; loop_ptr
        stosw                   ; [di+10] = loop_ptr, di += 2
        ret

; =============================================================================
; FOR_PTR_HLP  convert FOR stack depth to frame pointer
; Inputs  : CX = depth index (0-based)
; Outputs : BX -> FOR_STK[CX]  (12 bytes per frame, v2.0: was 8)
; Clobbers: AX, BX
; =============================================================================
for_ptr_hlp:
        ; depth is always <= 3 (FOR stack capped at 4 slots, checked in
        ; DO_FOR), so AL*12 never exceeds AH=0 -- 8-bit MUL replaces the
        ; old shift-add sequence.
        mov  al, 12
        mul  cl                 ; AX = CL * 12 (AH stays 0, CL<=3)
        xchg ax, bx
        add  bx, FOR_STK
        ret

; =============================================================================
; EXPECT_TOKEN_OR_KW  match a sub-keyword by token byte or plain text
; Inputs  : AL = token value (e.g. TK_TO), BX -> keyword table entry (text match)
; Outputs : CF=0 matched (SI advanced), CF=1 no match
; Clobbers: AX, DI, DL
; =============================================================================
expect_token_or_kw:
        ; CMP with equal operands guarantees CF=0, and INC never touches
        ; CF (8086 quirk, so multi-word increment loops don't disturb an
        ; in-flight carry chain) -- so the explicit CLC on the match path
        ; is redundant and dropped.
        call spaces
        cmp  byte [si], al
        je   etk_match
        call kw_match
        ret
etk_match:
        inc  si                 ; CF still 0 from the CMP above
        ret

; =============================================================================
; DO_NEXT  NEXT <var>
; v2.0: limit/step/var are float; var += step via flt_add, exit test via
; flt_cmp.
; Increments loop variable, tests exit condition, loops or pops frame.
; Inputs  : SI -> line body after NEXT token
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_A, FLT_B, FLT_C
; =============================================================================
dn_no_for:
        mov  al, ERR_NF
        jmp  do_error
do_next:
        call spaces
        call get_var_addr       ; DI = &var
        mov  cx, [FOR_SP]
dn_search:
        jcxz dn_no_for          ; stack empty: no matching FOR
        dec  cx
        call for_ptr_hlp        ; BX -> frame
        cmp  [bx], di           ; var_ptr match?
        jne  dn_search

        ; GOTCHA: CX (the matched frame index, needed by DN_DONE below)
        ; must be saved across FLT_ADD and FLT_CMP -- both clobber CX
        ; per their own headers, same reasoning as BX's save/restore
        ; around FLT_CMP further down. See v2.1 change history.
        push cx                 ; save frame index across flt_add/flt_cmp

        ; var += step  (FLT_A = var, FLT_B = step, result -> var)
        ; BX (frame pointer) is never touched by the movsw setup below,
        ; so a single push/pop around just the flt_add call (which does
        ; clobber BX) is enough -- no need to also save/restore it
        ; around the FLT_A load.
        push di                 ; &var
        mov  si, di
        mov  di, FLT_A
        movsw
        movsw                   ; FLT_A = var
        lea  si, [bx+6]         ; &frame.step
        mov  di, FLT_B
        movsw
        movsw                   ; FLT_B = step
        push bx
        call flt_add            ; FLT_A = var + step
        pop  bx
        pop  di                 ; &var
        mov  si, FLT_A
        movsw
        movsw                   ; var = FLT_A (write back)

        ; Exit test: positive step -> exit when var > limit
        ;            negative step -> exit when var < limit
        ; flt_cmp(var, limit): AX = -1/0/1 (var<limit / == / var>limit)
        lea  si, [bx+2]         ; &frame.limit
        mov  di, FLT_B
        movsw
        movsw                   ; FLT_B = limit  (FLT_A already = var)
        push bx                 ; flt_cmp clobbers BX -- save frame ptr
        call flt_cmp            ; AX = sign(var - limit)
        pop  bx                 ; restore frame ptr
        pop  cx                 ; restore frame index (see GOTCHA above)

        ; AX (the -1/0/1 result) is used directly -- inc/dec + jz/jnz
        ; replaces the old push-ax/pop-dx + cmp-against-1-or-0xFF
        ; cascade. AX==1 (positive step, var>limit) makes 'dec ax' hit
        ; zero; AX==-1 (negative step, var<limit) makes 'inc ax' hit zero.
        mov  dl, [bx+7]         ; step's sign byte: frame.step is at
                                 ; bx+6..bx+9, MBF4 byte+1 holds the sign
                                 ; bit (bit 7), so bx+6+1 = bx+7
        test dl, 0x80
        jz   dn_pos_step
        inc  ax
        jz   dn_done
dn_loop:
        mov  ax, [bx+10]
        mov  [RUN_NEXT], ax     ; jump back to top of loop
        ret
dn_pos_step:
        dec  ax
        jnz  dn_loop
dn_done:
        mov  [FOR_SP], cx       ; pop frame (CX = correct new depth)
        ret


; =============================================================================
; DO_DELAY  DELAY <count>  (ROM / real-hardware build only)
; One unit ≈ 0.1 seconds at 5 MHz.  No effect in YASM/8bitworkshop build.
; v2.0: expr returns float; truncated to int16 here (loop counter is
; always integer).
; Inputs  : SI -> count expression
; Clobbers: AX, BX, CX, DX, FLT_A
; =============================================================================
do_delay:
        call expr
        call flt_to_int          ; AX = int16(FLT_A) = count
.outer_loop:
        mov  cx, 29412          ; ~0.1 s at 5 MHz (17 cy/iter)
.inner_loop:
        loop .inner_loop
        dec  ax
        jnz  .outer_loop
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
kw_delay:   db 0x44,0x45,0x4C,0x41,T_Y        ; DELAY
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
kw_sin:     db 0x53,0x49,T_N                  ; SIN
kw_cos:     db 0x43,0x4F,T_S                  ; COS
kw_tan:     db 0x54,0x41,T_N                  ; TAN
kw_atn:     db 0x41,0x54,T_N                  ; ATN
kw_asin:    db 0x41,0x53,0x49,T_N             ; ASIN
kw_acos:    db 0x41,0x43,0x4F,T_S             ; ACOS
kw_sqrt:    db 0x53,0x51,0x52,T_T             ; SQRT
        db 0                                    ; sentinel

; =============================================================================
; TOKEN -> KEYWORD STRING POINTER TABLE  (same order as st_tab / TK_xx)
; =============================================================================
tk_kw_tab:
        dw kw_print, kw_if, kw_goto, kw_list, kw_run, kw_new
        dw kw_input, kw_rem, kw_end, kw_let, kw_poke, kw_free
        dw kw_help, kw_gosub, kw_return
        dw kw_for, kw_next, kw_out, kw_delay
        dw kw_then, kw_to, kw_step     ; v2.2: sub-keywords now tokenized
        dw 0    ; GOTCHA: required sentinel 
        
; Sub-keyword pointer entries.
; then_tab/to_tab/step_tab double as a detokenizer table for do_list:
;   [then_tab + (token - TK_THEN) * 2]  -> correct kw_* pointer.
; Entries are contiguous so get_token_ptr arithmetic (BX=then_tab) works.
then_tab:   dw kw_then          ; TK_THEN = 0x93, index 0
to_tab:     dw kw_to            ; TK_TO   = 0x94, index 1
step_tab:   dw kw_step          ; TK_STEP = 0x95, index 2
; PRINT-only functions (single entry each; matched individually, not iterated)
chrs_tab:   dw kw_chrs
tab_tab:    dw kw_tab

; =============================================================================
; STRINGS  (bit-7 terminated)
; =============================================================================
str_banner: db "miniBASIC 8088 v3.5"
CRLF:       db 0x0D, 0x0A + 0x80

; =============================================================================
; STATEMENT HANDLER TABLE  (indexed by token - TK_PRINT, one word per entry)
; =============================================================================
st_tab:
        dw do_print,  do_if,     do_goto,   do_list,  do_run,   do_new
        dw do_input,  do_rem,    do_end,    do_let,   do_poke,  do_free
        dw do_help,   do_gosub,  do_return, do_for,   do_next,  do_out, do_delay

; =============================================================================
; OPERATOR TABLES  {char(1), handler_ptr(2), ...}, 0x00 sentinel
; =============================================================================

tab_add:                        ; additive level (v2.0: float)
        db '+'
        dw flt_add
        db '-'
        dw flt_sub
        db 0

tab_mul:                        ; multiplicative level (v2.0: float, except %)
        db '*'
        dw flt_mul
        db '/'
        dw flt_div
        db '%'
        dw flt_mod
        db 0

; =============================================================================
; FUNCTION DISPATCH TABLE  {kw_ptr(2), handler_ptr(2), ...}, dw 0 sentinel
; =============================================================================
func_tab:
        dw kw_rnd,  do_rnd_func
        dw kw_peek, do_peek_func
        dw kw_in,   do_in_func
        dw kw_usr,  do_usr_func
        dw kw_abs,  flt_abs
        dw kw_sin,  do_sin_func
        dw kw_cos,  do_cos_func
        dw kw_tan,  do_tan_func
        dw kw_atn,  do_atn_func
        dw kw_asin, do_asin_func
        dw kw_acos, do_acos_func
        dw kw_sqrt, do_sqrt_func
        dw 0

; (ROM_END moved below, after the MBF4 float library -- see merge note)

; =============================================================================
; MBF4 32-BIT FLOAT LIBRARY  (merged from mbfloat_v14.asm; see file header
; CHANGE HISTORY for the v2.0 merge notes and mbfloat's own v0.6-v0.14
; history below for everything upstream of the merge).
;
; FORMAT  (4-byte Microsoft Binary Format, single precision)
;   Byte 0   : biased exponent.  0x00 = exact zero.
;              Stored exponent = true_exponent + 0x80.
;              Value = (-1)^sign * 2^(exp-0x80) * 0.1mmm...
;   Byte 1   : bit7=sign, bits6:0 = mant[22:16]
;   Byte 2   : mant[15:8]
;   Byte 3   : mant[7:0]
;   Implied leading 1 at bit23 (restored during arithmetic).
;
; CALLING CONVENTION
;   FLT_A (4 bytes) = primary operand and result
;   FLT_B (4 bytes) = secondary operand
;   FLT_C (4 bytes) = LHS park, used only by this file's prec_engine glue
;                      (new in v2.0; the float library itself never
;                      touches FLT_C)
;   FLT_SA          = result sign (0x00 or 0x80)
;   FLT_SB          = sign of B (flt_add)
;   FLT_ER          = result exponent (flt_mul / flt_div)
;   FLT_DE          = flt_print decimal exponent / flt_parse sign stash
;   FLT_DB (3 bytes)= flt_div B-mantissa spill
;   AX = integer in/out.  SI preserved by all routines that clobber it.
;   IBUF is reused as flt_print's 7-digit extraction scratch; safe because
;   flt_print is never reached while a line is being edited or run mid-token.
;
; v2.0 MERGE-TIME CHANGES TO THIS LIBRARY (everything else below this
; header is byte-for-byte mbfloat_v14.asm's library body):
;
; NOTE ON LAYOUT (unchanged from mbfloat_v14.asm): small helpers
; (flt_zero, norm_pack, flt_negate, flt_b_to_a, flt_abs, copy helpers)
; are placed FIRST so that all JMPs from the arithmetic routines are
; backward jumps to lower addresses. With org 0xF000 a forward JMP of
; >0x1000 bytes would wrap around into RAM; backward JMPs to addresses
; above 0xF000 are always safe. This ordering is preserved exactly as
; in the original file -- do not reshuffle without re-checking every
; jmp/jcc target stays backward.
; =============================================================================

; =============================================================================
; FLT_ZERO  FLT_A = 0.0
; Inputs  : (none)
; Outputs : FLT_A = 0.0
; Clobbers: AX
; =============================================================================
flt_zero:
        xor  ax, ax
        mov  [FLT_A+0], ax
        mov  [FLT_A+2], ax
        ret

; =============================================================================
; NORM_PACK  normalise CH:DX, round, pack into FLT_A
;
; Placed early so all callers (flt_add, flt_mul, flt_div) jump backward.
;
; Register convention on entry:
;   BH = biased exponent
;   CH = mant[23:16]  (bit7 = implied-1 when normalised)
;   DH = mant[15:8]
;   DL = mant[7:0]
;   AL = sub-guard byte (round-half-up; not stored)
;   [FLT_SA] = result sign (0x00 or 0x80)
;
; Inputs  : BH, CH, DH, DL, AL, [FLT_SA]
; Outputs : FLT_A packed
; Clobbers: AX, BX, CX, DX
; =============================================================================
norm_pack:
np_lp:
        or   ch, ch
        js   np_round
        jnz  np_bit
        ; CH=0: byte-shift optimisation
        sub  bh, 8
        jbe  np_zero
        mov  ch, dh
        mov  dh, dl
        mov  dl, al
        xor  al, al
        jmp  np_lp
np_bit:
        ; 1-bit left-shift AL:DL:DH:CH. CF already 0 here (set by the
        ; 'or ch,ch' above, which is the only path into np_bit) so the
        ; explicit clc is dead weight -- dropped.
        rcl  al, 1
        rcl  dx, 1
        rcl  ch, 1
        dec  bh
        jnz  np_lp
np_zero:
        jmp  flt_zero

np_round:
        ; Round-half-up via sub-guard AL
        add  al, 0x80
        jnc  np_pack
        inc  dx                 ; ripples DL->DH carry as one 16-bit op;
                                 ; ZF set iff DX wrapped 0xFFFF->0x0000
                                 ; (both bytes maxed), same condition the
                                 ; old separate inc dl/inc dh pair checked
        jnz  np_pack
        inc  ch
        jnz  np_pack
        ; Mantissa overflow on round: all three incs above fell through,
        ; so dl=dh=ch=0 here exactly -- set ch=0x80 directly (cheaper than
        ; stc/rcr ch,1, which would compute the same result via rotation).
        mov  ch, 0x80
        inc  bh
        jz   np_zero

np_pack:
        mov  [FLT_A+0], bh
        mov  al, [FLT_SA]
        and  ch, 0x7F
        or   ch, al
        mov  [FLT_A+1], ch
        mov  [FLT_A+2], dh
        mov  [FLT_A+3], dl
        ret

; =============================================================================
; DO_ABS_FUNC  ABS(n) -> absolute value.  v2.0: float-native (was int16).
; Inputs  : FLT_A = value (from eat_paren_expr)
; Outputs : FLT_A = |value|
; Clobbers: AX
; =============================================================================
flt_abs:
        and  byte [FLT_A+1], 0x7F
        ret

; =============================================================================
; SIGN_XOR  FLT_SA = sign(FLT_A) XOR sign(FLT_B)
; Shared by flt_mul and flt_div (identical sequence in both).
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_SA = 0x00 or 0x80
; Clobbers: AL
; =============================================================================
sign_xor:
        mov  al, [FLT_A+1]
        xor  al, [FLT_B+1]
        and  al, 0x80
        mov  [FLT_SA], al
        ret

; =============================================================================
; Copy helpers
; v0.10: bodies merged into a shared 'cp4' tail (each entry point just
; loads SI/DI for its own direction, then falls/jumps into one copy).
; Clobbers: AX (word variants), or CX,SI,DI (movsb)
; =============================================================================
flt_b_to_a:
        push si
        mov  si, FLT_B
        mov  di, FLT_A
        jmp  cp4
flt_a_to_b:
        push si
        mov  si, FLT_A
        mov  di, FLT_B
cp4:
        movsw
        movsw
        pop  si
        ret

; =============================================================================
; FLT_A_PUSH / FLT_A_POP  shared "park all of FLT_A on the real stack"
; tail, used everywhere a routine needs to protect FLT_A across a sub-call
; that clobbers it. v2.3: factored out of 6 duplicated inline copies.
; GOTCHA: a plain CALL/RET wrapper doesn't work here -- CALL pushes the
; return address UNDER (push side) or ABOVE (pop side) the very words
; we're parking, so a bare 'ret' would either return into parked data or
; try to pop the return address as if it were FLT_A. Both routines instead
; pop the return address into BX up front and JMP back through it, which
; keeps the net stack effect identical to the old inline code (net +4
; bytes pushed, in order [FLT_A+2] then [FLT_A+0], on the push side).
; Clobbers: BX. Preserves flags.
; =============================================================================
flt_a_push:
        pop  bx
        push word [FLT_A+2]
        push word [FLT_A+0]
        jmp  bx

flt_a_pop:
        pop  bx
        pop  word [FLT_A+0]
        pop  word [FLT_A+2]
        jmp  bx

; FLT_B_POP  mirror of FLT_A_POP, pops into FLT_B instead. Added v3.0 for
; FLT_SIN, which parks values with FLT_A_PUSH but needs them back in
; FLT_B (as the second operand for a following FLT_MUL) rather than A.
flt_b_pop:
        pop  bx
        pop  word [FLT_B+0]
        pop  word [FLT_B+2]
        jmp  bx

; =============================================================================
; LOAD_HALF_PI_A  FLT_A = PI/2 (as MBF4)
; v2.9: factored out of 2 duplicated inline copies (FLT_ATAN's reciprocal
; branch, FLT_ACOS) -- found by asmdup.py after the ASIN/ACOS port.
; Inputs  : (none)
; Outputs : FLT_A = PI/2
; Clobbers: DI, SI
; =============================================================================
load_half_pi_a:
        mov  di, FLT_A
        mov  si, half_pi_const
        jmp cp5

; =============================================================================
; FLT_PI_2_B / FLT_PI_B / FLT_2PI_B  load PI/2, PI, or 2*PI into FLT_B
; v3.0, added for FLT_SIN's range reduction. PI and 2*PI are derived from
; PI/2 via the exponent-byte INC trick (same idea FLT_SQRT already uses
; for halving, just incrementing instead) rather than stored as their
; own MBF4 constants -- avoids adding two more 4-byte tables when the
; existing HALF_PI_CONST already gets us there for free.
; Inputs  : (none)
; Outputs : FLT_B = PI/2, PI, or 2*PI respectively
; Clobbers: DI, SI
; =============================================================================
flt_pi_2_b:
        mov  di, FLT_B
        mov  si, half_pi_const
cp5:        
        movsw
        movsw
        ret

; Do not Split flt_2pi_b from flt_pi_b
flt_pi_b:
        call flt_pi_2_b
        jmp  short pi_inc          ; PI/2 -> PI (shared with flt_2pi_b below)

flt_2pi_b:
        call flt_pi_b              ; FLT_B = PI, falls through for the
                                    ; 2nd increment
pi_inc:
        inc  byte [FLT_B+0]        ; exponent+1 -> value*2
        ret

; =============================================================================
; DO_RND_FUNC  RND(n) -> pseudo-random value in [0, n)
; v2.0: limit n truncated to int16, result promoted back to float.
; Inputs  : FLT_A = limit n (from eat_paren_expr)
; Outputs : FLT_A = value in range [0, n), as a float
; Clobbers: AX, BX, CX, DX, FLT_A
; =============================================================================
do_rnd_func:
        call flt_to_int         ; AX = int16(FLT_A) = limit
        push ax                 ; save limit
        call rnd_shuffle        ; advance LFSR -> AX
        pop  cx                 ; CX = limit
        call math_mod           ; AX = AX % CX
        ;jmp  flt_from_int        ; tail-call: FLT_A = float(AX)
        ; drop through
; =============================================================================
; FLT_FROM_INT  AX (signed int16) -> FLT_A
; FLT_FROM_INT_B  AX (signed int16) -> FLT_B  (preserves FLT_A and SI)
;
; v0.12: merged into one body via a DI destination parameter. The old
; flt_from_int_b wrote into FLT_A (via flt_from_int), saved/restored
; FLT_A's original bytes around that call, and copied the result into
; FLT_B via movsb (which needed SI, hence also saving/restoring SI).
; Writing directly to [di+n] means flt_from_int_b never touches FLT_A or
; SI at all -- preserved trivially, no save/restore needed for either.
;
; S5: direct stores only; no push/call flt_zero/pop.
; Normalisation loop: left-shift AX until bit15 set, counting down from 0x90.
;
; Inputs  : AX = signed 16-bit integer
; Outputs : FLT_A (flt_from_int) or FLT_B (flt_from_int_b)
; Clobbers: AX, BX, CX, DI
; =============================================================================
flt_from_int:
        mov  di, FLT_A
        DB 0x3B ; consume next 3 bytes
flt_from_int_b:
        mov  di, FLT_B
ffi_main:
        or   ax, ax
        je   ffi_zero
        mov  bl, 0x00           ; sign = positive
        jns  ffi_pos
        mov  bl, 0x80           ; sign = negative
        neg  ax
ffi_pos:
        mov  cl, 0x90           ; biased exponent for 2^16
ffi_lp:
        or   ax, ax             ; SF=bit15; cheaper than 'test ax,0x8000'
        js   ffi_found
        shl  ax, 1
        dec  cl
        jmp  ffi_lp
ffi_found:
        ; AX: bit15=implied-1, AH bits[6:0]=mant[22:16], AL=mant[15:8]
        mov  [di+0], cl
        and  ah, 0x7F
        or   ah, bl
        mov  [di+1], ah
        mov  [di+2], al
        mov  byte [di+3], 0
        ret
ffi_zero:
        xor  ax, ax
        mov  [di+0], ax
        mov  [di+2], ax
        ret

; =============================================================================
; FLT_TEN_B  FLT_B = 10  (preserves FLT_A and SI, via flt_from_int_b)
; Shared by flt_print and flt_parse, each of which loads the constant 10
; into FLT_B before a multiply, divide, or compare. 8 call sites collapse
; to one body + 8 short calls.
; Inputs  : —
; Outputs : FLT_B = 10
; Clobbers: AX, BX, CX, DI
; =============================================================================
flt_ten_b:
        mov  ax, 10
        jmp  flt_from_int_b     ; tail-call

; =============================================================================
; MUL_BY_TEN  FLT_A = FLT_A * 10
; Inputs  : FLT_A
; Outputs : FLT_A
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_SA, FLT_ER
;           (union of flt_ten_b's and flt_mul's own clobbers)
; =============================================================================
mul_by_ten:
        call flt_ten_b
        jmp  flt_mul            ; tail-call (forward - safe, within range)

; =============================================================================
; DIV_BY_TEN  FLT_A = FLT_A / 10
; Inputs  : FLT_A
; Outputs : FLT_A
; Clobbers: AX, BX, CX, DX, DI, FLT_SA, FLT_ER, FLT_DB
;           (union of flt_ten_b's and flt_div's own clobbers)
; =============================================================================
div_by_ten:
        call flt_ten_b
        jmp  flt_div            ; tail-call (forward - safe, within range)

; =============================================================================
; FLT_TO_INT  FLT_A -> AX (signed int16, truncate toward zero)
;
; S8 (orig): sign saved to DL (free during shift of BX); FLT_TS slot
;   eliminated.
; v0.10: the byte at FLT_A+1 is read once into BH and copied to DL (was
;   two separate memory reads); the dead 'and bh,0x7F' before 'or bh,0x80'
;   is dropped (OR forces bit7=1 regardless of the AND); and the sign
;   test is deferred to a post-loop 'shl dl,1' (bit7 -> CF) instead of
;   'and dl,0x80' + 'or dl,dl' -- no masking needed since only the
;   carry-out is used.
;
; True exponent e = exp-0x80. Extract top 16 bits of 24-bit mantissa into BX,
; right-shift by (16-e), apply sign.
;
; Inputs  : FLT_A
; Outputs : AX. Saturates at +/-32767; exact -32768 supported.
; Clobbers: AX, BX, CX, DX
; =============================================================================
flt_to_int:
        mov  al, [FLT_A+0]
        or   al, al
        jz   fti_zero
        sub  al, 0x80           ; true exponent
        ; v3.0 BUG FIX: was a single 'jle fti_zero' here. For AL<0x80
        ; (any fractional magnitude, true_exp<0), SUB AL,0x80 sets OF=1
        ; *and* SF=1 together (0x80 read as -128 for signed-overflow
        ; purposes conflicts with its intended use as a +128 bias), so
        ; JLE's SF!=OF test never fires and this fast path was silently
        ; skipped for every |value|<1 input -- unreachable until FLT_SIN
        ; (v3.0) became the first caller to ever pass flt_to_int a
        ; sub-1-magnitude value; every prior caller (PEEK/POKE/TAB/RND
        ; args) always passed magnitude >=1. Confirmed via a live
        ; single-step trace (SIN(0.00001) computing x/2*PI, which should
        ; truncate to 0, instead fell through into the shift-based path
        ; with a negative true_exp and came out with a large nonzero
        ; garbage AX). Root cause verified by hand from the standard
        ; x86 subtraction-overflow flag formula, not guessed. Splitting
        ; into ZF/SF-only checks (no OF) sidesteps the conflict; costs
        ; 2 bytes over the single JLE.
        jz   fti_zero           ; true_exp == 0 -> |value| < 1 (boundary)
        js   fti_zero           ; true_exp < 0  -> |value| < 1
        cmp  al, 16
        jg   fti_sat
        mov  bh, [FLT_A+1]
        mov  dl, bh             ; copy of A's sign+mantissa byte; bit7
                                 ; (the sign) is tested later via shl dl,1
        or   bh, 0x80           ; restore implied-1 (and bh,0x7F first
                                 ; was dead: or forces bit7=1 regardless)
        mov  bl, [FLT_A+2]
        mov  cl, 16
        sub  cl, al             ; shift = 16 - true_exp
        shr  bx, cl             ; native variable shift; cl=0 (true_exp==16,
                                 ; i.e. exact -32768) is a documented no-op
                                 ; on 8086 -- replaces the old do-while loop
                                 ; AND fixes its cl=0 wraparound bug for free
        xchg ax, bx              ; 1-byte XCHG-with-AX form; bx is dead
                                  ; afterward so the swap costs nothing
        shl  dl, 1              ; original bit7 (sign) -> CF; bits0-6
                                 ; don't matter, only CF is used below
        jnc  fti_r
        neg  ax
fti_r:  ret
fti_zero:
        xor  ax, ax
        ret
fti_sat:
        ; Genuine overflow (true_exp>16): saturate to +-32767 by sign.
        ; The old -32768-specific triple-check here (exp==0x90, byte1==0x80,
        ; bytes2:3==0) was unreachable -- this label is only entered when
        ; true_exp>16, but exact -32768 has true_exp==16 exactly, so it
        ; never got here. The shr bx,cl fix above now handles that value
        ; correctly on the normal path with no special-casing needed.
        test byte [FLT_A+1], 0x80
        jz   fti_sat_pos
fti_sat_neg:
        mov  ax, -32767
        ret
fti_sat_pos:
        mov  ax, 32767
        ret

; =============================================================================
; FLT_INT  FLT_A = truncate(FLT_A) toward zero, staying a float.
; v3.0, added for FLT_SIN's range reduction (needs floor(x/2*PI) as a
; float again, to multiply back by 2*PI and subtract). Trivial
; composition of two routines that already existed (this codebase had
; no float->float truncation before -- no BASIC INT() keyword either,
; unlike the codebase the ported FLT_SIN's "Reuses your BASIC's INT"
; comment assumed).
; GOTCHA: FLT_TO_INT saturates at +/-32767 rather than erroring, so
; |FLT_A| > 32767 silently truncates to a saturated value here too --
; same int16 ceiling this codebase's PEEK/POKE/TAB/etc. args already
; live with (see the file header), not a new limitation. FLT_SIN only
; ever calls this on x/2*PI after taking |x|, so this caps the usable
; input angle magnitude at a bit over 32767*2*PI (~205887) before SIN's
; range reduction itself goes wrong -- more than enough for realistic
; BASIC programs, but worth knowing.
; Inputs  : FLT_A
; Outputs : FLT_A = truncate(FLT_A)
; Clobbers: AX, BX, CX, DX, DI
; =============================================================================
flt_int:
        call flt_to_int          ; AX = int16(FLT_A), truncated
        jmp  flt_from_int         ; tail-call: FLT_A = float(AX)

; =============================================================================
; FLT_CMP  compare FLT_A with FLT_B (signed)
;
; Code-golf rewrite: compute A-B via the existing flt_sub (which already
; restores FLT_B internally), test the 32-bit result for exact zero, then
; map the sign bit of the difference straight to -1/+1 via the "SBB ternary
; trick" -- no branching needed for the nonzero case.
;
;   shl al,1   : sign bit of the difference -> Carry Flag
;   sbb ax,ax  : CF=1 -> AX=0xFFFF (-1); CF=0 -> AX=0x0000
;   or  al,1   : 0xFFFF stays 0xFFFF (-1); 0x0000 becomes 0x0001 (+1)
;
; FLT_A is saved/restored around the subtract (mov/pop do not affect flags,
; so the zero-flag from the difference test survives the restore intact).
; FLT_B is left unchanged because flt_sub itself restores it.
;
; Inputs  : FLT_A, FLT_B
; Outputs : AX = -1 (A<B), 0 (A==B), +1 (A>B). FLT_A and FLT_B unchanged.
; Clobbers: AX, BX, CX, DX, DI (via flt_sub's flt_negate_b calls)
; =============================================================================
flt_cmp:
        call flt_a_push
        call flt_sub
        mov  cx, [FLT_A+0]
        or   cx, [FLT_A+2]
        mov  al, [FLT_A+1]
        call flt_a_pop
        jz   fcmp_zero
        shl  al, 1
        sbb  ax, ax
        or   al, 1
        ret
fcmp_zero:
        xor  ax, ax
        ret

; =============================================================================
; FLT_SUB  FLT_A = FLT_A - FLT_B
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A
; Clobbers: AX, BX, CX, DX, DI, FLT_SA, FLT_SB
;           (union of flt_add's own clobbers plus DI, via flt_negate_b)
; =============================================================================
flt_sub:
        call flt_negate_b
        call flt_add
       ; jmp  flt_negate_b       ; restore FLT_B (tail-call)
        ; drop through
; =============================================================================
; FLT_NEGATE / FLT_NEGATE_B  flip the sign of FLT_A or FLT_B in place
; FLT_ABS  clear the sign of FLT_A in place (force positive)
;
; v0.13 design note: flt_negate/flt_negate_b are merged via a DI
; destination parameter (same pattern as v0.12's flt_from_int_b). This
; adds DI to flt_negate_b's clobber set, and transitively flt_sub's and
; flt_cmp's -- but DI is already fair-game scratch throughout this
; codebase (a documented clobber of flt_parse, flt_mul, flt_div, and
; flt_print already), so this is consistent with the existing
; convention, not a new constraint.
;
; Inputs  : FLT_A (flt_negate, flt_abs) or FLT_B (flt_negate_b)
; Outputs : same location, sign flipped (negate) or cleared (abs)
; Clobbers: flt_negate/flt_abs: nothing. flt_negate_b: DI.
; =============================================================================
flt_negate_b:
        mov  di, FLT_B
        DB 0x3B ; 
flt_negate:
        mov  di, FLT_A
fneg_tail:
        cmp  byte [di], 0
        je   flt_neg_r
        xor  byte [di+1], 0x80
flt_neg_r:
        ret

; =============================================================================
; FLT_ADD  FLT_A = FLT_A + FLT_B
;
; v0.12: FLT_T eliminated. Working mantissa CH:DX (24-bit) for the larger
; operand, as before; smaller operand now lives in AL(hi):BX(mid:lo), a
; pure register 24-bit value -- shr al,1/rcr bx,1 handles its alignment
; shift as one native 16-bit rotate plus one byte shift (the old memory
; version needed two separate byte-level rcrs), and the final add/sub
; against CH:DX is register-register instead of register-memory.
; This displaces the larger operand's exponent (formerly cached in BH)
; out of registers entirely -- it's reloaded from [FLT_A+0] (unchanged in
; memory throughout the routine) at the 3 points it's actually needed.
; Shift count in CL; result exponent in BH only from fa_norm_reload on.
;
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A = sum
; Clobbers: AX, BX, CX, DX, FLT_SA, FLT_SB
; =============================================================================
flt_add:
        ; BX doubles as a structural pointer to FLT_A for the early loads
        ; below ([bx+n] is a disp8 ref vs [FLT_A+n]'s disp16). AL/AH are
        ; also kept live across the fa_chkb/fa_both_nz jump (both are
        ; pure fall-through paths), removing the old redundant
        ; reload-and-recompare at fa_both_nz.
        mov  bx, FLT_A
        mov  al, [bx]
        test al, al
        jnz  fa_chkb
        jmp  flt_b_to_a         ; A=0 -> result=B
fa_chkb:
        mov  ah, [FLT_B+0]
        test ah, ah
        jnz  fa_both_nz
        ret                     ; B=0 -> result=A unchanged

fa_both_nz:
        ; Compare exponents; if B is larger, swap FLT_A<->FLT_B so the rest of
        ; the routine only has to handle ONE case ("FLT_A is the larger-or-
        ; equal operand"). This replaces what used to be two near-mirror
        ; 19-instruction load blocks with one swap (18 bytes) + one block.
        cmp  al, ah
        jnb  fa_signs           ; FLT_A already >= FLT_B, no swap needed

        mov  ax, [bx]
        xchg ax, [FLT_B+0]
        mov  [bx], ax
        mov  ax, [bx+2]
        xchg ax, [FLT_B+2]
        mov  [bx+2], ax

fa_signs:
        ; FLT_A is now guaranteed the larger-or-equal operand.
        mov  al, [bx+1]
        and  al, 0x80
        mov  [FLT_SA], al       ; sign of larger
        mov  al, [FLT_B+1]
        and  al, 0x80
        mov  [FLT_SB], al       ; sign of smaller

        ; Load larger (FLT_A) mantissa into CH:DX. Word-load+xchg gets
        ; DH:DL (mid:lo) from one word load instead of two byte loads.
        mov  dx, [bx+2]
        xchg dh, dl
        mov  ch, [bx+1]
        or   ch, 0x80

        ; Shift count (expA - expB) -- grab expA via BX before it's
        ; repurposed as the smaller operand's mantissa, below.
        mov  cl, [bx]
        sub  cl, [FLT_B+0]

        ; Load smaller (FLT_B) mantissa directly into AL(hi):BX(mid:lo) --
        ; no FLT_T memory scratch needed. The natural little-endian word
        ; load gives bl=mid,bh=lo (backwards); xchg corrects the order so
        ; BX, read as one 16-bit value, equals mid*256+lo.
        mov  bx, [FLT_B+2]
        xchg bh, bl             ; bh=B_mid, bl=B_lo
        mov  al, [FLT_B+1]
        or   al, 0x80           ; al=B_hi, implied-1 restored

fa_align:
        test cl, cl
        jz   fa_addorsub
        cmp  cl, 25             ; shift >= 25: smaller vanishes
        jb   fa_do_align
        jmp  fa_smaller_gone

fa_do_align:
fa_byte_lp:
        cmp  cl, 8
        jb   fa_bit_lp
        sub  cl, 8
        ; 8-bit downshift of the 24-bit AL:BH:BL value (mid->lo, hi->mid,
        ; zero-fill at top) -- replaces the old 3-byte memory shift.
        mov  bl, bh
        mov  bh, al
        xor  al, al
        test cl, cl
        jnz  fa_byte_lp
        jmp  fa_addorsub

fa_bit_lp:
        ; 24-bit right-shift in 2 instructions: shr al,1 needs no preceding
        ; clc (SHR never reads CF in, only produces it), and its carry-out
        ; feeds directly into the native 16-bit rcr bx,1 -- which handles
        ; the BH<->BL internal carry as part of one hardware op, where the
        ; old memory version needed two separate byte-level rcrs.
        shr  al, 1
        rcr  bx, 1
        dec  cl
        jnz  fa_bit_lp

fa_addorsub:
        ; AL/BX now hold the smaller operand's mantissa (needed below for
        ; the add/sub), so the sign-equality check uses CL instead -- it's
        ; guaranteed 0 here (the align loop above decrements it to exactly
        ; 0 on every exit path) and free until the loop runs again.
        mov  cl, [FLT_SA]
        cmp  cl, [FLT_SB]
        je   fa_same_sign

        ; Different signs: subtract smaller from larger. DX/BX share the
        ; same mid:lo byte-order convention (both built via the word-load+
        ; xchg trick above), so the low/high halves line up and a single
        ; 16-bit sub/sbb replaces the old 3x byte-wise sub/sbb/sbb.
        sub  dx, bx
        sbb  ch, al
        jnc  fa_norm_reload

        ; Borrow out: two's-complement negate the 24-bit CH:DX value in 3
        ; instructions (neg DX sets CF iff DX!=0; adc folds that carry
        ; into CH; neg CH completes it), then flip sign.
        neg  dx
        adc  ch, 0
        neg  ch
        test ch, ch
        jnz  fa_flip_sign
        test dx, dx
        jz   fa_zero
fa_flip_sign:
        xor  byte [FLT_SA], 0x80
        jmp  fa_norm_reload

fa_same_sign:
        ; Add with carry chain (LSB first). Register-register, same
        ; reasoning as the subtract path above.
        add  dx, bx
        adc  ch, al
        jnc  fa_norm_reload
        ; Carry out of CH: shift right 1, bump exponent (no guard byte).
        ; BH was the smaller operand's mid byte (dead now) -- reload the
        ; real exponent from memory before incrementing it.
        rcr  ch, 1
        rcr  dx, 1
        mov  bh, [FLT_A+0]
        inc  bh
        jz   fa_zero
        jmp  fa_norm            ; skip the (now-redundant) reload below

fa_smaller_gone:
fa_norm_reload:
        mov  bh, [FLT_A+0]      ; reload exponent (BH held smaller's mid
                                 ; byte through the add/sub above, or is
                                 ; still garbage on the fa_smaller_gone
                                 ; early-exit path)
fa_norm:
        ; fa_smaller_gone now falls through here instead of duplicating
        ; its own "mov bh,[FLT_A+0]/xor al,al/jmp norm_pack" tail. CH:DX
        ; on that path is always FLT_A's original (nonzero, since A was
        ; checked nonzero at entry) mantissa, so the zero-test below is
        ; always false there -- harmless, and it de-duplicates ~5 bytes
        ; of tail code at the cost of two untaken tests on that one path.
        test ch, ch
        jnz  fa_np
        test dx, dx
        jz   fa_zero
fa_np:
        xor  al, al
        jmp  norm_pack          ; backward jump - safe

fa_zero:
        jmp  flt_zero           ; backward jump - safe

; =============================================================================
; FLT_MUL  FLT_A = FLT_A * FLT_B
;
; S18: clean 4-step register plan mirroring MBF5 v0.5 (proven correct).
;
; Mantissa layout before multiply loop:
;   SI  = 00:B_hi  (B byte1 | 0x80, zero-extended)
;   DX  = B_lo     (B bytes 2:3 as word)
;   DI  = 00:A_hi  (A byte1 | 0x80, zero-extended)
;   stack[top]  = A_lo  (A bytes 2:3 as word)
;   stack[top+2]= 00:A_hi (=DI, for step 2 restore)
;
; Accumulator: BX (high word), CX (low word), AL (sub-guard)
;
; Step 1: A_lo * B_lo -> DX:AX; CX=DX (high 16); AL=AH (guard); pop A_lo
; Step 2: A_lo * B_hi -> DX:AX; CX+=AX; BX=DX; pop DX (=00:A_hi)
; Step 3: B_lo * A_hi -> DX:AX; CX+=AX; BX+=DX  (B_lo re-read from [FLT_B])
; Step 4: A_hi * B_hi -> DX:AX; BX+=AX           (DI*SI, AX fits in 16 bits)
;
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A = product
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_SA, FLT_ER
; =============================================================================
flt_mul:
        mov  al, [FLT_A+0]
        or   al, al
        jnz  fmul_anz
        ret                     ; 0 * x = 0
fmul_anz:
        mov  bl, [FLT_B+0]
        or   bl, bl
        jnz  fmul_bnz
        jmp  flt_zero           ; x * 0 = 0  (backward - safe)
fmul_bnz:
        push si

        ; Exponent: eA + eB - 0x80
        add  al, bl
        sub  al, 0x80
        mov  [FLT_ER], al

        ; Result sign = sign_A XOR sign_B (shared helper)
        call sign_xor

        ; Word-load + xchg replaces two byte-loads for both mantissa lo
        ; words below -- mem[n]/mem[n+1] land as AL/AH (or DL/DH) in the
        ; "wrong" (lo-byte-first) half via the natural little-endian
        ; word load; xchg swaps them back into the mid:lo order the
        ; multiply steps expect. 1 byte cheaper per pair than two
        ; separate byte moves.
        mov  dx, [FLT_B+2]
        xchg dh, dl              ; DX = B_lo
        mov  bl, [FLT_B+1]
        or   bl, 0x80           ; dead 'and bl,0x7F' removed
        xor  bh, bh
        mov  si, bx             ; SI = 0x00:B_hi

        ; Load A mantissa: DI=00:A_hi (byte in LOW position), AX=A_lo
        mov  ax, [FLT_A+2]
        xchg ah, al              ; AX = A_lo
        mov  cl, [FLT_A+1]
        or   cl, 0x80           ; dead 'and cl,0x7F' removed
        xor  ch, ch
        mov  di, cx             ; DI = 0x00:CL

        push di                 ; push 00:A_hi (for step 2 restore)
        push ax                 ; push A_lo    (for step 1)

        ; Step 1: A_lo * B_lo
        mul  dx                 ; DX:AX = A_lo * B_lo
        mov  cx, dx             ; CX = high word
        pop  ax                 ; restore A_lo (AX's pre-pop value is
                                 ; never observed -- the old 'mov al,ah /
                                 ; xor ah,ah' guard-byte setup here was
                                 ; dead code)

        ; Step 2: A_lo * B_hi
        mul  si                 ; DX:AX = A_lo * B_hi
        add  cx, ax
        adc  dx, 0
        mov  bx, dx
        pop  dx                 ; restore DX = 00:A_hi

        ; Step 3: B_lo * A_hi  (B_lo re-read; DX = 00:A_hi)
        mov  ax, [FLT_B+2]
        xchg ah, al              ; AX = B_lo
        mul  dx                 ; DX:AX = B_lo * A_hi
        add  cx, ax
        adc  bx, dx              ; carry from the add above folds
                                  ; straight into BX along with DX --
                                  ; replaces the old jnc/inc/add-bx,dx
                                  ; three-step

        ; Step 4: A_hi * B_hi  (DI=00:A_hi, SI=00:B_hi; product fits in AX)
        mov  ax, di
        mul  si                 ; DX:AX = A_hi * B_hi (DX discarded, always 0)
        add  bx, ax

        ; Normalise: BH bit7 must be set
        or   bh, bh
        js   fms_norm_ok
        ; Product < 0.5: left-shift BX:CX:AL, dec exponent
        dec  byte [FLT_ER]
        jnz  fms_do_shift
        pop  si
        jmp  flt_zero           ; exponent underflow - backward, safe
fms_do_shift:
        shl  al, 1
        rcl  cx, 1
        rcl  bx, 1

fms_norm_ok:
        ; Shuffle to norm_pack layout: CH=mant[23:16], DH=mant[15:8], DL=mant[7:0]
        ; BX: BH=mant[23:16], BL=mant[15:8]
        ; CX: CH=mant[7:0],   CL=noise
        mov  dh, bl             ; DH = mant[15:8]
        mov  dl, ch             ; DL = mant[7:0]
        mov  ch, bh             ; CH = mant[23:16]
        mov  bh, [FLT_ER]       ; BH = exponent
        pop  si
        jmp  norm_pack          ; backward jump - safe

; =============================================================================
; FLT_DIV  FLT_A = FLT_A / FLT_B
;
; 24-bit shift-and-subtract, 32 iterations. B mantissa (3 bytes) spilled
; to FLT_DB[0..2].
;
; GOTCHA: the remainder's high byte MUST live in a genuine 8-bit register
; (CH), not as a "00:byte" value inside a 16-bit register -- a 16-bit
; rcl carries in at bit0/out at bit15, which is the wrong end if the
; meaningful byte sits in the high half.
;
; GOTCHA: the quotient MUST accumulate in a genuine 32-bit pair (DX:AX),
; not DX:AL -- 32 iterations can produce up to 32 significant quotient
; bits before the leading 1 is known to have appeared. This also means
; AX/DX (the live quotient) cannot be reused as scratch for loading B's
; bytes during compare/subtract; B's bytes are read directly from
; [FLT_DB+n] instead.
;
; Loop registers:
;   DX:AX = 32-bit quotient accumulator (top 24 bits = mantissa, AL = guard)
;   BX    = remainder lo word (16-bit, live)
;   CH    = remainder hi byte (8-bit, live)
;   CL    = iteration counter
;
; Speculative (non-restoring) subtract: each iteration unconditionally
; subtracts B from CH:BX byte-wise; if the final byte borrows, the
; remainder was < B, so B is added back and the quotient bit stays 0;
; otherwise the subtraction is kept and the quotient bit is set. A
; 25th-bit shift-out (jc fdiv_ov) means the remainder is unconditionally
; >= B, so the subtract is committed without the borrow check.
;
; FLT_DB is little-endian (DB+0=B_lo, DB+1=B_mid, DB+2=B_hi) -- as a
; 16-bit word, DB+0 equals B_mid*256+B_lo, matching BX's existing
; mid*256+lo packing for A.
;   prescale check's separate AH/AL byte loads into one word compare,
;   and all three byte-wise sub/sbb/add/adc triples into word+byte pairs.
;
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A = quotient
; Clobbers: AX, BX, CX, DX, FLT_SA, FLT_ER, FLT_DB
; =============================================================================
flt_div:
        mov  bl, [FLT_B+0]
        or   bl, bl
        jnz  fdiv_bnz
        ; Float divide by zero raises the same ?2 error as integer divide
        ; by zero, via do_error -- see ERR_OV. Inlined: only ever reached
        ; from here, so the extra jmp to a separate label was overhead.
        mov  al, ERR_OV
        jmp  do_error            ; tail-call into uBASIC's error handler
fdiv_bnz:
        mov  al, [FLT_A+0]
        or   al, al
        jnz  fdiv_anz
        ret                     ; 0 / x = 0
fdiv_anz:
        push di                 ; DI = pointer into FLT_DB for the rest
                                 ; of this routine -- [di+n] is a disp8
                                 ; reference vs [FLT_DB+n]'s disp16, and
                                 ; FLT_DB is read/written ~10 times below,
                                 ; so this nets a real saving even after
                                 ; the push/pop. DI restored before
                                 ; return, so the documented clobber list
                                 ; is unaffected.

        ; Exponent: eA - eB + 0x80
        sub  al, bl
        add  al, 0x80
        mov  [FLT_ER], al

        ; Result sign = sign_A XOR sign_B (shared helper)
        call sign_xor
        mov  di, FLT_DB
        mov  dl, [FLT_B+1]
        or   dl, 0x80           ; dl = B_hi, implied-1 restored; kept
                                 ; live on purpose -- reused by the
                                 ; prescale compare below instead of a
                                 ; reload
        mov  [di+2], dl         ; B_hi at +2 (little-endian layout below)
        mov  ax, [FLT_B+2]      ; al=B_mid, ah=B_lo (raw word load)
        xchg al, ah              ; al=B_lo, ah=B_mid
        mov  [di+0], ax         ; DB+0=B_lo, DB+1=B_mid -- as a 16-bit
                                 ; word this equals B_mid*256+B_lo, the
                                 ; same packing BX already uses for A's
                                 ; mid:lo bytes below

        ; Load A mantissa: CH = A_hi (8-bit), BX = A_bytes2:3 (16-bit)
        mov  bx, [FLT_A+2]
        xchg bh, bl              ; BX = A mid:lo
        mov  ch, [FLT_A+1]
        or   ch, 0x80           ; dead 'and ch,0x7F' removed

        ; Pre-scale: if A >= B shift right 1, inc exponent
        cmp  ch, dl             ; DL still holds B_hi; no reload
        jb   fdiv_prescaled
        ja   fdiv_prescale
        cmp  bx, [di]           ; single word compare
        jb   fdiv_prescaled
fdiv_prescale:
        shr  ch, 1
        rcr  bx, 1
        inc  byte [FLT_ER]
fdiv_prescaled:

        ; Initialise 32-bit quotient accumulator DX:AX
        xor  ax, ax
        cwd                      ; AX=0 (bit15=0) -> cwd sign-extends to
                                  ; DX=0, cheaper than a second xor
        mov  cl, 32

fdiv_loop:
        ; Shift 32-bit quotient left 1: AX(lo) -> DX(hi)
        shl  ax, 1
        rcl  dx, 1

        ; Shift 24-bit remainder left 1: BX(lo) -> CH(hi, genuine 8-bit)
        shl  bl, 1
        rcl  bh, 1
        rcl  ch, 1
        jnc  fdiv_no_ov          ; no 25th-bit overflow -> speculative path

        ; 25th-bit overflow: remainder unconditionally >= B, commit
        sub  bx, [di]
        sbb  ch, [di+2]
fdiv_set_bit:
        inc  ax                 ; sets AX's LSB (AX's low bit is always 0
                                 ; here, just shifted in by 'shl ax,1' above)
        jmp  short fdiv_next

fdiv_no_ov:
        ; Speculative subtract: CH:BX -= B. If the final byte borrows,
        ; the remainder was < B; undo by adding B back. Otherwise commit
        ; and set the bit.
        sub  bx, [di]
        sbb  ch, [di+2]
        jnc  fdiv_set_bit        ; no borrow -> commit (shared with above)
        add  bx, [di]            ; borrowed -> restore
        adc  ch, [di+2]

fdiv_next:
        dec  cl
        jnz  fdiv_loop

        ; 32-bit quotient = DX:AX (DH:DL:AH:AL from MSB to LSB).
        ; Top 24 bits (mantissa) = DH:DL:AH: bottom 8 bits (AL) = guard.
        ; norm_pack wants: CH=mant[23:16], DH=mant[15:8], DL=mant[7:0], AL=guard
        xchg dl, ah              ; dl=old AH (mant[7:0]); ah=old DL (stashed)
        mov  ch, dh              ; CH = old DH = mant[23:16]
        mov  dh, ah              ; DH = old DL (via the xchg stash above)
        ; AL already holds the guard byte (old AL) - leave unchanged
        mov  bh, [FLT_ER]
        pop  di
        jmp  norm_pack          ; backward jump - safe

; =============================================================================
; FLT_SQRT  FLT_A = sqrt(FLT_A), Newton-Raphson, 5 iterations
;
; Ported from the 65C02 uBASIC6502 float library's FLT_SQRT. Initial guess:
; halve the biased exponent byte and re-bias by +64. Valid here because
; this codebase's MBF4 format is excess-128 biased with mantissa in
; [0.5,1) -- cross-checked against DO_ATN_FUNC's own exponent-byte
; threshold (biased exponent 0x82 <=> value 2.0, which only holds under
; that convention), not just assumed from the source this was ported from.
; Iterates x_(n+1) = (S/x_n + x_n) / 2, halving via exponent decrement
; rather than a real divide-by-2 (same trick as the source).
;
; BUG FIXED from the ported source: the original 6502 draft's comment said
; "JSR COPY_A_TO_B ; Move x_n from T_X into FLT_B", but COPY_A_TO_B copies
; FLT_A (not T_X) into FLT_B -- and FLT_A had just been reloaded with S at
; that point, so as literally written it would have computed S/S=1 every
; iteration instead of S/x_n. A second comment further down ("Restore x_n
; into FLT_B from T_X") had no code under it at all -- never implemented in
; the source. Fixed here by copying x_n from T_X into FLT_B directly. Only
; one such copy is needed (not one per FLT_DIV/FLT_ADD call) because neither
; FLT_DIV nor FLT_ADD documents FLT_B as clobbered -- both leave it
; unchanged in memory, confirmed against their own header comments.
;
; BUG FIXED (v3.1, found v2.9): FLT_SQRT(0) -- reached whenever ASIN/ACOS
; hit exactly |x|=1, where 1-x^2 is exact zero -- produced garbage
; instead of 0. The exponent-halving seed step below doesn't understand
; MBF4's exact-zero sentinel (exponent byte 0x00 means "this is zero",
; not "a tiny normalized value"); blindly halving-and-rebiasing 0x00
; produced a nonzero "quasi-zero" seed (0x40), and 5 Newton-Raphson
; iterations on that converge to a large garbage value, not 0. FLT_ASIN
; then divided x by that quasi-zero -- small enough to blow up the
; result, but not exactly zero, so FLT_DIV's real divide-by-zero check
; never caught it. Confirmed via ASIN(0.99)/ASIN(0.999)/ASIN(0.9999),
; which all correctly trended toward PI/2 -- this was specifically the
; exact-zero case, not general imprecision near the boundary. Fixed
; with an explicit zero check before the seed step; FLT_A is already
; all-zero bytes in that case, so returning immediately is exact, not
; approximate.
;
; Domain: FLT_A assumed non-negative otherwise; no further error check
; (caller's responsibility, same as the ported source).
;
; Inputs  : FLT_A = S (value to take the square root of)
; Outputs : FLT_A = sqrt(S)
; Clobbers: AX, BX, CX, DX, DI, SQRT_S, SQRT_X
; =============================================================================
flt_sqrt:
        cmp  byte [FLT_A+0], 0
        je   fsqrt_zero          ; exact zero: sqrt(0)=0, FLT_A is
                                  ; already correct as-is
        push si
        mov  si, FLT_A
        mov  di, SQRT_S
        movsw
        movsw                     ; T_S = S (original input, preserved
                                   ; across all 5 iterations)

        shr  byte [FLT_A+0], 1    ; halve the biased exponent
        add  byte [FLT_A+0], 64   ; re-bias -> fast initial guess x_0

        mov  cx, 5                ; 5 iterations: plenty for a 24-bit mantissa
fsqrt_nr:
        mov  si, FLT_A
        mov  di, SQRT_X
        movsw
        movsw                     ; T_X = x_n (current guess, backed up)
        mov  si, SQRT_S
        mov  di, FLT_A
        movsw
        movsw                     ; FLT_A = S
        mov  si, SQRT_X
        mov  di, FLT_B
        movsw
        movsw                     ; FLT_B = x_n

        push cx
        call flt_div               ; FLT_A = S / x_n (FLT_B still x_n --
                                    ; neither FLT_DIV nor FLT_ADD clobbers it)
        call flt_add               ; FLT_A = (S / x_n) + x_n
        dec  byte [FLT_A+0]        ; /2 via exponent decrement
        pop  cx
        loop fsqrt_nr

        pop  si
fsqrt_zero:
        ret

; =============================================================================
; FLT_ATAN  FLT_A = atan(FLT_A), full domain, rational approximation
;
; Ported from the 65C02 uBASIC6502 float library's FLT_ATAN:
; atan(x) = x / (1 + 0.28086*x^2), a minimax rational approximation
; valid for |x|<=1 (max error ~0.005 rad in that range). This is a
; straight port with no bug to fix (unlike FLT_SQRT) -- the 6502
; source's FLT_ATAN was correct as written.
;
; Extended here to the full domain via the same reciprocal identity the
; CORDIC-based DO_ATN_FUNC this replaces already used: for |x|>=1.0,
; atan(x) = sign(x) * (PI/2 - atan(1/|x|)). This extension isn't just
; nice-to-have: FLT_ASIN (added alongside this, see change history)
; evaluates atan(x/sqrt(1-x^2)), whose argument grows without bound as
; |x| -> 1, so ATAN needs to stay accurate well past |x|=1 for ASIN/
; ACOS's accuracy near the domain edges.
;
; GOTCHA: the |x|>=1.0 test is a raw exponent-byte compare, not a call
; to FLT_CMP, for the same reason DO_ATN_FUNC's own |x|>=2.0 test was --
; FLT_ADD's larger-operand swap doesn't restore FLT_B's identity (see
; the RESIDUAL QUIRK note at the top of the file). Biased exponent 0x81
; <=> value 1.0 under this codebase's excess-128/[0.5,1)-mantissa MBF4
; convention (same convention DO_ATN_FUNC's own 0x82<=>2.0 test relies
; on).
;
; Inputs  : FLT_A = x
; Outputs : FLT_A = atan(x), radians, range (-PI/2, PI/2)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, ATN_FLAGS
; =============================================================================
flt_atan:
        push si
        mov  al, [FLT_A+1]
        and  al, 0x80              ; AL = original sign bit
        mov  [ATN_FLAGS], al
        call flt_abs                ; FLT_A = |x|
        cmp  byte [FLT_A+0], 0x81   ; exponent >= 0x81 <=> |x| >= 1.0
        jb   fatan_have_arg
        call flt_a_to_b             ; FLT_B = |x|
        mov  ax, 1
        call flt_from_int           ; FLT_A = 1.0
        call flt_div                ; FLT_A = 1.0 / |x|
        inc  byte [ATN_FLAGS]      ; set bit0 ("was range-reduced"); safe
                                    ; as INC (not OR 0x01) because bit0 is
                                    ; guaranteed clear here -- ATN_FLAGS was
                                    ; just set to the sign bit alone (0x00
                                    ; or 0x80) a few lines up
fatan_have_arg:
        call flt_a_push             ; protect the argument across FLT_MUL
        call flt_a_to_b             ; FLT_B = x
        call flt_mul                ; FLT_A = x^2
        mov  di, FLT_B
        mov  si, const_0_28086
        movsw
        movsw                      ; FLT_B = 0.28086
        call flt_mul                ; FLT_A = 0.28086 * x^2
        mov  ax, 1
        call flt_from_int_b         ; FLT_B = 1.0
        call flt_add                ; FLT_A = 1.0 + 0.28086*x^2  [denom]
        call flt_a_to_b             ; FLT_B = denom
        call flt_a_pop              ; FLT_A = x (or 1/|x|)
        call flt_div                ; FLT_A = x / denom  (rational approx)

        test byte [ATN_FLAGS], 0x01
        jz   fatan_sign
        call flt_a_to_b             ; FLT_B = atan(1/|x|)
        call load_half_pi_a         ; FLT_A = PI/2
        call flt_sub                ; FLT_A = PI/2 - atan(1/|x|)
fatan_sign:
        test byte [ATN_FLAGS], 0x80
        jz   fatan_done
        call flt_negate
fatan_done:
        pop  si
        ret

; =============================================================================
; FLT_ASIN  FLT_A = asin(FLT_A), radians, range (-PI/2, PI/2)
;
; Ported from the 65C02 uBASIC6502 float library's FLT_ASIN, unchanged
; logic -- no bug found here (unlike FLT_SQRT in v2.6):
; asin(x) = atan(x / sqrt(1-x^2)).
;
; Domain: |x|<=1. |x|>1 not checked -- same as FLT_SQRT and the ported
; source, out-of-domain input silently produces a meaningless result
; rather than an error (FLT_SQRT has no sign check, so 1-x^2<0 just
; gets sqrt'd as if positive). Matches this codebase's existing level
; of rigor for SIN/COS/ATN, none of which validate their domain either.
;
; BUG FIXED (v3.2, found via a real test program hitting it every run):
; |x|==1 exactly used to divide by zero (v3.1: clean ?2 error) or worse
; (v2.9: FLT_SQRT(0) garbage). asin(+-1) is a well-defined limit
; (+-PI/2) even though the general x/sqrt(1-x^2) formula can't reach it
; (1-x^2 is exact zero there). The v3.1 fix (making FLT_SQRT(0) exact
; instead of garbage) turned this into an honest error rather than
; silent garbage, which seemed sufficient at the time -- but a
; symmetric raster-sphere test program hit x==1 at its exact center
; pixel on every single run, not as a rare edge case, making "clean
; error" impractical in practice. Short-circuits here instead, before
; ever reaching FLT_SQRT/FLT_DIV.
;
; Inputs  : FLT_A = x
; Outputs : FLT_A = asin(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, ATN_FLAGS, SQRT_S,
;           SQRT_X (via FLT_SQRT/FLT_ATAN)
; NOTE: does NOT self-preserve SI (unlike FLT_SQRT/FLT_ATAN) -- its only
; callers (DO_ASIN_FUNC, FLT_ACOS) already wrap the whole call in their
; own push/pop si, so a second layer here is dead weight. If a future
; caller needs SI preserved across FLT_ASIN without wrapping it itself,
; add the wrap back.
; =============================================================================
; =============================================================================
; ONE_MINUS_MUL  FLT_A = 1.0 - (FLT_A * FLT_B)
; Factored out of FLT_ASIN and FLT_SIN's polynomial phase -- both had this
; exact 5-instruction sequence inline.
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A = 1.0 - (FLT_A * FLT_B)
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_A, FLT_B, FLT_SA, FLT_SB
;           (union of FLT_MUL's, FLT_FROM_INT's, and FLT_SUB's own clobbers)
; =============================================================================
one_minus_mul:
        call flt_mul
        call flt_a_to_b
        mov  ax, 1
        call flt_from_int
        jmp  flt_sub             ; tail-call: FLT_A = 1.0 - FLT_B

flt_asin:
        call flt_a_push           ; park x on the real stack
        call flt_a_to_b           ; FLT_B = x
        call one_minus_mul        ; FLT_A = 1.0 - x^2
        cmp  byte [FLT_A+0], 0     ; exact zero <=> |x|==1 boundary
        jne  fasin_general
        call flt_a_pop             ; FLT_A = original x (need its sign)
        mov  al, [FLT_A+1]
        and  al, 0x80              ; AL = sign of x
        push ax
        call load_half_pi_a        ; FLT_A = PI/2
        pop  ax
        or   al, al
        jz   fasin_boundary_done   ; x>=0: asin(1) = +PI/2
        xor  byte [FLT_A+1], 0x80  ; x<0:  asin(-1) = -PI/2
fasin_boundary_done:
        ret
fasin_general:
        call flt_sqrt               ; FLT_A = sqrt(1 - x^2)
        call flt_a_to_b           ; FLT_B = sqrt(1 - x^2)  [denominator]
        call flt_a_pop             ; FLT_A = x
        call flt_div                ; FLT_A = x / sqrt(1 - x^2)
        jmp  flt_atan                ; tail-call: FLT_A = atan(result) = asin(x)

; =============================================================================
; FLT_ACOS  FLT_A = acos(FLT_A), radians, range (0, PI)
;
; Ported from the 65C02 uBASIC6502 float library's FLT_ACOS, unchanged
; logic -- acos(x) = PI/2 - asin(x). Same domain caveat as FLT_ASIN.
;
; Inputs  : FLT_A = x
; Outputs : FLT_A = acos(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, ATN_FLAGS, SQRT_S,
;           SQRT_X (via FLT_ASIN)
; NOTE: does NOT self-preserve SI, same reasoning as FLT_ASIN -- its
; only caller (DO_ACOS_FUNC) already wraps the call.
; =============================================================================
flt_acos:
        call flt_asin              ; FLT_A = asin(x)
        call flt_a_to_b            ; FLT_B = asin(x)
        call load_half_pi_a        ; FLT_A = PI/2
        jmp  flt_sub                ; tail-call: FLT_A = PI/2 - asin(x)

; =============================================================================
; FLT_SIN  FLT_A = sin(FLT_A), radians, full domain
;
; Ported from a 65C02 trig library (2-term minimax polynomial, accurate
; on [0,PI/2] after range reduction mod 2*PI and quadrant folding).
; Straight port, no bug found -- traced the real-stack depth through
; every branch (FLT_A_PUSH/FLT_A_POP/FLT_B_POP's 4-byte float parks,
; interleaved with a 1-word sign-tracker push/pop) and it balances on
; every path, matching the source's own PHA/PLA + PUSH_FLT_A/POP_FLT_A
; interleaving. Replaces CORDIC (v3.0 -- see change history); the
; polynomial is plausibly *more* precise than CORDIC's ~14-bit
; fixed-point, not just smaller, though not independently re-derived
; here -- accuracy is only as good as the source's own claim.
;
; GOTCHA: the sign tracker lives in AX on the real stack (AL=0x00 or
; 0x80, AH don't-care) rather than a byte push, since 8086 has no
; byte-sized push/pop -- pushed once at entry, popped+flipped+re-pushed
; once (net stack-depth-neutral) if x lands past PI during folding, and
; popped for good at the very end.
;
; Domain: magnitude of x limited by FLT_INT's int16 saturation on
; x/(2*PI) -- see FLT_INT's own header for the ceiling. Not checked
; beyond that, matching this codebase's existing rigor for its other
; trig functions.
;
; Inputs  : FLT_A = x
; Outputs : FLT_A = sin(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B
; =============================================================================
flt_sin:
        mov  al, [FLT_A+1]
        and  al, 0x80              ; AL = sign tracker (0x00 or 0x80)
        push ax
        call flt_abs                ; FLT_A = |x|

        call flt_a_push             ; park |x|
        call flt_2pi_b              ; FLT_B = 2*PI
        call flt_div                 ; FLT_A = |x| / 2*PI
        call flt_int                  ; FLT_A = INT(|x| / 2*PI)
        call flt_2pi_b                 ; FLT_B = 2*PI
        call flt_mul                    ; FLT_A = INT(...) * 2*PI
        call flt_a_to_b                  ; FLT_B = INT(...) * 2*PI
        call flt_a_pop                    ; FLT_A = |x|
        call flt_sub                       ; FLT_A = |x| mod 2*PI

        call flt_pi_b                       ; FLT_B = PI
        call flt_sub                         ; FLT_A = x' - PI
        test byte [FLT_A+1], 0x80
        jz   fsin_gt_pi
        call flt_pi_b                         ; x' <= PI: restore x' by
        call flt_add                          ; adding PI back
        jmp  fsin_check_pi_2
fsin_gt_pi:
        pop  ax                                ; x' > PI: flip sign tracker
        xor  al, 0x80                          ; (net stack depth unchanged)
        push ax
fsin_check_pi_2:
        call flt_a_to_b                         ; FLT_B = x'
        call load_half_pi_a                      ; FLT_A = PI/2
        call flt_sub                              ; FLT_A = PI/2 - x'
        test byte [FLT_A+1], 0x80
        jz   fsin_le_pi_2
        call flt_pi_2_b                            ; x' > PI/2:
        call flt_add                                ; x' = PI/2 + (PI/2-x')
        jmp  fsin_eval_poly
fsin_le_pi_2:
        call flt_a_to_b                              ; x' <= PI/2:
        call load_half_pi_a                           ; restore
        call flt_sub                                   ; x' = PI/2-(PI/2-x')
fsin_eval_poly:
        ; sin(x') ~= x' * (1 - x'^2 * (0.16605 - 0.00761 * x'^2))
        call flt_a_push               ; park x'
        call flt_a_to_b
        call flt_mul                   ; FLT_A = x'^2
        call flt_a_push                 ; park x'^2

        mov  di, FLT_B
        mov  si, const_c2
        movsw
        movsw                            ; FLT_B = 0.00761
        call flt_mul                      ; FLT_A = 0.00761 * x'^2
        call flt_a_to_b
        mov  di, FLT_A
        mov  si, const_c1
        movsw
        movsw                              ; FLT_A = 0.16605
        call flt_sub                        ; FLT_A = 0.16605 - 0.00761*x'^2

        call flt_b_pop                       ; FLT_B = x'^2
        call one_minus_mul                    ; FLT_A = 1.0 - x'^2*(...)

        call flt_b_pop                           ; FLT_B = x'
        call flt_mul                              ; FLT_A = x' * (...)

        pop  ax                                    ; sign tracker
        xor  [FLT_A+1], al
        ret

; =============================================================================
; FLT_COS  FLT_A = cos(FLT_A) = sin(PI/2 - FLT_A)
; Ported from the same 65C02 trig library. Straight port, trivial.
; Inputs  : FLT_A = x
; Outputs : FLT_A = cos(x)
; Clobbers: same as FLT_SIN (tail-calls it)
; =============================================================================
flt_cos:
        call flt_a_to_b            ; FLT_B = x
        call load_half_pi_a        ; FLT_A = PI/2
        call flt_sub                ; FLT_A = PI/2 - x
        jmp  flt_sin                 ; tail-call: FLT_A = sin(PI/2 - x)

; =============================================================================
; FLT_TAN  FLT_A = tan(FLT_A) = sin(x) / cos(x)
;
; Ported from the same 65C02 trig library. The source parks cos(x) via a
; 6-byte manual shuffle into its own T_X scratch (presumably to conserve
; the 6502's tiny 256-byte hardware stack); on 8088 the real stack has
; much more headroom, so this reimplements the same idea with a plain
; scratch RAM slot (TAN_C) rather than replicating the shuffle -- cos(x)
; can't be parked via FLT_A_PUSH/the stack the way it works elsewhere in
; this file, because FLT_SIN (called next, to get sin(x)) clobbers FLT_B
; internally, and the stack-park mechanism here only has one slot (FLT_A)
; to round-trip through -- a second parked value needs real memory.
;
; Domain: not checked. tan(x) is undefined at x=+-PI/2, +-3PI/2, etc.,
; where cos(x)=0 -- FLT_DIV's own divide-by-zero check (?2) will catch
; the exact-zero case; near-zero cos(x) (x close to but not exactly the
; singularity) will just produce a very large but finite result, same
; level of rigor as this codebase's other trig functions.
;
; Inputs  : FLT_A = x
; Outputs : FLT_A = tan(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, TAN_C (and everything
;           FLT_SIN/FLT_COS/FLT_DIV clobber)
; =============================================================================
flt_tan:
        call flt_a_push            ; park x
        call flt_cos                 ; FLT_A = cos(x)
        mov  si, FLT_A
        mov  di, TAN_C
        movsw
        movsw                        ; TAN_C = cos(x) (safe: FLT_SIN never
                                      ; touches this scratch)
        call flt_a_pop               ; FLT_A = x
        call flt_sin                   ; FLT_A = sin(x)
        mov  si, TAN_C
        mov  di, FLT_B
        movsw
        movsw                        ; FLT_B = cos(x)
        jmp  flt_div                   ; tail-call: FLT_A = sin(x) / cos(x)

; =============================================================================
; FLT_PRINT  print FLT_A as decimal to terminal
;
; S22: FLT_A restored via pop [FLT_A+n] at end.
;
; Inputs  : FLT_A
; Outputs : (none); FLT_A unchanged (restored before return) EXCEPT when
;           input is exact zero (exponent byte = 0x00): that path tail-calls
;           output directly and never reaches the pop-restore at fp_print_done.
;           FLT_A is all-zeros on that path anyway, so the value is correct;
;           the invariant is just not maintained via the stack mechanism.
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_SA, FLT_ER, FLT_DE
; =============================================================================
flt_print:
        mov  al, [FLT_A+0]
        or   al, al
        jnz  fp_notzero
        mov  al, '0'
        jmp  output

fp_notzero:
        test byte [FLT_A+1], 0x80
        jz   fp_notneg
        mov  al, '-'
        call output
        call flt_abs             ; dropped the push ax/pop ax that used
                                  ; to bracket this -- the AL value it
                                  ; preserved was never read; the very
                                  ; next line reloads AL from [FLT_A+0]
                                  ; regardless. flt_abs itself touches no
                                  ; registers (see its header).
fp_notneg:
        ; Decimal exponent estimate: de = (exp - 0x80) * 77 >> 8
        ; 8-bit IMUL (AX = AL*CL) needs no preceding CBW and no 16-bit
        ; immediate load for CX -- AL is already the signed byte being
        ; scaled, so AL*77 (8x8->16) equals the old CBW-then-16x16
        ; result exactly, just cheaper to set up.
        mov  al, [FLT_A+0]
        sub  al, 0x80
        mov  cl, 77
        imul cl
        mov  [FLT_DE], ah

        ; S22: save FLT_A on stack
        call flt_a_push

        ; Scale to [1,10)
        mov  al, [FLT_DE]
        cbw
        xchg cx, ax
        or   cx, cx
        jz   fp_scale_done
        jl   fp_scale_up
fp_scale_down:
        push cx
        call div_by_ten
        pop  cx
        loop fp_scale_down
        jmp  fp_scale_done
fp_scale_up:
        push cx
        call mul_by_ten
        pop  cx
        inc  cx
        jnz  fp_scale_up

fp_scale_done:
        ; Verify range [1,10); adjust if off by 1
        call flt_ten_b
        call flt_cmp
        cmp  ax, -1
        je   fp_chk_lo
        call div_by_ten
        inc  byte [FLT_DE]
        jmp  fp_extract
fp_chk_lo:
        mov  ax, 1
        call flt_from_int_b
        call flt_cmp
        cmp  ax, -1
        jne  fp_extract
        call mul_by_ten
        dec  byte [FLT_DE]

fp_extract:
        ; Extract 7 digits into IBUF.
        ;
        ; DI's own position (relative to IBUF) already tells us how many
        ; digits remain, since it advances by exactly one per iteration --
        ; folding the counter into the pointer means only DI needs
        ; protecting across the calls that clobber it (flt_from_int_b,
        ; flt_sub, mul_by_ten); flt_to_int itself never touches DI, so the
        ; first call of each iteration needs no save/restore at all.
        mov  al, [FLT_DE]
        cbw
        push ax                 ; save decimal exponent for print phase
        mov  di, IBUF
fp_dig_lp:
        call flt_to_int
        push ax                 ; save digit (di not pushed -- not needed here)
        push di
        call flt_from_int_b
        call flt_sub
        test byte [FLT_A+1], 0x80
        jz   fp_no_clamp
        call flt_zero
fp_no_clamp:
        pop  di
        pop  ax
        add  al, '0'
        stosb                    ; writes AL to [DI], DI++

        push di
        call mul_by_ten          ; safe to run on the 7th iteration too --
                                  ; the result is never used (FLT_A is
                                  ; restored from FLT_A_PUSH at
                                  ; fp_print_done regardless), and
                                  ; flt_mul/mul_by_ten never traps on
                                  ; overflow (only flt_div's div-by-zero
                                  ; does)
        pop  di
        cmp  di, IBUF+7
        jne  fp_dig_lp

fp_dig_done:
        ; Round: digit[6] >= '5' -> round up. Either way, digit[6] itself
        ; is never displayed afterward (see fp_blankseven) -- it's the
        ; least reliable of the 7 extracted digits (most mul_by_ten
        ; calls -> most compounded mantissa-rounding error), so it's only
        ; ever consulted for this round-up decision.
        mov  al, [IBUF+6]
        cmp  al, '5'
        jb   fp_blankseven
        mov  bx, IBUF+5
fp_round_lp:
        inc  byte [bx]
        cmp  byte [bx], '9'+1
        jb   fp_blankseven
        mov  byte [bx], '0'
        dec  bx
        cmp  bx, IBUF-1
        jg   fp_round_lp
        mov  byte [IBUF], '1'
        pop  ax
        inc  ax
        push ax
fp_blankseven:
        mov  byte [IBUF+6], '0'

fp_strip_setup:
        ; fp_strip used to walk back from IBUF+7 dropping every trailing
        ; '0', stopping only at IBUF itself. That's correct for zeros in
        ; the FRACTIONAL part (e.g. 2.500000 -> "2.5"), but wrong for
        ; zeros that are part of the INTEGER part -- a value like
        ; 1000000 (digits "1000000", all 6 trailing zeros significant)
        ; was being stripped all the way down to a single "1". Fixed by
        ; popping the decimal-exponent-in-flight HERE (before stripping,
        ; not after) and using IBUF + max(0, digits_before_point) as the
        ; floor instead of plain IBUF, so stripping can never cross into
        ; the integer part. BX is kept holding that same value afterward
        ; (no need to push/pop it again for the print-phase setup below).
        ;
        ; NOTE: both the rounding path (digit[6]>='5') and the no-rounding
        ; path (digit[6]<'5', via the jb above) must land HERE -- this is
        ; the only place that pops the decimal-exponent value pushed back
        ; in fp_extract, so the jb above targets this label, not fp_strip
        ; directly, or the pop would be skipped on the no-rounding path
        ; (leaving BX garbage and the stack permanently unbalanced).
        pop  bx                  ; bx = de (decimal exponent), NOT yet +1
        mov  dx, IBUF
        or   bx, bx
        js   fp_strip_floor_done ; de<0 means digits_before_point<=0: value < 1.0,
                                  ; whole digit string is fractional, unrestricted
                                  ; stripping down to IBUF is correct
        add  dx, bx               ; dx = IBUF+de = IBUF+digits_before_point-1 directly
                                  ; -- no separate -1 step needed
fp_strip_floor_done:
        inc  bx                  ; bx = de+1 = digits before decimal point (the
                                  ; print-phase value; dx above already has its
                                  ; own -1 baked in via using de, not bx, as the base)

fp_strip:
        dec  di
        cmp  byte [di], '0'
        jne  fp_strip_done
        cmp  di, dx              ; stop at (or below) the integer/fraction boundary
        jle  fp_strip_done
        jmp  fp_strip
fp_strip_done:
        inc  di

        ; Print with decimal point (BX already holds digits-before-point)
        mov  si, IBUF
        or   bx, bx
        jg   fp_print_lp
        mov  al, '0'
        call output
        mov  al, '.'
        call output
        neg  bx
fp_lead_zero:
        or   bx, bx
        jle  fp_print_lp
        mov  al, '0'
        call output
        dec  bx
        jmp  fp_lead_zero
fp_print_lp:
        ; v0.13: when the buffer is exhausted (si==di) but bx still > 0,
        ; the value needs more integer-part width than the 7-digit buffer
        ; holds (e.g. 900000000, 9 digits) -- pad with '0' instead of
        ; exiting. bx<=0 here still correctly means "done" (covers both
        ; the exact-end case and the already-past-the-point case, since a
        ; fractional tail can legitimately drive bx negative).
        cmp  si, di
        jb   fp_print_have_digit
        or   bx, bx
        jle  fp_print_done
        mov  al, '0'
        jmp  fp_print_emit
fp_print_have_digit:
        mov  al, [si]
        inc  si
fp_print_emit:
        call output
        dec  bx
        jnz  fp_print_lp
        cmp  si, di
        jnb  fp_print_done
        mov  al, '.'
        call output
        jmp  fp_print_lp
fp_print_done:
        ; S22: restore FLT_A directly from stack
        call flt_a_pop
        ret

; =============================================================================
; FLT_PARSE  decimal string at [SI] -> FLT_A
;
; Recursive rewrite: the integer part is accumulated iteratively (digit-by-
; digit, multiply-by-10-then-add, same as before) but the FRACTIONAL part is
; built by recursing to the end of the fractional digits first, then
; unwinding: each frame computes (digit + result_of_rest) / 10. This gets
; the same value as the old "count decimal places, then divide by 10^n at
; the end" approach, but needs no place-counter and naturally stops at
; exactly the typed digits with no separate scaling pass. Recursion depth
; is bounded by the number of *typed* fractional-digit characters (a BASIC
; program's source text), not by the parsed value's magnitude, so it is
; safe in a way a similar trick for the integer part would not be.
;
; GOTCHA: the input sign is stashed in FLT_DE, not FLT_SA. FLT_SA is
; flt_add's own "sign of larger operand" scratch and gets clobbered by
; every flt_add call made while accumulating digits -- stashing the
; sign there would lose it before it could be applied. FLT_DE is safe
; for the whole parse (flt_print's scratch, untouched by flt_add/mul/div,
; and flt_parse never calls flt_print).
;
; GOTCHA: FPAR_CHK_DOT's decimal-point path and FPAR_SIGN_APPLY's
; plain-integer path are NOT symmetric on purpose. PARSE_FRAC's own base
; case (PFRAC_END) already un-consumes the digit string's terminator, so
; the decimal-point path must NOT also do FPAR_SIGN_APPLY's 'dec si' --
; doing both double-decrements SI. See v2.1 change history.
;
; Inputs  : SI -> null/CR-terminated decimal string
; Outputs : FLT_A = parsed value; SI advanced past last char consumed
; Clobbers: AX, BX, CX, DX, DI, FLT_A, FLT_B, FLT_SA, FLT_DE
; =============================================================================
flt_parse:
        call flt_zero
        mov  byte [FLT_DE], 0

fpar_skip:
        lodsb
        cmp  al, ' '
        je   fpar_skip
        cmp  al, '-'
        jne  fpar_plus
        mov  byte [FLT_DE], 0x80
        jmp  fpar_int_lp
fpar_plus:
        cmp  al, '+'
        je   fpar_int_lp
        dec  si                 ; not a sign char -- un-consume it

fpar_int_lp:
        lodsb
        sub  al, '0'
        cmp  al, 9
        ja   fpar_chk_dot       ; not a digit -> check for a decimal point
        cbw                     ; AX = digit (sign-extend; digit is 0..9, AH stays 0)
        push ax                 ; save digit across the *10 step
        call mul_by_ten         ; FLT_A = FLT_A * 10
        pop  ax
        call flt_from_int_b     ; FLT_B = digit
        call flt_add            ; FLT_A = FLT_A + digit
        jmp  fpar_int_lp

fpar_chk_dot:
        cmp  al, '.' - '0'      ; AL already has '0' subtracted; '.'-'0' = -2
        jne  fpar_sign_apply
        call flt_a_push          ; stash integer part while FLT_A is reused below
        call parse_frac         ; FLT_A = value of the fractional digits (0.xxx)
        call flt_a_to_b         ; FLT_B = fraction
        call flt_a_pop           ; restore integer part into FLT_A
        call flt_add            ; FLT_A = integer + fraction
        ; GOTCHA: no 'dec si' here -- parse_frac's own base case already
        ; did it. See the GOTCHA note in this routine's header.
        cmp  byte [FLT_DE], 0
        je   fpar_done
        jmp  flt_negate          ; tail-call (backward jump - safe)

fpar_sign_apply:
        dec  si                 ; leave SI pointing at the char that ended the number
        cmp  byte [FLT_DE], 0
        je   fpar_done
        jmp  flt_negate         ; tail-call (backward jump - safe)
fpar_done:
        ret

; -----------------------------------------------------------------------------
; PARSE_FRAC  recursive helper: evaluate the fractional digit string at [SI]
; Reaches the end of the digits first (recursing in), then builds the value
; from the last digit backward as it unwinds: result = (digit + rest) / 10.
; Inputs  : SI -> first fractional digit
; Outputs : FLT_A = value of the fractional digits read (in [0,1)); SI left
;           pointing AT the terminating non-digit char (pfrac_end's own
;           'dec si' un-consumes it) -- NOT one-past it. v2.0 fix: the
;           comment here previously claimed this "mirrors flt_parse's own
;           SI convention so the caller's later dec si lines up", but that
;           was incorrect -- flt_parse's decimal-point call site no longer
;           applies a second dec si after calling this (see fpar_chk_dot).
; Clobbers: AX, BX, CX, DX, DI, FLT_A, FLT_B
; -----------------------------------------------------------------------------
parse_frac:
        lodsb
        sub  al, '0'
        cmp  al, 9
        ja   pfrac_end          ; not a digit -> base case, bottom of the fraction
        cbw
        push ax                 ; save this digit across the recursive call
        call parse_frac         ; FLT_A = value of all digits after this one
        pop  ax
        call flt_from_int_b     ; FLT_B = this digit
        call flt_add            ; FLT_A = digit + rest
        jmp  div_by_ten         ; tail-call: FLT_A = (digit + rest) / 10
pfrac_end:
        dec  si                 ; un-consume the non-digit terminator
        jmp  flt_zero           ; base case: nothing left -> 0.0

; =============================================================================
; DO_USR_FUNC  USR(addr) — call arbitrary machine-code address
; v2.0: argument truncated to int16 (call address); the called routine's
; own contract is unchanged (still receives nothing in particular, still
; returns its result in AX as plain int16 machine-code convention) -- only
; the BASIC-side argument and return value are now float at the boundary.
; Inputs  : FLT_A = call address (from eat_paren_expr)
; Outputs : FLT_A = float(AX), where AX = return value from called routine
; Clobbers: AX, whatever the called routine clobbers
; =============================================================================
do_usr_func:
        call flt_to_int         ; AX = int16(FLT_A) = call address
        call ax       ; CALL (not JMP) so we return here after
        jmp  flt_from_int        ; tail-call: FLT_A = float(AX)

; =============================================================================
; Shared float constants (MBF4). half_pi_const used by FLT_ATAN, FLT_ASIN
; (via LOAD_HALF_PI_A), FLT_ACOS, FLT_SIN, FLT_COS (via FLT_PI_2_B/
; FLT_PI_B/FLT_2PI_B). const_0_28086 is FLT_ATAN's rational-approximation
; coefficient; const_c1/const_c2 are FLT_SIN's polynomial coefficients.
; All three non-trivial constants were generated and verified via this
; ROM's own flt_parse (A=<value>, then PEEK'd back out of VARS), not
; hand-derived -- see v2.8/v3.0 change history.
; NOTE (v3.0): this block used to sit between CORDIC_ATAN_TAB and
; CORDIC_ROTATE_FX; relocated here, unchanged, when CORDIC was removed
; -- these four are the only things from that stretch of the file that
; were still needed.
; =============================================================================
half_pi_const: db 0x81, 0x49, 0x0F, 0xDB  ; PI/2 as MBF4
const_0_28086: db 0x7F, 0x0F, 0xCC, 0xE2  ; 0.28086 as MBF4
const_c1:      db 0x7E, 0x2A, 0x09, 0x02  ; 0.16605 as MBF4
const_c2:      db 0x79, 0x79, 0x5D, 0x4D  ; 0.00761 as MBF4

; =============================================================================
; DO_SIN_FUNC  SIN(x)          DO_COS_FUNC  COS(x)
; v3.0: thin wrappers around FLT_SIN/FLT_COS's minimax-polynomial
; implementation -- replaces the CORDIC engine entirely (CORDIC_ROTATE_FX,
; CORDIC_REDUCE, CORDIC_PICK, FLT_TO_FX14, FX14_TO_FLT, and the old
; merged TRIG_COMMON dispatch, ~360 bytes total including CORDIC_ATAN_TAB
; and FX14_SCALE). See v3.0 change history for the full accounting.
; Inputs  : FLT_A = x (radians, from eat_paren_expr)
; Outputs : FLT_A = sin(x) / cos(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B
; =============================================================================
do_sin_func:
        push si
        call flt_sin
        pop  si
        ret

do_cos_func:
        push si
        call flt_cos
        pop  si
        ret

; =============================================================================
; DO_ATN_FUNC  ATN(x) -> arctangent in radians, range (-PI/2, PI/2).
; v2.8: now a thin wrapper around FLT_ATAN's rational approximation
; (see its own header) -- was CORDIC vectoring-mode before. SI handling
; kept identical to the CORDIC version even though FLT_ATAN's own
; internal push/pop si already protects it, to stay consistent with
; DO_SIN_FUNC/DO_COS_FUNC's pattern and because this is the E2_FUNC_CALL
; tail-jump entry point, not just an internal helper.
; Inputs  : FLT_A = x (from eat_paren_expr)
; Outputs : FLT_A = atan(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, ATN_FLAGS
; =============================================================================
do_atn_func:
        push si
        call flt_atan
        pop  si
        ret

; =============================================================================
; DO_ASIN_FUNC  ASIN(x) -> arcsine in radians, range (-PI/2, PI/2).
; Thin wrapper around FLT_ASIN, same pattern as DO_ATN_FUNC.
; Inputs  : FLT_A = x (from eat_paren_expr)
; Outputs : FLT_A = asin(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, ATN_FLAGS, SQRT_S,
;           SQRT_X
; =============================================================================
do_asin_func:
        push si
        call flt_asin
        pop  si
        ret

; =============================================================================
; DO_ACOS_FUNC  ACOS(x) -> arccosine in radians, range (0, PI).
; Thin wrapper around FLT_ACOS, same pattern as DO_ATN_FUNC.
; Inputs  : FLT_A = x (from eat_paren_expr)
; Outputs : FLT_A = acos(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, ATN_FLAGS, SQRT_S,
;           SQRT_X
; =============================================================================
do_acos_func:
        push si
        call flt_acos
        pop  si
        ret

; =============================================================================
; DO_SQRT_FUNC  SQRT(x) -> square root.
; v3.1: FLT_SQRT existed since v2.6 as ASIN's internal dependency but was
; never exposed as its own BASIC keyword until now. Thin wrapper, same
; pattern as DO_ATN_FUNC.
; Inputs  : FLT_A = x (from eat_paren_expr)
; Outputs : FLT_A = sqrt(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, SQRT_S, SQRT_X
; =============================================================================
do_sqrt_func:
        push si
        call flt_sqrt
        pop  si
        ret

; =============================================================================
; DO_TAN_FUNC  TAN(x) -> tangent.
; Thin wrapper around FLT_TAN, same pattern as DO_ATN_FUNC.
; Inputs  : FLT_A = x (from eat_paren_expr)
; Outputs : FLT_A = tan(x)
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_A, FLT_B, TAN_C
; =============================================================================
do_tan_func:
        push si
        call flt_tan
        pop  si
        ret

ROM_END:

; =============================================================================
; RESET VECTOR  at 0xFFF0 - ROM version
; 8086 resets to CS=0xFFFF IP=0x0000 -> phys 0xFFFF0.
; =============================================================================
%ifndef __YASM_MAJOR__
        org 0xFFF0
        cld
%endif

reset_vec:
        ; Configure 8755 Port A: bit1=RX(input), all others output; TX idles high
        mov  al, 0xFD
        out  DDR_A, al
        mov  al, TX
        out  PORT_A, al

%ifdef __YASM_MAJOR__
        jmp  start	; same segment
%else
        ; FAR JMP to CS=0xF000 IP=0x0000  (opcode: EA 00 00 00 F0)
        db   0xEA
        dw   0x0000             ; IP
        dw   0xF000             ; CS
%endif

        times 4096-($-start) db 0xFF    ; pad to exactly 4 KB
