; =============================================================================
; miniBASIC 8088  v2.2  (was: uBASIC 8088 v1.7.5)
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Tiny-BASIC-derived interpreter for the 8088/8086 with MBF4 32-bit
; floating-point support, CORDIC SIN/COS, and a float-driven showcase
; program. Base architecture, hardware/memory map, line-store format,
; statement/operator set, identical to uBASIC8088.asm v1.7.5
;
; WHAT'S DIFFERENT FROM v1.7.5:
;   - All variables and expression results are 32-bit MBF4 floats, not
;     16-bit ints (bitwise/relational ops and PEEK/POKE/IN/OUT/TAB/CHR$
;     arguments truncate to int16 at the point of use; RND(n)'s limit
;     likewise).
;   - New functions: SIN(x), COS(x) (fixed-point Q14 CORDIC internally;
;     ~14-bit precision, less than MBF4's native ~23-bit mantissa).
;   - ROM expanded  2 KB -> 4 KB (2732 EPROM); 
;   - SHOWCASE_DATA rewritten to exercise float arithmetic, SIN/COS+TAB
;     (sine wave), and a floating-point Mandelbrot.
;
; Credit: Oscar Toledo G. for bootBASIC inspiration and TinyASM 8086 assembler.
;         XTulator CPU core by Mike Chambers.
;
; ---------------------------------------------------------------------------
; LANGUAGE REFERENCE  (delta from v1.7.5 only -- see that file for the rest)
; ---------------------------------------------------------------------------
;
; Functions   : adds COS(x), SIN(x) to v1.7.5's ABS/IN/NOT/PEEK/RND/USR
; Variables   : A..Z, 32-bit MBF4 float (v1.7.5: signed 16-bit)
; Numbers     : 32-bit MBF4 float, ~6-7 significant decimal digits
;               (v1.7.5: signed 16-bit, -32768..32767)
;
; ---------------------------------------------------------------------------
; BUILD
; ---------------------------------------------------------------------------
;
; Assemble ROM image:
;     tinyasm -f bin miniBASIC8088.asm -o miniBASIC_rom.bin
;
; Assemble and run in the supplied simulator:
;     ./sim_rom miniBASIC8088.asm
;
; See the supplied Makefile for toolchain build details.
;
; ROM budget (4096 bytes)
;     Current size : 4011 bytes
;     Free space   : 85 bytes
;
; The ROM target is the limiting configuration for future features.
;
; =============================================================================
; CHANGE HISTORY
; =============================================================================
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
;   - Miscellaneous ROM optimisations.
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
                                         ;   that use -- do not share with
                                         ;   CORDIC_C (see below); they were
                                         ;   merged once and that was a bug.
CORDIC_X:       equ RAM_BASE + 0x10D    ; word    : CORDIC Q14 fixed-point X
CORDIC_Y:       equ RAM_BASE + 0x10F    ; word    : CORDIC Q14 fixed-point Y
CORDIC_Z:       equ RAM_BASE + 0x111    ; word    : CORDIC Q14 fixed-point Z (angle)
CORDIC_T:       equ RAM_BASE + 0x113    ; word    : CORDIC shifted-X scratch
                                         ;   (SIN/COS, see CORDIC section)
CORDIC_C:       equ RAM_BASE + 0x115    ; 4 bytes : CORDIC_REDUCE's own angle/t
                                         ;   stash. GOTCHA: must stay separate
                                         ;   from FLT_C -- see FLT_C's comment
                                         ;   above and the v2.1 change history
                                         ;   for why merging them is a bug.
RUNNING:        equ RAM_BASE + 0x119    ; byte    : 0=immediate mode, 1=running
PROG_END:       equ RAM_BASE + 0x11A    ; word    : one past last program byte
PROGRAM:        equ RAM_BASE + 0x11C    ; program store start
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
        db 0x0A,0x00, 0x87, "miniBASIC 8088 v2.1 showcase", 0x0D ; 10  REM
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
        ; ── Sine wave: CORDIC SIN(x) driving TAB(n) ─────────────────────────────
        db 0xDC,0x00, 0x80, 0x22, "--- SINE WAVE (SIN+TAB) ---", 0x22, 0x0D ; 220 PRINT
        db 0xE6,0x00, 0x8F, "I=0", 0x94, "24", 0x0D ; 230 FOR I=0 TO(0x94) 24
        db 0xF0,0x00, "X=I*6.2832/24", 0x0D        ; 240 X=I*6.2832/24
        db 0xFA,0x00, 0x80, "TAB(20+19*SIN(X));", 0x22, "*", 0x22, 0x0D ; 250 PRINT TAB(...);"*"
        db 0x04,0x01, 0x90, "I", 0x0D              ; 260 NEXT I
        db 0x0E,0x01, 0x80, 0x22, 0x22, 0x0D       ; 270 PRINT ""
        ; ── Mandelbrot: native MBF4 float, no fixed-point scaling needed ─────
        db 0x18,0x01, 0x80, 0x22, "--- MANDELBROT (FLOAT) ---", 0x22, 0x0D ; 280 PRINT
        db 0x22,0x01, 0x8F, "I=-1", 0x94, "1 ", 0x95, "0.18", 0x0D ; 290 FOR I=-1 TO(0x94) 1 STEP(0x95) 0.18
        db 0x2C,0x01, 0x8F, "C=-2", 0x94, "0.5 ", 0x95, "0.045", 0x0D ; 300 FOR C=-2 TO(0x94) 0.5 STEP(0x95) 0.045
        db 0x36,0x01, "A=C:B=I:E=0", 0x0D          ; 310
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
        call stmt_line          ; no line number: execute immediately
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
; STMT_LINE  execute ':'-separated statements from SI
; Inputs  : SI -> statement text (tokenised or raw)
; Outputs : (none)
; Clobbers: AX, BX, CX, DX, SI, DI (via stmt)
; =============================================================================
stmt_line:
        call stmt
        call spaces
        cmp  byte [si], ':'
        jne  sl_ret
        inc  si                 ; consume ':'
        jmp  stmt_line

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
; PEEK_LINE  test whether SI is at end-of-statement (CR or ':')
; Inputs  : SI -> current position
; Outputs : ZF=1 at CR or ':', ZF=0 otherwise
; Clobbers: (none)
; =============================================================================
peek_line:
        call spaces
        cmp  byte [si], ':'
        je   sl_ret
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
; =============================================================================
var_store:
        pop  di
        push si
        mov  si, FLT_A
        cld
        movsw
        movsw
        pop  si
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
        call expr_bitwise       ; FLT_A = left operand (full float precision)
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
        push word [FLT_A+2]
        push word [FLT_A+0]

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
        call expr_bitwise       ; FLT_A = right operand (float)
        call flt_a_to_b         ; FLT_B = RHS
        pop  dx                 ; DL = operator bitmask
        pop  word [FLT_A+0]     ; restore LHS float into FLT_A
        pop  word [FLT_A+2]
        call flt_cmp            ; AX = -1 (LT), 0 (EQ), +1 (GT)
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
; BITWISE_AND / BITWISE_OR / BITWISE_XOR
; Inputs  : AX = left operand, CX = right operand  (int16; truncated from
;           float at expr_bitwise's boundary, see below)
; Outputs : AX = result
; Clobbers: (none beyond AX)
; =============================================================================
bitwise_and:
        and  ax, cx
        ret

bitwise_or:
        or   ax, cx
        ret

bitwise_xor:
        xor  ax, cx
        ret

; =============================================================================
; EXPR_BITWISE  bitwise level (& | ^), lowest precedence among binary ops.
; v2.0: ALWAYS returns float in FLT_A, left  untouched when no & | ^
; operator follows (thecommon case) -- only when a
; bitwise operator IS present is  this truncate to int16, run the
; integer op, and promote the result back to float before returning.
; Inputs  : SI -> expression text
; Outputs : FLT_A = result
; Clobbers: AX, BX, CX, DX, SI, FLT_A, FLT_B, FLT_C
; =============================================================================
expr_bitwise:
        call expr_add            ; FLT_A = left operand (full float precision)
        call spaces
        mov  dl, [si]
        cmp  dl, '&'
        je   .has_op
        cmp  dl, '|'
        je   .has_op
        cmp  dl, '^'
        je   .has_op
        ret                      ; no bitwise operator: FLT_A already
                                  ; holds the correct (float) result
.has_op:
        call flt_to_int          ; AX = int16(FLT_A) = left operand
.lp:
        call spaces
        mov  bx, tab_bitwise
.search:
        cmp  byte [bx], 0
        je   .done
        cmp  [bx], dl
        je   .found
        add  bx, 3
        jmp  .search
.found:
        inc  si                  ; consume operator char
        push ax                  ; save running int16 LHS
        push word [bx+1]         ; save handler address
        call expr_add            ; FLT_A = next operand (float)
        call flt_to_int          ; AX = int16(FLT_A)
        xchg cx, ax              ; CX = RHS
        pop  bx                  ; BX = handler
        pop  ax                  ; AX = LHS
        call bx                  ; AX = AX op CX
        call spaces
        mov  dl, [si]            ; peek next operator char (may be none)
        cmp  dl, '&'
        je   .lp
        cmp  dl, '|'
        je   .lp
        cmp  dl, '^'
        je   .lp
.done:
        jmp  flt_from_int        ; tail-call: FLT_A = float(AX)

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
.search:
        cmp  byte [bx], 0
        je   .done
        cmp  [bx], dl
        je   .found
        add  bx, 3              ; next entry: char(1) + handler_ptr(2)
        jmp  .search
.found:
        inc  si                 ; consume operator char
        push word [FLT_A+0]     ; park LHS on the real stack -- see the
        push word [FLT_A+2]     ; GOTCHA note above for why
        mov  di, [bp]           ; DI = next-level func (BP still valid:
                                 ; nothing between the top of .lp and here
                                 ; touches it)
        push word [bx+1]        ; save handler address
        call di                 ; get RHS -> FLT_A
        call flt_a_to_b         ; FLT_B = RHS
        pop  bx                 ; BX = handler
        pop  word [FLT_A+2]     ; restore LHS from the real stack
        pop  word [FLT_A+0]
        call bx                 ; FLT_A = FLT_A op FLT_B
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
        je   e2_par
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
        push word [FLT_A+0]
        push word [FLT_A+2]
        mov  al, ')'
        call expect
        pop  word [FLT_A+2]
        pop  word [FLT_A+0]
        ret

; =============================================================================
; DO_ABS_FUNC  ABS(n) -> absolute value.  v2.0: float-native (was int16).
; Inputs  : FLT_A = value (from eat_paren_expr)
; Outputs : FLT_A = |value|
; Clobbers: AX
; =============================================================================
do_abs_func:
        jmp  flt_abs            ; tail-call

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
; DO_NOT_FUNC  NOT(n) -> bitwise complement
; v2.0: argument truncated to int16, result promoted back to float.
; Inputs  : FLT_A = value (from eat_paren_expr)
; Outputs : FLT_A = ~int16(value), as a float
; Clobbers: AX, FLT_A
; =============================================================================
do_not_func:
        call flt_to_int         ; AX = int16(FLT_A)
        not  ax
        jmp  flt_from_int        ; tail-call: FLT_A = float(AX)

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
        jmp  flt_from_int        ; tail-call: FLT_A = float(AX)

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
        cld
        movsw
        movsw
        pop  si
        ret

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
        jnb  ipl_lp
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
        push cx
        call deline             ; delete existing line
        pop  cx
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
do_goto:
        call expr
        call flt_to_int          ; AX = int16(FLT_A) = target line#
        call find_line
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
        call stmt_line
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
        call expr               ; FLT_A = target line#
        call flt_to_int          ; AX = int16(FLT_A)
        call find_line          ; DI -> line >= AX
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
        cld
        movsw
        movsw                   ; initialise loop variable (4 bytes)
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
        cld
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
        mov  bx, cx
        add  bx, bx             ; * 2  no `shl rX, n` on 8086
        add  bx, bx             ; * 4
        mov  ax, bx             ; AX = depth * 4
        add  bx, ax
        add  bx, ax             ; BX = depth*4 + depth*4 + depth*4 = depth*12
        add  bx, FOR_STK
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
        call kw_match
        ret
etk_match:
        inc  si
        clc
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
        cld
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
        push di                 ; &var
        push bx                 ; &frame
        mov  si, di
        mov  di, FLT_A
        movsw
        movsw                   ; FLT_A = var
        pop  bx
        push bx
        mov  si, bx
        add  si, 6              ; &frame.step
        mov  di, FLT_B
        movsw
        movsw                   ; FLT_B = step
        call flt_add            ; FLT_A = var + step
        pop  bx
        pop  di                 ; &var
        mov  si, FLT_A
        movsw
        movsw                   ; var = FLT_A (write back)

        ; Exit test: positive step -> exit when var > limit
        ;            negative step -> exit when var < limit
        ; flt_cmp(var, limit): AX = -1/0/1 (var<limit / == / var>limit)
        mov  si, bx
        add  si, 2              ; &frame.limit
        mov  di, FLT_B
        movsw
        movsw                   ; FLT_B = limit  (FLT_A already = var)
        push bx                 ; flt_cmp clobbers BX -- save frame ptr
        call flt_cmp            ; AX = sign(var - limit)
        pop  bx                 ; restore frame ptr
        pop  cx                 ; restore frame index (see GOTCHA above)
        push ax                 ; save comparison result
        mov  al, [bx+7]         ; step's sign byte: frame.step is at
                                 ; bx+6..bx+9, MBF4 byte+1 holds the sign
                                 ; bit (bit 7), so bx+6+1 = bx+7
        pop  dx                 ; DX(low byte) = comparison result
        test al, 0x80
        jnz  dn_neg_step
        ; positive step: exit when var > limit  (cmp result == 1)
        cmp  dl, 1
        je   dn_done
        jmp  dn_loop
dn_neg_step:
        ; negative step: exit when var < limit  (cmp result == -1/0xFF)
        cmp  dl, 0xFF
        je   dn_done
dn_loop:
        mov  ax, [bx+10]
        mov  [RUN_NEXT], ax     ; jump back to top of loop
        ret
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
str_banner: db "miniBASIC 8088 v2.2"
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

tab_bitwise:                    ; bitwise level (lowest among binary operators)
        db '&'                  ; v2.0: still int16-typed; truncated/promoted
        dw bitwise_and           ; in expr_bitwise itself (only when an
        db '|'                  ; operator is actually present -- see
        dw bitwise_or            ; expr_bitwise's header)
        db '^'
        dw bitwise_xor
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
        dw kw_sin,  do_sin_func
        dw kw_cos,  do_cos_func
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
flt_negate:
        mov  di, FLT_A
        jmp  short fneg_tail
flt_negate_b:
        mov  di, FLT_B
fneg_tail:
        cmp  byte [di], 0
        je   flt_neg_r
        xor  byte [di+1], 0x80
flt_neg_r:
        ret

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
        cld
        movsw
        movsw
        pop  si
        ret

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
flt_from_int_b:
        mov  di, FLT_B
        jmp  short ffi_main
flt_from_int:
        mov  di, FLT_A
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
        jle  fti_zero           ; |value| < 1
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
        push word [FLT_A+2]
        push word [FLT_A+0]
        call flt_sub
        mov  cx, [FLT_A+0]
        or   cx, [FLT_A+2]
        mov  al, [FLT_A+1]
        pop  word [FLT_A+0]
        pop  word [FLT_A+2]
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
        jmp  flt_negate_b       ; restore FLT_B (tail-call)

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
        mov  al, [FLT_A+0]
        or   al, al
        jnz  fa_chkb
        jmp  flt_b_to_a         ; A=0 -> result=B
fa_chkb:
        mov  al, [FLT_B+0]
        or   al, al
        jnz  fa_both_nz
        ret                     ; B=0 -> result=A unchanged

fa_both_nz:
        ; Compare exponents; if B is larger, swap FLT_A<->FLT_B so the rest of
        ; the routine only has to handle ONE case ("FLT_A is the larger-or-
        ; equal operand"). This replaces what used to be two near-mirror
        ; 19-instruction load blocks with one swap (18 bytes) + one block.
        mov  al, [FLT_A+0]
        mov  ah, [FLT_B+0]
        cmp  al, ah
        jnb  fa_signs           ; FLT_A already >= FLT_B, no swap needed

        mov  ax, [FLT_A+0]
        xchg ax, [FLT_B+0]
        mov  [FLT_A+0], ax
        mov  ax, [FLT_A+2]
        xchg ax, [FLT_B+2]
        mov  [FLT_A+2], ax

fa_signs:
        ; FLT_A is now guaranteed the larger-or-equal operand.
        mov  al, [FLT_A+1]
        and  al, 0x80
        mov  [FLT_SA], al       ; sign of larger
        mov  al, [FLT_B+1]
        and  al, 0x80
        mov  [FLT_SB], al       ; sign of smaller

        ; Load larger (FLT_A) mantissa into CH:DX. The exponent (normally
        ; cached in BH) is instead reloaded from [FLT_A+0] at the 3 points
        ; it's actually needed below (fa_norm_reload, the carry-bump path,
        ; fa_smaller_gone) -- [FLT_A+0] stays valid/unchanged in memory
        ; throughout, and this frees BH for the smaller operand's mid
        ; byte, below.
        mov  ch, [FLT_A+1]
        or   ch, 0x80
        mov  dh, [FLT_A+2]
        mov  dl, [FLT_A+3]

        ; Load smaller (FLT_B) mantissa directly into AL(hi):BX(mid:lo) --
        ; no FLT_T memory scratch needed. The natural little-endian word
        ; load gives bl=mid,bh=lo (backwards); xchg corrects the order so
        ; BX, read as one 16-bit value, equals mid*256+lo.
        mov  bx, [FLT_B+2]
        xchg bh, bl             ; bh=B_mid, bl=B_lo
        mov  al, [FLT_B+1]
        or   al, 0x80           ; al=B_hi, implied-1 restored

        mov  cl, [FLT_A+0]
        sub  cl, [FLT_B+0]      ; shift count = expA - expB

fa_align:
        or   cl, cl
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
        or   cl, cl
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

        ; Different signs: subtract smaller from larger. Direct register-
        ; register sub/sbb -- no memory operand at all now.
        sub  dl, bl
        sbb  dh, bh
        sbb  ch, al
        jnc  fa_norm_reload

        ; Borrow out: two's-complement negate result, flip sign
        not  dl
        not  dh
        not  ch
        inc  dl
        adc  dh, 0
        adc  ch, 0
        or   ch, ch
        jnz  fa_flip_sign
        or   dx, dx
        jz   fa_zero
fa_flip_sign:
        xor  byte [FLT_SA], 0x80
        jmp  fa_norm_reload

fa_same_sign:
        ; Add with carry chain (LSB first). Register-register, same
        ; reasoning as the subtract path above.
        add  dl, bl
        adc  dh, bh
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

fa_norm_reload:
        mov  bh, [FLT_A+0]      ; reload exponent (BH held smaller's mid
                                 ; byte through the add/sub above)
fa_norm:
        or   ch, ch
        jnz  fa_np
        or   dx, dx
        jz   fa_zero
fa_np:
        xor  al, al
        jmp  norm_pack          ; backward jump - safe

fa_zero:
        jmp  flt_zero           ; backward jump - safe
fa_smaller_gone:
        mov  bh, [FLT_A+0]      ; reload exponent (never touched BH yet
                                 ; on this early-exit path, but BH still
                                 ; holds garbage from fa_signs' B_mid load)
        xor  al, al
        jmp  norm_pack          ; backward jump - safe

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
        mov  bl, [FLT_B+1]
        or   bl, 0x80           ; dead 'and bl,0x7F' removed
        xor  bh, bh
        mov  si, bx             ; SI = 0x00:BL
        mov  dh, [FLT_B+2]
        mov  dl, [FLT_B+3]     ; DX = B_lo

        ; Load A mantissa: DI=00:A_hi (byte in LOW position), AX=A_lo
        mov  cl, [FLT_A+1]
        or   cl, 0x80           ; dead 'and cl,0x7F' removed
        xor  ch, ch
        mov  di, cx             ; DI = 0x00:CL
        mov  ah, [FLT_A+2]
        mov  al, [FLT_A+3]     ; AX = A_lo

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
        jnc  fms10
        inc  dx
fms10:
        mov  bx, dx
        pop  dx                 ; restore DX = 00:A_hi

        ; Step 3: B_lo * A_hi  (B_lo re-read; DX = 00:A_hi)
        mov  ah, [FLT_B+2]
        mov  al, [FLT_B+3]     ; AX = B_lo
        mul  dx                 ; DX:AX = B_lo * A_hi
        add  cx, ax
        jnc  fms20
        inc  dx
fms20:
        add  bx, dx

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
        jmp  fdiv_by_zero
fdiv_bnz:
        mov  al, [FLT_A+0]
        or   al, al
        jnz  fdiv_anz
        ret                     ; 0 / x = 0
fdiv_anz:
        ; Exponent: eA - eB + 0x80
        sub  al, bl
        add  al, 0x80
        mov  [FLT_ER], al

        ; Result sign = sign_A XOR sign_B (shared helper)
        call sign_xor
        mov  dl, [FLT_B+1]
        or   dl, 0x80           ; dl = B_hi, implied-1 restored
        mov  [FLT_DB+2], dl     ; B_hi at +2 (little-endian layout below)
        mov  ax, [FLT_B+2]      ; al=B_mid, ah=B_lo (raw word load)
        xchg al, ah              ; al=B_lo, ah=B_mid
        mov  [FLT_DB+0], ax     ; DB+0=B_lo, DB+1=B_mid -- as a 16-bit
                                 ; word this equals B_mid*256+B_lo, the
                                 ; same packing BX already uses for A's
                                 ; mid:lo bytes below

        ; Load A mantissa: CH = A_hi (8-bit), BX = A_bytes2:3 (16-bit)
        mov  ch, [FLT_A+1]
        or   ch, 0x80           ; dead 'and ch,0x7F' removed
        mov  bh, [FLT_A+2]
        mov  bl, [FLT_A+3]

        ; Pre-scale: if A >= B shift right 1, inc exponent
        cmp  ch, dl             ; DL still holds FLT_DB+2(B_hi); no reload
        jb   fdiv_prescaled
        ja   fdiv_prescale
        cmp  bx, [FLT_DB+0]     ; single word compare (was 2 byte loads
                                 ; + cmp, before the FLT_DB reorder)
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
        jc   fdiv_ov            ; 25th-bit overflow: remainder unconditionally >= B

        ; Speculative subtract: CH:BX -= B (word+byte, direct memory
        ; operands, no scratch register needed -- AX/DX, the live 32-bit
        ; quotient, are never touched). If the final byte borrows, the
        ; remainder was < B; undo by adding B back (restore). Otherwise
        ; commit and set the bit.
        sub  bx, [FLT_DB+0]
        sbb  ch, [FLT_DB+2]
        jc   fdiv_restore        ; borrowed -> remainder was < B -> undo below
        or   al, 1               ; no borrow -> commit, set quotient bit 0
        jmp  fdiv_next

fdiv_restore:
        ; Undo the speculative subtract (add B back)
        add  bx, [FLT_DB+0]
        adc  ch, [FLT_DB+2]
        jmp  fdiv_next

fdiv_ov:
        ; S20: 25th-bit overflow guarantees remainder >= B; commit unconditionally
        sub  bx, [FLT_DB+0]
        sbb  ch, [FLT_DB+2]
        or   al, 1

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
        jmp  norm_pack          ; backward jump - safe

fdiv_by_zero:
        ; v2.0: was push si/mov si,s_div0/call print_sz/pop si/jmp flt_zero
        ; (printed "DIV0!" to the console and returned 0.0). Float divide
        ; by zero now raises the same ?2 error as integer divide by zero,
        ; via uBASIC's do_error -- see language reference / ERR_OV.
        mov  al, ERR_OV
        jmp  do_error            ; tail-call into uBASIC's error handler

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
        push ax
        mov  al, '-'
        call output
        pop  ax
        call flt_abs
fp_notneg:
        ; Decimal exponent estimate: de = (exp - 0x80) * 77 >> 8
        mov  al, [FLT_A+0]
        sub  al, 0x80
        cbw
        mov  cx, 77
        imul cx
        mov  al, ah
        mov  [FLT_DE], al

        ; S22: save FLT_A on stack
        push word [FLT_A+0]
        push word [FLT_A+2]

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
        mov  [di], al
        inc  di
        cmp  di, IBUF+7
        je   fp_dig_done
        push di
        call mul_by_ten
        pop  di
        jmp  fp_dig_lp

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
        pop  word [FLT_A+2]
        pop  word [FLT_A+0]
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
        push word [FLT_A+0]     ; stash integer part while FLT_A is reused below
        push word [FLT_A+2]
        call parse_frac         ; FLT_A = value of the fractional digits (0.xxx)
        call flt_a_to_b         ; FLT_B = fraction
        pop  word [FLT_A+2]     ; restore integer part into FLT_A
        pop  word [FLT_A+0]
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
; (Relocated here from its old spot as a filler between the reset vector
; and the final ROM pad -- this routine grew from 2 to 11 bytes for the
; float conversion and no longer fits in that 2-byte gap.)
; Inputs  : FLT_A = call address (from eat_paren_expr)
; Outputs : FLT_A = float(AX), where AX = return value from called routine
; Clobbers: AX, whatever the called routine clobbers
; =============================================================================
do_usr_func:
        call flt_to_int         ; AX = int16(FLT_A) = call address
        call ax       ; CALL (not JMP) so we return here after
        jmp  flt_from_int        ; tail-call: FLT_A = float(AX)

; =============================================================================
; CORDIC SIN/COS  (v2.1)
;
; Computes SIN(x) and COS(x) for any real x in radians via fixed-point
; (Q14: 1 sign + 1 integer + 14 fraction bits, range +-1.99994) rotation-
; mode CORDIC, converted to/from MBF4 float only at the boundary. The
; iteration loop itself is plain 16-bit integer shift/add/sub -- this is
; substantially smaller than a float-native CORDIC core (measured via a
; standalone prototype during design: ~143 bytes fixed-point core vs
; ~224 bytes float-native, despite paying for both float<->fixed
; conversions either way) because each of the 14 iterations avoids a
; flt_add/flt_sub call in favour of one SAR/ADD/SUB triple.
;
; ALGORITHM
;   1. Reduce x to t = x mod 2*PI, t in [0, 2*PI).
;   2. quadrant = floor(t / (PI/2)), 0-3;  r = t - quadrant*(PI/2), r in
;      [0, PI/2).
;   3. Convert r to Q14 fixed-point (Z0); seed X0 = 1/K (Q14, K = CORDIC
;      gain for 14 iterations), Y0 = 0 -- this bakes the gain correction
;      into the seed for free, so no post-multiply is needed.
;   4. Run 14 rotation-mode iterations: X=cos(r), Y=sin(r) in Q14.
;   5. Recombine per quadrant (verified against real sin/cos numerically
;      for all 4 quadrants and exact axis boundaries before writing this):
;        Q0: sin=Y      cos=X
;        Q1: sin=X      cos=-Y
;        Q2: sin=-Y     cos=-X
;        Q3: sin=-X     cos=Y
;   6. Convert the selected Q14 result back to float.
;
; Precision: ~14 bits (~4 decimal digits), noticeably less than MBF4's
; ~23-bit mantissa (~7 digits) -- fine for general use (graphics, basic
; trig); flag this if a program ever needs full float precision from
; SIN/COS specifically.
; =============================================================================

CORDIC_N:       equ 14          ; iteration count
CORDIC_INVK:    equ 9949        ; 1/K in Q14, K = gain for 14 iterations

; 14 entries x 2 bytes = 28 bytes: atan(2^-i) in Q14, i=0..13
cordic_atan_tab:
        dw 12868, 7596, 4014, 2037, 1023, 512, 256
        dw 128, 64, 32, 16, 8, 4, 2

two_pi_const:  db 0x83, 0x49, 0x0F, 0xDB  ; 2*PI as MBF4
half_pi_const: db 0x81, 0x49, 0x0F, 0xDB  ; PI/2 as MBF4
fx14_scale:    db 0x8F, 0x00, 0x00, 0x00  ; 16384.0 as MBF4 (Q14 scale factor)

; =============================================================================
; CORDIC_ROTATE_FX  fixed-point Q14 rotation-mode CORDIC core.
; Inputs  : CORDIC_X, CORDIC_Y, CORDIC_Z preloaded (Q14)
; Outputs : CORDIC_X, CORDIC_Y updated in place (CORDIC_Z left ~0, unused
;           by callers here)
; Clobbers: AX, BX, CX
; =============================================================================
cordic_rotate_fx:
        xor  cx, cx              ; CL doubles as iteration index AND shift
                                  ; count (always < 16, so CH staying 0
                                  ; throughout is harmless)
.loop:
        mov  ax, [CORDIC_X]
        sar  ax, cl              ; AX = X >> i  (arithmetic: sign-preserving)
        mov  [CORDIC_T], ax       ; T = X >> i  (need the OLD X for Y's update)

        mov  ax, [CORDIC_Y]
        sar  ax, cl               ; AX = Y >> i

        push cx                  ; table lookup needs BX; stash CX first
        mov  bx, cx
        add  bx, bx
        add  bx, cordic_atan_tab
        pop  cx

        cmp  word [CORDIC_Z], 0
        jl   .neg

        sub  word [CORDIC_X], ax ; X -= (Y>>i)
        mov  ax, [CORDIC_T]
        add  word [CORDIC_Y], ax ; Y += (X_old>>i)
        mov  ax, [bx]
        sub  word [CORDIC_Z], ax ; Z -= atan(i)
        jmp  .next

.neg:
        add  word [CORDIC_X], ax ; X += (Y>>i)
        mov  ax, [CORDIC_T]
        sub  word [CORDIC_Y], ax ; Y -= (X_old>>i)
        mov  ax, [bx]
        add  word [CORDIC_Z], ax ; Z += atan(i)

.next:
        inc  cx
        cmp  cx, CORDIC_N
        jb   .loop
        ret

; =============================================================================
; FLT_TO_FX14  convert FLT_A (already range-reduced to [0,PI/2)) to a
; Q14 fixed-point word.
; Inputs  : FLT_A
; Outputs : AX = Q14 value
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_A, FLT_B
; =============================================================================
flt_to_fx14:
        mov  si, fx14_scale
        call load_const_b
        call flt_mul             ; FLT_A *= 16384.0
        jmp  flt_to_int          ; tail-call: AX = int16(FLT_A)

; =============================================================================
; FX14_TO_FLT  convert a Q14 fixed-point word back to float.
; Inputs  : AX = Q14 value
; Outputs : FLT_A = AX / 16384.0
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_A, FLT_B
; =============================================================================
fx14_to_flt:
        call flt_from_int        ; FLT_A = float(AX)
        mov  si, fx14_scale
        call load_const_b
        jmp  flt_div             ; tail-call: FLT_A /= 16384.0

; =============================================================================
; LOAD_CONST_B  copy a 4-byte ROM float constant into FLT_B.
; Shared tail for CORDIC_REDUCE's four same-shaped constant loads (was
; inlined 4 times at ~13 bytes each; SI is never live across any of
; those call sites in CORDIC_REDUCE, so the push/pop si those inline
; copies used for caller-safety was pure overhead here -- dropped).
; Inputs  : SI -> 4-byte ROM float constant
; Outputs : FLT_B = that constant
; Clobbers: AX, DI
; =============================================================================
load_const_b:
        mov  di, FLT_B
        cld
        movsw
        movsw
        ret

; =============================================================================
; CORDIC_REDUCE  reduce FLT_A (any-magnitude radians) to r in [0,PI/2)
; and report the quadrant.
; GOTCHA: uses its own dedicated CORDIC_C scratch, not FLT_C -- see
; FLT_C's RAM-map comment for why.
; Inputs  : FLT_A = angle in radians
; Outputs : FLT_A = reduced angle r in [0,PI/2); AL = quadrant (0-3)
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_A, FLT_B, CORDIC_C
cordic_reduce:
        ; CORDIC_C = original angle (need it twice: once /2pi for the
        ; quotient, once again for the final subtract)
        mov  si, FLT_A
        mov  di, CORDIC_C
        cld
        movsw
        movsw

        ; FLT_A = angle / (2*PI)
        mov  si, two_pi_const
        call load_const_b
        call flt_div
        call flt_to_int          ; AX = int16(angle / 2pi), truncated toward 0

        ; FLT_A = quotient * 2*PI
        call flt_from_int
        mov  si, two_pi_const
        call load_const_b
        call flt_mul

        ; FLT_A = original_angle - quotient*2*PI  (= t, in (-2pi,2pi))
        call flt_a_to_b          ; FLT_B = quotient*2*PI
        mov  si, CORDIC_C
        mov  di, FLT_A
        cld
        movsw
        movsw                    ; FLT_A = original angle
        call flt_sub             ; FLT_A = angle - quotient*2*PI

        ; If still negative, add 2*PI once to land in [0, 2*PI). Exact
        ; zero has byte[0]==0 (the MBF4 zero encoding) and never sets
        ; bit 7 of byte[1] in this library's convention, so testing bit
        ; 7 directly is sufficient -- no separate exact-zero case needed.
        test byte [FLT_A+1], 0x80
        jz   .nonneg
        mov  si, two_pi_const
        call load_const_b
        call flt_add
.nonneg:
        ; quadrant = floor(t / (PI/2)); since t is now in [0,2*PI), plain
        ; truncation toward zero IS floor here (no negative case left).
        ; flt_div computes FLT_A/FLT_B -> FLT_A, so FLT_A must be t and
        ; FLT_B must be PI/2 -- t is already in FLT_A; just load FLT_B.
        mov  si, FLT_A
        mov  di, CORDIC_C        ; stash t again (flt_div will overwrite
                                  ; FLT_A with the quotient, but the
                                  ; quadrant calc below needs t itself
                                  ; back to compute r = t - quadrant*PI/2)
        cld
        movsw
        movsw
        mov  si, half_pi_const
        call load_const_b
        call flt_div             ; FLT_A = t / (PI/2)
        call flt_to_int          ; AL = quadrant (0-3; t in [0,2pi) so
                                  ; this is always exactly 0,1,2, or 3)

        ; r = t - quadrant*(PI/2)
        push ax                  ; save quadrant across flt_from_int/mul
        call flt_from_int        ; FLT_A = float(quadrant)
        mov  si, half_pi_const
        call load_const_b
        call flt_mul             ; FLT_A = quadrant * (PI/2)
        call flt_a_to_b          ; FLT_B = quadrant*(PI/2)
        mov  si, CORDIC_C
        mov  di, FLT_A
        cld
        movsw
        movsw                    ; FLT_A = t (restored from stash)
        call flt_sub             ; FLT_A = t - quadrant*(PI/2) = r
        pop  ax                  ; AL = quadrant (restored)
        ret

; =============================================================================
; CORDIC_PICK  shared quadrant recombination for DO_SIN_FUNC/DO_COS_FUNC.
; Register selection (which of CORDIC_X/CORDIC_Y) alternates on quadrant
; bit0 for BOTH functions -- confirmed against the verified quadrant
; table (q0..q3: sin=[Y,X,Y,X], cos=[X,Y,X,Y], i.e. always opposite
; parity from each other, same "bit0 selects" rule). The NEGATE rule is
; NOT the same for both, though -- re-derived numerically rather than
; assumed, since a first attempt at this guessed wrong:
;   SIN negates when quadrant bit1 = 1            (q2,q3)
;   COS negates when quadrant bit0 XOR bit1 = 1    (q1,q2)
; The ENTIRE negate decision is computed into DL before AX is loaded
; with the result word at all -- two earlier drafts of this routine
; each clobbered half of AX (first AH, then AL, via an 'and al,1' that
; forgot AL already held the low byte of the loaded result) while
; computing this decision after the load. Confirmed via emulated
; testing: COS(0) returned 0.984436 instead of 1.0 because of exactly
; this (16383 corrupted to 16129 by a stray 'and al,1'). DL is never
; touched by the [bx]/[si] load (that only writes AX), so there is no
; register left whose "still holds quadrant bits, not yet the result"
; state could be confused with "now holds half the result" -- the two
; pieces of data physically cannot collide on the same register this way.
; Inputs  : AL = quadrant (0-3)
;           SI = THIS function's bit0=0 (primary) register address
;                (DX cannot address memory on 8086 -- only BX/BP/SI/DI
;                can -- an earlier draft of this routine used DX here
;                and tinyasm correctly rejected it)
;           BX = THIS function's bit0=1 (secondary) register address
;           CH = 0 for SIN's negate rule, 1 for COS's negate rule
; Outputs : AX = selected, correctly-signed Q14 result
; Clobbers: AX, CL, DL
; =============================================================================
cordic_pick:
        ; Compute everything from the quadrant FIRST, entirely in
        ; CL/DL, before AX is touched at all.
        mov  cl, al
        and  cl, 1               ; CL = bit0(quadrant) -- register select
        mov  dl, al
        shr  dl, 1
        and  dl, 1               ; DL = bit1(quadrant)
        test ch, ch
        jz   .have_negate        ; CH=0 (SIN): negate rule is bit1 alone
        xor  dl, cl              ; CH=1 (COS): negate rule is bit0 XOR bit1
.have_negate:
        ; Now safe to load the result -- CL/DL hold everything needed,
        ; and neither AL's nor AH's prior content matters any more.
        test cl, 1
        jz   .primary
        mov  ax, [bx]
        jmp  .test
.primary:
        mov  ax, [si]
.test:
        test dl, 1
        jz   .done
        neg  ax
.done:
        ret

; =============================================================================
; DO_SIN_FUNC  SIN(x)
; Inputs  : FLT_A = x (radians, from eat_paren_expr)
; Outputs : FLT_A = sin(x)
; Clobbers: AX, BX, CX, DX, DI, FLT_A, FLT_B, CORDIC_* (SI preserved)
; =============================================================================
do_sin_func:
        ; GOTCHA: reached via a tail-call chain (expr2's e2_func_call is
        ; "jmp [bx+2]", not "call"), so SI holds the caller's real parse
        ; position on entry and must still hold it on return -- but
        ; CORDIC_PICK and FX14_TO_FLT both use SI internally. Save/
        ; restore it here, and call FX14_TO_FLT (not tail-call into it,
        ; which would clobber SI again after the restore). See change
        ; history.
        push si
        call cordic_reduce        ; FLT_A = r [0,PI/2), AL = quadrant
        mov  [FLT_DE], al        ; stash quadrant -- CORDIC_T can't be
                                  ; used for this: cordic_rotate_fx
                                  ; overwrites the whole word every
                                  ; iteration of its own loop. FLT_DE is
                                  ; untouched by every flt_* call used
                                  ; between here and reading it back.
        call flt_to_fx14          ; AX = Q14(r)
        mov  [CORDIC_Z], ax
        mov  word [CORDIC_X], CORDIC_INVK
        mov  word [CORDIC_Y], 0
        call cordic_rotate_fx
        mov  al, [FLT_DE]        ; recover stashed quadrant
        mov  si, CORDIC_Y         ; SIN's primary (bit0=0) register is Y
        mov  bx, CORDIC_X
        mov  ch, 0                ; SIN's negate rule
        call cordic_pick
        call fx14_to_flt         ; FLT_A = float(AX)  (real call, not
                                  ; tail-call: SI must be restored AFTER
                                  ; this returns, since it clobbers SI too)
        pop  si                  ; restore the real parse position
        ret

; =============================================================================
; DO_COS_FUNC  COS(x)
; GOTCHA: see DO_SIN_FUNC's header -- same SI-preservation issue
; (CORDIC_PICK and FX14_TO_FLT both clobber SI internally; the real
; parse-position SI is saved/restored around the whole sequence).
; Inputs  : FLT_A = x (radians, from eat_paren_expr)
; Outputs : FLT_A = cos(x)
; Clobbers: AX, BX, CX, DX, DI, FLT_A, FLT_B, CORDIC_* (SI preserved)
; =============================================================================
do_cos_func:
        push si
        call cordic_reduce
        mov  [FLT_DE], al
        call flt_to_fx14
        mov  [CORDIC_Z], ax
        mov  word [CORDIC_X], CORDIC_INVK
        mov  word [CORDIC_Y], 0
        call cordic_rotate_fx
        mov  al, [FLT_DE]
        mov  si, CORDIC_X         ; COS's primary (bit0=0) register is X
        mov  bx, CORDIC_Y
        mov  ch, 1                ; COS's negate rule
        call cordic_pick
        call fx14_to_flt         ; real call, not tail-call -- see
                                  ; DO_SIN_FUNC's header for why
        pop  si                  ; restore the real parse position
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
