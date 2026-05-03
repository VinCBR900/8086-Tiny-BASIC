; =============================================================================
; uBASIC 8088  v1.7.2
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Tiny BASIC interpreter for the 8088/8086.  Single-segment, integer-only.
; Targets a 2 KB code ROM + 2 KB RAM embedded system; also runs in
; 8bitworkshop (x86 mode) with a pre-loaded Mandelbrot showcase.
;
; Credit: Oscar Toledo G. for bootBASIC inspiration and tinyasm assembler.
;    
; ---------------------------------------------------------------------------
; LANGUAGE REFERENCE
; ---------------------------------------------------------------------------
;
; Statements  : PRINT  IF..THEN  GOTO  GOSUB  RETURN  FOR..TO[..STEP]  NEXT
;               LET  INPUT  REM  OUT  END  RUN  LIST  NEW  POKE  FREE  HELP
; Expressions : + - * / % & | = < > <= >= <>   unary-
;               CHR$(n)  PEEK(addr)  IN(port)  USR(addr) TAB(spaces) variables A..Z
; Numbers     : signed 16-bit  (-32768 .. 32767)
; Multi-stmt  : colon separator ':'  (avoid FOR/NEXT or GOSUB/RETURN on same line)
; Errors      : ?0 syntax   ?1 undefined line   ?2 divide/zero   ?3 out of memory
;               ?4 bad variable   ?5 RETURN without GOSUB   ?6 NEXT without FOR
;		?B break (ROM only)
;
; Line store  : <lo> <hi> <tokenised body> <CR>
;   Line numbers 1-32767
;   PRINT String literals and REM bodies stored verbatim.
;
; =============================================================================
; BUILD INSTRUCTIONS
; =============================================================================
;
; Assembler: Oscar Toledo's tinyasm  (https://github.com/nanochess/tinyasm)
;    tinyasm -f bin uBASIC8088.asm -o uBASIC_rom.bin
; ---------------------------------------------------------------------------
; Variant 1: Standalone 2 KB ROM  (real hardware or `sim_rom.c` simulator)
; ---------------------------------------------------------------------------
;
;   Hardware:
;     CPU    : Intel 8088 @ 5 MHz (or compatible)
;     ROM    : 2 KB  phys 0xF800-0xFFFF  (address line A12=1 selects ROM)
;     RAM    : 2 KB  phys 0x0000-0x07FF  (address line A12=0 selects RAM)
;     Serial : Intel 8755 MMIO
;                Port A (0x00)  bit 0 = TX (output)   bit 1 = RX (input)
;                DDR A  (0x02)  init: 0xFD  (all outputs except RX)
;              Baud rate: 4800 baud @ 5 MHz  (BAUD = 60 loop constant)
;     Reset  : 8086 reset -> phys 0xFFFF0 -> FAR JMP to 0xF800:0x0000
;     INT 0  : divide-by-zero  -> print ?2, re-enter interpreter
;     INT 2  : NMI / break key -> print ?B, re-enter interpreter
;
;   Memory map:
;     ORIGIN   = 0xF800       ROM: 0xF800-0xFFFF, reset stub at 0xFFF0
;     RAM_BASE = 0x0000       RAM: 0x0000-0x07FF
;     STACK    = 0x0800       top of RAM, grows downward
;     I/O      = bitbang UART via 8755 Port A
;
;   Simulate:
;     gcc -O2 -o sim_rom sim_rom.c cpu.c    # XTulator cpu core by Mike Chambers
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
; ---------------------------------------------------------------------------
; Variant 2: 8bitworkshop online IDE  (YASM assembler, FREEDOS EXE)
; ---------------------------------------------------------------------------
;
;   Open directly at https://8bitworkshop.com in 8086 mode.
;   YASM defines __YASM_MAJOR__ which selects this variant automatically.
;   Assembled as a FREEDOS EXE; auto executes the Mandelbrot showcase.
;
;   Memory map:
;     ORIGIN   = 0xF800       (8bitworkshop segment base)
;     RAM_BASE = 0x0000,      RAM: 0x0000-0x0FFF, 4Kbyte
;     I/O      = BIOS INT 10h / INT 16h
;
; =============================================================================
; CHANGE HISTORY
; =============================================================================
;   v1.7.2 (2026-05-02)  Refactored operator table, Added & | BOOL operators
;     - added PRINT TAB(spaces), fix NEXT error 
;   v1.7.1 (2026-05-02)  Size optimisation (11 bytes saved, 67->78 bytes free):
;     - xor ah,ah -> cbw in stmt dispatch, get_var_addr, do_list detokenize
;       (AL is always 0..25 at these points so sign-extension is safe; cbw=1B
;       vs xor ah,ah=3B saves 2B each, 6B total).
;     - tokenize: removed push ax/pop ax around call spaces (spaces only
;       touches SI, never AX; push/pop were unnecessary, 2B saved).
;     - do_for: replaced mov [INS_TMP],di / mov di,[INS_TMP] with mov bp,di /
;       mov di,bp (BP free in do_for; avoids memory round-trip, 3B saved).
;
;   v1.7.0 (2026-05-01)  Refactor and LIST range feature.
;     - LIST accepts optional <start>,<end> line range.
;     - EXPR2 function dispatch table refactor.
;     - Size refactoring: LET/INPUT, POKE/OUT, tokenizer, stmt dispatcher.
;     - editln: push cx / pop cx around call deline (deline's rep movsb
;       zeroed CX, causing insline to write zero body bytes on line replace).
;   v1.6.1 (2026-04-30)  Bug-fix release (sim_rom  debugging):
;     - sbb ax,ax before jl in relational eval clobbered flags from cmp;
;       all signed < / > comparisons were wrong.  Removed sbb.
;     - Showcase token bytes updated for TK_OUT insertion (THEN/TO/STEP each
;       shifted up by one: 0x91->0x92, 0x92->0x93, 0x93->0x94).
;   v1.6.0 (2026-04-26)  Added IN / OUT port I/O commands.
;     Refactored statement dispatch table to create space; misc size savings.
;   v1.5.0 (2026-04-23)  ROM target.
;     Bitbang UART (8755 Port A), interrupt vectors (DIV0/NMI), reset stub at
;     0xFFF0, bdly delay routine, showcase PROG_END init, rep stosw range fix.
;   v1.4.0 (2026-04-19)  FOR / NEXT with optional STEP.
;     4-entry FOR stack; TK_FOR=0x8F TK_NEXT=0x90 TK_THEN=0x91 TK_TO=0x92
;     TK_STEP=0x93; several kw_match/DI/CX clobber fixes; error ?6 NEXT
;     without FOR; dp_str/new_line tail-call optimisation.
;   v1.3.0 (2026-04-17)  Tokeniser + line-editor refactor.
;     Keywords stored as 0x80-0x8F tokens; LIST detokenises; 
;     stmt fast-path dispatch; insline/deline/editln refactored.
;     detokenises; stmt fast-path dispatch; insline/deline/editln refactored.
;     v1.3.1: LIST CR+LF fix; showcase re-encoded in tokenised form 
;   v1.2.0 (2026-04-17)  GOSUB / RETURN.
;     8-entry GOSUB stack; error ?5 RETURN without GOSUB.
;   v1.1.0 (2026-04-17)  Bug fixes.
;     Relational xor ax,ax; do_new clears full program store; tinyasm equ
;     compatibility; dp_str_eol dead label removed.
;   v1.0.0 (2026-04-09)  First release.
;     Clean 8088 port of uBASIC 65c02 v17.0.  Numerous expression, segment,
;     and editor bug fixes through v1.0.9 (see git log for detail).
;     IBUF expanded to 64 bytes; Mandelbrot showcase embedded (v1.0.9).
; =============================================================================

        	cpu 8086
                
; Configure origin and RAM base for target platform.
; 2KB ROM at 0xF800, 2KB RAM at 0x0000
ORIGIN:         equ 0xF800
RAM_BASE:       equ 0x0000
%ifdef __YASM_MAJOR__           ; 8bitworkshop: yasm defines __YASM_MAJOR__
RAM_SIZE:       equ 4096        ; 8bitworkshop: 4KB address space
%else
RAM_SIZE:       equ 2048        ; 2KB RAM (A12=0 selects RAM, A12=1 selects ROM)
%endif
        
; --- RAM layout (all relative to RAM_BASE) -----------------------------------
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
ERR_SN:         equ 0x30  ; Syntax
ERR_UL:         equ 0x31  ; 
ERR_OV:         equ 0x32  ; Overflow 
ERR_OM:         equ 0x33  ; Out of memory
ERR_UK:         equ 0x34  ;
ERR_RT:         equ 0x35  ; RETURN without GOSUB
ERR_NF:         equ 0x36  ; NEXT without FOR
ERR_BRK:        equ 0x42  ; NMI break: prints "?B"

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
BAUD:           equ 57          ; bit-period loop count: 17cy/iter @5MHz ~4800baud

; =============================================================================
; Pre-loaded showcase (8BitWorkshop only).  Type RUN to execute, NEW to clear.
; Feature demos   lines 10-190 : arithmetic, comparisons, FOR/NEXT, GOSUB
; Mandelbrot      lines 200-340: FOR loops for rows/cols, GOSUB 600 for escape
; Subroutines     500=sum1..10, 550=factorial5, 600=record-escape
;
; Fixed-point scale 1/64.  16 Mandelbrot iterations.  ASCII density chars.
; Tokens v1.6.0: PRINT=0x80 IF=0x81 GOSUB=0x8D RETURN=0x8E END=0x88
;         FOR=0x8F NEXT=0x90 OUT=0x91 THEN=0x92 TO=0x93 STEP=0x94 REM=0x87
; =============================================================================

; 8bitworkshop default org 0
%ifdef __YASM_MAJOR__
	mov ax, reset_vec; Trampoline for 8bitworkshop, overwritten when running
	jmp ax          ; One way to do a Near jump greater than 32768
	times PROGRAM - ($-$$) db 0  ;  Pad over program VARS/Equates (3byte mov, 2byte jump)

SHOWCASE_DATA:
        ; ── Feature demos ────────────────────────────────────────────────────
        db 0x0A,0x00,0x87,"uBASIC 8088 v1.6.0 showcase",0x0D         ; 10  REM ...
        db 0x14,0x00,0x80,0x22,"--- ARITHMETIC ---",0x22,0x0D         ; 20  PRINT
        db 0x1E,0x00,0x80,0x22,"2+3=",0x22,";2+3;",0x22,"  6*7=",0x22,";6*7",0x0D   ; 30
        db 0x28,0x00,0x80,0x22,"20/4=",0x22,";20/4;",0x22,"  17%5=",0x22,";17%5",0x0D ; 40
        db 0x32,0x00,0x80,0x22,"--- COMPARISONS ---",0x22,0x0D        ; 50
        db 0x3C,0x00,0x81,"5>3 ",0x92,0x80,0x22,"5>3 ok",0x22,0x0D   ; 60  IF THEN(0x92) PRINT
        db 0x46,0x00,0x81,"3<5 ",0x92,0x80,0x22,"3<5 ok",0x22,0x0D   ; 70
        db 0x50,0x00,0x81,"3>=3 ",0x92,0x80,0x22,"3>=3 ok",0x22,0x0D ; 80
        db 0x5A,0x00,0x81,"4<>3 ",0x92,0x80,0x22,"4<>3 ok",0x22,0x0D ; 90
        db 0x64,0x00,0x80,0x22,"--- FOR/NEXT ---",0x22,0x0D           ; 100
        db 0x6E,0x00,0x8F,"I=1 ",0x93,"5",0x0D                       ; 110 FOR I=1 TO(0x93) 5
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
        db 0xC8,0x00,0x8F,"I=-64 ",0x93,"56 ",0x94,"6",0x0D           ; 200 FOR I=-64 TO(0x93) 56 STEP(0x94) 6
        db 0xD2,0x00,0x8F,"C=-128 ",0x93,"16 ",0x94,"4",0x0D          ; 210 FOR C=-128 TO(0x93) 16 STEP(0x94) 4
        db 0xDC,0x00,"D=I:A=C:B=D:E=0",0x0D                           ; 220 init row
        db 0xE6,0x00,0x8F,"N=1 ",0x93,"16",0x0D                       ; 230 FOR N=1 TO(0x93) 16
        db 0xF0,0x00,"T=A*A/64-B*B/64+C",0x0D                         ; 240 iterate
        db 0xFA,0x00,"B=2*A*B/64+D:A=T",0x0D                          ; 250
        db 0x04,0x01,0x81,"A*A/64+B*B/64>256 ",0x92,0x8D,"600",0x0D  ; 260 IF .. THEN(0x92) GOSUB 600
        db 0x0E,0x01,0x90,"N",0x0D                                     ; 270 NEXT N
        db 0x18,0x01,0x81,"E>0 ",0x92,0x80,"CHR$(E+32);",0x0D         ; 280 IF E>0 THEN(0x92) PRINT
        db 0x22,0x01,0x81,"E=0 ",0x92,0x80,"CHR$(32);",0x0D           ; 290 IF E=0 THEN(0x92) PRINT
        db 0x2C,0x01,0x90,"C",0x0D                                     ; 300 NEXT C
        db 0x36,0x01,0x80,0x22,0x22,0x0D                               ; 310 PRINT "" (newline)
        db 0x40,0x01,0x90,"I",0x0D                                     ; 320 NEXT I
        db 0x4A,0x01,0x88,0x0D                                         ; 330 END
        ; ── Subroutine 500: sum 1..10 ─────────────────────────────────────────
        db 0xF4,0x01,"S=0",0x0D                                        ; 500
        db 0xFE,0x01,0x8F,"J=1 ",0x93,"10",0x0D                       ; 510 FOR J=1 TO(0x93) 10
        db 0x08,0x02,"S=S+J",0x0D                                      ; 520
        db 0x12,0x02,0x90,"J",0x0D                                     ; 530 NEXT J
        db 0x1C,0x02,0x8E,0x0D                                         ; 540 RETURN
        ; ── Subroutine 550: factorial 5 ───────────────────────────────────────
        db 0x26,0x02,"F=1",0x0D                                        ; 550
        db 0x30,0x02,0x8F,"K=1 ",0x93,"5",0x0D                        ; 560 FOR K=1 TO(0x93) 5
        db 0x3A,0x02,"F=F*K",0x0D                                      ; 570
        db 0x44,0x02,0x90,"K",0x0D                                     ; 580 NEXT K
        db 0x4E,0x02,0x8E,0x0D                                         ; 590 RETURN
        ; ── Subroutine 600: record escape iteration ───────────────────────────
        db 0x58,0x02,0x81,"E=0 ",0x92,"E=N",0x0D                      ; 600 IF E=0 THEN(0x92) E=N
        db 0x62,0x02,0x8E,0x0D                                         ; 610 RETURN
        dw 0                                                            ; end sentinel
SHOWCASE_END:
        times ORIGIN-($-$$) db 0
%else
        org ORIGIN
%endif

; =============================================================================
; INIT  cold start
; Clobbers: everything
; =============================================================================
start:
%ifdef __YASM_MAJOR__
	cld
	mov ax, cs      ; EXE: normalise DS/ES/SS to CS (FREEDOS leaves them at PSP)
%else
        ; ROM cold start: CS=0xF800 after far JMP. RAM is at segment 0.
        xor ax, ax      ; AX=0 -> DS=ES=SS=0 (RAM segment)
%endif
        mov ds, ax
        mov es, ax
        mov ss, ax
        mov sp, STACK_TOP
        mov di, RAM_BASE

%ifndef __YASM_MAJOR__
        ; Zero ALL RAM first (variables, FOR stack, program store, vector area).
        ; Must do this BEFORE installing vectors or setting PROG_END.
        mov cx, RAM_SIZE / 2
%else
        ; Zero vars area (not program store - showcase lives there).
        mov cx, PROGRAM / 2     ; words to zero = 0xB9/2 = 92 words
        xor ax, ax              ; AX=0 -> DS=ES=SS=0 (RAM segment)
%endif
        rep stosw

        ; Install interrupt vectors into the now-zeroed IVT.
        ; meaningless for 8bitworkshop
        xor di, di              ; DI -> [0x0000] = INT 0 (divide error)
        xchg ax,di
        mov ax, divide_error
        stosw                   ; [0x0000] = IP of divide_error
        mov ax, 0xf800
        push ax			; save 0xf800
        stosw                   ; [0x0002] = CS = 0xF800
        mov di, 8               ; DI -> [0x0008] = INT 2 (NMI)
        mov ax, nmi_handler
        stosw                   ; [0x0008] = IP of nmi_handler
        pop ax
        stosw                   ; [0x000A] = CS = 0xF800

%ifndef __YASM_MAJOR__
        ; PROG_END = empty program (set after rep stosw so it isn't wiped)
        mov word [PROG_END], PROGRAM
%else
        ; PROG_END points past last showcase byte (excl sentinel)
        mov word [PROG_END], PROGRAM+(SHOWCASE_END-SHOWCASE_DATA)-2
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
        sub al, TK_PRINT        ; AL = 0..17 (index into st_tab)
        cbw
        add ax, ax              ; AX = index * 2 (dw handler table)
        mov bx, st_tab
        add bx, ax              ; BX -> correct st_tab handler
        jmp [bx]                ; dispatch directly (saves kw_match loop)
        ; --- Text fall-through (direct mode) ---
stmt_text:
        mov bx, tk_kw_tab
        mov cx, NUM_TOKENS
stmt_lp:
        call kw_match
        jnc stmt_call
        add bx, 2
        loop stmt_lp
        jmp do_let              ; no keyword -> implicit LET/assignment
stmt_call:
        mov ax, bx
        sub ax, tk_kw_tab
        add ax, st_tab
        mov bx, ax
        jmp [bx]                ; indirect call to handler  

; =============================================================================
; DO_INPUT  INPUT <var>
; =============================================================================
do_input:
        ; Validate variable letter before proceeding
	call let_input_hlpr
        push di                 ; save var addr: input_line/expr clobber DI
        mov al, '?'
        call output
        call output_space
        call input_line
        jmp short let_store_ax
        
; =============================================================================
; DO_LET  [LET] <var> = <expr>
; =============================================================================
do_let:
	call let_input_hlpr
        push di                 ; save var addr: expr clobbers DI via kw_match
        call expect_equals
let_store_ax:
        call expr
        pop di
        mov [di], ax
        ret
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
	; drop through
        
; =============================================================================
; GET_VAR_ADDR  letter at [SI] -> DI=&var, SI advanced
; =============================================================================
get_var_addr:
        lodsb
        call uc_al
        sub al, 'A'
        cbw
        add ax, ax
        add ax, VARS
        mov di, ax
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
; DO_LIST - LIST <start,end> note start/end optional but both must be provided
; =============================================================================
do_list:
        mov di, PROGRAM		; Default first line memory
        mov bp, 0x7fff 		; default last line number	
        call peek_line		; just LIST?
        je dl_lp		; Yes, show every line
        call poke_out_hlpr  	; No - get di = arg1, ax = arg2    
        mov bp,ax		; save last line
        mov ax,di		; get arg1
        call find_line		; start line in DI
dl_lp:
        mov ax, [di]        	; Get line number word at DI
        test ax, ax         	; Shortest way to check for NULL sentinel
        jz dl_done          	; Exit if 0
	cmp bp,ax 		; are we at at last line
        jl dl_done
        call output_number
        call output_space

        mov si, di          	; SI = DI + 2
        add si, 2

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
        cbw
        add ax, ax              ; word offset
        mov bx, tk_kw_tab
        add bx, ax
        mov bx, [bx]            ; BX -> keyword string
        ; print keyword chars (bit-7 terminated), followed by space
        push si
        mov si, bx
dl_kw_lp:
        call dp_str
        call output_space
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
        jne dp_chrs
        inc si		; skip over "
dp_str:
        lodsb
    	cmp al, 0x22	; check for PRINT terminator
    	je dp_after
    
    	test al, 0x80	; check for top bit terminator
    	jz loop_print
    
    	and al, 0x7f
    	jmp output        ; Tail-call, output will RET for us

loop_print:
        call output
        jmp short dp_str		
        
dp_chrs:
        mov bx, chrs_tab	; Check for CHR$
        call kw_match
        jc dp_tab
        call eat_paren_expr
        call output
        jmp short dp_after
dp_tab:
        mov bx, tab_tab	; Check for TAB
        call kw_match
        jc dp_num
        call eat_paren_expr
        mov cx,ax
        jcxz dp_after           ; TAB(0) -> nothing
tab_loop:
	call output_space
        loop tab_loop
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
; DO_FREE  print free program-store bytes - also do_pritn newline
; =============================================================================
do_free:
        mov ax, PROGRAM_TOP
        sub ax, [PROG_END]
        call output_number
        call output_space
        mov si, kw_free
        call dp_str
dp_nl:
	jmp new_line     ; Tail-call: new_line will RET for us

; =============================================================================
; DO_HELP  print all keywords
; =============================================================================
do_help:
    mov si, kw_tab_start
dh_lp:
    call dp_str
    call output_space
    cmp byte [si], 0 ; Check for sentinel
    jne dh_lp        ; Loop back if not zero
    jmp new_line     ; Tail-call: new_line will RET for us

; =============================================================================
; DO_POKE  POKE <addr>, <val> - need to finagle addresses above 32768
; DO_OUT  OUT <addr>, <val> 
; =============================================================================
poke_out_hlpr:
        call expr               ; Get address
        push ax                 ; Save it
        mov al, ','
        call expect       ; Consolidates the old 'cmp/jne/inc' block
        call expr               ; Get value
        pop di                  ; DI = address
dp_ret:        
        ret                     ; AL = value (from expr result)

; do_poke and do_out now just call the helper and execute one instruction
do_poke:
        call poke_out_hlpr
        mov [di], al
        ret

do_out:
        call poke_out_hlpr
        mov dx, di
        out dx, al
        ret

; =============================================================================
; Centralized syntax checker
; Input: AL = character to expect
; =============================================================================
expect_equals:
        mov al, '='
expect:
        call spaces
        cmp [si], al
        jne JERRSN              ; Jump to Syntax Error if no match
        inc si                  ; Consume the character
sp_r:
	ret

JERRSN:
        mov al, ERR_SN
        jmp do_error

; =============================================================================
; SPACES  skip spaces; preserves AX, BX, CX, DX
; =============================================================================
spaces:
        cmp byte [si], ' '
        jne sp_r	; return
        inc si
        jmp short spaces

; =============================================================================
; EXPR  evaluate expression including relational operators
;
; Inputs  : SI -> expression text
; Outputs : AX = signed 16-bit result; true=0xFFFF false=0x0000
; Clobbers: AX, BX, CX, DX, SI
expr:
        call    expr_bool        ; Left operand -> AX
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
        call    expr_bool        ; Right operand -> AX
        pop     dx              ; Restore mask (DL)
        pop     bx              ; BX = left operand

        cmp     bx, ax          ; Compare left vs right
        
        ; 1. Generate 'Equal' bit (Bit 1 / value 2)
        mov     ax, 2           ; Assume equal (2 bytes)
        jz      .check          ; If ZF=1, we are done with AL=2 (2 bytes)

        ; 2. Generate 'LT' (1) or 'GT' (4) bit
        ; If we are here, ZF=0. Flags from cmp bx,ax are still live.
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
; PREC_ENGINE: Generic Precedence Level Handler
; Handles / * + - & | 
; BX = Table Pointer, DI = Next Level Function Pointer
; =============================================================================
expr_bool:      ; lowest precidence
        mov bx, tab_bool
        mov di, expr_add
        jmp short prec_engine

bool_and:
        and ax, cx
        ret
        
bool_or:
        or ax, cx
        ret
        
expr_add:
        mov bx, tab_add
        mov di, expr1
        jmp short prec_engine

expr1:
        mov bx, tab_mul
        mov di, expr2   ; functions highest precidence
        ; Fall through to engine

prec_engine:
        push bx                 ; [Stack] Save Table
        push di                 ; [Stack] Save Next-Level Func
        call di                 ; Get initial LHS (Left Hand Side) value
.lp:
        mov bp, sp              ; Use BP to peek at the saved BX/DI
        mov di, [bp]            ; DI = Next-Level Func
        mov bx, [bp+2]          ; BX = Table
        
        call spaces
        mov dl, [si]            ; Peek at char in IBUF
.search:
        cmp byte [bx], 0        ; End of table?
        je .done                ; No match, we are finished with this level
        cmp [bx], dl            ; Does char match operator in table?
        je .found
        add bx, 3               ; Next entry (Char + Word)
        jmp .search

.found:
        inc si                  ; Consume operator character
        push ax                 ; [Stack] Save LHS
        push word [bx+1]        ; [Stack] Save Math Handler Address
        call di                 ; Get RHS (Right Hand Side) value -> AX
        mov cx, ax              ; CX = RHS
        pop bx                  ; BX = Math Handler
        pop ax                  ; AX = LHS
        call bx                 ; Execute math: AX = AX (op) CX
        jmp .lp                 ; Repeat for chaining (e.g., 1+2+3)

.done:
        add sp, 4               ; Clean up BX and DI from stack
        ret

math_add:
        add ax, cx
        ret

math_sub:
        sub ax, cx
        ret

math_mul:
        imul cx                 ; AX = AX * CX
        ret

math_div:
        or cx, cx               ; Division by zero?
        je .err
        cwd                     ; Sign-extend AX into DX for IDIV
        idiv cx                 ; AX = quotient, DX = remainder
        ret
.err:
        mov al, ERR_OV          ; Overflow/Div-by-zero error code
        jmp do_error        

math_mod:
        call math_div           ; Perform division
        mov ax, dx              ; Return remainder
        ret

; =============================================================================
; EXPR2  Functions
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

        ; Unified Function Loop
        mov bx, func_tab
e2_func_lp:
        cmp word [bx], 0        ; End of table?
        je e2_nusr              ; No match found
        push bx                 ; Save table pointer
        call kw_match           ; Try to match keyword at [SI]
        pop bx                  ; Restore table pointer
        jnc e2_func_call        ; Match found! Jump to handler
        add bx, 4               ; Next entry (2 bytes pointer + 2 bytes handler)
        jmp e2_func_lp

e2_func_call:
        jmp [bx+2]              ; Indirect jump to the function handler

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

do_abs_func:
        call eat_paren_expr     ; AX = value
        or   ax, ax             ; Set flags (2 bytes)
        jns  .done              ; Jump if positive (2 bytes)
        neg  ax                 ; Negate if negative (2 bytes)
.done:
        ret
        
do_peek_func:
        call peek_in_hlp
        mov al, [bx]
        ret

do_in_func:
        call peek_in_hlp
        mov dx, bx
        in al,dx
        ret

do_usr_func:
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
        call output_space
        call backsp
        jmp ipl_lp
backsp:
        mov al, 0x08
        jmp output	; tail call
        
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
putchar:
output:
%ifdef __YASM_MAJOR__
    push bx
    mov ah, 0x0e
    mov bx, 0x0007
    int 0x10
    pop bx
    ret
%else
    mov ah, al          ; AH = character to send
    mov al, 0           ; Start bit (Line goes low)
    out PORT_A, al
    call bdly
    mov bl, 9           ; 8 Data bits + 1 Stop bit = 9 total iterations
    stc                 ; Carry=1: This will eventually shift out as our stop bit
.out_bit:
    rcr ah, 1           ; Rotate LSB into Carry; previous Carry into AH bit 7
    sbb al, al          ; If CF=1, AL=FF. If CF=0, AL=00.
    and al, TX          ; Mask for the TX pin (e.g., bit 1)
    out PORT_A, al      ; Send the bit
    call bdly
    stc                 ; Ensure Carry is 1 for the next shift-in
    dec bx              ; Use BX (1-byte dec) instead of BL (2-byte dec)
    jnz .out_bit
    ret
%endif
        
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

output_space:
        mov al, ' '
        jmp output
        
; =============================================================================
; INPUT_KEY -> AL
; ROM: bitbang UART RX from 8755 Port A bit1.  Others: BIOS INT 16h.
; =============================================================================
getchar:
input_key:
%ifdef __YASM_MAJOR__
    mov ah, 0x00
    int 0x16
    ret
%else
.ik_wait:
    in al, PORT_A           ; Read port
    test al, RX             ; Check RX line (usually bit 1)
    jnz .ik_wait            ; Loop until start bit (Low)
    
    call bdly               ; Center of start bit (or end, per original logic)
    mov ah, 0x80             ; AH = Marker bit. When it shifts out, we're done.
.ik_bit:
    in al, PORT_A           ; Read data bit
    shr al, 1               ; Move bit 1 into bit 0...
    shr al, 1               ; ...and bit 0 into Carry Flag (CF)
    rcr ah, 1               ; Rotate CF into AH, and AH's LSB into CF
    call bdly               ; Wait for next bit period
    jnc .ik_bit             ; If the marker '1' hasn't fallen into CF, loop
    mov al, ah              ; Move result to return register
    ret

; =============================================================================
; bdly (bit-delay) - Optimized for size
; Clobbers: CX (saved by Marker Bit logic above instead of push/pop)
; =============================================================================
bdly:
    mov cx, BAUD            ; 3 bytes
    loop $                  ; 2 bytes - 17 cycles per iteration on 8088
    ret                     ; 1 byte
%endif

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

; =============================================================================
; LINE EDITOR  
; =============================================================================
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
        push cx                 ; save body+CR length: deline's rep movsb zeroes CX
        call deline             ; delete it; DI unchanged (still insert point)
        pop cx                  ; restore body+CR length
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
	call 	gosub_hlp
	mov     ax, [RUN_NEXT]
        mov     [si], ax        ; push RUN_NEXT onto gosub stack
        mov     [RUN_NEXT], di  ; set PC to GOSUB target line
        ret

gosub_hlp:
	add     bx, bx          ; BX = byte offset (depth * 2)
        mov     si, GOSUB_STK   ; SI -> base of gosub stack
        add     si, bx          ; SI -> this slot
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
	call 	gosub_hlp
        mov     ax, [si]        ; pop return address
        mov     [RUN_NEXT], ax  ; restore PC to line after GOSUB
        ret

gs_underflow:
        mov     al, ERR_RT      ; ?5 return without gosub
        jmp     do_error

; =============================================================================
; DO_REM (Execution Phase) - skips the rest of the line during program run.
; =============================================================================
do_rem:
        mov di, si              ; Make stosb a no-op by writing to read-ptr
        mov ah, 0x0d            ; We want to skip until the end of the line (CR)
		; drop through
; =============================================================================
; SHARED TERMINATOR HELPER (The "Skip vs. Copy" Trick)
; Input:  AH = Terminator char (e.g., '"' or 0x0d)
;         SI = Read pointer, DI = Write pointer
; Note:   If DI = SI, this effectively becomes a "Skip" loop.
; =============================================================================
copy_si_di:
        lodsb                   ; Read char from SI into AL
        stosb                   ; Write AL to DI
        cmp al, 0x0d            ; Always stop at Carriage Return
        je  .done
        cmp al, ah              ; Did we hit our specific terminator?
        jne copy_si_di          ; If not, keep copying
.done:
        ret
; =============================================================================
; TOKENIZE  Convert keyword text -> token bytes in-place in IBUF.
; Input:  SI -> start of body text in IBUF (after line number was parsed)
; Converts keywords to bytes 0x80+ in-place in IBUF.
; Output: body in IBUF replaced with tokenized form; SI unchanged.
; String literals (between quotes) and REM bodies passed through verbatim.
; Token bytes 0x80..0x8F replace keywords; all other chars copied as-is.
; Since tokenized form is always <= original, in-place is safe.
; Clobbers: AX, BX, CX, DX, DI (not SI - caller needs it)
; =============================================================================
tokenize:
        push si                 ; Preserve SI for the caller
        mov di, si              ; Start write-pointer at start of body

tk_lp:
        lodsb                   ; Read next character
        cmp al, 0x0d            ; End of line?
        je  tk_done
        
        ; --- Handle String Literals ---
        cmp al, '"'             ; Start of a "string"?
        jne tk_not_str
        stosb                   ; Write the opening quote
        mov ah, '"'             ; Tell helper to look for the closing quote
        call copy_si_di
        jmp tk_lp               ; Continue tokenizing after the string

tk_not_str:
        ; --- Try Keyword Match ---
        dec si                  ; Back up SI to include the char we just read
        mov bx, tk_kw_tab       ; Start of keyword pointer table
tk_try:
        cmp word [bx], 0        ; End of keyword table?
        je  tk_char             ; No match found: process as literal character
        
        push di                 ; kw_match clobbers DI
        push bx                 ; Save table pointer for index calculation
        call kw_match           ; Compare [SI] against keyword in table
        pop bx
        pop di
        jc  tk_next_kw          ; No match: try next keyword in table

        ; --- Match Found: Emit Token ---
        mov ax, bx
        sub ax, tk_kw_tab       ; Get byte offset into table
        shr ax, 1               ; Convert offset to index (0, 1, 2...)
        add al, TK_PRINT        ; Add base token value (e.g., 0x80)
        stosb                   ; Write token byte to DI

        call spaces             ; Consume trailing spaces (spaces only touches SI)

        cmp al, TK_REM          ; Was the keyword REM?
        jne tk_lp               ; If not, keep tokenizing
        mov ah, 0x0d            ; If REM, copy the rest of the line verbatim
        call copy_si_di
        jmp tk_finish           ; REM is always the end of a line

tk_next_kw:
        add bx, 2               ; Move to next entry in pointer table
        jmp tk_try

tk_char:
        lodsb                   ; No keyword matched: get the char back
        stosb                   ; Write it literally
        jmp tk_lp

tk_done:
        stosb                   ; Write the final Carriage Return
tk_finish:
        pop si                  ; Restore SI to the start of the body
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
        mov     [INS_TMP], di   ; save var_ptr: prec_engine clobbers BP (mov bp,sp)
        call    expect_equals	; parse '='
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
        mov     di, [INS_TMP]   ; reload var_ptr (prec_engine clobbers BP)
        mov     [bx],   di      ; var_ptr
        mov     [bx+2], cx      ; limit
        mov     [bx+4], ax      ; step
        mov     ax, [RUN_NEXT]
        mov     [bx+6], ax      ; loop_ptr = address of line AFTER FOR

        inc     dx
        mov     [FOR_SP], dx    ; bump depth
        ret

df_syn:
        jmp     JERRSN

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
kw_out:	    db 0x4f,0x55, T_T	
kw_to:      db 0x54,T_O                       ; TO   (T,O+0x80)
kw_step:    db 0x53,0x54,0x45,T_P             ; STEP (S,T,E,P+0x80)
; not commands but still want to print in help
kw_then:    db 0x54,0x48,0x45,T_N
kw_chrs:    db 0x43,0x48,0x52,T_DS
kw_peek:    db 0x50,0x45,0x45,T_K
kw_usr:     db 0x55,0x53,T_R
kw_in:	    db 0x49, T_N
kw_tab:	    db 0x54, 0x41, T_B		; TAB
            db 0

; --- Token -> keyword string pointer table (same order as st_tab / TK_xx) ---
; 17 stmt entries + 3 sub-keyword entries
; Stmt (0x80-0x90): PRINT IF GOTO LIST RUN NEW INPUT REM END LET POKE FREE HELP GOSUB RETURN FOR NEXT
; Sub-kw (0x91-0x93): THEN TO STEP (not dispatched by stmt)
tk_kw_tab:
        dw kw_print, kw_if, kw_goto, kw_list, kw_run, kw_new
        dw kw_input, kw_rem, kw_end, kw_let, kw_poke, kw_free
        dw kw_help, kw_gosub, kw_return
        dw kw_for, kw_next, kw_out      ; tokens TK_FOR=0x8F, TK_NEXT=0x90, TK_OUT=0x91
; residuals - sub-keywords: TK_THEN=0x92, TK_TO=0x93, TK_STEP=0x94
then_tab:       dw kw_then
to_tab:         dw kw_to
step_tab:       dw kw_step
        dw 0                    ; sentinel


; =============================================================================
; Strings (bit 7 terminated)
; =============================================================================
str_banner: db "uBASIC 8088 v1.7.1"
CRLF:	    db 0x0d, 0x0a + 0x80	

; --- Statement handlers table in TK_PRINT order --------------------
st_tab:
        dw do_print, do_if, do_goto, do_list, do_run, do_new
        dw do_input, do_rem, do_end, do_let, do_poke, do_free
        dw do_help, do_gosub, do_return, do_for, do_next, do_out

; Additive Level Table
tab_add:
        db '+'
        dw math_add
        db '-'
        dw math_sub
        db 0                    ; Sentinel

; Multiplicative Level Table
tab_mul:
        db '*'
        dw math_mul
        db '/'
        dw math_div
        db '%'
        dw math_mod
        db 0                    ; Sentinel
; Boolean Table
tab_bool:
        db '&'
        dw bool_and
        db '|'
        dw bool_or
        db 0                    ; Sentinel

; ---  Function Table ---
func_tab:
chrs_tab:
	dw kw_chrs, eat_paren_expr	; special as only effective in print
        dw kw_peek, do_peek_func
        dw kw_in,   do_in_func
        dw kw_usr,  do_usr_func
        dw 0                    ; Sentinel
tab_tab: dw kw_tab
ROM_END:

; --- Reset vector at 0xFFF0 -------------------------------------------
; 8086 resets CS=0xFFFF IP=0x0000 -> phys 0xFFFF0.
%ifdef __YASM_MAJOR__
        times 0x7f0-($-start) db 0xff	; pad
%else        
	org 0xfff0
	cld
%endif
reset_vec:
        ; 8755 serial: bit1=RX(in), rest=out; TX idle high
        mov al, 0xFD
        out DDR_A, al
        mov al, TX
        out PORT_A, al
        
%ifdef __YASM_MAJOR__
        jmp start
%else
; We need a FAR JMP to set CS=0xF800 and IP=0x0000.
; JMP FAR 0xF800:0x0000 = EA 00 00 00 F8 (5 bytes)
        db 0xEA                 ; far JMP opcode
        dw 0x0000               ; IP = 0x0000
        dw 0xF800               ; CS = 0xF800 -> start
%endif
	times 2048-($-start) db 0xff	; pad
