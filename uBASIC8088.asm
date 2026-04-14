; =============================================================================
; uBASIC 8088  v1.0.10
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Tiny BASIC for single-segment 8088/8086 systems.
; Target: <=2048 bytes code ROM, 4096 bytes RAM.
;
; Re-engineered from uBASIC 65c02 v17.0 (same features, no tokenizer).
; Credit to Oscar Toledo for his bootBASIC inspiration
;
; Statements : PRINT IF..THEN GOTO LET INPUT REM END RUN LIST NEW POKE FREE HELP
; Expressions: + - * / %  = < > <= >= <>  unary-  CHR$(n) PEEK(addr) USR(addr) A-Z
; Numbers    : signed 16-bit (-32768..32767)
; Multi-stmt : colon separator ':'
; Errors     : ?0 syntax  ?1 undef line  ?2 div/zero  ?3 out of mem  ?4 bad variable
;
; Segment model: CS=DS=ES=SS=0x0000 (single segment, flat).
;   Boot sector (bootsect.asm) loads 5 sectors to 0x7E00 and jumps there.
;   All absolute addresses in RAM (VARS, PROGRAM, etc.) are segment-0 offsets.
;
; I/O: BIOS INT 10h/AH=0Eh display, INT 16h/AH=00h keyboard.
;
; Line store: <linenum_lo> <linenum_hi> <raw ASCII body> <CR>  (no tokenization)
;
; RAM layout (0x1000..0x1FFF, segment 0):
;   0x1000  vars[26]   52 bytes   A-Z word variables (2 bytes each)
;   0x1034  running     1 byte    0=immediate, 1=running
;   0x1035  (pad)       1 byte
;   0x1036  curln       2 bytes   current executing line number (for errors)
;   0x1038  ibuf       64 bytes   input line buffer (max 62 chars + CR)
;   0x1078  prog_end    2 bytes   one-past-last byte of program store
;   0x107A  run_next    2 bytes   next-line pointer for run loop
;   0x107C  ins_tmp     2 bytes   temp word for insline end-marker address
;   0x107E  program     ...       program store (grows toward 0x1E00)
;   0x1E00  PROGRAM_TOP           top of program store / base of stack
;   0x1E00..0x1FFF                stack (512 bytes, SP init = 0x2000)
;
; History:
;   V1.0.10 Size optimization	
;   v1.0.9+ COM_BUILD: %ifdef COM_BUILD assembles as DOS .COM (ORG 0x0100)
;            for cross-checking in 8bitworkshop/DOSBox. I/O unchanged
;            (INT 10h display, INT 16h keyboard). RAM 0x1000-0x1FFF
;            stays well above PSP. Build: tinyasm -f com ... (uses -DCOM_BUILD).
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
                
	; configure Program origin based on Target Platform                
%ifdef __YASM_MAJOR__
    		SECTION .text	; running under 8 bit workshop
%define COM_BUILD	; include demo program
ORIGIN:		equ 0    

%elifdef	COM_BUILD	; testing with freedos
ORIGIN: 	equ 0x0100      ; DOS COM file: PSP at 0x0000, code at 0x0100

%else
ORIGIN:  	equ 0x7E00      ; ROM: boot sector loads to 0x0000:0x7E00
%endif
		org ORIGIN
        
; --- RAM addresses (segment 0 offsets) ---------------------------------------
VARS:           equ 0x1000      ; 52 bytes: A-Z variables (word each)
RUNNING:        equ 0x1034      ; byte:  0=immediate, 1=running
CURLN:          equ 0x1036      ; word:  current line number (error reports)
IBUF:           equ 0x1038      ; 64 bytes: input line buffer (max 62 chars+CR)
RUN_NEXT:       equ 0x107A      ; word:  next-line pointer for run loop
INS_TMP:        equ 0x107C      ; word:  insline end-marker temp
PROGRAM:        equ 0x107E      ; program store starts here
PROG_END:       equ 0x1078      ; word:  one past last program byte
PROGRAM_TOP:    equ 0x1E00      ; top of program store / base of stack
STACK_TOP:      equ 0x2000      ; initial SP

; --- Error codes -------------------------------------------------------------
ERR_SN:         equ "0" ; 0
ERR_UL:         equ "1" ; 1
ERR_OV:         equ "2" ; 
ERR_OM:         equ "3" ;
ERR_UK:         equ "4" ;

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

; =============================================================================
; INIT  cold start; entered at 0x0000:0x7E00 by boot sector
; Clobbers: everything
; =============================================================================
start:
        cld
        ; Ensure CS=DS=ES=SS=0 (boot sector sets them but be defensive)
        push cs
        pop ds
        push cs
        pop es
        push cs
        pop ss
        mov sp, STACK_TOP

        ; Zero all RAM control area: VARS through end of Program
        mov di, VARS
        xor ax, ax
        mov cx, 63	; this should include program RAM if no demo
	rep stosw	; clear memory
        
%ifndef COM_BUILD
        call do_new
%else
        mov word [PROG_END], PROGRAM+(SHOWCASE_END-SHOWCASE_DATA)-2
%endif

	mov si, str_banner
        call dp_str
        call do_free
        ; fall through into main_loop

; =============================================================================
; MAIN_LOOP  prompt / dispatch loop
; =============================================================================
main_loop:
        mov sp, STACK_TOP
        mov byte [RUNNING], 0

	mov al, '>'
        call output
        
        call input_line         ; read line; SI -> IBUF

        call peek_line     	; blank line?
        je main_loop

        call input_number       ; parse optional line number -> AX
        or ax, ax
        jne ml_numbered
        call stmt_line          ; CALL - correct return address
        jmp main_loop
ml_numbered:
        call editln             ; numbered line: store/edit in program
        jmp main_loop

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
        ret

; =============================================================================
; STMT  execute one statement from SI
; =============================================================================
stmt:
        call peek_line
        je stmt_ret
        mov bx, st_tab
stmt_lp:
        mov ax, [bx]
        or ax, ax
        je stmt_let             ; sentinel -> implicit LET/assignment
        call kw_match
        jnc stmt_call
        add bx, 4
        jmp stmt_lp
stmt_call:
        call [bx+2]             ; indirect call to handler
stmt_ret:
        ret
        
stmt_let:
; =============================================================================
; DO_LET  [LET] <var> = <expr>
; =============================================================================
do_let:
        call spaces
        mov al, [si]
        call uc_al
        cmp al, 'A'
        jb JERRUK
        cmp al, 'Z'
        ja JERRUK
        call get_var_addr
        push di
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
        push si
        call spaces
        mov di, [bx]
kw_lp:
        mov al, [di]
        inc di
        mov ah, al
        and al, 0x7f
        call uc_al
        mov dl, [si]
        inc si
 uc_dl:
        cmp dl, 'a'
        jb uc_dl_r
        cmp dl, 'z'
        ja uc_dl_r
        and dl, 0xdf
uc_dl_r:
        cmp al, dl
        jne kw_fail
        test ah, 0x80           ; last char?
        jz kw_lp
        ; verify word boundary
        mov al, [si]
        call uc_al
        cmp al, '_'
        je kw_fail
        cmp al, 'A'
        jb kw_ok
        cmp al, 'Z'
        jbe kw_fail
        cmp al, '0'
        jb kw_ok
        cmp al, '9'
        jbe kw_fail
kw_ok:
        pop ax
        clc
        ret
kw_fail:
        pop si
        stc
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
do_if_false:
uc_al_r:
dg_ret:
        ret

; =============================================================================
; DO_IF  IF <expr> [THEN] <stmt>
; =============================================================================
do_if:
        call expr
        or ax, ax
        je do_if_false
        mov bx,then_tab
        call kw_match
        jmp stmt

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
        jmp     run_loop
        
; =============================================================================
; DO_NEW
; =============================================================================
do_new:
	xor ax,ax
	mov word [PROGRAM], ax
        mov word [PROG_END], PROGRAM
        ; drop through        

; =============================================================================
; DO_END  END statement - stops program execution
; =============================================================================
do_end:
	xor ax,ax
	mov di, [RUN_NEXT]
        mov word [di],ax
run_end:
        mov byte [RUNNING], al
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
        call output
        cmp al, 0x0d        ; Check for CR
        jne dl_body

        mov al, 0x0a
        call output

        call next_line_ptr  ; Assuming this updates DI to the next record
        jmp dl_lp

; =============================================================================
; DO_REM  skip rest of line
; =============================================================================
do_rem:
        lodsb           ; AL = [SI], then SI++
        cmp al, 0x0d    ; Was it the CR?
        jne do_rem      ; If not, keep going
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
        jmp dp_str		
        
dp_str_eol:
        dec si
dp_nl:
        call new_line
dp_ret:
        ret
dp_expr:
        mov bx, chrs_tab
        call kw_match
        jc dp_num
        call eat_paren_expr
        call output
        jmp dp_after
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
        jmp dp_top

; =============================================================================
; DO_INPUT  INPUT <var>
; =============================================================================
do_input:
        ; Validate variable letter before proceeding
        call spaces
        mov al, [si]
        call uc_al
        cmp al, 'A'
        jb di_err
        cmp al, 'Z'
        ja di_err
        call get_var_addr       ; DI = &var; SI advanced
        push di
        mov al, '?'
        call output
        mov al, ' '
        call output
        call input_line
        call expr
        pop di
        mov [di], ax
        ret
di_err:
        mov al, ERR_UK
        jmp do_error

; =============================================================================
; DO_POKE  POKE <addr>, <val> - need to finagle addresses above 32768
; =============================================================================
do_poke:
        call expr
        mov di, ax
        call spaces
        cmp byte [si], ','
        jne JERRSN
        inc si
        call expr
        mov [di], al
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
;
; FIX v1.0.2: restored missing 'cmp al,>' before 'je rel_gt'.
; FIX v1.0.5: push/pop AX around peek (BX unsafe - clobbered by expr2).
;             rel_xx pop left before rel_setup. rel_setup reverted to original.
; =============================================================================
expr:
        call expr_add           ; left operand -> AX
        push ax                 ; save left (BX clobbered by expr2 kw_match loads)
        call spaces
        mov al, [si]            ; peek for relational operator
        cmp al, '='
        je rel_eq
        cmp al, '<'
        je rel_lt
        cmp al, '>'
        je rel_gt
        pop ax                  ; no relational op: restore result
        ret

; rel_setup: in AX=left; out AX=right BX=left
rel_setup:
        push ax                 ; save left
        call expr_add           ; right -> AX
        pop bx                  ; BX = left
        ret

rel_eq:
        pop ax                  ; left from expr's push
        inc si
        call rel_setup
        cmp bx, ax
        je rel_t
        jmp rel_f

rel_lt:
        inc si
        call spaces
        mov al, [si]
        cmp al, '>'
        je rel_ne
        cmp al, '='
        je rel_le
        pop ax                  ; left operand from expr's push
        call rel_setup          ; plain <: BX=left AX=right
        cmp bx, ax
        jl rel_t
        jmp rel_f
rel_ne:
        pop ax                  ; left
        inc si
        call rel_setup
        cmp bx, ax
        jne rel_t
        jmp rel_f
rel_le:
        pop ax                  ; left
        inc si
        call rel_setup
        cmp bx, ax
        jle rel_t
        jmp rel_f

rel_gt:
        inc si
        call spaces
        mov al, [si]
        cmp al, '='
        je rel_ge
        pop ax                  ; left operand from expr's push
        call rel_setup          ; plain >: BX=left AX=right
        cmp bx, ax
        jg rel_t
        jmp rel_f
rel_ge:
        pop ax                  ; left
        inc si
        call rel_setup
        cmp bx, ax
        jge rel_t
        jmp rel_f

rel_t:  mov ax, 0xffff
        ret
rel_f:  xor ax, ax
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
ea_ret:
        ret

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
e1_ret:
        ret

; =============================================================================
; EXPR2  atom level
; =============================================================================
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
        call eat_paren_expr
        ret
e2_nchrs:
        ; PEEK
        mov bx, peek_tab
        call kw_match
        jc e2_npeek
        call eat_paren_expr
        mov bx, ax
        xor ah, ah
        mov al, [bx]
        ret
e2_npeek:
        ; USR
        mov bx, usr_tab
        call kw_match
        jc e2_nusr
        call eat_paren_expr
        call ax
        ret
e2_nusr:
        ; Reload AL from [si] - kw_match may have clobbered it
        mov al, [si]
        ; decimal literal?
        cmp al, '0'
        jb e2_var
        cmp al, '9'
        ja e2_var
        call input_number
        ret
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
        
e2_neg:
        inc si
        call expr2
        neg ax
        ret
e2_pos:
        inc si
        jmp expr2


epe_err:
        mov al, ERR_SN
        jmp do_error


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
        jne sp_r
        inc si
        jmp spaces

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
        jmp inm_lp
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
        mov al, 0x08
        call output
        mov al, ' '
        call output
        mov al, 0x08
        call output
        jmp ipl_lp
ipl_nbs:
        cmp al, 0x0d
        je ipl_cr
        cmp cx, 62
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
; OUTPUT  AL -> BIOS INT 10h TTY
; =============================================================================
output:
        push bx
        mov ah, 0x0e
        mov bx, 0x0007
        int 0x10
        pop bx
        ret
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
; INPUT_KEY  -> AL  (BIOS INT 16h)
; =============================================================================
input_key:
        mov ah, 0x00
        int 0x16
        ret

; =============================================================================
; LINE EDITOR  (from uBASIC8088 v2.1.0)
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
	ret

; EDITLN  AX=linenum, SI->body+CR
editln:
        push ax
        call spaces
        pop dx                  ; DX = line number
        mov bx, si              ; BX = body pointer
        mov cx, 1
el_len:
        cmp byte [si], 0x0d
        je el_ldone
        inc si
        inc cx
        jmp el_len
el_ldone:
        push bx                 ; save body ptr: find_line/deline clobber BX
        mov ax, dx
        call find_line          ; walk_lines clobbers BX
        cmp [di], dx
        jne el_noex
        call deline             ; also clobbers BX
el_noex:
        pop bx                  ; restore body pointer
        cmp byte [bx], 0x0d     ; empty body = delete only
        je el_done
        mov si, bx              ; SI = body pointer
        mov ax, dx              ; AX = line number
        add cx, 2               ; +2 for linenum word
	; drop through
        
; INSLINE  AX=linenum, DI=insert-pt, SI->body, CX=total-size
insline:
        push ax                 ; [BP+6] line number
        push cx                 ; [BP+4] total size
        push si                 ; [BP+2] body pointer
        push di                 ; [BP+0] insertion point

        call find_program_end
        mov [INS_TMP], di

        mov bp, sp
        mov cx, [bp+4]
        mov ax, [bp+6]

        mov bx, [INS_TMP]
        add bx, 2
        add bx, cx
        cmp bx, PROGRAM_TOP
        ja ins_oom

        mov di, [INS_TMP]
        mov si, di
        inc si
        add di, cx
        inc di
        mov bx, [INS_TMP]
        add bx, 2
        sub bx, [bp+0]
        push bx
        mov cx, bx
        std
        rep movsb
        cld

        pop bx                  ; discard move count
        mov ax, [bp+6]
        mov si, [bp+2]
        mov cx, [bp+4]
        add sp, 8               ; discard 4 original saves

        call find_line          ; DI -> insertion slot
        mov [di], ax
        add di, 2
ins_copy:
        lodsb
        stosb
        cmp al, 0x0d
        jne ins_copy

        add [PROG_END], cx
        mov word [di], 0
        ret

ins_oom:
        add sp, 8
        mov ax, ERR_OM
        jmp do_error

; DELINE  delete line at DI
deline:
        push di
        call next_line_ptr
        mov si, di
        call find_program_end
        mov cx, di
        add cx, 2
        pop di
        mov bx, si
        sub bx, di
        sub cx, si
        rep movsb
        sub [PROG_END], bx
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
; not commands but still want to print in help
kw_then:    db 0x54,0x48,0x45,T_N
kw_chrs:    db 0x43,0x48,0x52,T_DS
kw_peek:    db 0x50,0x45,0x45,T_K
kw_usr:     db 0x55,0x53,T_R

            db 0

; =============================================================================
; Strings (bit 7 terminated)
; =============================================================================
str_banner: db "uBASIC 8088 v1.0.10"
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
        dw 0
        ; non commands but still match
peek_tab:       dw kw_peek
usr_tab:        dw kw_usr
then_tab:       dw kw_then
chrs_tab:       dw kw_chrs

ROM_END:	

; =============================================================================
; Pre-loaded showcase + Mandelbrot program (ROM build only, %ifndef COM_BUILD)
; Copied to RAM at startup. Type RUN to execute, NEW to clear.
; Lines 10-160: feature demos.  Lines 170-400: Mandelbrot renderer.
; Fixed-point arithmetic (scale 1/64), 16 iterations, ASCII density display.
; =============================================================================
%ifdef COM_BUILD
	times PROGRAM - ($-$$) nop
SHOWCASE_DATA:
        db 0x0A,0x00,"REM uBASIC 8088 - SHOWCASE",0x0D  ; 10 REM uBASIC 8088 - SHOWCASE
        db 0x14,0x00,"PRINT ",0x22,"-- uBASIC 8088 v1.0.9 --",0x22,0x0D  ; 20 PRINT "-- uBASIC 8088 v1.0.9 --"
        db 0x1E,0x00,"PRINT ",0x22,"--- ARITHMETIC ---",0x22,0x0D  ; 30 PRINT "--- ARITHMETIC ---"
        db 0x28,0x00,"PRINT ",0x22,"3+4=",0x22,";3+4;",0x22,"  6*7=",0x22,";6*7",0x0D  ; 40 PRINT "3+4=";3+4;"  6*7=";6*7
        db 0x32,0x00,"PRINT ",0x22,"20/4=",0x22,";20/4;",0x22,"  17%5=",0x22,";17%5",0x0D  ; 50 PRINT "20/4=";20/4;"  17%5=";17%5
        db 0x3C,0x00,"PRINT ",0x22,"--- COMPARISONS ---",0x22,0x0D  ; 60 PRINT "--- COMPARISONS ---"
        db 0x46,0x00,"IF 5>3 THEN PRINT ",0x22,"5>3 ok",0x22,0x0D  ; 70 IF 5>3 THEN PRINT "5>3 ok"
        db 0x50,0x00,"IF 3<5 THEN PRINT ",0x22,"3<5 ok",0x22,0x0D  ; 80 IF 3<5 THEN PRINT "3<5 ok"
        db 0x5A,0x00,"IF 3>=3 THEN PRINT ",0x22,"3>=3 ok",0x22,0x0D  ; 90 IF 3>=3 THEN PRINT "3>=3 ok"
        db 0x64,0x00,"IF 4<>3 THEN PRINT ",0x22,"4<>3 ok",0x22,0x0D  ; 100 IF 4<>3 THEN PRINT "4<>3 ok"
        db 0x6E,0x00,"PRINT ",0x22,"--- LOOP ---",0x22,0x0D  ; 110 PRINT "--- LOOP ---"
        db 0x78,0x00,"I=1",0x0D  ; 120 I=1
        db 0x82,0x00,"IF I>5 THEN GOTO 160",0x0D  ; 130 IF I>5 THEN GOTO 160
        db 0x8C,0x00,"PRINT I;",0x0D  ; 140 PRINT I;
        db 0x96,0x00,"I=I+1:GOTO 130",0x0D  ; 150 I=I+1:GOTO 130
        db 0xA0,0x00,"PRINT ",0x22,0x22,0x0D  ; 160 PRINT ""
        db 0xAA,0x00,"PRINT ",0x22,"--- MANDELBROT ---",0x22,0x0D  ; 170 PRINT "--- MANDELBROT ---"
        db 0xB4,0x00,"I=-64",0x0D  ; 180 I=-64
        db 0xBE,0x00,"IF I>56 THEN GOTO 400",0x0D  ; 190 IF I>56 THEN GOTO 400
        db 0xC8,0x00,"D=I",0x0D  ; 200 D=I
        db 0xD2,0x00,"C=-128",0x0D  ; 210 C=-128
        db 0xDC,0x00,"IF C>16 THEN GOTO 370",0x0D  ; 220 IF C>16 THEN GOTO 370
        db 0xE6,0x00,"A=C:B=D:E=0:N=1",0x0D  ; 230 A=C:B=D:E=0:N=1
        db 0xF0,0x00,"IF N>16 THEN GOTO 310",0x0D  ; 240 IF N>16 THEN GOTO 310
        db 0xFA,0x00,"T=A*A/64-B*B/64+C",0x0D  ; 250 T=A*A/64-B*B/64+C
        db 0x04,0x01,"B=2*A*B/64+D:A=T",0x0D  ; 260 B=2*A*B/64+D:A=T
        db 0x0E,0x01,"IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N",0x0D  ; 270 IF A*A/64+B*B/64>256 THEN IF E=0 THEN E=N
        db 0x18,0x01,"N=N+1:IF N<=16 THEN GOTO 240",0x0D  ; 280 N=N+1:IF N<=16 THEN GOTO 240
        db 0x36,0x01,"IF E>0 THEN PRINT CHR$(E+32);",0x0D  ; 310 IF E>0 THEN PRINT CHR$(E+32);
        db 0x40,0x01,"IF E=0 THEN PRINT CHR$(32);",0x0D  ; 320 IF E=0 THEN PRINT CHR$(32);
        db 0x4A,0x01,"C=C+4",0x0D  ; 330 C=C+4
        db 0x54,0x01,"GOTO 220",0x0D  ; 340 GOTO 220
        db 0x72,0x01,"PRINT ",0x22,0x22,0x0D  ; 370 PRINT ""
        db 0x7C,0x01,"I=I+6",0x0D  ; 380 I=I+6
        db 0x86,0x01,"GOTO 190",0x0D  ; 390 GOTO 190
        db 0x90,0x01,"END",0x0D  ; 400 END
        dw 0
SHOWCASE_END:
%endif
