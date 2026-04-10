; =============================================================================
; uBASIC 8088  v1.0.1
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Tiny BASIC for single-segment 8088/8086 systems.
; Target: <=2048 bytes code ROM, 4096 bytes RAM.
; Re-engineered from uBASIC 65c02 v17.0 (same features, no tokenizer).
;
; Statements : PRINT IF..THEN GOTO LET INPUT REM END RUN LIST NEW POKE FREE HELP
; Expressions: + - * / %  = < > <= >= <>  unary-  CHR$(n) PEEK(addr) USR(addr) A-Z
; Numbers    : signed 16-bit (-32768..32767)
; Multi-stmt : colon separator ':'
; Errors     : ?0 syntax  ?1 undef line  ?2 div/zero  ?3 out of mem  ?4 bad variable
;
; Segment model: CS=DS=ES=SS (single segment).
;   Works as DOS .COM file and as bare-metal EPROM (no segment arithmetic needed).
; I/O: BIOS INT 10h display, INT 16h keyboard. Replace output/input_key for UART.
;
; Line store format: <linenum_lo> <linenum_hi> <raw ASCII body> <CR>
; No tokenisation - body stored and re-parsed every execution.
;
; RAM layout (flat single segment, 0x1000..0x1FFF):
;   0x1000  vars[26]   52 bytes  A-Z word variables
;   0x1034  running     1 byte   0=immediate 1=running
;   0x1035  (pad)       1 byte
;   0x1036  curln       2 bytes  current executing line# (for error reports)
;   0x1038  ibuf       34 bytes  input line buffer
;   0x105A  prog_end    2 bytes  one-past-last byte of program store
;   0x105C  run_next    2 bytes  next-line DI for run loop
;   0x105E  ins_tmp     2 bytes  temp for insline end-marker address
;   0x1060  program     ...      program store (grows toward 0x1E00)
;   0x1E00  PROGRAM_TOP          top of usable program store
;   0x1E00..0x1FFF               stack (512 bytes, SP init = 0x2000)
;
; History:
;   v1.0.1 (2026-04-09)  Minor tweaks to print string routine
;   v1.0.0 (2026-04-09)  First release. Clean 8088 port from uBASIC 65c02 v17.0.
;                         Line editor logic from uBASIC8088 v2.1.0.
; =============================================================================

        cpu 8086
        org 0x0100              ; COM file origin. Change to 0x0 for EPROM. 
                                ; Comment out for 8 bit workshop
	; section .TEXT		; Uncomment to test in 8bitworkshop
        
; --- RAM addresses -----------------------------------------------------------
VARS:           equ 0x1000      ; 52 bytes: A-Z variables (word each)
RUNNING:        equ 0x1034      ; byte:  0=immediate, 1=running
CURLN:          equ 0x1036      ; word:  current line number (error reports)
IBUF:           equ 0x1038      ; 34 bytes: input line buffer
PROG_END:       equ 0x105A      ; word:  one past last program byte
RUN_NEXT:       equ 0x105C      ; word:  next-line pointer for run loop
INS_TMP:        equ 0x105E      ; word:  insline end-marker temp
PROGRAM:        equ 0x1060      ; program store starts here
PROGRAM_TOP:    equ 0x1E00      ; top of program store / base of stack
STACK_TOP:      equ 0x2000      ; initial SP

; --- Error codes -------------------------------------------------------------
ERR_SN:         equ 0           ; syntax / bad expression
ERR_UL:         equ 1           ; undefined line number
ERR_OV:         equ 2           ; division or modulo by zero
ERR_OM:         equ 3           ; out of memory
ERR_UK:         equ 4           ; bad variable name

; --- Keyword last-byte constants: ASCII value | 0x80 -------------------------
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
; INIT  cold start
; Clobbers: everything
; =============================================================================
start:
        cld
        mov sp, STACK_TOP
	push cs			; required for 8bitworkshop, minimal mode ROM
        pop ds			; and load from Bootsector
        mov di, VARS            ; zero A-Z variables (26 words = 52 bytes)
        xor ax, ax
        mov cx, 26
        rep stosw               ; also zeroes RUNNING,CURLN,ibuf... to 0x1060

        mov word [PROG_END], PROGRAM
        mov word [PROGRAM], 0   ; empty program end-marker

        mov si, str_banner	; signon banner
        call dp_str
        call do_free            ; print free bytes on startup
        ; fall through into main_loop

; =============================================================================
; MAIN_LOOP  prompt / dispatch loop
; =============================================================================
main_loop:
        mov sp, STACK_TOP
        mov byte [RUNNING], 0

        mov al, '>'
        call output
        mov al, ' '
        call output
        call input_line         ; read line; SI -> IBUF

        call spaces
        cmp byte [si], 0x0d     ; blank line?
        je main_loop

        call input_number       ; parse optional line number -> AX
        or ax, ax
        je stmt_line            ; no number: direct command

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
        add al, '0'
        call output             ; print "?N"
        cmp byte [RUNNING], 0
        je do_error_nl
        mov si, str_in          ; " IN "
        call dp_str
;      call print_z
        mov ax, [CURLN]
        call output_number
do_error_nl:
        call new_line
        jmp main_loop

; =============================================================================
; STMT_LINE  execute ':'-separated statements on line at SI
; Clobbers: AX, BX, CX, DX, SI, DI
; =============================================================================
stmt_line:
        call stmt
sl_chk:
        call spaces
        cmp byte [si], ':'
        jne sl_ret
        inc si
        jmp stmt_line
sl_ret:
        ret

; =============================================================================
; STMT  execute one statement from SI
; Clobbers: AX, BX, CX, DX, SI, DI
; =============================================================================
stmt:
        call spaces
        cmp byte [si], 0x0d
        je stmt_ret
        mov bx, st_tab          ; walk dispatch table
stmt_lp:
        mov ax, [bx]
        or ax, ax
        je stmt_let             ; sentinel: fall through to LET/assignment
        call kw_match
        jnc stmt_call
        add bx, 4               ; next entry (kw_ptr word + handler word)
        jmp stmt_lp
stmt_call:
        call [bx+2]             ; indirect call to handler
        ret
stmt_let:
        jmp do_let
stmt_ret:
        ret

; --- Statement dispatch table: dw kw_ptr, dw handler; ends with dw 0 --------
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
        dw 0                    ; sentinel

; =============================================================================
; KW_MATCH  case-insensitive keyword match at [SI]
;
; Inputs  : BX -> table entry, word 0 = keyword string ptr
;           SI -> input text (leading spaces skipped internally)
; Outputs : CF=0 matched, SI advanced past keyword
;           CF=1 no match, SI unchanged
; Clobbers: AX, DI, DL
;
; Keywords use bit-7 termination on their last byte.
; After matching, next source char must not be A-Z, a-z, 0-9, or _.
; =============================================================================
kw_match:
        push si
        call spaces             ; skip leading spaces in input
        mov di, [bx]            ; DI -> keyword string
kw_lp:
        mov al, [di]            ; keyword byte
        inc di
        mov ah, al              ; save for bit-7 test later
        and al, 0x7f            ; strip bit 7 to get true ASCII
        call uc_al              ; uppercase (keyword already UC, defensive)
        mov dl, [si]            ; source byte
        inc si
        call uc_dl              ; uppercase source
        cmp al, dl
        jne kw_fail
        test ah, 0x80           ; last char of keyword?
        jz kw_lp
        ; Full match -- verify word boundary in source
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
        pop ax                  ; discard saved SI
        clc
        ret
kw_fail:
        pop si                  ; restore SI
        stc
        ret

; =============================================================================
; UC_AL / UC_DL  uppercase register A or DL
; Clobbers: only the named register
; =============================================================================
uc_al:
        cmp al, 'a'
        jb uc_al_r
        cmp al, 'z'
        ja uc_al_r
        and al, 0xdf
uc_al_r:
        ret

uc_dl:
        cmp dl, 'a'
        jb uc_dl_r
        cmp dl, 'z'
        ja uc_dl_r
        and dl, 0xdf
uc_dl_r:
        ret

; =============================================================================
; DO_IF  IF <expr> [THEN] <stmt>
; False: returns (stmt_line stops at CR naturally)
; True:  consumes optional THEN, falls into stmt
; =============================================================================
do_if:
        call expr
        or ax, ax
        je do_if_false
        mov bx, then_tab
        call kw_match           ; consume optional THEN; CF=1 = absent (SI restored)
        jmp stmt
do_if_false:
        ret

then_tab:       dw kw_then, 0   ; single-entry table for THEN match

; =============================================================================
; DO_GOTO  GOTO <expr>
; =============================================================================
do_goto:
        call expr               ; AX = target line number
        push ax
        call find_line          ; DI -> first line >= AX
        pop bx
        cmp [di], bx
        je dg_found
        mov ax, ERR_UL
        jmp do_error
dg_found:
        mov [RUN_NEXT], di      ; update next-line pointer
        cmp byte [RUNNING], 0
        jne dg_ret
        mov byte [RUNNING], 1   ; start running if in immediate mode
        jmp run_loop
dg_ret:
        ret

; =============================================================================
; DO_RUN  RUN: execute from first line
; =============================================================================
do_run:
        mov di, PROGRAM
        mov byte [RUNNING], 1
        mov [RUN_NEXT], di
run_loop:
        mov di, [RUN_NEXT]
        cmp word [di], 0        ; end marker?
        je run_end
        mov ax, [di]
        mov [CURLN], ax         ; save line# for error reports
        push di
        call next_line_ptr      ; DI -> next line
        mov [RUN_NEXT], di      ; default advance
        pop di
        lea si, [di+2]          ; SI -> body of current line
        call stmt_line
        jmp run_loop
run_end:
        mov byte [RUNNING], 0
        ret

; =============================================================================
; DO_LIST  LIST: print all program lines
; =============================================================================
do_list:
        mov di, PROGRAM
dl_lp:
        cmp word [di], 0
        je dl_done
        mov ax, [di]
        call output_number
        mov al, ' '
        call output
        lea si, [di+2]          ; SI -> body
dl_body:
        lodsb
        call output
        cmp al, 0x0d
        jne dl_body
        mov al, 0x0a
        call output
        call next_line_ptr      ; DI -> next line (updates DI in place)
        jmp dl_lp
dl_done:
        ret

; =============================================================================
; DO_NEW  NEW: clear program store
; =============================================================================
do_new:
        mov word [PROGRAM], 0
        mov word [PROG_END], PROGRAM
        ret

; =============================================================================
; DO_END  END: stop execution
; =============================================================================
do_end:
        mov byte [RUNNING], 0
        ; Force run_loop exit: write end-marker where run_next points
        mov di, [RUN_NEXT]
        mov word [di], 0
        ret

; =============================================================================
; DO_REM  REM: skip rest of line
; =============================================================================
do_rem:
        cmp byte [si], 0x0d
        je do_rem_r
        inc si
        jmp do_rem
do_rem_r:
        ret

; =============================================================================
; DO_PRINT  PRINT [item [; item] ...]
; dp_str - si is ptr to string. Like Print_z but terminator is 0x0d for automatic
; newline or 0x22,";" to supress newline
; Items: "string literal", CHR$(n), numeric expression.
; ';' between items suppresses inter-item space.
; Trailing ';' suppresses final CR+LF.
; Clobbers: AX, BX, CX, DX, SI, DI
; =============================================================================
do_print:
dp_top:
        call spaces
        cmp byte [si], 0x0d
        je dp_nl
        cmp byte [si], '"'
        jne dp_expr
        inc si                  ; consume opening '"'
dp_str:
        lodsb
        cmp al, '"'
        je dp_after
        cmp al, 0x0d            ; unterminated string: treat as end
        je dp_str_eol
        call output
        jmp dp_str
dp_str_eol:
        dec si                  ; put CR back for caller
        jmp dp_nl

dp_expr:
        ; CHR$(n) or numeric expression?
        mov bx, chrs_tab
        call kw_match
        jc dp_num               ; not CHR$: evaluate as number
        call eat_paren_expr     ; CHR$(expr) -> AX
        call output
        jmp dp_after
dp_num:
        call expr
        call output_number
dp_after:
        call spaces
        cmp byte [si], ';'
        jne dp_nl
        inc si                  ; consume ';'
        call spaces
        cmp byte [si], 0x0d
        je dp_ret               ; trailing ';': suppress CR+LF
        jmp dp_top
dp_nl:
        call new_line
dp_ret:
        ret

chrs_tab:       dw kw_chrs, 0

; =============================================================================
; DO_INPUT  INPUT <var>
; =============================================================================
do_input:
        call get_var_addr       ; DI = &var; SI advanced past letter
        push di
        mov al, '?'
        call output
        mov al, ' '
        call output
        call input_line         ; read expression text; SI -> IBUF
        call expr               ; AX = value
        pop di
        mov [di], ax
        ret

; =============================================================================
; DO_LET  [LET] <var> = <expr>  (LET keyword optional)
; =============================================================================
do_let:
        call spaces
        mov al, [si]
        call uc_al
        cmp al, 'A'
        jb dl_err
        cmp al, 'Z'
        ja dl_err
        call get_var_addr       ; DI = &var; SI advanced
        push di
        call spaces
        cmp byte [si], '='
        jne dl_err2
        inc si
        call expr               ; AX = value
        pop di
        mov [di], ax
        ret
dl_err2:
        pop di
dl_err:
        mov ax, ERR_UK
        jmp do_error

; =============================================================================
; DO_POKE  POKE <addr>, <val>
; =============================================================================
do_poke:
        call expr               ; AX = address
        mov di, ax
        call spaces
        cmp byte [si], ','
        jne dpk_err
        inc si
        call expr               ; AX = value
        mov [di], al
        ret
dpk_err:
        mov ax, ERR_SN
        jmp do_error

; =============================================================================
; DO_FREE  FREE: print free program-store bytes
; =============================================================================
do_free:
        mov ax, PROGRAM_TOP
        sub ax, [PROG_END]
        call output_number
        mov al, ' '
        call output
        mov si, str_free
        ;call print_z
        call dp_str
        ret

; =============================================================================
; DO_HELP  HELP: print all keywords
;
; Walks kw_tab_start, printing each char (stripping bit 7 on last byte);
; separates keywords with space; ends with CR+LF.
; The 0x00 sentinel byte ends the walk.
; =============================================================================
do_help:
        mov si, kw_tab_start
dh_lp:
        lodsb
        or al, al
        je dh_done              ; 0x00 sentinel
        push ax
        and al, 0x7f
        call output
        pop ax
        test al, 0x80           ; last char of this keyword?
        jz dh_lp
        mov al, ' '             ; space between keywords
        call output
        jmp dh_lp
dh_done:
        call new_line
        ret

; =============================================================================
; EXPR  evaluate expression, including relational operators
;
; Inputs  : SI -> expression text
; Outputs : AX = signed 16-bit result; true=0xFFFF false=0x0000; SI advanced
; Clobbers: AX, BX, CX, DX, SI
;
; Precedence (low to high): relational  <  additive  <  mul/div/mod  <  atom
; =============================================================================
expr:
        call expr_add           ; left -> AX
        call spaces
        mov al, [si]
        cmp al, '='
        je rel_eq
        cmp al, '<'
        je rel_lt
        je rel_gt
        ret                     ; no relational op

; rel_setup: first op char already consumed by caller.
; In: AX=left; Out: AX=right BX=left
rel_setup:
        push ax
        call expr_add
        pop bx
        ret

rel_eq:
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
        call rel_setup          ; plain <
        cmp bx, ax
        jl rel_t
        jmp rel_f
rel_ne:
        inc si
        call rel_setup
        cmp bx, ax
        jne rel_t
        jmp rel_f
rel_le:
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
        call rel_setup          ; plain >
        cmp bx, ax
        jg rel_t
        jmp rel_f
rel_ge:
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
        push ax                 ; save left
        inc si
        call expr1              ; right -> AX
        pop bx                  ; BX = left
        cmp dl, '-'
        jne ea_add
        neg ax                  ; subtract: negate right then add
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
        push dx                 ; save operator char
        push ax                 ; save left operand
        call expr2              ; right -> AX
        pop cx                  ; CX = left operand
        pop dx                  ; DL = operator

        cmp dl, '*'
        jne e1_div
        xchg ax, cx             ; AX = left, CX = right
        imul cx                 ; DX:AX = left*right; keep AX (low 16 bits)
        jmp e1_lp

e1_div:
        ; CX=left, AX=right (divisor)
        or ax, ax
        je e1_zero
        xchg ax, cx             ; AX = left (dividend), CX = right (divisor)
        cwd                     ; sign-extend AX -> DX:AX
        idiv cx                 ; AX = quotient, DX = remainder
        cmp dl, '%'
        jne e1_lp
        mov ax, dx              ; mod: use remainder
        jmp e1_lp

e1_zero:
        mov ax, ERR_OV
        jmp do_error
e1_ret:
        ret

; =============================================================================
; EXPR2  atom level: parens, unary, CHR$, PEEK, USR, number, variable
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

        ; CHR$(n)
        mov bx, chrs_tab
        call kw_match
        jc e2_nchrs
        call eat_paren_expr     ; -> AX (character code)
        ret

e2_nchrs:
        ; PEEK(addr)
        mov bx, peek_tab
        call kw_match
        jc e2_npeek
        call eat_paren_expr     ; AX = address
        mov bx, ax
        xor ah, ah
        mov al, [bx]            ; read byte at address
        ret

e2_npeek:
        ; USR(addr)
        mov bx, usr_tab
        call kw_match
        jc e2_nusr
        call eat_paren_expr     ; AX = address
        call ax                 ; call user routine; return value in AX
        ret

e2_nusr:
        ; Decimal literal?
        cmp al, '0'
        jb e2_var
        cmp al, '9'
        ja e2_var
        call input_number       ; -> AX, SI past digits
        ret

e2_var:
        ; Variable A-Z (case-insensitive)?
        call uc_al
        cmp al, 'A'
        jb e2_bad
        cmp al, 'Z'
        ja e2_bad
        call get_var_addr       ; DI = &var; SI advanced
        mov ax, [di]
        ret

e2_bad:
        xor ax, ax              ; unrecognised atom: return 0
        ret

e2_par:
        inc si                  ; consume '('
        call expr
        call spaces
        cmp byte [si], ')'
        jne e2_perr
        inc si
        ret
e2_perr:
        mov ax, ERR_SN
        jmp do_error

e2_neg:
        inc si
        call expr2
        neg ax
        ret

e2_pos:
        inc si
        jmp expr2               ; tail call (saves one RET)

; --- eat_paren_expr: skip spaces, expect '(', eval expr, expect ')' -> AX ---
eat_paren_expr:
        call spaces
        cmp byte [si], '('
        jne epe_err
        inc si
        call expr
        call spaces
        cmp byte [si], ')'
        jne epe_err
        inc si
        ret
epe_err:
        mov ax, ERR_SN
        jmp do_error

peek_tab:       dw kw_peek, 0
usr_tab:        dw kw_usr,  0

; =============================================================================
; GET_VAR_ADDR  map letter at [SI] to variable address
;
; Inputs  : SI -> letter (not yet consumed; any case)
; Outputs : DI = &vars[(letter-'A')*2]; SI advanced past letter
; Clobbers: AX, DI
; =============================================================================
get_var_addr:
        lodsb
        call uc_al
        sub al, 'A'             ; 0..25
        xor ah, ah
        add ax, ax              ; *2 (word per variable)
        add ax, VARS
        mov di, ax
        ret

; =============================================================================
; SPACES  skip spaces at SI; preserves AX, BX, CX, DX
; =============================================================================
spaces:
        cmp byte [si], ' '
        jne sp_r
        inc si
        jmp spaces
sp_r:   ret

; =============================================================================
; INPUT_NUMBER  parse unsigned decimal integer from [SI] -> AX
;
; Inputs  : SI -> text (spaces NOT skipped here; call spaces first if needed)
; Outputs : AX = value (0 if no digits); SI -> first non-digit char
; Clobbers: AX, BX, CX
; =============================================================================
input_number:
        xor bx, bx              ; BX = accumulator
inm_lp:
        mov al, [si]
        sub al, '0'
        jb inm_done
        cmp al, 9
        ja inm_done
        inc si
        cbw                     ; zero-extend digit to AX
        xchg ax, bx             ; AX = old accum, BX = new digit
        mov cx, 10
        mul cx                  ; AX = old_accum * 10
        add bx, ax              ; BX = accum*10 + digit
        jmp inm_lp
inm_done:
        mov ax, bx
        ret

; =============================================================================
; OUTPUT_NUMBER  print signed 16-bit AX to terminal
;
; Clobbers: AX, CX, DX
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
        div cx                  ; AX = quotient, DX = remainder
        push dx                 ; save this digit
        or ax, ax
        je on_digit
        call output_number      ; recurse for higher digits
on_digit:
        pop ax
        add al, '0'
        jmp output              ; tail call

; =============================================================================
; OUTPUT  write AL to terminal via BIOS INT 10h TTY function
; Clobbers: AX, BX (BIOS uses BH=page, BL=attr)
; =============================================================================
output:
        push bx
        mov ah, 0x0e
        mov bx, 0x0007          ; page 0, white on black
        int 0x10
        pop bx
        ret

; =============================================================================
; NEW_LINE  emit CR + LF
; Clobbers: AX, BX
; =============================================================================
new_line:
        mov al, 0x0d
        call output
        mov al, 0x0a
        jmp output              ; tail call

; =============================================================================
; INPUT_LINE  read edited line from keyboard into IBUF
;
; Outputs : IBUF = typed characters + CR; SI -> IBUF
; Clobbers: AX, CX, DI
; =============================================================================
input_line:
        mov di, IBUF
        xor cx, cx              ; character count
ipl_lp:
        call input_key          ; AL = keystroke
        cmp al, 0x08            ; backspace?
        jne ipl_nbs
        or cx, cx
        je ipl_lp               ; ignore BS at start of line
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
        cmp al, 0x0d            ; CR = end of line
        je ipl_cr
        cmp cx, 32              ; buffer full?
        jnb ipl_lp
        call output             ; echo
        stosb
        inc cx
        jmp ipl_lp
ipl_cr:
        call output             ; echo CR
        stosb                   ; store CR in buffer
        mov si, IBUF
        ret

; =============================================================================
; INPUT_KEY  read one keystroke (blocking) via BIOS INT 16h
; Outputs : AL = ASCII character
; Clobbers: AX
; =============================================================================
input_key:
        mov ah, 0x00
        int 0x16
        ret

; =============================================================================
; LINE EDITOR
; Retained and adapted from uBASIC8088 v2.1.0
; =============================================================================

; -----------------------------------------------------------------------------
; WALK_LINES
; Function: walk program from start; stop at first line >= AX or end marker
; Inputs  : AX = stop threshold (0xFFFF = walk to true end)
; Outputs : DI = pointer to first line with line# >= AX, or end marker
; Clobbers: AX, BX
; -----------------------------------------------------------------------------
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
wl_done:
        ret

; FIND_LINE  DI -> first line >= AX (or end marker)
find_line:
        jmp walk_lines          ; AX already set by caller

; FIND_PROGRAM_END  DI -> end marker
find_program_end:
        mov ax, 0xffff
        jmp walk_lines

; -----------------------------------------------------------------------------
; NEXT_LINE_PTR
; Function: advance DI to start of next line (past current line's CR)
; Inputs  : DI -> current line header (word linenum + body + CR)
; Outputs : DI -> next line header
; Clobbers: DI
; -----------------------------------------------------------------------------
next_line_ptr:
        add di, 2               ; skip 2-byte line number
nlp_lp:
        cmp byte [di], 0x0d
        je nlp_done
        inc di
        jmp nlp_lp
nlp_done:
        inc di                  ; skip CR
        ret

; -----------------------------------------------------------------------------
; EDITLN
; Function: insert / replace / delete a numbered program line
; Inputs  : AX = line number; SI -> body text starting at first non-space
;           after the line number (i.e. spaces already skipped by main_loop)
; Outputs : program image updated in RAM; PROG_END updated
; Clobbers: AX, BX, CX, DX, SI, DI, BP
; -----------------------------------------------------------------------------
editln:
        push ax
        call spaces             ; skip any spaces between line# and body
        pop dx                  ; DX = line number
        mov bx, si              ; BX -> body start (save for later)

        ; Count body length: CX = bytes including CR
        mov cx, 1
el_len:
        cmp byte [si], 0x0d
        je el_ldone
        inc si
        inc cx
        jmp el_len
el_ldone:
        ; Find insertion/match point
        mov ax, dx
        call find_line          ; DI -> first line >= AX, or end marker
        cmp [di], dx            ; exact match?
        jne el_noex
        call deline             ; delete existing line with same number
el_noex:
        cmp byte [bx], 0x0d     ; empty body (just line number) = delete only
        je el_done
        mov si, bx              ; SI -> body
        mov ax, dx              ; AX = line number
        add cx, 2               ; +2 for line number word in store
        call insline
el_done:
        ret

; -----------------------------------------------------------------------------
; INSLINE
; Function: insert a new line into the program store
; Inputs  : AX = line number
;           DI = insertion point (from find_line after any deline)
;           SI -> body text (first char through CR inclusive)
;           CX = total bytes to store (2-byte header + body + CR)
; Outputs : program shifted up; new line written; PROG_END updated
; Clobbers: AX, BX, CX, DX, SI, DI, BP
;
; Strategy:
;   1. Find end marker -> save to INS_TMP (RAM temp, avoids BP/SP conflict).
;   2. OOM check.
;   3. Shift [insert_point .. end_marker+1] upward by CX bytes using STD movsb.
;   4. Re-find insert point via find_line (DI was clobbered by movsb).
;   5. Write 2-byte header then copy body.
;   6. Update PROG_END.
; -----------------------------------------------------------------------------
insline:
        ; Save all inputs - we need them after find_program_end clobbers DI
        push ax                 ; [SP+6] line number
        push cx                 ; [SP+4] total size (header + body + CR)
        push si                 ; [SP+2] body pointer
        push di                 ; [SP+0] insertion point

        call find_program_end   ; DI = end marker address
        mov [INS_TMP], di       ; save to RAM - avoids BP register conflict

        ; OOM check: require end_marker + 2 + total_size <= PROGRAM_TOP
        mov bx, di
        add bx, 2
        ; 8086 has no [SP+n] addressing. Use BP as frame pointer instead.
        mov bp, sp
        ; Frame: [BP+0]=DI_insert [BP+2]=SI_body [BP+4]=CX_size [BP+6]=AX_linenum
        mov cx, [bp+4]          ; CX = total line size
        mov ax, [bp+6]          ; AX = line number (restore from frame)

        mov bx, [INS_TMP]
        add bx, 2
        add bx, cx
        cmp bx, PROGRAM_TOP
        ja ins_oom

        ; Shift bytes from [insert..end+1] up by CX (= line size) bytes
        ; Use STD (decrement): source top = end_marker+1, dest top = end_marker+1+CX
        ; bytes_to_move = end_marker + 2 - insert_point
        mov di, [INS_TMP]
        mov si, di
        inc si                  ; SI = end_marker + 1 (top of source)
        add di, cx
        inc di                  ; DI = end_marker + 1 + CX (top of dest)
        mov bx, [INS_TMP]
        add bx, 2
        sub bx, [bp+0]          ; BX = bytes to move = end+2 - insert_point
        push bx                 ; save move count
        mov cx, bx
        std
        rep movsb               ; shift upward
        cld

        ; Reload saved inputs from BP frame (movsb clobbered SI and DI)
        pop bx                  ; discard saved move count
        mov ax, [bp+6]          ; AX = line number
        mov si, [bp+2]          ; SI = body pointer
        mov cx, [bp+4]          ; CX = total line size
        add sp, 8               ; discard 4 original push saves (AX,CX,SI,DI)

        ; Re-find insertion point (DI was clobbered by movsb)
        call find_line          ; DI -> correct insertion slot

        ; Write line number header
        mov [di], ax
        add di, 2

        ; Copy body + CR
ins_copy:
        lodsb
        stosb
        cmp al, 0x0d
        jne ins_copy

        ; Update PROG_END
        add [PROG_END], cx      ; CX = total line size

        ; Write new end marker after inserted line (find_program_end recalculates)
        mov word [di], 0
        ret

ins_oom:
        add sp, 8               ; clean stack (4 original pushes)
        mov ax, ERR_OM
        jmp do_error

; -----------------------------------------------------------------------------
; DELINE
; Function: delete the line at DI by sliding content above it downward
; Inputs  : DI -> first byte of line to delete (line number word)
; Outputs : program slid down; PROG_END decremented by deleted line size
; Clobbers: AX, BX, CX, SI, DI
; -----------------------------------------------------------------------------
deline:
        push di                 ; save destination (start of deleted line)
        call next_line_ptr      ; DI -> first byte of next line (past deleted CR)
        mov si, di              ; SI = source for slide-down
        call find_program_end   ; DI = end marker
        mov cx, di
        add cx, 2               ; CX = end_marker + 2 (include end marker word)
        pop di                  ; DI = destination (start of deleted line)
        mov bx, si
        sub bx, di              ; BX = size of deleted line (source - dest)
        sub cx, si              ; CX = bytes to slide down (from next-line to end+2)
        rep movsb               ; forward move (no overlap since dest < source)
        sub [PROG_END], bx      ; update program end
        ret

; =============================================================================
; Keyword strings (bit-7 termination on last byte; 0x00 ends do_help table)
;
; kw_tab_start is walked by do_help (stops at 0x00 sentinel).
; Statement keywords follow in st_tab dispatch order.
; Expression keywords (THEN, CHR$, PEEK, USR) come after the sentinel.
; Using db hex to avoid any assembler quoting issues with special chars.
; =============================================================================
kw_tab_start:
kw_print:   db 0x50,0x52,0x49,0x4e,T_T     ; PRINT
kw_if:      db 0x49,T_F                     ; IF
kw_goto:    db 0x47,0x4f,0x54,T_O          ; GOTO
kw_list:    db 0x4c,0x49,0x53,T_T          ; LIST
kw_run:     db 0x52,0x55,T_N               ; RUN
kw_new:     db 0x4e,0x45,T_W              ; NEW
kw_input:   db 0x49,0x4e,0x50,0x55,T_T    ; INPUT
kw_rem:     db 0x52,0x45,T_M              ; REM
kw_end:     db 0x45,0x4e,T_D             ; END
kw_let:     db 0x4c,0x45,T_T             ; LET
kw_poke:    db 0x50,0x4f,0x4b,T_E        ; POKE
kw_free:    db 0x46,0x52,0x45,T_E        ; FREE
kw_help:    db 0x48,0x45,0x4c,T_P        ; HELP
            db 0                           ; sentinel (end of do_help table)

; Expression-only keywords (not in do_help walk, not in st_tab)
kw_then:    db 0x54,0x48,0x45,T_N         ; THEN
kw_chrs:    db 0x43,0x48,0x52,T_DS        ; CHR$
kw_peek:    db 0x50,0x45,0x45,T_K         ; PEEK
kw_usr:     db 0x55,0x53,T_R              ; USR

; =============================================================================
; String constants (null-terminated)
; =============================================================================
str_banner: db "uBASIC 8088 v1.0.0", 0x0d
str_in:     db " IN ",0x22,";" ; no newline
str_free:   db "FREE",0x0d

; pad to 2048 bytes
	times 2048-($-$$) db 0xff
	

ROM_END:
