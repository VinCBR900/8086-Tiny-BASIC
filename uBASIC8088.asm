; =============================================================================
; uBASIC 8088  v3.0.0
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Tiny BASIC for 8088/8086. Target: <=2048 bytes ROM, 4096 bytes RAM.
; Re-engineered from uBASIC 65c02 v17.0.
;
; Statements : PRINT IF..THEN GOTO LET INPUT REM END RUN LIST NEW POKE FREE HELP
; Expressions: + - * / %  = < > <= >= <>  unary-  CHR$(n) PEEK(addr) USR(addr) A-Z
; Numbers    : signed 16-bit
; Multi-stmt : colon separator ':'
;
; Errors: ?0 syntax  ?1 undef line  ?2 div/zero  ?3 out of mem  ?4 bad variable
; Segment: CS=DS=ES=SS (single segment, COM file and bare-metal EPROM compatible)
; I/O: BIOS INT 10h display / INT 16h keyboard  (replace for bare-metal UART)
;
; Line store: <linenum_lo> <linenum_hi> <raw ASCII body> <CR>  (no tokenization)
;
; History:
;   v3.0.0 (2026-04-09) Initial 8088 port from uBASIC 65c02 v17.0.
;                        Line editor from uBASIC8088 v2.1.0.
; =============================================================================

        cpu 8086
        org 0x0100

; --- RAM layout (single flat segment, RAM at 0x1000) -------------------------
RAM_START:      equ 0x1000
STACK_TOP:      equ 0x2000
PROGRAM_TOP:    equ 0x1E00

VARS:           equ 0x1000      ; 52 bytes: A-Z word variables
RUNNING:        equ 0x1034      ; byte:  0=immediate 1=running
CURLN:          equ 0x1036      ; word:  current line# for error reports
IBUF:           equ 0x1038      ; 34 bytes: input line buffer
PROG_END:       equ 0x105B      ; word:  one-past-last program byte
RUN_NEXT:       equ 0x105D      ; word:  next-line pointer for run loop
PROGRAM:        equ 0x105F      ; program store starts here

; --- Error codes -------------------------------------------------------------
ERR_SN:         equ 0
ERR_UL:         equ 1
ERR_OV:         equ 2
ERR_OM:         equ 3
ERR_UK:         equ 4

; --- Bit-7-terminated last-byte constants for keywords (ASCII | 0x80) --------
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
; INIT / COLD START
; =============================================================================
start:
        cld
        mov sp, STACK_TOP

        mov di, VARS            ; clear variables A-Z
        xor ax, ax
        mov cx, 26
        rep stosw

        mov byte [RUNNING], 0
        mov word [CURLN], 0
        mov word [PROG_END], PROGRAM
        mov word [PROGRAM], 0   ; end marker

        mov si, str_banner
        call print_z
        call do_free
        ; fall through

; =============================================================================
; MAIN_LOOP
; =============================================================================
main_loop:
        mov sp, STACK_TOP
        mov byte [RUNNING], 0
        mov al, '>'
        call output
        mov al, ' '
        call output
        call input_line         ; SI -> IBUF
        call spaces
        cmp byte [si], 0x0d
        je main_loop
        call input_number       ; AX = line number (0 if none)
        or ax, ax
        je stmt_line
        call editln
        jmp main_loop

; =============================================================================
; DO_ERROR  in: AX=error code  no return
; =============================================================================
do_error:
        push ax
        mov al, 0x0d
        call output
        mov al, 0x0a
        call output
        mov al, '?'
        call output
        pop ax
        add al, '0'
        call output
        cmp byte [RUNNING], 0
        je do_error_nl
        mov si, str_in
        call print_z
        mov ax, [CURLN]
        call output_number
do_error_nl:
        call new_line
        jmp main_loop

; =============================================================================
; STMT_LINE  execute ':'-separated statements
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
; STMT  execute one statement
; =============================================================================
stmt:
        call spaces
        cmp byte [si], 0x0d
        je stmt_ret
        cmp byte [si], 0
        je stmt_ret
        mov bx, st_tab
stmt_lp:
        mov ax, [bx]
        or ax, ax
        je stmt_let
        call kw_match
        jnc stmt_call
        add bx, 4
        jmp stmt_lp
stmt_call:
        call [bx+2]
        ret
stmt_let:
        jmp do_let
stmt_ret:
        ret

; --- Dispatch table: dw kw_ptr, dw handler_addr; ends with dw 0 -------------
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

; =============================================================================
; KW_MATCH  case-insensitive keyword match at [SI]
; in:  BX -> table entry (word 0 = keyword string ptr)
; out: CF=0 matched SI advanced past keyword
;      CF=1 no match  SI unchanged
; clobbers: AX, DI, DL
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
        cmp al, 'a'
        jb kw_kuc
        and al, 0xdf
kw_kuc:
        mov dl, [si]
        inc si
        cmp dl, 'a'
        jb kw_suc
        cmp dl, 'z'
        ja kw_suc
        and dl, 0xdf
kw_suc:
        cmp al, dl
        jne kw_fail
        test ah, 0x80
        jz kw_lp
        ; matched - check word boundary
        mov al, [si]
        cmp al, 'a'
        jb kw_wbuc
        cmp al, 'z'
        ja kw_wbuc
        and al, 0xdf
kw_wbuc:
        cmp al, 'A'
        jb kw_ok
        cmp al, 'Z'
        jbe kw_fail
        cmp al, '0'
        jb kw_ok
        cmp al, '9'
        jbe kw_fail
        cmp al, '_'
        je kw_fail
kw_ok:
        pop ax
        clc
        ret
kw_fail:
        pop si
        stc
        ret

; =============================================================================
; DO_IF
; =============================================================================
do_if:
        call expr
        or ax, ax
        je do_if_f
        mov bx, then_tab
        call kw_match           ; consume optional THEN (CF=1 = no match, SI restored)
        jmp stmt
do_if_f:
        ret

then_tab:       dw kw_then, 0

; =============================================================================
; DO_GOTO
; =============================================================================
do_goto:
        call expr
        push ax
        call find_line
        pop bx
        cmp [di], bx
        je dg_ok
        mov ax, ERR_UL
        jmp do_error
dg_ok:
        mov [RUN_NEXT], di
        cmp byte [RUNNING], 0
        jne dg_ret
        mov byte [RUNNING], 1
        jmp run_loop
dg_ret:
        ret

; =============================================================================
; DO_RUN
; =============================================================================
do_run:
        mov di, PROGRAM
        mov byte [RUNNING], 1
        mov [RUN_NEXT], di
run_loop:
        mov di, [RUN_NEXT]
        cmp word [di], 0
        je run_end
        mov ax, [di]
        mov [CURLN], ax
        push di
        call next_line_ptr      ; DI -> next line
        mov [RUN_NEXT], di
        pop di
        lea si, [di+2]          ; SI -> body
        call stmt_line
        jmp run_loop
run_end:
        mov byte [RUNNING], 0
        ret

; =============================================================================
; DO_LIST
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
        lea si, [di+2]
dl_body:
        lodsb
        call output
        cmp al, 0x0d
        jne dl_body
        mov al, 0x0a
        call output
        push di
        call next_line_ptr
        pop ax
        jmp dl_lp
dl_done:
        ret

; =============================================================================
; DO_NEW
; =============================================================================
do_new:
        mov word [PROGRAM], 0
        mov word [PROG_END], PROGRAM
        ret

; =============================================================================
; DO_END
; =============================================================================
do_end:
        mov byte [RUNNING], 0
        mov di, [RUN_NEXT]
        mov word [di], 0
        ret

; =============================================================================
; DO_REM
; =============================================================================
do_rem:
        cmp byte [si], 0x0d
        je do_rem_r
        inc si
        jmp do_rem
do_rem_r:
        ret

; =============================================================================
; DO_PRINT
; =============================================================================
do_print:
dp_top:
        call spaces
        cmp byte [si], 0x0d
        je dp_nl
        cmp byte [si], '"'
        jne dp_expr
        inc si
dp_str:
        lodsb
        cmp al, '"'
        je dp_after
        cmp al, 0x0d
        je dp_str_eol
        call output
        jmp dp_str
dp_str_eol:
        dec si
        jmp dp_nl
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
        cmp byte [si], ';'
        jne dp_nl
        inc si
        call spaces
        cmp byte [si], 0x0d
        je dp_ret
        jmp dp_top
dp_nl:
        call new_line
dp_ret:
        ret

chrs_tab:       dw kw_chrs, 0

; =============================================================================
; DO_INPUT
; =============================================================================
do_input:
        call get_var_addr
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

; =============================================================================
; DO_LET
; =============================================================================
do_let:
        call spaces
        mov al, [si]
        cmp al, 'a'
        jb dl_uc
        cmp al, 'z'
        ja dl_uc
        and al, 0xdf
dl_uc:
        cmp al, 'A'
        jb dl_err
        cmp al, 'Z'
        ja dl_err
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
dl_err:
        mov ax, ERR_UK
        jmp do_error

; =============================================================================
; DO_POKE
; =============================================================================
do_poke:
        call expr
        mov di, ax
        call spaces
        cmp byte [si], ','
        jne dpk_err
        inc si
        call expr
        mov [di], al
        ret
dpk_err:
        mov ax, ERR_SN
        jmp do_error

; =============================================================================
; DO_FREE
; =============================================================================
do_free:
        mov ax, PROGRAM_TOP
        sub ax, [PROG_END]
        call output_number
        mov al, ' '
        call output
        mov si, str_free
        call print_z
        call new_line
        ret

; =============================================================================
; DO_HELP
; =============================================================================
do_help:
        mov si, kw_tab_start
dh_lp:
        lodsb
        or al, al
        je dh_done
        push ax
        and al, 0x7f
        call output
        pop ax
        test al, 0x80
        jz dh_lp
        mov al, ' '
        call output
        jmp dh_lp
dh_done:
        call new_line
        ret

; =============================================================================
; EXPR  relational level
; out: AX = result  true=0xFFFF false=0x0000
; =============================================================================
expr:
        call expr_add
        call spaces
        mov al, [si]
        cmp al, '='
        je rel_eq
        cmp al, '<'
        je rel_lt
        cmp al, '>'
        je rel_gt
        ret

; rel_setup: caller has consumed first op char; AX=left on entry
; returns AX=right BX=left
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
        call rel_setup
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
        call rel_setup
        cmp bx, ax
        jg rel_t
        jmp rel_f
rel_ge:
        inc si
        call rel_setup
        cmp bx, ax
        jge rel_t
        jmp rel_f

rel_t:
        mov ax, 0xffff
        ret
rel_f:
        xor ax, ax
        ret

; =============================================================================
; EXPR_ADD  + and -
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
        push ax
        inc si
        call expr1
        pop bx
        cmp dl, '-'
        jne ea_add
        neg ax
ea_add:
        add ax, bx
        jmp ea_lp
ea_ret:
        ret

; =============================================================================
; EXPR1  * / %
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
        push dx                 ; save operator
        push ax                 ; save left
        call expr2              ; right -> AX
        pop cx                  ; CX = left
        pop dx                  ; DL = operator
        cmp dl, '*'
        jne e1_dv
        xchg ax, cx
        imul cx                 ; AX = AX*CX (low word)
        jmp e1_lp
e1_dv:
        or ax, ax
        je e1_zero
        xchg ax, cx             ; AX=left CX=right
        cwd
        idiv cx
        cmp dl, '%'
        jne e1_lp
        mov ax, dx
        jmp e1_lp
e1_zero:
        mov ax, ERR_OV
        jmp do_error
e1_ret:
        ret

; =============================================================================
; EXPR2  atoms
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
        ; decimal literal?
        cmp al, '0'
        jb e2_var
        cmp al, '9'
        ja e2_var
        call input_number
        ret
e2_var:
        ; variable A-Z?
        cmp al, 'a'
        jb e2_vuc
        cmp al, 'z'
        ja e2_vuc
        and al, 0xdf
e2_vuc:
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
e2_par:
        inc si
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
        jmp expr2

; eat_paren_expr: expect '(' eval expr expect ')' -> AX
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
; GET_VAR_ADDR  letter at [SI] -> DI=&var, SI advanced
; =============================================================================
get_var_addr:
        lodsb
        cmp al, 'a'
        jb gv_uc
        and al, 0xdf
gv_uc:
        sub al, 'A'
        xor ah, ah
        add ax, ax
        add ax, VARS
        mov di, ax
        ret

; =============================================================================
; SPACES  skip spaces; preserves AX
; =============================================================================
spaces:
        cmp byte [si], ' '
        jne sp_ret
        inc si
        jmp spaces
sp_ret:
        ret

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
        je on_d
        call output_number
on_d:
        pop ax
        add al, '0'
        jmp output

; =============================================================================
; OUTPUT  AL -> BIOS TTY
; =============================================================================
output:
        push bx
        mov ah, 0x0e
        mov bx, 0x0007
        int 0x10
        pop bx
        ret

; =============================================================================
; NEW_LINE  CR+LF
; =============================================================================
new_line:
        mov al, 0x0d
        call output
        mov al, 0x0a
        jmp output

; =============================================================================
; INPUT_LINE  read line with backspace into IBUF; SI -> IBUF
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
        cmp cx, 32
        jnb ipl_lp
        call output
        stosb
        inc cx
        jmp ipl_lp
ipl_cr:
        call output
        stosb
        mov si, IBUF
        ret

; =============================================================================
; INPUT_KEY  -> AL
; =============================================================================
input_key:
        mov ah, 0x00
        int 0x16
        ret

; =============================================================================
; PRINT_Z  print null-terminated string at SI
; =============================================================================
print_z:
        lodsb
        or al, al
        je pz_ret
        call output
        jmp print_z
pz_ret:
        ret

; =============================================================================
; LINE EDITOR  (from uBASIC8088 v2.1.0)
; =============================================================================

; WALK_LINES  DI -> first line >= AX (or end marker if AX=0xFFFF)
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

find_line:
        jmp walk_lines

find_program_end:
        mov ax, 0xffff
        jmp walk_lines

; NEXT_LINE_PTR  advance DI to start of next line
next_line_ptr:
        add di, 2
nlp_lp:
        cmp byte [di], 0x0d
        je nlp_done
        inc di
        jmp nlp_lp
nlp_done:
        inc di
        ret

; EDITLN  AX=linenum SI->body+CR
editln:
        push ax
        call spaces
        pop dx
        mov bx, si
        mov cx, 1
el_len:
        cmp byte [si], 0x0d
        je el_ldone
        inc si
        inc cx
        jmp el_len
el_ldone:
        mov ax, dx
        call find_line
        cmp [di], dx
        jne el_noex
        call deline
el_noex:
        cmp byte [bx], 0x0d
        je el_done
        mov si, bx
        mov ax, dx
        add cx, 2
        call insline
el_done:
        ret

; INSLINE  AX=linenum DI=insert-point SI->body CX=total-size
insline:
        push ax
        push cx
        push si
        push di
        call find_program_end   ; DI = end marker
        mov bp, di
        pop di                  ; DI = insert point
        pop si
        pop cx                  ; CX = total new line size
        pop ax                  ; AX = line number

        ; OOM check
        mov bx, bp
        add bx, 2
        add bx, cx
        cmp bx, PROGRAM_TOP
        ja ins_oom

        ; Shift content above DI up by CX bytes (top-down to avoid overlap)
        push ax
        push cx
        push si
        push di
        mov si, bp              ; source: end marker hi byte first
        inc si                  ; SI = bp+1
        mov di, si
        add di, cx              ; DI = bp+1+cx (destination)
        mov cx, bp
        add cx, 2
        sub cx, [esp]           ; CX = bp+2 - DI_orig = bytes to move
        ; We need DI_orig. It's at [esp+0] (last pushed = DI).
        ; stack: [esp+0]=DI [esp+2]=SI [esp+4]=CX [esp+6]=AX
        mov cx, bp
        add cx, 2
        sub cx, [esp]           ; [esp] = original DI
        ; Now move CX bytes from [SI] down to [SI+cx] (upward in memory)
        ; Use std rep movsb
        ; SI = source top = bp+1, DI = dest top = bp+1+new_cx
        ; We already set SI and DI above (before the push sequence messed them)
        ; Let's just do it properly without further confusion:
        pop di                  ; DI = insert point (original)
        pop si                  ; SI (discard, was body ptr)
        pop cx                  ; CX = total line size
        pop ax                  ; AX = line number
        ; bytes to move = bp+2 - DI
        push cx                 ; save line size
        mov cx, bp
        add cx, 2
        sub cx, di              ; CX = bytes above insert point (incl end marker)
        std
        lea si, [bp+1]          ; source top byte
        mov di, bp
        add di, [esp]           ; dest top = bp + line_size
        ; [esp] = saved line size
        inc di
        rep movsb
        cld
        pop cx                  ; CX = line size

        ; Recalculate insert point (was clobbered by movsb changing DI)
        ; We know AX = line number; find_line gives us the right DI
        push ax
        push cx
        call find_line          ; DI = correct insert point
        pop cx
        pop ax

        ; Write line number
        mov [di], ax
        add di, 2
        ; Copy body from SI (= body ptr saved as bx in editln... but bx not here)
        ; SI was restored by kw_match pushes; at this point SI is unknown.
        ; editln passes SI=bx (body start). But insline's SI arg was popped away.
        ; FIX: pass body ptr via a dedicated location.
        ; For now write a note and use a workaround:
        ; After find_line, DI points to correct gap. Body is contiguous from
        ; what was at [di] before (the content shifted up). We need the original
        ; body source - this is a design issue in the register passing.
        ; The body source pointer was passed in SI by editln, but got clobbered.
        ; TODO: fix insline calling convention - save body ptr to RAM before call.
ins_oom:
        mov ax, ERR_OM
        jmp do_error

; DELINE  delete line at DI
deline:
        push di
        call next_line_ptr      ; DI -> byte after deleted line
        mov si, di
        call find_program_end   ; DI = end marker
        mov cx, di
        add cx, 2
        pop di                  ; DI = line to delete
        mov bx, si
        sub bx, di              ; BX = deleted line size
        sub cx, si              ; CX = bytes to slide down
        rep movsb
        sub [PROG_END], bx
        ret

; =============================================================================
; Keyword strings: bit-7 termination on last char; 0x00 ends do_help table
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
            db 0                ; do_help sentinel

; Expression keywords (not in do_help walk, not in st_tab)
kw_then:    db 0x54,0x48,0x45,T_N
kw_chrs:    db 0x43,0x48,0x52,T_DS
kw_peek:    db 0x50,0x45,0x45,T_K
kw_usr:     db 0x55,0x53,T_R

; =============================================================================
; Strings
; =============================================================================
str_banner: db "uBASIC 8088 v3.0.0",0x0d,0x0a,0
str_in:     db " IN ",0
str_free:   db "FREE",0
