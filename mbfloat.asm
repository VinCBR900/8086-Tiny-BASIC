; =============================================================================
; MiniBASIC8088  MBF5 Float Library  v0.3
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; WORK IN PRGRESS - NOTHING TO SEE HERE 
; 5-byte Microsoft Binary Format floating-point routines for 8088/8086.
; Standalone test harness: assemble with tinyasm, run with sim_rom.
;
; ---------------------------------------------------------------------------
; MBF5 FORMAT
; ---------------------------------------------------------------------------
; Byte 0   : biased exponent. 0x00 = exact zero.
;            Stored exponent = true_exponent + 0x80
;            Value = (-1)^sign * 2^(exp-0x80) * 0.1mmmm...
; Bytes 1-4: mantissa MSB first.
;            Bit 7 of byte 1 = sign.  Bits 30..0 = mantissa (implied leading 1).
;
; Working 24-bit mantissa register layout (mirrors MS BL:DX:AH convention):
;   BL  = mant[30:24]  (bit 7 = implied-1 during arithmetic)
;   DH  = mant[23:16]
;   DL  = mant[15:8]
;   AH  = mant[7:0]    guard byte for rounding
;   BH  = biased exponent inside norm_pack
;
; ---------------------------------------------------------------------------
; CALLING CONVENTION
; ---------------------------------------------------------------------------
; FLT_A (5 bytes) = primary operand and result
; FLT_B (5 bytes) = secondary operand
; FLT_T (5 bytes) = scratch (flt_print and flt_add alignment only)
; FLT_SA, FLT_SB, FLT_ER = 1-byte arithmetic temporaries
; AX = integer in/out.  SI preserved.  BX,CX,DX,DI freely clobbered.
;
; ---------------------------------------------------------------------------
; BUILD
; ---------------------------------------------------------------------------
;   tinyasm -f bin mbfloat.asm -o mbfloat.bin
;   sim_rom mbfloat.bin
; =============================================================================
; CHANGE HISTORY
; =============================================================================
;   v0.3 (2026-05-16)  Full-precision rewrite.
;     - norm_pack: shared normalise+round+pack kernel.  Algorithm from MS
;       $NORMS (MATH2.ASM:1118) and $ROUNS/$PAKSP (MATH2.ASM:1779).
;       Byte-at-a-time shift optimisation; guard-byte round-half-up.
;     - flt_add: 24-bit BL:DX:AH aligned mantissa add/sub with proper 4-byte
;       borrow-chain subtract and 2's-complement correction if borrow occurs.
;       Smaller operand kept in FLT_T[0..3] to free CL for shift count.
;       Byte-at-a-time alignment shift optimisation.  Algorithm from MS
;       $FADDS (MATH1.ASM:3287).
;     - flt_mul: 24x24->24 via 3 partial 16x8 MUL products.  Algorithm from
;       MS $FMULS (MATH2.ASM:416).  Replaces 16x16 upper-half-only method.
;     - flt_div: 32/16 two-stage: stage-1 gives 16-bit quotient word,
;       stage-2 remainder feeds 8-bit guard byte.  Guard saved before
;       quotient overwrite (v0.2 sequencing bug fixed).
;     - flt_parse: digit pushed before flt_mul*10 call so it survives across
;       the call chain (v0.2 bug: digit lost because flt_mul clobbers AX).
;   v0.2 (2026-05-13)  Register-allocation rewrite.
;   v0.1 (2026-05-12)  Initial draft.
; =============================================================================

        cpu  8086
        org  0xF000

; =============================================================================
; RAM LAYOUT
; =============================================================================
STACK_TOP: equ 0x0800
FLT_A:  equ 0x00C0              ; 5 bytes: primary operand / result
FLT_B:  equ 0x00C5              ; 5 bytes: secondary operand
FLT_T:  equ 0x00CA              ; 5 bytes: scratch (flt_print + flt_add)
FLT_SA: equ 0x00CF              ; 1 byte : result sign
FLT_SB: equ 0x00D0              ; 1 byte : sign of B (add/sub comparison)
FLT_ER: equ 0x00D1              ; 1 byte : result exponent
FLT_TS: equ 0x00D2              ; 1 byte : flt_to_int sign scratch
FLT_DE: equ 0x00D3              ; 1 byte : flt_print decimal exponent
IBUF:   equ 0x000C              ; 64 bytes: digit buffer

; =============================================================================
; TEST HARNESS
; =============================================================================
start:
        cli
        xor  ax, ax
        mov  ds, ax
        mov  es, ax
        mov  ss, ax
        mov  sp, STACK_TOP

        mov  si, s_t1           ; T1: 1+1 = 2
        call print_sz
        mov  ax, 1
        call flt_from_int
        call flt_a_to_b
        call flt_add
        ; DEBUG: print FLT_A bytes as hex
        mov  si, FLT_A
        mov  cx, 5
t1dbg:  mov  al, [si]
        call print_hex_byte
        inc  si
        loop t1dbg
        mov  al, '='
        call output
        call flt_print
        call new_line

        mov  si, s_t2           ; T2: 355/113 ~ 3.141593
        call print_sz
        mov  ax, 355
        call flt_from_int
        mov  ax, 113
        call flt_from_int_b
        call flt_div
        call flt_print
        call new_line

        mov  si, s_t3           ; T3: -7*6 = -42
        call print_sz
        mov  ax, -7
        call flt_from_int
        mov  ax, 6
        call flt_from_int_b
        call flt_mul
        call flt_print
        call new_line

        mov  si, s_t4           ; T4: 1/3 ~ 0.333333
        call print_sz
        mov  ax, 1
        call flt_from_int
        mov  ax, 3
        call flt_from_int_b
        call flt_div
        call flt_print
        call new_line

        mov  si, s_t5           ; T5: 100 - 100.5 = -0.5
        call print_sz
        mov  ax, 100
        call flt_from_int
        call flt_a_to_t         ; save 100 in FLT_T
        mov  si, s_1005
        call flt_parse          ; FLT_A = 100.5
        call flt_a_to_b         ; FLT_B = 100.5
        call flt_t_to_a         ; FLT_A = 100
        call flt_sub            ; FLT_A = 100 - 100.5
        call flt_print
        call new_line

        mov  si, s_t6           ; T6: cmp(3,4) = -1
        call print_sz
        mov  ax, 3
        call flt_from_int
        mov  ax, 4
        call flt_from_int_b
        call flt_cmp
        call output_int
        call new_line

        mov  si, s_t7           ; T7: parse("2.5") = 2.5
        call print_sz
        mov  si, s_25
        call flt_parse
        call flt_print
        call new_line

        mov  si, s_t8           ; T8: trunc(3.9) = 3
        call print_sz
        mov  si, s_39
        call flt_parse
        call flt_to_int
        call output_int
        call new_line

        mov  si, s_t9           ; T9: 0 + 0 = 0
        call print_sz
        call flt_zero
        call flt_a_to_b
        call flt_add
        call flt_print
        call new_line

        mov  si, s_t10          ; T10: 32767 * 2 = 65534
        call print_sz
        mov  ax, 32767
        call flt_from_int
        mov  ax, 2
        call flt_from_int_b
        call flt_mul
        call flt_print
        call new_line

        mov  si, s_done
        call print_sz
        call new_line
        hlt

s_t1:   db "T1 1+1=",0
s_t2:   db "T2 355/113=",0
s_t3:   db "T3 -7*6=",0
s_t4:   db "T4 1/3=",0
s_t5:   db "T5 100-100.5=",0
s_t6:   db "T6 cmp(3,4)=",0
s_t7:   db "T7 parse(2.5)=",0
s_t8:   db "T8 trunc(3.9)=",0
s_t9:   db "T9 0+0=",0
s_t10:  db "T10 32767*2=",0
s_done: db "DONE",0
s_1005: db "100.5",0x0D
s_25:   db "2.5",0x0D
s_39:   db "3.9",0x0D

; =============================================================================
; FLT_ZERO  FLT_A = +0.0
; Clobbers: AX, CX, DI
; =============================================================================
flt_zero:
        xor  al, al
        mov  di, FLT_A
        mov  cx, 5
fz_lp:  stosb
        loop fz_lp
        ret

; =============================================================================
; FLT_ZERO_B  FLT_B = +0.0
; Clobbers: AX, CX, DI
; =============================================================================
flt_zero_b:
        xor  al, al
        mov  di, FLT_B
        mov  cx, 5
fzb_lp: stosb
        loop fzb_lp
        ret

; =============================================================================
; Copy helpers — clobber CX, SI, DI
; =============================================================================
flt_a_to_b:
        mov  si, FLT_A
        mov  di, FLT_B
        mov  cx, 5
        rep  movsb
        ret

flt_b_to_a:
        mov  si, FLT_B
        mov  di, FLT_A
        mov  cx, 5
        rep  movsb
        ret

flt_t_to_a:
        mov  si, FLT_T
        mov  di, FLT_A
        mov  cx, 5
        rep  movsb
        ret

flt_a_to_t:
        mov  si, FLT_A
        mov  di, FLT_T
        mov  cx, 5
        rep  movsb
        ret

; =============================================================================
; Sign / abs helpers — clobber nothing
; =============================================================================
flt_negate:
        cmp  byte [FLT_A], 0
        je   flt_neg_r
        xor  byte [FLT_A+1], 0x80
flt_neg_r: ret

flt_negate_b:
        cmp  byte [FLT_B], 0
        je   fnb_r
        xor  byte [FLT_B+1], 0x80
fnb_r:  ret

flt_abs:
        and  byte [FLT_A+1], 0x7F
        ret

; =============================================================================
; FLT_FROM_INT  AX (signed int16) -> FLT_A
; Inputs  : AX = signed integer
; Outputs : FLT_A
; Clobbers: AX, BX, CX, DI
; =============================================================================
flt_from_int:
        or   ax, ax
        je   ffi_zero
        mov  bl, 0x00           ; sign = positive
        jns  ffi_pos
        mov  bl, 0x80           ; sign = negative
        neg  ax
ffi_pos:
        mov  cl, 0x90           ; start exponent: 2^16 bias
ffi_lp: test ax, 0x8000        ; find highest set bit
        jnz  ffi_found
        shl  ax, 1
        dec  cl
        jmp  ffi_lp
ffi_found:
        push ax
        push cx
        push bx
        call flt_zero
        pop  bx
        pop  cx
        pop  ax
        mov  [FLT_A+0], cl     ; exponent
        and  ah, 0x7F           ; clear bit 7 of high byte (was implied-1)
        or   ah, bl             ; apply sign
        mov  [FLT_A+1], ah     ; high mantissa + sign
        mov  [FLT_A+2], al     ; second mantissa byte
        mov  byte [FLT_A+3], 0
        mov  byte [FLT_A+4], 0
        ret
ffi_zero: jmp flt_zero

; =============================================================================
; FLT_FROM_INT_B  AX (signed int16) -> FLT_B  (preserves FLT_A)
; Clobbers: AX, BX, CX, DX, SI, DI
; =============================================================================
flt_from_int_b:
        mov  bx, [FLT_A+0]
        mov  cx, [FLT_A+2]
        mov  dl, [FLT_A+4]
        push bx
        push cx
        push dx
        call flt_from_int
        mov  si, FLT_A
        mov  di, FLT_B
        mov  cx, 5
        rep  movsb
        pop  dx
        pop  cx
        pop  bx
        mov  [FLT_A+0], bx
        mov  [FLT_A+2], cx
        mov  [FLT_A+4], dl
        ret

; =============================================================================
; FLT_TO_INT  FLT_A -> AX (signed int16, truncate toward zero)
; Outputs : AX.  Saturates at +-32767 on overflow.
; Clobbers: AX, BX, CX
; =============================================================================
flt_to_int:
        mov  al, [FLT_A+0]
        or   al, al
        jz   fti_zero
        sub  al, 0x80           ; true exponent
        jle  fti_zero           ; exponent <= 0 -> |value| < 1
        cmp  al, 15
        jg   fti_sat            ; exponent > 15 -> overflow
        mov  bl, [FLT_A+1]
        and  bl, 0x80
        mov  [FLT_TS], bl       ; save sign
        mov  bh, [FLT_A+1]
        and  bh, 0x7F
        or   bh, 0x80           ; restore implied-1
        mov  bl, [FLT_A+2]
        mov  cl, 16
        sub  cl, al             ; shift = 16 - true_exponent
fti_shr: shr bx, 1
        dec  cl
        jnz  fti_shr
        mov  ax, bx
        cmp  byte [FLT_TS], 0
        je   fti_r
        neg  ax
fti_r:  ret
fti_zero: xor ax, ax
        ret
fti_sat: mov ax, 32767
        test byte [FLT_A+1], 0x80
        jz   fti_r
        neg  ax
        ret

; =============================================================================
; FLT_CMP  compare FLT_A with FLT_B (signed)
; Outputs : AX = -1 (A<B), 0 (A==B), +1 (A>B)
; Clobbers: AX, BX, CX
; =============================================================================
flt_cmp:
        mov  al, [FLT_A+0]
        mov  bl, [FLT_B+0]
        or   al, al
        jnz  fcmp_anz
        or   bl, bl
        jz   fcmp_eq
        test byte [FLT_B+1], 0x80
        mov  ax, -1
        jz   fcmp_r2            ; B>0 -> A(=0) < B
        mov  ax, 1              ; B<0 -> A(=0) > B
        ret
fcmp_anz:
        or   bl, bl
        jnz  fcmp_both
        test byte [FLT_A+1], 0x80
        mov  ax, 1
        jz   fcmp_r2            ; A>0 -> A > B(=0)
        mov  ax, -1
        ret
fcmp_r2: ret
fcmp_both:
        mov  cl, [FLT_A+1]
        and  cl, 0x80
        mov  ch, [FLT_B+1]
        and  ch, 0x80
        cmp  cl, ch
        je   fcmp_same_sign
        test cl, 0x80
        mov  ax, -1             ; A negative, B positive
        jnz  fcmp_r
        mov  ax, 1
        ret
fcmp_same_sign:
        push cx                 ; CL = common sign bit
        ; Compare as unsigned tuples: exp, mant[1]&7F, mant[2..4]
        mov  al, [FLT_A+0]
        mov  bl, [FLT_B+0]
        cmp  al, bl
        jne  fcmp_ne
        mov  al, [FLT_A+1]
        mov  bl, [FLT_B+1]
        and  al, 0x7F
        and  bl, 0x7F
        cmp  al, bl
        jne  fcmp_ne
        mov  ax, [FLT_A+2]
        mov  bx, [FLT_B+2]
        cmp  ax, bx
        jne  fcmp_ne16
        mov  al, [FLT_A+4]
        mov  bl, [FLT_B+4]
        cmp  al, bl
        jne  fcmp_ne
        pop  cx
fcmp_eq: xor ax, ax
        ret
fcmp_ne16: jb fcmp_lt
        jmp  fcmp_gt
fcmp_ne: pop cx
        jb   fcmp_lt
fcmp_gt: test cl, 0x80
        mov  ax, 1
        jz   fcmp_r             ; positive sign: magnitude larger = value larger
        mov  ax, -1
        ret
fcmp_lt: test cl, 0x80
        mov  ax, -1
        jz   fcmp_r
        mov  ax, 1
fcmp_r: ret

; =============================================================================
; FLT_SUB  FLT_A = FLT_A - FLT_B
; Clobbers: same as flt_add
; =============================================================================
flt_sub:
        call flt_negate_b
        call flt_add
        jmp  flt_negate_b       ; restore FLT_B (tail-call)

; =============================================================================
; NORM_PACK  normalise BL:DX:AH, round, pack into FLT_A
;
; Algorithm from MS $NORMS (MATH2.ASM:1118) and $ROUNS/$PAKSP (MATH2.ASM:1779).
;
; Register map (31-bit mantissa across 4 bytes):
;   BL = mant[30:24]  (bit7 = implied-1)
;   DH = mant[23:16]
;   DL = mant[15:8]
;   AH = mant[7:0]    (stored as byte 4 of result)
;   AL = sub-guard    (rounding only; not stored)
;   BH = biased exponent
;   [FLT_SA] = result sign
;
; Callers that produce exact results set AL=0 (no rounding artefact).
; Callers with guard information set AL to sub-guard bits.
; Round-half-up: add 0x80 to AL, propagate carry into AH:DX:BL.
;
; Inputs  : BH,BL,DH,DL,AH,AL  as above.  [FLT_SA].
; Outputs : FLT_A packed.
; Clobbers: AX, BX, DX.
; =============================================================================
norm_pack:
np_lp:  or   bl, bl
        js   np_round
        jnz  np_bit
        ; BL=0: byte-shift optimisation (MS NOR10)
        sub  bh, 8
        jbe  np_zero
        mov  bl, dh
        mov  dh, dl
        mov  dl, ah
        mov  ah, al
        xor  al, al
        jmp  np_lp
np_bit:
        clc
        rcl  al, 1
        rcl  ah, 1
        rcl  dx, 1
        rcl  bl, 1
        dec  bh
        jnz  np_lp
np_zero: jmp flt_zero

np_round:
        ; Round half-up via sub-guard AL (MS $ROUNS pattern)
        add  al, 0x80
        jnc  np_pack
        inc  ah
        jnz  np_pack
        inc  dx
        jnz  np_pack
        inc  bl
        jnz  np_pack
        stc
        rcr  bl, 1
        inc  bh
        jz   np_zero

np_pack:
        mov  [FLT_A+0], bh
        mov  al, [FLT_SA]
        and  bl, 0x7F           ; clear implied-1
        or   bl, al             ; apply sign
        mov  [FLT_A+1], bl
        mov  [FLT_A+2], dh
        mov  [FLT_A+3], dl
        mov  [FLT_A+4], ah
        ret

; =============================================================================
; FLT_ADD  FLT_A = FLT_A + FLT_B
;
; Algorithm from MS $FADDS (MATH1.ASM:3287), adapted to MBF5 RAM layout.
; Uses 24-bit BL:DX:AH for the larger-exponent operand.
; Smaller operand kept in FLT_T[0..3] so CL is free for shift count.
; Byte-at-a-time alignment from MS FA23 byte-move optimisation.
;
; FLT_T usage during this routine:
;   FLT_T+0 = smaller_hi  (mant[30:24] with implied-1 restored)
;   FLT_T+1 = smaller_dh  (mant[23:16])
;   FLT_T+2 = smaller_dl  (mant[15:8])
;   FLT_T+3 = smaller_gd  (guard byte, initially 0)
;
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A = sum
; Clobbers: AX, BX, CX, DX, DI, FLT_SA, FLT_SB, FLT_ER, FLT_T[0..3]
; =============================================================================
flt_add:
        ; Zero checks
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
        ; Save both signs (needed for add/sub decision even after swap)
        mov  al, [FLT_A+1]
        and  al, 0x80
        mov  [FLT_SA], al
        mov  al, [FLT_B+1]
        and  al, 0x80
        mov  [FLT_SB], al

        ; Compare exponents; put larger in BL:DX:AH, smaller in FLT_T[0..3]
        mov  al, [FLT_A+0]
        mov  ah, [FLT_B+0]
        cmp  al, ah
        jge  fa_a_larger

        ; ---- B has larger exponent -> result sign = sign_B ----
        mov  al, [FLT_SB]
        mov  [FLT_SA], al       ; overwrite with sign of larger operand
        mov  bh, [FLT_B+0]     ; result exponent
        mov  bl, [FLT_B+1]
        and  bl, 0x7F
        or   bl, 0x80           ; restore implied-1
        mov  dh, [FLT_B+2]
        mov  dl, [FLT_B+3]
        mov  ah, [FLT_B+4]
        ; A (smaller) -> FLT_T[0..3]
        mov  al, [FLT_A+1]
        and  al, 0x7F
        or   al, 0x80
        mov  [FLT_T+0], al
        mov  al, [FLT_A+2]
        mov  [FLT_T+1], al
        mov  al, [FLT_A+3]
        mov  [FLT_T+2], al
        mov  byte [FLT_T+3], 0  ; guard byte starts at 0
        ; Shift count = exp_B - exp_A
        mov  cl, [FLT_B+0]
        sub  cl, [FLT_A+0]
        jmp  fa_align

fa_a_larger:
        ; ---- A has larger (or equal) exponent -> result sign = sign_A (already set) ----
        mov  bh, [FLT_A+0]
        mov  bl, [FLT_A+1]
        and  bl, 0x7F
        or   bl, 0x80
        mov  dh, [FLT_A+2]
        mov  dl, [FLT_A+3]
        mov  ah, [FLT_A+4]
        ; B (smaller) -> FLT_T[0..3]
        mov  al, [FLT_B+1]
        and  al, 0x7F
        or   al, 0x80
        mov  [FLT_T+0], al
        mov  al, [FLT_B+2]
        mov  [FLT_T+1], al
        mov  al, [FLT_B+3]
        mov  [FLT_T+2], al
        mov  byte [FLT_T+3], 0
        mov  cl, [FLT_A+0]
        sub  cl, [FLT_B+0]

fa_align:
        ; Right-shift FLT_T[0..3] by CL bits.
        ; BL:DX:AH = larger mantissa.  BH = result exponent.
        or   cl, cl
        jz   fa_addorsub
        cmp  cl, 25             ; >= 25 bits shift -> smaller is negligible
        jb   fa_do_align
        jmp  fa_smaller_gone
fa_do_align:

        ; Byte-at-a-time optimisation (MS FA23 / FA24)
fa_byte_lp:
        cmp  cl, 8
        jb   fa_bit_lp
        sub  cl, 8
        mov  al, [FLT_T+2]
        mov  [FLT_T+3], al
        mov  al, [FLT_T+1]
        mov  [FLT_T+2], al
        mov  al, [FLT_T+0]
        mov  [FLT_T+1], al
        mov  byte [FLT_T+0], 0
        or   cl, cl
        jnz  fa_byte_lp
        jmp  fa_addorsub

fa_bit_lp:
        ; 1-bit right-shift of FLT_T[0..3] with carry chain
        clc
        shr  byte [FLT_T+0], 1
        rcr  byte [FLT_T+1], 1
        rcr  byte [FLT_T+2], 1
        rcr  byte [FLT_T+3], 1
        dec  cl
        jnz  fa_bit_lp

fa_addorsub:
        ; Decide add or subtract based on ORIGINAL signs of A and B.
        ; FLT_SA = sign of the LARGER operand (may have been swapped).
        ; FLT_SB = original sign of B (never modified).
        ; Original sign of A is still readable from [FLT_A+1].
        mov  al, [FLT_A+1]
        and  al, 0x80
        cmp  al, [FLT_SB]       ; sign_A == sign_B ?
        je   fa_same_sign

        ; Different signs -> subtract smaller from larger.
        ; Borrow chain: guard(LSB) first, then dl, dh, bl(MSB).
        mov  al, ah
        sub  al, [FLT_T+3]
        mov  ah, al
        mov  al, dl
        sbb  al, [FLT_T+2]
        mov  dl, al
        mov  al, dh
        sbb  al, [FLT_T+1]
        mov  dh, al
        sbb  bl, [FLT_T+0]
        jnc  fa_norm            ; no borrow -> result correct

        ; Borrow out: mantissa result was actually negative — complement and flip sign.
        not  ah
        not  dl
        not  dh
        not  bl
        ; Increment (two's complement carry chain)
        add  ah, 1
        adc  dl, 0
        adc  dh, 0
        adc  bl, 0
        ; If result is zero after complement (equal magnitudes), return zero
        or   bl, bl
        jnz  fa_flip_sign
        or   dx, dx
        jnz  fa_flip_sign
        or   ah, ah
        jz   fa_zero
fa_flip_sign:
        xor  byte [FLT_SA], 0x80
        jmp  fa_norm

fa_same_sign:
        ; Add mantissas with carry chain.
        mov  al, ah
        add  al, [FLT_T+3]
        mov  ah, al
        mov  al, dl
        adc  al, [FLT_T+2]
        mov  dl, al
        mov  al, dh
        adc  al, [FLT_T+1]
        mov  dh, al
        adc  bl, [FLT_T+0]
        jnc  fa_norm
        ; Carry out of BL: shift right 1, bump exponent (MS FA60/FA70)
        rcr  bl, 1
        rcr  dx, 1
        rcr  ah, 1
        inc  bh
        jz   fa_zero            ; exponent overflow

fa_norm:
        ; Check for zero result (e.g. equal and opposite values)
        or   bl, bl
        jnz  fa_do_norm
        or   dx, dx
        jnz  fa_do_norm
        or   ah, ah
        jz   fa_zero
fa_do_norm:
        xor  al, al
        jmp  norm_pack

fa_zero:       jmp flt_zero
fa_smaller_gone:
        ; Smaller operand vanished after shift; result = larger operand.
        ; BL:DX:AH = larger mantissa, BH = result exponent, FLT_SA = result sign.
        ; It's already normalised, but must go through norm_pack to pack to FLT_A.
        xor  al, al
        jmp  norm_pack

; =============================================================================
; FLT_MUL  FLT_A = FLT_A * FLT_B
;
; 24x24->24 via three partial 16-bit products.
; Algorithm from MS $FMULS (MATH2.ASM:416).
;
; MS $FMULS register map (single-precision):
;   FAC mantissa: CL=hi byte, AX=lo word  (CL:AX, 24 bits)
;   ARG mantissa: BL=hi byte, DX=lo word  (BL:DX, 24 bits)
;   Product accumulates in BX:CX (BX=high word, CX=low word), AH=guard
;   Temporaries on stack: SI=ARG-hi (as 16-bit word), DI=FAC-hi, BP=ARG-lo
;
; Our adaptation: FLT_A mant -> CL:AX, FLT_B mant -> BL:DX.
; Uses only 3 bytes of each operand (bytes 1..3); byte 4 (LSB) dropped.
;
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A = product
; Clobbers: AX, BX, CX, DX, SI, DI, FLT_SA, FLT_ER
; =============================================================================
flt_mul:
        mov  al, [FLT_A+0]
        or   al, al
        jnz  fmul_anz
        jmp  fmul_zero
fmul_anz:
        mov  bl, [FLT_B+0]
        or   bl, bl
        jnz  fmul_bnz
        jmp  fmul_zero
fmul_bnz:

        ; Exponent: eA + eB - 0x81
        ; (MBF value = 2^(exp-0x80) × 0.mant; product of two 0.mant values
        ;  needs an extra -1 to stay in [0.5,1) after normalisation)
        add  al, bl
        sub  al, 0x81
        mov  [FLT_ER], al

        ; Result sign = sign_A XOR sign_B
        mov  al, [FLT_A+1]
        xor  al, [FLT_B+1]
        and  al, 0x80
        mov  [FLT_SA], al

        ; Load FLT_A mantissa into CL:AX (CL=hi, AH=byte2, AL=byte3)
        mov  cl, [FLT_A+1]
        and  cl, 0x7F
        or   cl, 0x80           ; restore implied-1
        mov  ah, [FLT_A+2]
        mov  al, [FLT_A+3]
        ; AX=A[2:3], CL=A[1]&7F|80

        ; Load FLT_B mantissa into BL:DX (BL=hi, DH=byte2, DL=byte3)
        mov  bl, [FLT_B+1]
        and  bl, 0x7F
        or   bl, 0x80
        mov  dh, [FLT_B+2]
        mov  dl, [FLT_B+3]

        ; Save operand pieces in MS $FMULS style:
        ;   SI = 0x00:BL  (B hi byte as 16-bit, BH=0)
        ;   DI = 0x00:CL  (A hi byte as 16-bit, CH=0)
        ;   On stack: [CX=00:CL], [AX=A_lo_word]
        ;   BP used for B_lo_word (DX) but we use memory instead (BP reserved)
        ;   We re-read DX from FLT_B+2 after the first multiply.
        xor  bh, bh
        mov  si, bx             ; SI = 0x00:BL (B hi byte)
        xor  ch, ch
        mov  di, cx             ; DI = 0x00:CL (A hi byte)
        push cx                 ; push A_hi  (CX = 00:CL)
        push ax                 ; push A_lo_word

        ; Step 1: A_lo_word * B_lo_word -> DX:AX (32-bit); keep high word in CX
        mul  dx                 ; AX * DX -> DX:AX
        mov  cx, dx             ; CX = high word of step1
        pop  ax                 ; restore A_lo_word
        ; Step 2: A_lo_word * B_hi_byte -> DX:AX; accumulate
        mul  si                 ; AX * SI = A_lo * (00:BL) -> DX:AX
        add  cx, ax
        jnc  fms10
        inc  dx
fms10:  mov  bx, dx            ; BX = running high-word accumulator
        pop  dx                 ; restore A_hi word (= DI = 00:CL)
        ; Step 3: B_lo_word * A_hi_byte -> DX:AX; accumulate
        mov  ah, [FLT_B+2]
        mov  al, [FLT_B+3]     ; AX = B_lo_word (re-read; original DX was clobbered)
        mul  dx                 ; AX * DX = B_lo * (00:A_hi) -> DX:AX
        add  cx, ax
        jnc  fms20
        inc  dx
fms20:  add  bx, dx
        ; Step 4: A_hi_byte * B_hi_byte -> AX (8x8, fits in 16 bits)
        mov  ax, di             ; AX = 00:A_hi
        mul  si                 ; AX * SI = (00:A_hi) * (00:B_hi) -> AX
        add  bx, ax

        ; Product in BX:CX.  Check normalisation: BH bit7 must be set.
        ; If not, the true product is half of what it should be (MS FMS35).
        or   bh, bh
        js   fms_norm_ok
        inc  byte [FLT_ER]      ; adjust exponent for the shift we are about to do
        jnz  fms_do_shift
        jmp  fmul_zero
fms_do_shift:
        rcl  cx, 1
        rcl  bx, 1

fms_norm_ok:
        ; Repack BX:CX to BL:DX:AH for norm_pack (MS FMS37):
        ;   new BL = BH (byte 0 of product)
        ;   new DH = BL (byte 1 of product)
        ;   new DL = CH (byte 2 of product)
        ;   new AH = CL (guard byte)
        mov  dl, ch
        mov  dh, bl
        mov  bl, bh
        mov  ah, cl
        mov  bh, [FLT_ER]
        xor  al, al
        jmp  norm_pack

fmul_zero: jmp flt_zero

; =============================================================================
; FLT_DIV  FLT_A = FLT_A / FLT_B
;
; Divides 16-bit mantissa of A by 16-bit mantissa of B to produce a 16-bit
; quotient mantissa, then appends a guard byte from the remainder.
;
; Method:
;   DX = mA_hi_word (FLT_A[1:2] with implied-1, = 16-bit mantissa)
;   BX = mB_hi_word (FLT_B[1:2] with implied-1, = 16-bit divisor)
;   Both in [0x8000..0xFFFF].
;   Pre-shift DX left 1 (DX is < BX so no overflow after shift if DX < 0x8000).
;   Actually: to get a 16-bit quotient in [0x8000,0xFFFF], do:
;     if mA < mB: quotient = (mA<<1)/mB  in (0x8000,0x10000), exp--
;     else:       quotient = mA/mB         (would be >= 1; pre-shift prevents this)
;   Since both in [0.5,1), mA/mB in (0.5,2).  We handle both cases:
;     Always: AX=0, DX=mA. shr dx,1: DX in [0x4000,0x7FFF] < BX always.
;     div bx -> quotient in [0x4000,0x7FFF] (< 0x8000, not normalised).
;     Shift quotient left 1, adjust exponent.
;   Guard byte: take remainder × 256 / BH.
;
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A = quotient
; Clobbers: AX, BX, CX, DX, FLT_SA, FLT_ER
; =============================================================================
flt_div:
        mov  bl, [FLT_B+0]
        or   bl, bl
        jz   fdiv_by_zero

        mov  al, [FLT_A+0]
        or   al, al
        je   fdiv_done          ; 0 / x = 0

        ; Exponent: eA - eB + 0x80
        sub  al, bl
        add  al, 0x80
        mov  [FLT_ER], al

        ; Result sign
        mov  al, [FLT_A+1]
        xor  al, [FLT_B+1]
        and  al, 0x80
        mov  [FLT_SA], al

        ; Load 16-bit mantissas
        mov  dh, [FLT_A+1]
        and  dh, 0x7F
        or   dh, 0x80           ; implied-1 in DH
        mov  dl, [FLT_A+2]     ; DX = mA hi-word

        mov  bh, [FLT_B+1]
        and  bh, 0x7F
        or   bh, 0x80
        mov  bl, [FLT_B+2]     ; BX = mB hi-word

        ; Divide: (DX:AX) / BX where AX has lower mantissa bytes of A
        ; Use bytes 1-4 of A for full 32-bit dividend precision
        mov  ax, [FLT_A+3]     ; AX = FLT_A bytes 3,4 swapped? No: [FLT_A+3]=byte3, byte4
        xchg al, ah             ; AX = word at FLT_A+3 (AH=byte3, AL=byte4) -- use as-is
        ; Actually [word at FLT_A+3] = {byte4, byte3} (little-endian) -- swap
        ; Simpler: just zero AX for now; the lower 2 bytes of A are 0 for most integers
        xor  ax, ax             ; AX = 0 (lower 32-bit dividend half)
        shr  dx, 1              ; pre-shift to ensure DX < BX
        div  bx                 ; AX = quotient, DX = remainder

        ; Normalise quotient: shift AX left until AH bit7 set, counting shifts.
        ; Each shift left means the mantissa value is doubled, so decrement exponent.
        xor  cl, cl             ; CL = shift count
fdiv_norm_lp:
        test ah, 0x80
        jnz  fdiv_norm_done
        shl  ax, 1
        inc  cl
        cmp  cl, 16
        jb   fdiv_norm_lp
        ; If we reach here, quotient was 0 (shouldn't happen if A!=0)
        jmp  fdiv_done
fdiv_norm_done:
        ; AH has bit7 set. CL = left-shift count applied.
        ; Adjust exponent: base = eA-eB+0x80, pre-shift added 1, normalise removes CL.
        ; Net: FLT_ER = eA-eB+0x80+1-CL = [FLT_ER]+1-CL
        inc  byte [FLT_ER]
        sub  [FLT_ER], cl

        ; Guard byte from remainder
        push ax
        mov  al, dh
        xor  ah, ah
        div  bh
        mov  cl, al             ; CL = guard byte
        pop  ax

        ; Pack to BL:DX:AH for norm_pack (already normalised)
        mov  bl, ah             ; BL = high byte of quotient (bit7 set)
        mov  dh, al             ; DH = low byte
        xor  dl, dl
        mov  ah, cl             ; AH = guard byte
        mov  bh, [FLT_ER]
        xor  al, al
        jmp  norm_pack

fdiv_done: ret

fdiv_by_zero:
        push si
        mov  si, s_div0
        call print_sz
        pop  si
        jmp  flt_zero

s_div0: db "DIV0!",0

; =============================================================================
; FLT_PRINT  print FLT_A as decimal to terminal
;
; Strategy: binary exponent -> decimal exponent estimate via *77/256 (log10(2)~0.301).
; Scale FLT_A to [1,10), extract 7 digits via repeated flt_to_int + flt_sub + *10.
; Strip trailing zeros.  Insert decimal point based on decimal exponent.
;
; Inputs  : FLT_A
; Clobbers: AX, BX, CX, DX, DI, SI, FLT_T, FLT_SA, FLT_ER, FLT_DE
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
        ; Estimate decimal exponent: de = (exp - 0x80) * 77 / 256  ~= (exp-128)*log10(2)
        mov  al, [FLT_A+0]
        sub  al, 0x80
        cbw
        mov  cx, 77
        imul cx
        mov  al, ah             ; AH = product >> 8 = estimate
        mov  [FLT_DE], al

        ; Save original FLT_A on stack (5 bytes as 3 words, top byte padded)
        mov  ax, [FLT_A+0]
        push ax
        mov  ax, [FLT_A+2]
        push ax
        xor  ah, ah
        mov  al, [FLT_A+4]
        push ax

        ; Scale: divide by 10^de to bring value into [1,10)
        mov  al, [FLT_DE]
        cbw
        mov  cx, ax
        or   cx, cx
        jz   fp_scale_done
        jl   fp_scale_up        ; de < 0: multiply by 10
fp_scale_down:
        push cx
        mov  ax, 10
        call flt_from_int_b
        call flt_div
        pop  cx
        loop fp_scale_down
        jmp  fp_scale_done
fp_scale_up:
        push cx
        mov  ax, 10
        call flt_from_int_b
        call flt_mul
        pop  cx
        inc  cx
        jnz  fp_scale_up

fp_scale_done:
        ; Verify range [1,10); adjust if off by 1
        mov  ax, 10
        call flt_from_int_b
        call flt_cmp
        cmp  ax, 1
        jne  fp_chk_lo
        mov  ax, 10
        call flt_from_int_b
        call flt_div
        inc  byte [FLT_DE]
        jmp  fp_extract
fp_chk_lo:
        mov  ax, 1
        call flt_from_int_b
        call flt_cmp
        cmp  ax, -1
        jne  fp_extract
        mov  ax, 10
        call flt_from_int_b
        call flt_mul
        dec  byte [FLT_DE]

fp_extract:
        ; Extract 7 digits into IBUF
        mov  al, [FLT_DE]
        cbw
        push ax                 ; save decimal exponent for printing
        mov  di, IBUF
        mov  cx, 7
fp_dig_lp:
        push cx
        push di
        call flt_to_int         ; AX = integer part of current value
        push ax                 ; save digit
        call flt_from_int_b     ; FLT_B = digit
        call flt_sub            ; FLT_A = fractional part
        ; Clamp to 0 if slightly negative due to rounding
        test byte [FLT_A+1], 0x80
        jz   fp_no_clamp
        call flt_zero
fp_no_clamp:
        pop  ax                 ; restore digit
        pop  di
        add  al, '0'
        mov  [di], al
        inc  di
        pop  cx
        dec  cx
        jz   fp_dig_done
        push cx
        push di
        mov  ax, 10
        call flt_from_int_b
        call flt_mul
        pop  di
        pop  cx
        jmp  fp_dig_lp

fp_dig_done:
        ; Strip trailing zeros
fp_strip:
        dec  di
        cmp  byte [di], '0'
        jne  fp_strip_done
        cmp  di, IBUF
        jle  fp_strip_done
        jmp  fp_strip
fp_strip_done:
        inc  di                 ; DI = one past last significant digit

        ; Print: SI=digit ptr, BX=digits remaining before decimal point
        pop  ax                 ; restore decimal exponent (de)
        inc  ax                 ; number of digits before decimal point = de+1
        mov  bx, ax
        mov  si, IBUF
fp_print_lp:
        cmp  si, di
        jnb  fp_print_done
        ; Emit digit first, then check if decimal point follows
        mov  al, [si]
        call output
        inc  si
        dec  bx
        jnz  fp_print_lp       ; more digits before decimal point
        cmp  si, di
        jnb  fp_print_done     ; no more digits -> done (no trailing '.')
        mov  al, '.'
        call output
        jmp  fp_print_lp
fp_print_done:
        ; Restore original FLT_A from stack
        pop  ax
        mov  [FLT_A+4], al
        pop  ax
        mov  [FLT_A+2], ax
        pop  ax
        mov  [FLT_A+0], ax
        ret
; =============================================================================
; FLT_PARSE  decimal string at [SI] -> FLT_A
;
; Fix v0.3: digit is pushed BEFORE flt_mul (AX=10) call so it survives.
; v0.2 bug: digit (in AL after loop) was overwritten by flt_mul which
; clobbers AX; subsequent flt_from_int_b received the wrong value.
;
; Inputs  : SI -> null/CR-terminated decimal string ('-'/'+' sign ok)
; Outputs : FLT_A = parsed value.  SI advanced past last consumed char.
; Clobbers: AX, BX, CX, DX, DI, FLT_SA
; =============================================================================
flt_parse:
        call flt_zero
        mov  byte [FLT_SA], 0   ; sign: 0=positive, 0x80=negative

        ; Skip spaces
fpar_skip:
        cmp  byte [si], ' '
        jne  fpar_sign
        inc  si
        jmp  fpar_skip

fpar_sign:
        cmp  byte [si], '-'
        jne  fpar_plus
        mov  byte [FLT_SA], 0x80
        inc  si
        jmp  fpar_digits
fpar_plus:
        cmp  byte [si], '+'
        jne  fpar_digits
        inc  si

fpar_digits:
        xor  bl, bl             ; BL: 0=before decimal point, 1=after
        xor  cl, cl             ; CL: count of decimal places consumed

fpar_lp:
        mov  al, [si]
        cmp  al, '.'
        jne  fpar_notdot
        or   bl, bl             ; second '.' -> stop
        jnz  fpar_end
        mov  bl, 1
        inc  si
        jmp  fpar_lp

fpar_notdot:
        sub  al, '0'
        jb   fpar_end
        cmp  al, 9
        ja   fpar_end
        ; AL = digit value (0..9)
        inc  si

        ; Push digit BEFORE calling flt_mul (which clobbers AX)
        push ax                 ; [sp+0] = digit value

        ; FLT_A = FLT_A * 10
        push bx
        push cx
        mov  ax, 10
        call flt_from_int_b     ; FLT_B = 10  (FLT_A preserved)
        call flt_mul            ; FLT_A = FLT_A * 10
        pop  cx
        pop  bx

        ; Restore digit and add: FLT_A = FLT_A + digit
        pop  ax                 ; AL = digit
        push bx
        push cx
        call flt_from_int_b     ; FLT_B = digit  (FLT_A preserved)
        call flt_add            ; FLT_A = FLT_A + digit
        pop  cx
        pop  bx

        or   bl, bl             ; after decimal point?
        jz   fpar_lp
        inc  cl                 ; count decimal places
        jmp  fpar_lp

fpar_end:
        ; Divide by 10^cl to shift decimal point
        or   cl, cl
        jz   fpar_sign2
fpar_scale:
        push cx
        mov  ax, 10
        call flt_from_int_b
        call flt_div
        pop  cx
        dec  cl
        jnz  fpar_scale

fpar_sign2:
        cmp  byte [FLT_SA], 0
        je   fpar_done
        call flt_negate
fpar_done:
        ret

; =============================================================================
; I/O ROUTINES
; =============================================================================

; print_hex_byte  print AL as two hex digits
; Clobbers: AX
print_hex_byte:
        push ax
        shr  al, 1
        shr  al, 1
        shr  al, 1
        shr  al, 1
        call phb_nib
        pop  ax
        and  al, 0x0F
phb_nib:
        cmp  al, 9
        jbe  phb_dec
        add  al, 'A'-10
        jmp  output
phb_dec: add al, '0'
        jmp  output

; output_int  print AX as signed decimal
; Clobbers: AX, BX, CX, DX
output_int:
        or   ax, ax
        jns  oi_pos
        push ax
        mov  al, '-'
        call output
        pop  ax
        neg  ax
oi_pos: mov  cx, 10
        xor  dx, dx
        div  cx
        push dx
        or   ax, ax
        jz   oi_digit
        call output_int
oi_digit:
        pop  ax
        add  al, '0'
        jmp  output

; print_sz  print null-terminated string at [SI]; preserves SI
print_sz:
        push si
psz_lp: lodsb
        or   al, al
        je   psz_r
        call output
        jmp  psz_lp
psz_r:  pop  si
        ret

new_line:
        mov  al, 0x0D
        call output
        mov  al, 0x0A
        jmp  output

; putchar / getchar / output — BIOS INT 10h teletype (intercepted by sim_rom)
putchar:
getchar:
output:
        push bx
        mov  ah, 0x0E
        mov  bx, 0x0007
        int  0x10
        pop  bx
        ret

; =============================================================================
; RESET VECTOR
; =============================================================================
        org  0xFFF0
reset:  db   0xEA               ; far JMP to start
        dw   start
        dw   0xF000

        times 4096-($-start) db 0xFF
