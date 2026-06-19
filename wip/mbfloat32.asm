; =============================================================================
; MiniBASIC8088  MBF4 Float Library
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Usage: tinyasm -f bin mbfloat_v07.asm -o mbfloat_v07.bin
;        sim_rom mbfloat_v07.asm
;
; 4-byte Microsoft Binary Format (single-precision) floating-point routines
; for 8088/8086.  Standalone test harness: 10 tests.
;
; ---------------------------------------------------------------------------
; VERSION HISTORY
; ---------------------------------------------------------------------------
; v0.6 : Initial MBF4 port from MBF5 (v0.5).
; v0.7 : Size optimisation pass (S1-S26) + wrap-fix.
;        S1  : flt_zero_b deleted.
;        S2  : flt_zero uses 2x mov word; removes CX/DI clobber.
;        S4  : flt_t_to_a / flt_a_to_t deleted (unused).
;        S5  : flt_from_int: direct stores, no push/call flt_zero/pop.
;        S6  : flt_from_int_b: push/pop [FLT_A+n] word saves 6 bytes.
;        S8  : flt_to_int: FLT_TS eliminated; sign saved to DL.
;        S15 : flt_add same-sign carry-out: drop rcr al,1 (no byte4).
;        S18 : flt_mul: correctness fix + clean 4-step plan (mirrors v0.5).
;        S19 : flt_div: fdiv_restore/fdiv_restore2 merged.
;        S20 : flt_div: fdiv_ov falls into fdiv_do_sub.
;        S22 : flt_print: pop [FLT_A+n] word for restore.
;        S26 : flt_parse: tail-call flt_negate at fpar_sign2.
;        WRAP: Reordered sections so all inter-routine JMPs are backward
;              short jumps within the 0xF000-0xFFFF window.  The helpers
;              (flt_zero, norm_pack, flt_negate, flt_b_to_a etc.) are placed
;              first so callers in flt_add/flt_mul/flt_div always jump back
;              to lower addresses, staying inside the ROM page.
;
; ---------------------------------------------------------------------------
; MBF4 FORMAT
; ---------------------------------------------------------------------------
; Byte 0   : biased exponent.  0x00 = exact zero.
;            Stored exponent = true_exponent + 0x80.
;            Value = (-1)^sign * 2^(exp-0x80) * 0.1mmm...
; Byte 1   : bit7=sign, bits6:0 = mant[22:16]
; Byte 2   : mant[15:8]
; Byte 3   : mant[7:0]
; Implied leading 1 at bit23 (restored during arithmetic).
; Verified: 1.0={81,00,00,00} 5.0={83,20,00,00}
;           10.0={84,20,00,00} 355.0={89,31,80,00}
;
; ---------------------------------------------------------------------------
; CALLING CONVENTION
; ---------------------------------------------------------------------------
; FLT_A (4 bytes) = primary operand and result
; FLT_B (4 bytes) = secondary operand
; FLT_T (3 bytes) = flt_add smaller-operand scratch
; FLT_SA          = result sign (0x00 or 0x80)
; FLT_SB          = sign of B (flt_add)
; FLT_ER          = result exponent (flt_mul / flt_div)
; FLT_DE          = flt_print decimal exponent
; FLT_SP (2 bytes)= flt_parse string pointer save
; FLT_DB (3 bytes)= flt_div B-mantissa spill
; AX = integer in/out.  SI preserved by all routines that clobber it.
;
; NOTE ON LAYOUT: Small helpers (flt_zero, norm_pack, flt_negate, flt_b_to_a,
; flt_abs, copy helpers) are placed FIRST so that all JMPs from the arithmetic
; routines are backward jumps to lower addresses.  With org 0xF000 a forward
; JMP of >0x1000 bytes would wrap around into RAM; backward JMPs to addresses
; above 0xF000 are always safe.
;
; ---------------------------------------------------------------------------
; BUILD
; ---------------------------------------------------------------------------
;   tinyasm -f bin mbfloat_v07.asm -o mbfloat_v07.bin
;   sim_rom mbfloat_v07.asm
; =============================================================================

        cpu  8086
        org  0xF000
        jmp  start              ; sim_rom enters at load address (0xF000); skip library

; =============================================================================
; RAM LAYOUT
; =============================================================================
STACK_TOP: equ 0x0800
FLT_A:  equ 0x00C0              ; 4 bytes: primary operand / result
FLT_B:  equ 0x00C4              ; 4 bytes: secondary operand
FLT_T:  equ 0x00C8              ; 3 bytes: flt_add smaller-operand scratch
FLT_SA: equ 0x00CB              ; 1 byte : result sign
FLT_SB: equ 0x00CC              ; 1 byte : sign of B
FLT_ER: equ 0x00CD              ; 1 byte : result exponent (mul/div)
FLT_DE: equ 0x00CE              ; 1 byte : flt_print decimal exponent
FLT_SP: equ 0x00CF              ; 2 bytes: flt_parse string pointer save
FLT_DB: equ 0x00D1              ; 3 bytes: flt_div B-mantissa spill
IBUF:   equ 0x000C              ; 64 bytes: digit buffer

; =============================================================================
; FLT_ZERO  FLT_A = +0.0
; S2: two word stores; no CX/DI clobber.
; Inputs  : —
; Outputs : FLT_A = 0
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
        ; 1-bit left-shift AL:DL:DH:CH
        clc
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
        inc  dl
        jnz  np_pack
        inc  dh
        jnz  np_pack
        inc  ch
        jnz  np_pack
        ; Mantissa overflow on round: shift right 1, bump exponent
        stc
        rcr  ch, 1
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
; Sign / abs helpers
; Clobbers: nothing
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
; Copy helpers
; Clobbers: AX (word variants), or CX,SI,DI (movsb)
; =============================================================================
flt_b_to_a:
        cld
        mov  si, FLT_B
        mov  di, FLT_A
        mov  cx, 4
        rep  movsb
        ret

flt_a_to_b:
        cld
        mov  si, FLT_A
        mov  di, FLT_B
        mov  cx, 4
        rep  movsb
        ret

; =============================================================================
; FLT_FROM_INT  AX (signed int16) -> FLT_A
;
; S5: direct stores only; no push/call flt_zero/pop.
; Normalisation loop: left-shift AX until bit15 set, counting down from 0x90.
;
; Inputs  : AX = signed 16-bit integer
; Outputs : FLT_A
; Clobbers: AX, BX, CX
; =============================================================================
flt_from_int:
        or   ax, ax
        je   ffi_zero
        mov  bl, 0x00           ; sign = positive
        jns  ffi_pos
        mov  bl, 0x80           ; sign = negative
        neg  ax
ffi_pos:
        mov  cl, 0x90           ; biased exponent for 2^16
ffi_lp:
        test ax, 0x8000
        jnz  ffi_found
        shl  ax, 1
        dec  cl
        jmp  ffi_lp
ffi_found:
        ; AX: bit15=implied-1, AH bits[6:0]=mant[22:16], AL=mant[15:8]
        mov  [FLT_A+0], cl
        and  ah, 0x7F
        or   ah, bl
        mov  [FLT_A+1], ah
        mov  [FLT_A+2], al
        mov  byte [FLT_A+3], 0
        ret
ffi_zero:
        jmp  flt_zero

; =============================================================================
; FLT_FROM_INT_B  AX (signed int16) -> FLT_B  (preserves FLT_A and SI)
;
; S6: push/pop [FLT_A+n] word to save/restore; saves 6 bytes vs v0.6.
;
; Inputs  : AX = signed 16-bit integer
; Outputs : FLT_B
; Clobbers: AX, BX, CX, DI
; =============================================================================
flt_from_int_b:
        push si
        push word [FLT_A+2]     ; save FLT_A bytes 2:3
        push word [FLT_A+0]     ; save FLT_A bytes 0:1
        call flt_from_int       ; result -> FLT_A
        cld
        mov  si, FLT_A
        mov  di, FLT_B
        mov  cx, 4
        rep  movsb              ; FLT_B = new value
        pop  word [FLT_A+0]     ; restore FLT_A bytes 0:1
        pop  word [FLT_A+2]     ; restore FLT_A bytes 2:3
        pop  si
        ret

; =============================================================================
; FLT_TO_INT  FLT_A -> AX (signed int16, truncate toward zero)
;
; S8: sign saved to DL (free during shift of BX); FLT_TS slot eliminated.
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
        mov  dl, [FLT_A+1]     ; save sign to DL (free reg)
        and  dl, 0x80
        mov  bh, [FLT_A+1]
        and  bh, 0x7F
        or   bh, 0x80           ; restore implied-1
        mov  bl, [FLT_A+2]
        mov  cl, 16
        sub  cl, al             ; shift = 16 - true_exp
fti_shr:
        shr  bx, 1
        dec  cl
        jnz  fti_shr
        mov  ax, bx
        or   dl, dl
        jz   fti_r
        neg  ax
fti_r:  ret
fti_zero:
        xor  ax, ax
        ret
fti_sat:
        ; Saturate; exact -32768: exp=0x90, byte1=0x80, bytes2:3=0
        test byte [FLT_A+1], 0x80
        jz   fti_sat_pos
        cmp  byte [FLT_A+0], 0x90
        jne  fti_sat_neg
        cmp  byte [FLT_A+1], 0x80
        jne  fti_sat_neg
        cmp  word [FLT_A+2], 0
        jne  fti_sat_neg
        mov  ax, -32768
        ret
fti_sat_neg:
        mov  ax, -32767
        ret
fti_sat_pos:
        mov  ax, 32767
        ret

; =============================================================================
; FLT_CMP  compare FLT_A with FLT_B (signed)
; Inputs  : FLT_A, FLT_B
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
        jz   fcmp_r2            ; B>0: A(=0) < B
        mov  ax, 1              ; B<0: A(=0) > B
        ret
fcmp_anz:
        or   bl, bl
        jnz  fcmp_both
        test byte [FLT_A+1], 0x80
        mov  ax, 1
        jz   fcmp_r2            ; A>0: A > B(=0)
        mov  ax, -1
        ret
fcmp_r2:
        ret
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
        mov  al, [FLT_A+2]
        mov  bl, [FLT_B+2]
        cmp  al, bl
        jne  fcmp_ne
        mov  al, [FLT_A+3]
        mov  bl, [FLT_B+3]
        cmp  al, bl
        jne  fcmp_ne
        pop  cx
fcmp_eq:
        xor  ax, ax
        ret
fcmp_ne:
        pop  cx
        jb   fcmp_lt
fcmp_gt:
        test cl, 0x80
        mov  ax, 1
        jz   fcmp_r
        mov  ax, -1
        ret
fcmp_lt:
        test cl, 0x80
        mov  ax, -1
        jz   fcmp_r
        mov  ax, 1
fcmp_r:
        ret

; =============================================================================
; FLT_SUB  FLT_A = FLT_A - FLT_B
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A
; Clobbers: same as flt_add
; =============================================================================
flt_sub:
        call flt_negate_b
        call flt_add
        jmp  flt_negate_b       ; restore FLT_B (tail-call)

; =============================================================================
; FLT_ADD  FLT_A = FLT_A + FLT_B
;
; Working mantissa CH:DX (24-bit) for larger operand.
; Smaller operand in FLT_T[0..2] (3 bytes; [2] is guard, init 0).
; Shift count in CL; result exponent in BH.
;
; Inputs  : FLT_A, FLT_B
; Outputs : FLT_A = sum
; Clobbers: AX, BX, CX, DX, DI, FLT_SA, FLT_SB, FLT_T[0..2]
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
        mov  al, [FLT_A+1]
        and  al, 0x80
        mov  [FLT_SA], al
        mov  al, [FLT_B+1]
        and  al, 0x80
        mov  [FLT_SB], al

        ; Compare exponents; put larger in BH:CH:DX, smaller in FLT_T
        mov  al, [FLT_A+0]
        mov  ah, [FLT_B+0]
        cmp  al, ah
        jnb  fa_a_larger

        ; B has larger exponent
        mov  al, [FLT_SB]
        mov  [FLT_SA], al
        mov  bh, [FLT_B+0]
        mov  ch, [FLT_B+1]
        and  ch, 0x7F
        or   ch, 0x80
        mov  dh, [FLT_B+2]
        mov  dl, [FLT_B+3]
        mov  al, [FLT_A+1]
        and  al, 0x7F
        or   al, 0x80
        mov  [FLT_T+0], al
        mov  al, [FLT_A+2]
        mov  [FLT_T+1], al
        mov  byte [FLT_T+2], 0
        mov  cl, [FLT_B+0]
        sub  cl, [FLT_A+0]
        jmp  fa_align

fa_a_larger:
        ; A has larger (or equal) exponent
        mov  bh, [FLT_A+0]
        mov  ch, [FLT_A+1]
        and  ch, 0x7F
        or   ch, 0x80
        mov  dh, [FLT_A+2]
        mov  dl, [FLT_A+3]
        mov  al, [FLT_B+1]
        and  al, 0x7F
        or   al, 0x80
        mov  [FLT_T+0], al
        mov  al, [FLT_B+2]
        mov  [FLT_T+1], al
        mov  byte [FLT_T+2], 0
        mov  cl, [FLT_A+0]
        sub  cl, [FLT_B+0]

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
        mov  al, [FLT_T+1]
        mov  [FLT_T+2], al
        mov  al, [FLT_T+0]
        mov  [FLT_T+1], al
        mov  byte [FLT_T+0], 0
        or   cl, cl
        jnz  fa_byte_lp
        jmp  fa_addorsub

fa_bit_lp:
        clc
        shr  byte [FLT_T+0], 1
        rcr  byte [FLT_T+1], 1
        rcr  byte [FLT_T+2], 1
        dec  cl
        jnz  fa_bit_lp

fa_addorsub:
        mov  al, [FLT_A+1]
        and  al, 0x80
        cmp  al, [FLT_SB]
        je   fa_same_sign

        ; Different signs: subtract smaller from larger
        mov  al, dl
        sub  al, [FLT_T+2]
        mov  dl, al
        mov  al, dh
        sbb  al, [FLT_T+1]
        mov  dh, al
        sbb  ch, [FLT_T+0]
        jnc  fa_norm

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
        jmp  fa_norm

fa_same_sign:
        ; Add with carry chain (LSB first)
        mov  al, dl
        add  al, [FLT_T+2]
        mov  dl, al
        mov  al, dh
        adc  al, [FLT_T+1]
        mov  dh, al
        adc  ch, [FLT_T+0]
        jnc  fa_norm
        ; S15: carry out of CH; shift right 1, bump exponent (no guard byte)
        rcr  ch, 1
        rcr  dx, 1
        inc  bh
        jz   fa_zero

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

        ; Result sign = sign_A XOR sign_B
        mov  al, [FLT_A+1]
        xor  al, [FLT_B+1]
        and  al, 0x80
        mov  [FLT_SA], al

        ; Load B mantissa: SI=00:B_hi (byte in LOW position), DX=B_lo
        mov  bl, [FLT_B+1]
        and  bl, 0x7F
        or   bl, 0x80
        xor  bh, bh
        mov  si, bx             ; SI = 0x00:BL
        mov  dh, [FLT_B+2]
        mov  dl, [FLT_B+3]     ; DX = B_lo

        ; Load A mantissa: DI=00:A_hi (byte in LOW position), AX=A_lo
        mov  cl, [FLT_A+1]
        and  cl, 0x7F
        or   cl, 0x80
        xor  ch, ch
        mov  di, cx             ; DI = 0x00:CL
        mov  ah, [FLT_A+2]
        mov  al, [FLT_A+3]     ; AX = A_lo

        push di                 ; push 00:A_hi (for step 2 restore)
        push ax                 ; push A_lo    (for step 1)

        ; Step 1: A_lo * B_lo
        mul  dx                 ; DX:AX = A_lo * B_lo
        mov  cx, dx             ; CX = high word
        mov  al, ah             ; AL = sub-guard
        xor  ah, ah
        pop  ax                 ; restore A_lo

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
; 24-bit shift-and-subtract, 32 iterations.
; B mantissa (3 bytes) spilled to FLT_DB[0..2].
;
; BUG FIX (this version, two separate bugs found and fixed):
;
; (1) The remainder's high byte MUST live in a genuine 8-bit register (CH),
;     not as a "00:byte" value inside a 16-bit register (the earlier SI-based
;     scheme). A 16-bit rcl carries in at bit0 and out at bit15; if the
;     meaningful byte sits in the HIGH half of that 16-bit register, the
;     carry-in from the lower limb lands at bit0 (wrong end) instead of bit8
;     (the byte's true bit0). That silently corrupted every division whose
;     remainder triggered a carry out of the low word during the loop (e.g.
;     355/113), while simpler cases (e.g. 1/3, which never overflows the low
;     word until the very end) happened to look correct.
;
; (2) The quotient MUST accumulate in a genuine 32-bit pair (DX:AX), not
;     DX:AL (24 bits) -- a 32-iteration loop produces up to 32 significant
;     quotient bits before the leading 1 is known to have appeared;
;     truncating to 24 bits mid-loop silently drops the top bits whenever the
;     quotient's leading 1 doesn't appear in the first 8 iterations. This in
;     turn meant AX/DX (the live quotient) could not be used as scratch for
;     loading B's bytes during compare/subtract, as the original code did
;     (which clobbered the quotient itself, not just the loop variables) --
;     fixed by using byte-wise subtract with direct memory operands against
;     [FLT_DB+n], needing no scratch register at all.
;
; Loop registers:
;   DX:AX = 32-bit quotient accumulator (top 24 bits = mantissa, AL = guard)
;   BX    = remainder lo word (16-bit, live)
;   CH    = remainder hi byte (8-bit, live -- genuine register, correct
;           carry-in/out semantics)
;   CL    = iteration counter
;
; Speculative (non-restoring) subtract: each iteration unconditionally
; subtracts B from CH:BX byte-wise; if the final byte borrows, the remainder
; was < B, so B is added back (restored) and the quotient bit stays 0;
; otherwise the subtraction is kept and the quotient bit is set. A 25th-bit
; shift-out (jc fdiv_ov) means the remainder is unconditionally >= B, so the
; subtract is committed without the borrow check.
;
; S19: fdiv_restore / fdiv_restore2 merged to one label.
; S20: fdiv_ov falls into the same subtract code as the main path.
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

        ; Result sign = sign_A XOR sign_B
        mov  al, [FLT_A+1]
        xor  al, [FLT_B+1]
        and  al, 0x80
        mov  [FLT_SA], al

        ; Spill B mantissa (implied-1) to FLT_DB[0..2]
        mov  al, [FLT_B+1]
        and  al, 0x7F
        or   al, 0x80
        mov  [FLT_DB+0], al
        mov  al, [FLT_B+2]
        mov  [FLT_DB+1], al
        mov  al, [FLT_B+3]
        mov  [FLT_DB+2], al

        ; Load A mantissa: CH = A_hi (8-bit), BX = A_bytes2:3 (16-bit)
        mov  ch, [FLT_A+1]
        and  ch, 0x7F
        or   ch, 0x80
        mov  bh, [FLT_A+2]
        mov  bl, [FLT_A+3]

        ; Pre-scale: if A >= B shift right 1, inc exponent
        mov  al, [FLT_DB+0]
        cmp  ch, al
        jb   fdiv_prescaled
        ja   fdiv_prescale
        mov  ah, [FLT_DB+1]
        mov  al, [FLT_DB+2]
        cmp  bx, ax
        jb   fdiv_prescaled
fdiv_prescale:
        shr  ch, 1
        rcr  bx, 1
        inc  byte [FLT_ER]
fdiv_prescaled:

        ; Initialise 32-bit quotient accumulator DX:AX
        xor  dx, dx
        xor  ax, ax
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

        ; Speculative subtract: CH:BX -= B (byte-wise, direct memory operands,
        ; no scratch register needed -- AX/DX, the live 32-bit quotient, are
        ; never touched).  If the final byte borrows, the remainder was < B;
        ; undo by adding B back (restore).  Otherwise commit and set the bit.
        sub  bl, [FLT_DB+2]
        sbb  bh, [FLT_DB+1]
        sbb  ch, [FLT_DB+0]
        jc   fdiv_restore        ; borrowed -> remainder was < B -> undo below
        or   al, 1               ; no borrow -> commit, set quotient bit 0
        jmp  fdiv_next

fdiv_restore:
        ; Undo the speculative subtract (add B back)
        add  bl, [FLT_DB+2]
        adc  bh, [FLT_DB+1]
        adc  ch, [FLT_DB+0]
        jmp  fdiv_next

fdiv_ov:
        ; S20: 25th-bit overflow guarantees remainder >= B; commit unconditionally
        sub  bl, [FLT_DB+2]
        sbb  bh, [FLT_DB+1]
        sbb  ch, [FLT_DB+0]
        or   al, 1

fdiv_next:
        dec  cl
        jnz  fdiv_loop

        ; 32-bit quotient = DX:AX (DH:DL:AH:AL from MSB to LSB).
        ; Top 24 bits (mantissa) = DH:DL:AH: bottom 8 bits (AL) = guard.
        ; norm_pack wants: CH=mant[23:16], DH=mant[15:8], DL=mant[7:0], AL=guard
        mov  bl, ah             ; save old AH (mant[7:0]) - BX is free here
        mov  ch, dh             ; CH = old DH = mant[23:16]
        mov  dh, dl             ; DH = old DL = mant[15:8]
        mov  dl, bl             ; DL = old AH = mant[7:0]
        ; AL already holds the guard byte (old AL) - leave unchanged
        mov  bh, [FLT_ER]
        jmp  norm_pack          ; backward jump - safe

fdiv_by_zero:
        push si
        mov  si, s_div0
        call print_sz
        pop  si
        jmp  flt_zero           ; backward jump - safe

s_div0: db "DIV0!",0

; =============================================================================
; FLT_PRINT  print FLT_A as decimal to terminal
;
; S22: FLT_A restored via pop [FLT_A+n] at end.
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
        mov  cx, ax
        or   cx, cx
        jz   fp_scale_done
        jl   fp_scale_up
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
        cmp  ax, -1
        je   fp_chk_lo
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
        push ax                 ; save decimal exponent for print phase
        mov  di, IBUF
        mov  cx, 7
fp_dig_lp:
        push cx
        push di
        call flt_to_int
        push ax
        call flt_from_int_b
        call flt_sub
        test byte [FLT_A+1], 0x80
        jz   fp_no_clamp
        call flt_zero
fp_no_clamp:
        pop  ax
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
        ; Round: digit[6] >= '5' -> round up
        mov  al, [IBUF+6]
        cmp  al, '5'
        jb   fp_strip
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

fp_strip:
        dec  di
        cmp  byte [di], '0'
        jne  fp_strip_done
        cmp  di, IBUF
        jle  fp_strip_done
        jmp  fp_strip
fp_strip_done:
        inc  di

        ; Print with decimal point
        pop  ax
        inc  ax
        mov  bx, ax             ; BX = digits before decimal point
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
        cmp  si, di
        jnb  fp_print_done
        mov  al, [si]
        call output
        inc  si
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
; S26: tail-call flt_negate at fpar_sign2.
;
; Inputs  : SI -> null/CR-terminated decimal string
; Outputs : FLT_A = parsed value; SI advanced past last char consumed
; Clobbers: AX, BX, CX, DX, DI, FLT_SA
; =============================================================================
flt_parse:
        call flt_zero
        mov  byte [FLT_SA], 0

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
        xor  bl, bl             ; BL: 0=before decimal, 1=after
        xor  cl, cl             ; CL: decimal place count

fpar_lp:
        mov  al, [si]
        cmp  al, '.'
        jne  fpar_notdot
        or   bl, bl
        jnz  fpar_end
        mov  bl, 1
        inc  si
        jmp  fpar_lp

fpar_notdot:
        sub  al, '0'
        jb   fpar_end
        cmp  al, 9
        ja   fpar_end
        xor  ah, ah
        inc  si
        mov  [FLT_SP], si

        push ax                 ; save digit
        push bx
        push cx
        mov  ax, 10
        call flt_from_int_b
        call flt_mul
        pop  cx
        pop  bx

        pop  ax                 ; restore digit
        push bx
        push cx
        call flt_from_int_b
        call flt_add
        pop  cx
        pop  bx

        mov  si, [FLT_SP]

        or   bl, bl
        jz   fpar_lp
        inc  cl
        jmp  fpar_lp

fpar_end:
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
        ; S26: tail-call flt_negate (skipped if positive)
        cmp  byte [FLT_SA], 0
        je   fpar_done
        jmp  flt_negate         ; backward jump - safe
fpar_done:
        ret

; =============================================================================
; I/O ROUTINES
; =============================================================================

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
oi_pos:
        mov  cx, 10
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
psz_lp:
        lodsb
        or   al, al
        je   psz_r
        call output
        jmp  psz_lp
psz_r:
        pop  si
        ret

new_line:
        mov  al, 0x0D
        call output
        mov  al, 0x0A
        jmp  output

; output / putchar / getchar -- BIOS INT 10h teletype (intercepted by sim_rom)
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
; TEST HARNESS  (10 tests)
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
        mov  si, s_1005
        call flt_parse
        call flt_a_to_b
        mov  ax, 100
        call flt_from_int
        call flt_sub
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
; RESET VECTOR
; =============================================================================
        org  0xFFF0
reset:  db   0xEA
        dw   start
        dw   0xF000

        times (0xF000+4096)-$ db 0xFF
