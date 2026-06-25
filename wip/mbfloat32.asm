; =============================================================================
; MiniBASIC8088  MBF4 Float Library
; Copyright (c) 2026 Vincent Crabtree, MIT License
;
; Usage: tinyasm -f bin mbfloat_v14.asm -o mbfloat_v14.bin
;        sim_rom mbfloat_v14.asm
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
;        Later in the v0.7 line (sharing pass): sign_xor helper shared by
;        flt_mul/flt_div; flt_ten_b/mul_by_ten/div_by_ten helpers collapse 8
;        call sites that each loaded FLT_B=10 then multiplied/divided;
;        flt_add's larger/smaller-operand load paths unified via an in-place
;        FLT_A<->FLT_B swap instead of two near-mirror 19-instruction blocks.
; v0.8 : Two bug fixes + a golf-inspired flt_cmp rewrite.
;        BUGFIX (SI clobber): flt_a_to_b/flt_b_to_a used SI as the movsb
;          source pointer and never restored it, silently breaking any
;          caller that relied on flt_add's documented "preserves SI" when
;          flt_add's FLT_A==0 fast path tail-jumped into flt_b_to_a. Fixed
;          by push si / ... / pop si in both copy helpers.
;        BUGFIX (sign lost in flt_parse): the input's +/- sign was stashed
;          in FLT_SA, but FLT_SA is flt_add's own "sign of larger operand"
;          scratch byte and gets overwritten by every flt_add call made
;          while accumulating digits -- so "-42" silently parsed as +42.
;          This was present in both the original iterative flt_parse and an
;          early draft of the recursive rewrite below; fixed by stashing the
;          sign in FLT_DE instead (flt_print's decimal-exponent scratch,
;          never touched by flt_add/flt_mul/flt_div, and never live at the
;          same time as a parse since flt_parse never calls flt_print).
;        flt_cmp: replaced the original tuple-by-tuple magnitude comparison
;          (159 bytes) with a rewrite built on the existing flt_sub plus a
;          branchless sign-to-(-1/+1) "SBB ternary trick" (42 bytes, -117
;          bytes). FLT_A is saved/restored around the subtract; FLT_B is
;          unchanged because flt_sub already restores it. The zero-flag
;          from testing the difference survives the restore because mov/pop
;          do not touch flags on 8086.
;        flt_parse: rewritten so the fractional digits are evaluated
;          recursively (recurse to the last typed digit, then unwind
;          building result=(digit+rest)/10) instead of iteratively
;          accumulating digits left-to-right and rescaling by 10^n at the
;          end. Same byte count as the iterative version (123 vs 124 bytes)
;          but removes the FLT_SP string-pointer round-trip and the
;          separate decimal-place counter. Recursion depth is bounded by
;          the number of *typed* fractional-digit characters (source text
;          length), not by the parsed value's magnitude, so it carries much
;          less stack-depth risk than a similar recursive trick applied to
;          the integer part would (an integer part can have up to ~38
;          significant decimal digits for MBF4's exponent range, all from a
;          single short literal like "1E38", whereas nobody hand-types 38
;          fractional digits). FLT_SP is consequently unused; left reserved
;          in the RAM map rather than renumbered.
;
; v0.9 : BUGFIX (flt_add, dropped mantissa byte): fa_signs zeroed FLT_T+2
;          instead of loading FLT_B+3, silently discarding the smaller
;          operand's least-significant mantissa byte before alignment --
;          every add/sub lost ~8 bits of B's precision. Fixed by loading
;          FLT_B+3 (mov-immediate-0 -> mov-from-memory; zero byte cost).
;          flt_mul/flt_div unaffected (already loaded all 3 of B's
;          mantissa bytes correctly).
;
; v0.10: Pure size-golf pass, no behaviour change (T1-T13 byte-for-byte
;        identical output before/after). Library code (org to 'start:')
;        shrank from 0x602 to 0x5DF bytes (1538 -> 1503, -35 bytes).
;          - Found a systematic dead-code pattern: 'and reg,0x7F' used to
;            clear the implied-1 bit immediately before 'or reg,0x80'
;            forces it back on unconditionally -- the AND never has any
;            effect, since OR with 0x80 sets bit7=1 regardless of the
;            AND's outcome. Removed 6 instances across flt_add (x2),
;            flt_mul (x2), flt_div (x2). The original 8088-math draft had
;            already spotted this once (flt_mul's B_hi load, commented
;            "Drop redundant 'and bl, 0x7F'") but it was never carried to
;            the other 6 sites when the routines were later rewritten.
;          - flt_to_int: same dead-AND pattern (1 more instance) plus two
;            separate reads of [FLT_A+1] merged into one (register copy
;            instead of a second memory fetch), plus the 'and dl,0x80 /
;            or dl,dl' sign test replaced with a deferred 'shl dl,1' that
;            extracts bit7 straight into CF (no masking needed since only
;            the carry-out is used after the shift loop).
;          - flt_add fa_addorsub: was re-deriving sign-of-larger via
;            'mov al,[FLT_A+1] / and al,0x80', duplicating work already
;            done in fa_signs and stashed in FLT_SA (nothing between the
;            two touches FLT_A+1) -- now just loads FLT_SA directly.
;          - flt_div: the restored B_hi byte was stored to FLT_DB+0 and
;            then immediately reloaded from memory for the pre-scale
;            compare. Kept in DL instead (dead by the time DX is zeroed
;            for the quotient accumulator), so the compare is reg-reg.
;          - flt_from_int: 'test ax,0x8000' (3-byte immediate test) in
;            the normalisation loop replaced with 'or ax,ax' (2 bytes);
;            both just need bit15 reflected into a flag (SF here).
;          - norm_pack: dropped a redundant 'clc' before the np_bit
;            left-shift chain -- CF is already 0 there, set by the
;            'or ch,ch' that is the only way to reach np_bit (OR always
;            clears CF on 8086, and jumps don't touch flags). Also: the
;            round-overflow path used 'stc / rcr ch,1' to produce 0x80
;            in ch, but ch is provably 0x00 at that point (the preceding
;            three inc/jnz all fell through) -- 'mov ch,0x80' does the
;            same job in 2 bytes instead of 3.
;          - flt_b_to_a / flt_a_to_b: merged into one shared copy tail
;            (cld/mov cx,4/rep movsb/pop si/ret); each entry point now
;            only sets up its own SI/DI before falling/jumping into it.
;        None of the above are speed optimisations -- several (e.g. the
;        flt_to_int and flt_div changes) trade a register-register op for
;        what was a register-immediate or memory op, which is sometimes
;        marginally slower on real 8088 hardware. Size was the only goal.
;
; v0.11: Second size-golf pass, no behaviour change (T1-T13 still byte-
;        for-byte identical). Library code 1503 -> 1481 bytes (-22).
;          - flt_add: the subtract-smaller-from-larger and add-with-carry
;            paths routed every byte through AL ('mov al,dl / sub al,mem /
;            mov dl,al', etc.) even though SUB/SBB/ADD/ADC can target
;            DL/DH/CH directly as the destination register -- 8086 has no
;            restriction requiring AL here, and the very next block (the
;            two's-complement negate on borrow) already operates on
;            dl/dh/ch directly, confirming this was simply unexploited.
;            -8 bytes each path, -16 total.
;          - flt_mul step 1: 'mov al,ah / xor ah,ah' (intended to set up
;            a rounding guard byte) is immediately and unconditionally
;            overwritten by the very next instruction ('pop ax') -- AX's
;            pre-pop value is never observed, so this was dead code.
;            -4 bytes. (Separately: tracing where AL's value actually
;            comes from by the time norm_pack is reached shows it ends up
;            holding the low byte of the A_hi*B_hi product, not a
;            meaningful guard bit for the overall product's rounding --
;            this looks like a pre-existing rounding-precision gap, not
;            something this golf pass touches; flagging per the
;            don't-mix-correctness-with-size-passes rule.)
;          - flt_div final shuffle: the 3-value rotation (old DH->CH, old
;            DL->DH, old AH->DL) used 4 movs through a BL scratch. An
;            xchg-based rotation ('xchg dl,ah' stashes old DL in AH, then
;            'mov dh,ah' retrieves it) does the same job in 3
;            instructions. -2 bytes.
;        Checked but NOT changed: several 'jcc near_label / jmp far_label
;        / near_label:' trampolines (flt_add/flt_mul/flt_div entry zero-
;        checks, flt_add's align-overflow check) look at first glance
;        like they could collapse to a single inverted jcc, but Jcc on
;        8086 is short-jump-only (rel8, +/-127 bytes) and every target
;        checked is 142-673 bytes away -- the trampoline is load-bearing,
;        not redundant.
;        Also checked but NOT changed (correctness, not size, so out of
;        scope here): flt_to_int's fti_sat block guards on true_exp > 16,
;        but the documented "exact -32768" case has true_exp == 16
;        exactly, which never reaches fti_sat at all -- the -32768
;        special-case comparisons inside fti_sat (exp==0x90 etc.) look
;        unreachable as currently gated. The normal-path shift loop
;        (fti_shr) also has a latent do-while-with-cl=0 issue for that
;        same true_exp==16 case (dec wraps 0->0xFF before the jnz check,
;        running 255 extra iterations instead of zero). Both were flagged
;        previously; still unfixed, still believed real, still untouched.
;
; v0.12: Third size-golf pass, based on a third-party review of v0.11
;        that was independently verified (hand-traced bit/byte semantics,
;        then confirmed empirically via sim_rom) before any of it was
;        applied -- one reviewed suggestion (a "BP remainder engine" for
;        flt_div) was rejected as incomplete/risky and is NOT included;
;        see the rejected-suggestions note below. Library code 1481 ->
;        1363 bytes (-118). T1-T13 unchanged; new targeted tests (exact
;        -32768 round-trip, saturation boundaries, byte-shift and
;        smaller-vanishes alignment paths, same-sign mantissa overflow,
;        division pre-scale path, FLT_A-preservation contract) added
;        during verification and all pass.
;          - flt_to_int: replaced the do-while shift loop with native
;            'shr bx,cl'. 8086 defines CL=0 as a true no-op (register and
;            flags both untouched), which ALSO fixes the true_exp==16
;            (exact -32768) edge case for free -- the old loop's 'dec cl'
;            wrapped 0->0xFF before the jnz check, running 255 spurious
;            iterations instead of zero.
;          - flt_to_int: fti_sat's -32768-specific triple-check (exp==
;            0x90, byte1==0x80, bytes2:3==0) is deleted -- it was always
;            unreachable (fti_sat is only entered when true_exp>16, but
;            exact -32768 has true_exp==16 exactly) and is now provably
;            unnecessary too, since the shr-by-cl fix above handles that
;            value correctly with no special-casing. The general
;            saturate-to-+-32767-by-sign logic is kept, for genuine
;            overflow (true_exp>16, which really can't fit in int16).
;          - flt_add: FLT_T eliminated entirely. The smaller operand's
;            mantissa now lives in AL(hi):BX(mid:lo) instead of a 3-byte
;            memory scratch -- 'shr al,1 / rcr bx,1' shifts it as one
;            native 16-bit rotate plus one byte shift (the memory version
;            needed two separate byte-level rcrs), and the final add/sub
;            against CH:DX becomes register-register instead of
;            register-memory. This displaced the larger operand's
;            exponent (formerly cached in BH for the routine's whole
;            duration) out of registers -- it's now reloaded from
;            [FLT_A+0] (unchanged in memory throughout) at the 3 points
;            actually needed. The sign-equality check at fa_addorsub
;            moved from AL (now busy holding the smaller mantissa's hi
;            byte) to CL (free and guaranteed 0 right after the align
;            loop). Also dropped a second redundant clc, at fa_bit_lp's
;            top: SHR never reads CF as input, only produces it, so
;            clearing CF before a chain that *starts* with SHR was always
;            dead code -- same class of bug as the norm_pack clc caught
;            in v0.10, just missed there. FLT_T removed from the RAM map.
;          - flt_div: FLT_DB reordered to little-endian (DB+0=B_lo,
;            DB+1=B_mid, DB+2=B_hi) so that DB+0, read as a 16-bit word,
;            equals B_mid*256+B_lo -- exactly BX's existing mid*256+lo
;            packing for A. Collapses the pre-scale check's separate
;            AH/AL byte loads into one word compare, and all three
;            byte-wise sub/sbb/add/adc triples (main loop, restore-on-
;            borrow, 25th-bit-overflow commit) into word+byte pairs.
;          - flt_from_int / flt_from_int_b: merged into one body via a DI
;            destination parameter. The old flt_from_int_b wrote into
;            FLT_A (via flt_from_int), saved/restored FLT_A's original
;            bytes around that call, and copied the result into FLT_B via
;            movsb (which needed SI, hence also saving/restoring SI).
;            Writing directly to [di+n] means flt_from_int_b never
;            touches FLT_A or SI at all -- both preserved trivially, no
;            save/restore needed for either. Register-indirect [di+n] is
;            also individually cheaper than the old direct-address
;            [FLT_A+n] stores (no disp16 for [di+0], 1-byte disp8 for the
;            rest, vs always a 2-byte disp16).
;          - cp4 (shared copy helper): two movsw instead of rep movsb
;            with cx=4 -- smaller for a known fixed 4-byte copy.
;          - parse_frac: its final step duplicated div_by_ten's body
;            inline ('call flt_ten_b / jmp flt_div') instead of just
;            calling div_by_ten, which does exactly that pair.
;          - norm_pack: 'inc dl/jnz/inc dh/jnz' replaced with a single
;            'inc dx/jnz' -- INC r16 is a 1-byte opcode on 8086, and a
;            16-bit increment correctly ripples the DL->DH carry as one
;            hardware op; ZF after it reflects whether the full 16-bit
;            value wrapped to exactly 0 (both bytes maxed), the same
;            condition the old two-step version was checking for.
;          - Micro-golf: 'mov ax,bx'->'xchg ax,bx' in flt_to_int and
;            'mov cx,ax'->'xchg cx,ax' in flt_print (XCHG-with-AX is a
;            special 1-byte 8086 opcode; both swapped-out registers are
;            dead at their respective points, confirmed by reading
;            forward to their next use). 'xor dx,dx/xor ax,ax'->
;            'xor ax,ax/cwd' for flt_div's quotient-accumulator init
;            (CWD sign-extends a known-zero AX into a guaranteed-zero DX).
;        REJECTED: a reviewed "BP remainder engine" for flt_div (move the
;        remainder's hi byte from CH into BL, freeing CX for the native
;        LOOP instruction). The bit-shift restructuring was internally
;        consistent, but the suggestion's setup snippet only showed the
;        B-side byte-swap for the new FLT_DB layout, not the equivalent
;        A-side swap needed to load BP correctly -- a naive direct word
;        load would reintroduce the exact byte-order bug this file's
;        very first review (pre-v0.6) found in the original flt_div. It
;        also assumes a different FLT_DB byte order than the reorder
;        actually adopted above, so the two are not both applicable as
;        written. Not implemented; flagging in case it's worth a fully
;        written-out, independently-tested attempt later.
;
; v0.13: flt_print correctness pass (not a size pass -- net +10 bytes,
;        1363 -> 1373, in exchange for fixing three real, confirmed
;        bugs). Test harness extended through T25; T1-T21 unchanged
;        except T2 and T4, whose displayed precision dropped by one
;        digit as a direct, deliberate consequence of the digit-7 fix
;        below (3.141592->3.14159, 0.3333333->0.333333 -- both still
;        correct to 6 significant figures).
;          - Trailing-zero over-stripping (e.g. 1000000 printed as "1"):
;            fp_strip walked back from IBUF+7 removing every trailing
;            '0' down to IBUF itself, with no concept of where the
;            integer/fraction boundary was -- it couldn't tell a
;            genuinely-fractional trailing zero (2.500000 -> "2.5",
;            correct to strip) from a zero that's part of the integer's
;            magnitude (1000000's six trailing zeros, NOT safe to strip).
;            Fixed (independently re-derived; matches a third-party
;            review of v0.11's sibling print-focused branch) by
;            computing a strip floor = IBUF + digits_before_point - 1
;            and refusing to strip at or below it.
;          - The floor fix alone doesn't cover values needing MORE
;            integer digits than the 7-digit extraction buffer holds at
;            all (e.g. 900000000, 9 digits -- was printing "9", improved
;            to "9000000" by the floor fix alone, still short 2 digits).
;            fp_print_lp now detects exactly this case (digit buffer
;            exhausted but the digits-before-point counter isn't yet 0)
;            and pads with literal '0' characters instead of exiting.
;          - Digit 7 (the last of the 7 extracted) was only blanked to
;            '0' on the round-up path (digit[6]>='5'); the no-round-up
;            path printed it as a real digit. Traced the actual decimal
;            drift through a 95.5 test case by hand-decoding every
;            FLT_A snapshot in the extraction loop: a value's inherent
;            sub-ULP binary-representation error compounds roughly 10x
;            per mul_by_ten call (confirmed: diffs of 9.3e-10, 1.5e-8,
;            1.2e-7 across 3 successive multiplies, each ~10x the last),
;            and since extracting 7 digits requires 6 such multiplies,
;            even a ~1e-6 starting error is large enough to flip the
;            7th digit by the time it's reached (this is exactly what
;            produced "95.50001" for the exact value 95.5). Digit 7 is
;            therefore never reliable enough to display -- fixed by
;            unconditionally blanking it after the round-up decision,
;            on both paths, rather than only the round-up one. This
;            does cost a digit of precision on values where digit 7
;            happened not to be corrupted (T2, T4) -- the alternative
;            (displaying it) means it's sometimes silently wrong with no
;            way to tell which case you're in, which is worse.
;          - Also adopted (independently re-derived; matches the same
;            sibling branch): fp_extract's loop counter folded into DI's
;            own position (cmp di,IBUF+7) instead of a separate CX
;            decrement -- only DI needs protecting across the calls that
;            clobber it now, and flt_to_int itself never touches DI, so
;            the first call of each iteration needs no save/restore.
;        Known still-open: none identified. The fti_sat unreachable-code
;        question from v0.11/v0.12 was resolved in v0.12 (shr bx,cl).
;
; v0.14: Small size-golf follow-on after v0.13's correctness pass.
;        Library 1373 -> 1364 bytes (-9). T1-T25 unchanged.
;          - fp_strip_setup: dx's target value (IBUF+digits_before_point-1,
;            or plain IBUF when digits_before_point<=0) was computed via
;            'inc bx / ... / add dx,bx / dec dx'. Since digits_before_point
;            -1 is just the raw decimal exponent (de) before its own +1
;            adjustment, computing dx as IBUF+de directly (moving the
;            'inc bx' to after the branch, where it's needed for the
;            later print-phase bx but not for dx) drops the separate dec
;            entirely. -1 byte.
;          - flt_negate / flt_negate_b merged via a DI destination
;            parameter, same pattern as v0.12's flt_from_int_b. This adds
;            DI to flt_negate_b's clobber set, and transitively
;            flt_sub's/flt_cmp's -- a real (if narrow) contract change,
;            flagged rather than silently absorbed. DI is already
;            fair-game scratch throughout this codebase, though: it was
;            already a documented clobber of flt_parse, flt_mul,
;            flt_div, and flt_print, and (per a closer look while making
;            this change) flt_cmp's clobber line had already listed DI
;            anyway, apparently defensively/inaccurately, since neither
;            flt_add nor norm_pack actually touch it -- it's accurate
;            now for a real reason instead. Register-indirect [di] /
;            [di+1] addressing is also individually cheaper than the old
;            direct-address [FLT_A] / [FLT_A+1] forms. -8 bytes.
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
; FLT_SA          = result sign (0x00 or 0x80)
; FLT_SB          = sign of B (flt_add)
; FLT_ER          = result exponent (flt_mul / flt_div)
; FLT_DE          = flt_print decimal exponent
; FLT_SP (2 bytes)= UNUSED (reserved; see RAM LAYOUT comment below)
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
;   tinyasm -f bin mbfloat_v14.asm -o mbfloat_v14.bin
;   sim_rom mbfloat_v14.asm
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
                                 ; 0x00C8-0xCA: unused (was FLT_T pre-v0.12)
FLT_SA: equ 0x00CB              ; 1 byte : result sign
FLT_SB: equ 0x00CC              ; 1 byte : sign of B
FLT_ER: equ 0x00CD              ; 1 byte : result exponent (mul/div)
FLT_DE: equ 0x00CE              ; 1 byte : flt_print decimal exponent
FLT_SP: equ 0x00CF              ; 2 bytes: UNUSED (was flt_parse string-pointer
                                 ; save in the iterative version; the recursive
                                 ; flt_parse below has no need for it). Left
                                 ; reserved rather than renumbered, since RAM
                                 ; layout isn't the size budget and renumbering
                                 ; risks an easy-to-miss stale reference.
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
; Sign / abs helpers
; v0.13: flt_negate/flt_negate_b merged via a DI destination parameter
; (same pattern as v0.12's flt_from_int_b). This adds DI to
; flt_negate_b's clobber set, and transitively flt_sub/flt_cmp's -- but
; DI is already fair-game scratch throughout this codebase (a documented
; clobber of flt_parse, flt_mul, flt_div, flt_print already), so this is
; consistent with the existing convention, not a new constraint.
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
flt_neg_r: ret

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
; Clobbers: same as flt_ten_b + flt_mul
; =============================================================================
mul_by_ten:
        call flt_ten_b
        jmp  flt_mul            ; tail-call (forward - safe, within range)

; =============================================================================
; DIV_BY_TEN  FLT_A = FLT_A / 10
; Inputs  : FLT_A
; Outputs : FLT_A
; Clobbers: same as flt_ten_b + flt_div
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
; Clobbers: same as flt_add, plus DI (via flt_negate_b)
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
; v0.12: FLT_DB reordered to little-endian (DB+0=B_lo, DB+1=B_mid,
;   DB+2=B_hi) -- as a 16-bit word, DB+0 now equals B_mid*256+B_lo,
;   matching BX's existing mid*256+lo packing for A. Collapses the
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
; BUG FIX (carried over from the iterative version): the input sign is
; stashed in FLT_DE, not FLT_SA. FLT_SA is flt_add's internal "sign of
; larger operand" scratch and gets clobbered by every flt_add call made
; while accumulating digits or combining the integer and fractional parts
; -- stashing the sign there meant it was long gone by the time it was
; applied (e.g. "-42" silently parsed as +42). FLT_DE is flt_print's
; decimal-exponent scratch, never touched by flt_add/flt_mul/flt_div, and
; flt_parse never calls flt_print, so the slot is safe for the whole parse.
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
; Outputs : FLT_A = value of the fractional digits read (in [0,1)); SI
;           advanced one past the last digit consumed (mirrors flt_parse's
;           own SI convention so the caller's later 'dec si' lines up)
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

        mov  si, s_t11          ; T11: parse("-42") = -42 
        call print_sz
        mov  si, s_n42
        call flt_parse
        call flt_print
        call new_line

        mov  si, s_t12          ; T12: parse("-100.5") = -100.5
        call print_sz
        mov  si, s_n1005
        call flt_parse
        call flt_print
        call new_line

        mov  si, s_t13          ; T13: -100.5 + 5 = -95.5 (flt_add swap path, mixed signs)
        call print_sz
        mov  si, s_n1005
        call flt_parse
        mov  ax, 5
        call flt_from_int_b
        call flt_add
        call flt_print
        call new_line

        mov  si, s_t14          ; T14: -32768 round-trip (exact edge case)
        call print_sz
        mov  ax, -32768
        call flt_from_int
        call flt_to_int
        call output_int
        call new_line

        mov  si, s_t15          ; T15: parse(1000000) trunc'd -> saturate to 32767
        call print_sz
        mov  si, s_1e6
        call flt_parse
        call flt_to_int
        call output_int
        call new_line

        mov  si, s_t16          ; T16: parse(-1000000) trunc'd -> saturate to -32767
        call print_sz
        mov  si, s_n1e6
        call flt_parse
        call flt_to_int
        call output_int
        call new_line

        mov  si, s_t17          ; T17: 16384 + 1 = 16385 (flt_add byte-shift align path)
        call print_sz
        mov  ax, 16384
        call flt_from_int
        mov  ax, 1
        call flt_from_int_b
        call flt_add
        call flt_to_int
        call output_int
        call new_line

        mov  si, s_t18          ; T18: 8 + 8 = 16 (flt_add same-sign mantissa overflow)
        call print_sz
        mov  ax, 8
        call flt_from_int
        call flt_a_to_b
        mov  ax, 8
        call flt_from_int
        call flt_add
        call flt_to_int
        call output_int
        call new_line

        mov  si, s_t19          ; T19: 3 - 5 = -2 (flt_add/sub borrow-out negate path)
        call print_sz
        mov  ax, 5
        call flt_from_int
        call flt_a_to_b
        mov  ax, 3
        call flt_from_int
        call flt_sub
        call flt_to_int
        call output_int
        call new_line

        mov  si, s_t20          ; T20: 9 / 3 = 3 (flt_div pre-scale path, equal exponents)
        call print_sz
        mov  ax, 9
        call flt_from_int
        mov  ax, 3
        call flt_from_int_b
        call flt_div
        call flt_to_int
        call output_int
        call new_line

        mov  si, s_t21          ; T21: flt_from_int_b preserves FLT_A: from_int(-100),
        call print_sz           ;   from_int_b(7) -> FLT_A still -100, FLT_B = 7
        mov  ax, -100
        call flt_from_int
        mov  ax, 7
        call flt_from_int_b
        call flt_to_int
        call output_int
        mov  si, s_comma
        call print_sz
        call flt_b_to_a
        call flt_to_int
        call output_int
        call new_line

        mov  si, s_t22          ; T22: 1000*1000 = 1000000 (flt_print trailing-zero
        call print_sz           ;   strip must not eat zeros that are part of the
        mov  ax, 1000           ;   integer magnitude -- was printing "1")
        call flt_from_int
        call flt_a_to_b
        call flt_mul
        call flt_print
        call new_line

        mov  si, s_t23          ; T23: 30000*30000 = 900000000 (needs MORE integer
        call print_sz           ;   digits than the 7-digit extraction buffer holds
        mov  ax, 30000          ;   at all -- was printing "9")
        call flt_from_int
        call flt_a_to_b
        call flt_mul
        call flt_print
        call new_line

        mov  si, s_t24          ; T24: 191/2 = 95.5 exactly (was printing "95.50001":
        call print_sz           ;   digit 7 is the least reliable of the 7 extracted
        mov  ax, 191            ;   digits -- most compounded mul_by_ten rounding
        call flt_from_int       ;   error -- and is now always discarded rather than
        mov  ax, 2              ;   displayed, used only for the round-up decision)
        call flt_from_int_b
        call flt_div
        call flt_print
        call new_line

        mov  si, s_t25          ; T25: 1000000 + 3, displayed at the library's honest
        call print_sz           ;   6-reliable-digit precision (was confidently wrong
        mov  ax, 1000           ;   as "1000002"; now correctly shows the best 6-sig-
        call flt_from_int       ;   fig approximation, 1000000, rather than a 7th
        call flt_a_to_b         ;   digit that can't be trusted)
        call flt_mul
        mov  ax, 3
        call flt_from_int_b
        call flt_add
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
s_t11:  db "T11 parse(-42)=",0
s_t12:  db "T12 parse(-100.5)=",0
s_t13:  db "T13 -100.5+5=",0
s_t14:  db "T14 -32768 roundtrip=",0
s_t15:  db "T15 trunc(1000000)=",0
s_t16:  db "T16 trunc(-1000000)=",0
s_t17:  db "T17 16384+1=",0
s_t18:  db "T18 8+8=",0
s_t19:  db "T19 3-5=",0
s_t20:  db "T20 9/3=",0
s_t21:  db "T21 from_int_b preserve(A=-100,B=7)=",0
s_t22:  db "T22 1000*1000=",0
s_t23:  db "T23 30000*30000=",0
s_t24:  db "T24 191/2=",0
s_t25:  db "T25 1000000+3 (6-digit precision)=",0
s_done: db "DONE",0
s_1005: db "100.5",0x0D
s_25:   db "2.5",0x0D
s_39:   db "3.9",0x0D
s_n42:  db "-42",0x0D
s_n1005: db "-100.5",0x0D
s_1e6:  db "1000000",0x0D
s_n1e6: db "-1000000",0x0D
s_comma: db ",",0

; =============================================================================
; RESET VECTOR
; =============================================================================
        org  0xFFF0
reset:  db   0xEA
        dw   start
        dw   0xF000

        times (0xF000+4096)-$ db 0xFF
