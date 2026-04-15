; ------------------------------------------------------------
; Minimal 8088 boot sector  v1.1
; Loads next 5 sectors (2560 bytes) to 0x0000:0x7E00 and executes.
; Assembled with tinyasm: tinyasm -f bin bootsect.asm -o boot.bin
; Also works with nasm:   nasm -f bin bootsect.asm -o boot.bin
;
; v1.1: Converted from NASM syntax to tinyasm syntax:
;       0xNN hex prefix (was NNh), dw 0,0 for dd (tinyasm has no dd),
;       removed 'bits 16' directive (tinyasm is always 16-bit).
; ------------------------------------------------------------

    org 0x7C00

; bits 16 not needed - tinyasm is always 16-bit

start:
    jmp short boot
    nop

; ------------------------------------------------------------
; BIOS Parameter Block (dummy - valid enough for BIOS to boot)
; ------------------------------------------------------------
oem_name:       db "MINIDOS "      ; 8 bytes

bytes_per_sec:  dw 512
secs_per_clus:  db 1
reserved_secs:  dw 1
num_fats:       db 2
root_entries:   dw 224
total_secs16:   dw 2880
media:          db 0xF0
secs_per_fat:   dw 9
secs_per_track: dw 18
num_heads:      dw 2
hidden_secs:    dw 0, 0            ; dd 0 (two words)
total_secs32:   dw 0, 0            ; dd 0 (two words)

; ------------------------------------------------------------
boot:
    cli
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, 0x7C00
    sti

    mov [boot_drive], dl        ; save BIOS boot drive

; ------------------------------------------------------------
; Load 5 sectors (2560 bytes) starting at sector 2 to 0x0000:0x7E00
; ------------------------------------------------------------
    xor ax, ax
    mov es, ax                  ; ES = 0x0000
    mov bx, 0x7E00              ; load address offset
    mov ah, 0x02                ; INT 13h: read sectors
    mov al, 5                   ; number of sectors (5 x 512 = 2560 bytes)
    mov ch, 0                   ; cylinder 0
    mov cl, 2                   ; start at sector 2
    mov dh, 0                   ; head 0
    mov dl, [boot_drive]

    int 0x13
    jc disk_error

; ------------------------------------------------------------
    jmp 0x0000:0x7E00           ; far jump to loaded code

; ------------------------------------------------------------
disk_error:
    mov si, err_msg
.print:
    lodsb
    or al, al
    jz .print                   ; spin on null (halt)
    mov ah, 0x0E
    mov bx, 0x0007
    int 0x10
    jmp .print

boot_drive: db 0
err_msg:    db "Disk error!", 0

; ------------------------------------------------------------
; Pad to 510 bytes then boot signature
; ------------------------------------------------------------
    times 510-($-$$) db 0
    dw 0xAA55
