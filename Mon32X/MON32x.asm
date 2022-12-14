
;--- mixing 32- and 64-bit sections in ".model flat"
;--- this requires JWasm v2.17 and using option -pe.

    .386
    .model flat
    option casemap:none
    option proc:private

TCMD equ 0      ; 1=activate 't'

lf  equ 10 

    .data?

qwRsp   dq ?    ; saved stack pointer
address dq ?    ; current offset for d cmd

buffer db 20 dup (?); keyboard buffer
lbuffer equ $ - offset buffer

    .code

f64 label fword
    dd offset main64
    dw 8        ; selector 8 is 64-bit code segment ( Dos32cm specific )

main proc c

    jmp [f64]

main endp

;--- define a string
CStr macro text:vararg
local sym
    .const
sym db text,0
    .code _TEXT64
    exitm <addr sym>
endm

    .x64

_TEXT64 segment use64 'CODE'
_TEXT64 ends

    .code _TEXT64

    include printf.inc

main64 proc

    and sp, 0fff0h
    invoke printf, CStr("Mon32x loaded at %X, rsp=%lX",lf), rbx, rsp
    invoke printf, CStr("ax=%X bx=%X cx=%X dx=%X si=%X di=%X bp=%X",lf), rax, rbx, rcx, rdx, rsi, rdi, rbp
    call set_exception_handlers
nextcmd:
    invoke printf, CStr("(cmds: a,d,q): ")

    mov ah,1        ; read a key from keyboard with echo
    int 21h
    lea rcx, nextcmd
    push rcx
    mov [qwRsp], rsp
    push rax
    invoke printf, CStr(lf)
    pop rax
    cmp al,'a'
    jz a_cmd
    cmp al,'d'
    jz d_cmd
    cmp al,'q'
    jz q_cmd
if TCMD
    cmp al,'t'
    jz t_cmd
endif
    cmp al,0dh      ;ENTER?
    jz newline
    mov ecx, eax
    invoke printf, CStr("unknown cmd: %c",lf), rcx
newline:
    ret
main64 endp

;--- get a line of characters
;--- ebx->buffer
;--- esi=size of buffer

getline proc
    xor edi, edi
    dec esi
nextkey:
    mov ah,1
    int 21h
    cmp al,0dh
    jz enter_pressed
    cmp al,08h
    jz backspace_pressed
    mov [ebx+edi],al
    inc edi
    cmp edi,esi
    jnz nextkey
    mov dl,0dh
    mov ah,2
    int 21h
    jmp enter_pressed
backspace_pressed:
    mov dl,20h
    mov ah,2
    int 21h
    cmp edi,0
    jz nextkey
    dec edi
    mov dl,08h
    mov ah,2
    int 21h
    jmp nextkey
enter_pressed:    
    mov byte ptr [ebx+edi],0
    ret
getline endp

;--- enter address (for d cmd)

a_cmd proc
    invoke printf, CStr("enter start address for d cmd: ")
    mov esi, lbuffer
    lea rbx, buffer
    call getline
    and edi,edi        ;at least 1 digit entered?
    jz done
    xor edi,edi
    xor rsi,rsi
    .while byte ptr [rbx+rdi]
        mov al,byte ptr [rbx+rdi]
        sub al,'0'
        jc error
        cmp al,9
        jbe @F
        or al, 20h
        sub al, 27h
        cmp al, 0fh
        ja error
@@:
        movzx eax,al
        shl rsi,4
        add rsi,rax
        inc rdi
    .endw
    mov [address],rsi
done:
    invoke printf, CStr(lf)
    ret
error:
    invoke printf, CStr(lf,"%s?",lf), addr buffer
    ret

a_cmd endp

;--- display memory dump

d_cmd proc
    mov rsi, [address]
    mov ecx, 8
nextline:
    push rcx
    invoke printf, CStr("%08lX: "), rsi
    mov ecx, 16
@@:
    push rcx
    lodsb
    movzx eax, al
    invoke printf, CStr("%02X "), rax
    pop rcx
    loop @B
    invoke printf, CStr(" ")
    sub rsi, 16
    mov ecx, 16
nextc:
    lodsb
    cmp al, 20h
    jnc @F
    mov al, '.'
@@:
    push rcx
    invoke printf, CStr("%c"), rax
    pop rcx
    loop nextc
    invoke printf, CStr(lf)
    pop rcx
    dec ecx
    jnz nextline
    mov [address], rsi
    ret
d_cmd endp

;--- 'q': back to real-mode

q_cmd proc
    mov ax,4c00h
    int 21h
q_cmd endp

if TCMD

;--- 't': some printf tests

t_cmd proc
    or rax, -1
    invoke printf, CStr("%u %d %x",10), rax, rax, rax
    or rax, -1
    invoke printf, CStr("%lu %ld %lx",10), rax, rax, rax
    ret
t_cmd endp

endif

;--- handle exception 0E so we won't terminate
;--- if an invalid address has been entered.

set_exception_handlers proc

    mov ecx, cs
    mov edx, offset exception0E
    mov bl, 0Eh
    mov ax, 203h
    int 31h
    ret

set_exception_handlers endp

;--- Dos32cm provides a 64-bit exception frame!

exception0E:
    sti
    mov rdx, [rsp+0*8]
    mov rcx, [rsp+1*8]
    mov rax, [rsp+2*8]
    mov rbx, cr2
    invoke printf, CStr(lf,"page fault, errcode=%X cs:eip=%X:%X cr2=%lX",lf), rdx, rax, rcx, rbx
    mov rsp, [qwRsp]
    ret

    end main
