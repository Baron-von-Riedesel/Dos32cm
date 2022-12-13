
;--- 64-bit code for direct video output

bs equ 8	; backspace

@getcursorpos macro
	movzx ebx, byte ptr flat:[462h];page
	mov bx, flat:[EBX*2+450h]
endm

@setcursorpos macro
	movzx ebx, byte ptr flat:[462h];page
	mov flat:[EBX*2+450h], ax
endm

;--- print a char
;--- no registers modified

VioPutChar proc

local	wCols:word
local	bChar:byte
local	bRows:byte

	push rax
	push rbx
	push rcx
	push rdx
	push rsi
	push rdi

	mov bChar, al
	mov ch, flat:[0484h]		; rows-1
	mov cl, flat:[044Ah]		; cols
	mov bRows, ch
	@getcursorpos				; bh=row, bl=col
	mov al, bh		;row pos
	mov ch, 0
	mov wCols, cx
	mul cl
	add ax, ax
	mov bh, 00  ; bx=col pos
	add bx, bx
	add bx, ax
	mov si, flat:[044Eh]		; page offset
	cmp word ptr flat:[0463H],3B4h
	jz @F
	add si, 8000h
@@:
	movzx esi, si
	add esi, 0B0000h

	mov al, bChar

	cmp al, 13
	jnz @F
	mov ax, bx
	shr ax, 1
	div cl
	mov al, ah
	xor ah, ah
	add ax, ax
	sub bx, ax
	jmp char_done
@@:
	cmp al, 10
	jnz @F
	add bx, cx
	add bx, cx
	jmp char_done
@@:
	cmp al, bs
	jnz @F
	sub bx, 2
	jmp char_done
@@:
	movzx ebx, bx
	mov flat:[ebx+esi], al
	inc bx
	inc bx
char_done:
	mov al, bRows
	inc al
	mul cl
	add ax, ax
	cmp bx, ax
	jc @F
	call scrollup
	mov bx, ax
@@:
	mov ax, bx
	push ax
	mov cx, wCols
	shr ax, 1
	div cl
	xchg al, ah
	@setcursorpos
	pop ax
	call cursorset
;	call slowdown
	pop rdi
	pop rsi
	pop rdx
	pop rcx
	pop rbx
	pop rax
	ret

if 0
slowdown:
	mov cx, 200
@@:
	in al, 61h
	and al, 10h
	cmp al, ah
	mov ah, al
	jz @B
	loop @B
	retn
endif

cursorset:
	add ax, flat:[044EH]	;offset page
	mov dx, flat:[0463H]
	shr ax, 1			;the CRT offset is one plane only, no attribute bytes 
	mov cl, al			;first high byte
	mov al, 0eh
	out dx, ax
	mov ah, cl			;then low byte
	mov al, 0fh
	out dx, ax
	retn

scrollup:			;scroll up one line
	mov edi, esi
	push rdi
	movzx esi, wCols
	lea esi, [esi*2+edi]
	mov cl, byte ptr wCols
	mov al, bRows
	mul cl
	movzx ecx, ax
	shr cx,1
	rep movsd
	push rdi
	mov cx, wCols
	mov eax,07200720h
	shr cx, 1
	rep stosd [edi]
	pop rax
	pop rdi
	sub rax, rdi
	retn

VioPutChar endp

