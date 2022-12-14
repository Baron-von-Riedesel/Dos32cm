
;--- simple printf() 64-bit

;--- simple printf implementation for DOS 32-bit
;--- supports:
;--- %x  : dword
;--- %lx : qword 
;--- %u  : dword
;--- %lu : qword 
;--- %d  : sdword
;--- %ld : sqword 
;--- %s  : near32 string out
;--- %c  : character

;--- i64toa(long long n, char * s, int base);
;--- convert 64-bit long long to string

i64toa PROC stdcall uses rdi number:qword, outb:ptr, base:dword

	mov ch,0
	movsxd rdi, base
	mov rax, number
	cmp rdi,-10
	jne @F
	neg rdi
	neg rax
	mov ch,'-'
@@:
	mov rbx, outb
	add rbx,22
	mov byte ptr [rbx],0
@@nextdigit:
	dec rbx
	xor rdx, rdx
	div rdi
	add dl,'0'
	cmp dl,'9'
	jbe @F
	add dl,7+20h
@@:
	mov [rbx], dl
	and rax, rax
	jne @@nextdigit
	cmp ch,0
	je @F
	dec rbx
	mov [rbx],ch
@@:
	mov rax, rbx
	ret

i64toa ENDP

printf PROC c uses rbx rsi rdi fmt:ptr, args:VARARG

local flag:byte
local longarg:byte
local size_:dword
local fillchr:dword
local szTmp[24]:byte

	lea rdi,args
@@L335:
	mov rsi,fmt
nextchar:
	lodsb
	or al,al
	je done
	cmp al,'%'
	je formatitem
	call handle_char
	jmp nextchar
done:
	ret 

formatitem:
	lea rax, @@L335
	push rax
	xor edx,edx
	mov [longarg],dl
	mov bl,1
	mov cl,' '
	cmp BYTE PTR [rsi],'-'
	jne @F
	dec bl
	inc esi
@@:
	mov [flag],bl
	cmp BYTE PTR [rsi],'0'
	jne @F
	mov cl,'0'
	inc esi
@@:
	mov [fillchr],ecx
	mov ebx,edx

	.while ( byte ptr [rsi] >= '0' && byte ptr [rsi] <= '9' )
		lodsb
		sub al,'0'
		movzx eax,al
		imul ecx,ebx,10		;ecx = ebx * 10
		add eax,ecx
		mov ebx,eax
	.endw

	mov [size_],ebx
	cmp BYTE PTR [rsi],'l'
	jne @F
	mov [longarg],1
	inc rsi
@@:
	lodsb
	mov [fmt], rsi
	cmp al,'x'
	je handle_x
	cmp al,'X'
	je handle_x
	cmp al,'d'
	je handle_d
	cmp al,'u'
	je handle_u
	cmp al,'s'
	je handle_s
	cmp al,'c'
	je handle_c
	and al,al
	jnz @F
	pop rax
	jmp done
handle_c:
	mov eax,[rdi]
	add rdi, 8
@@:
	call handle_char
	retn

handle_s:
	mov rsi,[rdi]
	add rdi, 8
	jmp print_string
handle_d:
	mov ebx,-10
	jmp @F
handle_u:
	mov ebx, 10
	jmp @F
handle_x:
	mov ebx, 16
@@:
	mov rax,[rdi]
	add rdi, 8
	cmp longarg, 0
	jnz print_num
	movsxd rax, eax
	and ebx, ebx
	js print_num
	mov eax, eax	; clear bits 32-63
print_num:
	lea rsi, szTmp
	invoke i64toa, rax, rsi, ebx
	mov rsi, rax

print_string:		;print asciiz string RSI
	mov rax, rsi
	.while byte ptr [rsi]
		inc rsi
	.endw
	sub rsi, rax
	xchg rax, rsi
	mov ebx,size_
	sub ebx,eax
	.if flag == 1
		.while sdword ptr ebx > 0
			mov eax, [fillchr]
			call handle_char	;print leading filler chars
			dec ebx
		.endw
	.endif

	.while byte ptr [rsi]
		lodsb
		call handle_char	;print char of string
	.endw

	.while sdword ptr ebx > 0
		mov eax, [fillchr]
		call handle_char	;print trailing spaces
		dec ebx
	.endw
	retn

handle_char:
	cmp al,10
	jnz @F
	mov al,13
	call @F
	mov al,10
@@:
	mov dl, al
	mov ah, 2
	int 21h
	retn

	align 4

printf ENDP


