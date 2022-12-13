
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

di64toa PROC stdcall uses rdi number:qword, outb:dword, base:dword

	mov ch,0
	mov edi, base
	mov eax, dword ptr number+0
	mov esi, dword ptr number+4
	cmp edi,-10
	jne @F
	neg edi
	and esi,esi
	jns @F
	neg esi
	neg eax
	sbb esi,0
	mov ch,'-'
@@:
	mov ebx,outb
	add ebx,22
	mov byte ptr [ebx],0
@@nextdigit:
	dec ebx
	xor edx,edx
	xchg eax,esi
	div edi
	xchg eax,esi
	div edi
	add dl,'0'
	cmp dl,'9'
	jbe @F
	add dl,7+20h
@@:
	mov [ebx],dl
	mov edx, eax
	or edx, esi
	jne @@nextdigit
	cmp ch,0
	je @F
	dec ebx
	mov [ebx],ch
@@:
	mov eax,ebx
	ret

di64toa ENDP

dprintf PROC c uses rax rbx rcx rdx rsi rdi fmt:ptr, args:VARARG

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
	call WriteChr
	jmp nextchar
done:
	ret 

formatitem:
	lea rax, offset @@L335
	push rax
	xor edx,edx
	mov [longarg],dl
	mov bl,1
	mov cl,' '
	cmp BYTE PTR [esi],'-'
	jne @F
	dec bl
	inc esi
@@:
	mov [flag],bl
	cmp BYTE PTR [esi],'0'
	jne @F
	mov cl,'0'
	inc esi
@@:
	mov [fillchr],ecx
	mov ebx,edx

	.while ( byte ptr [esi] >= '0' && byte ptr [esi] <= '9' )
		lodsb
		sub al,'0'
		movzx eax,al
		imul ecx,ebx,10		;ecx = ebx * 10
		add eax,ecx
		mov ebx,eax
	.endw

	mov [size_],ebx
	cmp BYTE PTR [esi],'l'
	jne @F
	mov [longarg],1
	inc esi
@@:
	lodsb
	mov [fmt],rsi
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
	mov eax,[edi]
	add edi, 8
@@:
	call WriteChr
	retn

handle_s:
	mov esi,[edi]
	add edi,8
	jmp print_string
handle_d:
handle_i:
	mov ebx,-10
	jmp @F
handle_u:
	mov ebx, 10
	jmp @F
handle_x:
	mov ebx, 16
@@:
	xor edx,edx
	mov rax,[edi]
	add edi,8
	cmp longarg,1
	jz @F
	mov eax, eax
@@:
	and ebx,ebx
	jns @F
	cdq
@@:
printnum:
	lea rsi, szTmp
	invoke di64toa, rax, esi, ebx
	mov esi, eax

print_string:		;print string ESI, size EAX
	mov eax, esi
	.while byte ptr [esi]
		inc esi
	.endw
	sub esi, eax
	xchg eax, esi
	mov ebx,size_
	sub ebx,eax
	.if flag == 1
		.while sdword ptr ebx > 0
			mov eax, [fillchr]
			call WriteChr	;print leading filler chars
			dec ebx
		.endw
	.endif

	.while byte ptr [esi]
		lodsb
		call WriteChr	;print char of string
	.endw

	.while sdword ptr ebx > 0
		mov eax, [fillchr]
		call WriteChr	;print trailing spaces
		dec ebx
	.endw
	retn

	align 4

dprintf ENDP


