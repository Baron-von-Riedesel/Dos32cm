
;--- poll keyboard ( US/GR key layout )

ifndef KEYS
KEYS textequ <KBD_US>
endif

;*** keyboard BIOS variables

KBDSTAT  equ   417h
BUFSTA	 equ   41Ah
BUFEND	 equ   41Ch
EBUFSTA  equ   480h
EBUFEND  equ   482h
KBDSTAT2 equ   496h

_DATA segment

lastk db 0

_DATA ends

CONST segment

%	include KEYS.inc

cntlkeystab label byte
	db 36h		;R-SH		;codes ctrl keys
	db 2Ah		;L-SH		;order matches bits in 0040h:0017h
	db 1Dh		;CTRL
	db 38h		;ALT
	db 46h		;SCROLL lock;10
	db 45h		;NUM lock	;20
	db 3Ah		;CAPS		;40
	db 52h		;INS		;80
LCTLKEYS equ $ - offset cntlkeystab

CONST ends

;*** get kbd state
;*** ZERO? if no key available

GetKbdStatus proc
	in al, 64h
	test al, 01h		; input buffer full?
	jz nokey
	mov ah, al
	in al, 60h
	test ah, 20h		; is it input from PS/2?
	jnz nokey
	call setkbdbiosvars	; set kbd BIOS variables
	mov [lastk], al
	jc nokey
	cmp al,80h
	jnc nokey
	ret
nokey:
	xor al,al
	ret

GetKbdStatus endp

;--- set kbd status bios variable
;--- used only if interrupts disabled

setkbdbiosvars proc
	push rax
	push rbx
	push rcx
	push rdi
	mov bh,al 				;check if ctrl, shift, alt, ...
	and al,7fh
	mov edi, offset cntlkeystab
	mov bl,00
	mov ecx, LCTLKEYS
	repnz scasb
	jnz nostd
	mov bl,80h
	shr bl,cl
	mov ch,bh
;	and byte ptr flat:[KBDSTAT+1],not 4 ;reset Sys-Req
	mov ax, flat:[KBDSTAT]
	and ah, not 4			; reset sys-req
	test ch, 80h			; key released or pressed?
	jz @F
	xor bl,0FFh
	and al,bl				; reset flag
	and ah,bl
	jmp setflags
@@:
	or al,bl				; set flag
	or ah,bl				; set flag
setflags:        
	cmp cl,4				; RSHIFT,LSHIFT,CTRL,ALT?
	jnb @F
	mov flat:[KBDSTAT+1],ah
	xor flat:[KBDSTAT],ah
	jmp donestat
@@:
	mov flat:[KBDSTAT],al
donestat:
	and ch,7Fh
	cmp ch,38h				;Alt key?
	jnz @F
	cmp [lastk],0E0h		;last key 0E0? 
	jnz @F
	and byte ptr flat:[KBDSTAT2],not 8	;AltGr
	and ah,08
	or flat:[KBDSTAT2],ah
@@:  
	cmp ch,52h				;INSERT IS a key!
	jz nostd
	pop rdi
	pop rcx
	pop rbx
	pop rax
	stc
	RET
nostd:
	pop rdi
	pop rcx
	pop rbx
	pop rax
	clc
	ret
setkbdbiosvars endp

; *** get char from KBD without wait

GetKbdChar proc
	call GetKbdStatus
	jz nokey
	mov al,[lastk]
	test al,80h
	jnz nokey			;key released, no further processing
	cmp al,39h			;space?
	jz isspace
	cmp al,56h
	ja getkbdxchar
	jb @F
	mov al,36h			; 56 -> 36
@@:
	cmp al,36h
	ja getkbdxchar
	mov ebx, offset normalk
	mov ah, flat:[KBDSTAT]
	test ah,3			;shift pressed
	jz @F
	mov ebx, offset shiftk
@@:
	test ah,4			;ctrl pressed?
	jz @F
	mov ebx, offset ctrlk
@@:
	test ah,8			;alt pressed?
	jz @F
	test byte ptr flat:[KBDSTAT2],8	;AltGr pressed?
	jz getkbdxchar2
	mov ebx, offset altgrk
@@:
	movzx eax,al
	add ebx, eax
	mov bl,byte ptr [ebx]
	and bl,bl
	jz getkbdxchar
	mov al,bl
	ret
isspace:
	mov ah,al
	mov al,' '
	ret
getkbdxchar2:
getkbdxchar:
	mov ah,al
	mov al,00
	ret

nokey:
	xor ax,ax
	ret
GetKbdChar endp

;--- get key and store it

KbdGetChar proc

	push rax
	push rbx
	call GetKbdChar
	and ax,ax
	jz nokey
	movzx ebx, word ptr flat:[BUFEND]
	mov flat:[ebx+400h], ax
	inc ebx
	inc ebx
	cmp bx, flat:[EBUFEND]
	jnz @F
	mov bx, flat:[EBUFSTA]
@@:
	mov flat:[BUFEND], bx
nokey:
	pop rbx
	pop rax
	ret

KbdGetChar endp

