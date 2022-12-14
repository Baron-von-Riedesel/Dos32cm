
;--- DOS stub program which switches to compatibility mode and back.
;--- Note: requires at least JWasm v2.13!
;--- Also: needs a 64bit cpu in real-mode to run.
;--- To create the binary enter:
;---  JWasm -mz DOS32cm.asm

    .model tiny
    .dosseg
    option casemap:none
    .stack 5120

ifndef ?IRQ0TORM
?IRQ0TORM equ 1	; 1=route IRQ0 (timer) to real-mode
endif
?IRQ1TORM equ 1	; 1=route IRQ1 (kbd) to real-mode
?LOWVIO   equ 1	; 1=low level video out
?KD       equ 0	; 1=support kernel debugger ( not yet )

;DGROUP group _TEXT	;makes a tiny model

    .x64p

    include peimage.inc
    include dpmi.inc

    option MZ:sizeof IMAGE_DOS_HEADER   ;set min size of MZ header if jwasm's -mz option is used

?MPIC  equ 78h	; master PIC base, remapped to 78h
?SPIC  equ 70h	; slave PIC, isn't changed
?RESETLME equ 0	; 1=(re)set EFER.LME for temp switch to real-mode
?RESETPAE equ 1	; 1=(re)set CR4.PAE  for temp switch to real-mode
?IDTADR   equ 100000h	;address of IDT

EMM struct  ;XMS block move help struct
_size  dd ?
srchdl dw ?
srcofs dd ?
dsthdl dw ?
dstofs dd ?
EMM ends

;--- define a string
CStr macro text:vararg
local sym
    .const
sym db text,0
    .code
    exitm <offset sym>
endm

CStr64 macro text:vararg
local sym
	.const
sym db text,0
    .code _TEXT64
    exitm <addr sym>
endm


@lgdt macro addr
;--- 16-bit variant ok, GDT remains in conv. memory
;    db 66h
    lgdt addr
endm
@lidt macro addr
;--- 16-bit variant ok, IDT remains in 24-bit address space
;    db 66h
    lidt addr
endm

@rep macro cmd
    db 67h
    rep cmd
endm

@wait macro         ;for debugging only
local lbl1
;    push ax
lbl1:
    in al,64h       ;key from keyboard arrived?
    test al,1
    jz lbl1
    in al,60h
    cmp al,81h      ;wait for ESC released
    jnz lbl1
;    pop ax
endm

;--- 16bit start/exit code

SEL_CODE64 equ 1*8
SEL_CODE32 equ 2*8
;SEL_DATA32 equ 3*8
SEL_FLAT   equ 3*8
SEL_CODE16 equ 4*8
SEL_DATA16 equ 5*8

    .code

GDT dq 0                ; null descriptor
    dw -1,0,9A00h,0AFh  ; 64-bit code descriptor
    dw -1,0,9A00h,0CFh  ; 32-bit code descriptor
    dw -1,0,9200h,0CFh  ; 32-bit data descriptor
    dw -1,0,9A00h,0h    ; 16-bit, 64k code descriptor
    dw -1,0,9200h,0h    ; 16-bit, 64k data descriptor
if ?KD
SEL_KD equ $ - GDT
    dw 0,0,0,0
    dw 0,0,0,0
    dw 0,0,0,0
endif
SIZEGDT equ $ - GDT

GDTR label fword        ; Global Descriptors Table Register
    dw SIZEGDT-1        ; limit of GDT (size minus one)
    dd offset GDT       ; linear address of GDT
IDTR label fword        ; IDTR in long mode
    dw 256*16-1         ; limit of IDT (size minus one)
    dd ?IDTADR          ; linear address of IDT
nullidt label fword     ; IDTR for real-mode
    dw 3FFh
    dd 0

    .data

xmsaddr dd 0
dwBase  dd 0	; linear base of stub
dwCSIP  label dword
adjust  dd 0	; used during rm init only
pPML4   dd 0
retad   label fword
        dd 0
        dw SEL_CODE64
if ?KD
pminit  df 0
endif
xmshdl  dw -1
fhandle dw -1

excvectors label dword
		dd 32*2 dup (0)

    .data?

if ?MPIC ne 8
storedIntM label dword
        dd 8 dup (?)
endif
if ?SPIC ne 70h
storedIntS label dword
        dd 8 dup (?)
endif
nthdr   IMAGE_NT_HEADERS <>
sechdr  IMAGE_SECTION_HEADER <>
emm     EMM <>   ;xms block move structure
emm2    EMM <>   ;another one for nested calls
dwESP    dd ?    ;protected-mode ESP
physBase dd ?    ;physical page start memory block
physCurr dd ?    ;physical page free memory
physImg  dd ?    ;physical page image base
ImgSize dd ?     ;image size in 4kB pages
fname   dd ?     ;file name of executable
wStkBot dw ?,?   ;real-mode stack bottom, offset & segment
wFlags  dw ?     ;used to store flags register
if ?MPIC ne 8
bPICM   db ?     ;saved master PIC mask
endif
if ?SPIC ne 70h
bPICS   db ?     ;saved slave PIC mask
endif

    .code

MapPages proto stdcall :dword, :dword, :dword

start16 proc

    push cs
    pop ds
    mov ax,ss
    mov dx,es
    sub ax,dx
    mov bx,sp
    shr bx,4
    add bx,ax
    mov ax,bx
    sub ax,10h
    shl ax,4
    push ds
    pop ss
    mov sp,ax       ; make a TINY model, CS=SS=DS
    mov wStkBot+0,ax
    mov wStkBot+2,ss
    mov ah,4Ah
    int 21h         ; free unused memory

    mov es,es:[002Ch]
    xor di,di
    xor al,al
    mov cx,-1
@@:
    repnz scasb
    cmp byte ptr es:[di],0
    jnz @B
    add di,3
    mov word ptr fname+0,di
    mov word ptr fname+2,es

    mov ax,cs
    movzx eax,ax
    shl eax,4
    mov [dwBase], eax
    add dword ptr [GDTR+2], eax ; convert offset to linear address
    mov word ptr [GDT + SEL_CODE16 + 2], ax
    mov word ptr [GDT + SEL_DATA16 + 2], ax
    shr eax,16
    mov byte ptr [GDT + SEL_CODE16 + 4], al
    mov byte ptr [GDT + SEL_DATA16 + 4], al

    smsw ax
    test al,1
    mov bp,CStr("Mode is V86. Need REAL mode to switch to LONG mode!")
    jnz @@error
    xor edx,edx
    mov eax,80000001h   ; test if long-mode is supported
    cpuid
    bt edx,29
    mov bp,CStr("No 64bit cpu detected.")
    jnc @@error
    mov ax,4300h
    int 2fh         ;XMS host available?
    test al,80h
    mov bp,CStr("No XMS host detected.")
    jz @@error
    mov ax,4310h
    int 2fh
    mov word ptr [xmsaddr+0],bx
    mov word ptr [xmsaddr+2],es

    mov ah,5        ;local enable A20
    call xmsaddr

    push ds
    lds dx,fname
    mov ax,3D00h
    int 21h
    pop ds
    mov bp,CStr("cannot open file.")
    jc  @@error
    mov fhandle,ax
    mov bx,ax
;--- load the file header
    sub sp,4096
    mov cx,sizeof IMAGE_DOS_HEADER
    mov dx,sp
    mov ah,3Fh
    int 21h
    cmp ax,cx
    mov bp,CStr("invalid file format.")
    jnz @@error
    mov di,sp
    cmp word ptr [di].IMAGE_DOS_HEADER.e_magic,"ZM"
    mov bp,CStr("invalid file format (no MZ header).")
    jnz @@error
    cmp word ptr [di].IMAGE_DOS_HEADER.e_lfarlc,sizeof IMAGE_DOS_HEADER
    mov bp,CStr("invalid file format (MZ header too small).")
    jb @@error
    mov cx,word ptr [di].IMAGE_DOS_HEADER.e_lfanew+2
    mov dx,word ptr [di].IMAGE_DOS_HEADER.e_lfanew+0
    mov ax,4200h
    int 21h
    mov dx,offset nthdr
    mov cx,sizeof IMAGE_NT_HEADERS
    mov ah,3Fh
    int 21h
    cmp ax,cx
    mov bp,CStr("invalid file format (cannot locate PE header).")
    jnz @@error
    cmp dword ptr nthdr.Signature,"EP"
    mov bp,CStr("invalid file format (no PE header).")
    jnz @@error
    cmp nthdr.FileHeader.Machine,IMAGE_FILE_MACHINE_I386
    mov bp,CStr("not a 32-bit binary.")
    jnz @@error
;    test nthdr.FileHeader.Characteristics,IMAGE_FILE_RELOCS_STRIPPED
;    mov bp,CStr("relocations stripped, cannot load.")
;    jnz @@error
    cmp nthdr.OptionalHeader.Subsystem,IMAGE_SUBSYSTEM_NATIVE
    mov bp,CStr("subsystem not native, cannot load.")
    jnz @@error
    cmp nthdr.OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT*sizeof IMAGE_DATA_DIRECTORY].Size_,0
    mov bp,CStr("image contains imports, cannot load.")
    jnz @@error
if 0
    cmp dword ptr nthdr.OptionalHeader.SizeOfStackReserve+4,0
    mov bp,CStr("requested stack size of image is > 4 GB.")
    jnz @@error
endif
if 0
    cmp dword ptr nthdr.OptionalHeader.SizeOfHeapReserve+4,0
    mov bp,CStr("requested heap size of image is > 4 GB.")
    jnz @@error
endif
    mov edx, nthdr.OptionalHeader.SizeOfImage
    mov eax, dword ptr nthdr.OptionalHeader.SizeOfStackReserve
    shr edx,12      ;convert to 4kB pages
    shr eax,12      ;convert to 4kB pages
    add edx, eax

    mov eax, dword ptr nthdr.OptionalHeader.SizeOfHeapReserve
    shr eax,12
    add edx, eax

;--- edx = size of image in 4kB pages
    mov ImgSize, edx

;--- add space for IDT and page tables
;--- needed: 1 page for IDT
;---         (ImgSize+1) / 512 pages for PTs (1 PT maps 2MB)
;---         (ImgSize+1) / (512*512) pages for PDs (1 PD maps 1GB)
;---         (ImgSize+1) / (512*512*512) pages for PDPT (1 PDPT maps 512 GB)
;---         1 page for PML4

    inc edx     ;add the page for IDT

    mov eax,edx
    shr eax,9   ;eax = pages for PTs
    add edx,eax
    shr eax,9
    add edx,eax ;eax = pages for PDs
    shr eax,9   ;eax = pages for PDPTs
    add edx,eax
;--- additional pages needed
;--- 3 for remainder of PDPT,PD,PT
;--- 1 for PML4
;--- 3 (PT,PD,PDPT) for mapping DOS conventional memory
;--- 1 extra page that may be needed to align to page boundary
    add edx,3+1+3+1

    test edx,0C0000000h
    mov bp,CStr("too much extended memory needed")
    jnz @@error
    push edx
    shl edx,2

;--- allocate the extended memory block needed for image + systabs
    mov ah,89h
    call xmsaddr
    cmp ax,1
    mov bp,CStr("XMS memory allocation failed.")
    jnz @@error
    mov xmshdl, dx
    mov emm.dsthdl, dx
    mov emm2.dsthdl, dx
    mov ah,0Ch      ;lock EMB 
    call xmsaddr
    cmp ax,1
    mov bp,CStr("cannot lock EMB.")
    jnz @@error
    push dx
    push bx

;--- clear the whole block
    call EnableUnreal
    pop ebx ;get base
    pop ecx ;get size (in pages)
    mov edi,ebx
    shl ecx,10  ;pages to dword
    xor eax,eax
    @rep stosd
    push ds
    pop es

;--- align to page boundary

    mov ax,bx
    neg ax
    add ax,1000h
    and ax,0fffh   ;0000,0400,0800,0C00 converted to 0000, 0C00, 0800, 0400
    movzx eax,ax
    mov adjust, eax
    add eax, ebx
    mov pPML4, eax      ; allocate the first page for PML4
    shr eax,12
    mov physBase, eax
    inc eax
    mov physCurr, eax	; start of free physical pages

;--- prepare EMB moves
    mov emm.srchdl, 0
    mov emm2.srchdl, 0
    mov word ptr emm.srcofs+2, ds
    mov word ptr emm2.srcofs+0, sp
    mov word ptr emm2.srcofs+2, ss

;---  map conventional memory
    invoke MapPages, 0, 0, 256

;--- init IDT
;--- setup ebx with linear address of _TEXT64
    mov ebx, _TEXT64
    shl ebx, 4
    mov di,sp
    call createIDT

    mov eax,physCurr
    inc physCurr
    push eax
    sub eax,physBase
    shl eax,12
    add eax,adjust
    mov emm2.dstofs, eax
    mov ecx, 1000h		;copy IDT to ext memory
    mov si, offset emm2
    call copy2ext
    pop ecx

;--- map IDT at 0x100000h
    invoke MapPages, ?IDTADR, ecx, 1

    mov eax,physCurr
    mov ecx,ImgSize
;    shl ecx,12
    add physCurr, ecx
    mov physImg, eax
    sub eax, physBase
	shl eax,12
    add eax, adjust
    mov emm.dstofs, eax

    mov ecx, sizeof IMAGE_NT_HEADERS
    mov si, offset emm
    mov word ptr [si].EMM.srcofs+0, offset nthdr
    call copy2ext ;copy PE header (ecx bytes) to extended memory
    mov word ptr [si].EMM.srcofs+0, offset sechdr

;--- now read & copy section headers ony by one;
;--- for each header read & copy section data.
    mov bx,fhandle
    xor cx,cx
    xor dx,dx
    mov ax,4201h	;get current file pos in DX:AX
    int 21h
    mov cx,nthdr.FileHeader.NumberOfSections
    .while cx
        push cx
        push dx
        push ax
        mov cx,dx
        mov dx,ax
        mov ax,4200h	;set file pos to CX:DX
        int 21h
        mov dx,offset sechdr
        mov cx,sizeof IMAGE_SECTION_HEADER
        mov ah,3Fh
        int 21h
        cmp ax,cx
        mov bp,CStr("cannot load section headers.")
        jnz @@error
        mov si,offset emm
        call copy2ext	;copy section header to PE header in image
        call readsection
        pop ax
        pop dx
        add ax,sizeof IMAGE_SECTION_HEADER
        adc dx,0
        pop cx
        dec cx
    .endw

    add sp,4096

    mov ah,3Eh
    int 21h
    mov fhandle,-1

;--- check that image base is either 
;--- in range 0-7fffffffffffh
;--- or in range ffff800000000000h-ffffffffffffffffh.
;--- then create address space for image.

if 0
    mov bp,CStr("Cannot map image; check image base!")
    mov eax,dword ptr nthdr.OptionalHeader.ImageBase+4
    shr eax,15
    jz @F
    cmp eax,1ffffh
    jnz @@error
@@:
endif
    invoke MapPages, nthdr.OptionalHeader.ImageBase, physImg, ImgSize
    jc @@error

;--- done setup the extended memory block;
;--- page tabs, IDT and image are initialized.

;--- disable int 23h termination (myint23)
;--- or exit program (@@exit2)
;    mov dx,offset myint23
    mov dx,offset @@exit2
    mov ax,2523h
    int 21h

    call setints

    cli

    mov eax, pPML4
    mov cr3, eax        ; load page-map level-4 base

if ?MPIC ne 8
    in al,21h
    mov bPICM,al
endif
if ?SPIC ne 70h
    in al,0A1h
    mov bPICS,al
endif
    mov dx,?SPIC shl 8 or ?MPIC
    call setpic

ife ?RESETLME
    mov ecx,0C0000080h  ; EFER MSR
    rdmsr               ; value is returned in EDX::EAX!
    or ah,1             ; enable long mode
    wrmsr
endif

    call switch2pm

    mov ebx,nthdr.OptionalHeader.ImageBase
    mov ecx,nthdr.OptionalHeader.SizeOfStackReserve
    add ecx,nthdr.OptionalHeader.SizeOfImage
    add ecx,ebx
    mov esi,nthdr.OptionalHeader.AddressOfEntryPoint
    add esi,ebx

    mov ax,SEL_FLAT
    mov ss,ax
    mov esp,ecx
    mov ds,ax
    mov es,ax
if 0 ; don't set fs/gs in long mode
    xor ax,ax
    mov fs,ax
    mov gs,ax
endif

;--- clear NT flag
    pushf
    and byte ptr [esp+1], 3Fh
    popf

;---  get 32-bit entry

    mov eax,seg start32
    shl eax,4
    add eax, offset start32
    pushd SEL_CODE32
    push eax
    retd

@@error::
    mov si,bp
nextchar:
    lodsb
    and al,al
    jz done
    mov dl,al
    mov ah,2
    int 21h
    jmp nextchar
newline db 13,10,'$'
done:
    mov dx,offset newline
    mov ah,9
    int 21
    jmp @@exit
myint23:
    iret
start16 endp

;--- create address space and map pages
;--- linaddr: start linear address space to create
;--- physpage: start physical page to map
;--- pages: no of pages to map

MapPages proc stdcall uses es linaddr:dword, physpage:dword, pages:dword

    call EnableUnreal
    .while pages
;        mov eax,dword ptr linaddr+0
;        mov edx,dword ptr linaddr+4
        mov eax, linaddr
        xor edx, edx

        mov cx,3
@@:
        shrd eax,edx,9
        shr edx,9
        push eax
        loop @B
;--- 3 offsets pushed:
;--- 1. bits 0-11 offset in PT   (linaddr [12-20])
;--- 2. bits 0-11 offset in PD   (linaddr [21-29])
;--- 3. bits 0-11 offset in PDPT (linaddr [30-38])
        shr eax,9           
;--- eax bits 0-11 offset in PML4 (linaddr [39-47])
;--- eax bits 11-27 [bitmask 0ffff800] must be identical
        and eax,0ff8h
        mov cx,3
        mov ebx,pPML4
@@:
        add ebx,eax
        mov edx,es:[ebx]
        .if edx == 0             ;does PDPTE/PDE/PTE exist?
            mov edx,physCurr
;--- the paging tables must be located below 4GB;
;--- if this is to be changed, unreal mode cannot be used here!
;            sub eax,eax
;            shld eax,edx,12
            shl edx,12
            or dl,11b
            mov es:[ebx+0],edx
;            mov es:[ebx+4],eax
            inc physCurr
        .endif
        pop eax
        and eax,0ff8h
        mov ebx,edx
;       and bx,0FC00h
        and bx,0F000h
        loop @B

        add ebx,eax
        test byte ptr es:[ebx],1     ;something already mapped there?
        stc
        jnz exit
        mov edx,physpage
        sub eax,eax
        shld eax,edx,12
        shl edx,12
        or dl,11b
        mov es:[ebx+0],edx
        mov es:[ebx+4],eax
        inc physpage
        add dword ptr linaddr+0,1000h
;        adc dword ptr linaddr+4,0
        dec pages
    .endw
    clc
exit:
    ret

MapPages endp

EnableUnreal proc
    cli
    @lgdt [GDTR]
    mov eax,cr0
    or al,1
    mov cr0,eax
    jmp @F
@@:
    push SEL_FLAT
    pop es
    and al,0FEh
    mov cr0,eax
    jmp @F
@@:
    sti
    xor ax,ax
    mov es,ax
    ret
EnableUnreal endp

;--- copy cx bytes to extended memory
;--- ds:si -> emm struct

copy2ext proc
    mov [si].EMM._size,ecx
    push ecx
    push bx
    mov ah,0bh
    call xmsaddr
    pop bx
    pop ecx
    cmp ax,1
    mov bp,CStr("error copying to extended memory.")
    jnz @@error
    add [si].EMM.dstofs,ecx
    ret
copy2ext endp

;--- read a section and copy it to extended memory
;--- DI = 4 kB buffer
;--- BX = file handle
;--- sechdr = current section

readsection proc

    mov eax, physImg
    sub eax, physBase
    shl eax,12
    add eax, adjust
    add eax, sechdr.VirtualAddress
    mov emm2.dstofs, eax

    mov dx, word ptr sechdr.PointerToRawData+0
    mov cx, word ptr sechdr.PointerToRawData+2
    mov ax,4200h
    int 21h
    mov esi, sechdr.SizeOfRawData
    .while esi
        mov ecx,esi
        cmp ecx,1000h
        jb @F
        mov cx,1000h
@@:
        mov dx,di
        mov ah,3Fh
        int 21h
        cmp ax,cx
        mov bp, CStr("cannot read section data.")
        jnz @@error
        sub esi, ecx
        push si
        mov si,offset emm2
        call copy2ext
        pop si
    .endw
    ret
readsection endp

;--- switch back to real-mode and exit

backtoreal proc

    mov ax,SEL_DATA16   ; set SS, DS and ES to 16bit, 64k data
    mov ds,ax
    mov es,ax
    mov ss,ax
    mov sp,[wStkBot]
    call switch2rm

@@exit2::
    mov ax, cs
    mov ss, ax          ; SS=DGROUP
    mov ds, ax          ; DS=DGROUP

ife ?RESETLME
    mov ecx,0C0000080h  ; EFER MSR
    rdmsr
    and ah,0feh         ; disable long mode (EFER.LME=0)
    wrmsr
endif
ife ?RESETPAE
    mov eax,cr4
    and al,0DFh         ; reset bit 5, disable PAE paging
    mov cr4,eax
endif
    mov dx,7008h
    call setpic
    call restoreints
@@exit::
    sti
    mov bx,fhandle
    cmp bx,-1
    jz @F
    mov ah,3Eh
    int 21h
@@:
    mov dx,xmshdl
    cmp dx,-1
    jz @F
    mov ah,0dh          ;unlock handle
    call xmsaddr
    mov ah,0Ah          ;free EMB
    mov dx,xmshdl
    call xmsaddr
@@:
    cmp xmsaddr,0
    jz @F
    mov ah,6            ;local disable A20
    call xmsaddr
@@:
    mov ax,4c00h
    int 21h
backtoreal endp

;--- switch to real-mode
;--- set CR0, EFER, CR4, IDTR

switch2rm proc
;--- disable paging & protected-mode
    mov eax,cr0
    and eax,7ffffffeh
    mov cr0, eax
    jmp far16 ptr @F
@@:
;--- disable long mode
if ?RESETLME
    mov ecx,0C0000080h  ; EFER MSR
    rdmsr
    and ah,0feh
    wrmsr
endif
if ?RESETPAE
    mov eax,cr4
    and al,0DFh         ; reset bit 5, disable PAE paging
    mov cr4,eax
endif
    @lidt cs:[nullidt]  ; IDTR=real-mode compatible values
    ret
switch2rm endp

;--- switch to protected-mode
;--- set GDTR, IDTR, EFER, CR4, CR3, CR0

switch2pm proc
    @lgdt cs:[GDTR]
    @lidt cs:[IDTR]
;--- (re)enable long mode
if ?RESETLME
    mov ecx,0C0000080h  ; EFER MSR
    rdmsr
    or ah,1
    wrmsr
endif
if ?RESETPAE
    mov eax,cr4
    or ax,220h          ; enable PAE (bit 5) and OSFXSR (bit 9)
    mov cr4,eax
endif
    mov eax,cs:pPML4
    mov cr3,eax
;--- enable protected-mode + paging
    mov eax,cr0
    or eax,80000001h
    mov cr0,eax
    ret
switch2pm endp

;--- call real-mode thru DPMI function ax=0x300
;--- interrupts disabled
;--- ESP is still flat
;--- ESP-> CS:EIP, then modified RMCS, without CS:IP;
;--- variable dwCSIP contains real-mode CS:IP

call_rmode proc

;--- switch stack to 16-bit, SS=DGROUP
    mov dx, SEL_DATA16
    movzx eax, cs:[wStkBot+2]
    shl eax, 4
    mov ss, dx
    sub esp, eax

    call switch2rm
    pop dword ptr cs:retad
    add sp,4   ;skip CS
    popad
    pop cs:wFlags
    pop es
    pop ds
    pop fs
    pop gs
    lss sp,[esp]
    push cs:wFlags   ;make an IRET frame
    call cs:[dwCSIP]
    lss sp,dword ptr cs:wStkBot
    push gs
    push fs
    push ds
    push es
    pushf
    cli
    pushad
    movzx esp,sp
    call switch2pm

;--- switch stack back to flat
    mov dx, SEL_FLAT
    movzx eax, cs:[wStkBot+2]
    shl eax, 4
    mov ss, dx
    add esp, eax

;--- also restore DS/ES, the 64-bit code can't do that
    mov ds, dx
    mov es, dx

    jmp cs:[retad]

call_rmode endp

;--- initialize interrupt gates in IDT 64-bit

make_int_gates proc
    mov eax, edx
    add eax, ebx
    stosw
    mov ax,SEL_CODE64
    stosw
    mov ax,si           ;int/trap gate
    stosd
    xor eax, eax
    stosd
    stosd
    loop make_int_gates
    ret
make_int_gates endp

;--- create IDT for long mode
;--- DI->4 KB buffer
;--- EBX->linear address of _TEXT64 segment
;--- interrupts and exceptions must be in 64-bit mode!

createIDT proc

    push di
    mov cx,32
    mov edx, offset exception
    add edx, ebx
make_exc_gates:
    mov eax,edx
    stosw
    mov ax,SEL_CODE64
    stosw
    mov ax,8E00h
    stosd
    xor eax, eax
    stosd
    stosd
    add edx,4
    loop make_exc_gates
    mov ecx,256-32
    mov edx,offset swint
    mov si, 8F00h
    call make_int_gates
    pop di

    push di
    push di
    lea di,[di+?MPIC*16]
    mov cx,8
    mov edx,offset Irq0007
    mov si, 8E00h
    call make_int_gates
    pop di
    lea di,[di+?SPIC*16]
    mov cx,8
    mov edx,offset Irq080F
    call make_int_gates
    pop di

;--- setup IRQ0, IRQ1, Int21, Int31

    mov si,offset tab1
    mov cx,sizetab1
nextitem:
    lodsw
    mov dx,ax
    lodsw
    movzx eax,ax
    add eax, ebx
    shl dx,4		; 16 bytes per vector
    push di
    add di,dx
    mov [di],ax
    shr eax,16
    mov [di+6],ax
    pop di
    loop nextitem
    ret

tab1 label word
    dw ?MPIC+0, offset clock
    dw ?MPIC+1, offset kbd
    dw 21h,     offset int21
    dw 31h,     offset int31
sizetab1 equ ($-tab1) shr 2

createIDT endp

;--- restore the interrupt vectors that we have modified
;--- DS=DGROUP

restoreints proc
    push 0
    pop es
if ?MPIC ne 8
    mov cx,8
    mov di,?MPIC*4
    mov si,offset storedIntM
    rep movsd
endif
if ?SPIC ne 70h
    mov cx,8
    mov di,?SPIC*4
    mov si,offset storedIntS
    rep movsd
endif
    ret
restoreints endp

;--- set the interrupt vectors that we will
;--- use for IRQs while in long mode. This avoids
;--- having to reprogram PICs for switches to real-mode
;--- DS=DGROUP

setints proc
    push 0
    pop es
if ?MPIC ne 8
    mov cx,8
    mov bx,?MPIC*4
    mov di,offset storedIntM
@@:
    mov eax, es:[bx]
    mov [di], eax
    add bx,4
    add di,4
    loop @B
endif
if ?SPIC ne 70h
    mov cx,8
    mov bx,?SPIC*4
    mov di,offset storedIntS
@@:
    mov eax, es:[bx]
    mov [di], eax
    add bx,4
    add di,4
    loop @B
endif
    push ds
    push es
    pop ds
if ?MPIC ne 8
    mov cx,8
    mov si,8*4
    mov di,?MPIC*4
    rep movsd
endif
if ?SPIC ne 70h
    mov cx,8
    mov si,70h*4
    mov di,?SPIC*4
    rep movsd
endif
    pop ds
    ret

setints endp

;--- reprogram/restore PIC
;--- DS=DGROUP

setpic proc

;--- change IRQ 0-7 to ?MPIC
if ?MPIC ne 8
    mov al,10001b       ; ICW1: initialization
    out 20h,al
    mov al,dl           ; ICW2: IRQ 0-7: interrupts ?MPIC-?MPIC+7
    out 21h,al
    mov al,100b         ; ICW3: slave connected to IRQ2
    out 21h,al
    mov al,1            ; ICW4: Intel environment, manual EOI
    out 21h,al
    mov al,bPICM
    out 21h,al
endif
;--- change IRQ 8-F to ?SPIC
if ?SPIC ne 70h
    mov al,10001b       ; ICW1: initialization
    out 0A0h,al
    mov al,dh           ; ICW2: IRQ 8-15: interrupts ?SPIC-?SPIC+7
    out 0A1h,al
    mov al,2            ; ICW3:
    out 0A1h,al
    mov al,1            ; ICW4: Intel environment, manual EOI
    out 0A1h,al
    mov al,bPICS
    out 0A1h,al
endif
    ret
setpic endp

;--- here's the 32bit code segment.

_TEXT32 segment para use32 public 'CODE'
_TEXT32 ends

	.code _TEXT32

start32 proc

    sti
    call esi
    mov ah,4Ch
    int 21h

start32 endp

if 0
;--- handle base relocs of PE image

baserelocs proc
    mov esi, [ebx].IMAGE_NT_HEADERS.OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_BASERELOC*sizeof IMAGE_DATA_DIRECTORY].VirtualAddress
    mov ecx, [ebx].IMAGE_NT_HEADERS.OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_BASERELOC*sizeof IMAGE_DATA_DIRECTORY].Size_
    mov edx, ebx
    sub edx, [ebx].IMAGE_NT_HEADERS.OptionalHeader.ImageBase
    add esi, ebx    ;RVA->linear
    add ecx, esi    ;ecx=end of relocs (linear)
nextpage:
    cmp esi, ecx
    jnc reloc_done
    push ecx
    lodsd           ;get RVA of page
    lea edi, [eax+ebx]  ;convert RVA to linear address
    lodsd
    lea ecx, [esi+eax-8];ecx=end of relocs for this page
    xor eax, eax
nextreloc:
    lodsw
    test ah,0F0h        ;must be < 1000h (size of a page)
    jz ignreloc
    and ah,0Fh          ;usually it's type 0A (dir64)
    add [eax+edi], edx
ignreloc:
    cmp esi, ecx
    jb nextreloc
    pop ecx
    jmp nextpage
reloc_done:
    ret
baserelocs endp

endif

;--- here's the 64bit code segment.
;--- it's needed even in compatibility mode because
;--- interrupt & exceptions must reside in 64-bit code.

_TEXT64 segment para use64 public 'CODE'
_TEXT64 ends

    .code _TEXT64

;--- screen output for default exception handlers

if ?LOWVIO
    include vioout.inc
    include dprintf.inc
endif
WriteChr proc
    cmp al,10
    jnz @F
    mov al,13
    call @F
    mov al,10
@@:
    push rdx
    mov dl,al
ife ?LOWVIO 
    mov ah,2
    int 21h
else
    call VioPutChar
endif
    pop rdx
    RET
WriteChr endp

WriteStrX proc  ;write string at rip
    push rsi
    mov esi, [esp+8]
    cld
@@:
    lodsb
    and al,al
    jz @F
    call WriteChr
    jmp @B
@@:
    mov [esp+8],esi
    pop rsi
    ret
WriteStrX endp

WriteQW:        ;write QWord in rax
    push rax
    shr rax,32
    call WriteDW
    pop rax
WriteDW:
    push rax
    shr rax,16
    call WriteW
    pop rax
WriteW:
    push rax
    shr rax,8
    call WriteB
    pop rax
WriteB:     ;write Byte in al
    push rax
    shr rax,4
    call WriteNb
    pop rax
WriteNb:
    and al,0Fh
    add al,'0'
    cmp al,'9'
    jbe @F
    add al,7
@@:
    jmp WriteChr

;--- exception handler
;--- might not work for exception 0Ch, since stack isn't switched.

exception:
excno = 0
    repeat 32
    push excno
    jmp @F
    excno = excno+1
    endm
@@:

    push rax
    mov eax, [esp+8]
    shl eax, 3
    add eax, [dwBase]
    cmp word ptr [eax+offset excvectors+4], 0
    jnz routeexc
    pop rax


    call WriteStrX
    db 10,"Exception ",0
    pop rax
    call WriteB
    call WriteStrX
    db " rsp=",0
    mov rax, rsp
    call WriteQW
    call WriteStrX
    db " imagebase=",0
;    mov rax,nthdr.OptionalHeader.ImageBase
    mov eax, nthdr.OptionalHeader.ImageBase
    call WriteDW
    call WriteStrX
    db " stubbase=",0
    mov eax, [dwBase]
    call WriteDW
if 0
    call WriteStrX
    db " rsi=",0
    mov rax,rsi
    call WriteQW
    call WriteStrX
    db " rdi=",0
    mov rax,rdi
    call WriteQW
endif

    call WriteStrX
    db 10,"[rsp]=",0
    xor ecx,ecx
@@:
    mov rax,[rsp+rcx*8]
    call WriteQW
    mov al,' '
    call WriteChr
    inc ecx
    cmp ecx,4
    jnz @B
    call WriteStrX
    db 10,"      ",0
@@:
    mov rax,[rsp+rcx*8]
    call WriteQW
    mov al,' '
    call WriteChr
    inc ecx
    cmp ecx,8
    jnz @B
    mov al,10
    call WriteChr

    mov ax,4cffh
    int 21h

;--- route exception to exception handler;
;--- frame is 64-bit, though.

routeexc:
    push rcx
    mov ecx, [eax+offset excvectors]
    mov ax, word ptr [eax+offset excvectors+4]
    mov [esp+2*8+0], ecx
    mov [esp+2*8+4], ax
    pop rcx
    pop rax
    retf


;--- IRQs 0-7

;--- macro @call_rm_irq, defines 16-bit part of interrupt routing

@call_rm_irq macro procname, interrupt
procname proc
    push eax	; here in 16-bit protected-mode
if ?RESETLME
    push ecx
    push edx
endif

;--- we're on a 32-bit flat stack, switch to 16-bit stack
    mov eax, esp
    push SEL_DATA16
    pop ss
    mov sp,cs:[wStkBot]
    push eax

    call switch2rm  ;modifies eax [, ecx, edx]
;--- SS still holds a selector - hence a possible temporary 
;--- stack switch inside irq handler would cause a crash.
    mov ss,cs:[wStkBot+2]
    int interrupt
    cli
    call switch2pm  ;modifies eax [, ecx, edx]
    pop eax
    push SEL_FLAT
    pop ss
    mov esp, eax

;--- reload DS/ES
    mov ax, ss
    mov ds, ax
    mov es, ax

if ?RESETLME
    pop edx
    pop ecx
endif
    pop eax
    retd
procname endp
endm

;--- macro @route_irq, defines 64-bit part of interrupt routing

@route_irq macro interrupt, prefix
    .code
    @call_rm_irq prefix&_rm,interrupt
    .data
p&prefix&_rm label fword
    dd offset prefix&_rm
    dw SEL_CODE16
    .code _TEXT64
prefix:
    call p&prefix&_rm
    iretq
endm

;--- route irq 1 (kbd) to real-mode
If ?IRQ1TORM
    @route_irq 09h, kbd
else
	include kbdinp.inc
kbd:
	call KbdGetChar
	jmp Irq0007
endif

if ?IRQ0TORM
;--- route irq 0 (pit clock) to real-mode
    @route_irq 08h, clock
else
clock:
    inc dword ptr flat:[46Ch]
endif
Irq0007:
    push rax
Irq0007_1:
    mov al,20h
    out 20h,al
    pop rax
swint:
    iretq
;--- IRQs 8-F
Irq080F:
    push rax
    mov al,20h
    out 0A0h,al
    jmp Irq0007_1

;--- load 32-bit registers ( clears upper 32bits of 64-bit regs, which doesn't matter in compatibility mode ). 

@loadreg macro reg
if 0
    push R&reg
    mov E&reg,[rsp+8].RMCS.rE&reg
    mov [rsp],E&reg
    pop R&reg
else
    mov E&reg,[esp].RMCS.rE&reg
endif
endm

;--- simple int 21h handler.
;--- handles ah=4Ch
;--- any other DOS function is transfered to real-mode

int21 proc
    cmp ah,4Ch
    jz int21_4c
    and byte ptr [esp+2*8],0FEh ;clear carry flag
    sub esp,38h
    mov [esp].RMCS.rEDI, edi
    mov [esp].RMCS.rESI, esi
    mov [esp].RMCS.rEBP, ebp
    mov [esp].RMCS.rEBX, ebx
    mov [esp].RMCS.rEDX, edx
    mov [esp].RMCS.rECX, ecx
    mov [esp].RMCS.rEAX, eax
    mov [esp].RMCS.rFlags, 0202h
;    mov [esp].RMCS.rES, es
;    mov [esp].RMCS.rDS, ds
    mov ax, [wStkBot+2]
    mov [esp].RMCS.rES, ax
    mov [esp].RMCS.rDS, ax
    mov [esp].RMCS.rFS, fs
    mov [esp].RMCS.rGS, gs
    mov dword ptr [esp].RMCS.regSP, 0
    push rdi
    lea edi,[esp+8]
    mov bx,21h
    mov cx,0
    mov ax,0300h
    int 31h
    pop rdi
    jc int21_carry
    mov al,byte ptr [esp].RMCS.rFlags
    mov byte ptr [esp+38h+2*8],al    ;set CF,ZF,...
    jmp @F
int21_carry:
    or  byte ptr [esp+38h+2*8],1    ;set carry flag
@@:
    @loadreg DI
    @loadreg SI
    @loadreg BP
    @loadreg BX
    @loadreg DX
    @loadreg CX
    @loadreg AX
    lea esp,[esp+38h]
    iretq

    .data

pback_to_real label ptr far16
    dw offset backtoreal
    dw SEL_CODE16

    .code _TEXT64

int21_4c:
    cli
    jmp [pback_to_real]

int21 endp

int31 proc

    and byte ptr [esp+2*8],0FEh	;clear carry flag
    cmp ax,0300h	;simulate real-mode interrupt?
    jz int31_300
    cmp ax,0203h	;set exception vector?
    jz int31_203
ret_with_carry:
    or byte ptr [esp+2*8],1 ;set carry flag
    iretq
    .data
pcall_rmode label ptr far32
    dd offset call_rmode
    dw SEL_CODE16
    .code _TEXT64
int31_300:
    push rax
    push rcx
    push rdx
    push rbx
    push rbp
    push rsi
    push rdi
    mov esi, edi

;--- the contents of the RMCS has to be copied
;--- to conventional memory. We use the DGROUP stack

    movzx ecx, [wStkBot+2]
    shl ecx, 4
    movzx ebx, bl
    mov eax, [ebx*4]
    mov bx, [wStkBot] 
    sub bx, 30h
    lea edi, [ebx+ecx]
    mov [dwCSIP], eax
    mov [dwESP], esp
    mov esp, edi
    cld
    cli
    movsq   ;copy 2Ah bytes
    movsq
    movsq
    movsq
    movsq
    movsw
    lodsd   ;get CS:IP
    lodsd   ;get SS:SP
    and eax,eax ;is a real-mode stack set?
    jnz @F
    mov eax,dword ptr [wStkBot] ;if no, use the default stack
@@:
    stosd
    call [pcall_rmode]
    mov esi, esp
    mov esp, [dwESP]
    mov edi, [esp]
    cld
    movsq   ;copy 2Ah bytes back, don't copy CS:IP & SS:SP fields
    movsq
    movsq
    movsq
    movsq
    movsw
;    sti
    pop rdi
    pop rsi
    pop rbp
    pop rbx
    pop rdx
    pop rcx
    pop rax
    iretq

;--- exception vectors in IDT must be 64-bit!
;--- so we don't modify the IDT directly, instead maintain a table
;--- of 32-bit vectors.

int31_203:
    cmp bl,20h
    jae ret_with_carry
    push rbx

;--- while direct memory accesses are handled correctly ( due to RIP-relative
;--- addressing in 64-bit ), offsets are different - they're still relative
;--- to DGROUP; there's no automatic fixup; hence we need to add DGROUP's
;--- linear address.

    movzx ebx, bl
    shl ebx, 3
    add ebx, [dwBase]
    add ebx, offset excvectors
;    invoke dprintf, CStr64("int 31h, 203: ebx=%X",10), rbx
    mov [ebx+0], edx
    mov [ebx+4], cx
    pop rbx
    iretq
int31 endp

    end start16
