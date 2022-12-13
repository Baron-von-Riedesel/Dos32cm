
  1. About Dos32cm

   Dos32cm is a kind of Dos64stub variant - it allows to run programs in long
  mode, but, unlike Dos64stub, uses long mode's compatibility sub-mode.
   Dos32cm is a so-called stub, a ( small ) program supposed to be added to a
  ( 32-bit PE ) binary by the link step. The stub will be executed when the
  binary is launched in DOS.
  
  When launched, Dos32cm will do:

   - check if the cpu is 64-bit
   - check if the PE image is "acceptable"
   - check if enough XMS memory is available
   - setup IDT and page tables for 64-bit PAE paging
   - read & move the image into extended memory
   - install a small "OS" (int 21h/31h) so one may call
     real-mode DOS functions that don't need pointer translation.
   - reprogram master PIC, so IRQs 00h-07h are mapped to Int 78h-7fh
   - switch to long-mode
   - call the entry point of the loaded 32-bit image

   Dos32cm doesn't require the image to contain base relocations,
  because it should be able to load the image at its prefered load address.
  Dos32cm maps just the memory that the image needs to run, that is:
  
   - image, stack & heap
   - conventional memory region, address 0-0xfffffh
   - IDT, mapped at 0x100000.

   Compatibility mode has its very own little quirks. Interrupts and
  Exceptions must be coded in 64-bit mode, changing registers FS/GS
  is not as simple as in legacy mode and a few other things might cause
  little surprises.


  2. Requirements
  
  to run an image with Dos32cm attached one needs:

   - a 64-bit CPU
   - an installed DOS
   - an installed XMS host
   - enough extended memory to load the image


  3. How to use the Stub?

  The stub is added to a 32-bit binary thru the link step. See file
  Makefile for how to do this with MS link or jwlink. The image must
  meet the following requirements:

   - Subsystem has to be "native"; avoids the image being loaded in Win32
   - no dll references ("imports") are possible

  There's a sample supplied, Mon32.asm. Mon32 allows to
  display a few 32-bit resources. It also shows how the Int21 emulation
  installed by Dos32cm is supposed to be used.

  The stubs install a tiny subset of the DPMI API. The functions that are
  supported are:
   - int 21h, ah=4Ch: terminate program
   - int 31h, ax=203h: set exception vector BL to CX:EDX
   - int 31h, ax=300h: simulate real-mode interrupt BL, EDI=real-mode call
     structure.
  

  4. Memory Layout
 
  Dos32cm doesn't map the paging tables, and the IDT is always mapped at
  0x100000h. The image itself is mapped at its prefered load address.


  5. License
  
  the source is distributed under the MIT license. See file
  COPYING for details. It was written by Andreas Grech.

