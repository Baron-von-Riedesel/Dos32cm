
  1. About Dos32cm

   Dos32cm is a kind of Dos64stub variant - it allows to run programs in long
  mode, but, unlike Dos64stub, launches the program in compatibility sub-mode.
   Dos32cm is a so-called stub, a ( small ) program supposed to be added to a
  ( 32-bit PE ) binary by the link step. The stub will be executed when the
  binary is launched in DOS.
  
  During startup, Dos32cm will do:

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


  2. Requirements
  
  to run an image with Dos32cm attached one needs:

   - a 64-bit CPU
   - an installed DOS
   - an installed XMS host
   - enough extended memory to load the image

  Tested on real machines and in Qemu.


  3. How to use the Stub?

  The stub is added to a 32-bit binary thru the link step. See the sample
  Makefiles for how to do this with jwlink, MS link or pestub ( a tool borrowed
  from HX ). The image must meet the following requirements:

   - Subsystem is "native"; the image really shouldn't run as Windows app
   - no dll references ("imports") are possible

  The stubs install a tiny subset of the DPMI API. The functions that are
  supported are:
   - int 21h, ah=4Ch: terminate program
   - int 31h, ax=203h: set exception vector BL to CX:EDX ( must be 64-bit! )
   - int 31h, ax=300h: simulate real-mode interrupt BL, EDI=real-mode call
     structure.
  
   One of the samples, Mon32, allows to display memory. It shows how the
  Int21 emulation installed by Dos32cm is supposed to be used and also may
  demonstrate page fault handling if an invalid address is entered.

   Another sample, Mon32x, demonstrates how to switch from compatibility mode
  to 64-bit mode. JWasm v2.17+ has to be used to create the binary, using
  JWasm's -pe option - no idea if there exist linkers that support mixing
  32- and 64-bit code in a PE binary.


  4. Memory Layout
 
  Dos32cm doesn't map the paging tables, and the IDT is always mapped at
  0x100000h. The image itself is mapped at its prefered load address.


  5. License
  
  the source is distributed under the MIT license. See file
  COPYING for details. It was written by Andreas Grech.

