
ODIR=..\build

all: $(ODIR) $(ODIR)\Mon32x.exe

$(ODIR):
	@mkdir $(ODIR)

$(ODIR)\Mon32x.exe: $(ODIR)\dos32cm.bin Mon32x.asm printf.inc Makefile
	@jwasm -nologo -pe -Fl$* -Fo$* -Sg Mon32x.asm
	@pestub -n -q -s $*.exe $(ODIR)\dos32cm.bin
	@patchpe -y:1 -s:0x4000 -h:0 $*.exe

clean:
	@erase $(ODIR)\Mon32x.exe
	@erase $(ODIR)\Mon32x.lst

