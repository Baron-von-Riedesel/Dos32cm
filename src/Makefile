
# NMake makefile, makes Dos32cm.bin
# needs JWasm

ODIR=..\build

all: $(ODIR) $(ODIR)\Dos32cm.bin

$(ODIR):
	@mkdir $(ODIR)

$(ODIR)\Dos32cm.bin: Dos32cm.asm
	@jwasm -nologo -mz -D?KD=1 -DKEYS=KBD_GR -W2 -Sg -Fl$* -Fo$(ODIR)\Dos32cm.bin Dos32cm.asm
