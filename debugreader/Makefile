MMCU=atmega328p
F_CPU=16000000UL

ROOT=./

CFLAGS:= -Wall -O2
LFLAGS:=-lm -Wl,-u,vfprintf -lprintf_flt

SRCDIR=$(ROOT)src/
INCDIR=$(ROOT)include/
BUILDDIR=$(ROOT)build/
OUTPUTDIR=$(ROOT)bin/

SRC:=$(wildcard $(SRCDIR)*.c)

_OBJS=$(patsubst $(SRCDIR)%.c,%.o,$(SRC))
OBJS=$(patsubst %,$(BUILDDIR)%,$(_OBJS))

all: compile install

%.o:	$(SRCDIR)%.c
	avr-gcc -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c $(CFLAGS) -I $(INCDIR) -c $(CFLAGS) $< -o $(BUILDDIR)$@

compile: $(_OBJS)
	avr-gcc $(LFLAGS) -mmcu=$(MMCU) -o $(BUILDDIR)arduino $(OBJS)
	avr-objcopy -O ihex $(BUILDDIR)arduino $(OUTPUTDIR)arduino.hex

install:
	avrdude -c arduino -p atmega328p -P /dev/serial/by-id/* -b 115200 -U flash:w:$(OUTPUTDIR)arduino.hex
	stty -F /dev/serial/by-id/* cs8 57600 raw

.PHONY: clean
clean:
	rm -rf $(BUILDDIR)*.o $(OUTPUTDIR)arduino

