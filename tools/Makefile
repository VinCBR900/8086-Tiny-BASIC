CC ?= Tcc
CFLAGS ?= -Wall -O2

.PHONY: all clean

all: asm2650 pipbug_wrap

asm2650: asm2650.c
	gcc $(CFLAGS) -o $@ $<

pipbug_wrap: pipbug_wrap.c 2650.c aa.h aa_strings.h
	gcc $(CFLAGS) -DGAMER -o $@ pipbug_wrap.c

clean:
	rm -f asm2650 pipbug_wrap
