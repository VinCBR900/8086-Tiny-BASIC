## Tools

this folder contains Tool sued ot 8088 Tiny BASIC Development, copied here so CODEX can access it.  

The Assembler is Oscar Toledo's tinyasm.c from #

https://github.com/nanochess/tinyasm

The simulator is tiny8086 suite from 

https://github.com/adriancable/8086tiny

We compile with NO_GRAPHICS for STDIO character I/O batch processing

The bootsector is how we test our code by appending the BASIC binary to the bootsector binary as the floppy image - no DOS, just BIOS interrupts 
