## Tools
this folder contains Tool used in 8088 Tiny BASIC Development, copied here so CODEX can access them.  

### Assembler
The Assembler is Oscar Toledo's **tinyasm.c** from 

https://github.com/nanochess/tinyasm

### PC Simulator
The PC Simulator is Adrian Cable's **tiny8086** suite from 

https://github.com/adriancable/8086tiny

We compile the simulator with NO_GRAPHICS for STDIO character I/O to enable batch processing

The bootsector is how we test our code by appending the BASIC binary to the bootsector binary as the floppy image - no DOS, just BIOS interrupts

### Embedded Simulator
We also use the CPU core from Mike Chamber's **XTulator** project to emulate the embedded target

https://github.com/mikechambers84/XTulator

We need a wrapper and stub files as not using full PC architectire with an 8088 in MIN mode.

