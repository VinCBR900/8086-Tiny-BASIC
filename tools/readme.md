## Tools
this folder contains Tool used in 8088 Tiny BASIC Development, copied here so CODEX can access them.  

### Assembler
The Assembler is Oscar Toledo's **tinyasm.c** from 

https://github.com/nanochess/tinyasm

### Embedded Simulator
We use the CPU core from Mike Chamber's **XTulator** project to emulate the embedded target

https://github.com/mikechambers84/XTulator

We need a wrapper and stub files as not using full PC architectire with an 8088 in MIN mode.

### PC Simulator
Before the CPU simulator, development used  Adrian Cable's **8086tiny** suite below.  We dropped this as the simulator (though very good) uses memory space for CPU registers which didnt work for embedded testing.

https://github.com/adriancable/8086tiny

We compiled the 8086tiny with NO_GRAPHICS for STDIO character I/O to enable batch processing.

The bootsector loaded the appended the BASIC binary to the bootsector binary as the floppy image - no DOS, just BIOS interrupts

