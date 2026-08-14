# 8086-Tiny-BASIC
> **AI Disclosure**: This code was developed with the assistance of AI (Claude by Anthropic, and Gemini by Google). The architecture, code, tests, and documentation were produced collaboratively between a human developer and an AI assistant. All code has been reviewed by the author.
>Specifically:
> - I architected, reviewed, hand optimized.
> - Claude created boilerplate code that was subsequentyl hand optimized, ran regression tests,bugfixed and helped with documentation.
> - Gemini created code fragments that were usually wrong but inspired `code golf` techniques.  
>
>To be frank, without these agents this work would not have been possible.  

Here we have two Tiny BASICs for the 8088 as inspired by Steve Ciarcia's article in BYTE magazine.
  * **Tokenzied 16bit signed Tiny BASIC - FAST** - includes `GOSUB`/`RETURN`, `FOR`/`NEXT`,`AND`/`OR`/`XOR`/`NOT`/`HEX$` and bitbang serial in 2kbytes of ROM
  * **Mini-BASIC 32bit float BASIC - TRIG/Transcendental** - includes `SIN`/`COS`/`TAN`/`ASIN`/`ACOS`/`ATAN`, and `LN`/`EXP`/`LOG`/`SQRT`,`^(Power)` and bitbang serial in 4kbytes of ROM

You can play with these online at the link below - both versions include a showcase BASIC demo - type `RUN` to execute, and `LIST` to view.
[https://vincbr900.github.io/8086-Tiny-BASIC/](https://vincbr900.github.io/8086-Tiny-BASIC/)

> If you've found these Tiny BASIC interpreters useful for learning, retrocomputing, or your own projects, you can buy me a coffee.  Donations are entirely optional but greatly appreciated.
> [!["Buy Me A Coffee"](https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png)](https://www.buymeacoffee.com/vpcrabtreeZ)

Credit to [Oscar Toledo's bootBASIC](https://github.com/VinCBR900/bootBASIC), which is leveraged here.   
Credit to **XTulator: A portable, open-source 80186 PC emulator, Copyright (C)2020 Mike Chambers**, for the CPU core. See [https://github.com/mikechambers84/XTulator](https://github.com/mikechambers84/XTulator)

## uBASIC 8088 - 2kbyte 16bit signed Int Tiny BASIC for Toy 8086/88 Embedded systems

Target: <=2048 bytes code ROM, 2048/4096 bytes RAM.

Copyright (c) 2026 Vincent Crabtree, MIT License

### Functionality
**Statements**: 
  - `END`, `FOR`..`TO`..`[STEP]` `NEXT`, `GOTO`, `GOSUB` `RETURN`, `IF` .. `THEN`, `INPUT`, `LET`, `OUT`, `POKE`, `PRINT [TAB(spaces)] [;] [CHR$(n)]`, `REM`
  - `FREE`, `HELP`, `LIST [start,end]`, `NEW`, `RUN`   

**Expressions**:  
  - Arithmetic: `+` `-` `*` `/` `%` (Mod)  `^` (Power)
  - Relational `<` `>` `<=` `>=` `<>`, unary`-` and `= (assignment)` 
  - Bitwise: `&` (and) `|` (or) `NOT(val)` (not)
  - functions: `PEEK(addr)`, `USR(addr)`, `IN(io)`, `ABS(val)`, `RND(limit)` 
  - variables `A`..`Z`

**Numbers**: signed 16-bit (`-32768..32767`)

**Errors**: `?0` syntax, `?1` undef line, `?2` div/zero, `?3` out of memory, `?4` bad variable, `?5` `RETURN` without `GOSUB`, `?6` `NEXT` without `FOR`, `?B` break into program (ROM version)

**Notes**
  * `^` Power supports negative base but not negative Exponent
  * Multi-Statement - colon separator `:` **Not Supported**

## To-Do - MiniBASIC8088 - 4kbyte 32bit Float Tiny BASIC for Toy 8086/88 Embedded systems
See the file header for now. 

## License

Copyright (c) 2026 Vincent Crabtree

**MIT License**
