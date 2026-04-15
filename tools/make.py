boot=open('boot.bin','rb').read()
basic=open('uBASIC8088_v1.0.9.bin','rb').read()
open('floppy.img','wb').write(boot+basic+bytes(2560-len(basic)))
