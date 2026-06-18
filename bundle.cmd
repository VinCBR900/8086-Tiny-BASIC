@echo off
tar cf bundle.tar *.c *.h Makefile
dir bundle.tar
gzip -9 bundle.tar
base64 bundle.tar.gz > bundle.txt
dir bundle.tar.gz,bundle.txt,*.c,*.h,Makefile
if exist bundle.tar.gz del bundle.tar.gz
if exist bundle.tar del bundle.tar
