..\Driver_Build_Tools\NASM\nasm.exe -f bin -o bootrom.com -DAS_COM .\bootrom.asm
..\Driver_Build_Tools\NASM\nasm.exe -f bin -o bootrom .\bootrom.asm & python checksum.py
