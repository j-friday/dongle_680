.\output\BinConvert  -oad .\output\boot\bk3633_boot.bin  .\output\stack\bk3633_stack.bin  .\output\app\bk3633_app_slave.bin -m 0x1F00 -l 0x2Db00 -v 0x0002 -rom_v 0x0002 -e 00000000 00000000 00000000 00000000

del .\output\app\bk3633_app_slave.bin
del .\output\app\*.out
