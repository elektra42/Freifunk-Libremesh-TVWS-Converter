
avrdude -p atmega8 -P /dev/ttyUSB1 -c stk500v2   -U flash:w:main.hex