#  Bicycle Bell/Horn firmware
#  Copyright (C) 2015  Stuart Longland
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

MCU=atmega32u4
PTYPE=stk500v2
PDEV=/dev/ttyACM0
.PHONY: %.pgm all

all: bell.pgm

%.pgm: %.hex
	avrdude -p $(MCU) -c $(PTYPE) -P $(PDEV) -e \
		-U flash:w:$^

%.elf: %.o
	avr-gcc -mmcu=$(MCU) -o $@ $^

%.o: %.c
	avr-gcc -mmcu=$(MCU) -Os -o $@ -c $<

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

bellsnd.c bellsnd.h: ../audio/bell.py
	cd ../audio ; python bell.py
	cp ../audio/bellsnd.[ch] .
hornsnd.c hornsnd.h: ../audio/horn.py
	cd ../audio ; python horn.py
	cp ../audio/hornsnd.[ch] .

bell.elf: bell.o bellsnd.o hornsnd.o
bell.o: bellsnd.h hornsnd.h
