# Makefile for simple AVR programming

# We'll pick up the GCC toolchain from the Arduino installation
ARDUINO=/home/john/Arduino/arduino-1.8.10

MCU45=attiny45
MCU20=attiny20
MCU23=attiny2313
MCU1616=attiny1616
MCU328=atmega328p
MCU1284=atmega1284p
MCU4809=atmega4809

CC=$(ARDUINO)/hardware/tools/avr/bin/avr-gcc
LD=$(ARDUINO)/hardware/tools/avr/bin/avr-gcc
OC=$(ARDUINO)/hardware/tools/avr/bin/avr-objcopy
SZ=$(ARDUINO)/hardware/tools/avr/bin/avr-size
DUDE=$(ARDUINO)/hardware/tools/avr/bin/avrdude

CFLAGS=-c -o $@ -O3
LDFLAGS=-o $@
OCFLAGS=-j .text -j .data -O ihex
SZFLAGS=-B -d
DUDEFLAGS=-C $(ARDUINO)/hardware/tools/avr/etc/avrdude.conf

# Programming port and programming device. Can be overridden from the
# command line, e.g. make ISPPORT=/dev/ttyUSB0 prog45
ISPPORT=/dev/ttyS4
ISPDEV=avrispv2
TPIPORT=USB
TPIDEV=usbasp
UPDIPORT=/dev/ttyUSB0
UPDIDEV=jtag2updi

OBJS=t1616.o t4809.o
ELFS=$(OBJS:.o=.elf)

# Default target will compile and link all C sources, but not program anything
all: $(ELFS)
.PHONY: all

t1616.elf: t1616.o
	$(LD) -mmcu=$(MCU1616) $(LDFLAGS) t1616.o
	$(SZ) --mcu=$(MCU1616) $(SZFLAGS) t1616.elf

t1616.o: t1616.c
	$(CC) -mmcu=$(MCU1616) $(CFLAGS) t1616.c

t4809.elf: t4809.o
	$(LD) -mmcu=$(MCU4809) $(LDFLAGS) t4809.o
	$(SZ) --mcu=$(MCU4809) $(SZFLAGS) t4809.elf

t4809.o: t1616.c
	$(CC) -mmcu=$(MCU4809) $(CFLAGS) t1616.c

# Targets to invoke the programmer and program the flash memory of the MCU
prog1616: t1616.elf
	$(DUDE) $(DUDEFLAGS) -c $(UPDIDEV) -P $(UPDIPORT) -p $(MCU1616) -e -U flash:w:t1616.elf:e

prog4809: t4809.elf
	$(DUDE) $(DUDEFLAGS) -c $(UPDIDEV) -P $(UPDIPORT) -p $(MCU4809) -e -U flash:w:t4809.elf:e

.PHONY: prog1616 prog4809

testtpi:
	$(DUDE) $(DUDEFLAGS) -c $(TPIDEV) -P $(TPIPORT) -p $(MCU20)

testupdi:
	$(DUDE) $(DUDEFLAGS) -c $(UPDIDEV) -P $(UPDIPORT) -p $(MCU1616)

.PHONY: testtpi testupdi

# Target 'clean' will delete all object files and ELF files
clean:
	-rm -f $(OBJS) $(ELFS)

.PHONY: clean

# USBasp upgrade using real Atmel AVRISP on /dev/ttyS4:
# /home/john/Arduino/arduino-1.8.10/hardware/tools/avr/bin/avrdude -C /home/john/Arduino/arduino-1.8.10/hardware/tools/avr/etc/avrdude.conf -c avrispv2 -P /dev/ttyS4 -p atmega8 -e -U flash:w:usbasp.atmega8.2011-05-28.hex:i
