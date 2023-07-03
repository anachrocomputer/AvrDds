# AvrDds

Some simple AVR programs to explore Direct Digital Synthesis.

The programs are in C and may be compiled with 'avr-gcc' on Linux.
They generate ELF output files which are suitable to program into
AVR chips for testing purposes.

The 'Makefile' also has targets for the AVR programming tool 'avrdude'.

## Chips Supported

At present, there's only support for the ATtiny1616 and the ATmega4809.
The main reason for this choice of chips is that I have dev boards
for those chips that I can use for testing.
Also, the ATtiny1616 has a built-in 8-bit DAC whereas the ATmega4809 must
be connected to an external SPI DAC (MCP4822).

## AVR Toolchain

The programs have been compiled, linked and tested using a Linux version
of the 'avr-gcc' toolchain.
This can be installed directly or as part of the Arduino IDE.

The compiler, linker and programmers are invoked from the Makefile in
the usual way.
Various parameters in the Makefile may be altered to suit the development
setup, e.g. the type of programmers used and the ports that they connect to.
The full pathname to the toolchain is also configured in the Makefile.

Special targets in the Makefile are provided to invoke the programming
device(s) and write the ELF files into the Flash memory in the chips.
These targets are called 'prog1616' and 'prog4809'.
The 'prog1616' and 'prog4809' targets invoke the UPDI programmer.

There's a Makefile target called 'clean' that deletes the object code files
and the ELF binary files.
It leaves the source code files untouched, of course.

## AVR Programmers

I have tested the code with a 'jtag2updi' implemented on an ATmega328P.

## Test Setup

Blinking LEDs, of course!
The LED should blink at 1Hz (500ms on, 500ms off).
This frequency may be measured as a means of verifying correct
clocking of the AVR chip.

On the ATtiny1616, a 500Hz square wave should be generated on pin PC1.
There's also a scope sync or trigger pulse on PC0 that is in phase with the
signal generated by the DDS.
The chip also generates a timing test pulse on PC2
(HIGH during the DDS ISR, LOW otherwise).
PC3 is the 1Hz LED.

PORTC is used for these GPIO digital pulses because that port cannot be used
for analog inputs on ADC0.
Analog signals are read via ADC0 on AIN1 and AIN4.
More ADC inputs to be added later.

On the ATmega4809, a 500Hz square wave should be generated on pin PA4.

On the ATtiny1616 and ATmega4809,
the serial port(s) should transmit a message at 9600 baud.
The ATmega4809 also produces a different message on each of its four
UARTs, but with a DIP-40 chip UART3 is inaccessible (only the 48-pin
surface-mount versions can use all four UARTs).

On the ATtiny1616 and ATmega4809,
serial input is accepted on UART0.
All the chips accept a letter 'r' to print the reset reason.
The ATtiny1616 and ATmega4809 also accept 'i' to print the chip ID
bytes, 'n' to print the unique serial number and 'f' to print the
values of the fuse registers.

To select different waveforms, the chips accept 's' for a sinewave,
'q' for a squarewave, 't' for a triangle wave, and 'w' for a sawtooth.
This may change to become voltage-controlled in a future version.

## Future Enhancements

* Implement external SPI DAC on ATmega4809
* Make waveform selection voltage-controlled
* Add more waveforms
* Store waveforms in Flash memory
* Add more control voltage (CV) analog inputs
* Test with other AVR chips (e.g. other 1-series ATtiny chips)
* Test with 48-pin SMD ATmega4809
* Add support for 2-series AVRs (e.g. ATtiny1626): two UARTs, 12-bit ADC, no DAC
* Configure and test the watchdog timer

