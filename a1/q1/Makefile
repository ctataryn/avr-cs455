# default target when "make" is run w/o arguments
all: cs455-a1q1.rom

cs455-a1q1.o: cs455-a1q1.c
	avr-gcc -c -g -O3 -Wall -mmcu=atmega16 -I/usr/local/avr/include -I. cs455-a1q1.c -o cs455-a1q1.o

cs455-a1q1.elf: cs455-a1q1.o
	avr-gcc cs455-a1q1.o -Wl,-Map=cs455-a1q1.map,--cref -mmcu=atmega16 -o cs455-a1q1.elf

# copy ROM (FLASH) object
cs455-a1q1.rom: cs455-a1q1.elf
	avr-objcopy -O ihex cs455-a1q1.elf cs455-a1q1.rom

# command to clean up junk (no source files) (invoked by "make clean")
clean:
	rm -f *.o *.rom *.elf *.map *~
