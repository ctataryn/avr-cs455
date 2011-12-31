# default target when "make" is run w/o arguments
all: cs455-a1q3.rom

cs455-a1q3.o: cs455-a1q3.c
	avr-gcc -c -g -O3 -Wall -mmcu=atmega16 -I/usr/local/avr/include -I. cs455-a1q3.c -o cs455-a1q3.o

cs455-a1q3.elf: cs455-a1q3.o
	avr-gcc cs455-a1q3.o -Wl,-Map=cs455-a1q3.map,--cref -mmcu=atmega16 -o cs455-a1q3.elf

# copy ROM (FLASH) object
cs455-a1q3.rom: cs455-a1q3.elf
	avr-objcopy -O ihex cs455-a1q3.elf cs455-a1q3.rom

# command to clean up junk (no source files) (invoked by "make clean")
clean:
	rm -f *.o *.rom *.elf *.map *~
