# Fire Bird V ATmega2560 bridge firmware.
#   make            build firebird.hex
#   make flash      upload over the robot's USB port through the NEX bootloader
#                   (PROGRAMMER/PORT are guesses until Phase 0 confirms the bootloader protocol)
#   make test       host-side protocol check (needs gcc)
MCU = atmega2560
F_CPU = 14745600UL
PORT ?= /dev/firebird
PROGRAMMER ?= avr109
BAUD ?= 115200

CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu11 -Wall -Wextra -ffunction-sections -fdata-sections
SRC = main.c protocol.c

firebird.hex: firebird.elf
	avr-objcopy -O ihex -R .eeprom $< $@
	avr-size --mcu=$(MCU) -C $<

firebird.elf: $(SRC) protocol.h
	avr-gcc $(CFLAGS) -Wl,--gc-sections -o $@ $(SRC)

flash: firebird.hex
	avrdude -p m2560 -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$<:i

test:
	gcc -std=c11 -Wall -Wextra -o test_protocol test_protocol.c protocol.c && ./test_protocol

clean:
	rm -f firebird.elf firebird.hex test_protocol

.PHONY: flash test clean
