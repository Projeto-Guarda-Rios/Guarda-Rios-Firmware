CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

CFLAGS  = -mcpu=cortex-m0plus -mthumb -Os -Wall \
          -ffunction-sections -fdata-sections -std=c99
LDFLAGS = -T stm32l053r8.ld -Wl,--gc-sections -nostdlib -lgcc

all: firmware.bin

firmware.elf: main.c stm32l053r8.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ main.c
	$(SIZE) $@

firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $< $@
	@echo "Built firmware.bin ($$(wc -c < $@) bytes)"

clean:
	rm -f firmware.elf firmware.bin

.PHONY: all clean
