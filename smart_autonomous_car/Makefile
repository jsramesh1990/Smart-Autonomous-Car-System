# Smart Autonomous Car - Makefile

# Project Configuration
PROJECT = smart_autonomous_car
MCU = atmega328p
F_CPU = 16000000UL
BAUD = 9600
PROGRAMMER = arduino
PORT = /dev/ttyUSB0

# Compiler Settings
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
AVRDUDE = avrdude

# Compiler Flags
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -Os -Wall
CFLAGS += -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS = -mmcu=$(MCU) -Wl,-Map=$(PROJECT).map -Wl,--gc-sections

# Include Directories
INCLUDES = -Isrc -Isrc/peripherals -Isrc/logic -Isrc/utils

# Source Files
SRC = src/main.c \
      src/peripherals/led_controller.c \
      src/peripherals/display_manager.c \
      src/peripherals/motor_driver.c \
      src/peripherals/sensor_suite.c \
      src/logic/line_follower.c \
      src/logic/wall_follower.c \
      src/logic/obstacle_avoider.c \
      src/logic/state_machine.c \
      src/utils/timer.c \
      src/utils/debug.c

# Object Files
OBJ = $(SRC:.c=.o)

# Default Target
all: $(PROJECT).elf $(PROJECT).hex size

# Build ELF
$(PROJECT).elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^

# Build HEX
$(PROJECT).hex: $(PROJECT).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Compile C files
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Size information
size: $(PROJECT).elf
	@echo "Memory Usage:"
	@$(SIZE) -C --mcu=$(MCU) $(PROJECT).elf

# Flash to microcontroller
flash: $(PROJECT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b 115200 -U flash:w:$(PROJECT).hex

# Generate disassembly
disasm: $(PROJECT).elf
	$(OBJDUMP) -d $< > $(PROJECT).dis

# Clean build files
clean:
	rm -f $(OBJ) $(PROJECT).elf $(PROJECT).hex $(PROJECT).map $(PROJECT).dis

# Test build
test: all
	@echo "Build successful!"

# Help
help:
	@echo "Available targets:"
	@echo "  all     - Build project (default)"
	@echo "  flash   - Flash to microcontroller"
	@echo "  size    - Show memory usage"
	@echo "  disasm  - Generate disassembly"
	@echo "  clean   - Remove build files"
	@echo "  test    - Test build"

.PHONY: all flash size disasm clean test help
