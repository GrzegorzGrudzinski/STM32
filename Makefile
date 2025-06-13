# Kompilator i narzędzia
CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

# Flagi kompilatora
CFLAGS = -mcpu=cortex-m4 -mthumb -Wall -O0 -g -ffreestanding -nostdlib -I.
CFLAGS += -Idrivers/Simple_RTOS
CFLAGS += -Idrivers/Gyro_I3G4250D_drivers
CFLAGS += -Idrivers/stm32f411xx_periph_drivers/Inc
CFLAGS += -Idrivers/stm32f411xx_periph_drivers/Inc/registers_bits

# Ścieżki do źródeł
SRCS := $(wildcard drivers/Simple_RTOS/*.c) \
        $(wildcard drivers/Gyro_I3G4250D_drivers/*.c) \
        $(wildcard drivers/stm32f411xx_periph_drivers/Src/*.c) \
        $(wildcard examples/*.c)

# Pliki obiektowe
OBJS := $(SRCS:.c=.o)

# Nazwa końcowa
TARGET = build/output.elf

# Domyślny cel
all: build $(TARGET)

# Linkowanie
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

# Kompilacja każdego .c do .o
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Katalog na build
build:
	mkdir -p build

# Czyszczenie
clean:
	rm -rf build
	find . -name "*.o" -delete

.PHONY: all clean build
