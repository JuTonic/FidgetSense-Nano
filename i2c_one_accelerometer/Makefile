TARGET = main
MCU = atmega328p
F_CPU = 16000000
PORT = /dev/cu.usbserial-110
BAUD = 115200
PROGRAMMER = arduino

CC = avr-gcc
CXX = avr-g++
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

BUILD_DIR = build

CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU)UL -Os -Wall
CXXFLAGS = $(CFLAGS)
LDFLAGS = -mmcu=$(MCU) -lm

SRC = $(TARGET).cpp

OBJ = $(addprefix $(BUILD_DIR)/, $(SRC:.cpp=.o))

ELF = $(BUILD_DIR)/$(TARGET).elf
HEX = $(BUILD_DIR)/$(TARGET).hex

all: $(HEX)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(ELF): $(OBJ) | $(BUILD_DIR)
	$(CXX) $(LDFLAGS) -o $@ $^

$(HEX): $(ELF) | $(BUILD_DIR)
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

upload: $(HEX)
	$(AVRDUDE) -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -D -U flash:w:$(HEX)

clean:
	rm -rf $(BUILD_DIR) screenlog.0

find-port:
	@echo "Available serial ports:"
	@ls /dev/cu.* 2>/dev/null || echo "No serial ports found"

.PHONY: all upload clean find-port