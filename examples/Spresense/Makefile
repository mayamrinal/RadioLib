# Makefile for Spresense NuttX RadioLib Example
# This Makefile is designed to work with NuttX build system

# Compiler and flags
CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CXXFLAGS = $(CFLAGS) -std=c++11 -fno-rtti -fno-exceptions
INCLUDES = -I$(NUTTX_DIR)/include -I$(NUTTX_DIR)/arch/arm/include -I$(RADIOLIB_DIR)/src

# Directories
NUTTX_DIR ?= /path/to/nuttx
RADIOLIB_DIR ?= /path/to/RadioLib
BUILD_DIR = build

# Source files
SOURCES = SpresenseNuttxExample.cpp \
          $(RADIOLIB_DIR)/src/hal/Spresense/SpresenseNuttxHal.cpp \
          $(RADIOLIB_DIR)/src/Module.cpp \
          $(RADIOLIB_DIR)/src/protocols/PhysicalLayer.cpp \
          $(RADIOLIB_DIR)/src/modules/SX1278/SX1278.cpp \
          $(RADIOLIB_DIR)/src/modules/SX1278/SX1278_Registers.cpp

# Object files
OBJECTS = $(SOURCES:%.cpp=$(BUILD_DIR)/%.o)

# Target
TARGET = spresense_radiolib_example

# Default target
all: $(BUILD_DIR)/$(TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)/src/hal/Spresense
	mkdir -p $(BUILD_DIR)/src/modules/SX1278

# Compile C++ files
$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/src/%.o: $(RADIOLIB_DIR)/src/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -DRADIOLIB_BUILD_SPRESENSE -c $< -o $@

# Link
$(BUILD_DIR)/$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -L$(NUTTX_DIR)/libs -lnuttx

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Install to NuttX
install: $(BUILD_DIR)/$(TARGET)
	cp $(BUILD_DIR)/$(TARGET) $(NUTTX_DIR)/bin/

# Help
help:
	@echo "Available targets:"
	@echo "  all     - Build the example"
	@echo "  clean   - Remove build files"
	@echo "  install - Install to NuttX"
	@echo "  help    - Show this help"
	@echo ""
	@echo "Environment variables:"
	@echo "  NUTTX_DIR     - Path to NuttX source (default: /path/to/nuttx)"
	@echo "  RADIOLIB_DIR  - Path to RadioLib source (default: /path/to/RadioLib)"

.PHONY: all clean install help 