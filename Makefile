# Makefile for ISA Flight Software Test Harness
# Targets: make, make run, make clean

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -O0 -g -Isrc -Iinclude
LDFLAGS = -lm -g

# Directories
SRC_DIR = src
BUILD_DIR = build
BIN_DIR = .

# Target executable
TARGET = $(BIN_DIR)/fswtest.exe

# Source files
SOURCES = fswtest.c \
          $(SRC_DIR)/major.c \
          $(SRC_DIR)/minor.c \
          $(SRC_DIR)/math_utils.c

# Object files (in build directory)
OBJECTS = $(BUILD_DIR)/fswtest.o \
          $(BUILD_DIR)/major.o \
          $(BUILD_DIR)/minor.o \
          $(BUILD_DIR)/math_utils.o

# Default target: build the program
all: $(TARGET)

# Link the executable
$(TARGET): $(OBJECTS)
	@echo "Linking $(TARGET)..."
	$(CC) $(OBJECTS) -o $(TARGET) $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

# Compile fswtest.c
$(BUILD_DIR)/fswtest.o: fswtest.c Makefile | $(BUILD_DIR)
	@echo "Compiling fswtest.c..."
	$(CC) $(CFLAGS) -c fswtest.c -o $(BUILD_DIR)/fswtest.o

# Compile source files from src directory
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c Makefile | $(BUILD_DIR)
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) -c $< -o $@

# Create build directory
$(BUILD_DIR):
	@echo "Creating build directory..."
	@mkdir $(BUILD_DIR)

# Run the test harness
run: $(TARGET)
	@echo "Running flight software test harness..."
	@$(TARGET)

# Clean build artifacts
clean:
	@echo "Cleaning build directory..."
	@cmd /c "if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)"
	@cmd /c "if exist $(TARGET) del /q $(TARGET)"
	@cmd /c "if exist fsw_output.csv del /q fsw_output.csv"
	@echo "Clean complete"

# Phony targets
.PHONY: all run clean

# Help target
help:
	@echo "ISA Flight Software Test Harness - Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  make       - Build the test harness (default)"
	@echo "  make run   - Build and run the test harness"
	@echo "  make clean - Remove build directory and executable"
	@echo "  make help  - Show this help message"
	@echo ""
	@echo "Requirements:"
	@echo "  - GCC compiler (MinGW-w64 on Windows)"
	@echo "  - sensordata.csv in current directory"
