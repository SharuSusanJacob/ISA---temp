# ISA Flight Software - Guidance Module Makefile
# MISRA C Compliant Implementation

# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -pedantic -std=c99 -O2
LDFLAGS = -lm

# Directories
BUILD_DIR = guidance_build

# Source files
SRCS = vector3.c \
	   quaternion.c \
       math_utils.c \
       coordinate_transforms.c \
       guidance.c \
       guidance_test.c

# Object files (with build directory prefix)
OBJS = $(addprefix $(BUILD_DIR)/, $(SRCS:.c=.o))

# Target executable
TARGET = $(BUILD_DIR)/guidance_test

# Default target
all: $(BUILD_DIR) $(TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Link object files to create executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile source files to object files
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build artifacts
clean:
	rm -rf $(BUILD_DIR)

# Run the test
run: $(TARGET)
	$(TARGET)

# Phony targets
.PHONY: all clean run