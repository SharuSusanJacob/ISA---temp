# Makefile for Navigation Module
# ISA Flight Software Project

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -pedantic -std=c99 -g -O2
INCLUDES = -I../guidance

# Source files
SOURCES = navigation.c navigation_test.c ../guidance/vector3.c  ../guidance/math_utils.c
OBJECTS = $(SOURCES:.c=.o)
TARGET = navigation_test

# Default target
all: $(TARGET)

# Linking
$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

# Compilation
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJECTS) $(TARGET) states.csv navout.csv

# Run tests
test: $(TARGET)
	./$(TARGET)

.PHONY: all clean test