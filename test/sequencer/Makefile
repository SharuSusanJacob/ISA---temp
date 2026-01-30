CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -I.

# Target executable name
TARGET = sequencer_sim.exe

# Source files
SRCS = sequencer.c sequencer_test.c

# Default target to build the executable
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS)

# Rule to run the simulation
run: $(TARGET)
	.\$(TARGET)

# Clean up build artifacts
clean:
	cmd /C del $(TARGET) sequencer_sim_output.csv
