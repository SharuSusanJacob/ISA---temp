CC=gcc
CFLAGS=-Wall -Wextra -std=c99 -O2

BUILD_DIR=build
TARGET=$(BUILD_DIR)/guidance_test
SRC=guidance.c math_utils.c coordinate_transforms.c vector3.c guidance_test.c
OBJ=$(SRC:%.c=$(BUILD_DIR)/%.o)

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJ) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ -lm

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)

run: $(TARGET)
	./$(TARGET)

