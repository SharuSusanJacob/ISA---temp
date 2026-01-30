# ISA Digital Autopilot (DAP) Makefile
# Enhanced with dual build support: XC32 for SAM verification, GCC for host testing

# Build type selection
ifdef SAM_BUILD
    CC = xc32-gcc
    CFLAGS = -Wall -Wextra -pedantic -std=c99 -O2 -g -mprocessor=32MX470F512H
    LDFLAGS = -lm
    BUILD_SUFFIX = _sam
    TARGET_DESC = SAM
else
    CC = gcc
    CFLAGS = -Wall -Wextra -pedantic -std=c99 -O2 -g
    LDFLAGS = -lm
    BUILD_SUFFIX = _dap
    TARGET_DESC = Host
endif

# Post-build action: run on host when not building for SAM
ifdef SAM_BUILD
RUN_AFTER_BUILD :=
else
RUN_AFTER_BUILD := $(MAKE) -s run
endif

# Directories (relative to this Makefile at test/dap)
ROOT_DIR = ../..
SRC_DIR = $(ROOT_DIR)/src
INCLUDE_DIR = $(ROOT_DIR)/include
TEST_DIR = .
BUILD_DIR = build$(BUILD_SUFFIX)

# Source files
MATH_SRC = $(wildcard $(SRC_DIR)/math/*.c)
# All C files in this dir except the test main will be treated as DAP module sources
DAP_SRC = $(filter-out $(TEST_DIR)/test_DAP.c,$(wildcard $(TEST_DIR)/*.c))
MAIN_SRC = $(TEST_DIR)/test_DAP.c

# Object files
MATH_OBJ = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(MATH_SRC))
DAP_OBJ = $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)/dap/%.o,$(DAP_SRC))
MAIN_OBJ = $(BUILD_DIR)/test_DAP.o

# All object files
OBJS = $(MATH_OBJ) $(DAP_OBJ) $(MAIN_OBJ)

# Target executable
TARGET = $(BUILD_DIR)/dap_test

# Default target - compile; also execute on host when using GCC
all: directories $(TARGET)
	@echo "✓ $(TARGET_DESC) build completed successfully"
	$(RUN_AFTER_BUILD)

# Check XC32 availability
check-xc32:
	@echo "Checking for XC32 compiler..."
	@if command -v xc32-gcc >/dev/null 2>&1; then \
		echo "✓ xc32-gcc found: $$(which xc32-gcc)"; \
		echo "✓ Version: $$(xc32-gcc --version | head -1)"; \
	else \
		echo "❌ xc32-gcc not found in PATH"; \
		echo "Add to PATH: export PATH=\"/opt/microchip/xc32/v4.60/bin:\$$PATH\""; \
		echo "Or run: echo 'export PATH=\"/opt/microchip/xc32/v4.60/bin:\$$PATH\"' >> ~/.bashrc"; \
		echo "Then: source ~/.bashrc"; \
		exit 1; \
	fi

# Direct SAM build target (XC32)
sam: check-xc32
	@echo "Building for SAM..."
	$(MAKE) SAM_BUILD=1 all

# Host build is default: use `make`

# Create build directories
directories:
	@mkdir -p $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/math
	@mkdir -p $(BUILD_DIR)/dap

# Link
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile test main
$(BUILD_DIR)/test_DAP.o: $(MAIN_SRC)
	$(CC) $(CFLAGS) -I$(ROOT_DIR) -I$(INCLUDE_DIR) -I$(TEST_DIR) -c -o $@ $<

# Compile math source files
$(BUILD_DIR)/math/%.o: $(SRC_DIR)/math/%.c
	$(CC) $(CFLAGS) -I$(INCLUDE_DIR) -c -o $@ $<

# Compile DAP source files (in this test directory)
$(BUILD_DIR)/dap/%.o: $(TEST_DIR)/%.c
	$(CC) $(CFLAGS) -I$(ROOT_DIR) -I$(INCLUDE_DIR) -I$(TEST_DIR) -c -o $@ $<

# Clean build files
clean:
	rm -rf build_host build_sam build_dap

# Run the program
run: $(TARGET)
ifdef SAM_BUILD
	@echo "Cannot run SAM binary on host system."
	@echo "✓ SAM binary created successfully at $(TARGET)"
	@echo "Use MPLAB X SIM or actual SAM hardware to execute."
else
	./$(TARGET)
endif

# Convenience targets
verify-xc32: sam

# Quick XC32 setup helper
setup-xc32:
	@echo "Setting up XC32 PATH for current session..."
	@echo "Run these commands:"
	@echo "  export PATH=\"/opt/microchip/xc32/v4.60/bin:\$$PATH\""
	@echo "  echo 'export PATH=\"/opt/microchip/xc32/v4.60/bin:\$$PATH\"' >> ~/.bashrc"
	@echo "  source ~/.bashrc"
	@echo "Then run: make check-xc32"

# Show available targets
help:
	@echo "Available targets:"
	@echo "  all          - Build for host (default)"
	@echo "  sam          - Build for SAM using XC32"
	@echo "  verify-xc32  - Alias for sam"
	@echo "  check-xc32   - Check if XC32 compiler is available"
	@echo "  setup-xc32   - Show XC32 PATH setup instructions"
	@echo "  clean        - Remove all build files"
	@echo "  help         - Show this help"

.PHONY: all clean run directories sam verify-xc32 help check-xc32 setup-xc32
