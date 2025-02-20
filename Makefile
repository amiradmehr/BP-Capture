# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Wextra -std=c99 -O2

# Target executable name
TARGET = x

# Source file
SRC = x.c

# Default target
all: $(TARGET)

# Rule to build the target executable
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET).exe $(SRC)

# Rule to run the program
run: $(TARGET)
	$(TARGET).exe

# Rule to clean up (remove the executable)
clean:
	del $(TARGET).exe

# Phony targets (not actual files)
.PHONY: all run clean