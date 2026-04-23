# Project Settings
TARGET = main.ihx
MCU    = stm8
DEVICE = STM8S005

# Paths
LIB_DIR     = lib/STM8S_StdPeriph_Driver
LIB_INC_DIR = $(LIB_DIR)/inc
LIB_SRC_DIR = $(LIB_DIR)/src
PROJECT_INC = inc
PROJECT_SRC = src

# Compiler and Flags
CC      = sdcc-sdcc
# -I. includes current dir, -I./inc includes project headers, -I$(LIB_INC_DIR) includes library headers
CFLAGS  = -m$(MCU) -DSDCC -D$(DEVICE) -I. -I./$(PROJECT_INC) -I$(LIB_INC_DIR)

# # Search paths for source files
# # This tells make where to look for .c files automatically
VPATH = $(PROJECT_SRC):$(LIB_SRC_DIR)

# Core Source Files
# List the .c files you need. 
# Note: We don't need the full path here because of VPATH
SOURCES = main.c \
          pwm.c \
          mpu.c \
          gpio.c \
          utils.c \
          serial.c \
          stm8s_gpio.c \
          stm8s_clk.c \
          stm8s_uart2.c \
          stm8s_tim1.c \
          stm8s_tim2.c \
          stm8s_i2c.c

# Generate list of object files (.rel)
# This strips the directory and changes .c to .rel
OBJECTS = $(notdir $(SOURCES:.c=.rel))

# Default Rule
all: upload

# Link everything
$(TARGET): $(OBJECTS)
	$(CC) -m$(MCU) $(OBJECTS) -o $(TARGET)

# General rule to compile ANY .rel from a .c file
# This handles main.c, custom_lib/*.c, and the StdPeriph/*.c
%.rel: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean Up
.PHONY: clean
clean:
	rm -f *.rel *.asm *.lst *.rst *.sym *.cdb *.map *.lk

# Upload to device
.PHONY: upload
upload: $(TARGET)
	stm8flash -c stlinkv2 -p stm8s005k6 -w $(TARGET)
