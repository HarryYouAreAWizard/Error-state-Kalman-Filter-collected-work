# yes, chat made this

# --- Compiler Configuration ---
CC       = gcc
CFLAGS   = -O3 -Wall -Wextra -Wpedantic -std=c11 -fPIC

TARGET   = eskf_filter
SHARED_LIB = libeskf.so

# --- Directory Paths ---
THIRDPARTY_DIR = third_party
CMSIS_DSP_DIR  = $(THIRDPARTY_DIR)/CMSIS-DSP
CMSIS_5_DIR    = $(THIRDPARTY_DIR)/CMSIS_5

# --- Include Paths ---
INCLUDES = \
    -I$(CMSIS_DSP_DIR)/Include \
    -I$(CMSIS_DSP_DIR)/PrivateInclude \
    -I$(CMSIS_5_DIR)/CMSIS/Core/Include \
	-Iinclude

# --- Host Emulation Platform Flags ---
PLATFORM_FLAGS = -D__GNUC_PYTHON__

# Combine all compilation flags
ALL_CFLAGS   = $(CFLAGS) $(INCLUDES) $(PLATFORM_FLAGS)

# --- Source Files (CMSIS-DSP Bundle Files Only) ---
CMSIS_DSP_SRCS = \
    $(CMSIS_DSP_DIR)/Source/BasicMathFunctions/BasicMathFunctions.c \
    $(CMSIS_DSP_DIR)/Source/BayesFunctions/BayesFunctions.c \
    $(CMSIS_DSP_DIR)/Source/CommonTables/CommonTables.c \
    $(CMSIS_DSP_DIR)/Source/ComplexMathFunctions/ComplexMathFunctions.c \
    $(CMSIS_DSP_DIR)/Source/ControllerFunctions/ControllerFunctions.c \
    $(CMSIS_DSP_DIR)/Source/DistanceFunctions/DistanceFunctions.c \
    $(CMSIS_DSP_DIR)/Source/FastMathFunctions/FastMathFunctions.c \
    $(CMSIS_DSP_DIR)/Source/FilteringFunctions/FilteringFunctions.c \
    $(CMSIS_DSP_DIR)/Source/InterpolationFunctions/InterpolationFunctions.c \
    $(CMSIS_DSP_DIR)/Source/MatrixFunctions/MatrixFunctions.c \
    $(CMSIS_DSP_DIR)/Source/QuaternionMathFunctions/QuaternionMathFunctions.c \
    $(CMSIS_DSP_DIR)/Source/StatisticsFunctions/StatisticsFunctions.c \
    $(CMSIS_DSP_DIR)/Source/SupportFunctions/SupportFunctions.c \
    $(CMSIS_DSP_DIR)/Source/TransformFunctions/TransformFunctions.c \
    $(CMSIS_DSP_DIR)/Source/WindowFunctions/WindowFunctions.c

# --- Kalman Filter Source Files ---
PROJECT_SRCS = $(wildcard src/*.c)

# --- Object File Transformations ---
OBJS = $(CMSIS_DSP_SRCS:.c=.o) $(PROJECT_SRCS:.c=.o)

# --- Build Rules ---
.PHONY: all shared clean

# Default builds both the standalone executable and the shared library
all: $(TARGET) $(SHARED_LIB)

# Target for standard standalone application
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) -lm

# Target for Python shared object library
shared: $(SHARED_LIB)

$(SHARED_LIB): $(OBJS)
	$(CC) -shared $(OBJS) -o $(SHARED_LIB) -lm

# Rule for compiling all C source files
%.o: %.c
	$(CC) $(ALL_CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET) $(SHARED_LIB)
