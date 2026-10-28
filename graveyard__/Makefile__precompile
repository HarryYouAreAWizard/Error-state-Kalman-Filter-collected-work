


CC = gcc # compiler choice
LD := ld # linker choice

# compiler flags
INCLUDE_DIRS := include \
				third_party/CMSIS-DSP/Include \
				third_party/CMSIS_5/CMSIS/Core/Include

CFLAGS := -pg \
		  $(addprefix -I, $(INCLUDE_DIRS)) \
		  -fPIC

# link to already compiled library
# LDFLAGS := -Lthird_party/CMSIS-DSP/build/Source \
# 		   -lCMSISDSP \
# 		   -lm
LDFLAGS := -Lbuild/cmsisdsp-pc/ -lCMSISDSP -lm


libmain: $(wildcard src/*.c) \
		 $(wildcard include/*.h)
		 $(CC) $(CFLAGS) \
		 $(wildcard src/*.c) \
		 -shared \
		 -o libmain.so \
		 $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)
	rm libmain.so



# # specify compiler and linker
# CC = gcc
# LD := ld # the name of the linker is ld


# # Specify paths to header and library directories
# INCLUDE_DIRS = 	include \
# 				third_party/CMSIS-DSP/Include \
# 				third_party/CMSIS_5/CMSIS/Core/Include

# LIBRARIES = third_party/CMSIS-DSP/build/Source/libCMSISDSP.a

# CFLAGS :=  $(addprefix -I, $(INCLUDE_DIRS)) -Wall -Wpedantic
# LDFLAGS := $(LIBRARIES)


# # where to put the build files
# BUILD_DIR := build

# # specify that the source files are everyting in src that end with .c
# C_SOURCES := $(wildcard src/*.c)


# # the objects to be created in the build directory are one-to-one with the source files
# OBJECTS = 	$(C_SOURCES:%.c=$(BUILD_DIR)/%.o)

# # compile the sourcefiles one at a time and put the objects in the build directory
# $(BUILD_DIR)/%.o: %.c
# 	@mkdir -p $(dir $@)
# 	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

# # the target build. Made by linking the objects and the libraries together? -shared makes it a shared library
# libmain.so:$(OBJECTS)
# 	@mkdir -p $(dir $@)
# 	$(LD) $(OBJECTS) $(LDFLAGS) -shared -o $@ 

# # $@ refers to the target itself 

# clean:
# 	rm -rf $(BUILD_DIR)

# .PHONY: all clean


# legacy compiler flags
# CFLAGS := -pg -Iinclude -Ithird_party/CMSIS-DSP/Include -Ithird_party/CMSIS_5/CMSIS/Core/Include
# LDFLAGS := -Lthird_party/CMSIS-DSP/build/Source -lCMSISDSP -lm


# # build main shared library
# libmain: src/user/ofelas/main.c src/user/ofelas/estimator.c src/user/ofelas/measurement_handler.c src/user/ofelas/ESKF.c src/user/ofelas/quaternions.c src/user/ofelas/linalg.c include/user/ofelas/estimator.h include/user/ofelas/measurement_handling.h include/user/ofelas/ESKF.h include/user/ofelas/quaternions.h include/user/ofelas/linalg.h
# 	$(CC) $(CFLAGS) -fPIC src/user/ofelas/main.c src/user/ofelas/estimator.c src/user/ofelas/measurement_handler.c src/user/ofelas/ESKF.c src/user/ofelas/quaternions.c src/user/ofelas/linalg.c -shared -o libmain.so $(LDFLAGS)





# # ensure cmsis-dsp library is built
# CFLAGS = -pg -Iinclude -Ithird_party/CMSIS-DSP/Include -Ithird_party/CMSIS_5/CMSIS/Core/Include
# LDFLAGS = -pg -Lthird_party/CMSIS-DSP/build/Source -lCMSISDSP -lm

# # build main
# main: src/user/ofelas/main.c src/user/ofelas/estimator.c src/user/ofelas/ESKF.c src/user/ofelas/measurement_handler.c src/user/ofelas/quaternions.c src/user/ofelas/linalg.c include/user/ofelas/measurement_handling.h include/user/ofelas/ESKF.h include/user/ofelas/estimator.h include/user/ofelas/quaternions.h include/user/ofelas/linalg.h
# 	$(CC) $(CFLAGS) src/user/ofelas/main.c src/user/ofelas/estimator.c src/user/ofelas/ESKF.c src/user/ofelas/measurement_handler.c src/user/ofelas/quaternions.c src/user/ofelas/linalg.c -o main $(LDFLAGS)
