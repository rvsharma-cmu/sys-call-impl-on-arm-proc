# Makefile for kernel demo

TOOLS   = arm-none-eabi
AS      = $(TOOLS)-as
CC      = $(TOOLS)-gcc
LD      = $(TOOLS)-ld.bfd
OBJCOPY = $(TOOLS)-objcopy
DUMP    = $(TOOLS)-objdump -D
GDB     = $(TOOLS)-gdb
MKDIR_P = mkdir -p
CP      = cp

############### LIBRARY INCLUSION ######################

USER_PROJ       = default
FLOAT           = soft
DEBUG           = 1
USER_ARG        = 0

USER_PROJ_BUILD  = user
PROJ_BUILD       = kernel
USER_PROJ_COMMON = user_common
PROJ             = kernel
BUILD            = build
BOOT             = asm
BIN              = bin

# Terminal color and modifier attributes. Make sure to handle when no terminal
# is connected.
# Return to the normal terminal colors
n := $(shell tty -s && tput sgr0)
# Red color
r := $(shell tty -s && tput setaf 1)
# Green color
g := $(shell tty -s && tput setaf 2)
# Yellow color
y := $(shell tty -s && tput setaf 3)
# Purple color
j := $(shell tty -s && tput setaf 5)
# Bold text
b := $(shell tty -s && tput bold)
# Underlined text
u := $(shell tty -s && tput smul)

# BIN INFO
HASH_KERNEL      = $(shell echo -n "$(DEBUG)$(OPTIMIZATION)$(FLOAT)" | md5sum | cut -d' ' -f1)
HASH_USER        = $(shell echo -n "$(DEBUG)$(OPTIMIZATION)$(FLOAT)$(USER_ARG)" | md5sum | cut -d' ' -f1)
BIN_DIR          = $(BUILD)/$(BIN)
BINARY           = $(PROJ)_$(USER_PROJ)_$(HASH_USER)

# KERNEL PROJ DIRS
K_BOOT_DIR       = $(PROJ)/$(BOOT)
U_BOOT_DIR       = $(USER_PROJ_COMMON)/$(BOOT)
K_INC_DIR        = $(PROJ)/include
K_LIB_DIR        = $(PROJ)/lib
K_SRC_DIR        = $(PROJ)/src
K_OBJ_DIR        = $(BUILD)/$(PROJ_BUILD)
K_OBJ_PROJ_DIR   = $(K_OBJ_DIR)/$(PROJ)_$(HASH_KERNEL)

# USER PROJ DIRS
U_COMMON_INC_DIR = $(USER_PROJ_COMMON)/include
U_COMMON_SRC_DIR = $(USER_PROJ_COMMON)/src
U_LIB_DIR        = $(USER_PROJ_COMMON)/lib
U_PROJ_DIR       = user_proj/$(USER_PROJ)/src
U_PROJ_INC_DIR   = user_proj/$(USER_PROJ)/include
U_OBJ_DIR        = $(BUILD)/$(USER_PROJ_BUILD)
U_OBJ_PROJ_DIR   = $(U_OBJ_DIR)/$(USER_PROJ)_$(HASH_USER)

# SRC FILES
K_SRC             = $(wildcard $(K_SRC_DIR)/*.c)
U_SRC             = $(wildcard $(U_PROJ_DIR)/*.c)
U_SRC_COMMON      = $(wildcard $(U_COMMON_SRC_DIR)/*.c)
U_BOOT            = $(wildcard $(U_BOOT_DIR)/*.S)
K_BOOT            = $(wildcard $(K_BOOT_DIR)/*.S)

# RULES
K_OBJ_RULE        = $(K_SRC:$(K_SRC_DIR)/%.c=$(K_OBJ_PROJ_DIR)/%.o)
U_OBJ_RULE        = $(U_SRC:$(U_PROJ_DIR)/%.c=$(U_OBJ_PROJ_DIR)/%.o)
U_COMMON_OBJ_RULE = $(U_SRC_COMMON:$(U_COMMON_SRC_DIR)/%.c=$(U_OBJ_PROJ_DIR)/%.o)
U_BOOT_RULE       = $(U_BOOT:$(U_BOOT_DIR)/%.S=$(U_OBJ_PROJ_DIR)/%.o)
K_BOOT_RULE       = $(K_BOOT:$(K_BOOT_DIR)/%.S=$(K_OBJ_PROJ_DIR)/%.o)

# OBJECTS
K_OBJECTS         = $(wildcard $(K_OBJ_PROJ_DIR)/*.o)
U_OBJECTS         = $(wildcard $(U_OBJ_PROJ_DIR)/*.o)

# Final output
OUTPUT            = $(BIN_DIR)/$(BINARY)

# Path to soft float lib
SOFT_FLOAT_LIB    = $(U_LIB_DIR)/soft_float/libgcc.a

# FLOAT_ARCH = -fsingle-precision-constant -Wdouble-promotion
# Case on float type, soft by default
ifeq ($(FLOAT), soft)
	U_LIB_FILES = $(U_LIB_DIR)/soft_float/libc.a $(SOFT_FLOAT_LIB)
	FLOAT_ARCH  += -mfloat-abi=soft
else
	U_LIB_FILES = $(U_LIB_DIR)/hard_float/libc.a $(U_LIB_DIR)/hard_float/libm.a  $(SOFT_FLOAT_LIB)
	FLOAT_ARCH  += -mfloat-abi=hard -mfpu=fpv4-sp-d16  -march=armv7e-m
endif

# DEBUGGING is enabled by default, you can reduce binary size by disabling the
# DEBUGGING
ifeq ($(DEBUG), 1)
	DEFINE_MACROS = -DDEBUG -g
	OPTIMIZATION  = -O0
else
	OPTIMIZATION = -O3 -funroll-all-loops
endif

ARCH                 = $(ARG) $(FLOAT_ARCH) -mslow-flash-data -mcpu=cortex-m4 -mlittle-endian -mthumb -ffreestanding
COMPILER_ERROR_FLAGS = -std=gnu99 -Wall -Werror -Wshadow -Wextra -Wunused
C_LIB_FLAG           = -nostdlib
CCFLAGS              += $(ARCH) $(COMPILER_ERROR_FLAGS) $(C_LIB_FLAG) $(OPTIMIZATION) $(DEFINE_MACROS)
K_CCFLAGS            = $(CCFLAGS) -nostartfiles
U_CCFLAGS            = $(CCFLAGS)

########################################################

################### ROOT RULES #########################
.PHONY: help setup flash doc clean veryclean $(BIN_DIR)/$(BINARY).elf
.SILENT:setup flash
# COMMENT LINE FOR VERBOSE LINKING
.SILENT:$(BIN_DIR)/$(BINARY).elf

help:
	@printf "$b18-349 Makefile Usage:$n\n"
	@printf "\n"
	@printf "$bTargets:$n\n"
	@printf "\t$bbuild$n\n"
	@printf "\t    Compiles and links the specified $bPROJ$n and $bUSER_PROJ$n.\n"
	@printf "\t    If no $bPROJ$n,$bUSER_PROJ$n is specified then kernel will be linked with default.\n"
	@printf "\n"
	@printf "\t$bflash$n\n"
	@printf "\t    Compile, link and upload binary to board over serial.\n"
	@printf "\t    Be sure to run $bwindow_ocd.batch$n if you are in windows.\n"
	@printf "\t    Be sure to run $b./linux.ocd$n if you are in linux/mac.\n"
	@printf "\n"
	@printf "\t$bview-dump$n\n"
	@printf "\t    Compile, link and show disassembled binary.\n"
	@printf "\n"
	@printf "\t$bdoc$n\n"
	@printf "\t    Builds doxygen and ouputs into $bdoxygen_docs$n.\n"
	@printf "\t    Check $bdoxygen.warn$n for errors\n"
	@printf "\n"
	@printf "\t$bclean$n\n"
	@printf "\t    Cleans up all of the files generated by compilation in the\n"
	@printf "\t    $b$(BUILD)$n directory.\n"
	@printf "\n"
	@printf "\t$bcleandoc$n\n"
	@printf "\t    Cleans up generated doxygen files\n"
	@printf "\n"
	@printf "$bVariables:$n\n"
	@printf "\t$bPROJ$n\n"
	@printf "\t    The code to run in supervisor mode.\n"
	@printf "\n"
	@printf "\t$bUSER_PROJ$n\n"
	@printf "\t    The user level program to link.\n"
	@printf "\n"
	@printf "\t$bUSER_ARG$n\n"
	@printf "\t    The user arg to be passed to the user program.\n"
	@printf "\t    Takes a space separated string .\n"
	@printf "\t    eg - $bUSER_ARG=\"Hello World\".\n"
	@printf "\n"
	@printf "\t$bOPTIMIZATION$n\n"
	@printf "\t    Sets the optimization level eg - $b-O3/-Os$n\n"
	@printf "\n"
	@printf "\t$bFLOAT$n\n"
	@printf "\t    Use soft or hard floating point libraries\n"
	@printf "\n"
	@printf "$bExamples:$n\n"
	@printf "\tmake build\n"
	@printf "\tmake build USER_PROJ=test_0_0\n"
	@printf "\tmake flash USER_PROJ=test_0_1 OPTIMIZATION=-O3\n"
	@printf "\tmake flash USER_PROJ=test_0_1 USER_ARG=\"1 2 3\"\n"

compile: $(BIN_DIR)/$(BINARY).bin
	@printf "\n$g$b$uBuilt PROJ=$(PROJ) with USER_PROJ=$(USER_PROJ), FLOAT=$(FLOAT), DEBUG=$(DEBUG), OPTIMIZATION=$(OPTIMIZATION)$n$n$n\n"

setup:
	$(MKDIR_P) $(BUILD)
	$(MKDIR_P) $(BIN_DIR)
	$(MKDIR_P) $(U_OBJ_DIR)
	$(MKDIR_P) $(K_OBJ_DIR)
	$(MKDIR_P) $(K_OBJ_PROJ_DIR)
	$(MKDIR_P) $(U_OBJ_PROJ_DIR)

build : setup compile

flash:	build
	cp util/init_template.nc /tmp/init.nc
	sed -i -e 's|<template>|$(BINARY)|g' /tmp/init.nc
	cp util/init_template.gdb /tmp/init.gdb
	sed -i -e 's|<template>|$(BINARY)|g' /tmp/init.gdb
	@printf "\n"
	@printf "$y***************************************************************\n$n"
	@printf "$yFlashing $(BINARY) to board...\n$n"
	@printf "$y***************************************************************\n$n"
	cat /tmp/init.nc | nc localhost 4444
	@printf "Opening GDB...\n"
	$(GDB) -x /tmp/init.gdb

view-dump: build dump

dump:
	$(DUMP) $(BIN_DIR)/$(BINARY).elf | less

########################################################

################# COMPILATION RULES ####################

$(K_OBJ_PROJ_DIR)/%.o: $(K_SRC_DIR)/%.c
	@printf "\n$b$yCompiling: $<$n$n\n" $<
	$(CC) -I$(K_INC_DIR) $(K_CCFLAGS) -c $< -o $@

$(K_OBJ_PROJ_DIR)/%.o: $(K_BOOT_DIR)/%.S
	@printf "\n$y$bAssembling: $<$n$n\n"
	$(AS) $< -o $@

$(U_OBJ_PROJ_DIR)/%.o: $(U_COMMON_SRC_DIR)/%.c
	@printf "\n$y$bCompiling: $<$n$n\n"
	python util/generate_argv.pyc /tmp/349_arg.h $(USER_ARG)
	$(CC) $(U_CCFLAGS) -I$(U_COMMON_INC_DIR) -c $< -o $@

$(U_OBJ_PROJ_DIR)/%.o: $(U_BOOT_DIR)/%.S
	@printf "\n$y$bCompiling: $<$n$n\n"
	$(CC) $(U_CCFLAGS) -I$(U_COMMON_INC_DIR) $< -o $@

$(U_OBJ_PROJ_DIR)/%.o: $(U_PROJ_DIR)/%.c
	$(CC) $(U_CCFLAGS) -I$(U_COMMON_INC_DIR) -I$(U_PROJ_INC_DIR) -c $< -o $@

########################################################

################### BINARY RULES #######################

$(BIN_DIR)/$(BINARY).bin:	$(BIN_DIR)/$(BINARY).elf
	@printf "\n$j$bCreating $(BINARY) binary file...$n$n\n"
	$(OBJCOPY) $(BIN_DIR)/$(BINARY).elf $(BIN_DIR)/$(BINARY).bin -O binary

$(BIN_DIR)/$(BINARY).elf: $(K_OBJ_RULE) $(K_BOOT_RULE) $(U_OBJ_RULE) $(U_BOOT_RULE) $(U_COMMON_OBJ_RULE)
	cp util/linker_template.lds /tmp/linker.lds
	sed -i -e 's|<K_OBJ_DIR>|$(K_OBJ_PROJ_DIR)|g' /tmp/linker.lds
	sed -i -e 's|<U_OBJ_DIR>|$(U_OBJ_PROJ_DIR)|g' /tmp/linker.lds
	@printf "\n$j$bLinking $(BINARY)...$n$n\n"
	$(LD) -T /tmp/linker.lds -o $(BIN_DIR)/$(BINARY).elf $(U_OBJECTS) $(K_OBJECTS) $(U_LIB_FILES)

########################################################

################### CLEANING RULES #####################

clean:
	$(RM) $(BIN_DIR)/*
	$(RM) -r $(K_OBJ_DIR)/*

veryclean:
	$(RM) -r $(BUILD)
	$(RM) doxygen.warn
	$(RM) -r doxygen_docs

########################################################

################### DOC RULES #########################

doc:
	doxygen util/doxygen/Doxyfile

cleandoc:
	$(RM) doxygen.warn
	$(RM) -r doxygen_docs
