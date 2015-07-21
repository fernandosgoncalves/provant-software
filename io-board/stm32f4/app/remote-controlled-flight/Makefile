############################################################################
#
#    Makefile for STM32F4-H407
#
#    Run 'make' to compile code. 
#
#    Run 'make doc' do compile documentation. Doxygen and Dot are required.
#    Both are available in the general Ubuntu repository, or can be compiled
#    from scratch. Doxygen version >1.8 is required.
#
############################################################################

# project name
PRJNAME = prj-remote-controlled-flight

# toolchain configuration
TOOLCHAIN_PREFIX = arm-none-eabi-

# cpu configuration
THUMB = -mthumb
MCU   = cortex-m4

#BOARD =-DSTM32F4_H407
BOARD =-DSTM32F4_DISCOVERY

# path, root and common dir
APPDIR := $(shell pwd)
PATH   := ${PATH}:$(APPDIR)
BASE   := $(APPDIR)/../../base
CORE   := $(APPDIR)/../../core

# project directories
OUTDIR       := $(APPDIR)/bin
INCDIR       := $(APPDIR)/inc
HALDIR		 := $(BASE)/hal
MODDIR       := $(BASE)/modules
LIBDIR       := $(BASE)/lib
FSMDIR       := $(BASE)/fsm

# Modules
MODULES += $(BASE)/modules/rc
MODULES += $(BASE)/modules/io
MODULES += $(BASE)/modules/datapr

# CMSIS Device Drivers directory
CMSISDIR     := $(CORE)/cmsis
CMSISSRCDIR  := $(CMSISDIR)/src
CMSISINCDIR  := $(CMSISDIR)/inc

# CMSIS DSP directory
DSPDIR 		:= $(CORE)/DSP_Lib
DSPSRC		:= $(DSPDIR)/Source
DSPLIB		:= $(DSPDIR)/Lib/GCC
DSPINC		:= $(DSPSRC)/*

# FreeRTOS directories
FRTDIR    := $(BASE)/FreeRTOSV7.5.2/FreeRTOS
FRTSRCDIR := $(FRTDIR)/Source
FRTINCDIR := $(FRTSRCDIR)/include
FRTMEMDIR := $(FRTSRCDIR)/portable/MemMang/
FRTPORDIR := $(FRTSRCDIR)/portable/GCC/ARM_CM4F

# C source files

# Application sources
C_SRC += $(APPDIR)/main.c
C_SRC += $(APPDIR)/*.c

# System, HAL and Modules
C_SRC += $(CORE)/system/*.c
C_SRC += $(BASE)/hal/*.c
C_SRC += $(BASE)/modules/rc/*.c
C_SRC += $(BASE)/modules/io/*.c
C_SRC += $(BASE)/modules/datapr/*.c

# Libraries 
C_SRC += $(BASE)/lib/provant/*.c

# DSP
#C_SRC += $(DSPSRC)/*/*.c

# FreeRTOS
C_SRC += $(FRTSRCDIR)/list.c 
C_SRC += $(FRTSRCDIR)/queue.c 
C_SRC += $(FRTSRCDIR)/tasks.c 
C_SRC += $(FRTSRCDIR)/timers.c 
C_SRC += $(FRTPORDIR)/port.c  
C_SRC += $(FRTMEMDIR)/heap_2.c

# FreeRTOS+Trace
C_SRC += $(LIBDIR)/trace/*.c

# Assembler source files
A_SRC = 
	
###################################################

# executables
DOXYGEN = doxygen

# include directories
INCDIRS = $(INCDIR) $(LIBDIR) $(MODULES) $(APPDIR) $(LIBDIR)/provant $(LIBDIR)/trace \
	      $(BASE)/lib $(BASE)/fsm $(BASE)/hal $(CORE)/system  						 \
	  	  $(CMSISDIR) $(CMSISINCDIR) $(CMSISINCDIR)/peripherals $(CMSISINCDIR)/core  \
	  	  $(FRTINCDIR) $(FRTPORDIR)													 \
	  	  $(LIBDIR)/trace/config $(LIBDIR)/trace/inc $(DSPINC)		 						

# add startup file to build
C_SRC += $(CMSISDIR)/startup_stm32f4xx.s	

# Define programs and commands.
CC      = $(TOOLCHAIN_PREFIX)gcc
OBJCOPY = $(TOOLCHAIN_PREFIX)objcopy
OBJDUMP = $(TOOLCHAIN_PREFIX)objdump
NM      = $(TOOLCHAIN_PREFIX)nm

# compiler flags
CFLAGS  = -g -T$(CORE)/system/stm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=$(MCU) -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I. $(patsubst %,-I%,$(INCDIRS))
CFlAGS += -DARM_MATH_CM4 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING
# verbose flags, all may be commented out
#CFLAGS += -Wall  -Wunused-value -Wunused-variable -Wunused-but-set-variable #-ftime-report
# cpp related flags
CFLAGS += -Og -O0 -ffunction-sections -fdata-sections -fno-exceptions --specs=nano.specs -Wl,--gc-sections  
CFLAGS += -std=c99 
CFLAGS += $(BOARD)
CFLAGS +=-D__FPU_PRESENT


###################################################

# build again when Makefile changes
BUILDONCHANGE = Makefile

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: doc lib proj

all: directories lib proj

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

doc:
	cd $(APPDIR)/doc && $(DOXYGEN) Doxyfile
	cd ..

directories:
	mkdir -p bin

lib:
	$(MAKE) -C $(CMSISDIR)

proj: 	$(PRJNAME).elf

$(PRJNAME).elf: $(C_SRC) 
	$(CC) $(CFLAGS) $^ -o $(OUTDIR)/$@ -L$(CMSISDIR) -L$(DSPLIB) -lc -lm -lstm32f4 -lstdc++ -lnosys -larm_cortexM4lf_math
	$(OBJCOPY) -O ihex $(OUTDIR)/$(PRJNAME).elf $(OUTDIR)/$(PRJNAME).hex
	$(OBJCOPY) -O binary $(OUTDIR)/$(PRJNAME).elf $(OUTDIR)/$(PRJNAME).bin

clean:
	rm -f *.o
	rm -f $(OUTDIR)/$(PRJNAME).elf
	rm -f $(OUTDIR)/$(PRJNAME).hex
	rm -f $(OUTDIR)/$(PRJNAME).bin
	
docclean:
	cd $(APPDIR)/doc && rm -r latex/ && rm -r html/

allclean: clean	
	$(MAKE) clean -C $(CMSISDIR)

