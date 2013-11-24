# LPC1114 Synthesizer
# Matt Sarnoff (msarnoff.org)
# November 24, 2013
#
# Based on the Makefile from the Microbuilder LPC1114 codebase:
# http://www.microbuilder.eu/Projects/LPC1114ReferenceDesign/LPC1114CodeBase.aspx
# Uses lpc21isp version 1.92 for device programming over FTDI USB-to-serial
# cable: https://github.com/capiman/lpc21isp
#
# The environment variable $FTDI_DEV must be set to your serial cable's device
# name. (e.g. /dev/cu.usbserial-A400fYRW)
#
# Type "make" to build.
# Type "make flash" to program the device. The device must be in programming
# mode; this can be accomplished by powering it on while pressing the
# "env select" button, or holding the "env select" button for a couple seconds
# while already running.
# Press the "chord pgm." button to restart after programming is finished.
#
# Programming will not work if a MIDI cable is connected.
# MAKE SURE THE MIDI CABLE IS DISCONNECTED BEFORE AND DURING PROGRAMMING.

OUTFILE = synth
OBJS = kernel.o hardware.o sound.o midi.o main.o
TABLES = notetable.c envtable.c cutofftable.c lfofreqtable.c modenvtable.c
OBJS += $(TABLES:.c=.o)

##########################################################################
# User configuration and firmware specific object files	
##########################################################################

# The target, flash and ram of the LPC1xxx microprocessor.
# Use for the target the value: LPC11xx, LPC13xx or LPC17xx
TARGET = LPC11xx
FLASH = 32K
SRAM = 4K

# crystal frequency in kHz
FXTAL_KHZ = 25000

# For USB support the LPC1xxx reserves 384 bytes from the sram,
# if you don't want to use the USB features, just use 0 here.
SRAM_USB = 0

VPATH = 

##########################################################################
# Debug settings
##########################################################################

# Set DEBUGBUILD to 'TRUE' for full debugging (larger, slower binaries), 
# or to 'FALSE' for release builds (smallest, fastest binaries)
DEBUGBUILD = FALSE

##########################################################################
# Library files 
##########################################################################
VPATH += lpc1xxx
OBJS += cpu.o systick.o

##########################################################################
# GNU GCC compiler prefix and location
##########################################################################
CROSS_COMPILE = arm-none-eabi-
AS = $(CROSS_COMPILE)gcc
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
LPC21ISP = lpc21isp

##########################################################################
# GNU GCC compiler flags
##########################################################################
ROOT_PATH = .
INCLUDE_PATHS = -I$(ROOT_PATH) -I$(ROOT_PATH)/project

##########################################################################
# Startup files
##########################################################################

LD_PATH = lpc1xxx
LD_SCRIPT = $(LD_PATH)/linkscript.ld
LD_TEMP = $(LD_PATH)/memory.ld

ifeq (LPC11xx,$(TARGET))
  CORTEX_TYPE=m0
else
  CORTEX_TYPE=m3
endif

CPU_TYPE = cortex-$(CORTEX_TYPE)
OBJS += $(TARGET)_handlers.o LPC1xxx_startup.o

##########################################################################
# Compiler settings, parameters and flags
##########################################################################
ifeq (TRUE,$(DEBUGBUILD))
  CFLAGS  = -c -g -O0 $(INCLUDE_PATHS) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CPU_TYPE) -DTARGET=$(TARGET) -fno-builtin $(OPTDEFINES)
  ASFLAGS = -c -g -O0 $(INCLUDE_PATHS) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CPU_TYPE) -D__ASSEMBLY__ -x assembler-with-cpp
else
  CFLAGS  = -c -g -Os $(INCLUDE_PATHS) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CPU_TYPE) -DTARGET=$(TARGET) -fno-builtin $(OPTDEFINES)
  ASFLAGS = -c -g -Os $(INCLUDE_PATHS) -Wall -mthumb -ffunction-sections -fdata-sections -fmessage-length=0 -mcpu=$(CPU_TYPE) -D__ASSEMBLY__ -x assembler-with-cpp
endif
LDFLAGS = -nostartfiles -mcpu=$(CPU_TYPE) -mthumb -Wl,--gc-sections
LDLIBS  = -lm
OCFLAGS = --strip-unneeded

all: firmware

%.c : %.py
	./tablegen.py $<

%.o : %.c
	$(CC) $(CFLAGS) -o $@ $<

%.o : %.s
	$(AS) $(ASFLAGS) -o $@ $<

firmware: $(OBJS) $(SYS_OBJS)
	-@echo "MEMORY" > $(LD_TEMP)
	-@echo "{" >> $(LD_TEMP)
	-@echo "  flash(rx): ORIGIN = 0x00000000, LENGTH = $(FLASH)" >> $(LD_TEMP)
	-@echo "  sram(rwx): ORIGIN = 0x10000000+$(SRAM_USB), LENGTH = $(SRAM)-$(SRAM_USB)" >> $(LD_TEMP)
	-@echo "}" >> $(LD_TEMP)
	-@echo "INCLUDE $(LD_SCRIPT)" >> $(LD_TEMP)
	$(LD) $(LDFLAGS) -T $(LD_TEMP) -o $(OUTFILE).elf $(OBJS) $(LDLIBS)
	-@echo ""
	$(SIZE) $(OUTFILE).elf
	-@echo ""
	$(OBJCOPY) $(OCFLAGS) -O binary $(OUTFILE).elf $(OUTFILE).bin
	$(OBJCOPY) $(OCFLAGS) -O ihex $(OUTFILE).elf $(OUTFILE).hex
  
clean:
	rm -f $(OBJS) $(TABLES) $(LD_TEMP) $(OUTFILE).elf $(OUTFILE).bin $(OUTFILE).hex

flash: firmware
	$(LPC21ISP) -control -controlswap $(OUTFILE).hex $(FTDI_DEV) 115200 $(FXTAL_KHZ)	

run: firmware
	$(LPC21ISP) -term -control -controlswap $(OUTFILE).hex $(FTDI_DEV) 115200 $(FXTAL_KHZ)	

dis: firmware
	$(OBJDUMP) -D $(OUTFILE).elf | less
