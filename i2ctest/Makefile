all: main.elf

TARGET_MCU:=CH32V003

TARGET:=main
EXTRA_CFLAGS:=-I../common/
ADDITIONAL_C_FILES:=../common/i2c.c

include ../ch32v003fun/ch32fun/ch32fun.mk

flash: cv_flash
clean: cv_clean
