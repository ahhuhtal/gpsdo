all: main.elf

CH32V003FUN:=../common
MINICHLINK:=../common

TARGET:=main
ADDITIONAL_C_FILES:=../common/i2c.c

include $(CH32V003FUN)/ch32v003fun.mk

flash: cv_flash
clean: cv_clean
