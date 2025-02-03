all: main.elf

CH32V003FUN:=../ch32v003fun/ch32v003fun
MINICHLINK:=../ch32v003fun/minichlink

TARGET:=main
EXTRA_CFLAGS:=-I../common/
ADDITIONAL_C_FILES:=../common/unixtime.c ../common/i2c.c

include $(CH32V003FUN)/ch32v003fun.mk

flash: cv_flash
clean: cv_clean
