SHELL=/bin/bash
export TOOLCHAIN_PATH ?= /opt/toolchain-mips_34kc_gcc-4.8-linaro_uClibc-0.9.33.2
export PATH := ${TOOLCHAIN_PATH}/bin:${PATH}
export STAGING_DIR ?= ${TOOLCHAIN_PATH}/staging_dir 
export SAINTLY_CROSS ?= mips-openwrt-linux-uclibc
export CC := ${SAINTLY_CROSS}-gcc
export LD := ${SAINTLY_CROSS}-ld
export AR := ${SAINTLY_CROSS}-ar

OBJ=decode.c saintly.c
OUT=saintly

.PHONY: all

all:
	${CC} -std=c11 -Ofast -lpthread -lm ${OBJ} -I. -o ${OUT}

clean:
	rm saintly