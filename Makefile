# Makefile for Linux etc.

.PHONY: all clean time test
all: gps-sdr-sim

rtcm3_inspect: tools/rtcm3_inspect.c
	${CC} ${CFLAGS} tools/rtcm3_inspect.c ${LDFLAGS} -o $@

SHELL=/bin/bash
CC=gcc
CXX=g++
CFLAGS=-O3 -Wall -D_FILE_OFFSET_BITS=64
CXXFLAGS=-O3 -Wall -std=c++17 $(shell pkg-config --cflags uhd)
ifdef USER_MOTION_SIZE
CFLAGS+=-DUSER_MOTION_SIZE=$(USER_MOTION_SIZE)
CXXFLAGS+=-DUSER_MOTION_SIZE=$(USER_MOTION_SIZE)
endif
LDFLAGS=-lm
UHD_LIBS=$(shell pkg-config --libs uhd)
BOOST_LIBDIRS=$(wildcard /opt/homebrew/lib /usr/local/lib)
BOOST_LIBS=$(addprefix -L,$(BOOST_LIBDIRS)) -lboost_program_options -lboost_thread

gps-sdr-sim: gpssim.o
	${CC} $< ${LDFLAGS} -o $@

gpssim.o: .user-motion-size gpssim.h

# Library object: gpssim without main() for linking into x300tx
gpssim-lib.o: gpssim.c gpssim.h .user-motion-size
	${CC} ${CFLAGS} -DGPS_SDR_SIM_LIB -c gpssim.c -o $@

player/rtcm3_nav.o: player/rtcm3_nav.cpp player/rtcm3_nav.hpp gpssim.h
	${CXX} ${CXXFLAGS} -isystem . -c player/rtcm3_nav.cpp -o $@

x300tx: player/x300tx.cpp gpssim-lib.o gpssim.h
	${CXX} ${CXXFLAGS} -isystem . player/x300tx.cpp gpssim-lib.o ${UHD_LIBS} ${LDFLAGS} -o $@

BLADE_CFLAGS=$(shell pkg-config --cflags libbladeRF 2>/dev/null)
BLADE_LIBS=$(shell pkg-config --libs libbladeRF 2>/dev/null || echo "-lbladeRF")

BLADE_LIBDIR=$(shell pkg-config --variable=libdir libbladeRF 2>/dev/null || echo "/usr/local/lib")

bladetx: player/bladetx.cpp player/rtcm3_nav.o gpssim-lib.o gpssim.h
	${CXX} -O3 -Wall -std=c++17 ${BLADE_CFLAGS} -isystem . player/bladetx.cpp player/rtcm3_nav.o gpssim-lib.o ${BLADE_LIBS} ${LDFLAGS} -Wl,-rpath,${BLADE_LIBDIR} -o $@

tests/test_parse_synth_revive: tests/test_parse_synth_revive.c gpssim.c gpssim.h
	${CC} ${CFLAGS} -DGPS_SDR_SIM_LIB -isystem . tests/test_parse_synth_revive.c gpssim.c ${LDFLAGS} -o $@

tests/test_revive_transform: tests/test_revive_transform.c gpssim.c gpssim.h
	${CC} ${CFLAGS} -DGPS_SDR_SIM_LIB -isystem . tests/test_revive_transform.c gpssim.c ${LDFLAGS} -o $@

tests/test_revive_scan: tests/test_revive_scan.c gpssim.c gpssim.h
	${CC} ${CFLAGS} -DGPS_SDR_SIM_LIB -isystem . tests/test_revive_scan.c gpssim.c ${LDFLAGS} -o $@

test: tests/test_parse_synth_revive tests/test_revive_transform tests/test_revive_scan
	tests/test_parse_synth_revive
	tests/test_revive_transform
	tests/test_revive_scan

tx: tx_samples_from_file.cpp
	${CXX} ${CXXFLAGS} $< ${UHD_LIBS} ${BOOST_LIBS} ${LDFLAGS} -o $@

.user-motion-size: .FORCE
	@if [ -f .user-motion-size ]; then \
		if [ "`cat .user-motion-size`" != "$(USER_MOTION_SIZE)" ]; then \
			echo "Updating .user-motion-size"; \
			echo "$(USER_MOTION_SIZE)" >| .user-motion-size; \
		fi; \
	else \
		echo "$(USER_MOTION_SIZE)" > .user-motion-size; \
	fi;

clean:
	rm -f gpssim.o gpssim-lib.o player/rtcm3_nav.o gps-sdr-sim x300tx bladetx tests/test_parse_synth_revive tests/test_revive_transform tests/test_revive_scan *.bin .user-motion-size

time: gps-sdr-sim
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 1
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 8
	time ./gps-sdr-sim -e brdc3540.14n -u circle.csv -b 16

.FORCE:

YEAR?=$(shell date +"%Y")
Y=$(patsubst 20%,%,$(YEAR))
%.$(Y)n:
	wget -q ftp://cddis.gsfc.nasa.gov/gnss/data/daily/$(YEAR)/brdc/$@.Z -O $@.Z
	uncompress $@.Z
