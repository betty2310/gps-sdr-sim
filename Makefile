# Makefile for Linux etc.

.PHONY: all clean time test test-x300tx-matched
all: gps-sdr-sim jammergen matchedgen iqmix

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
JAMMER_SOURCE_OBJ=tools/jammer_source.o
GPS_CA_OBJ=tools/gps_ca.o
MATCHED_CODE_SOURCE_OBJ=tools/matched_code_source.o
MATCHED_CODE_PLAN_OBJ=tools/matched_code_plan.o
SHA256_OBJ=tools/sha256.o

gps-sdr-sim: gpssim.o $(GPS_CA_OBJ)
	${CC} $^ ${LDFLAGS} -o $@

$(GPS_CA_OBJ): tools/gps_ca.c tools/gps_ca.h
	${CC} ${CFLAGS} -c tools/gps_ca.c -o $@

$(JAMMER_SOURCE_OBJ): tools/jammer_source.c tools/jammer_source.h
	${CC} ${CFLAGS} -c tools/jammer_source.c -o $@

$(MATCHED_CODE_SOURCE_OBJ): tools/matched_code_source.c tools/matched_code_source.h tools/gps_ca.h
	${CC} ${CFLAGS} -c tools/matched_code_source.c -o $@

$(MATCHED_CODE_PLAN_OBJ): tools/matched_code_plan.c tools/matched_code_plan.h tools/matched_code_source.h
	${CC} ${CFLAGS} -c tools/matched_code_plan.c -o $@

$(SHA256_OBJ): tools/sha256.c tools/sha256.h
	${CC} ${CFLAGS} -c tools/sha256.c -o $@

jammergen: tools/jammergen.c $(JAMMER_SOURCE_OBJ)
	${CC} ${CFLAGS} tools/jammergen.c $(JAMMER_SOURCE_OBJ) ${LDFLAGS} -o $@

matchedgen: tools/matchedgen.c $(MATCHED_CODE_SOURCE_OBJ) $(GPS_CA_OBJ)
	${CC} ${CFLAGS} tools/matchedgen.c $(MATCHED_CODE_SOURCE_OBJ) $(GPS_CA_OBJ) ${LDFLAGS} -o $@

iqmix: tools/iqmix.c
	${CC} ${CFLAGS} $< ${LDFLAGS} -o $@

gpssim.o: .user-motion-size gpssim.h tools/gps_ca.h

# Library object: gpssim without main() for linking into x300tx
gpssim-lib.o: gpssim.c gpssim.h .user-motion-size
	${CC} ${CFLAGS} -DGPS_SDR_SIM_LIB -c gpssim.c -o $@

player/rtcm3_nav.o: player/rtcm3_nav.cpp player/rtcm3_nav.hpp gpssim.h
	${CXX} ${CXXFLAGS} -isystem . -c player/rtcm3_nav.cpp -o $@

x300tx: player/x300tx.cpp player/matched_code_alignment.h player/rtcm3_nav.o gpssim-lib.o $(GPS_CA_OBJ) $(MATCHED_CODE_SOURCE_OBJ) $(MATCHED_CODE_PLAN_OBJ) $(SHA256_OBJ) gpssim.h
	${CXX} ${CXXFLAGS} -isystem . player/x300tx.cpp player/rtcm3_nav.o gpssim-lib.o $(GPS_CA_OBJ) $(MATCHED_CODE_SOURCE_OBJ) $(MATCHED_CODE_PLAN_OBJ) $(SHA256_OBJ) ${UHD_LIBS} ${LDFLAGS} -o $@

jammertx: player/jammertx.cpp $(JAMMER_SOURCE_OBJ) tools/jammer_source.h
	${CXX} ${CXXFLAGS} -isystem . player/jammertx.cpp $(JAMMER_SOURCE_OBJ) ${UHD_LIBS} ${LDFLAGS} -o $@

BLADE_CFLAGS=$(shell pkg-config --cflags libbladeRF 2>/dev/null)
BLADE_LIBS=$(shell pkg-config --libs libbladeRF 2>/dev/null || echo "-lbladeRF")

BLADE_LIBDIR=$(shell pkg-config --variable=libdir libbladeRF 2>/dev/null || echo "/usr/local/lib")

bladetx: player/bladetx.cpp player/rtcm3_nav.o gpssim-lib.o $(GPS_CA_OBJ) gpssim.h
	${CXX} -O3 -Wall -std=c++17 ${BLADE_CFLAGS} -isystem . player/bladetx.cpp player/rtcm3_nav.o gpssim-lib.o $(GPS_CA_OBJ) ${BLADE_LIBS} ${LDFLAGS} -Wl,-rpath,${BLADE_LIBDIR} -o $@

revive_candidates: tools/revive_candidates.cpp player/rtcm3_nav.o gpssim-lib.o $(GPS_CA_OBJ) gpssim.h
	${CXX} -O3 -Wall -std=c++17 -isystem . tools/revive_candidates.cpp player/rtcm3_nav.o gpssim-lib.o $(GPS_CA_OBJ) ${LDFLAGS} -o $@

tests/test_parse_synth_revive: tests/test_parse_synth_revive.c gpssim.c gpssim.h $(GPS_CA_OBJ)
	${CC} ${CFLAGS} -DGPS_SDR_SIM_LIB -isystem . tests/test_parse_synth_revive.c gpssim.c $(GPS_CA_OBJ) ${LDFLAGS} -o $@

tests/test_revive_transform: tests/test_revive_transform.c gpssim.c gpssim.h $(GPS_CA_OBJ)
	${CC} ${CFLAGS} -DGPS_SDR_SIM_LIB -isystem . tests/test_revive_transform.c gpssim.c $(GPS_CA_OBJ) ${LDFLAGS} -o $@

tests/test_revive_scan: tests/test_revive_scan.c gpssim.c gpssim.h $(GPS_CA_OBJ)
	${CC} ${CFLAGS} -DGPS_SDR_SIM_LIB -isystem . tests/test_revive_scan.c gpssim.c $(GPS_CA_OBJ) ${LDFLAGS} -o $@

tests/test_jammer_source: tests/test_jammer_source.c $(JAMMER_SOURCE_OBJ) tools/jammer_source.h
	${CC} ${CFLAGS} -isystem . tests/test_jammer_source.c $(JAMMER_SOURCE_OBJ) ${LDFLAGS} -o $@

tests/test_gps_ca: tests/test_gps_ca.c $(GPS_CA_OBJ) tools/gps_ca.h
	${CC} ${CFLAGS} -isystem . tests/test_gps_ca.c $(GPS_CA_OBJ) ${LDFLAGS} -o $@

tests/test_matched_code_source: tests/test_matched_code_source.c $(MATCHED_CODE_SOURCE_OBJ) $(GPS_CA_OBJ) tools/matched_code_source.h player/matched_code_alignment.h
	${CC} ${CFLAGS} -isystem . tests/test_matched_code_source.c $(MATCHED_CODE_SOURCE_OBJ) $(GPS_CA_OBJ) ${LDFLAGS} -o $@

tests/test_matched_code_plan: tests/test_matched_code_plan.c $(MATCHED_CODE_PLAN_OBJ) tools/matched_code_plan.h
	${CC} ${CFLAGS} -isystem . tests/test_matched_code_plan.c $(MATCHED_CODE_PLAN_OBJ) ${LDFLAGS} -o $@

tests/test_sha256: tests/test_sha256.c $(SHA256_OBJ) tools/sha256.h
	${CC} ${CFLAGS} -isystem . tests/test_sha256.c $(SHA256_OBJ) ${LDFLAGS} -o $@

test: jammergen matchedgen iqmix tests/test_parse_synth_revive tests/test_revive_transform tests/test_revive_scan tests/test_jammer_source tests/test_gps_ca tests/test_matched_code_source tests/test_matched_code_plan tests/test_sha256
	tests/test_parse_synth_revive
	tests/test_revive_transform
	tests/test_revive_scan
	tests/test_jammer_source
	tests/test_gps_ca
	tests/test_matched_code_source
	tests/test_matched_code_plan
	tests/test_sha256
	cd processing && uv run python ../tests/test_cw_dataset.py

test-x300tx-matched: x300tx matchedgen
	python3 tests/test_x300tx_matched_cli.py

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
	rm -f gpssim.o gpssim-lib.o player/rtcm3_nav.o tools/jammer_source.o tools/gps_ca.o tools/matched_code_source.o tools/matched_code_plan.o tools/sha256.o gps-sdr-sim jammergen matchedgen iqmix jammertx x300tx bladetx revive_candidates tests/test_parse_synth_revive tests/test_revive_transform tests/test_revive_scan tests/test_jammer_source tests/test_gps_ca tests/test_matched_code_source tests/test_matched_code_plan tests/test_sha256 *.bin .user-motion-size

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
