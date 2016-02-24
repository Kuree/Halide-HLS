#### Halide flags
HALIDE_BIN_PATH := ../../..
HALIDE_SRC_PATH := ../../..
include ../../support/Makefile.inc

#### HLS flags
include ../hls_support/Makefile.inc

all: out.png

pipeline: pipeline.cpp
	$(CXX) $(CXXFLAGS) -Wall -g $^ $(LIB_HALIDE) -o $@ $(LDFLAGS) -ltinfo

pipeline_hls.cpp pipeline_native.o pipeline_zynq.c pipeline_zynq.o: pipeline
	HL_DEBUG_CODEGEN=0 ./pipeline

run: run.cpp pipeline_hls.cpp hls_target.cpp pipeline_native.o
	$(CXX) $(CXXFLAGS) -O1 -DNDEBUG $(HLS_CXXFLAGS) -g -Wall -Werror $^ -lpthread -ldl $(PNGFLAGS) -o $@

out.png: run
	./run ../../images/rgb.png out.png

#pipeline_zynq.o: pipeline_zynq.c
#	$(CXX) -c -O2 $(CXXFLAGS) -g -Wall -Werror $^ -o $@

run_zynq.o: run_zynq.cpp
	$(CXX) -c $(CXXFLAGS) -g -Wall -Werror $^ -o $@  $(PNGFLAGS)

run_zynq: pipeline_zynq.o pipeline_native.o run_zynq.o
	$(CXX) -Wall -Werror $^ -lpthread -ldl -o $@  $(PNGFLAGS)

out_zynq.png: run_zynq
	HL_NUM_THREADS=3 ./run_zynq ../../images/gray.png out_zynq.png

clean:
	rm -f pipeline run run_zynq
	rm -f out.png out_zynq.png
	rm -f pipeline_native.h pipeline_native.o
	rm -f pipeline_hls.h pipeline_hls.cpp
	rm -f pipeline_zynq.h pipeline_zynq.c pipeline_zynq.o
	rm -f run_zynq.o
	rm -f hls_target.h hls_target.cpp