GCC_RESULT := $(shell g++-11 --version >/dev/null 2>&1 || (echo "Your command failed with $$?"))

ifeq (,${GCC_RESULT})
    CXX := g++-11
else
    CXX := g++
endif

# CXXFLAGS = -std=c++11 -DLOG -O3 -march=native
CXXFLAGS = -std=c++11 -O3 -march=native
# CXXFLAGS = -std=c++11 -pg -g  -Wall -Wextra -pedantic -DDEBUG
# CXXFLAGS = -std=c++11 -g -Wall -Wno-sign-compare -Wextra -pedantic -DDEBUG
# CXXFLAGS = -std=c++11 -g -DLOG -Wall -Wno-sign-compare -Wextra -pedantic -DDEBUG

LIBS = db.o \
		graph.o \
		parse.o \
		tensor.o \
		node.o \
		net.o \
		log.o


RES = main.cpp
EXE = topart
TESTCASE_DIR = ./input
# OUTPUT_DIR = ./output
OUTPUT_DIR = /dev/shm
VERIFY_DIR = ./verifier

.PHONY: test clean upload

CASE = 1

all: $(LIBS)
	$(CXX) $(CXXFLAGS) $(RES) $(LIBS) -o $(EXE)
	-rm $(LIBS)

debug:
	-rm $(OUTPUT_DIR)/*.txt
	@echo "r $(TESTCASE_DIR)/B$(CASE).txt $(OUTPUT_DIR)/B$(CASE).txt"
	@gdb -q -ex 'file $(EXE)'

test:
	-rm $(OUTPUT_DIR)/*.txt
	@echo "./$(EXE) $(TESTCASE_DIR)/B$(CASE).txt $(OUTPUT_DIR)/B$(CASE).txt"
	time ./$(EXE) $(TESTCASE_DIR)/B$(CASE).txt $(OUTPUT_DIR)/B$(CASE).txt
	./verify $(TESTCASE_DIR)/B$(CASE).txt $(OUTPUT_DIR)/B$(CASE).txt

clean:
	-rm $(EXE)
	-rm $(LIBS)

benchmark: packing_gz
	-rm -rf ../HW4_grading
	tar -zxvf ../HW4_grading.tar.gz -C ../
	mkdir -p ../HW4_grading/student/111062506
	cp CS6135_HW4_111062506.tar.gz ../HW4_grading/student/111062506/
	cd ../HW4_grading && bash HW4_grading.sh

upload:
	-rm -rf upload
	mkdir -p upload
	cp *.cpp upload
	cp *.h upload
	cp Makefile upload
	cp -r input/ upload
	cp verify upload
	ssh -p 10022 fpga-111062506@maklab.cs.nthu.edu.tw 'rm -rf ~/upload'
	scp -P10022 -r upload fpga-111062506@maklab.cs.nthu.edu.tw:~/
	-rm -rf upload

# submit: testall packing_gz benchmark
# 	echo "finish"

# packing_gz:
# 	rm -rf HW4 CS6135_HW4_111062506.tar.gz
# 	mkdir -p HW4/src
# 	cp *.cpp HW4/src/
# 	cp *.h HW4/src/
# 	cp Makefile HW4/src/
# 	cp README HW4/src/
# 	mkdir -p HW4/output
# 	mkdir -p HW4/bin
# 	cp ../bin/hw4 HW4/bin/hw4
# 	# cp ../vlsi-hw4.pdf HW4/CS6135_HW4_111062506_report.pdf
# 	echo "below is unneccesary"
# 	cp -r ../testcase HW4/
# 	cp -r ../verifier HW4/
# 	tar -zcvf CS6135_HW4_111062506.tar.gz HW4/
# 	cp CS6135_HW4_111062506.tar.gz ../
# 	-rm -rf HW4

# send_cases:
# 	scp -r testcases g111062506@ic:~/hw2

# send:
# 	scp -r src u107062115@ic:~/hw2

# packing:
# 	-rm -rf CS3130_PA2 PA2_107062115.tar.gz
# 	mkdir CS3130_PA2
# 	cp -r src/ CS3130_PA2/
# 	cp Makefile CS3130_PA2
# 	tar -zcvf PA2_107062115.tar.gz CS3130_PA2/
# 	-rm -rf CS3130_PA2
