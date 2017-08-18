.PHONY: all
all: matrices

CFLAGS=-g -Wall -Wno-reorder
#CFLAGS=-Os -Wall -Wno-reorder -D_RELEASE

IGL_INC=-I/usr/local/igl/igl_lib/
ANT_INC=-I../../external/AntTweakBar/include
ANT_LIB=-L../../external/AntTweakBar/lib -lAntTweakBar
EIGEN3_INC=-I/usr/local/include/eigen3 -I/usr/local/include/eigen3/unsupported
FRAMEWORKS=-framework OpenGL -framework GLUT -framework AppKit

CC=llvm-g++
#CC=g++

INC=-I.. -I. ${EIGEN3_INC} ${ANT_INC} ${IGL_INC} ${GLSL_INC}
LIB=${ANT_LIB} ${FRAMEWORKS}

matrices: main.o arap_dof.o WunderSVD3x3.o
	${CC} ${CFLAGS} main.o arap_dof.o wunderSVD3x3.o -o matrices ${LIB}
#matrices: main.o
#${CC} ${CFLAGS} main.o -o matrices ${LIB}

main.o: main.cpp
	${CC} ${CFLAGS} -c main.cpp -o main.o ${INC}

arap_dof.o: ../arap_dof.cpp
	${CC} ${CFLAGS} -c ../arap_dof.cpp -o arap_dof.o ${INC}

WunderSVD3x3.o: ../WunderSVD3x3.cpp ../Singular_Value_Decomposition_Preamble.hpp
	${CC} ${CFLAGS} -c ../WunderSVD3x3.cpp -o WunderSVD3x3.o ${INC}


.PHONY: clean
clean:
	rm -f matrices main.o arap_dof.o WunderSVD3x3.o
