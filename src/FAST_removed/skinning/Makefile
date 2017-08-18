# Make file for GLUT skinning app
.PHONY: all
all: skinning

#CFLAGS=-g -Wall -Werror -Wno-reorder
CFLAGS=-O3 -Wall -Wno-reorder -Wno-unknown-pragmas -D_RELEASE 
#CFLAGS+=-DIGL_HEADER_ONLY
IGL=/usr/local/igl/igl_lib/
IGL_LIB=-L$(IGL)/lib -ligl
#IGL_LIB=
IGL_INC=-I$(IGL)/include

ANT_INC=-I../external/AntTweakBar/include
ANT_LIB=-L../external/AntTweakBar/lib -lAntTweakBar
EIGEN3_INC=-I/usr/local/include/eigen3 -I/usr/local/include/eigen3/unsupported
FRAMEWORKS=-framework OpenGL -framework GLUT -framework AppKit


INC=-I. ${EIGEN3_INC} ${ANT_INC} ${IGL_INC}
LIB=${ANT_LIB} ${FRAMEWORKS} ${IGL_LIB}

CPP_FILES=$(wildcard ./*.cpp)
#C_FILES=$(wildcard ./*.c)
OBJ_FILES=$(addprefix obj/,$(notdir $(CPP_FILES:.cpp=.o))) 
#$(addprefix obj/,$(notdir $(C_FILES:.c=.o)))

skinning: obj $(OBJ_FILES)
	g++ -o skinning $(CFLAGS) $(OBJ_FILES) $(LIB)

obj:
	mkdir -p obj

#obj/%.o: %.c %.h
#	gcc $(CFLAGS) -o $@ -c $< $(INC)
obj/%.o: %.cpp %.h
	g++ $(CFLAGS) -o $@ -c $< $(INC)
# Special rule for Ladislav's MT19937.h MT19937F.cpp pair
obj/%.o: %.cpp
	g++ $(CFLAGS) -o $@ -c $< $(INC)

#skinning: main.o Skinning.o SkinningCallback.o arap_dof.o Bone.o WunderSVD3x3.o BoneBoneCopyMap.o  MT19937F.o
#	g++ ${CFLAGS} main.o Bone.o Skinning.o SkinningCallback.o arap_dof.o WunderSVD3x3.o BoneBoneCopyMap.o  MT19937F.o -o skinning ${LIB}
##-rm Skinning.o main.o SkinningCallback.o
#
#main.o: main.cpp Skinning.h CTGALoader.h
#	g++ ${CFLAGS} -c main.cpp -o main.o ${INC}
#
#arap_dof.o: arap_dof.cpp arap_dof.h arap_dof.h arap_rhs.h columnize.h covariance_scatter_matrix.h fit_rotations.h group_sum_matrix.h lbs_matrix.h partition.h polar_dec.h polar_svd.h uncolumnize.h 
#	g++ ${CFLAGS} -c arap_dof.cpp -o arap_dof.o ${INC}
#
#Bone.o: Bone.cpp Bone.h
#	g++ ${CFLAGS} -c Bone.cpp -o Bone.o ${INC}
#
#BoneBoneCopyMap.o: BoneBoneCopyMap.cpp BoneBoneCopyMap.h Animation.h Bone.h
#	g++ ${CFLAGS} -c BoneBoneCopyMap.cpp -o BoneBoneCopyMap.o ${INC}
#
#Skinning.o: Skinning.cpp Skinning.h draw_vbo_mesh.h draw_mesh.h SkinningCallback.h GLSL/lbs.h GLSL/directionalperpixel.h GLSL/directionalperpixelcolor.h GLSL/weightcolors.h GLSL/scalar.h Animation.h lerp.h BoneBoneCopyMap.h
#	g++ ${CFLAGS} -c Skinning.cpp -o Skinning.o ${INC} 
#
#WunderSVD3x3.o: WunderSVD3x3.cpp WunderSVD3x3.h Singular_Value_Decomposition_Preamble.hpp Singular_Value_Decomposition_Kernel_Declarations.hpp Singular_Value_Decomposition_Main_Kernel_Body.hpp
#	g++ ${CFLAGS} -c WunderSVD3x3.cpp -o WunderSVD3x3.o ${INC}
#
#SkinningCallback.o: SkinningCallback.cpp SkinningCallback.h
#	g++ ${CFLAGS} -c SkinningCallback.cpp -o SkinningCallback.o ${INC}
#
#MT19937F.o: MT19937F.cpp MT19937.h
#	g++ ${CFLAGS} -c MT19937F.cpp -o MT19937F.o ${INC}

.PHONY: clean
clean:
	rm -f $(OBJ_FILES)
	rm -f skinning
