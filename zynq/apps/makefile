# Makefile empty template to cross-compile apps for petalinux 2021.2

DEFS =  
OPT = -std=c++1z -Ofast -fmax-errors=5 # -w -pthread -flto -lrt -static -funroll-all-loops -pipe -fopenmp -frename-registers
EXCL_FILE =
OUT_OBJ = hello_world
INC =
SRC = 
OBJS_SRC = $(filter-out $(EXCL_FILE), $(wildcard $(SRC)))
COMP = arm-linux-gnueabihf-g++-8
OBJ_LINK = *.o

all: OBJ

OBJ: $(OBJS_SRC)
	reset
	$(COMP) $(OPT) -c $(OBJS_SRC) $(INC) $(DEFS)
	$(COMP) $(OPT) -o $(OUT_OBJ) $(OBJ_LINK) $(INC) $(DEFS)
	rm $(OBJ_LINK)

clean:
	rm *.o *.so

