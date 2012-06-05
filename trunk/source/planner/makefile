# This builds the MotionPlanner program and places it in the
# ../../acroname/aBinary directory.  This will only build if the planner
# directory this file lives in is a sibling of the acroname download containing
# among other things the aInclude, aBinary, an aUser directory. This is the host
# module for an autonomous robotic platform which includes Tea files and 
# definitions.
#
# When we set up the build environment below, some things warrant description:
#
#   1. We specify a global #define of aUNIX in the CFLAGS to identify 
#      That we are building for unix.
#
#   2. We include or compile additional files that are in both the 
#      acroname includes and sources.
#
#   3. We specify the rpath linker search path of ".".  This allows the
#      shared libraries we link against to be found when this executable
#      is co-located.  This avoids the need for system install or setting
#      an LD_LIBRARY_PATH variable, but both of these alternate approaches
#      will still work.
#
#   4. We link against the crypto an ssl libraries as well as 
#      4 in the acroname download (aUtil, aIO, aStem, and aUI).


OS := $(shell uname)
PREFIX = ../
APP = MotionPlanner
CC = g++
CFLAGS = -DaUNIX
ifeq ($(OS), Darwin)
CFLAGS += -DaMACX
endif
CFLAGS += -DaDEBUG_H\
	 -I$(PREFIX)acroname/aInclude \
	 -I$(PREFIX)acroname/aSource \
	 -I$(PREFIX)acroname/aSystem \
	 -I$(PREFIX)acroname/aUser
CFLAGS += -m32 -Wall -Werror
CFLAGS += -Woverloaded-virtual
LFLAGS = -Wl,-rpath,.
LFLAGS += -Wl,-rpath,@loader_path/. 
LFLAGS += -Wl,-rpath,@loader_path/../cgi-bin/ 
LFLAGS += -Wl,-rpath,@loader_path/../$(ACRONAME)/aBinary
SRC = $(PREFIX)acroname/aSource
BIN = $(PREFIX)acroname/aBinary
LIBS = -lpthread -lcrypto -lssl

ifeq ($(OS), Darwin)
FRAMEWORKS = -F$(PREFIX)acroname/aBinary \
			 -framework aUtil \
			 -framework aIO \
			 -framework aMath \
			 -framework aStem
else
LIBS += -L/lib -L/usr/lib
LIBS += -L$(PREFIX)acroname/aBinary -laUtil -laIO -laMath -laStem
endif 

ifeq ($(OS), Darwin)
OBJ = macx_$(APP)_Data
else
OBJ = unix_$(APP)_Data
endif

OBJECTS = \
			$(OBJ)/planner.o \
			$(OBJ)/controller.o \
			$(OBJ)/motModule.o \
			$(OBJ)/logger.o \
			$(OBJ)/locModule.o \
			$(OBJ)/mat_math.o \
			$(OBJ)/repulsiveForces.o\
			$(OBJ)/gps.o\
      $(OBJ)/gyroL3G4200D.o\
      $(OBJ)/gyro.o

.PHONY : app
app : $(OBJ) $(BIN)/$(APP)

$(OBJ) : 
	mkdir $(OBJ)

$(BIN)/MotionPlanner : $(OBJ)/MotionPlanner.o $(OBJECTS)
	$(CC) -g $(CFLAGS) $(LFLAGS) $^ $(FRAMEWORKS) -o $@ $(LIBS)

$(OBJ)/MotionPlanner.o : main.cpp
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ)/%.o : %.cpp %.h
	$(CC) -c $(CFLAGS) $< -o $@
	
.PHONY : motModuleTest
motModuleTest : $(OBJ)
	$(CC) -c $(CFLAGS) logger.cpp -o $(OBJ)/logger.o
	$(CC) -c $(CFLAGS) -DaDEBUG_MOTMODULE motModule.cpp -o $(OBJ)/motModule.o
	$(CC) -g $(CFLAGS) -DaDEBUG_MOTMODULE $(LFLAGS) $(OBJ)/motModule.o $(OBJ)/logger.o $(LIBS) $(FRAMEWORKS) -o $(BIN)/motModuleTest
  
.PHONY : repulsiveForceTest
repulsiveForceTest : $(OBJ)
	$(CC) -c $(CFLAGS) logger.cpp -o $(OBJ)/logger.o
	$(CC) -c $(CFLAGS) -DaDEBUG_FREPULSIVE repulsiveForces.cpp -o $(OBJ)/repulsiveForces.o
	$(CC) -c $(CFLAGS) motModule.cpp -o $(OBJ)/motModule.o
	$(CC) -g $(CFLAGS) -DaDEBUG_FREPULSIVE $(LFLAGS) $(OBJ)/motModule.o $(OBJ)/repulsiveForces.o $(OBJ)/logger.o $(LIBS) $(FRAMEWORKS) -o $(BIN)/repulsiveForceTest

.PHONY : posTest
posTest : $(OBJ)
	$(CC) -c $(CFLAGS) logger.cpp -o $(OBJ)/logger.o
	$(CC) -c $(CFLAGS) mat_math.cpp -o $(OBJ)/mat_math.o
	$(CC) -c $(CFLAGS) gps.cpp -o $(OBJ)/gps.o
	$(CC) -c $(CFLAGS) -DaDEBUG_FLOC locModule.cpp -o $(OBJ)/locModule.o
	$(CC) -g $(CFLAGS) -DaDEBUG_FLOC $(LFLAGS) $(OBJ)/mat_math.o $(OBJ)/locModule.o $(OBJ)/logger.o $(OBJ)/gps.o $(LIBS) $(FRAMEWORKS) -o $(BIN)/posTest 

.PHONY : loggerTest
loggerTest : $(OBJ)
	$(CC) -c $(CFLAGS) -DaDEBUG_LOGGER logger.cpp -o $(OBJ)/logger.o
	$(CC) -g $(CFLAGS) -DaDEBUG_LOGGER $(LFLAGS) $(OBJ)/logger.o $(LIBS) $(FRAMEWORKS) -o $(BIN)/loggerTest

.PHONY : MatrixTest
MatrixTest : $(OBJ)/mat_math.o
	$(CC) -c $(CFLAGS) matTest.cpp -o $(OBJ)/matTest.o
	$(CC) -g $(CFLAGS)  $(LFLAGS) $(OBJ)/matTest.o $(OBJ)/mat_math.o $(LIBS) $(FRAMEWORKS) -o $(BIN)/MatrixTest

.PHONY : plannerTest
plannerTest : $(OBJ) $(OBJ)/logger.o
	$(CC) -c $(CFLAGS) -DaDEBUG_PLANNER planner.cpp -o $(OBJ)/planner.o 
	$(CC) -g $(CFLAGS) -DaDEBUG_PLANNER $(LFLAGS) $(OBJ)/logger.o $(OBJ)/planner.o $(LIBS) $(FRAMEWORKS) -o $(BIN)/plannerTest

.PHONY : gyroTest
gyroTest : $(OBJ) $(OBJ)/logger.o
	$(CC) -c $(CFLAGS) gyroL3G4200D.cpp -o $(OBJ)/gyroL3G4200D.o
	$(CC) -c $(CFLAGS) -DaDEBUG_GYRO gyro.cpp -o $(OBJ)/gyro.o
	$(CC) -g $(CFLAGS) -DaDEBUG_GYRO $(LFLAGS) $(OBJ)/logger.o $(OBJ)/gyroL3G4200D.o $(OBJ)/gyro.o $(LIBS) $(FRAMEWORKS) -o $(BIN)/gyroTest


.PHONY : clean
clean :
	rm -rf $(OBJ)
	rm -f $(BIN)/$(APP)
	rm -f $(BIN)/motModuleTest
	rm -f $(BIN)/loggerTest
	rm -f $(BIN)/MatrixTest	
	rm -f $(BIN)/plannerTest	
	rm -f $(BIN)/repulsiveForceTest

