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


PREFIX = ../
APP = MotionPlanner
CC = g++
CFLAGS = -DaUNIX -DaDEBUG_H\
	 -I$(PREFIX)acroname/aInclude \
	 -I$(PREFIX)acroname/aSource \
	 -I$(PREFIX)acroname/aSystem \
	 -I$(PREFIX)acroname/aUser
CFLAGS += -m32
LFLAGS = -Wl,-rpath,.
SRC = $(PREFIX)acroname/aSource
BIN = $(PREFIX)acroname/aBinary
LIBS = -lpthread -lcrypto -lssl \
       -L/lib \
       -L$(PREFIX)acroname/aBinary -laUtil -laIO -laMath -laStem

OBJ = unix_$(APP)_Data

OBJECTS = $(OBJ)/controller.o \
	  $(OBJ)/planner.o \
	  $(OBJ)/motModule.o 



.PHONY : app
app : $(OBJ) $(BIN)/$(APP)

$(OBJ) : 
	mkdir $(OBJ)

$(BIN)/MotionPlanner : $(OBJ)/MotionPlanner.o $(OBJECTS)
	$(CC) -g $(CFLAGS) $(LFLAGS) $^ $(LIBS) -o $@

$(OBJ)/MotionPlanner.o : main.cpp
	$(CC) -c $(CFLAGS) $< -o $@

$(OBJ)/%.o : %.cpp %.h
	$(CC) -c $(CFLAGS) $< -o $@


.PHONY : clean
clean :
	rm -rf $(OBJ)
	rm -f $(BIN)/$(APP)
