# examples/aGarciaKey makefile
#
# This builds the aGarciaKey demonstration program and places it in the
# ../../acroname/aBinary directory.  This will only build if the examples
# directory this file lives in is a sibling of the acroname Garcia download.
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
#   4. We link against the pthread, X11, and OpenGL libraries as well as 
#      4 in the acroname download (aUtil, aIO, aStem, and aUI).

APP = aGarciaKey
CC = g++
CFLAGS = -DaUNIX \
	 -I../../acroname/aInclude \
	 -I../../acroname/aSource \
	 -I../../acroname/aSystem
LFLAGS = -Wl,-rpath,.
SRC = ../../acroname/aSource
BIN = ../../acroname/aBinary
LIBS = -lpthread -lcrypto -lssl -lX11 -lGL -lGLU \
       -L../../acroname/aBinary -laUtil -laIO -laMath -laStem -laUI

OBJ = unix_$(APP)_Data

.PHONY : app
app : $(OBJ) $(BIN)/$(APP)

$(OBJ) : 
	mkdir $(OBJ)

$(BIN)/MotionPlanner : $(OBJ)/MotionPlanner.o
	$(CC) -g $(CFLAGS) $(LFLAGS) $^ $(LIBS) -o $@

$(OBJ)/MotionPlanner.o : MotionPlanner.cpp MotionPlanner.h
	$(CC) -c $(CFLAGS) $< -o $@

#$(OBJ)/acpGarcia.o : $(SRC)/acpGarcia.cpp $(SRC)/acpGarcia.h
#	$(CC) -c $(CFLAGS) $< -o $@

.PHONY : clean
clean :
	rm -rf $(OBJ)
	rm -f $(BIN)/$(APP)
