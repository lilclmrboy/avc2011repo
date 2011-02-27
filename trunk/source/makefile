# whole project makefile

OS := (shell uname)
ifeq ($(OS), Darwin)
TAROS = mac
else
TAROS = linux
endif

.PHONY : all
all : MotionPlanner TEAWare

.PHONY : acroname
acroname :
	tar -xvmf acroname_$(TAROS).tgz

.PHONY : TEAWare
TEAWare :
	make -C acroname/aBinary
	
.PHONY : MotionPlanner
MotionPlanner :
	make -C planner	

.PHONY : clean
clean :
	make -C acroname/aBinary clean
	make -C planner clean

