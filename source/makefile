# whole project makefile

.PHONY : all
all : MotionPlanner TEAWare

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

