# whole project makefile

OS := $(shell uname)
EXTRACTION = tar -xvmf
EXTRACTIONEXT = tgz

ifeq ($(OS), Darwin)
TAROS = mac
else
ifeq ($(OS), Linux)
TAROS = linux
else
TAROS = win
EXTRACTION = zip
EXTRACTIONEXT = zip
endif
endif

.PHONY : all
all : MotionPlanner TEAWare

.PHONY : acroname
acroname :
	$(EXTRACTION) acroname_$(TAROS).$(EXTRACTIONEXT)

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

.PHONY : acroclean
acroclean : 
	tar -tf acroname_$(TAROS).tgz | find -type f -exec ls {}
	#rm -Rfv acroname/aDocumentation
	#rm -rdv acroname/aBinary/aIO.framework/
	#rm -rfv acroname/aBinary/aMoto/
	#rm -rfv acroname/aBinary/aSymonym/
	
	