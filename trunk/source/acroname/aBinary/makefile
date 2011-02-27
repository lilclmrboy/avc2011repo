# avc2011/source/aBinary/makefile

.PHONY : all
all : dump

.PHONY : dump
dump : 
	mkdir -p ../aObject
	./steep gMonitorGP.tea	
	./steep irSensors.tea
	./steep sonarSensors.tea
	./steep motionUpdate.tea
	./steep locSensors.tea
	./makedump avc2011.xml

.PHONY : clean
clean:
	rm -rf ../aObject
