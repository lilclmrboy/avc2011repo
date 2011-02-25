# avc2011/source/aBinary/makefile

.PHONY : all
all : dump

.PHONY : dump
dump : 
	./steep gMonitorGP.tea	
	./steep irSensors.tea
	./steep sonarSensors.tea
	./steep motionUpdate.tea
	./makedump avc2011.xml

.PHONY : clean
clean:
	rm ../aObject/gMonitorGP.cup
	rm ../aObject/irSensors.cup
	rm ../aObject/sonarSensors.cup
	rm ../aObject/motionUpdate.cup
	rm ../aObject/avc2011.dump
