# avc2011/source/aBinary/makefile

.PHONY : all
all : dump2012

.PHONY : dump2011
dump2011 : 
	cp -v ../aUser/aGP2Reflexes.tea ../aSystem/
	mkdir -p ../aObject
	./steep gMonitorGP.tea	
	./steep irSensors.tea
	./steep sonarSensors.tea
	./steep motionUpdate.tea
	./steep locSensors.tea
	./leaf link.leaf
	./makedump avc2011.xml

.PHONY : dump2012
dump2012 : 
	mkdir -p ../aObject
	./makedump avc2012.xml

.PHONY : clean
clean:
	rm -rf ../aObject
	rm -f ../aUser/link.bag
