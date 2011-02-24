#include "motModule.h"

avcControlUpdate 
avcMotion::updateControl(const avcForceVector& potential) {

	printf("Received Force Vector (x,y): %d, %d \n",
		potential.x, potential.y);
}
