#include "motModule.h"
#include "avc2011Structs.h"

/////////////////////////////////////////////////////////////////////////////
// IMPORTANT: This procedure must be called with a trailing NULL parameter.
// Example: getVelocity(l,r,vect1,vect2,NULL);
void
avcMotion::getVelocity(int& lV, int& rV, ... ) {
	va_list pArgLst;
	va_start(pArgLst, rV);
	avcForceVector* arg= va_arg( pArgLst, avcForceVector*);
	 
	while (arg != NULL) {
	 	printf("Received Force Vector (x,y): %d, %d \n",
			arg->x, arg->y);
		arg = va_arg(pArgLst, avcForceVector*);	
	}
}