#include "motModule.h"
#include "avc2011Structs.h"

/////////////////////////////////////////////////////////////////////////////
// IMPORTANT: This procedure must be called with a trailing NULL parameter.
// Example: getVelocity(l,r,vect1,vect2,NULL);
void
avcMotion::getVelocity(int& lV, int& rV, int ct, ... ) {
	
	va_list pArgLst;
	va_start(pArgLst, ct);
	avcForceVector* arg= NULL;
	 
	for (int i = 0; i < ct; ++i) {
	 	arg = va_arg(pArgLst, avcForceVector*);
		if(arg == NULL) return;
		printf("Received Force Vector (x,y): %d, %d \n",
			arg->x, arg->y);
	}
}
