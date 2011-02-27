///////////////////////////////////////////////////////////////////////////
//
// main.cpp
//
///////////////////////////////////////////////////////////////////////////

#include "controller.h"

int 
main(int argc, 
     const char* argv[]) 
{
		int retVal = 0;
  	try {
  		avcController controller;			
			retVal = controller.init(argc, argv);
			if(!retVal)
				retVal = controller.run();
	
  	} catch (acpException& exception) {
    	printf("exception! %s", exception.msg());
    	//all hell broke loose so return a higher error val.
			return 3000;
  	}

  // We returned successfully from the main process loop.
  // exit gracefully with a bow.
  return retVal;

} // main
