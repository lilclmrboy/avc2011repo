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
  	try {
  		avcController controller;
		controller.init(argc, argv);
		controller.run();

  	} catch (acpException& exception) {
    		printf("exception! %s", exception.msg());
    		//all hell broke loose so return a higher error val.
		return 3;
  	}
  // We returned successfully from the main process loop.
  // exit gracefully with a bow.
  return 0;

} // main
