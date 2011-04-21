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
    switch (retVal) {
      case aErrTimeout:
	printf("Couldn't connect to stem. Check connections, jackass.\n");
	PlaySound("crap.wav");
	return 0;
	break;
      default:
	break;
    }
    if(!retVal)
      retVal = controller.run();
    
  } catch (acpException& exception) {
    printf("exception! %s", exception.msg());
    //all hell broke loose so return a higher error val.
    return 3000;
  }
  
  // We returned successfully from the main process loop.
  // exit gracefully with a bow.
  return 0;
  
} // main
