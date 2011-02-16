///////////////////////////////////////////////////////////////////////////
//
// main.cpp
//
///////////////////////////////////////////////////////////////////////////

#include "aCmd.tea"
#include "aStem.h"
#include "avc2011Defs.tea"

#define TIMEOUT 10

int 
main(int argc, 
     const char* argv[]) 
{
  try {
    // The stem object encapsulates most of our protocol and link management.
    acpStem stem;
    aIOLib ioRef;
    aErr e;
    acpList<acpString> args;
    
    // Create a setting file to hold our link settings so we can change 
    // the settings without recompiling.
    aSettingFileRef settings;
    if(aIO_GetLibRef(&ioRef, &e)) 
      throw acpException(e, "Getting aIOLib reference");

    if (aSettingFile_Create(ioRef, 
			                      128,
			                      "planner.config",
			                      &settings,
			                      &e))
      throw acpException(e, "creating settings");
    
    aArguments_Separate(ioRef, settings, &args, argc, argv);
    
    // This starts up the stem link processing and is called once to spawn
    // the link management thread... based on the settings.
    stem.startLinkThread(settings);

    // Wait until we have a solid heartbeat connection so we know there is 
    // someone to talk to.
    aInt32 timeout = 0;
    printf("awaiting heartbeat");
    do {
      printf(".");
      fflush(stdout);
      aIO_MSSleep(ioRef, 500, NULL);
      ++timeout;
    } while (!stem.isConnected() && timeout < TIMEOUT);
    printf("done\n");
   
    
    // Now, just loop getting values from the scratchpad.
    printf("Starting Pad Read\n");
    while (1 && timeout < TIMEOUT) {
    
      aShort x, y = 0;
      
      x = stem.PAD_IO(aGP2_MODULE, aSPAD_REPULSIVE_UX);
      x = x << 8;
      x += stem.PAD_IO(aGP2_MODULE, aSPAD_REPULSIVE_UX+1);
      
      y = stem.PAD_IO(aGP2_MODULE, aSPAD_REPULSIVE_UY);
      y = y << 8;
      y += stem.PAD_IO(aGP2_MODULE, aSPAD_REPULSIVE_UY+1);
      
      printf("X,Y IR repulsive components (%d, %d)\n", x, y);
       
	    // sleep a bit so we don't wail on the processor
	    aIO_MSSleep(ioRef, 100, NULL);
    } // while
    
    // Clean up if we ever get out of the endless loop above.
    if (aSettingFile_Destroy(ioRef, settings, &e))
      throw acpException(e, "unable to destroy settings");

  } catch (acpException& exception) {
    printf("exception! %s", exception.msg());
    return 1;
  }

  return 0;

} // main
