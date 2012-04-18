/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpHeadlessChickenApp.cpp   	                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Implementation of Acroname aHeadlessChickenApp     //
//              application.                                       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// Copyright 1994-2012. Acroname Inc.                              //
//                                                                 //
// This software is the property of Acroname Inc.  Any             //
// distribution, sale, transmission, or re-use of this code is     //
// strictly forbidden except with permission from Acroname Inc.    //
//                                                                 //
// To the full extent allowed by law, Acroname Inc. also excludes  //
// for itself and its suppliers any liability, wheither based in   //
// contract or tort (including negligence), for direct,            //
// incidental, consequential, indirect, special, or punitive       //
// damages of any kind, or for loss of revenue or profits, loss of //
// business, loss of information or data, or other financial loss  //
// arising out of or in connection with this software, even if     //
// Acroname Inc. has been advised of the possibility of such       //
// damages.                                                        //
//                                                                 //
// Acroname Inc.                                                   //
// www.acroname.com                                                //
// 720-564-0373                                                    //
//                                                                 //
/////////////////////////////////////////////////////////////////////

#include "a40PinStem.h"
#include "aVersion.h"
#include <math.h>
#include "acpHeadlessChickenApp.h"

/////////////////////////////////////////////////////////////////////

acpHeadlessChicken::acpHeadlessChicken(acpStem& cstem) :
acpStemApp("chicken", cstem),
m_stem(cstem),
m_bInited(false),
m_bGP2Present(false),
m_nextUpdateSystem(0),
m_bAwaitingUpdate(false),
m_updateIndex(0)
{
}


/////////////////////////////////////////////////////////////////////

acpHeadlessChicken::~acpHeadlessChicken(void)
{
}

/////////////////////////////////////////////////////////////////////

void 
acpHeadlessChicken::stemCreateUI(void)
{
  acpString name;
  int i = 0;
  acpView* v = getContentView();
  acpRowView* r = new acpRowView(v);
  acpColumnView* c = new acpColumnView(r);
  
  createLabelControl(c, acpControlLabel::kHeading, "USBStem 1.0 Controls");
  
  m_pUserLED = createCheckboxControl(c, "User LED");
  
  // Show general analog controls that are tied to accel 3 axis
  for (i = 0; i < a40PINSTEM_NUM_A2D; i++) {
    m_analogs[i] = new acpStemA2D(this, c, a40PINSTEM_MODULE, i);
    m_analogs[i]->setEnable(true);
    m_analogs[i]->setMinWidth(200);
    m_analogs[i]->setScaling(3.3f);
    m_analogs[i]->setUnits("V");
  }
  
  createLabelControl(c, acpControlLabel::kHeading, "GP 2.0 Controls");
  m_pGPStatus = createLabelControl(c, acpControlLabel::kBody, "---");
  
  // Show servo controls
  for (i = 0; i < aGP_NUMSERVOS_USED; i++) {
    m_pServo[i] = new acpStemServo(this, c, aGP_MODULE, i);
    m_pServo[i]->setEnable(false);
  }
  
} // stemCreateUI


/////////////////////////////////////////////////////////////////////

#define aCHICKEN_UPDATEMS 1000

bool 
acpHeadlessChicken::stemUIIdle(void)
{
  bool bBusy = false;
  unsigned long now;
  aIO_GetMSTicks(ioRef(), &now, NULL);
  
  if (!m_bInited) {
    
    // Check that a GP is present by sending a debug packet
    m_stem.sleep(500);
    
    aUInt8 data[2] = {33,44};
    if (m_stem.DEBUG(aGP_MODULE, data, 2)) {
      m_bGP2Present = true;
      
      for (int i = 0; i < aGP_NUMSERVOS_USED; i++)
        m_pServo[i]->setEnable(true);
      
      // Set the GUI status
      m_pGPStatus->setText("Connected");
    }
    else {
      // Set the GUI status
      m_pGPStatus->setText("Not responding. Controls Disabled");
    }
    
    // Configure and set any digital pins 
    
    m_bInited = true;
    shrinkWrap();
    
    return bBusy;
  }
  
  if (m_bInited && !bBusy) {
    
    // see if analog is updated and then advance if so
    if (m_analogs[m_updateIndex]->updated()) {
      do {
        m_updateIndex++;
        m_updateIndex %= a40PINSTEM_NUM_A2D;
      } while (!m_analogs[m_updateIndex]->isEnabled());
      bBusy = true;
    }
    
    // Update the servos 
    if (m_bGP2Present && !bBusy) {
      for (int i = 0; i < aGP_NUMSERVOS_USED; i++) {
        m_pServo[i]->refresh();
      }
    }
    
    // update when we should
    if (m_nextUpdateSystem < now) {
      m_analogs[m_updateIndex]->sendUpdate();
      m_nextUpdateSystem += aCHICKEN_UPDATEMS;
    }
    
  }
  
  return bBusy;
  
} //  stemUIIdle

/////////////////////////////////////////////////////////////////////


bool 
acpHeadlessChicken::handleValue(acpView* pControl)
{
  
  // Toggle the show profile plot
  if (pControl == m_pUserLED) {
    
    m_stem.DIO(a40PINSTEM_MODULE, 
               a40PINSTEM_DIG_USERLED, 
               *m_pUserLED ? true : false);
    
    return true;
  }
    
  // otherwise, hand it off to base class
  return acpStemApp::handleValue(pControl);
  
} //  handleValue


/////////////////////////////////////////////////////////////////////

int 
main(const int argc, 
     const char* argv[])
{
  int retVal = 0;
  
  try {
    acpStem stem;
    acpHeadlessChicken app(stem);
    
    try {
      if (app.acpStemApp::init(argc, argv))
        retVal = app.runApp();
    } catch (acpException& exception) {
      acpString msg("Error: ");
      msg += exception.msg();
      app.message(msg);
      throw exception;
    }
  } catch (acpException& exception) {
    printf("Exception: %s\n", exception.msg());
    retVal = exception.error();
  } catch (...) {
    printf("Unknown exception caught!");
  }
  
  retVal += aLeakCheckCleanup();
  
  return(retVal);
  
} // main


/////////////////////////////////////////////////////////////////////
// wrapper since windows entry is WinMain.  Just chains to main.

#ifdef aWIN
#include <windows.h>
int 
WINAPI WinMain(HINSTANCE hInst,
               HINSTANCE hPreInst, 
               LPSTR lpszCmdLine, 
               int nCmdShow)
{
  return(main(0, NULL));
}
#endif // aWIN
