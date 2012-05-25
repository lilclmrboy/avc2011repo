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

#define aCHICKEN_UPDATEMS 500

/////////////////////////////////////////////////////////////////////

acpHeadlessChicken::acpHeadlessChicken(acpStem& stem) :
acpStemApp("chicken", stem),
m_pTabs(NULL),
m_stem(stem),
m_bInited(false),
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
//  acpView* v = getContentView();
//  acpRowView* r = new acpRowView(v);
//  acpColumnView* c = new acpColumnView(r);
  
  m_pTabs = createTabControl(getContentView());
  
  // 40 pin module stuff
  acpControl* p40pinmodule = m_pTabs->addTab();
  p40pinmodule->setText("General");

  acpColumnView* c = new acpColumnView(p40pinmodule);

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
  

  
} // stemCreateUI


/////////////////////////////////////////////////////////////////////

bool 
acpHeadlessChicken::stemUIIdle(void)
{
  bool bBusy = false;
  unsigned long now;
  aIO_GetMSTicks(ioRef(), &now, NULL);
  
  if (!m_bInited) {
    
    // THe GP 2.0 is much slower than our code. Let's let it catch up
    m_stem.sleep(500);
        
    // Configure and set any digital pins 
    
    m_bInited = true;
    shrinkWrap();
    
    return bBusy;
  }
  
  if (m_bInited) {
    
    switch (m_pTabs->currentTab()) {
      case aUSBSTEMPANE:
        // see if analog is updated and then advance if so
        if (m_analogs[m_updateIndex]->updated()) {
          do {
            m_updateIndex++;
            m_updateIndex %= a40PINSTEM_NUM_A2D;
          } while (!m_analogs[m_updateIndex]->isEnabled());
          bBusy = true;
        }
        
        // update when we should
        if (m_nextUpdateSystem < now) {
          m_analogs[m_updateIndex]->sendUpdate();
          m_nextUpdateSystem += aCHICKEN_UPDATEMS;
        }
        
        break;
        
      default:
        break;
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


bool 
acpHeadlessChicken::handleSelection(acpView* pControl)
{
  
  // Toggle the show profile plot
  if (pControl == m_pTabs) {
    
    switch (m_pTabs->currentTab()) {        
      default:
        break;
    }
    
    return true;
  }
  
  // otherwise, hand it off to base class
  return acpStemApp::handleValue(pControl);
  
} //  handleValue


// Update the servos 



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
