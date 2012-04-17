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

#define aOVEN_PLOT_UPDATE             1000


/////////////////////////////////////////////////////////////////////

acpHeadlessChicken::acpHeadlessChicken(acpStem& stem) :
acpStemApp("chicken", stem),
m_stem(stem),
m_bInited(false),
//m_bRunPlot(false),
//m_bShowProfile(false),
//m_bShowTemperature(false),
//m_bFinishedProfile(false),
//m_bWaitingForTemperatureReading(false),
m_nextUpdateSystem(0)
//m_nextPlotUpdate(0),
//m_nextTemperatureTime(0),
//m_nextRelayTime(0),
//m_temperatureDesiredIndex(0),
//m_temperatureIndex(0),
//m_ItermIndex(0),
//m_fTemperatureDesired(0.0f),
//m_fTemperatureCurrent(20.0f),
//m_fpid(0.0f),
//m_fLastError(0.0f),
//m_fCurrentError(0.0f),
//m_LastErrorTime(0.0f),
//m_pShowProfile(NULL),
//m_pRunSystem(NULL),
//m_TLength(0)
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
  acpView* v = getContentView();
  acpRowView* r = new acpRowView(v);
  acpColumnView* c = new acpColumnView(r);
  
  m_pUserLED = createCheckboxControl(c, "User LED");
  
} // stemCreateUI


/////////////////////////////////////////////////////////////////////

bool 
acpHeadlessChicken::stemUIIdle(void)
{
  bool bBusy = false;
  
  if (!m_bInited) {
    
    m_bInited = true;
    shrinkWrap();
    
    return bBusy;
  }
  
  return bBusy;
  
} //  stemUIIdle

/////////////////////////////////////////////////////////////////////


bool 
acpHeadlessChicken::handleValue(acpView* pControl)
{
  
  // Toggle the show profile plot
  if (pControl == m_pUserLED) {
    
    m_stem.DIO(a40PINSTEM_MODULE, a40PINSTEM_DIG_USERLED, *m_pUserLED ? true : false);
    
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
