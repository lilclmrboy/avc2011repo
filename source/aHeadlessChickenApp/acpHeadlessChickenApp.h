/////////////////////////////////////////////////////////////////////
//                                                                 //
// file: acpRFiCB2App.h   	       	                           //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
// description: Definition of the Acroname aHeadlessChicken exampl //
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

#include "acpStemApp.h"
#include "acpStemA2D.h"
#include "acpStemDIO.h"
#include "acpStemServo.h"
#include "aMath.h"
#include "avc2011.h"

class acpHeadlessChicken : public acpStemApp 
{
public:
  acpHeadlessChicken(acpStem& stem);
  virtual ~acpHeadlessChicken(void);
    
  // basic stem app stuff
  void stemCreateUI(void);
  bool stemUIIdle(void);
  
  // general app event handling
  bool handleValue(acpView* pControl);

  acpStem& m_stem;

private:
  
  bool m_bInited;

  unsigned long m_nextUpdateSystem;

  acpControlCheckbox* m_pUserLED;
  acpStemA2D* m_analogs[a40PINSTEM_NUM_A2D];
  acpStemDIO* m_digitals[a40PINSTEM_NUM_DIG];
  acpStemServo* m_pServo[aGP_NUMSERVOS_USED];
  
  acpStemSuperControl* m_pCurrentUpdate;
  

};
