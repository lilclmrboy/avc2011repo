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
#include "acpStemAnalog.h"
#include "acpStemDIO.h"
#include "aMath.h"

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
//  bool m_bRunPlot;
//  bool m_bShowProfile;
//  bool m_bShowTemperature;
//  bool m_bFinishedProfile;
//  bool m_bWaitingForTemperatureReading;
//  float m_fTemperatureDesired;
//  float m_fTemperatureCurrent;
//  float m_fLastError;
//  float m_fCurrentError;
//  float m_fpid;
//  aInt32 m_ItermIndex;
//  unsigned long m_LastErrorTime;
//  aUInt8 m_module;
//  
//  // floating point time and temp points
//  acpList<acpVec2> m_profilePTs;
//  
//  acpString m_profile;
//  
  unsigned long m_nextUpdateSystem;
//  unsigned long m_nextTemperatureTime;
//  unsigned long m_nextRelayTime;
//  unsigned long m_nextPlotUpdate;
//  unsigned int m_temperatureDesiredIndex;
//  unsigned int m_temperatureIndex;
//  acpControlDrawable* m_pGraphView;
  acpControlCheckbox* m_pUserLED;
//  acpControlCheckbox* m_pShowProfile;
//  acpControlCheckbox* m_pRelayOn;
//  acpControlCheckbox* m_pShowTemperature;  
//  acpControlLabel* m_pSystemDiags;
//  acpControlValue* m_pCurrentTemperature;
//  acpControlValue* m_pCurrentError;
//  acpControlValue* m_pTemperatureCalibrate;
  

};
