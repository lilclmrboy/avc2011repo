/////////////////////////////////////////////////////////////////////////////
// File: avc2011s.h
// This file contain what we need to be consistent across project files
//
#ifndef _avc2011_H_
#define _avc2011_H_

// Acroname specific header files
#include "aStem.h"

// Standard libraries
#include <vector>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

// Project specific headers
#include "avc2011Structs.h"
#include "avc2011Defs.tea"

// Utilites
#ifdef aDEBUG_H
#define aDEBUG_PRINT(arg, ... ) printf(arg, ##__VA_ARGS__);fflush(stdout)
#else 
#define aDEBUG_PRINT(arg)
#endif

// Key values for settings file references
// This takes an integer value (recommended 0-200 range)
#define aKEY_VELOCITY_SETPOINT_MAX "setpoint_max"

// This is the default output file for text based logging
#define aLOGGER_CONFIG "logger.config"

// This is the default output file for text based logging
#define aKEY_LOGGER_FILENAME "logfile"
#define aLOGGER_FILENAME "logger.log"

#endif //_avc2011_H_
