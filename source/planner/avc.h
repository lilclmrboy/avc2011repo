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
#include "avcStructs.h"
#include "avcDefs.tea"
#include "logger.h"

// Utilites
#ifdef aDEBUG_H
#define aDEBUG_PRINT(arg, ... ) printf(arg, ##__VA_ARGS__);fflush(stdout)
#else 
#define aDEBUG_PRINT(arg)
#endif

// Key values for settings file references
// This takes an integer value (recommended 0-200 range)
#define aKEY_VELOCITY_SETPOINT_MAX "setpoint_max"
#define aKEY_REPULSE_WEIGHT "repulse_weight" //float
#define aKEY_NORMALIZE_MOTIVATION_VECTOR "normalize_motivation" //flag
#define aKEY_MAX_UNPASSED_DISTANCE "maxUnPassedDistanceToWaypoint" //float meters
#define aKEY_MIN_UNPASSED_DISTANCE "minUnPassedDistanceToWaypoint" //float meters
#define aKEY_UNPASSED_SLICE_ZETA "unpassedZeta" //float degrees
#define aKEY_TRACKFILE "mapfile"


// This is the default output file for text based logging
#define aLOGGER_CONFIG "logger.config"

// This is the default output file for text based logging
#define aKEY_LOGGER_FILENAME "logfile"
#define aLOGGER_FILENAME "logger.log"

// Physical dimension keys, and default values.
//#define aKEY_WHEEL_RADIUS "wheel_radius"
//#define aWHEEL_RADIUS 0.0762f /* radius of wheel in meters (6 inches) */

//#define aKEY_WHEEL_TRACK "wheel_track"
//#define aWHEEL_TRACK 0.162f /* distance in meters between contact point of wheel. */

#define aKEY_WHEEL_BASE "wheel_base"
#define WHEEL_BASE 0.3302f

#define KEY_METER_PER_TICK "meter_per_tick"
#define METER_PER_TICK	0.04358f // 0.47244 meters/rev divided by 10.84 ticks/rev

//cubic curve for servo setpoints > 128
// pconst + x * pa1 + x^2 * pa2 + x^3 * pa3
#define PCONST -108.9563
#define PA1	1.3637
#define PA2	-5.4696e-3
#define PA3 8.1417e-6


//cubic curve for serov sepoints < 128
// nconst + x * na1 + x^2 * na2 + x^3 * n a3
#define NCONST 19.7800
#define NA1	-0.1004
#define NA2	-1.0727e-3
#define NA3 9.4986e-6


// Maximum turn angle
#define MAX_TURNANGLE 0.3168111

// degree of latitude per meter at 40 degrees latitude.
#define aLAT_PER_METER 0.00000900620125

// degree of Longitude per meter
#define aLON_PER_METER 0.000011710448481

// Main controller loop cycle delay
#define aCONTROLLER_LOOP_DELAY_KEY     "loopdelay"
#define aCONTROLLER_LOOP_DELAY_DEFAULT 100

// Main controller input voltage minimum control value
#define aCONTROLLER_INPUTV_CONTROLLER_KEY     "minInputVoltageController"
#define aCONTROLLER_INPUTV_CONTROLLER_DEFAULT 12.0f

/////////////////////////////////////////////////////////////////////////////
// We may want to move this out somewhere or into it's own seperate class
// so other modules can use it
int PlaySound(const char * file);

#endif //_avc2011_H_
