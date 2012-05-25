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
#include "avc2012Defs.tea"
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
#define WHEEL_BASE 0.25f

#define KEY_METER_PER_TICK "meter_per_tick"
#define METER_PER_TICK	0.04358f // 0.47244 meters/rev divided by 10.84 ticks/rev

// degree of latitude per meter at 40 degrees latitude.
#define aLAT_PER_METER 0.00000900620125

// degree of Longitude per meter
#define aLON_PER_METER 0.000011710448481

// Maximum turn angle
#define MAX_TURNANGLE 0.698131701f


#endif //_avc2011_H_
