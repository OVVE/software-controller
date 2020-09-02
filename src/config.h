/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

//******************************************************************************
//
// Hardware Settings
//
//******************************************************************************

//==============================================================================
// Unit Settings
//==============================================================================

// Prefines for unit version:
//  Hardware v0: Preproduction units with variety of hardware
//  Hardware v1: Production candidate unit, fixed set of hardware
//#define HARDWARE_V0
#define HARDWARE_V1

//==============================================================================
// Motor Settings
//==============================================================================

// Motor
#if defined(HARDWARE_V0)
#define MOTOR_NANOTEC__ST6018D4508__GP56_T2_26_HR
// #define MOTOR_STEPPERONLINE__23HS30_2804S_HG10
// #define MOTOR_STEPPERONLINE__23HS22_2804S_HG15
// #define MOTOR_STEPPERONLINE__23HS22_2804S_HG20
// #define MOTOR_STEPPERONLINE__23HS22_2804S_HG50

#define MOTOR_ACCELERATION
#elif defined(HARDWARE_V1)
#define MOTOR_NANOTEC__ST6018D4508__GP56_T2_26_HR

#define MOTOR_ACCELERATION
#endif

// Motor Controller
#if defined(HARDWARE_V0)
#define MOTOR_CONTROLLER_NANOTEC__CL4_E_2_12_5VDI
// #define MOTOR_CONTROLLER_STEPPERONLINE_ISD08
// #define MOTOR_CONTROLLER_STEPPERONLINE_DM332T
#elif defined(HARDWARE_V1)
#define MOTOR_CONTROLLER_NANOTEC__CL4_E_2_12_5VDI
#endif

//==============================================================================
// Sensor Settings
//==============================================================================

// Pressure Sensor
// Determine which pressure sensor is installed in the system
#if defined(HARDWARE_V0)
// #define PRESSURE_SENSOR_MPXV7025
// #define PRESSURE_SENSOR_MPXV7007
#define PRESSURE_SENSOR_SSCDRRN100MDAA5
#elif defined(HARDWARE_V1)
#define PRESSURE_SENSOR_SSCDRRN100MDAA5
#endif

// Airflow Sensor
// Determine which airflow sensor is installed in the system
#define AIRFLOW_SENSOR_PMF4103A

//******************************************************************************
//
// Debug Settings
//
//******************************************************************************

// Global log flag
// If commented out, all logging features will be disabled
#define LOG

// Global log backends flags
// Defines level of logging to particular backends
#define LOG_BACKEND_LINK  INFO
#define LOG_BACKEND_DEBUG VERBOSE

// TOOD: Not working right now, come back to this later
// Global plotting flag
// In order to enable the standard Arduino Serial Plotter, all other types of
// messages must be disabled and only a single module can plot a single set
// of variables at one time. As such, to use the DEBUG_PLOT macro, define
// DEBUG_PLOTTING here with the module name (DEBUG_MODULE) to be plotted,
// example commented out below:
// #define DEBUG_PLOTTING "control"

// HAL Logging Flags
// Defines level of logging for that HAL
// TODO: Add more HAL logging if needed
#define LOG_MOTOR_HAL          DEBUG
#define LOG_I2C_HAL            DEBUG
#define LOG_BATTERY_SENSOR_HAL DEBUG

// Module Logging Flags
// Defines level of logging for that module
#define LOG_CONTROL_MODULE    DEBUG
#define LOG_SENSORS_MODULE    DEBUG
#define LOG_LINK_MODULE       DEBUG
#define LOG_PARAMETERS_MODULE DEBUG

// Main Loop Loggign Flags
// Defines level of logging for the main loop
#define LOG_MAIN_LOOP         DEBUG

