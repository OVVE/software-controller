
//******************************************************************************
//
// Hardware Settings
//
//******************************************************************************

//==============================================================================
// Motor Settings
//==============================================================================

// Motor
#define MOTOR_NANOTEC__ST6018D4508__GP56_T2_26_HR
// #define MOTOR_STEPPERONLINE__23HS30_2804S_HG10
// #define MOTOR_STEPPERONLINE__23HS22_2804S_HG15
// #define MOTOR_STEPPERONLINE__23HS22_2804S_HG20
// #define MOTOR_STEPPERONLINE__23HS22_2804S_HG50

#define MOTOR_ACCELERATION

// Motor Controller
#define MOTOR_CONTROLLER_NANOTEC__CL4_E_2_12_5VDI
// #define MOTOR_CONTROLLER_STEPPERONLINE_ISD08
// #define MOTOR_CONTROLLER_STEPPERONLINE_DM332T

//==============================================================================
// Sensor Settings
//==============================================================================

// Pressure Sensor
// Determine which pressure sensor is installed in the system
// #define PRESSURE_SENSOR_MPXV7025
// #define PRESSURE_SENSOR_MPXV7007
#define PRESSURE_SENSOR_SSCDRRN100MDAA5

//calibrate the bias of the pressure sensor at startup. Requires the user to make sure that the sensor sees environmental pressure at startup as well.
// #define PRESSURE_SENSOR_CALIBRATION_AT_STARTUP

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
#define LOG_MOTOR_HAL         DEBUG

// Module Logging Flags
// Defines level of logging for that module
#define LOG_CONTROL_MODULE    DEBUG
#define LOG_SENSORS_MODULE    DEBUG
#define LOG_LINK_MODULE       DEBUG
#define LOG_PARAMETERS_MODULE DEBUG

// Main Loop Loggign Flags
// Defines level of logging for the main loop
#define LOG_MAIN_LOOP         DEBUG

