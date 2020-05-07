
//******************************************************************************
//
// Hardware Settings
//
//******************************************************************************

//==============================================================================
// Motor Settings
//==============================================================================

// Motor
// Determine which motor type and gearing is installed in the system
// #define MOTOR_23HS22_2804S_HG50_50_1
#define MOTOR_23HS22_2804S_HG50_20_1
// #define MOTOR_23HS22_2804S_HG50_15_1
// #define MOTOR_23HS30_2804S_HG10_10_1

// Motor Controller
// Determine which motor controller is installed in the system
#define MOTOR_CONTROLLER_ISD08
// #define MOTOR_CONTROLLER_DM332T

//==============================================================================
// Sensor Settings
//==============================================================================

// Pressure Sensor
// Determine which pressure sensor is installed in the system
// #define PRESSURE_SENSOR_MPXV7025
// #define PRESSURE_SENSOR_MPXV7007
#define PRESSURE_SENSOR_SSCDRRN100MDAA5

// Airflow Sensor
// Determine which airflow sensor is installed in the system
#define AIRFLOW_SENSOR_PMF4103A

//******************************************************************************
//
// Debug Settings
//
//******************************************************************************

// Global debug flag
// If commented out, all debug features will be disabled
#define DEBUG

// Global plotting flag
// In order to enable the standard Arduino Serial Plotter, all other types of
// messages must be disabled and only a single module can plot a single set
// of variables at one time. As such, to use the DEBUG_PLOT macro, define
// DEBUG_PLOTTING here with the module name (DEBUG_MODULE) to be plotted,
// example commented out below:
// #define DEBUG_PLOTTING "control"

// Global debug port
// Defines which serial port all debug messages/plotting will be sent, default
// is Serial, the USB Serial port
// #define DEBUG_SERIAL_PORT Serial

// HAL Debug Flags
// If commented out, debug features from that HAL will be disabled
// TODO: Add more HAL debug features if needed
#define DEBUG_MOTOR_HAL

// Module Debug Flags
// If commented out, debug features from that module will be disabled
#define DEBUG_CONTROL_MODULE
#define DEBUG_SENSORS_MODULE
#define DEBUG_LINK_MODULE
#define DEBUG_PARAMETERS_MODULE

// Main Loop Debug Flags
// If commented out, debug features from that module will be disabled
#define DEBUG_MAIN_LOOP
#define DEBUG_MAIN_LOOP_METRICS