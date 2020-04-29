#include "../hal/hal.h"
#include "../hal/motor.h"

#include <assert.h>
#include <util/atomic.h>

#include "Arduino.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#define DEBUG
#define DEBUG_MODULE "motor"
#include "../util/debug.h"

//**************************************
// Pin Definitions
//**************************************

// Motor Pins
#define PIN_MOTOR_STEP      7 // Each pulse moves the motor by a (micro)step.
#define PIN_MOTOR_ENABLE    8 // Enable locks the motor in place when it is not moving, which consumes power.
#define PIN_MOTOR_DIRECTION 9

#define PIN_MOTOR_ENABLE_FALSE LOW
#define PIN_MOTOR_ENABLE_TRUE  HIGH

#define PIN_MOTOR_DIRECTION_OPEN  LOW
#define PIN_MOTOR_DIRECTION_CLOSE HIGH

// Limit Switch Pins
// Connect C (common) to ground and NC (normally closed) to the specified pin
// (pull-up mode).
#define PIN_LIMIT_BOTTOM 5
#define PIN_LIMIT_TOP    6

#define PIN_LIMIT_TRIPPED     HIGH
#define PIN_LIMIT_NOT_TRIPPED LOW

//**************************************
// Motor Definitions
//**************************************

#define MOTOR_STEPS_PER_REVOLUTION 200

// TODO: Need to define step angle in terms of microsteps per revolution.
// #define MOTOR_MICROSTEPS_PER_REVOLUTION

// millidegrees of shaft rotation (including the gearbox) per motor step

// stepperonline 23HS22-2804S-HG50
// NEMA 23 50:1
// #define MOTOR_STEP_ANGLE 36

// stepperonline 
// NEMA 23 20:1
// #define MOTOR_STEP_ANGLE 90

// stepperonline 23HS22-2804S-HG50
// NEMA 23 15:1
#define MOTOR_STEP_ANGLE 120

// stepperonline 23HS30-2804S-HG10
// NEMA 23 10:1
// #define MOTOR_STEP_ANGLE 180


//**************************************
// Motor Controller Definitions
//**************************************

// stepperonline ISD08
// Microstep Resolution
// The ISD08 supports the following settings:
//  100 (whole steps)
//  200 (half steps)
//  400 (quarter steps)
//  800 (eigth steps)
// 1600 (sixteenth steps)
#define MC_MICROSTEP_RESOLUTION 400
#define MC_MICROSTEPS_PER_STEP (MC_MICROSTEP_RESOLUTION/100)

// stepperonline DM332T
// Pulse/rev
// The DM332T supports the following settings:
//  400 (half steps)
//  800 (quarter steps)
// 1600 (eigth steps)
// 3200 (sixteenth steps)
// 4000
// 6400
// 8000
// 12800
// #define MC_MICROSTEPS_PER_REVOLUTION 800
// #define MC_MICROSTEPS_PER_STEP (MC_MICROSTEPS_PER_REVOLUTION/MOTOR_STEPS_PER_REVOLUTION)

// The amount of time (in microseconds) that the direction signal must be
// stable before a pulse on the PWM signal.
#define MC_DIRECTION_SETUP_TIME 35

// The duration of each pulse (in microseconds) of the PWM signal.
#define MC_PULSE_DURATION 16 

// TODO: detect potential overflow in 16-bit motor_position variable

//**************************************
// Motor State Machine Definitions
//**************************************
#define MOTOR_STATE_OFF   0
#define MOTOR_STATE_HOLD  1
#define MOTOR_STATE_OPEN  2
#define MOTOR_STATE_CLOSE 3
#define MOTOR_STATE_NONE  UINT8_MAX

//**************************************
// Motor Control Definitions
//**************************************

// TODO: Update this based on maximum PWM frequency and control loop frequency.
#define MOTOR_CONTROL_STEP_ERROR 5

#define MOTOR_POSITION_INCREMENT_OPEN  false
#define MOTOR_POSITION_INCREMENT_CLOSE true

//**************************************
// Motor Control Variables
//**************************************
static volatile struct {
  // Precomputed value so that we can minimize the amount of math operations
  // inside of an ISR.
  bool     counter_update;
  uint16_t counter_TOP;

  int16_t  step_position;
} motor_control;

// Public
volatile int16_t motor_position; // Motor position in microsteps from zero.

// Private

// TODO
// It's okay to read this variable outside of a critical section, as long as we
// assume that it can never overflow.
static volatile bool motor_position_increment;

static int16_t motor_position_command;

//**************************************
// Motor State Machine Variables
//**************************************
static uint8_t motor_state = MOTOR_STATE_NONE;
static volatile uint8_t motor_state_pending = MOTOR_STATE_NONE;

//*****************************************************************************
// Timer 4 Configuration: Motor Movement and Position Tracking
//
// 
// Configure Timer 4 to generate a PWM output to the motor controller causing
// it to move one (micro)step per pulse. The PWM output is enabled and disabled
// dynamically, so there are separate *_enabled and *_disabled values for
// TCCR4B. Configure an interrupt to occur for every PWM pulse to track the
// motor position.
// 
// WGM4[3:0] = 0b1001 (PWM, Phase and Frequency Correct Mode, TOP: OCR4A)
// CS3[2:0] = 0b010 (clk/8 prescaler)
//
//*****************************************************************************

// COM4B[1:0] = 0b10 (Enable non-inverting PWM output on OC4B -> ATMega 2560 PH4 -> Arduino D7)
static uint8_t TCCR4A_value = (1<<COM4B1) | (0<<COM4B0) | (0<<WGM41) | (1<<WGM40); // pulse high

// COM4B[1:0] = 0b11 (Enable inverting PWM output on OC4B -> ATMega 2560 PH4 -> Arduino D7)
// static uint8_t TCCR4A_value = (1<<COM4B1) | (1<<COM4B0) | (0<<WGM41) | (1<<WGM40); // pulse low

static uint8_t TCCR4B_value_disabled = (1<<WGM43) | (0<<WGM42);
static uint8_t TCCR4B_value_enabled = TCCR4B_value_disabled | (0<<CS42) | (1<<CS41) | (0<<CS40);

// TODO Keep this forward declaration?
static void motor_state_register_pending(uint8_t next_state);

// TODO: Make this such that it can be called for WDT reset. Document this.
//
// TODO
// TOP:
//  * PWM Disable (start and stop at TOP)
//
// BOTTOM:
//  * Count Pulse
//
// Speed update is allowed to occur any time, as the update to OCR4A is atomic.
//
// This allows for a fixed pulse width via OCR4B.
// OCR4B = 16 (32 us pulse width) 
// Can probably initialize OCR4A to 16 as well to initialize output, and can
// keep counter running! i.e. pwm enable/disable just sets OCR4A to 16 as well.
// This can get out of sync with the state machine, but a direction variable
// could be used as well. 
// Change output to inverting for low pulse, non-inverting for high pulse.
void timer4_init()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR4A = TCCR4A_value;
    TCCR4B = TCCR4B_value_disabled;
    
    TCNT4 = 0;
    OCR4A = 100; // TODO: Decide on a good value to use here and document it.
    OCR4B = MC_PULSE_DURATION; // TODO: microsecond to timer count conversion?
    
    // Enable interrupt at timer 4 BOTTOM.
    TIFR4 = _BV(TOV4); // Clear Timer 4 Overflow Flag
    TIMSK4 = _BV(TOIE4); // Set Timer 4 Overflow Interrupt Enable
  }
  
  // Initialize the timer and allow it to count up to TOP, at which point the
  // interrupt at TOP will disable the timer.
  motor_state_register_pending(MOTOR_STATE_OFF);
  TCCR4B = TCCR4B_value_enabled;

  // TODO: Encapsulate this in a function.
  while (motor_state_pending != MOTOR_STATE_NONE) {}
}

static inline void timer4_enable() {
  // TODO
  // Implement a macro to convert microseconds to timer values.
  TCNT4 = MC_PULSE_DURATION + MC_DIRECTION_SETUP_TIME*2;
  TCCR4B = TCCR4B_value_enabled;
}

static inline void timer4_disable() {
  TCCR4B = TCCR4B_value_disabled;
}

// Enable interrupt at timer 4 TOP.
static inline void timer4_interrupt_TOP_enable()
{
  // Clear Timer 4 Output Compare A Match Flag
  // This prevents the interrupt from being taken immediately after enabling
  // it.
  TIFR4 = _BV(OCF4A); 
  
  // Set Timer 4 Output Compare A Match Interrupt Enable
  TIMSK4 |= _BV(OCIE4A);
}

// Disable interrupt at timer 4 BOTTOM.
static inline void timer4_interrupt_TOP_disable()
{
  // Clear Timer 4 Compare A Match Interrupt Enable
  TIMSK4 &= ~_BV(OCIE4A);
}

static inline void timer4_set_TOP(uint16_t counter_value)
{
  OCR4A = counter_value;
}

//*****************************************************************************
// Motor PWM Control
//*****************************************************************************

static inline void motor_pwm_enable()
{
  // TODO: refactor
  timer4_set_TOP(motor_control.counter_TOP);
  motor_control.counter_update = false;

  timer4_enable();
}

static inline void motor_pwm_disable()
{
  timer4_disable();
}

/*
// Enable interrupts on the phase of the PWM signal where there is no pulse.
static inline void motor_pwm_phase_nopulse_interrupt_enable()
{
  timer4_interrupt_TOP_enable();
}

// Disable interrupts on the phase of the PWM signal where there is no pulse.
static inline void motor_pwm_phase_nopulse_interrupt_disable()
{
  timer4_interrupt_TOP_disable();
}
*/

//*****************************************************************************
// Motor State Machine
//*****************************************************************************

// Sets the motor state machine to the MOTOR_STATE_OFF state, and configures
// its outputs accordingly.
static void motor_state_set_OFF()
{
  motor_pwm_disable();
  digitalWrite(PIN_MOTOR_ENABLE, PIN_MOTOR_ENABLE_FALSE);

  motor_state = MOTOR_STATE_OFF;
  DEBUG_PRINT( "MOTOR_STATE_OFF" );
}

// Sets the motor state machine to the MOTOR_STATE_HOLD state, and configures
// its outputs accordingly.
static void motor_state_set_HOLD()
{
  motor_pwm_disable();
  digitalWrite(PIN_MOTOR_ENABLE, PIN_MOTOR_ENABLE_TRUE);

  motor_state = MOTOR_STATE_HOLD;
  DEBUG_PRINT( "MOTOR_STATE_HOLD" );
}

// Sets the motor state machine to the MOTOR_STATE_OPEN state, and configures
// its outputs accordingly.
static inline void motor_state_set_OPEN()
{

  digitalWrite(PIN_MOTOR_ENABLE, PIN_MOTOR_ENABLE_TRUE);
  digitalWrite(PIN_MOTOR_DIRECTION, PIN_MOTOR_DIRECTION_OPEN);
  motor_position_increment = MOTOR_POSITION_INCREMENT_OPEN;
  motor_pwm_enable();
  
  motor_state = MOTOR_STATE_OPEN;
  DEBUG_PRINT( "MOTOR_STATE_OPEN" );
}

// Sets the motor state machine to the MOTOR_STATE_CLOSE state, and configures
// its outputs accordingly.
static inline void motor_state_set_CLOSE()
{
  digitalWrite(PIN_MOTOR_DIRECTION, PIN_MOTOR_DIRECTION_CLOSE);
  digitalWrite(PIN_MOTOR_ENABLE, PIN_MOTOR_ENABLE_TRUE);
  motor_position_increment = MOTOR_POSITION_INCREMENT_CLOSE;
  motor_pwm_enable();
  
  motor_state = MOTOR_STATE_CLOSE;
  DEBUG_PRINT( "MOTOR_STATE_CLOSE" );
}

// Registers a provided motor state as pending to be installed in the next
// available phase window of the PWM signal.
static void motor_state_register_pending(uint8_t next_state)
{
  motor_state_pending = next_state;
  timer4_interrupt_TOP_enable();
}

// Installs a pending motor state update. It is expected that this function
// will only be called within the correct phase window of the PWM signal.
static inline void motor_state_install_pending()
{
  if (motor_state_pending == MOTOR_STATE_OFF) {
    motor_state_set_OFF();
  } else if (motor_state_pending == MOTOR_STATE_HOLD) {
    motor_state_set_HOLD();
  } else {
    // TODO: error
  }

  timer4_interrupt_TOP_disable();
  motor_state_pending = MOTOR_STATE_NONE;
}

// Determines whether the current motor state is a moving state.
static bool motor_state_moving()
{
  return ((motor_state == MOTOR_STATE_OPEN) ||
          (motor_state == MOTOR_STATE_CLOSE));
}

// Manages motor state machine transitions.
//
// A transition from a non-moving state to any other state is made immediately
// via motor_state_set().
//
// A transition from a moving state to a non-moving state is registered as
// pending to be installed in the next available phase window of the PWM signal
// via motor_state_register_pending().
//
// A transition from a moving state directly to another moving state is treated
// as an error.
void motor_state_transition(uint8_t next_state)
{
  if (motor_state == next_state) {
    return;
  }
 
  if (motor_state_pending != MOTOR_STATE_NONE) {
    if (next_state != motor_state_pending) {
      // TODO: error
    } else {
      return;
    }
  }

  if (next_state == MOTOR_STATE_OFF) {
    if (motor_state_moving()) {
      motor_state_register_pending(MOTOR_STATE_OFF);
    } else {
      motor_state_set_OFF();
    }
  } else if (next_state == MOTOR_STATE_HOLD) {
    if (motor_state_moving()) {
      motor_state_register_pending(MOTOR_STATE_HOLD);
    } else {
      motor_state_set_HOLD();
    }
  } else if (next_state == MOTOR_STATE_OPEN) {
    if (motor_state == MOTOR_STATE_CLOSE) {
      // assert(motor_state != MOTOR_STATE_CLOSE);
      // TODO: error
    }
    motor_state_set_OPEN();
  } else if (next_state == MOTOR_STATE_CLOSE) {
    if (motor_state == MOTOR_STATE_OPEN) {
      // assert(motor_state != MOTOR_STATE_OPEN);
      // TODO: error
    }
    motor_state_set_CLOSE();
  } else {
    // TODO: error
  }
}

//*****************************************************************************
// Motor Position Control
//*****************************************************************************

// Set the current position as the zero point.
void motor_position_set_zero()
{
  assert((motor_state != MOTOR_STATE_OPEN) && (motor_state != MOTOR_STATE_CLOSE));

  // TODO: Check for critical section.
  motor_position = 0;
  motor_position_command = 0;
}

// Sets the motor command positon.
//
// Parameter
// ---------
// uint8_t position   The absolute position to which to move the motor.
//                    units: degrees
void motor_position_set(int8_t position)
{
    // Convert to millidegrees and account for direction.
    motor_position_command = (int16_t) ((((int32_t) position)*1000)/MOTOR_STEP_ANGLE);
}

//*****************************************************************************
// Motor Speed Control
//*****************************************************************************

// Sets the motor command speed.
//
// Parameter
// ---------
// uint8_t  speed   The speed at which to move the motor.
//                  units: MPRM (milli-RPM)
void motor_speed_set(uint16_t speed)
{
  // Convert from MRPM to frequency.
  uint32_t frequency = (((((uint32_t) speed)*360U)/60U)/((uint8_t) MOTOR_STEP_ANGLE))*((uint8_t) MC_MICROSTEPS_PER_STEP);

  DEBUG_PRINT("frequency: %d", frequency);

  // Convert from frequency to counter value.
  // TODO static/#define timer4_frequency
  motor_control.counter_TOP = (uint16_t) (1000000UL/frequency);
  
  motor_control.counter_update = true;
}

// Moves the motor to the home position (fully open).
void motor_home()
{
  // Move the motor far enough in the OPEN direction (more than the total
  // possible excursion) in order to guarantee that the top limit switch will
  // be tripped.
  motor_position_set(-90);
  motor_speed_set(5000UL);
}

static void motor_state_update()
{
  // TODO: Reading motor_position should be in a critical section.
  int32_t motor_step_delta = motor_position_command - motor_position;

  if (motor_step_delta > MOTOR_CONTROL_STEP_ERROR) {
    if (digitalRead(PIN_LIMIT_BOTTOM) == PIN_LIMIT_TRIPPED) {
      // Do not move the motor in the CLOSE direction if the bottom limit
      // switch is tripped.
      motor_state_transition(MOTOR_STATE_HOLD);
    } else {
      motor_state_transition(MOTOR_STATE_CLOSE);
    }
  } else if (motor_step_delta < -MOTOR_CONTROL_STEP_ERROR) {
    if (digitalRead(PIN_LIMIT_TOP) == PIN_LIMIT_TRIPPED) {
      // Do not move the motor in the OPEN direction if the top limit switch is
      // tripped. There is no need to go through motor_state_transition()
      // because the zero reference point is set here.
      motor_state_set_HOLD();
      motor_position_set_zero();
    } else {
      motor_state_transition(MOTOR_STATE_OPEN);
    }
  } else {
    if ((motor_state == MOTOR_STATE_OPEN) ||
        (motor_state == MOTOR_STATE_CLOSE)) {
      // Hold the motor in place in order to maintain the correct position
      // without having to zero the motor again.
      motor_state_transition(MOTOR_STATE_HOLD);
    }
  }
}

//*****************************************************************************
// Interrupt Service Routines
//*****************************************************************************

// Timer 4 BOTTOM
ISR(TIMER4_OVF_vect) {
  // There is a pulse on the PWM signal. Count the pulse to track the motor
  // position.
  if (motor_position_increment) {
    motor_position++;
  } else {
    motor_position--;
  }
}

// Timer 4 TOP
ISR(TIMER4_COMPA_vect)
{
  motor_state_install_pending();
  DEBUG_PRINT("Timer 4 TOP - TCNT4: %u", TCNT4);
}

//*****************************************************************************
// motorHal Interface Implementation
//*****************************************************************************
int8_t motorHalInit(void)
{
  // PWM Signal Setup
  // Configure the PWM pin as an input during timer initialization in order to
  // prevent glitches on the output PWM signal.
  pinMode(PIN_MOTOR_STEP, INPUT);
  timer4_init();
  
  // Motor Pin Configuration
  pinMode(PIN_MOTOR_STEP, OUTPUT);
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_DIRECTION, OUTPUT);

  // Limit Switch Pin Configuration
  pinMode(PIN_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(PIN_LIMIT_TOP, INPUT_PULLUP);

  motor_position_set(INT8_MIN);
  motor_speed_set(5000U);

  while (true) {
    delay(3371);
    DEBUG_PRINT("timer4_enable");
    timer4_enable();
    delay(3637);
    timer4_disable();
    DEBUG_PRINT("timer4_disable, TCNT4: %d", TCNT4);
  }

  // TODO The following code has a few long pauses, which may want to be
  // avoided during WDT reset.

  // Move the motor to the home position to zero the motor position.
  
  while (true) {
    delay(10000);
    motorHalCommand(INT8_MIN, 5000U);
    while(motorHalStatus() == HAL_IN_PROGRESS) {}
    delay(10000);
    motorHalCommand(INT8_MAX, 10000U);
    while(motorHalStatus() == HAL_IN_PROGRESS) {}
  }

  // Move to the top of the bag.
  // TODO: This value will vary depending upon the installation location of the
  // top limit switch, as well as the type of bag. Ultimately, the sensors
  // should be used to find the top of the bag.
  motorHalCommand(15, 5000U);
  while (motorHalStatus() == HAL_IN_PROGRESS) {}
  delay(1000);

  return HAL_OK;
}

// Absolute position, relative to the motor zero point.
// Speed given in MRPM (millirevolutions per second).
int8_t motorHalCommand(uint8_t position, uint16_t speed)
{
  motor_position_set(position);
  motor_speed_set(speed);
  
  motor_state_update();

  if (motor_state_moving()) {
    return HAL_IN_PROGRESS;
  } else {
    return HAL_OK;
  }
}

int8_t motorHalStatus(void)
{
  motor_state_update();

  if (motor_state_moving()) {
    return HAL_IN_PROGRESS;
  } else {
    return HAL_OK;
  }
}

