
#include "../hal/hal.h"
#include "../hal/motor.h"

#include <assert.h>
#include <stdint.h>

#include "Arduino.h"

#include <avr/io.h>
#include <avr/interrupt.h>

// #define DEBUG
#define DEBUG_MODULE "motor"
#include "../util/debug.h"

//**************************************
// Pin Definitions
//**************************************

// Motor Pins
#define PIN_MOTOR_PWM       7 // Each pulse moves the motor by a (micro)step.
#define PIN_MOTOR_ENABLE    8 // Enable locks the motor in place when it is not moving, which consumes power.
#define PIN_MOTOR_DIRECTION 9

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

//**************************************
// Motor Control Definitions
//**************************************
#define MOTOR_STATE_OFF   0
#define MOTOR_STATE_HOLD  1
#define MOTOR_STATE_OPEN  2
#define MOTOR_STATE_CLOSE 3

#define MOTOR_DIRECTION_OPEN   -1
#define MOTOR_DIRECTION_CLOSE   1

// TODO
// This value should be derived from Timer 3 and Timer 4 frequencies.
#define MOTOR_CONTROL_STEP_ERROR 5

//**************************************
// Motor Control Variables
//**************************************

// Staging area for motor move commands.
static struct {
  bool    valid = false;
  uint8_t direction = 0;
  uint8_t distance = 0;
} motor_move_command;

static volatile struct {
  int32_t step_command = 0;
  int32_t step_position = 0;
  int8_t  microstep_position = 0;
} motor_control;

// Motor State Machine
static volatile uint8_t motor_state = MOTOR_STATE_OFF;

static volatile bool motor_busy = false; 

//*****************************************************************************
// Timer 3 Configuration: Motor State Control
//
// Configure Timer 3 to generate an interrupt to control motor state.
// WGM3[3:0] = 0b1001 (PWM, Phase and Frequency Correct Mode, TOP: OCR3A)
// CS3[2:0] = 0b010 (clk/8 prescaler)
//*****************************************************************************
void timer3_setup() {
  TCCR3A = (0<<WGM31) | (1<<WGM30);
  TCCR3B = (1<<WGM33) | (0<<WGM32);
  TCNT3 = 0;
  // OCR3A = 20000; // 50 Hz (1,000,000 / 20,000 = 50)
  OCR3A = 10000; // 100 Hz (1,000,000 / 10,000 = 100)
  TIMSK3 = (1<<OCIE3A);
  TCCR3B |= (0<<CS32) | (1<<CS31) | (0<<CS30);
}

//*****************************************************************************
// Timer 4 Configuration: Motor Movement and Position Tracking
//
// Configure Timer 4 to generate a PWM output to the motor to cause it to move
// one (micro)step per pulse. The PWM output is enabled and disabled
// dynamically, so there are separate *_enabled and *_disabled values for
// TCCR4B. Configure an interrupt to occur for every PWM pulse to track the
// motor position.
//
// WGM4[3:0] = 0b1001 (PWM, Phase and Frequency Correct Mode, TOP: OCR4A)
// COM4B[1:0] = 0b10 (Enable PWM output on OC4B -> ATMega 2560 PH4 -> Arduino D7)
// CS3[2:0] = 0b010 (clk/8 prescaler)
//
// Create values for the control registers such that the timer can be enabled
// and disabled dynamically.
//*****************************************************************************
static uint8_t TCCR4A_value = (1<<COM4B1) | (0<<COM4B0) | (0<<WGM41) | (1<<WGM40);
static uint8_t TCCR4B_value_disabled = (1<<WGM43)  | (0<<WGM42);
static uint8_t TCCR4B_value_enabled = TCCR4B_value_disabled | (0<<CS42)   | (1<<CS41)  | (0<<CS40);

void timer4_setup()
{
  TCCR4A = TCCR4A_value;
  TCCR4B = TCCR4B_value_disabled;
  TCNT4 = 0;
  
  // TODO
  OCR4A = 625;
  OCR4B = OCR4A/2; // 50% duty cycle

  TIMSK4 = (1<<OCIE4A);
}

void timer4_enable() {
  TCCR4B = TCCR4B_value_enabled;
}

void timer4_disable() {
  TCCR4B = TCCR4B_value_disabled;
}


//*****************************************************************************
// Motor State Control
//*****************************************************************************

void motor_pwm_enable()
{
  timer4_enable();
}

void motor_pwm_disable()
{
  timer4_disable();
}

// Manages the motor state machine.
void set_motor_state(uint8_t next_state)
{
  if (motor_state == next_state) {
    return;
  }

  if (next_state == MOTOR_STATE_OFF) {
    motor_pwm_disable();
    digitalWrite(PIN_MOTOR_ENABLE, LOW);
    motor_busy = false;
  } else if (next_state == MOTOR_STATE_HOLD) {
    motor_pwm_disable();
    digitalWrite(PIN_MOTOR_ENABLE, HIGH);
    motor_busy = false;
  } else if (next_state == MOTOR_STATE_OPEN) {
    // TODO
    /*
    if (motor_state == MOTOR_STATE_CLOSE) {
      // Should going directly from the CLOSE state to the OPEN state with no
      // intermediate state transition constitute an error?
      //
      // Not sure if restoring interrupt state is necessary here.
      assert(motor_state != MOTOR_STATE_CLOSE);
    }
    */

    digitalWrite(PIN_MOTOR_DIRECTION, PIN_MOTOR_DIRECTION_OPEN);
    digitalWrite(PIN_MOTOR_ENABLE, HIGH);
    motor_pwm_enable();
  } else if (next_state == MOTOR_STATE_CLOSE) {
    // TODO
    /*
    if (motor_state == MOTOR_STATE_OPEN) {
      // Should going directly from the OPEN state to the CLOSE state with no
      // intermediate state transition be an error?

      // Not sure if restoring interrupt state is necessary here.
      assert(motor_state != MOTOR_STATE_OPEN);
    }
    */

    digitalWrite(PIN_MOTOR_DIRECTION, PIN_MOTOR_DIRECTION_CLOSE);
    digitalWrite(PIN_MOTOR_ENABLE, HIGH);
    motor_pwm_enable();
  }

  motor_state = next_state;
}

//*****************************************************************************
// Motor Position Control
//*****************************************************************************

// Set the current position as the zero point.
void motor_position_zero()
{
  assert((motor_state != MOTOR_STATE_OPEN) && (motor_state != MOTOR_STATE_CLOSE));

  motor_control.microstep_position = 0;
  motor_control.step_position = 0;
  motor_control.step_command = 0;
}

// Moves the motor to an angle relative to its current position.
//
// Parameters
// ----------
// int32_t angle      The angle (in degrees) to move the motor.
// int8_t  direction  The direction to move the motor. One of:
//                      * MOTOR_DIRECTION_OPEN
//                      * MOTOR_DIRECTION_CLOSE
void motor_move(uint8_t angle, int8_t direction)
{
  // TODO: Do we want to have assertions? How are they implemented in this
  // system? (i.e. Do they cause a watchdog reset, print a debug message to the
  // console, etc.)
  assert((direction == MOTOR_DIRECTION_OPEN) ||
         (direction == MOTOR_DIRECTION_CLOSE));

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    motor_busy = true;
    

    // Deriving motor_control.step_command from motor_control.step_position can
    // cause error to accumulate from movement to movement. If
    // motor_step_position is to be used for position control, then it needs  to
    // be updated atomically in the motor position tracking ISR.
    // motor_control.step_command = motor_control.step_position + direction*((angle*1000)/MOTOR_STEP_ANGLE);

    // Convert to millidegrees and account for direction.
    motor_control.step_command += direction*((((uint32_t) angle)*1000)/MOTOR_STEP_ANGLE);
  }
}

// Moves the motor to the home position (fully open).
void motor_home()
{
  // Move the motor far enough in the OPEN direction (360_, more than the total
  // possible excursion) in order to guarantee that the top limit switch will
  // be tripped.
  motor_move(360, MOTOR_DIRECTION_OPEN);
}

// Moves the motor to the away position (fully closed).
void motor_away() {
  // Move the motor far enough in the CLOSE direction (360_, more than the
  // total possible excursion) in order to guarantee that the bottom limit
  // switch will be tripped.
  motor_move(360, MOTOR_DIRECTION_CLOSE);
}

//*****************************************************************************
// Interrupt Service Routines
//*****************************************************************************

// Motor State Control ISR
static inline void motor_state_control_ISR()
{
  int32_t motor_step_delta = motor_control.step_command - motor_control.step_position;

  if (motor_step_delta > MOTOR_CONTROL_STEP_ERROR) {
    if (digitalRead(PIN_LIMIT_BOTTOM) == PIN_LIMIT_TRIPPED) {
      // Do not move the motor in the CLOSE direction if the bottom limit switch is tripped.
      set_motor_state(MOTOR_STATE_HOLD);
    } else {
      set_motor_state(MOTOR_STATE_CLOSE);
    }
  } else if (motor_step_delta < -MOTOR_CONTROL_STEP_ERROR) {
    if (digitalRead(PIN_LIMIT_TOP) == PIN_LIMIT_TRIPPED) {
      // Do not move the motor in the OPEN direction if the top limit switch is tripped.
      set_motor_state(MOTOR_STATE_HOLD);
      motor_position_zero();
    } else {
      set_motor_state(MOTOR_STATE_OPEN);
    }
  } else {
    if ((motor_state == MOTOR_STATE_OPEN) ||
        (motor_state == MOTOR_STATE_CLOSE)) {
      // Hold the motor in place in order to maintain the correct position
      // without having to zero the motor again.
      set_motor_state(MOTOR_STATE_HOLD);
    }
  }
}

// Motor Position Tracking ISR
static inline void motor_position_tracking_ISR()
{
  if (motor_state == MOTOR_STATE_OPEN) {
    if (--motor_control.microstep_position <= -MC_MICROSTEPS_PER_STEP) {
      motor_control.step_position--;
      motor_control.microstep_position = 0;
    }
  } else if (motor_state == MOTOR_STATE_CLOSE) {
    if (++motor_control.microstep_position >= MC_MICROSTEPS_PER_STEP) {
      motor_control.step_position++;
      motor_control.microstep_position = 0;
    }
  }
}

ISR(TIMER3_COMPA_vect)
{
  motor_state_control_ISR();
}

ISR(TIMER4_COMPA_vect)
{
  motor_position_tracking_ISR();
}

//*****************************************************************************
// motorHal Interface Implementation
//*****************************************************************************
int motorHalInit(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    timer3_setup();
    timer4_setup();

    // Motor Pin Configuration
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_MOTOR_ENABLE, OUTPUT);
    pinMode(PIN_MOTOR_DIRECTION, OUTPUT);

    // Limit Switch Pin Configuration
    pinMode(PIN_LIMIT_BOTTOM, INPUT_PULLUP);
    pinMode(PIN_LIMIT_TOP, INPUT_PULLUP);
  }

  // TODO The following code has a few long pauses, which may want to be
  // avoided during WDT reset.

  // Move the motor to the home position to zero the motor position.
  motor_home();
  while (motor_busy) {}
  delay(1000);

  // Move to the top of the bag.
  // TODO: This value will vary depending upon the installation location of the
  // top limit switch, as well as the type of bag. Ultimately, the sensors
  // should be used to find the top of the bag.
  motor_move(15, MOTOR_DIRECTION_CLOSE);
  while (motor_busy) {} 
  delay(1000);

  return HAL_OK;
}

// TODO: Implement duration.
int motorHalBegin(unsigned int direction, unsigned int distance, unsigned int duration)
{
  motor_move_command.distance = distance;

  if (direction == MOTOR_HAL_DIRECTION_INHALATION) {
    motor_move_command.direction = MOTOR_DIRECTION_CLOSE;
  } else if (direction == MOTOR_HAL_DIRECTION_EXHALATION) {
    motor_move_command.direction = MOTOR_DIRECTION_OPEN;
  } else {
    return HAL_FAIL;
  }

  motor_move_command.valid = true;
  
  return HAL_OK;
}

int motorHalRun(void)
{
  if (motor_move_command.valid) {
    motor_move(motor_move_command.distance, motor_move_command.direction);
    motor_move_command.valid = false;
  }

  if (motor_busy) {
    return HAL_IN_PROGRESS;
  } else {
    return HAL_OK;
  }
}

