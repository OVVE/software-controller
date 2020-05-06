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
#define PIN_MOTOR_PWM       7 // Each pulse moves the motor by a (micro)step.
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

// millidegrees of shaft rotation (including the gearbox) per motor step

// stepperonline 23HS22-2804S-HG50
// NEMA 23 50:1
// #define MOTOR_STEP_ANGLE 36

// stepperonline 
// NEMA 23 20:1
#define MOTOR_STEP_ANGLE 90
#define MOTOR_SPEED_MIN 400
#define MOTOR_SPEED_MAX 65535

// stepperonline 23HS22-2804S-HG50
// NEMA 23 15:1
//#define MOTOR_STEP_ANGLE 120

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

#define MAX_MOTOR_ACC_CHANGE_PER_CYCLE 100
#define MAX_MOTOR_SPEED_DIR_CHANGE 200


//**************************************
// Motor Control Variables
//**************************************
static volatile struct {
  // Precomputed value so that we can minimize the amount of math operations
  // inside of an ISR.
  bool     counter_update;
  uint16_t counter_TOP;

  int32_t  step_position;
  int8_t   microstep_position;
  uint8_t  direction;
  int16_t speedSet; // +/- -> direction
  int16_t speedRequested; // +/-> direction
} motor_control;

//**************************************
// Motor State Machine Variables
//**************************************
static volatile uint8_t motor_state = MOTOR_STATE_OFF;
static volatile uint8_t motor_state_pending = MOTOR_STATE_NONE;

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
  TIMSK3 = (1<<TOIE3);
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
// COM4B[1:0] = 0b10 (Enable non-inverting PWM output on OC4B -> ATMega 2560 PH4 -> Arduino D7)
// CS3[2:0] = 0b010 (clk/8 prescaler)
//
// Create values for the control registers such that the timer can be enabled
// and disabled dynamically.
//*****************************************************************************
static uint8_t TCCR4A_value = (1<<COM4B1) | (0<<COM4B0) | (0<<WGM41) | (1<<WGM40);
static uint8_t TCCR4B_value_disabled = (1<<WGM43) | (0<<WGM42);
static uint8_t TCCR4B_value_enabled = TCCR4B_value_disabled | (0<<CS42) | (1<<CS41) | (0<<CS40);

void timer4_setup()
{
  TCCR4A = TCCR4A_value;
  TCCR4B = TCCR4B_value_disabled;
  TCNT4 = 0;
  
  // TODO: Set defaults.
  OCR4A = 625;
  OCR4B = OCR4A/2; // 50% duty cycle

  TIMSK4 = (1<<OCIE4A); // Enable interrupt at TOP.
}

static inline void timer4_enable() {
  TCCR4B = TCCR4B_value_enabled;
}

static inline void timer4_disable() {
  TCCR4B = TCCR4B_value_disabled;
  TCNT4  = 0; // TODO: Does TCNT need to be written to zero after OCR4A/OCR4B get written in order to latch in their values?
}

static inline void timer4_interrupt_BOTTOM_enable()
{
  TIFR4 = _BV(TOV4); // Clear the Timer 4 Overflow Flag.
  TIMSK4 |= _BV(TOIE4); // Set the Timer 4 Overflow Interrupt Enable.
}

static inline void timer4_interrupt_BOTTOM_disable()
{
  TIMSK4 &= ~_BV(TOIE4); // Clear the Timer 4 Overflow Interrupt Enable.
}

static inline void timer4_set_TOP(uint16_t counter_value)
{
  OCR4A = counter_value;
  OCR4B = counter_value/2; // 50% duty cycle
}

//*****************************************************************************
// Motor PWM Control
//*****************************************************************************

static inline void motor_pwm_enable()
{
  timer4_set_TOP(motor_control.counter_TOP);
  motor_control.counter_update = false;
  timer4_enable();
}

static inline void motor_pwm_disable()
{
  timer4_disable();
}

static inline void motor_pwm_phase_interrupt_enable()
{
  timer4_interrupt_BOTTOM_enable();
}

static inline void motor_pwm_phase_interrupt_disable()
{
  timer4_interrupt_BOTTOM_disable();
}

//*****************************************************************************
// Motor State Machine
//*****************************************************************************

// Sets the motor state machine to the MOTOR_STATE_OFF state, and configures
// its outputs accordingly.
static void motor_state_set_OFF()
{
  DEBUG_PRINT("MOTOR_STATE_OFF");
  motor_pwm_disable();
  digitalWrite(PIN_MOTOR_ENABLE, PIN_MOTOR_ENABLE_FALSE);

  motor_state = MOTOR_STATE_OFF;
}

// Sets the motor state machine to the MOTOR_STATE_HOLD state, and configures
// its outputs accordingly.
static void motor_state_set_HOLD()
{
  DEBUG_PRINT("MOTOR_STATE_HOLD");
  motor_pwm_disable();
  digitalWrite(PIN_MOTOR_ENABLE, PIN_MOTOR_ENABLE_TRUE);

  motor_state = MOTOR_STATE_HOLD;
}

// Sets the motor state machine to the MOTOR_STATE_OPEN state, and configures
// its outputs accordingly.
static inline void motor_state_set_OPEN()
{
  DEBUG_PRINT("MOTOR_STATE_OPEN");
  digitalWrite(PIN_MOTOR_DIRECTION, PIN_MOTOR_DIRECTION_OPEN);
  digitalWrite(PIN_MOTOR_ENABLE, PIN_MOTOR_ENABLE_TRUE);
  motor_pwm_enable();
  
  motor_state = MOTOR_STATE_OPEN;
}

// Sets the motor state machine to the MOTOR_STATE_CLOSE state, and configures
// its outputs accordingly.
static inline void motor_state_set_CLOSE()
{
  DEBUG_PRINT("MOTOR_STATE_CLOSE");
  digitalWrite(PIN_MOTOR_DIRECTION, PIN_MOTOR_DIRECTION_CLOSE);
  digitalWrite(PIN_MOTOR_ENABLE, PIN_MOTOR_ENABLE_TRUE);
  motor_pwm_enable();
  
  motor_state = MOTOR_STATE_CLOSE;
}

// Registers a provided motor state as pending to be installed in the next
// available phase window of the PWM signal.
static void motor_state_register_pending(uint8_t next_state)
{
  motor_state_pending = next_state;
  motor_pwm_phase_interrupt_enable();
}

// Installs a pending motor state update. It is expected that this function
// will only be called within the correct phase window of the PWM signal.
static void motor_state_install_pending()
{
  if (motor_state_pending == MOTOR_STATE_OFF) {
    motor_state_set_OFF();
  } else if (motor_state_pending == MOTOR_STATE_HOLD) {
    motor_state_set_HOLD();
  } else {
    // TODO: error
  }

  motor_pwm_phase_interrupt_disable();
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

  motor_control.step_position = 0;
  motor_control.microstep_position = 0;
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


  if (speed>MOTOR_SPEED_MAX)
    speed=MOTOR_SPEED_MAX;

  if (speed<MOTOR_SPEED_MIN)
    speed=MOTOR_SPEED_MIN;

  // Convert from MRPM to frequency.
  uint32_t frequency = ((((uint32_t) MC_MICROSTEPS_PER_STEP)*((((uint32_t) speed)*360U)/60U))/((uint32_t) MOTOR_STEP_ANGLE));

  //make sure register values are in meaningful range
  if (frequency<16)
    frequency=16;

  if (frequency>1000000UL)
    frequency=1000000UL;

  // Convert from frequency to counter value.
  motor_control.counter_TOP = (uint16_t) (1000000UL/frequency);
  
  motor_control.counter_update = true;
}

static void motor_state_update()
{
  if (motor_control.direction == MOTOR_DIR_CLOSE) {
    if (digitalRead(PIN_LIMIT_BOTTOM) == PIN_LIMIT_TRIPPED) {
      // Do not move the motor in the CLOSE direction if the bottom limit
      // switch is tripped.
      DEBUG_PRINT("LIMIT_BOTTOM_TRIPPED");
      motor_state_transition(MOTOR_STATE_HOLD);
    } else {
      motor_state_transition(MOTOR_STATE_CLOSE);
    }
  } else if (motor_control.direction == MOTOR_DIR_OPEN) {
    if (digitalRead(PIN_LIMIT_TOP) == PIN_LIMIT_TRIPPED) {
      // Do not move the motor in the OPEN direction if the top limit switch is
      // tripped. There is no need to go through motor_state_transition()
      // because the zero reference point is set here.
      DEBUG_PRINT("LIMIT_CLOSE_TRIPPED");
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

static inline void motor_position_update()
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
  } else {
    static uint16_t error_count = 0;
    DEBUG_PRINT("motor_position_update ERROR %d", ++error_count);
    // TODO: error
  }
}

//*****************************************************************************
// Interrupt Service Routines
//*****************************************************************************

// Timer 3 BOTTOM
ISR(TIMER3_OVF_vect)
{
  // motor_state_update();
}

// Timer 4 BOTTOM
ISR(TIMER4_OVF_vect) {
  motor_state_install_pending();
}

// Timer 4 TOP
ISR(TIMER4_COMPA_vect)
{
  if (motor_control.counter_update) {
    timer4_set_TOP(motor_control.counter_TOP);
    motor_control.counter_update = false;
  }

  motor_position_update();
}


//*****************************************************************************
// motorHal Interface Implementation
//*****************************************************************************
int8_t motorHalInit(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    // TODO: Remove/relocate Timer 3
    // timer3_setup();
    
    timer4_setup();

    // Motor Pin Configuration
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_MOTOR_ENABLE, OUTPUT);
    pinMode(PIN_MOTOR_DIRECTION, OUTPUT);

    // Limit Switch Pin Configuration
    pinMode(PIN_LIMIT_BOTTOM, INPUT_PULLUP);
    pinMode(PIN_LIMIT_TOP, INPUT_PULLUP);
  }

  motor_control.speedSet=0;
  // TODO The following code has a few long pauses, which may want to be
  // avoided during WDT reset.

  // Move the motor to the home position to zero the motor position.
  do
  {
    motorHalCommand(MOTOR_DIR_OPEN, 5000U);
    delay(10);
  } while (motor_state_moving());
  
  delay(1000);

  // Move to the top of the bag.
  // TODO: This value will vary depending upon the installation location of the
  // top limit switch, as well as the type of bag. Ultimately, the sensors
  // should be used to find the top of the bag.
  do
  {
    motorHalCommand(MOTOR_DIR_CLOSE, 5000U);
    delay(10);
  } while ((motorHalGetPosition()<MOTOR_INIT_POSITION) && (motorHalGetStatus()==MOTOR_STATUS_MOVING));
  
  do
  {
    motorHalCommand(MOTOR_DIR_STOP, 0U);
    delay(10);
  } while ((motorHalGetStatus()==MOTOR_STATUS_MOVING));

  
  return HAL_OK;
}


void motorLimitAcceleration(uint8_t dir, uint16_t speed)
{
  motor_control.speedRequested=speed;
  if (dir==MOTOR_DIR_CLOSE)
    motor_control.speedRequested*=-1;
  else if (dir==MOTOR_DIR_STOP)
    motor_control.speedRequested=0;

  //limit acceleration
  /*if (motor_control.speedRequested>motor_control.speedSet+MAX_MOTOR_ACC_CHANGE_PER_CYCLE)
        motor_control.speedSet+=MAX_MOTOR_ACC_CHANGE_PER_CYCLE;
    else if (motor_control.speedRequested<motor_control.speedSet-MAX_MOTOR_ACC_CHANGE_PER_CYCLE)
        motor_control.speedSet-=MAX_MOTOR_ACC_CHANGE_PER_CYCLE;
    else*/
        motor_control.speedSet=motor_control.speedRequested;
   
  DEBUG_PRINT("SpeedSet: %i %i",motor_control.speedRequested,motor_control.speedSet);

  if (motor_control.speedSet>0)
  {
    motor_speed_set(motor_control.speedSet);
    motor_control.direction=MOTOR_DIR_OPEN;      
  } if (motor_control.speedSet<0)
  {
    motor_speed_set((uint16_t)-motor_control.speedSet);
    motor_control.direction=MOTOR_DIR_CLOSE;      
  }
  else
  {
    motor_control.direction=MOTOR_DIR_STOP;      
  }
}

// DIR in MOTOR_DIR_OPEN or MOTOR_DIR_CLOSE
// Speed given in MRPM (millirevolutions per second).
int8_t motorHalCommand(uint8_t dir, uint16_t speed)
{

  motorLimitAcceleration(dir,speed);

  motor_state_update();
}

int32_t motorHalGetPosition(void)
{
    return motor_control.step_position*MOTOR_STEP_ANGLE; //in mdeg
}

int8_t motorHalGetStatus(void)
{
  if (motor_state_moving())
    return MOTOR_STATUS_MOVING;
  else if (digitalRead(PIN_LIMIT_TOP) == PIN_LIMIT_TRIPPED)
    return MOTOR_STATUS_SWITCH_TRIPPED_TOP;
  else if (digitalRead(PIN_LIMIT_BOTTOM) == PIN_LIMIT_TRIPPED)
    return MOTOR_STATUS_SWITCH_TRIPPED_BOTTOM;  
    
}