#include "../config.h"

#include "../hal/hal.h"
#include "../hal/motor.h"

#include <assert.h>
#include <util/atomic.h>

#include "Arduino.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#ifdef DEBUG_MOTOR_HAL
#define DEBUG
#define DEBUG_MODULE "motor"
#include "../util/debug.h"
#endif

//******************************************************************************
// Pin Definitions
//******************************************************************************

// Motor Pins
#define PIN_MOTOR_DIRECTION 9
#define PIN_MOTOR_ENABLE    8 // Enable locks the motor in place when it is not
                              // moving, which consumes power.
#define PIN_MOTOR_STEP      7 // Each pulse moves the motor by a (micro)step.

#define PIN_MOTOR_ENABLE_FALSE LOW
#define PIN_MOTOR_ENABLE_TRUE  HIGH

#define PIN_MOTOR_DIRECTION_OPEN  LOW
#define PIN_MOTOR_DIRECTION_CLOSE HIGH

#define PIN_MOTOR_ENABLE_FALSE LOW
#define PIN_MOTOR_ENABLE_TRUE  HIGH

// Limit Switch Pins
// 
// Connect C (common) to ground and NC (normally closed) to the specified pin,
// which is configured to pull-up mode.
#define PIN_LIMIT_BOTTOM 5
#define PIN_LIMIT_TOP    6

#define PIN_LIMIT_TRIPPED     HIGH
#define PIN_LIMIT_NOT_TRIPPED LOW

//******************************************************************************
// Motor Definitions
//******************************************************************************
#ifdef MOTOR_NANOTEC__ST6018D4508__GP56_T2_26_HR
// Nanotec ST6018D4508   NEMA 24
// Nanotec GP56-T2-26-HR Gear Ratio 26:1
#define MOTOR_STEPS_PER_REVOLUTION (26*200)
#elif defined MOTOR_STEPPERONLINE__23HS30_2804S_HG10
// STEPPERONLINE 23HS30-2804S-HG10
// NEMA 23, 76 mm, Gear Ratio 10:1
#define MOTOR_STEPS_PER_REVOLUTION (10*200)
#elif defined MOTOR_STEPPERONLINE__23HS22_2804S_HG15
// STEPPERONLINE 23HS22-2804S-HG15
// NEMA 23, 56 mm,Gear Ratio 15:1
#define MOTOR_STEPS_PER_REVOLUTION (15*200)
#elif defined MOTOR_STEPPERONLINE__23HS22_2804S_HG20
// STEPPERONLINE 23HS22-2804S-HG20
// NEMA 23, 56 mm, Gear Ratio 20:1
#define MOTOR_STEPS_PER_REVOLUTION (20*200) 
#elif defined MOTOR_STEPPERONLINE__23HS22_2804S_HG50
// STEPPERONLINE 23HS22-2804S-HG50
// NEMA 23, 56 mm ,Gear Ratio 50:1
#define MOTOR_STEPS_PER_REVOLUTION (50*200)
#else
#error No motor has been selected.
#endif

//******************************************************************************
// Motor Controller Definitions
//
// Each motor controller that can be selected has the following definitions:
//
// MC_STEP_PULSE_HIGH
// Configures the step signal to normally low and pulse high for each
// (micro)step.
//
// MC_STEP_PULSE_LOW
// Configures the step signal to normally high and pulse low for each
// (micro)step.
//
// MC_DIRECTION_SETUP_TIME
// The amount of time (in microseconds) that the direction signal must be stable
// before a pulse on the PWM signal.
//
// MC_PULSE_WIDTH
// The width of each pulse (in microseconds) of the step signal.
//******************************************************************************

#ifdef MOTOR_CONTROLLER_NANOTEC__CL4_E_2_12_5VDI
// Nanotec CL4-E-2-12-5VDI

// Clock Direction Multiplier
#define MC_OBJECT_2057 128

// Clock Direction Divider
#define MC_OBJECT_2058   1

// MC_OBJECT_2057 and MC_OBJECT_2058 are configured above for quarter steps.
#define MC_MICROSTEPS_PER_STEP (512/(MC_OBJECT_2057/MC_OBJECT_2058))

#define MC_DIRECTION_SETUP_TIME 35 // us
#define MC_STEP_PULSE_HIGH
#define MC_STEP_PULSE_WIDTH 16 // us
#elif defined MOTOR_CONTROLLER_STEPPERONLINE_ISD08
// stepperonline ISD08
// Microstep Resolution
// The ISD08 supports the following settings:
//  100 (whole steps)
//  200 (half steps)
//  400 (quarter steps)
//  800 (eigth steps)
// 1600 (sixteenth steps)
#if defined(MOTOR_CONTROLLER_ISD08)
#define MC_MICROSTEP_RESOLUTION 400
#define MC_MICROSTEPS_PER_STEP (MC_MICROSTEP_RESOLUTION/100)

#define MC_DIRECTION_SETUP_TIME 35 // us
#define MC_STEP_PULSE_LOW
#define MC_STEP_PULSE_WIDTH 16 // us
#elif defined MOTOR_CONTROLLER_STEPPERONLINE_DM332T
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
#elif defined(MOTOR_CONTROLLER_DM332T)
#define MC_MICROSTEPS_PER_REVOLUTION 800
#define MC_MICROSTEPS_PER_STEP (MC_MICROSTEPS_PER_REVOLUTION/MOTOR_STEPS_PER_REVOLUTION)

#endif

//**************************************
// Motor State Machine Definitions
//**************************************
=======
#define MC_MICROSTEPS_PER_REVOLUTION 800
#define MC_MICROSTEPS_PER_STEP (MC_MICROSTEPS_PER_REVOLUTION/200)

#define MC_DIRECTION_SETUP_TIME 35 // us
#define MC_STEP_PULSE_LOW
#define MC_STEP_PULSE_WIDTH 16 // us
#else
#error No motor controller has been selected (see motor.h).
#endif

//******************************************************************************
// Motor State Machine Definitions
//******************************************************************************
#define MOTOR_STATE_NONE -1
>>>>>>> develop-motor
#define MOTOR_STATE_OFF   0
#define MOTOR_STATE_HOLD  1
#define MOTOR_STATE_OPEN  2
#define MOTOR_STATE_CLOSE 3
#define MOTOR_STATE_NONE  UINT8_MAX

<<<<<<< HEAD
//**************************************
// Motor Control Definitions
//**************************************

// TODO: Update this based on maximum PWM frequency and control loop frequency.
#define MOTOR_CONTROL_STEP_ERROR 5



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
  int32_t speedSet; // +/- -> direction
  int32_t speedRequested; // +/-> direction
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
=======
// These motor state aliases establish the convention that maps
// positive/negative position to direction.
#define MOTOR_STATE__DIRECTION_POSITIVE MOTOR_STATE_CLOSE
#define MOTOR_STATE__DIRECTION_NEGATIVE MOTOR_STATE_OPEN

//******************************************************************************
// Motor Control Definitions
//******************************************************************************
#define MOTOR_POSITION_MIN INT16_MIN
#define MOTOR_POSITION_MAX INT16_MAX

//******************************************************************************
// Motor Control Variables
//******************************************************************************
#if (UINTMAX_MAX != UINT64_MAX)
#error Overflow checks not possible.
#endif

// Motor position in microsteps from zero.
static volatile int16_t motor_position;

// Verify that int16_t motor_position can represent a range of +/- 90 degrees.
#if ((( 90 * MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP) /            \
      DEGREES_PER_REVOLUTION) > INT16_MAX)
#error Overflow condition detected.
#endif

#if (((-90 * MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP) /            \
      DEGREES_PER_REVOLUTION) < INT16_MIN)
#error Overflow condition detected.
#endif

// Motor State Machine
static int8_t motor_state = MOTOR_STATE_NONE;
>>>>>>> develop-motor

//******************************************************************************
// Timer 4 Configuration: Motor Movement and Position Tracking
//
// Configure Timer 4 to generate a PWM signal output to the motor controller,
// causing it to advance the motor by one (micro)step per pulse. The PWM signal
// is enabled and disabled dynamically, so there are separate *_enabled and
// *_disabled values for TCCR4B. Configure an interrupt for each PWM pulse to
// track the motor position.
//******************************************************************************
#define TIMER4_PRESCALER 8

#ifdef MC_STEP_PULSE_HIGH
#if (PIN_MOTOR_STEP == 7)
// WGM4[3:0] = 0b1001 (PWM, Phase and Frequency Correct Mode, TOP: OCR4A)
// COM4B[1:0] = 0b10 (Enable non-inverting PWM output on OC4B -> ATMega 2560 PH4 -> Arduino D7)
#define TCCR4A_VALUE ((1<<COM4B1) | (0<<COM4B0) | (0<<WGM41) | (1<<WGM40))
#define TCCR4B_VALUE_DISABLED ((1<<WGM43) | (0<<WGM42))
#else
#error Unsupported value for PIN_MOTOR_STEP.
#endif
#elif defined MC_STEP_PULSE_LOW
#if (PIN_MOTOR_STEP == 7)
// WGM4[3:0] = 0b1001 (PWM, Phase and Frequency Correct Mode, TOP: OCR4A)
<<<<<<< HEAD
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
=======
// COM4B[1:0] = 0b11 (Enable inverting PWM output on OC4B -> ATMega 2560 PH4 -> Arduino D7)
#define TCCR4A_VALUE ((1<<COM4B1) | (1<<COM4B0) | (0<<WGM41) | (1<<WGM40))
#define TCCR4B_VALUE_DISABLED ((1<<WGM43) | (0<<WGM42))
#else
#error Unsupported value for PIN_MOTOR_STEP.
#endif
#else
#error No motor controller step pulse convention has been defined.
#endif

#if (TIMER4_PRESCALER == 8)
// CS3[2:0] = 0b010 (clk/8 prescaler)
#define TCCR4B_VALUE_ENABLED (TCCR4B_VALUE_DISABLED | (0<<CS42) | (1<<CS41) | (0<<CS40))

// TODO: Consider adding compile-time checks for the platform and clock
// frequency.

// The frequency (Hz) at which the Timer 4 counter updates.
#define TIMER4_COUNT_FREQUENCY (16000000/TIMER4_PRESCALER)
#else
#error Unsupported clock prescaler.
#endif

//                     16 MHz clock
// PWM Frequency = -------------------
//                 2 * prescaler * TOP
#define PWM_FREQUENCY_BASE (16000000/(2*TIMER4_PRESCALER))
#define PWM_FREQUENCY_MIN  ((PWM_FREQUENCY_BASE/UINT16_MAX)+1)
#define PWM_FREQUENCY_MAX  (PWM_FREQUENCY_BASE/100)

// Enable Timer 4.
static inline void timer4_enable() {
#if (TIMER4_COUNT_FREQUENCY == 2000000)
  TCNT4 = MC_STEP_PULSE_WIDTH + MC_DIRECTION_SETUP_TIME*2;
#else 
#error Unsupported value for TIMER4_COUNT_FREQUENCY.
#endif
  
  TCCR4B = TCCR4B_VALUE_ENABLED;
}

// Disable Timer 4.
static void timer4_disable()
{
  if (TCCR4B == TCCR4B_VALUE_ENABLED) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      // Poll for TCNT4 TOP. This can take as long as the period of
      // PWM_FREQUENCY_MIN.
      while (true) {
        // Obtain 2 adjacent samples from TCNT4.
        uint16_t TCNT4_sample1 = TCNT4;

#if (TIMER4_COUNT_FREQUENCY == 2000000)
        // Wait for the timer to increment/decrement.
        asm volatile (
          "nop" "\n\t"
          "nop" "\n\t"
          "nop" "\n\t"
          "nop" "\n\t"
          ::  
        );
#else 
#error Unsupported value for TIMER4_COUNT_FREQUENCY.
#endif

        uint16_t TCNT4_sample2 = TCNT4;

        // Check if the timer is  counting down.
        if (TCNT4_sample2 < TCNT4_sample1) {

          // Check if the timer was not in the middle of a pulse when it was
          // last read a few cycles ago.
          if (TCNT4_sample2 > ((uint8_t) MC_STEP_PULSE_WIDTH)) {
            
            // There is a reasonable likelihood that the timer is not in the
            // middle of a pulse, so disable the timer now.
            TCCR4B = TCCR4B_VALUE_DISABLED;

            if (TCNT4 > ((uint8_t) MC_STEP_PULSE_WIDTH)) {
              // The timer is stopped and not in the middle of a pulse, so the
              // polling loop can now exit.
              DEBUG_PRINT("Timer 4 Disabled, TCNT4: %u", TCNT4);
              break;
            } else {
              // The timer is stopped in the middle of a pulse, so re-enable the
              // timer and continue polling.
              TCCR4B = TCCR4B_VALUE_ENABLED;
            }
          }
        }

        // Open an interrupt window.
        NONATOMIC_BLOCK(NONATOMIC_FORCEOFF) {}
      }
    }
  }
}

// Set a new value for Timer 4 TOP. This changes the frequency of the PWM
// signal. It can be updated at any time without any glitches on the output.
static inline void timer4_TOP_set(uint16_t TOP)
{
  OCR4A = TOP;
}

// TODO: Consider the implications of calling this during WDT reset.
void timer4_init()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR4A = TCCR4A_VALUE;
    TCCR4B = TCCR4B_VALUE_DISABLED;
    
    TCNT4 = 0;
#if (TIMER4_COUNT_FREQUENCY == 2000000)
    OCR4A = 100; // TODO: Document why this value was chosen.
    OCR4B = MC_STEP_PULSE_WIDTH;
#else
#error Unsupported value for TIMER4_COUNT_FREQUENCY.
#endif

    // Enable the interrupt at timer 4 BOTTOM.
    TIFR4 = _BV(TOV4); // Clear Timer 4 Overflow Flag
    TIMSK4 = _BV(TOIE4); // Set Timer 4 Overflow Interrupt Enable
  }
    
  TCCR4B = TCCR4B_VALUE_ENABLED; // The timer starts counting up from zero.
  timer4_disable(); // This will disable the timer once it starts counting down.
}

//******************************************************************************
// Motor PWM Control
//******************************************************************************

// Enable the PWM signal.
static inline void motor_pwm_enable(void)
>>>>>>> develop-motor
{
  timer4_set_TOP(motor_control.counter_TOP);
  motor_control.counter_update = false;
  timer4_enable();
}

<<<<<<< HEAD
static inline void motor_pwm_disable()
=======
// Disable the PWM signal.
static inline void motor_pwm_disable(void)
>>>>>>> develop-motor
{
  timer4_disable();
}

<<<<<<< HEAD
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
=======
// Set the PWM frequency.
#if (PWM_FREQUENCY_MIN > UINT16_MAX)
#error Overflow condition detected.
#endif

#if (PWM_FREQUENCY_MAX > UINT16_MAX)
#error Overflow condition detected.
#endif

#if (PWM_FREQUENCY_MIN > PWM_FREQUENCY_MAX)
#error Error in PWM frequency range.
#endif

#if (PWM_FREQUENCY_BASE > UINT32_MAX)
#error Overflow condition detected.
#endif

#if ((PWM_FREQUENCY_BASE / PWM_FREQUENCY_MIN) > UINT16_MAX)
#error Overflow condition detected.
#endif

#if ((PWM_FREQUENCY_BASE / PWM_FREQUENCY_MAX) > UINT16_MAX)
#error Overflow condition detected.
#endif

static inline void motor_pwm_frequency_set(uint16_t frequency)
{
  if (frequency < PWM_FREQUENCY_MIN) {
    frequency = PWM_FREQUENCY_MIN;
    DEBUG_PRINT("Warning: Limiting motor PWM frequency to maximum value %u Hz", frequency);
  }

  if (frequency > PWM_FREQUENCY_MAX) {
    frequency = PWM_FREQUENCY_MAX;
    DEBUG_PRINT("Warning: Limiting motor PWM frequency to minimum value %u Hz", frequency);
  }

  // Convert from frequency (Hz) to timer TOP value.
  timer4_TOP_set(((uint32_t) PWM_FREQUENCY_BASE) / frequency);
>>>>>>> develop-motor
}

//******************************************************************************
// Motor Position
//******************************************************************************

// Set the current position as the zero point.
void motor_position_set_zero()
{
<<<<<<< HEAD
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
      motor_state_transition(MOTOR_STATE_HOLD);
    } else {
      motor_state_transition(MOTOR_STATE_CLOSE);
    }
  } else if (motor_control.direction == MOTOR_DIR_OPEN) {
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
=======
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    motor_position = 0;
  }
}

//******************************************************************************
// Motor State Machine
//******************************************************************************

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
  
  motor_state = MOTOR_STATE_OPEN;
  motor_pwm_enable();
  
  DEBUG_PRINT( "MOTOR_STATE_OPEN" );
}

// Sets the motor state machine to the MOTOR_STATE_CLOSE state, and configures
// its outputs accordingly.
static inline void motor_state_set_CLOSE()
{
  digitalWrite(PIN_MOTOR_DIRECTION, PIN_MOTOR_DIRECTION_CLOSE);
  digitalWrite(PIN_MOTOR_ENABLE, PIN_MOTOR_ENABLE_TRUE);
  
  motor_state = MOTOR_STATE_CLOSE;
  motor_pwm_enable();
  
  DEBUG_PRINT( "MOTOR_STATE_CLOSE" );
}

// Determines whether the current motor state is a moving state.
static bool motor_state_moving()
{
  return ((motor_state == MOTOR_STATE_OPEN) ||
          (motor_state == MOTOR_STATE_CLOSE));
}

// Manages motor state machine transitions.
//
// Performs limit switch checks and stops the motor if necessary.
//
// A transition from a non-moving state to any other state is made immediately
// via motor_state_set_*().
//
// A transition from a moving state directly to any other moving state is
// treated as an error.
int8_t motor_state_transition(uint8_t next_state)
{
  int8_t motor_status = MOTOR_HAL_STATUS_ERROR;

  if (next_state == MOTOR_STATE_OFF) {
    if (motor_state != MOTOR_STATE_OFF) {
      motor_state_set_OFF();
    }
    motor_status = MOTOR_HAL_STATUS_STOPPED;
  } else if (next_state == MOTOR_STATE_HOLD) {
    if (motor_state != MOTOR_STATE_HOLD) {
      motor_state_set_HOLD();
    }
    motor_status = MOTOR_HAL_STATUS_STOPPED;
  } else if (next_state == MOTOR_STATE_OPEN) {
    if (motor_state == MOTOR_STATE_CLOSE) {
      motor_state_set_HOLD();
      DEBUG_PRINT("Error: Detected illegal transition from MOTOR_STATE_CLOSE to MOTOR_STATE_OPEN");
      motor_status = MOTOR_HAL_STATUS_ERROR;
    } else if (digitalRead(PIN_LIMIT_TOP) == PIN_LIMIT_TRIPPED) {
      if (motor_state != MOTOR_STATE_HOLD) {
        // Do not move the motor in the OPEN direction if the top limit switch is
        // tripped.
        motor_state_set_HOLD();
        motor_position_set_zero();
        DEBUG_PRINT("Top limit switch tripped.");
      }
      motor_status = MOTOR_HAL_STATUS_LIMIT_TOP;
    } else {
      if (motor_state != MOTOR_STATE_OPEN) {
        motor_state_set_OPEN();
      }
      motor_status = MOTOR_HAL_STATUS_MOVING;
    }
  } else if (next_state == MOTOR_STATE_CLOSE) {
    if (motor_state == MOTOR_STATE_OPEN) {
      motor_state_set_HOLD();
      DEBUG_PRINT("Error: Detected illegal transition from MOTOR_STATE_OPEN to MOTOR_STATE_CLOSE");
      motor_status = MOTOR_HAL_STATUS_ERROR;
    } else if (digitalRead(PIN_LIMIT_BOTTOM) == PIN_LIMIT_TRIPPED) {
      if (motor_state != MOTOR_STATE_HOLD) {
        // Do not move the motor in the CLOSE direction if the bottom limit switch
        // is tripped.
        motor_state_set_HOLD();
        DEBUG_PRINT("Bottom limit switch tripped.");
      }
      motor_status = MOTOR_HAL_STATUS_LIMIT_BOTTOM;
    } else {
      if (motor_state != MOTOR_STATE_CLOSE) {
        motor_state_set_CLOSE();
      }
      motor_status = MOTOR_HAL_STATUS_MOVING;
>>>>>>> develop-motor
    }
  }

<<<<<<< HEAD
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
=======
  return motor_status;
}

//******************************************************************************
// Motor Speed Control
//******************************************************************************

#if (MOTOR_STEPS_PER_REVOLUTION > UINT16_MAX)
#error Overflow condition detected.
#endif

#if (MC_MICROSTEPS_PER_STEP > UINT16_MAX)
#error Overflow condition detected.
#endif
>>>>>>> develop-motor

#if ((MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP) > UINT16_MAX)
#error Overflow condition detected.
#endif

#if ((UINT16_MAX * MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP) >      \
     UINT32_MAX)
#error Overflow condition detected.
#endif

<<<<<<< HEAD
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
  } while ((motorHalGetPosition()<MOTOR_INIT_POSITION) && (motorHalGetStatus()!=MOTOR_STATUS_SWITCH_TRIPPED_BOTTOM));
  
  motorHalCommand(MOTOR_DIR_STOP_NOW, 0U);
  

  //step response test
  #ifdef MOTOR_STEP_RESPONSE_TEST

    for (int i=0;i<5;i++)
    {
    do
    {
      motorHalCommand(MOTOR_DIR_CLOSE, MAX_MOTOR_SPEED_CLOSING);
      delay(10);
    } while ((motorHalGetPosition()<50000) && (motorHalGetStatus()!=MOTOR_STATUS_SWITCH_TRIPPED_BOTTOM));

    do
    {
      motorHalCommand(MOTOR_DIR_OPEN, MAX_MOTOR_SPEED_OPENING);
      delay(10);
    } while ((motorHalGetPosition()>20000) && (motorHalGetStatus()!=MOTOR_STATUS_SWITCH_TRIPPED_TOP));
    }
    motorHalCommand(MOTOR_DIR_STOP_NOW, 0U);
  #endif  
  
  return HAL_OK;
}

// DIR in MOTOR_DIR_OPEN or MOTOR_DIR_CLOSE
// Speed given in MRPM (millirevolutions per second).
int8_t motorHalCommand(uint8_t dir, uint16_t speed)
{

 if (dir==MOTOR_DIR_OPEN)
    motor_control.speedRequested=speed;
  else if (dir==MOTOR_DIR_CLOSE)
    motor_control.speedRequested=-(int32_t)speed;
  else if (dir==MOTOR_DIR_STOP)
    motor_control.speedRequested=0;
  else if (dir==MOTOR_DIR_STOP_NOW)
  {
      motor_control.speedRequested=0;
      motor_control.speedSet=0;
      motor_control.direction=MOTOR_DIR_STOP;      
      motor_state_update();
      return HAL_OK;
  }

  //limit acceleration
  if (motor_control.speedRequested>motor_control.speedSet+MAX_MOTOR_ACC_CHANGE_PER_CYCLE_OPENING)
        motor_control.speedSet+=MAX_MOTOR_ACC_CHANGE_PER_CYCLE_OPENING;
    else if (motor_control.speedRequested<motor_control.speedSet-MAX_MOTOR_ACC_CHANGE_PER_CYCLE_CLOSING)
        motor_control.speedSet-=MAX_MOTOR_ACC_CHANGE_PER_CYCLE_CLOSING;
    else
        motor_control.speedSet=motor_control.speedRequested;
   
  if (motor_control.speedSet>0)
  {
    motor_speed_set(motor_control.speedSet);
    motor_control.direction=MOTOR_DIR_OPEN;      
    motor_state_update();
  } else if (motor_control.speedSet<0)
  {
    motor_speed_set((uint16_t)-motor_control.speedSet);
    motor_control.direction=MOTOR_DIR_CLOSE;      
    motor_state_update();
  }
  else
  {
    motor_control.direction=MOTOR_DIR_STOP;      
    motor_state_update();
  }

      return HAL_OK;

}

int32_t motorHalGetPosition(void)
{
    return motor_control.step_position*MOTOR_STEP_ANGLE; //in mdeg
=======
#if (MOTOR_HAL_RPM_MULTIPLIER > UINT32_MAX)
#error Overflow condition detected.
#endif

#if (SECONDS_PER_MINUTE > UINT8_MAX)
#error Overflow condition detected.
#endif

#if ((MOTOR_HAL_RPM_MULTIPLIER * SECONDS_PER_MINUTE) > UINT32_MAX)
#error Overflow condition detected.
#endif

#if ((((MOTOR_HAL_SPEED_MAX) *                                                 \
       (MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP)) /                \
      (MOTOR_HAL_RPM_MULTIPLIER * SECONDS_PER_MINUTE)) > PWM_FREQUENCY_MAX)
#error Maximum PWM frequency exceeded.
#endif

void motor_speed_set(uint16_t speed)
{
  static uint16_t last_speed = 0;

  if (speed == last_speed) {
    return;
  } else {
    last_speed = speed;
  }

  if (speed == 0) {
    motor_state_set_HOLD();
    return;
  }

  if (speed > MOTOR_HAL_SPEED_MAX) {
    speed = MOTOR_HAL_SPEED_MAX;
    DEBUG_PRINT("Warning: Limiting speed to maximum value %u RPM*%u", speed, MOTOR_HAL_RPM_MULTIPLIER);
  }

  //--------------------------------------------------------
  // Convert from RPM*MOTOR_HAL_RPM_MULTIPLIER to frequency.
  //--------------------------------------------------------
  uint32_t frequency = (uint32_t) speed;

  frequency *= ((uint16_t) MOTOR_STEPS_PER_REVOLUTION) *
               ((uint16_t) MC_MICROSTEPS_PER_STEP);
  frequency /= ((uint32_t) MOTOR_HAL_RPM_MULTIPLIER) *
               ((uint8_t)  SECONDS_PER_MINUTE);
 
  motor_pwm_frequency_set(frequency);
}

//******************************************************************************
// Interrupt Service Routine
//******************************************************************************

// Timer 4 BOTTOM
ISR(TIMER4_OVF_vect) {
  // This interrupt occurs when there is a pulse on the PWM signal. Update the
  // motor position.
  if (motor_state == MOTOR_STATE__DIRECTION_POSITIVE) {
    motor_position++;
  } else if (motor_state == MOTOR_STATE__DIRECTION_NEGATIVE) {
    motor_position--;
  }
}

//******************************************************************************
// motorHal Interface Implementation
//******************************************************************************
int8_t motorHalInit(void)
{
  // PWM Signal Setup
  // Configure the step pin as an input during timer initialization in order to
  // prevent glitches on the output PWM signal.
  pinMode(PIN_MOTOR_STEP, INPUT);
  timer4_init();
  
  // Motor Pin Configuration
  pinMode(PIN_MOTOR_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_STEP, OUTPUT);

  // Limit Switch Pin Configuration
  pinMode(PIN_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(PIN_LIMIT_TOP, INPUT_PULLUP);

  // Initialize motor state to MOTOR_STATE_OFF.
  motor_state_set_OFF();

  return HAL_OK;
}

int8_t motorHalCommand(uint8_t command, uint16_t speed)
{
  int8_t motor_status = MOTOR_HAL_STATUS_ERROR;

  if (command == MOTOR_HAL_COMMAND_OFF) {
    motor_status = motor_state_transition(MOTOR_STATE_OFF);
  } else if (command == MOTOR_HAL_COMMAND_HOLD) {
    motor_status = motor_state_transition(MOTOR_STATE_HOLD);
  } else if (command == MOTOR_HAL_COMMAND_OPEN) {
    motor_speed_set(speed);
    motor_status = motor_state_transition(MOTOR_STATE_OPEN);
  } else if (command = MOTOR_HAL_COMMAND_CLOSE) {
    motor_speed_set(speed);
    motor_status = motor_state_transition(MOTOR_STATE_CLOSE);
  }

  return motor_status;
}

#if (MOTOR_HAL_DEGREE_MULTIPLIER > INT32_MAX)
#error Overflow condition detected.
#endif

#if (DEGREES_PER_REVOLUTION > INT16_MAX)
#error Overflow condition detected.
#endif

#if ((MOTOR_HAL_DEGREE_MULTIPLIER * DEGREES_PER_REVOLUTION) > INT32_MAX)
#error Overflow condition detected.
#endif

#if ((MOTOR_HAL_DEGREE_MULTIPLIER * DEGREES_PER_REVOLUTION) >                  \
     (INT64_MAX / (MOTOR_POSITION_MAX)))
#error Overflow condition detected.
#endif

#if ((MOTOR_HAL_DEGREE_MULTIPLIER * DEGREES_PER_REVOLUTION) >                  \
     (INT64_MIN / (MOTOR_POSITION_MIN)))
#error Overflow condition detected.
#endif

#if ((((MOTOR_POSITION_MAX) *                                                  \
       (MOTOR_HAL_DEGREE_MULTIPILER * DEGREES_PER_REVISION)) /                 \
      (MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP)) > INT32_MAX)
#error Overflow condition detected.
#endif

#if ((((MOTOR_POSITION_MIN) *                                                  \
       (MOTOR_HAL_DEGREE_MULTIPILER * DEGREES_PER_REVISION)) /                 \
      (MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP)) < INT32_MIN)
#error Overflow condition detected.
#endif

#if (MOTOR_STEPS_PER_REVOLUTION > INT16_MAX)
#error Overflow condition detected.
#endif

#if (MC_MICROSTEPS_PER_STEP > INT8_MAX)
#error Overflow condition detected.
#endif

#if ((MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP) > INT16_MAX)
#error Overflow condition detected.
#endif

int32_t motorHalGetPosition(void)
{
  (void) motorHalGetStatus();

  //----------------------------------------------------------------
  // Convert from microsteps to degrees*MOTOR_HAL_DEGREE_MULTIPLIER.
  //----------------------------------------------------------------
  int64_t result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    result = (int64_t) motor_position;
  }

  result *= ((int32_t) MOTOR_HAL_DEGREE_MULTIPLIER) *
            ((int16_t) DEGREES_PER_REVOLUTION);
  result /= ((int16_t) MOTOR_STEPS_PER_REVOLUTION) *
            ((int8_t)  MC_MICROSTEPS_PER_STEP);

  return (int32_t) result;
}

int8_t motorHalGetStatus(void)
{
  return motor_state_transition(motor_state);
>>>>>>> develop-motor
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
