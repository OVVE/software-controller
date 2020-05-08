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

//******************************************************************************
// Pin Definitions
//******************************************************************************

// Motor Pins
#define PIN_MOTOR_DIRECTION 9
#define PIN_MOTOR_ENABLE    8 // Enable locks the motor in place when it is not
                              // moving, which consumes power.
#define PIN_MOTOR_STEP      7 // Each pulse moves the motor by a (micro)step.

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
#error No motor has been selected (see motor.h).
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
#define MOTOR_STATE_OFF   0
#define MOTOR_STATE_HOLD  1
#define MOTOR_STATE_OPEN  2
#define MOTOR_STATE_CLOSE 3

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
{
  timer4_enable();
}

// Disable the PWM signal.
static inline void motor_pwm_disable(void)
{
  timer4_disable();
}

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
}

//******************************************************************************
// Motor Position
//******************************************************************************

// Set the current position as the zero point.
void motor_position_set_zero()
{
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
    }
  }

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

#if ((MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP) > UINT16_MAX)
#error Overflow condition detected.
#endif

#if ((UINT16_MAX * MOTOR_STEPS_PER_REVOLUTION * MC_MICROSTEPS_PER_STEP) >      \
     UINT32_MAX)
#error Overflow condition detected.
#endif

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
}

