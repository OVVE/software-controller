
#include "../../hal/hal.h"
#include "../../hal/sensor/sensor.h"
#include <Arduino.h>
#include <util/atomic.h>

// Sensor pin defines
#define FLOW_SENSE_PIN     A0
#define PRESSURE_SENSE_PIN A1


// The following function initializes Timer 5 to do pressure and flow sensor data collection
int sensorHalInit(void)
{
  // Sensor timer
  TCCR5A=0; // set timer control registers
  TCCR5B=0x0A;  // set timer to clear timer on compare (CTC), and prescaler to divide by 8
  TCNT5=0;  // initialize counter to 0
  OCR5A= 10000;  // Set interrupt to 2khz
  TIMSK5 |= 0x02; // enable timer compare interrupt

  return HAL_OK;
}


//--------------------------------------------------------------------------------------------------
int16_t pSum; // pressure data accumulator
// This routine reads the pressure value as a voltage off the analog pin and inserts it into the filter
void pressureSensorHalFetch(){
  int16_t pIn;  // pressure value from sensor.
  pIn=analogRead(PRESSURE_SENSE_PIN);
  pSum = pSum-(pSum>>4)+pIn; // filter
}

// This routine returns the pressure value out of the filter
int pressureSensorHalGetValue(int16_t *value){
  int32_t temp;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
	temp = pSum>>4;
  }
  int32_t pascals = ((temp * 217L) - 110995L)>>2;	// convert to Pascals
  int32_t u100umH2O = (pascals * 4177L)>>12;		// convert Pascals to 0.1mmH2O
  *value = (int16_t)u100umH2O;						// return as 16 bit signed int
  return HAL_OK;
}


//--------------------------------------------------------------------------------------------------
#define ADC_1V_OFFSET  204     // 1V offset removed
#define FLOW_SENSOR_SCALING 27 // should have been 24.42 if the sensor and ADC were 100% to the spec
int16_t fSum;    // flow sensor data accumulator
int16_t fVolSum; // volume accumulator (1 unit is 1/12ml), max stored value >2.7L
// This routine reads the flow sensor value as a voltage off the analog pin and inserts it into the filter
// The flow gets integrated into the Volume accumulator
void airflowSensorHalFetch(){
  int16_t fIn;  // pressure value from sensor.
  fIn=analogRead(FLOW_SENSE_PIN) - ADC_1V_OFFSET; // 1V offset removed
  fSum = fSum-(fSum>>3)+fIn; // filter
  // get precise volume : integrate flow at every sample
  int16_t f = (fSum>>3)*FLOW_SENSOR_SCALING; // flow in 0.01 SLM
  fVolSum += f/50; // /60*1000/1000/100*10 = /600 = /(50*12)
    // /60 --> convert SLM to liters per second
    // *1000 --> convert liters to cubic centimeters
    // /1000 --> we count time in milliseconds
    // /100  --> convert 0.01 mlps to 1 mlps
    // *mt_delta --> current measurement time delta in milliseconds

}
// This routine returns the flow sensor value out of the filter
int airflowSensorHalGetValue(int16_t *value){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    *value = (fSum>>3)*FLOW_SENSOR_SCALING; // flow in 0.01 SLM
  }  
  return HAL_OK;
}

// This routine returns the integrated volume value
int airVolumeSensorHalGetValue(int16_t *value){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    *value = fVolSum/12; // volume in ml
  }
}

void resetVolume() {
  fVolSum = 0;
}


//--------------------------------------------------------------------------------------------------
// This interrupt service routine is called every 0.5ms in order to handle timed
// events like sensor reads
// Note:
//  The serial comms triggers every 100ms.
//  The MPXV7025 pressure sensor has a response time of 1ms, and a warm-up time of 20ms.
//    Currently sampling every 5ms and running through a /16 filter.
//  The Posifa PMF4103V flow sensor has a response time of 5ms. Currently sampling every 10ms and
//    running through a /8 filter
int16_t timer5Count=0;
ISR(TIMER5_COMPA_vect){
  timer5Count++;
  pressureSensorHalFetch();
  if(timer5Count==2){
    airflowSensorHalFetch();
	timer5Count=0;
  }
}