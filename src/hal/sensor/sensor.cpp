
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
int16_t fSum;    // flow sensor data accumulator
int16_t fVolSum; // volume accumulator, units of 0.05[mL], meaning it can store up to ~1.6L ((2^15 -1) * 0.05[mL] = 1638[mL])
// This routine reads the flow sensor value as a voltage off the analog pin and inserts it into the filter
// The flow gets integrated into the Volume accumulator
void airflowSensorHalFetch(){
  int16_t fIn;  // pressure value from sensor.
  fIn=analogRead(FLOW_SENSE_PIN);
  fSum = fSum-(fSum>>3)+fIn; // filter
  
  // get precise volume : integrate flow at every sample
  int16_t f; // flow in 0.01 SLM
  airflowSensorHalGetValue(&f);
  // In order to preserve precision of the volume, the acculumator should be in
  // units on the order of 0.1mL, since the longest breath cycle is 6 seconds 
  // (5[bpm], 1:1I:E = 12[sec/breath] / 2 = 6[sec/breath] for inhalation/exhalation stages)
  // Over 6 seconds, a sampling rate of 10[Hz] means 60 samples and if the calculation
  // error is about 1 unit off every time, units of 0.05[mL] means a difference of only
  // 3[mL] over the course of a breath (60[samples] * 0.05[mL] = 3[mL]).
  // From 0.01[SLM] flow to 0.05[mL] volume:
  //  Vol = Flow * dt
  //      = (Flow[SLM] / (60[sec/min]) * (200[0.05mL/0.01L])) * (0.01[sec])
  //      = Flow * 200 / 100 / 60
  //      = Flow / 30
  // TODO: Consider using units that avoids division entirely
  fVolSum += f / 30;
}

// This routine returns the flow sensor value out of the filter
int airflowSensorHalGetValue(int16_t *value){
  int32_t temp;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    temp = (fSum>>3);
  }  
  // fSum (now temp) is filtered ADC values, from PMF4003V data sheet:
  // flow = (Vout - 1[V]) / 4[V] * Range = (Vout - 1[V]) / 4[V] * 20000[0.01SLM]
  // Vout = ADCVal / 2^10 * 5[V] = ADCVal * 5[V] / 1024
  // So:
  // flow = (ADCVal / 1024 * 5[V] - 1[V]) / 4[V] * 20000[0.01SLM]
  // flow = (ADCVal * 5 - 1 * 1024) / 1024 / 4 * 20000
  // flow = (ADCVal * 5 * 20000 - 1024 * 20000) / 4096
  // flow = (ADCVal * 100000 - 20480000
  *value = (int16_t) (((temp * 100000L) - 20480000L) >> 12);
  return HAL_OK;
}

// This routine returns the integrated volume value
int airVolumeSensorHalGetValue(int16_t *value){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    // Since volume is stored in units [0.05mL], convert to [mL]
    *value = fVolSum / 200;
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