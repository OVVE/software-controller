
#include "../../hal/hal.h"
#include "../../hal/sensor/sensor.h"
#include <Arduino.h>
#include <util/atomic.h>



#define ADC_READ_STATE_IDLE 0
#define ADC_READ_STATE_RUNNING 1
#define ADC_READ_STATE_READY 2
#define AIRFLOW_BIAS_AVERAGE 32


static int8_t sensorAdcReadState=ADC_READ_STATE_IDLE;
static int16_t airFlowSensorBias=0;
// The following function initializes Timer 5 to do pressure and flow sensor data collection
int sensorHalInit(void)
{

  //we're not using the arduino analogRead() routines to have full control over the ADC
  
  //init ADC with a prescaler of 128 to get a good noise free reading:

  ADMUX =   bit (REFS0) ;//Ref=Avcc

  ADCSRA = bit (ADEN) | bit (ADPS0) |  bit (ADPS1) | bit (ADPS2);  // Prescaler of 128

  airflowSensorCalibrateBias(); 

  return HAL_OK;
}

/*low level ADC routines to read ADC data non-blocked */

//starts an ADC reading
int sensorAdcStartAnalogRead(int pin)
{
  if (sensorAdcReadState!=ADC_READ_STATE_IDLE)
     return HAL_IN_PROGRESS;

  ADMUX =   (ADMUX&0xF8) | (pin & 0x07);  // AVcc   
  ADCSRA |= bit (ADSC);
 
  sensorAdcReadState=ADC_READ_STATE_RUNNING;

  return HAL_OK;
}

//checks if ADC data is available. Needs to return HAL_OK before data can be read with sensorAdcCheckGetData()
int sensorAdcCheckRead(void)
{
   if (sensorAdcReadState==ADC_READ_STATE_IDLE)
	return HAL_FAIL;

   if (bit_is_clear(ADCSRA, ADSC))
   { 
	sensorAdcReadState=ADC_READ_STATE_READY;
	return HAL_OK;
   }else
   {
 	return HAL_IN_PROGRESS;
   }
}

//returns raw ADC data if available
int sensorAdcGetData(int16_t* data)
{
 if (sensorAdcReadState==ADC_READ_STATE_READY)
 {
        *data=ADC;
	sensorAdcReadState=ADC_READ_STATE_IDLE;
	return HAL_OK;
 }else
 {
 	return HAL_FAIL;
 }
}

int16_t sensorAdcAnalogReadBlocked(int pin, int16_t *rawData)
{

	if (sensorAdcStartAnalogRead(pin)!=HAL_OK)
		return HAL_FAIL;
	while (sensorAdcCheckRead()==HAL_IN_PROGRESS);
	if ( sensorAdcGetData(rawData)!=HAL_OK)
		return HAL_FAIL;

	return HAL_OK;
	
}
//--------------------------------------------------------------------------------------------------

// This routine returns the pressure value out of the filter
int pressureSensorHalGetValue(int16_t adcRawData, int16_t *value){
  int32_t pascals = (((int32_t)adcRawData * 217L) - 110995L)>>2;	// convert to Pascals
  int32_t u100umH2O = (pascals * 4177L)>>12;		// convert Pascals to 0.1mmH2O
  *value = (int16_t)u100umH2O;						// return as 16 bit signed int
  return HAL_OK;
}


//--------------------------------------------------------------------------------------------------

//calibrates bias (blocking for ~ 16ms, only call at startup!)
int airflowSensorCalibrateBias(void)
{
	int32_t bias=0;
	uint8_t i;
	int16_t rawData;	
	for (i=0;i<AIRFLOW_BIAS_AVERAGE;i++)
	{
       	    if (sensorAdcAnalogReadBlocked(FLOW_SENSE_PIN,&rawData)!=HAL_OK)
		return HAL_FAIL;
	    bias+=rawData;
	}

	bias/=AIRFLOW_BIAS_AVERAGE;
	
	//update bias
	airFlowSensorBias=0;
	
	//convert to correct unit
	airflowSensorHalGetFlow(bias,&airFlowSensorBias);

	return HAL_OK;	
}

//returns current airflow sensor bias
int16_t airflowSensorGetBias(void)
{
	return airFlowSensorBias;
}

// This routine returns the flow sensor value out of the filter
int airflowSensorHalGetFlow(int16_t adcRawData, int16_t *value){

  // fSum (now temp) is filtered ADC values, from PMF4003V data sheet:
  // flow = (Vout - 1[V]) / 4[V] * Range = (Vout - 1[V]) / 4[V] * 20000[0.01SLM]
  // Vout = ADCVal / 2^10 * 5[V] = ADCVal * 5[V] / 1024
  // So:
  // flow = (ADCVal / 1024 * 5[V] - 1[V]) / 4[V] * 20000[0.01SLM]
  // flow = (ADCVal * 5 - 1 * 1024) / 1024 / 4 * 20000
  // flow = (ADCVal * 5 * 20000 - 1024 * 20000) / 4096
  // flow = (ADCVal * 100000 - 20480000
  *value = (int16_t) (((((int32_t)adcRawData * 100000L) - 20480000L) >> (int32_t)12) - airFlowSensorBias);
  return HAL_OK;
}

int32_t fVolSum; // volume accumulator, units of 0.01[mL]

// This routine updated the integrated volume value
int airflowSensorHalUpdateVolume(int16_t flowValue)
{
	static long lastMicros=0;
	volatile long currentTime=micros(); //only measure micros once for best accuracy. Volatiles to prevent compiler optimisation around here.
	volatile long currentTimeDiff=currentTime-lastMicros;
	lastMicros=currentTime;

	//integrate with dt in 100us accuracy
	//integral is not a standard unit intentionally to preserve high resolution
	fVolSum+=((int32_t)flowValue*((int32_t)currentTimeDiff))/(int32_t)100; 


	return HAL_OK;
}


// This routine returns the integrated volume value
int32_t airflowSensorHalGetVolume(void)
{

  //total=/600 for 100us timing accuracy and SLM to 0.01ml Volume scaling
  
  return fVolSum/600;
}

int airflowSensorHalResetVolume(void) {
  fVolSum = 0;
  return HAL_OK;
}


