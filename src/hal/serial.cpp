//#include <C:\Users\rmcinnis\Documents\Arduino\libraries\due_eeprom\LUFA 100807\LUFA\Drivers\Peripheral\Serial.h>
#include <Arduino.h>
#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"

#define MaxLength 200

byte BufferLength;
String Buffer;
char SendBuffer[MaxLength + 1];

int serialHalInit(void)
{
  SendBuffer[0] = NULL;
  Serial.begin(115200);  // this function does not return anything
  // not sure how we can do something like while (!serial)
  // return MODULE_FAIL;
  return HAL_OK;
}

int serialHalGetData(void)
{
  while (Serial.available())
  {       
    if (BufferLength>=MaxLength) {BufferLength=0;Buffer = "";}  // Overflow buffer reset
    char inChar = (char)Serial.read();
    if (inChar != '\r') { 
      //Serial.print(inChar); 
      if (inChar == '\n') 
      {
        Buffer += (char)0; 
        //SerialCommand();
        int current_length = BufferLength;
        BufferLength=MaxLength; // force buffer reset
        return HAL_OK;
      } 
      else
      {
        Buffer += inChar;
        BufferLength++;
      }
      //if (BufferLength>=MaxLength) {BufferLength=0;Buffer = "";}  // Overflow buffer reset
    } // if (inChar !=  
  } // while serial.available
  return HAL_IN_PROGRESS;
}

int serialHalSendData()
{
  // TODO: Implement
  //*value = 0;
  if (SendBuffer[0] != NULL) {
     Serial.println(SendBuffer);
  }
  SendBuffer[0] = NULL;
  return HAL_OK;
}