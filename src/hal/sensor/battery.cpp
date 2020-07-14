
#include <stdint.h>

#include <Arduino.h>

#include "../../config.h"

#include "../../hal/hal.h"
#include "../../hal/i2c.h"

#include "../../hal/sensor/battery.h"

#define LOG_MODULE "battery"
#define LOG_LEVEL LOG_BATTERY_SENSOR_HAL
#include "../../util/log.h"

#define INA226_REG_CONFIG      0x00
#define INA226_REG_SHUNT_VOLT  0x01
#define INA226_REG_BUS_VOLT    0x02
#define INA226_REG_POWER       0x03
#define INA226_REG_CURRENT     0x04
#define INA226_REG_CALIBRATION 0x05
#define INA226_REG_MASK        0x06
#define INA226_REG_ALERT       0x07
#define INA226_REG_MAN_ID      0xFE
#define INA226_REG_DIE_ID      0xFF

#define CHARGER_DEVICE_ADDR    0x45
#define BATTERY_DEVICE_ADDR    0x44

// Battery capacity in [mWs]
#define BATTERY_FULL         (120000UL * 3600UL)

#define CHARGING_COUNTER_MAX      3
#define CHARGING_CURRENT_MIN      0

#define WRITE_REG 0x00
#define READ_DATA 0x01

#define CHARGER 0x00
#define BATTERY 0x01

#define GET_STATUS  0x00
#define GET_VSHUNT  0x01
#define GET_VBUS    0x02
#define GET_CURRENT 0x03
#define GET_POWER   0x04

static uint32_t energyInBattery = 0;

struct ina226Registers {
  int16_t  vshunt;
  int16_t  vbus;
  int16_t  current;
  int16_t  power;
};

static struct ina226Registers chargerRegs;
static struct ina226Registers batteryRegs;

static int ina226ReadRegister(uint8_t deviceAddr, uint8_t reg, uint16_t* value)
{
  static uint8_t step = WRITE_REG;
  
  switch (step) {
    case WRITE_REG:
      if (i2cHalWrite(deviceAddr, &reg, sizeof(reg)) == HAL_IN_PROGRESS) {
        break;
      } else {
        step = READ_DATA;
      }
    case READ_DATA:
      if (i2cHalRead(deviceAddr, (uint8_t*) value, sizeof(*value)) == HAL_IN_PROGRESS) {
        // INA226 comes in MSB first, so reorder to be little endian
        *value = (*value << 8) | (*value >> 8);
        break;
      } else {
        step = WRITE_REG;
        return HAL_OK;
      }
  }
  return HAL_IN_PROGRESS;
}

static int ina226WriteRegister(uint8_t deviceAddr, uint8_t reg, uint16_t value)
{
  uint8_t buf[] = {reg, (uint8_t) ((value >> 8) & 0xff), (uint8_t) (value & 0xff)};
  return i2cHalWrite(deviceAddr, buf, sizeof(buf));
}

static void ina226Setup(uint8_t deviceAddr)
{
  // Reset device
  while (ina226WriteRegister(deviceAddr,
                             INA226_REG_CONFIG,
                             bit(15)) == HAL_IN_PROGRESS);
                             
  // Calibration Register
  // CAL = 0.00512 / (CurrentResolution * Rsht)
  //     = 0.00512 / (1 [mA/bit] * 5 [mOhm])
  //     = 0.00512 / (0.001 * 0.005)
  //     = 1024
  while (ina226WriteRegister(deviceAddr,
                             INA226_REG_CALIBRATION,
                             1024) == HAL_IN_PROGRESS);
  
  // Setup the configuration register
  // [11:9] AVG    - 512    (6)
  // [8:6]  VBUSCT - 1.1ms  (4)
  // [5:3]  VSHCT  - 1.1ms  (4)
  // [2:0]  MODE   - Shunt & Bus, Cont (7)
  while (ina226WriteRegister(deviceAddr,
                             INA226_REG_CONFIG,
                             (6 << 9) | (4 << 6) | (4 << 3) | (7 << 0)) == HAL_IN_PROGRESS);
}

static int ina226GetRegisters(uint8_t deviceAddr, struct ina226Registers* regs)
{
  static uint8_t step = GET_STATUS;
  static uint16_t status = 0;
  
  switch (step) {
    case GET_STATUS:
      if (ina226ReadRegister(deviceAddr, INA226_REG_MASK, &status) == HAL_IN_PROGRESS) {
        return HAL_IN_PROGRESS;
      }

      if (status & bit(3)) {
        LOG_PRINT(VERBOSE, "Conversion complete!");
        // Conversion complete, move on to getting the values
        step = GET_VSHUNT;
      } else {
        return HAL_IN_PROGRESS;
      }
    case GET_VSHUNT:
      if (ina226ReadRegister(deviceAddr, INA226_REG_SHUNT_VOLT, (uint16_t*) &(regs->vshunt)) == HAL_IN_PROGRESS) {
        return HAL_IN_PROGRESS;
      }
      LOG_PRINT(VERBOSE, "Got vshunt %d", regs->vshunt);
      step = GET_VBUS;
    case GET_VBUS:
      if (ina226ReadRegister(deviceAddr, INA226_REG_BUS_VOLT, (uint16_t*) &(regs->vbus)) == HAL_IN_PROGRESS) {
        return HAL_IN_PROGRESS;
      }
      LOG_PRINT(VERBOSE, "Got vshunt %d", regs->vbus);
      step = GET_CURRENT;
    case GET_CURRENT:
      if (ina226ReadRegister(deviceAddr, INA226_REG_CURRENT, (uint16_t*) &(regs->current)) == HAL_IN_PROGRESS) {
        return HAL_IN_PROGRESS;
      }
      LOG_PRINT(VERBOSE, "Got current %d", regs->current);
      step = GET_POWER;
    case GET_POWER:
      if (ina226ReadRegister(deviceAddr, INA226_REG_POWER, (uint16_t*) &(regs->power)) == HAL_IN_PROGRESS) {
        return HAL_IN_PROGRESS;
      }
      LOG_PRINT(VERBOSE, "Got power %d", regs->power);
      step = GET_STATUS;
  }
  
  return HAL_OK;
}

int batterySensorHalInit(void)
{
  ina226Setup(CHARGER_DEVICE_ADDR);
  ina226Setup(BATTERY_DEVICE_ADDR);
  
  // TODO: Try to guess at the current battery status somehow
  energyInBattery = BATTERY_FULL;
  return HAL_OK;
}

int batterySensorHalFetch(void)
{
  static uint8_t step = CHARGER;
  
  switch (step) {
    case CHARGER:
      if (ina226GetRegisters(CHARGER_DEVICE_ADDR, &chargerRegs) == HAL_OK) {
        step = BATTERY;
      } else {
        break;
      }
    case BATTERY:
      if (ina226GetRegisters(BATTERY_DEVICE_ADDR, &batteryRegs) == HAL_OK) {
        step = CHARGER;
        LOG_PRINT(DEBUG, "Charger: %d %d %d %d",
                  chargerRegs.vshunt, chargerRegs.vbus,
                  chargerRegs.current, chargerRegs.power);
        LOG_PRINT(DEBUG, "Battery: %d %d %d %d",
                  batteryRegs.vshunt, batteryRegs.vbus,
                  batteryRegs.current, batteryRegs.power);  
        return HAL_OK;
      }
  }
  
  return HAL_IN_PROGRESS;
}

int batterySensorHalGetValue(uint8_t* value, bool* isCharging)
{
  static uint8_t chargingCount = CHARGING_COUNTER_MAX;
  
  // Determine is the system is charging from the charge current
  if (chargerRegs.current <= CHARGING_CURRENT_MIN) {
    if (chargingCount > 0) {
      chargingCount--;
    }
  } else {
    chargingCount = CHARGING_COUNTER_MAX;
  }
  
  *isCharging = (chargingCount > 0);
  
  // Eastimate [mWs] from battery current
  // deltaPower = power * 25 [mW] * dt
  //            = power * 25 [mW] * 1.1 [ms/sample] * 512 [sample] / 1000 [ms/sec]
  //            = power * 15482.5 / 1000 [mWs]
  int32_t deltaPower = (((int32_t) batteryRegs.power) * 15483L) / 1000L;
  
  // energyInBattery [mWs] += deltaPower [mWs]
  energyInBattery = ((int32_t) energyInBattery + deltaPower);
  
  *value = (uint8_t) (((energyInBattery / 1000UL) * 100UL) /
                      (BATTERY_FULL / 1000UL));
  
  return HAL_OK;
}