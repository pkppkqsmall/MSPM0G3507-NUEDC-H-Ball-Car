/*
 * INA226 on shared I2C0 bus with MPU6050.
 *
 * The addresses below assume the values on the PCB are 7-bit I2C addresses.
 */

#ifndef INA226_H_
#define INA226_H_

#include <stdint.h>

#define INA226_ADDR_M1                 (0x42U)
#define INA226_ADDR_M2                 (0x40U)

#define INA226_RSHUNT_M1_OHM           (0.001336f)
#define INA226_RSHUNT_M2_OHM           (0.001336f)

#define INA226_REG_CONFIG              (0x00U)
#define INA226_REG_SHUNT_VOLT          (0x01U)
#define INA226_REG_BUS_VOLT            (0x02U)
#define INA226_REG_POWER               (0x03U)
#define INA226_REG_CURRENT             (0x04U)
#define INA226_REG_CALIBRATION         (0x05U)
#define INA226_REG_MASK_ENABLE         (0x06U)
#define INA226_REG_ALERT_LIMIT         (0x07U)
#define INA226_REG_MANUFACTURER_ID     (0xFEU)
#define INA226_REG_DIE_ID              (0xFFU)

typedef struct
{
    float shunt_voltage_v;
    float bus_voltage_v;
    float current_a;
    float power_w;
} INA226_Measurement;

int INA226_Init(uint8_t addr);
int INA226_ReadIDs(uint8_t addr, uint16_t *manufacturer_id, uint16_t *die_id);
int INA226_ReadShuntVoltage(uint8_t addr, float *shunt_voltage_v);
int INA226_ReadBusVoltage(uint8_t addr, float *bus_voltage_v);
int INA226_ReadCurrentByR(uint8_t addr, float rshunt_ohm, float *current_a);
int INA226_ReadPowerByR(uint8_t addr, float rshunt_ohm, float *power_w);
int INA226_ReadMeasurement(uint8_t addr, float rshunt_ohm, INA226_Measurement *measurement);

#endif  /* INA226_H_ */
