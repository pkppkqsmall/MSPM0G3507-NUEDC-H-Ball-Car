#include "ina226.h"
#include "mspm0_i2c.h"

#include <stddef.h>

#define INA226_CONFIG_DEFAULT          (0x4527U)
#define INA226_SHUNT_VOLT_LSB_V        (2.5e-6f)
#define INA226_BUS_VOLT_LSB_V          (1.25e-3f)

static int ina226_write16(uint8_t addr, uint8_t reg, uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFFU);

    return mspm0_i2c_write(addr, reg, 2, data);
}

static int ina226_read16(uint8_t addr, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    int ret;

    if (value == NULL)
        return -1;

    ret = mspm0_i2c_read(addr, reg, 2, data);
    if (ret)
        return ret;

    *value = ((uint16_t)data[0] << 8) | data[1];
    return 0;
}

int INA226_Init(uint8_t addr)
{
    return ina226_write16(addr, INA226_REG_CONFIG, INA226_CONFIG_DEFAULT);
}

int INA226_ReadIDs(uint8_t addr, uint16_t *manufacturer_id, uint16_t *die_id)
{
    int ret;

    if ((manufacturer_id == NULL) || (die_id == NULL))
        return -1;

    ret = ina226_read16(addr, INA226_REG_MANUFACTURER_ID, manufacturer_id);
    if (ret)
        return ret;

    return ina226_read16(addr, INA226_REG_DIE_ID, die_id);
}

int INA226_ReadShuntVoltage(uint8_t addr, float *shunt_voltage_v)
{
    uint16_t raw;
    int16_t signed_raw;
    int ret;

    if (shunt_voltage_v == NULL)
        return -1;

    ret = ina226_read16(addr, INA226_REG_SHUNT_VOLT, &raw);
    if (ret)
        return ret;

    signed_raw = (int16_t)raw;
    *shunt_voltage_v = ((float)signed_raw) * INA226_SHUNT_VOLT_LSB_V;
    return 0;
}

int INA226_ReadBusVoltage(uint8_t addr, float *bus_voltage_v)
{
    uint16_t raw;
    int ret;

    if (bus_voltage_v == NULL)
        return -1;

    ret = ina226_read16(addr, INA226_REG_BUS_VOLT, &raw);
    if (ret)
        return ret;

    *bus_voltage_v = ((float)raw) * INA226_BUS_VOLT_LSB_V;
    return 0;
}

int INA226_ReadCurrentByR(uint8_t addr, float rshunt_ohm, float *current_a)
{
    float shunt_voltage_v;
    int ret;

    if ((current_a == NULL) || (rshunt_ohm <= 0.0f))
        return -1;

    ret = INA226_ReadShuntVoltage(addr, &shunt_voltage_v);
    if (ret)
        return ret;

    *current_a = shunt_voltage_v / rshunt_ohm;
    return 0;
}

int INA226_ReadPowerByR(uint8_t addr, float rshunt_ohm, float *power_w)
{
    float bus_voltage_v;
    float current_a;
    int ret;

    if (power_w == NULL)
        return -1;

    ret = INA226_ReadBusVoltage(addr, &bus_voltage_v);
    if (ret)
        return ret;

    ret = INA226_ReadCurrentByR(addr, rshunt_ohm, &current_a);
    if (ret)
        return ret;

    *power_w = bus_voltage_v * current_a;
    return 0;
}

int INA226_ReadMeasurement(uint8_t addr, float rshunt_ohm, INA226_Measurement *measurement)
{
    int ret;

    if ((measurement == NULL) || (rshunt_ohm <= 0.0f))
        return -1;

    ret = INA226_ReadShuntVoltage(addr, &measurement->shunt_voltage_v);
    if (ret)
        return ret;

    ret = INA226_ReadBusVoltage(addr, &measurement->bus_voltage_v);
    if (ret)
        return ret;

    measurement->current_a = measurement->shunt_voltage_v / rshunt_ohm;
    measurement->power_w = measurement->bus_voltage_v * measurement->current_a;

    return 0;
}
