#include "VL6180.h"
 
extern I2C_HandleTypeDef hi2c2;
 
#define ADDRESS_DEFAULT 0b0101001
 
static uint16_t const ScalerValues[] = {0, 253, 127, 84};
 
VL6180X *lidar;
 
void setup_VL6180X(void)        {
    lidar->address      = 0x29 << 1;//ADDRESS_DEFAULT;
    lidar->scaling      = 0;
    lidar->ptp_offset   = 0;
    lidar->io_timeout   = 0;
    lidar->did_timeout  = false;
}
 
void setAddress(uint8_t new_addr)   {
    writeReg(I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F);
    lidar->address = new_addr;
}
 
uint8_t getAddress(void)        {
    return lidar->address;
}
 
void getWhoAmI(VL6180X_Identity *Identity){
    Identity->idModel           = readReg(IDENTIFICATION__MODEL_ID);
    Identity->idModelRevMajor   = readReg(IDENTIFICATION__MODEL_REV_MAJOR);
    Identity->idModelRevMinor   = readReg(IDENTIFICATION__MODEL_REV_MINOR);
    Identity->idModuleRevMajor  = readReg(IDENTIFICATION__MODULE_REV_MAJOR);
    Identity->idModuleRevMinor  = readReg(IDENTIFICATION__MODULE_REV_MINOR);
}
 
bool init(){
    bool xStatus = false;
    lidar->ptp_offset = readReg(SYSRANGE__PART_TO_PART_RANGE_OFFSET);
 
    if (readReg(SYSTEM__FRESH_OUT_OF_RESET) == 1)
    {
        lidar->scaling = 1;
 
        writeReg(0x207, 0x01);
        writeReg(0x208, 0x01);
        writeReg(0x096, 0x00);
        writeReg(0x097, 0xFD); // RANGE_SCALER = 253
        writeReg(0x0E3, 0x00);
        writeReg(0x0E4, 0x04);
        writeReg(0x0E5, 0x02);
        writeReg(0x0E6, 0x01);
        writeReg(0x0E7, 0x03);
        writeReg(0x0F5, 0x02);
        writeReg(0x0D9, 0x05);
        writeReg(0x0DB, 0xCE);
        writeReg(0x0DC, 0x03);
        writeReg(0x0DD, 0xF8);
        writeReg(0x09F, 0x00);
        writeReg(0x0A3, 0x3C);
        writeReg(0x0B7, 0x00);
        writeReg(0x0BB, 0x3C);
        writeReg(0x0B2, 0x09);
        writeReg(0x0CA, 0x09);
        writeReg(0x198, 0x01);
        writeReg(0x1B0, 0x17);
        writeReg(0x1AD, 0x00);
        writeReg(0x0FF, 0x05);
        writeReg(0x100, 0x05);
        writeReg(0x199, 0x05);
        writeReg(0x1A6, 0x1B);
        writeReg(0x1AC, 0x3E);
        writeReg(0x1A7, 0x1F);
        writeReg(0x030, 0x00);
 
        writeReg(SYSTEM__FRESH_OUT_OF_RESET, 0);
        xStatus = true;
    }  else  {
        uint16_t s = readReg16Bit(RANGE_SCALER);
 
        if      (s == ScalerValues[3]) { lidar->scaling = 3; }
        else if (s == ScalerValues[2]) { lidar->scaling = 2; }
        else                           { lidar->scaling = 1; }
 
        lidar->ptp_offset *= lidar->scaling;
        xStatus = false;
    }
    return xStatus;
}
 
void configureDefault(void) {
    writeReg(READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);
    writeReg(SYSALS__ANALOGUE_GAIN, 0x46);
    writeReg(SYSRANGE__VHV_REPEAT_RATE, 0xFF);
    writeReg16Bit(SYSALS__INTEGRATION_PERIOD, 0x0063);
    writeReg(SYSRANGE__VHV_RECALIBRATE, 0x01);
    writeReg(SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09);
    writeReg(SYSALS__INTERMEASUREMENT_PERIOD, 0x31);
    writeReg(SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);
    writeReg(SYSRANGE__MAX_CONVERGENCE_TIME, 0x31);
    writeReg(INTERLEAVED_MODE__ENABLE, 0);
    setScaling(1);
}
 
void setScaling(uint8_t new_scaling) {
    uint8_t const DefaultCrosstalkValidHeight = 20; // default value of SYSRANGE__CROSSTALK_VALID_HEIGHT
    if (new_scaling < 1 || new_scaling > 3) { return; }
    lidar->scaling = new_scaling;
    writeReg16Bit(RANGE_SCALER, ScalerValues[lidar->scaling]);
    writeReg(SYSRANGE__PART_TO_PART_RANGE_OFFSET, lidar->ptp_offset / lidar->scaling);
    writeReg(SYSRANGE__CROSSTALK_VALID_HEIGHT, DefaultCrosstalkValidHeight / lidar->scaling);
    uint8_t rce = readReg(SYSRANGE__RANGE_CHECK_ENABLES);
    writeReg(SYSRANGE__RANGE_CHECK_ENABLES, (rce & 0xFE) | (lidar->scaling == 1));
}
 
uint8_t getScaling(void) { return lidar->scaling; }
 
uint8_t readRangeSingle(void)       {
    writeReg(SYSRANGE__START, 0x01);
    return readRangeContinuous();
}
 
uint16_t readRangeSingleMillimeters(void) { return (uint16_t)lidar->scaling * readRangeSingle(); }
 
uint16_t readAmbientSingle(void)        {
    writeReg(SYSALS__START, 0x01);
    return readAmbientContinuous();
 
}
 
uint16_t constrain(uint16_t x, uint16_t a, uint16_t b)          {
    uint16_t retVal = x;
    if (x >= a && x <= b)   retVal = x;
    else if (x < a)  retVal = a;
    else if (x > b)  retVal = b;
 
    return retVal;
}
 
void startRangeContinuous(uint16_t period)      {
    int16_t period_reg = (int16_t)(period / 10) - 1;
    period_reg = constrain(period_reg, 0, 254);
 
    writeReg(SYSRANGE__INTERMEASUREMENT_PERIOD, period_reg);
    writeReg(SYSRANGE__START, 0x03);
}
 
void startAmbientContinuous(uint16_t period)    {
    int16_t period_reg = (int16_t)(period / 10) - 1;
    period_reg = constrain(period_reg, 0, 254);
 
    writeReg(SYSALS__INTERMEASUREMENT_PERIOD, period_reg);
    writeReg(SYSALS__START, 0x03);
}
 
void startInterleavedContinuous(uint16_t period)        {
    int16_t period_reg = (int16_t)(period / 10) - 1;
    period_reg = constrain(period_reg, 0, 254);
 
    writeReg(INTERLEAVED_MODE__ENABLE, 1);
    writeReg(SYSALS__INTERMEASUREMENT_PERIOD, period_reg);
    writeReg(SYSALS__START, 0x03);
}
 
void stopContinuous()           {
    writeReg(SYSRANGE__START, 0x01);
    writeReg(SYSALS__START, 0x01);
 
    writeReg(INTERLEAVED_MODE__ENABLE, 0);
}
 
uint8_t readRangeContinuous(void)           {
    uint16_t millis_start = HAL_GetTick();
    while ((readReg(RESULT__INTERRUPT_STATUS_GPIO) & 0x04) == 0)
    {
        if (lidar->io_timeout > 0 && ((uint16_t)HAL_GetTick() - millis_start) > lidar->io_timeout)
        {
          lidar->did_timeout = true;
          return 255;
        }
    }
 
    uint8_t range = readReg(RESULT__RANGE_VAL);
    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01);
 
    return range;
}
 
 
uint16_t readRangeContinuousMillimeters(void) { return (uint16_t)lidar->scaling * readRangeContinuous(); }
 
uint16_t readAmbientContinuous(void)        {
    uint16_t millis_start = HAL_GetTick();
    while ((readReg(RESULT__INTERRUPT_STATUS_GPIO) & 0x20) == 0)
    {
        if (lidar->io_timeout > 0 && ((uint16_t)HAL_GetTick() - millis_start) > lidar->io_timeout)
        {
          lidar->did_timeout = true;
          return 0;
        }
    }
 
    uint16_t ambient = readReg16Bit(RESULT__ALS_VAL);
    writeReg(SYSTEM__INTERRUPT_CLEAR, 0x02);
 
    return ambient;
}
 
bool timeoutOccurred(void)          {
    bool tmp = lidar->did_timeout;
    lidar->did_timeout = false;
    return tmp;
}
 
void setTimeout(uint16_t timeout) { lidar->io_timeout = timeout; }
 
uint16_t getTimeout(void) { return lidar->io_timeout; }
 
void writeReg(uint16_t reg, uint8_t value)      {
    char data_write[3];
    data_write[0] = ((reg >> 8) & 0xff);
    data_write[1] = (reg & 0xff);
    data_write[2] = value;
    HAL_I2C_Master_Transmit(&hi2c2, lidar->address, (uint8_t*)data_write, 3, 1000);
    lidar->last_status = 0;
 
}
 
void writeReg16Bit(uint16_t reg, uint16_t value)        {
    char data_write[4];
    data_write[0] = ((reg >> 8) & 0xff);
    data_write[1] = (reg & 0xff);
    data_write[2] = ((value >> 8) & 0xff);
    data_write[3] = (value & 0xff);
    HAL_I2C_Master_Transmit(&hi2c2, lidar->address, (uint8_t*)data_write, 4, 1000);
    lidar->last_status = 0;
}
 
void writeReg32Bit(uint16_t reg, uint32_t value)        {
    char data_write[6];
    data_write[0] = ((reg >> 8) & 0xff);
    data_write[1] = (reg & 0xff);
    data_write[2] = ((value >> 24) & 0xff);
    data_write[3] = ((value >> 16) & 0xff);
    data_write[4] = ((value >> 8) & 0xff);
    data_write[5] = (value & 0xff);
    HAL_I2C_Master_Transmit(&hi2c2, lidar->address, (uint8_t*)data_write, 6, 1000);
    lidar->last_status = 0;
}
 
uint8_t readReg(uint16_t reg)           {
    uint8_t value;
    char data_write[2];
    char data_read[1];
 
    data_write[0] = ((reg >> 8) & 0xff);
    data_write[1] = (reg & 0xff);
 
    HAL_I2C_Master_Transmit(&hi2c2, lidar->address, (uint8_t*)data_write, 2, 1000);
    HAL_I2C_Master_Receive(&hi2c2, lidar->address, (uint8_t*)data_read, 1, 1000);
 
    value = data_read[0];
    lidar->last_status = 0;
 
    return value;
}
 
uint16_t readReg16Bit(uint16_t reg)         {
    uint16_t value;
    char data_write[2];
    char data_read[2];
 
    data_write[0] = ((reg >> 8) & 0xff);
    data_write[1] = (reg & 0xff);
 
    HAL_I2C_Master_Transmit(&hi2c2, lidar->address, (uint8_t*)data_write, 2, 1000);
    HAL_I2C_Master_Receive(&hi2c2, lidar->address, (uint8_t*)data_read, 2, 1000);
 
    value = (uint16_t)data_read[0] << 8;
    value |= data_read[1];
    lidar->last_status = 0;
 
    return value;
}
 
uint32_t readReg32Bit(uint16_t reg)         {
    uint32_t value;
    char data_write[2];
    char data_read[4];
 
    data_write[0] = ((reg >> 8) & 0xff);
    data_write[1] = (reg & 0xff);
 
    HAL_I2C_Master_Transmit(&hi2c2, lidar->address, (uint8_t*)data_write, 2, 1000);
    HAL_I2C_Master_Receive(&hi2c2, lidar->address, (uint8_t*)data_read, 4, 1000);
 
    value  = (uint32_t)data_read[0] << 24;
    value |= (uint32_t)data_read[1] << 16;
    value |= (uint16_t)data_read[2] << 8;
    value |= data_read[3];
    lidar->last_status = 0;
 
    return value;
}