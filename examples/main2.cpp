#include <Arduino.h>
#include "StableBMA.h" // 引入 StableBMA 库
#define ACC_ENABLED 1
#define BMA_VERSION 456
#define ATCHY_VER  YATCHY
#if BMA_VERSION < 500
#define BMAM_4 1 // BMA major
#define BMAM_5 0
#else
#define BMAM_5 1
#define BMAM_4 0
#endif


RTC_DATA_ATTR StableBMA SBMA; // Class
// 步数相关数据
RTC_DATA_ATTR bool initedAcc = 0;
uint8_t initAccTries = 4; // Max 15
RTC_DATA_ATTR bool stepsInited = 1;
RTC_DATA_ATTR uint8_t stepDay; // For steps to reset each days
#define ACC_MAX_TRIES 8
// 声明函数

// IIC相关配置
bool initedI2C = false;
uint8_t i2cInitCount = 0;
#define I2C_FREQ 100 // In Khz, BMA456 requires 100 not 50



#if ATCHY_VER == YATCHY || ATCHY_VER == WATCHY_3 
  #if ATCHY_VER == YATCHY
    #define I2C_SDA_PIN 22
    #define I2C_SCL_PIN 23
    #define I2C_FREQ 100 // In Khz, BMA456 requires 100 not 50
  #elif ATCHY_VER == WATCHY_3
    #define I2C_SDA_PIN 12
    #define I2C_SCL_PIN 11
    #define I2C_FREQ 10 // In Khz
  #endif

bool initedI2C = false;
uint8_t i2cInitCount = 0;

bool initI2C()
{
    if (initedI2C == false)
    {
        Serial.println("Starting to init I2C line");
        if (i2cInitCount > 5)
        {
            return false;
        }
        if (Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ * 1000) == false)
        {
            Serial.println("Failed to begin I2C");
            i2cInitCount = i2cInitCount + 1;
            delayTask(10);
            return initI2C();
        } else {
            Serial.println("Inited I2C line");
        }
        initedI2C = true;
    }
    return true;
}

void deInitI2C() {
    initedI2C = false;
    i2cInitCount = 10; // This turns it off forever in this session
    bool wireEnd = Wire.end();
    Serial.println("Wire end status: " + BOOL_STR(wireEnd));
}
#endif









#if ACC_ENABLED

#define ACC_MAX_TRIES 8

#if BMAM_4
uint16_t readRegisterBMA4(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len)
{
#if ATCHY_VER == YATCHY || ATCHY_VER == WATCHY_3
    if (initI2C() == false)
    {
        return 0;
    }
#endif
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)address, (uint8_t)len);
    uint8_t i = 0;
    while (Wire.available())
    {
        data[i++] = Wire.read();
    }
    return 0;
}

uint16_t writeRegisterBMA4(uint8_t address, uint8_t reg, uint8_t *data, uint16_t len)
{
#if ATCHY_VER == YATCHY || ATCHY_VER == WATCHY_3
    if (initI2C() == false)
    {
        return 0;
    }
#endif
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(data, len);
    return (0 != Wire.endTransmission());
}

#elif BMAM_5
int8_t readRegisterBMA5(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
#if ATCHY_VER == YATCHY
    if (initI2C() == false)
    {
        return -1; // Return non-zero on initialization failure
    }
#endif

    if (intf_ptr == NULL)
    {
        return -1; // Handle NULL pointer error
    }

    uint8_t address = *(uint8_t *)intf_ptr; // Extract device address from intf_ptr

    Wire.beginTransmission(address);
    Wire.write(reg_addr);
    int8_t endResult = Wire.endTransmission(); // Complete transmission

    if (endResult != 0)
    {
        return endResult; // Return I2C transmission error code
    }

    // Request 'length' bytes from the device
    uint8_t received = Wire.requestFrom(address, (uint8_t)length);
    if (received != length)
    {
        return -2; // Not all bytes received
    }

    // Read received bytes into data buffer
    for (uint32_t i = 0; i < length; i++)
    {
        if (!Wire.available())
        {
            return -3; // Data not available when expected
        }
        reg_data[i] = Wire.read();
    }

    return 0; // Success
}

int8_t writeRegisterBMA5(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length,
                         void *intf_ptr)
{
#if ATCHY_VER == YATCHY
    if (initI2C() == false)
    {
        return -1; // Return non-zero on initialization failure
    }
#endif

    if (intf_ptr == NULL)
    {
        return -1; // Handle NULL pointer error
    }

    uint8_t address = *(uint8_t *)intf_ptr; // Extract device address from intf_ptr

    Wire.beginTransmission(address);
    Wire.write(reg_addr);                   // Send register address
    Wire.write(reg_data, length);           // Send data bytes
    int8_t result = Wire.endTransmission(); // Complete transmission and get status

    return result; // Return 0 on success, non-zero on failure
}
#endif

// Credits to TinyWatchy
bool accConfig()
{
    bool status = true;
    // Enabling default BMA config
    if (SBMA.defaultConfig(true) == false)
    {
        Serial.println("defaultConfig failed");
        return false;
    }

    // This is not needed for steps, idk for else
    // lookForFalse(SBMA.enableAccel(), &status);

    // if (status == false)
    // {
    //     Serial.println("enableAccel failed");
    //     return false;
    // }

    if (SBMA.enableStepCount() == false) {
        Serial.println("enableStepCount failed");
        return false;
    }

    /*
    // We should not need this for pure step counting, Watchy v3 users maybe
    lookForFalse(SBMA.enableFeature(BMA423_STEP_CNTR_INT, true), &status);
    if(status == false) {
        Serial.println("enableFeature(BMA423_STEP_CNTR_INT failed");
        return status;
    }
    */

    return true;
}

void initAcc()
{
    Serial.println("initAcc Launched");
    if (initedAcc == false)
    {
        initAccTries = initAccTries + 1;
        if (initAccTries > ACC_MAX_TRIES)
        {
            Serial.println("Acc init try limit");
            return;
        }
        uint8_t type;
#if ATCHY_VER == WATCHY_1
        type = 1;
#elif ATCHY_VER == WATCHY_1_5
        type = 2;
#elif ATCHY_VER == WATCHY_2
        type = 2;
#elif ATCHY_VER == WATCHY_3
        type = 3;
#elif ATCHY_VER == YATCHY
        type = 4;
#endif

        Serial.println("Acc watchy type is: " + String(type));
        Serial.println("BMA version: " + String(BMA_VERSION));
#if BMAM_4
        if (SBMA.begin4(type, BMA4_I2C_ADDR_PRIMARY, BMA_VERSION, readRegisterBMA4, writeRegisterBMA4) == false)
        {
            Serial.println("Failed to init bma");
            return;
        }
#elif BMAM_5
        if (SBMA.begin5(type, BMA4_I2C_ADDR_PRIMARY, BMA_VERSION, readRegisterBMA5, writeRegisterBMA5) == false)
        {
            Serial.println("Failed to init bma");
            return;
        }
#endif

        if (accConfig() == false)
        {
            Serial.println("Failed to init bma - config");
            return;
        }

// Frezes acc data somehow
#if 0
        if(SBMA.selfTest() == false) {
            Serial.println("Self test failed");
            return;
        }
#endif

        initedAcc = true;
    }
    else
    {
        Serial.println("Axc is already inited");
    }
}

// All in one function to get steps, it managed everything
// TODO: after changing watchface that doesn't use steps, the acc is still turned on with this feature while its not used
uint16_t getSteps()
{    
    uint16_t steps = 0;
    if (initedAcc == true)
    {
        if (stepsInited == false)
        {
            stepsInited = true;
            SBMA.resetStepCounter();
        }
        else
        {
            if (stepDay != timeRTCLocal.Day)
            {
                stepDay = timeRTCLocal.Day;
                SBMA.resetStepCounter();
            }
            else
            {
                steps = (uint16_t)SBMA.getCounter();
            }
        }
    }
    else
    {
        if (initAccTries > ACC_MAX_TRIES)
        {
            Serial.println("Too many init tries");
            return 8;
        }
        initAcc();
        return getSteps();
    }
    Serial.println("Returning steps: " + String(steps));
    return steps;
}
#else

uint16_t getSteps()
{
    Serial.println("AXC is turned off. This is a fallback function");
    return 1234;
}

#endif