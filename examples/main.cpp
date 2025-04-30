#include <Arduino.h>
#include <StableBMA.h>
#include <Wire.h>
#include <Arduino.h>
#include <StableBMA.h>

// 定义I2C通信的读写和延时函数
short unsigned int readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) {
        return 1; // 返回非零值表示错误
    }
    Wire.requestFrom((int)dev_addr, (int)len);
    for (int i = 0; i < len; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
        } else {
            return 1; // 返回非零值表示错误
        }
    }
    return 0; // 返回零值表示成功
}

short unsigned int writeRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (int i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    if (Wire.endTransmission() != 0) {
        return 1; // 返回非零值表示错误
    }
    return 0; // 返回零值表示成功
}

void delayMs(uint32_t period) {
    delay(period);
}

// 创建StableBMA对象
StableBMA sensor;

void setup() {
    Serial.begin(115200);
      // 将引脚设置为输出模式
  pinMode(25, OUTPUT);
  // 向引脚写入低电平
  digitalWrite(25, LOW);
    Wire.begin(26,14);

    // 初始化BMA456传感器
    if (sensor.begin(readRegister, writeRegister, delayMs, 1, BMA4_I2C_ADDR_PRIMARY, true, 4, 35, 456)) {
        Serial.println("BMA456初始化成功");
        // 设置默认配置
        if (sensor.defaultConfig(true)) {
            Serial.println("BMA456默认配置设置成功");
        } else {
            Serial.println("BMA456默认配置设置失败");
        }

        // 启用双击唤醒功能
        if (sensor.enableDoubleClickWake(true)) {
            Serial.println("双击唤醒功能已启用");
        } else {
            Serial.println("双击唤醒功能启用失败");
        }

        // 启用翻转唤醒功能
        if (sensor.enableTiltWake(true)) {
            Serial.println("翻转唤醒功能已启用");
        } else {
            Serial.println("翻转唤醒功能启用失败");
        }

        // 启用步数计数功能
        if (sensor.enableFeature(BMA456_STEP_CNTR, true)) {
            Serial.println("步数计数功能已启用");
        } else {
            Serial.println("步数计数功能启用失败");
        }

        // 启用步数计数中断
        if (sensor.enableStepCountInterrupt(true)) {
            Serial.println("步数计数中断已启用");
        } else {
            Serial.println("步数计数中断启用失败");
        }
    } else {
        Serial.println("BMA456初始化失败");
    }
}

void loop() {
    // 读取加速度数据
    typedef struct bma4_accel Accel;
    Accel acc;
    if (sensor.getAccel(acc)) {
        Serial.print("加速度数据 - X: ");
        Serial.print(acc.x);
        Serial.print(", Y: ");
        Serial.print(acc.y);
        Serial.print(", Z: ");
        Serial.println(acc.z);
    } else {
        Serial.println("读取加速度数据失败");
    }

    // 读取温度数据
    float temperature = sensor.readTemperature(true);
    Serial.print("温度: ");
    Serial.print(temperature);
    Serial.println(" °C");

    // 更新中断状态
    if (sensor.getINT()) {
        // 检测双击事件
        if (sensor.isDoubleClick()) {
            Serial.println("检测到双击事件");
        }

        // 检测翻转事件
        if (sensor.isTilt()) {
            Serial.println("检测到翻转事件");
        }
    }

    // 读取步数
    uint32_t stepCount = sensor.getCounter();
    Serial.print("步数: ");
    Serial.println(stepCount);

    delay(1000);
}