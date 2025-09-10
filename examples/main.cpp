#include <Arduino.h>
#include <StableBMA.h>
#include <Wire.h>

// 定义I2C读写函数（适配BMA456的I2C通信）
short unsigned int readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) {
        return 1; // 通信错误
    }
    Wire.requestFrom((int)dev_addr, (int)len);
    for (int i = 0; i < len; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
        } else {
            return 1; // 数据读取不完整
        }
    }
    return 0; // 成功
}

short unsigned int writeRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (int i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    if (Wire.endTransmission() != 0) {
        return 1; // 写入失败
    }
    return 0; // 成功
}

// 延时函数（库要求的回调）
void delayMs(uint32_t period) {
    delay(period);
}

// 创建StableBMA对象
StableBMA bma;

void setup() {
    // 初始化串口（调试用）
    Serial.begin(115200);
    while (!Serial) {} // 等待串口就绪

    // 初始化I2C（SDA=21, SCL=22，可根据硬件修改）
    Wire.begin(26, 27);
    Serial.println("I2C初始化完成");

    // 初始化BMA456
    // 参数说明：atchyVersion=1, I2C地址=BMA4_I2C_ADDR_PRIMARY, 型号=456, 中断引脚=35
    if (bma.begin4(1, BMA4_I2C_ADDR_PRIMARY, 456, readRegister, writeRegister)) {
        Serial.println("BMA456初始化成功");

        // 配置默认参数（低功耗模式）
        if (bma.defaultConfig(true)) {
            Serial.println("默认配置设置成功（低功耗模式）");
        } else {
            Serial.println("默认配置设置失败");
        }

        // 启用双击唤醒功能
        if (bma.enableDoubleClickWake(true)) {
            Serial.println("双击唤醒功能已启用");
        } else {
            Serial.println("双击唤醒功能启用失败");
        }

        // 启用翻转检测功能
        if (bma.enableTiltWake(true)) {
            Serial.println("翻转检测功能已启用");
        } else {
            Serial.println("翻转检测功能启用失败");
        }

        // 启用步数计数功能
        if (bma.enableFeature(BMA456_STEP_CNTR, true)) {
            Serial.println("步数计数功能已启用");
        } else {
            Serial.println("步数计数功能启用失败");
        }

        // 启用步数中断
        if (bma.enableStepCountInterrupt(true)) {
            Serial.println("步数计数中断已启用");
        } else {
            Serial.println("步数计数中断启用失败");
        }

    } else {
        Serial.println("BMA456初始化失败，请检查硬件连接！");
        while (1) {} // 初始化失败时阻塞
    }
}

void loop() {
    // 读取加速度数据（原始值）
    Accel acc;
    if (bma.getAccel(&acc)) {
        Serial.print("加速度: X=");
        Serial.print(acc.x);
        Serial.print(", Y=");
        Serial.print(acc.y);
        Serial.print(", Z=");
        Serial.println(acc.z);
    } else {
        Serial.println("读取加速度失败");
    }

    // 读取温度（摄氏度）
    float temp = bma.readTemperature(true);
    Serial.print("温度: ");
    Serial.print(temp);
    Serial.println(" °C");

    // 检测中断事件
    if (bma.getINT()) { // 中断引脚触发
        // 检测双击事件
        if (bma.isDoubleClick()) {
            Serial.println("检测到双击事件！");
        }
        // 检测翻转事件
        if (bma.isTilt()) {
            Serial.println("检测到翻转事件！");
        }
    }

    // 读取当前步数
    uint32_t steps = bma.getCounter();
    Serial.print("当前步数: ");
    Serial.println(steps);

    delay(1000); // 1秒刷新一次
}
