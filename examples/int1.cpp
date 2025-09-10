#include <Arduino.h>
#include <StableBMA.h>
#include <Wire.h>
#include <esp_sleep.h>

// 1. 硬件配置
#define BMA_INT1_PIN    4           // BMA456 INT1 → ESP32 GPIO4
#define I2C_SDA_PIN     26          // ESP32 I2C SDA
#define I2C_SCL_PIN     27          // ESP32 I2C SCL
#define BMA_I2C_ADDR    0x18        // BMA456 I2C地址（AD0=GND）

// 2. 全局变量
StableBMA bma;
RTC_DATA_ATTR uint32_t wakeup_count = 0;  // 唤醒次数（RTC内存，不丢失）

// 3. I2C读写回调函数（适配StableBMA库）
short unsigned int bma_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    if (Wire.endTransmission(false) != 0) return 1;  // 通信失败

    if (Wire.requestFrom((int)dev_addr, (int)len) != len) return 1;  // 数据不完整
    for (int i = 0; i < len; i++) data[i] = Wire.read();
    return 0;  // 成功
}

short unsigned int bma_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    for (int i = 0; i < len; i++) Wire.write(data[i]);
    return (Wire.endTransmission() == 0) ? 0 : 1;  // 成功返回0
}

// 4. 延时回调函数（库要求）
void bma_delay(uint32_t ms) {
    delay(ms);
}

// 5. BMA456初始化（含中断配置）
bool bma456_init() {
    // 初始化I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);  // 100kHz I2C时钟
    delay(100);

    // 初始化BMA456（StableBMA库的begin4函数：5个参数）
    // 参数：atchyVersion=1, I2C地址, 型号=456, 读回调, 写回调
    if (!bma.begin4(1, BMA_I2C_ADDR, 456, bma_read, bma_write)) {
        Serial.println("❌ BMA456初始化失败");
        return false;
    }
    Serial.println("✅ BMA456初始化成功");

    // 配置默认参数（低功耗模式）
    if (!bma.defaultConfig(true)) {
        Serial.println("❌ 默认配置失败");
        return false;
    }
    Serial.println("✅ 低功耗模式配置成功");

    // 配置BMA456中断（关键：绑定双击/翻转到INT1）
    // 1. 启用双击中断（映射到INT1）
    if (!bma.enableDoubleClickWake(true)) {
        Serial.println("❌ 双击中断启用失败");
        return false;
    }
    // 2. 启用翻转中断（映射到INT1）
    if (!bma.enableTiltWake(true)) {
        Serial.println("❌ 翻转中断启用失败");
        return false;
    }
    Serial.println("✅ 双击/翻转中断已绑定到INT1");

    // 3. 配置中断极性（高电平触发，需与ESP32唤醒配置匹配）
    // 注意：StableBMA库可能通过寄存器配置极性，若默认低电平需修改此处
    uint8_t int_polarity = 0x01;  // 0x00=低电平，0x01=高电平
    uint8_t reg_data = int_polarity;
    if (bma_write(BMA_I2C_ADDR, 0x15, &reg_data, 1) != 0) {  // 0x15=INT1_CTRL寄存器
        Serial.println("❌ 中断极性配置失败");
        return false;
    }
    Serial.printf("✅ 中断极性配置为：%s电平触发\n", (int_polarity ? "高" : "低"));

    return true;
}

// 6. ESP32深度睡眠配置（启用INT1唤醒）
void esp32_sleep_config() {
    // 配置GPIO4为输入（上拉/下拉根据中断极性调整）
    // 若BMA456中断为高电平触发：用下拉输入（避免浮空误触发）
    pinMode(BMA_INT1_PIN, INPUT_PULLDOWN);
    delay(10);

    // 启用EXT1唤醒源（检测GPIO4的高电平）
    uint64_t wakeup_mask = 1ULL << BMA_INT1_PIN;
    esp_sleep_enable_ext1_wakeup(wakeup_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // 低功耗优化：关闭不必要的RTC外设
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    Serial.println("✅ ESP32睡眠配置完成，等待中断唤醒...");
}

// 7. 唤醒后事件处理
void handle_wakeup() {
    wakeup_count++;
    Serial.printf("\n===== 第%d次唤醒 =====\n", wakeup_count);

    // 读取BMA456中断事件
    if (bma.getINT()) {  // 检测到中断
        if (bma.isDoubleClick()) {
            Serial.println("📌 唤醒原因：双击事件");
        } else if (bma.isTilt()) {
            Serial.println("📌 唤醒原因：翻转事件");
        } else {
            Serial.println("📌 唤醒原因：未知中断");
        }
    } else {
        Serial.println("📌 唤醒原因：非BMA456中断");
    }

    // 读取加速度数据（唤醒后验证传感器状态）
    Accel acc;
    if (bma.getAccel(&acc)) {
        Serial.printf("📊 加速度：X=%d, Y=%d, Z=%d\n", acc.x, acc.y, acc.z);
    }

    // 读取步数（可选）
    uint32_t steps = bma.getCounter();
    Serial.printf("👣 当前步数：%d\n", steps);

    Serial.println("======================\n");
    delay(2000);  // 延时2秒，方便查看日志
}

// 8. 主函数
void setup() {
    // 初始化串口（唤醒后打印日志）
    Serial.begin(115200);
    delay(1500);  // 等待串口监视器连接

    // 初始化BMA456
    if (!bma456_init()) {
        Serial.println("❌ 系统初始化失败，停止运行");
        while (1) delay(1000);
    }

    // 处理唤醒事件（首次启动或中断唤醒）
    handle_wakeup();

    // 进入深度睡眠
    esp32_sleep_config();
    esp_deep_sleep_start();  // 此句后代码不执行
}

void loop() {
    // 深度睡眠时不执行loop
}
