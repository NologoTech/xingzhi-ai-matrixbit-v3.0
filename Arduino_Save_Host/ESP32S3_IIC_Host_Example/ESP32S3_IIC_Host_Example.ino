#include <Wire.h>
#include <vector>
using namespace std;

// 基类：I2C设备基础通信（替代ESP-IDF的I2cDevice）
class I2cDevice {
public:
    I2cDevice(uint8_t addr) : addr_(addr) {}
    virtual ~I2cDevice() {}

protected:
    // I2C写寄存器（核心通信函数）
    bool WriteReg(uint8_t reg, uint8_t data) {
        Wire.beginTransmission(addr_);
        Wire.write(reg);       // 发送寄存器地址
        Wire.write(data);      // 发送数据
        uint8_t err = Wire.endTransmission();
        if (err != 0) {
            Serial.printf("I2C写失败（寄存器0x%02X，错误码%d）\n", reg, err);
            return false;
        }
        return true;
    }

    // I2C读寄存器（核心通信函数）
    uint8_t ReadReg(uint8_t reg) {
        Wire.beginTransmission(addr_);
        Wire.write(reg);               // 发送要读取的寄存器地址
        uint8_t err = Wire.endTransmission(false);  // 不释放总线
        if (err != 0) {
            Serial.printf("I2C读地址失败（寄存器0x%02X，错误码%d）\n", reg, err);
            return 0xFF;  // 错误默认值
        }

        // 读取1字节数据
        if (Wire.requestFrom(addr_, (uint8_t)1) != 1) {
            Serial.printf("I2C读数据失败（寄存器0x%02X）\n", reg);
            return 0xFF;
        }
        return Wire.read();
    }

    uint8_t addr_;  // I2C设备地址
};

// Rp2040设备控制类（核心功能实现）
class Rp2040 : public I2cDevice {
public:
    // 构造函数：初始化I2C地址（默认0x55）
    Rp2040(uint8_t addr = 0x55) : I2cDevice(addr) {
        Serial.println("Rp2040 init success");
        uint8_t reg_25io_option = 1;
        WriteReg(0x66, reg_25io_option);  // 初始化IO配置
    }

    // 启动周期性读取（参数：间隔毫秒）
    void StartReading(unsigned long interval_ms = 100) {
        interval_ = interval_ms;
        last_read_time_ = millis();  // 记录首次启动时间
        reading_ = true;
        Serial.printf("开始周期性读取（间隔%dms）\n", interval_ms);
    }

    // 停止周期性读取
    void StopReading() {
        reading_ = false;
        Serial.println("停止周期性读取");
    }

    // 设置寄存器读取范围（原功能保留）
    void SetReadRegRange(uint8_t start_addr, uint8_t end_addr) {
        if (start_addr <= end_addr && end_addr <= 0xFF) {
            read_start_addr_ = start_addr;
            read_end_addr_ = end_addr;
            Serial.printf("设置读取范围：0x%02X-0x%02X\n", start_addr, end_addr);
        } else {
            Serial.println("无效的寄存器范围");
        }
    }

    // 读取IO状态（原功能保留）
    void ready_IO() {
        uint8_t data = ReadReg(0x01);
        Serial.printf("IO状态（寄存器0x01）：0x%02X\n", data);
    }

    // 读取多个寄存器（返回合并值，原逻辑保留）
    uint16_t ReadMultipleRegs(uint8_t start_addr, uint8_t end_addr) {
        if (start_addr > end_addr || end_addr > 0xFF) {
            Serial.println("无效的寄存器范围");
            return 0;
        }
        Serial.printf("读取寄存器 0x%02X-0x%02X：\n", start_addr, end_addr);
        for (uint8_t addr = start_addr; addr <= end_addr; addr++) {
            uint8_t data = ReadReg(addr);
            Serial.printf("寄存器0x%02X：0x%02X\n", addr, data);
        }
        // 合并0x15和0x16寄存器值（原逻辑）
        data_15 = ReadReg(0x15);
        data_16 = ReadReg(0x16);
        dataall = data_15 + data_16 * 255;  // 原计算逻辑
        return dataall;
    }

    // 位操作（原功能保留）
    void SetOutputStateAtAddr(uint8_t reg_addr, uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(reg_addr);
        data = (data & ~(1 << bit)) | (level << bit);  // 位修改
        WriteReg(reg_addr, data);
    }

    // 合并读取两个寄存器（原功能保留）
    uint16_t ReadCombinedRegs(uint8_t high_addr, uint8_t low_addr) {
        uint8_t high = ReadReg(high_addr);
        uint8_t low = ReadReg(low_addr);
        return (high << 8) | low;  // 高8位+低8位
    }

    // 控制RGB灯（原功能保留）
    void SetRGBColor(uint8_t red, uint8_t green, uint8_t blue) {
        WriteReg(0x12, red);    // 红色寄存器
        WriteReg(0x13, green);  // 绿色寄存器
        WriteReg(0x14, blue);   // 蓝色寄存器
        Serial.printf("RGB设置：R=%d, G=%d, B=%d\n", red, green, blue);
    }

    // 设置亮度（原功能保留）
    void SetBrightness(uint8_t brightness) {
        WriteReg(0x12, brightness);  // 红色通道亮度（示例）
        Serial.printf("亮度设置：%d\n", brightness);
    }

    // 读取温度（原功能保留）
    uint8_t GetTemperature() {
        return ReadReg(0x15);  // 温度寄存器（假设）
    }

    // 读取光线强度（原功能保留）
    uint8_t GetLightIntensity() {
        return ReadReg(0x16);  // 光线寄存器（假设）
    }

    // 设置PWM输出
    bool SetPwmOutput(uint8_t channel, uint8_t value) {
        if (channel < 0 || channel > 16) {
            Serial.println("PWM通道超出范围（0-16）");
            return false;
        }
        if (value > 255) {
            Serial.println("PWM值超出范围（0-255）");
            return false;
        }
        uint8_t reg = 0x20 + channel;  // PWM寄存器起始地址
        WriteReg(reg, value);
        Serial.printf("PWM通道%d（0x%02X）设置为%d\n", channel, reg, value);
        return true;
    }


    // 设置舵机角度（原功能保留）
    bool SetServoAngle(uint8_t servo_id, uint8_t angle) {
        if (servo_id < 1 || servo_id > 17) {
            Serial.println("舵机ID超出范围（1-17）");
            return false;
        }
        if (angle > 180) {
            Serial.println("舵机角度超出范围（0-180）");
            return false;
        }
        uint8_t reg = 0x30 + servo_id;  // 舵机寄存器起始地址
        WriteReg(reg, angle);
        Serial.printf("舵机%d（0x%02X）设置为%d度\n", servo_id, reg, angle);
        return true;
    }

    // 批量设置舵机角度（原功能保留）
    bool SetServoAngles(const vector<pair<uint8_t, uint8_t>>& servo_angles) {
        bool ok = true;
        for (const auto& p : servo_angles) {
            if (!SetServoAngle(p.first, p.second)) ok = false;
        }
        return ok;
    }

    // 周期性任务处理（替代原定时器回调）
    void HandlePeriodicTask() {
        if (!reading_) return;  // 未启动则跳过

        unsigned long now = millis();
        if (now - last_read_time_ < interval_) return;  // 未到间隔时间

        last_read_time_ = now;  // 更新上次执行时间
        OnReadTimer();  // 执行原定时器回调逻辑
    }

private:
    bool reading_ = false;               // 是否启动周期性读取
    unsigned long interval_ = 100;       // 周期性任务间隔（毫秒）
    unsigned long last_read_time_ = 0;   // 上次执行时间（用于millis()判断）
    uint8_t read_start_addr_ = 0x15;     // 读取起始寄存器
    uint8_t read_end_addr_ = 0x16;       // 读取结束寄存器

    // 原定时器回调中的业务逻辑（移植后保留）
    void OnReadTimer() {

        // 舵机控制：每周期增加60度，到180度重置
        // if (Servocount < 180) {
        //     Servocount += 60;
        // } else {
        //     Servocount = 0;
        // }


        //舵机控制：每周期增加60度，到180度重置
        if (Servocount  <180 ) {
            Servocount = 180;
        } else {
            Servocount = 0;
        }



        // 控制指定舵机（4、3、12号）
        SetServoAngle(9, Servocount);
        SetServoAngle(3, Servocount);
        SetServoAngle(12,Servocount);
        SetServoAngle(4, Servocount);
        // 读取光敏电阻数据（0x15-0x16）
        datadisplay = ReadMultipleRegs(0x15, 0x16);
        Serial.printf("光敏数据：%d\n", datadisplay);

        // RGB灯控制：光敏值<50时固定亮，否则流水灯
        if (datadisplay < 150) {
            SetRGBColor(10, 10, 10);  // 低光时固定亮度
            color_flag = 1;
        } else {
            color_flag = 0;
        }

        if (color_flag == 0) {
            // 流水灯逻辑（1-4循环切换颜色）
            color_count++;
            switch (color_count) {
                case 1: SetRGBColor(100, 0, 0); break;    // 红
                case 2: SetRGBColor(0, 100, 0); break;    // 绿
                case 3: SetRGBColor(0, 0, 100); break;    // 蓝
                case 4: SetRGBColor(20, 20, 20);          // 白
                        color_count = 0; break;
            }
        }
    }

    // 原全局变量转为类成员变量
    uint16_t dataall = 0;
    uint16_t datadisplay = 0;
    uint8_t data_15 = 0;
    uint8_t data_16 = 0;
    uint8_t Servocount = 0;
    int color_count = 0;
    int color_flag = 0;
};

// 全局Rp2040对象（在Arduino中方便调用）
Rp2040 rp2040;

void setup() {
    // 初始化串口（调试用）
    Serial.begin(115200);
    while (!Serial);  // 等待串口准备

    // 初始化I2C（Wire库默认SDA=21, SCL=22，可根据硬件修改）
    Wire.begin(1, 2);
    Wire.setClock(50000);  // 设置I2C时钟400KHz

    // 新增：检测I2C设备是否在线（以Rp2040的地址0x55为例）
    Wire.beginTransmission(0x55);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("I2C初始化异常：设备0x55未响应（错误码%d）\n", err);
        // 错误码含义：0=成功，1=数据溢出，2=地址发送失败，3=数据发送失败
    } else {
        Serial.println("I2C初始化成功，设备在线");
    }

    // 启动周期性任务（间隔3000ms=3秒，原逻辑中的3秒周期）
    rp2040.StartReading(1500);
}

void loop() {
    // 周期性处理任务（原逻辑保留）
    rp2040.HandlePeriodicTask();

    // ---------------------- PWM控制示例 ----------------------
    // 1. 初始化：开机时设置所有PWM通道为0（可选）
    static bool isFirstRun = true;
    if (isFirstRun) {
        // 初始化所有通道为0（避免上电后引脚电平不确定）
        for (int ch = 0; ch <= 16; ch++) {
            rp2040.SetPwmOutput(ch, 0);
        }
        isFirstRun = false;
        Serial.println("PWM初始化完成（所有通道置0）");
    }

    // 2. 循环设置不同通道的PWM值（示例：每2秒切换一次）
    static unsigned long lastPwmTime = 0;
    static int phase = 0; // 切换阶段标记
    if (millis() - lastPwmTime > 2000) { // 每2秒执行一次
        lastPwmTime = millis();

        switch (phase) {
            case 0:
                // 阶段0：设置通道0（0x20）为50%占空比（255*50%=128）
                rp2040.SetPwmOutput(0, 128);
                // 设置通道5（0x25）为20%占空比（51）
                rp2040.SetPwmOutput(5, 51);
                Serial.println("阶段0：通道0=128，通道5=51");
                break;
            case 1:
                // 阶段1：设置通道0为100%（255），通道5为0
                rp2040.SetPwmOutput(0, 255);
                rp2040.SetPwmOutput(5, 0);
                // 新增：设置通道10（0x2A）为70%（178）
                rp2040.SetPwmOutput(10, 178);
                Serial.println("阶段1：通道0=255，通道5=0，通道10=178");
                break;
            case 2:
                // 阶段2：所有通道复位为0
                for (int ch = 0; ch <= 16; ch++) {
                    rp2040.SetPwmOutput(ch, 0);
                }
                Serial.println("阶段2：所有通道复位为0");
                break;
        }

        // 切换阶段（0→1→2→0循环）
        phase = (phase + 1) % 3;
    }
}