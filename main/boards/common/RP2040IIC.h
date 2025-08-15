#ifndef RP2040IIC_H
#define RP2040IIC_H

#include "i2c_device.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include <functional>
#include <mutex>  // 用于线程安全

class Rp2040 : public I2cDevice {
public:
    // 禁止拷贝构造和赋值（单例禁止复制）
    Rp2040(const Rp2040&) = delete;
    Rp2040& operator=(const Rp2040&) = delete;

    // 单例全局访问点：首次调用需传入i2c_bus和addr，后续调用可省略
    static Rp2040* getInstance(i2c_master_bus_handle_t i2c_bus = nullptr, uint8_t addr = 0x55) {
        // 双重检查锁定（线程安全）
        
        //  WriteReg(0x66, 1);

        if (instance_ == nullptr) {
            std::lock_guard<std::mutex> lock(mutex_);  // 加锁防止并发创建
            if (instance_ == nullptr) {
                // 确保首次调用时传入有效i2c_bus
                if (i2c_bus == nullptr) {
                    return nullptr;  // 首次调用必须提供i2c_bus
                }
                instance_ = new Rp2040(i2c_bus, addr);
            }
        }
        return instance_;
    }

    // 销毁单例实例（可选，根据需求调用）
    static void destroyInstance() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (instance_ != nullptr) {
            delete instance_;
            instance_ = nullptr;
        }
    }

    // 原有公开方法保持不变
    esp_err_t StartReading(float interval_ms = 100);
    esp_err_t StopReading();
    void SetReadRegRange(uint8_t start_addr, uint8_t end_addr);
    void ready_IO();
    void SetBrightness(uint8_t brightness);
    uint16_t ReadMultipleRegs(uint8_t start_addr, uint8_t end_addr);
    void SetOutputStateAtAddr(uint8_t reg_addr, uint8_t bit, uint8_t level);
    uint16_t ReadCombinedRegs(uint8_t high_addr, uint8_t low_addr);
    void SetRGBColor(uint8_t red, uint8_t green, uint8_t blue);
    void io25_set_option();
    bool etPwmOutput(uint8_t channel, uint8_t value);
    uint8_t GetTemperature();
    uint8_t GetLightIntensity();
    // uint8_t getPwmOutput(uint8_t channel) const;
    bool SetPwmOutput(uint8_t channel, uint8_t value);
    bool SetServoAngle(uint8_t servo_id, uint8_t angle);
    bool SetServoAngles(const std::vector<std::pair<uint8_t, uint8_t>>& servo_angles);

private:
    // 私有化构造函数，仅内部可调用
    Rp2040(i2c_master_bus_handle_t i2c_bus, uint8_t addr);
    
    ~Rp2040() {
        // 析构时停止定时器，释放资源
        StopReading();
    }

    // 静态成员：单例实例和互斥锁
    static Rp2040* instance_;
    static std::mutex mutex_;

    // 原有私有成员保持不变
    bool reading_ = false;
    esp_timer_handle_t read_timer_handle_ = nullptr;
    uint8_t read_start_addr_ = 0x15;
    uint8_t read_end_addr_ = 0x16;
    uint8_t high_addr = 0x16;
    uint8_t low_addr = 0x15;
    uint8_t latest_temperature_ = 0;
    uint8_t latest_light_intensity_ = 0;

    // 内部使用的成员变量（从全局移到类内，避免全局变量）
    uint16_t dataall = 0;
    uint16_t datadisplay = 0;
    uint8_t data_15 = 0;
    uint8_t data_16 = 0;
    uint8_t Servocount = 0;
    uint16_t callback_count = 0;
    int color_count = 0;
    int color_flag = 0;

    static void ReadTimerCallback(void* arg);
    void OnReadTimer();
};

#endif