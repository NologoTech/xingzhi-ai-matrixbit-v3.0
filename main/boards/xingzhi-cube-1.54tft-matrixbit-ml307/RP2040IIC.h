#ifndef RP2040IIC_H
#define RP2040IIC_H

#include "i2c_device.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include <functional>

class Rp2040 : public I2cDevice {
public:

    Rp2040(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x55);

    esp_err_t StartReading(float interval_ms = 100);
    esp_err_t StopReading();

  

    void SetReadRegRange(uint8_t start_addr, uint8_t end_addr);

    void ready_IO();
    void SetBrightness(uint8_t brightness);
    // void ReadMultipleRegs(uint8_t start_addr, uint8_t end_addr);
    uint16_t ReadMultipleRegs(uint8_t start_addr, uint8_t end_addr);
    void SetOutputStateAtAddr(uint8_t reg_addr, uint8_t bit, uint8_t level);
    uint16_t ReadCombinedRegs(uint8_t high_addr, uint8_t low_addr);
    void SetRGBColor(uint8_t red, uint8_t green, uint8_t blue);
    void io25_set_option();
    bool etPwmOutput(uint8_t channel, uint8_t value);
    uint8_t GetTemperature();
    uint8_t GetLightIntensity();
    bool SetPwmOutput(uint8_t channel, uint8_t value);
    bool SetServoAngle(uint8_t servo_id, uint8_t angle);
    bool SetServoAngles(const std::vector<std::pair<uint8_t, uint8_t>>& servo_angles);

private:

    bool reading_ = false;
    esp_timer_handle_t read_timer_handle_ = nullptr;
    // 新增：存储周期性读取的寄存器范围（默认读取0x01到0x05）
    uint8_t read_start_addr_ = 0x15;
    uint8_t read_end_addr_ = 0x16;
    uint8_t high_addr = 0x16;
    uint8_t low_addr = 0x15;
    uint8_t latest_temperature_ = 0;
    uint8_t latest_light_intensity_ = 0;

    static void ReadTimerCallback(void* arg);
    void OnReadTimer();

};

#endif