#include "rp2040iic.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <mutex>

#define TAG "PRP2040IIC"

// 初始化静态成员
Rp2040* Rp2040::instance_ = nullptr;
std::mutex Rp2040::mutex_;

// 私有构造函数实现（原构造函数逻辑不变）
Rp2040::Rp2040(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
    printf("rp2040 init success\n");
    uint8_t reg_25io_option = 1;
    WriteReg(0x66, reg_25io_option);
}

// 原有方法实现保持不变（仅调整全局变量为类内成员）
esp_err_t Rp2040::StartReading(float interval_ms) {
    if (reading_) {
        return ESP_OK;
    }
    
    esp_timer_create_args_t timer_args = {
        .callback = &Rp2040::ReadTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sc7a20h_read_timer",
        .skip_unhandled_events = true
    };
    
    esp_err_t err = esp_timer_create(&timer_args, &read_timer_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建定时器失败 (err=0x%x)", err);
        return err;
    }
    
    err = esp_timer_start_periodic(read_timer_handle_, (uint64_t)(interval_ms * 1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动定时器失败 (err=0x%x)", err);
        esp_timer_delete(read_timer_handle_);
        read_timer_handle_ = nullptr;
        return err;
    }
    
    reading_ = true;
    ESP_LOGI(TAG, "开始周期性读取，间隔 %.1f ms", interval_ms);
    return ESP_OK;
}

esp_err_t Rp2040::StopReading() {
    if (!reading_) {
        return ESP_OK;
    }
    
    if (read_timer_handle_) {
        esp_timer_stop(read_timer_handle_);
        esp_timer_delete(read_timer_handle_);
        read_timer_handle_ = nullptr;
    }
    
    reading_ = false;
    ESP_LOGI(TAG, "停止周期性读取");
    return ESP_OK;
}

void Rp2040::ReadTimerCallback(void* arg) {
    Rp2040* sensor = static_cast<Rp2040*>(arg);
    sensor->OnReadTimer();
}

void Rp2040::OnReadTimer() {
    //原逻辑不变，注意全局变量已改为类内成员（如Servocount、color_count等）
    // if(Servocount < 180) {
    //     Servocount += 60;
    //     SetServoAngle(3, Servocount);
    //     SetServoAngle(12, Servocount);
    //     SetServoAngle(9, Servocount);
    //     SetServoAngle(26, Servocount);
    // } else {
    //     Servocount = 0;
    // }

    datadisplay = (ReadMultipleRegs(0x15, 0x16) - 5610);
    ESP_LOGE("TAG","datadisplay = %d", datadisplay);

    if(datadisplay<100)
    {
        // etPwmOutput(17, 255);
        // etPwmOutput(18, 0);

        // etPwmOutput(19, 255);
        // etPwmOutput(22, 0);
        // SetRGBColor(5, 0, 0); 
    }
    else
    {

        // etPwmOutput(17, 0);
        // etPwmOutput(18, 255);

        // etPwmOutput(22, 255);
        // etPwmOutput(19, 0);
        // SetRGBColor(5, 0, 0);
    }

     

    // if(datadisplay < 50) {
    //     SetRGBColor(255, 255, 255);
    //     color_flag = 1;
    // } else {
    //     color_flag = 0;
    // }

    // if(color_flag == 0) {
    //     color_count++;
    //     if(color_count == 1) {
    //         SetRGBColor(10, 0, 0);
    //     } else if(color_count == 2) {
    //         SetRGBColor(0, 10, 0); 
    //     } else if(color_count == 3) {
    //         SetRGBColor(0, 0, 10);
    //     } else if(color_count == 4) {
    //         SetRGBColor(10, 10, 10);
    //         color_count = 0;
    //     }
    // }
}

void Rp2040::SetReadRegRange(uint8_t start_addr, uint8_t end_addr) {
    if (start_addr <= end_addr && end_addr <= 0xFF) {
        read_start_addr_ = start_addr;
        read_end_addr_ = end_addr;
        ESP_LOGI(TAG, "已设置寄存器读取范围：0x%02X-0x%02X", start_addr, end_addr);
    } else {
        ESP_LOGE(TAG, "无效的寄存器范围（0x%02X-0x%02X）", start_addr, end_addr);
    }
}

void Rp2040::ready_IO() {
    uint8_t Data_iicready = ReadReg(0x01);
    printf("Data_iicready = 0x%02X\n", Data_iicready);
}

void Rp2040::io25_set_option() {
    WriteReg(0x66, 1);
}

uint16_t Rp2040::ReadMultipleRegs(uint8_t start_addr, uint8_t end_addr) {
    if (start_addr > end_addr || end_addr > 0xFF) {
        printf("Invalid register range\n");
        ESP_LOGE(TAG, "地址异常");
        return 0;
    }
    
    printf("Reading registers from 0x%02X to 0x%02X:\n", start_addr, end_addr);
    
    for (uint8_t addr = start_addr; addr <= end_addr; addr++) {
        uint8_t data = ReadReg(addr);
        printf("Register 0x%02X: 0x%02X\n", addr, data);
    }

    data_15 = ReadReg(0x15);
    data_16 = ReadReg(0x16);
    dataall = (data_15 + data_16 * 255);  // 注意：原代码此处可能有误（应为data_16<<8 + data_15）
    return dataall;
}

void Rp2040::SetOutputStateAtAddr(uint8_t reg_addr, uint8_t bit, uint8_t level) {
    uint8_t data = ReadReg(reg_addr);
    data = (data & ~(1 << bit)) | (level << bit);
    WriteReg(reg_addr, data);
}

uint16_t Rp2040::ReadCombinedRegs(uint8_t high_addr, uint8_t low_addr) {
    uint8_t high_byte = ReadReg(high_addr);
    uint8_t low_byte = ReadReg(low_addr);
    uint16_t result = (high_byte << 8) | low_byte;
    ESP_LOGE(TAG, "读取两个寄存器0x%02X和0x%02X合并为16位值：0x%04X", high_addr, low_addr, result);
    return result;
}

void Rp2040::SetRGBColor(uint8_t red, uint8_t green, uint8_t blue) {
    WriteReg(0x12, red);
    WriteReg(0x13, green);
    WriteReg(0x14, blue);
    printf("RGB颜色已设置 - 红: 0x%02X, 绿: 0x%02X, 蓝: 0x%02X\n", red, green, blue);
}

void Rp2040::SetBrightness(uint8_t brightness) {
    WriteReg(0X12, brightness);
    printf("红色值 - 0x%02X\n", brightness);
}

uint8_t Rp2040::GetTemperature() {
    return ReadReg(0x15);
}

uint8_t Rp2040::GetLightIntensity() {
    return ReadReg(0x16);
}

bool Rp2040::etPwmOutput(uint8_t channel, uint8_t value) {
    if (channel < 0 || channel > 30) {
        printf("PWM通道号超出范围 (0-30)\n");
        return false;
    }
    if (value < 0 || value > 255) {
        printf("PWM值超出范围 (0-255)\n");
        return false;
    }
    uint8_t reg_addr = 0x10 + channel;
    WriteReg(reg_addr, value);
    printf("PWM通道 %d (寄存器0x%02X) 设置为: 0x%02X (%d%%)\n", 
           channel, reg_addr, value, (value * 100) / 255);
    return true;
}

// uint8_t Rp2040::getPwmOutput(uint8_t channel) {
//     // 检查通道号是否在有效范围内
//     if (channel < 0 || channel > 30) {
//         printf("PWM通道号超出范围 (0-30)\n");
//         return 0; // 返回0表示错误
//     }
    
//     // 计算对应的寄存器地址（与etPwmOutput保持一致）
//     uint8_t reg_addr = 0x10 + channel;
    
//     // 读取该寄存器的值
//     uint8_t value = ReadReg(reg_addr);
    
//     // 打印调试信息
//     printf("读取PWM通道 %d (寄存器0x%02X) 当前值: 0x%02X (%d%%)\n", 
//            channel, reg_addr, value, (value * 100) / 255);
           
//     return value;
// }

bool Rp2040::SetServoAngle(uint8_t servo_id, uint8_t angle) {
    if (angle < 0 || angle > 180) {
        printf("舵机角度超出范围 (0-180度)\n");
        return false;
    }
    uint8_t reg_addr = 0x30 + servo_id;
    WriteReg(reg_addr, angle);
    printf("舵机 %d (寄存器0x%02X) 设置为: %d 度\n", servo_id, reg_addr, angle);
    return true;
}

bool Rp2040::SetServoAngles(const std::vector<std::pair<uint8_t, uint8_t>>& servo_angles) {
    bool all_success = true;
    for (const auto& pair : servo_angles) {
        if (!SetServoAngle(pair.first, pair.second)) {
            all_success = false;
        }
    }
    return all_success;
}