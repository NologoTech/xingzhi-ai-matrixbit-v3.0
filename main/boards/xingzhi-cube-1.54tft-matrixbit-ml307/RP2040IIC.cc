#include "rp2040iic.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "PRP2040IIC"

//  class Rp2040 : public I2cDevice {
// public:
uint16_t dataall = 0;
uint16_t datadisplay = 0;
uint8_t data_15 = 0;
uint8_t data_16 = 0;
uint8_t Servocount = 0;
Rp2040::Rp2040(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {

    // for (int i = 0; i < 4; i++) {
    //     uint8_t reg_addr = 0x01 + i;
    //     uint8_t reg_data = 0x01 + i;
    //     WriteReg(reg_addr, reg_data);
    //     printf("Write 0x%02X to register 0x%02X\n", reg_data, reg_addr);
    // }

        printf("rp2040 init success\n");
        uint8_t reg_25io_option = 1;
        WriteReg(0x66, reg_25io_option);
    // ReadMultipleRegs(0x01, 0x05);


 }

//创建定时器
 esp_err_t Rp2040::StartReading(float interval_ms) {
    if (reading_) {
        return ESP_OK; // 已经在读取中
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
        return ESP_OK; // 已经停止
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

// 静态定时器回调函数
void Rp2040::ReadTimerCallback(void* arg) {
    Rp2040* sensor = static_cast<Rp2040*>(arg);
    sensor->OnReadTimer();
}
uint16_t callback_count = 0;
int color_count = 0;
int color_flag = 0;
// 定时器触发的读取操作 oycation 3s //定时器3s读取每隔三秒rgb变换颜色，舵机转动60度 目前可控制的舵机为4 -3 -12舵机其他舵机存在被占用不可操控 多舵机会导致舵机转动机器重启
void Rp2040::OnReadTimer() {
    // if(Servocount < 180)
    // {
    //     // Servocount = Servocount + 60;
    //     //     // SetServoAngle(9,Servocount);
    //     //     //SetServoAngle(7,Servocount); //LED_PIN
    //     //     //  SetServoAngle(4,Servocount);
    //     //      SetServoAngle(3,Servocount);
    //     //      SetServoAngle(12,Servocount);
    // }
    // else{
    //     Servocount = 0;
    // }
            //读取所有寄存器
            // 调用ReadMultipleRegs，使用配置的寄存器范围（默认0x01到0x05）datadisplay光敏电阻的值

            // datadisplay = ReadMultipleRegs(0x15, 0x16);

            // ESP_LOGE("TAG","datadisplay = %d",datadisplay);

            // //etPwmOutput(1, 255); 需要写入的
            // if(datadisplay <50)
            // {
            //     SetRGBColor(10, 10, 10); //调节rgb参数
            //     color_flag = 1;
            // }
            // else
            // {
            //     color_flag = 0;
            // }

            // if(color_flag == 0)
            // {
            //     //流水灯
            //     color_count++;
            //     if(color_count == 1)
            //     {
            //         SetRGBColor(10, 0, 0);
            //     }
            //     else if(color_count == 2)
            //     {
            //         SetRGBColor(0, 10, 0);
            //     }
            //     else if(color_count == 3)
            //     {
            //         SetRGBColor(0, 0, 10);
            //     }
            //     else if(color_count == 4)
            //     {
            //         SetRGBColor(10, 10, 10);
            //         color_count = 0;
            //     }
            // }
}

//实现寄存器范围设置
void Rp2040::SetReadRegRange(uint8_t start_addr, uint8_t end_addr) {
    if (start_addr <= end_addr && end_addr <= 0xFF) {
        read_start_addr_ = start_addr;
        read_end_addr_ = end_addr;
        ESP_LOGI(TAG, "已设置寄存器读取范围：0x%02X-0x%02X", start_addr, end_addr);
    } else {
        ESP_LOGE(TAG, "无效的寄存器范围（0x%02X-0x%02X），范围需满足start <= end且<=0xFF", start_addr, end_addr);
    }
}

        void Rp2040::ready_IO()
        {
            uint8_t  Data_iicready =  ReadReg(0x01);
            printf("Data_iicready = 0x%02X\n", Data_iicready);

        }
        void Rp2040::io25_set_option()
        {
           WriteReg(0x66, 1);
        }

        uint16_t Rp2040::ReadMultipleRegs(uint8_t start_addr, uint8_t end_addr) {
            
        if (start_addr > end_addr || end_addr > 0xFF) {
            printf("Invalid register range\n");
            ESP_LOGE(TAG, "地址异常");
        }
        
        printf("Reading registers from 0x%02X to 0x%02X:\n", start_addr, end_addr);
        
        for (uint8_t addr = start_addr; addr <= end_addr; addr++) {
            uint8_t data = ReadReg(addr);
            printf("Register 0x%02X: 0x%02X\n", addr, data);
        }

        data_15 = ReadReg(0x15);
        data_16 = ReadReg(0x16);

        dataall =(ReadReg(0x15)+ReadReg(0x16)*255);
        return dataall;
    }

        // 基础位操作函数 - 指定寄存器地址版本  极性寄存器 0x06-0x0B
        void Rp2040:: SetOutputStateAtAddr(uint8_t reg_addr, uint8_t bit, uint8_t level) {
            uint8_t data = ReadReg(reg_addr);
            data = (data & ~(1 << bit)) | (level << bit);
            WriteReg(reg_addr, data);
        }

    // 新增：合并读取两个寄存器为16位值
    uint16_t Rp2040:: ReadCombinedRegs(uint8_t high_addr, uint8_t low_addr) {
        uint8_t high_byte = ReadReg(high_addr);
        uint8_t low_byte = ReadReg(low_addr);
        // return (high_byte << 8) | low_byte;
        return (high_byte << 8) | low_byte;
        ESP_LOGE(TAG, "读取两个寄存器0x%02X和0x%02X合并为16位值：0x%04X", high_addr, low_addr, (high_byte << 8) | low_byte);
    }

    // 新增：控制RGB灯
    void Rp2040:: SetRGBColor(uint8_t red, uint8_t green, uint8_t blue) {
        WriteReg(0x12, red);    // 设置红色分量
        WriteReg(0x13, green);  // 设置绿色分量
        WriteReg(0x14, blue);   // 设置蓝色分量
        
        printf("RGB颜色已设置 - 红: 0x%02X, 绿: 0x%02X, 蓝: 0x%02X\n", red, green, blue);
    }

    void Rp2040:: SetBrightness(uint8_t brightness) {
        WriteReg(0X12, brightness);  // 设置亮度
        
        printf("红色值 - 0x%02X\n", brightness);
    }


        //增加：读取芯片温度
    uint8_t Rp2040:: GetTemperature() {
        return ReadReg(0x15); // 假设温度数据存储在0x15寄存器
    }

      //读取光线值
          // 增加：读取光线强度
    uint8_t Rp2040:: GetLightIntensity() {
        return ReadReg(0x16); // 假设光线数据存储在0x16寄存器
    }

    // 设置PWM输出
    bool Rp2040:: etPwmOutput(uint8_t channel, uint8_t value) {
        if (channel < 0 || channel > 16) {
            printf("PWM通道号超出范围 (0-16)\n");
            return false;
        }
        
        if (value < 0 || value > 255) {
            printf("PWM值超出范围 (0-255)\n");
            return false;
        }
        
        uint8_t reg_addr = 0x20 + channel;
        WriteReg(reg_addr, value);
        
        printf("PWM通道 %d (寄存器0x%02X) 设置为: 0x%02X (%d%%)\n", 
               channel, reg_addr, value, (value * 100) / 255);
        
        return true;
    }

    // 设置单个舵机角度
bool Rp2040:: SetServoAngle(uint8_t servo_id, uint8_t angle) {
    if (servo_id < 1 || servo_id > 17) {
        printf("舵机ID超出范围 (1-17)\n");
        return false;
    }
    
    if (angle < 0 || angle > 180) {
        printf("舵机角度超出范围 (0-180度)\n");
        return false;
    }
    
    uint8_t reg_addr = 0x30 + servo_id; // 寄存器地址 0x31 到 0x41
    WriteReg(reg_addr, angle);
    
    printf("舵机 %d (寄存器0x%02X) 设置为: %d 度\n", 
           servo_id, reg_addr, angle);
    
    return true;
}

// 批量设置多个舵机角度
bool Rp2040:: SetServoAngles(const std::vector<std::pair<uint8_t, uint8_t>>& servo_angles) {
    bool all_success = true;
    
    for (const auto& pair : servo_angles) {
        uint8_t servo_id = pair.first;
        uint8_t angle = pair.second;
        
        if (!SetServoAngle(servo_id, angle)) {
            all_success = false;
        }
    }
    
    return all_success;
}

//  };