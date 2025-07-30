#include "uartcmdsend.h"
#include "esp_log.h"
#include "esp_system.h"
#include <functional>
#include <cstring> // 添加这一行，包含memmove函数的声明
#include <string>
#include "application.h"
#include "assets/lang_config.h"
#include "ssid_manager.h"
// using namespace MyNamespace; // 在文件顶部声明命名空间
// 或直接使用作用域运算符
// app.PlaySound(MyNamespace::Lang::Sounds::P3_CHARGING);
#include "wifi_station.h"
#include "esp_wifi.h"
#include "esp_event.h"


#include "freertos/event_groups.h"

// 定义事件组和位
static EventGroupHandle_t wifi_event_group = NULL;
#define CONNECTED_BIT BIT0


// 静态成员变量初始化
UartSender *UartSender::instance_ = nullptr;

// 在文件顶部或合适的全局位置
std::string UartSender::ascii_str_part1;
std::string UartSender::ascii_str_part2;

static const char *TAG = "UART_SENDER_1";
static wifi_config_t sta_config;
static bool gl_sta_got_ip = false;
static bool ssid_sta = false;
static bool passd_sta = false;
int Volince_flag = 0;
int Volince_count = 0;
// 在uartcmdsend.cc文件的全局作用域中添加
// std::string UartSender::static_ascii_str_part1;
// std::string UartSender::static_ascii_str_part2;

// 构造函数实现
UartSender::UartSender(uart_port_t uart_num, const uart_config_t &uart_config)
    : uart_num_(uart_num), uart_config_(uart_config), rx_buffer_index_(0), last_rx_time_(0),
      frame_state_(FrameState::IDLE), frame_buffer_index_(0)
{

    // 配置 UART 参数
    ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config_));

    // 设置 UART 引脚
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, ECHO_TEST_TXD, ECHO_TEST_RXD,
                                 ECHO_TEST_RTS, ECHO_TEST_CTS));

    // 安装 UART 驱动
    ESP_ERROR_CHECK(uart_driver_install(uart_num_, 256, 0, 0, nullptr, 0));

    ESP_LOGI(TAG, "UART initialized on port %d", uart_num_);


    // 创建定时器用于周期性检查接收数据
    esp_timer_create_args_t clock_timer_args = {
        .callback = [](void *arg)
        {
            UartSender *sender = static_cast<UartSender *>(arg);
            sender->checkForReceivedData();
            // if (Volince_flag == 1)
            // {
            //    Volince_count++;
            //    if(Volince_count == 1)  //5s
            //    {
            //       Volince_flag = 0;
            //       Volince_count = 0;
            //       printf("Volince_flag = 0\n");
            //    }
            // }
     
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "clock_timer",
        .skip_unhandled_events = true};
    esp_timer_create(&clock_timer_args, &clock_timer_handle_);
    esp_timer_start_periodic(clock_timer_handle_, 1000000); 

    // 音频文件
    //  auto& app = Application::GetInstance();
    //  app.ResetDecoder();
    //  app.PlaySound(Lang::Sounds::P3_CHARGING); // 开始充电音效

    // 设置默认回调函数
    setReceiveCallback([](const std::vector<uint8_t> &data)
                       {
                           // 默认空回调
                       });
}

// 析构函数实现
UartSender::~UartSender()
{
    if (instance_)
    {
        // 停止并删除定时器
        if (clock_timer_handle_)
        {
            esp_timer_stop(clock_timer_handle_);
            esp_timer_delete(clock_timer_handle_);
            clock_timer_handle_ = nullptr;
        }

        // 卸载 UART 驱动
        uart_driver_delete(uart_num_);
        delete instance_;
        instance_ = nullptr;
        ESP_LOGI(TAG, "UART driver deleted");
    }
}

// 初始化方法实现
void UartSender::Initialize(uart_port_t uart_num, const uart_config_t &uart_config)
{
    if (!instance_)
    {
        instance_ = new UartSender(uart_num, uart_config);
    }
    else
    {
        ESP_LOGW(TAG, "UART already initialized on port %d", instance_->uart_num_);
    }
}

// 获取实例方法实现
UartSender &UartSender::GetInstance()
{
    if (!instance_)
    {
        // 处理未初始化情况
        ESP_LOGE(TAG, "UART not initialized! Call Initialize() first.");

        // 创建默认配置
        uart_config_t default_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
        };

        // 使用 UART0 作为默认端口
        UartSender::Initialize(UART_NUM_1, default_config);
    }
    return *instance_;
}

// 发送字符串实现
void UartSender::sendString(const std::string &send_data)
{
    if (!instance_)
    {
        ESP_LOGE(TAG, "UART not initialized!");
        return;
    }
    // 发送数据
    int bytes_sent = uart_write_bytes(uart_num_, send_data.c_str(), send_data.length());

    if (bytes_sent >= 0)
    {
        // ESP_LOGI(TAG, "Sent %d bytes: %s", bytes_sent, send_data.c_str());
    }
    else
    {
        ESP_LOGE(TAG, "Failed to send data");
    }
}

// 发送数据实现 用作iot
void UartSender::sendNumber(int number)
{
    if (!instance_)
    {
        ESP_LOGE(TAG, "UART not initialized!");
        return;
    }

    // 将数字转换为字节数据
    uint8_t byteData = static_cast<uint8_t>(number);

    // 发送单个字节
    int bytes_sent = uart_write_bytes(uart_num_, &byteData, 1);

    if (bytes_sent >= 0)
    {
        ESP_LOGI(TAG, "Sent number: %d", number);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to send number");
    }
}

// 机器狗特定指令发送 crc数据校验
void UartSender::sendRealTimeData(const std::vector<uint8_t> &realTimeData)
{
    if (!instance_)
    {
        ESP_LOGE(TAG, "UART not initialized!");
        return;
    }

    // 构建实时数据帧结构：起始符（0x3A） + 数据 + CRC + 结束符（0x7E）
    std::vector<uint8_t> realTimeFrame;
    realTimeFrame.push_back(0x3A); // 起始符

    // 校验数据有效性（至少包含1字节数据）
    if (realTimeData.empty())
    {
        ESP_LOGE(TAG, "Real-time data is empty!");
        return;
    }

    // 添加数据部分
    realTimeFrame.insert(realTimeFrame.end(), realTimeData.begin(), realTimeData.end());

    // 计算CRC（仅对数据部分校验）
    uint8_t crc = calculateCRC(realTimeData.data(), realTimeData.size());
    realTimeFrame.push_back(crc);  // 添加CRC字节
    realTimeFrame.push_back(0x7E); // 结束符

    // 发送帧数据
    int bytes_sent = uart_write_bytes(uart_num_, realTimeFrame.data(), realTimeFrame.size());
    if (bytes_sent >= 0)
    {
        ESP_LOGI(TAG, "Sent real-time frame: %d bytes", bytes_sent);
        // 打印发送的字节数据（调试用）
        for (uint8_t b : realTimeFrame)
        {
            // printf("RealTime Sent: 0x%02X ", b);
            printf("0x%02X ", b);
        }
        printf("\n");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to send real-time data");
    }
}

// 设置接收回调函数
void UartSender::setReceiveCallback(std::function<void(const std::vector<uint8_t> &)> callback)
{
    receive_callback_ = callback;
}

// 初始化GPIO引脚
void UartSender::initGpio() {
    // 配置AUDIO_CODEC_PA_PIN为输出模式
    gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "GPIO initialized for AUDIO_CODEC_PA_PIN: %d", GPIO_NUM_21);
}

// 在 UartSender 类中添加回发方法 先接收后发送
void UartSender::sendFrame(const std::vector<uint8_t> &data)
{
    if (!instance_)
        return;

    // 构建帧结构：起始符（0x3A） + 数据 + CRC + 结束符（0x7E）
    std::vector<uint8_t> frame;
    frame.push_back(0x3A); // 起始符

    // frame.insert(frame.end(), data.begin(), data.end()); // 数据部分 //将所有的data数据放入frame中
    frame.insert(frame.end(), data.begin(), data.end());
    // 计算 CRC（数据部分）
    uint8_t crc = calculateCRC(data.data(), data.size());

    frame.push_back(crc);  // 添加 CRC 字节
    frame.push_back(0x7E); // 结束符

    // 发送帧数据
    int bytes_sent = uart_write_bytes(uart_num_, frame.data(), frame.size());
    if (bytes_sent >= 0)
    {
        ESP_LOGI(TAG, "Sent frame: %d bytes", bytes_sent);
        // 打印发送的字节数据（可选）
        for (uint8_t b : frame)
        {
            printf("Sent: 0x%02X ", b);
        }
        printf("\n");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to send frame");
    }
}

// 在uartcmdsend.cpp文件中实现这个方法
void UartSender::sendCustomData(const std::vector<uint8_t> &customData)
{
    if (!instance_)
    {
        ESP_LOGE(TAG, "UART not initialized!");
        return;
    }

    // 直接使用现有的sendFrame方法，它已经实现了帧封装和CRC校验
    sendFrame(customData);
}


// 检查并处理接收到的数据
void UartSender::checkForReceivedData()
{
    if (!instance_)
    {
        ESP_LOGE(TAG, "UART not initialized!");
        return;
    }

    // 获取当前时间
    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒

    // 检查接收缓冲区中的数据
    size_t rx_bytes = 0;
    esp_err_t err = uart_get_buffered_data_len(uart_num_, &rx_bytes);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_get_buffered_data_len failed: %d", err);
        return;
    }

    if (rx_bytes > 0)
    {
        // 读取数据
        uint8_t data[BUF_SIZE];
        int length = uart_read_bytes(uart_num_, data, rx_bytes, 0);
        if (length > 0)
        {
            // 更新最后接收时间
            last_rx_time_ = current_time;

            for (int i = 0; i < length; i++)
            {
                //printf("data[%d] = %d (0x%02X)\n", i, data[i], (unsigned char)data[i]);
            }

            // 处理接收到的字节流，查找帧
            for (int i = 0; i < length; i++)
            {
                processByte(data[i]);
            }
        }
    }
    else if (frame_buffer_index_ > 0 && (current_time - last_rx_time_) > UART_RX_TIMEOUT)
    {
        // 超时处理：重置帧解析状态
        ESP_LOGW(TAG, "Frame receive timeout, resetting parser");
        resetFrameParser();
    }
}


// 计算CRC校验值（所有字节相加后&0xFF）
uint8_t UartSender::calculateCRC(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    printf("length = %d\n", length);
    // 打印所有数据
    for (size_t i = 0; i < length; i++)
    {
        printf("jiaoyandata[%d] = %d (0x%02X)\n", i, data[i], (unsigned char)data[i]);
    }

    for (size_t i = 0; i < length; i++)
    {
        // crc += (unsigned char)data[i];
        crc += data[i];
    }
    return crc & 0xFF;
}
int Volincelisting_flag = 0;
int Vonum_flag = 0;
int Uart_flag = 0;
int Passed_flag = 0;
bool connectResult = 0;
// 根据命令值发送不同的响应数据
void UartSender::sendResponseByCommand(uint8_t cmd, uint8_t cmd4)
{
    //  ESP_LOGE(TAG, "into!");
    
    // std::vector<uint8_t> responseData;

    // // 根据接收到的cmd值构建不同的响应数据
    // switch (cmd)
    // {
    // // case 0x01:                                                           // 命令码0x01
    // //     responseData = {0x03, 0x81, 0x31, 0x00, 0x0B, 0x31, 0x2e, 0x30}; // 协议内容1.0版本 step1
    // //     break;
    // case 0x02: //账号密码
    // {

    //     // 获取不含CRC的有效数据
    //     std::vector<uint8_t> dataWithoutCRC(frame_buffer_, frame_buffer_ + frame_buffer_index_ - 1);
    //     // 查找0x2e在dataWithoutCRC中的位置
    //     auto dot_it = std::find(dataWithoutCRC.begin(), dataWithoutCRC.end(), 0x2e);
    //     // 提取第五个数据之后到0x2e之前的数据(索引4之后到dot_it之间)
    //     std::vector<uint8_t> hex_bytes_part1; // 用户名的值
    //     if (dataWithoutCRC.size() > 5 && dot_it != dataWithoutCRC.end() && dot_it > dataWithoutCRC.begin() + 4)
    //     {
    //         hex_bytes_part1.assign(dataWithoutCRC.begin() + 5, dot_it);

    //         ascii_str_part1 = UartSender::HexBytesToAscii(hex_bytes_part1);
    //         //标志位
            
    //         ESP_LOGW(TAG, "Part1 ASCII: %s", ascii_str_part1.c_str());
    //         ssid_sta = 1;

    //         //strncpy(reinterpret_cast<char*>(sta_config.sta.ssid), ascii_str_part1.c_str(), ascii_str_part1.length());

    //         // 方法2：使用更安全的字符串复制函数
    //         snprintf(reinterpret_cast<char*>(sta_config.sta.ssid), sizeof(sta_config.sta.ssid), "%s", ascii_str_part1.c_str());

    //         //strncpy(reinterpret_cast<char*>(sta_config.sta.ssid), "Onion3", strlen("Onion3"));
    //     }
    //     else
    //     {
    //         ESP_LOGW(TAG, "未找到有效Part1数据");
    //     }

    //     // 提取0x2e之后的数据
    //     std::vector<uint8_t> hex_bytes_part2; // 密码的值
    //     if (dot_it != dataWithoutCRC.end() && std::next(dot_it) != dataWithoutCRC.end())
    //     {
    //         hex_bytes_part2.assign(std::next(dot_it), dataWithoutCRC.end());

    //         ascii_str_part2 = UartSender::HexBytesToAscii(hex_bytes_part2);
    //         ESP_LOGW(TAG, "Part2 ASCII: %s", ascii_str_part2.c_str());
    //           //标志位
    //         passd_sta = 1;
    //         //strncpy(reinterpret_cast<char*>(sta_config.sta.password), "asdfghjkl", strlen("asdfghjkl"));
    //         //  strncpy(reinterpret_cast<char*>(sta_config.sta.password), ascii_str_part2.c_str(),  ascii_str_part2.length());
    //         // 方法2：使用更安全的字符串复制函数
    //         snprintf(reinterpret_cast<char*>(sta_config.sta.password), sizeof(sta_config.sta.ssid), "%s", ascii_str_part2.c_str());

    //     } 
    //     else
    //     {
    //         ESP_LOGW(TAG, "未找到有效Part2数据");
    //     }

    //     if (passd_sta==1 && ssid_sta==1)
    //     {
    //         printf("sta_config.sta.ssid = %s\n", sta_config.sta.ssid);
    //         printf("sta_config.sta.password = %s\n", sta_config.sta.password);
    //         // sta_config.sta.bssid_set = 0;
    //         esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    //     }else
    //     {
    //         passd_sta=0;
    //         ssid_sta=0;
    //     }
    //     break;
    // }
    // break;
    // case 0x03: // 命令码0x03
    // {
    //     auto &app = Application::GetInstance(); // 在switch外初始化   
    //     auto &codec = Board::GetInstance();                             //   //responseData = {0x03, 0x83, 0x00, 0x08, 0x02, 0x03}; // 示例控制确认
    //     switch (cmd4)
    //     {   
    //     case 0x01: // 0x01指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_10); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_10); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x02: // 0x02指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_14); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_014); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x03: // 0x03指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_23); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_23); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x04: // 0x04指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_004); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_004); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x05: // 0x05指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_005); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_005); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x06:
    //     {
                
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             { 
    //                 // Volince_flag = 1;
    //                 ESP_LOGE(TAG, "预等待");
    //                 app.ResetDecoder();
    //                 app.PlaySound(Lang::Sounds::P3_006); //检测到新版本
    //                 ESP_LOGE(TAG, "等待开始");
    //                 ESP_LOGE(TAG, "kDeviceStateSpeaking");
    //                 break;
    //             }
    //             else //listing
    //             {    
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_006); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             

    //             ESP_LOGE(TAG, "检测到新版本!");
    //     } // 0x06指令
    //     break;
    //     case 0x07: // 0x07指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_007); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_007); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     break;
    //     case 0x08: // 0x08指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_008); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_008); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x09: // 0x09指令
    //     {
    //            if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_009); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_009); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x0A: // 0x0A指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_010); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_010); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x0B: // 0x0B指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_011); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_011); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x0C: // 0x0C指令
    //     {
    //            if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_012); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_012); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x0D: // 0x0D指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_013); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_013); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x0E: // 0x0E指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_014); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_014); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x0F: // 0x0F指令
    //     {
    //            if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_015); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_015); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x10: // 0x10指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_016); //检测到新版本
    //                 break;
    //             }
    //             else    
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_016); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x11: // 0x11指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_017); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_017); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x12: // 0x12指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_018); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_018); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x13: // 0x13指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();  
    //                 auto codec = Board::GetInstance().GetAudioCodec();
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_019); //检测到新版本
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_019); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     case 0x14: // 0x14指令
    //     {
    //             if(app.GetDeviceState() == kDeviceStateSpeaking)
    //             {
    //                 app.ResetDecoder();                 
    //                 auto codec = Board::GetInstance().GetAudioCodec();  
    //                 codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_WIFIZHANGHAOMIMA); //检测到新版本
    //                 printf("P3_WIFIZHANGHAOMIMA");
    //                 break;
    //             }
    //             else
    //             {
    //                 auto& app = Application::GetInstance();
    //                 app.SetDeviceState(kDeviceStateIdle);
    //                 // auto codec = Board::GetInstance().GetAudioCodec();
    //                 // codec->EnableOutput(true);
    //                 app.PlaySound(Lang::Sounds::P3_WIFIZHANGHAOMIMA); //检测到新版本
    //                 vTaskDelay(pdMS_TO_TICKS(1500));
    //                 ESP_LOGE(TAG, "检测到新版本!2222222");
    //                 break;
    //             }             
    //     } 
    //     default:
    //         break;
    //     }
    // }
    // break;

    // default:                                           // 未知命令
    //     responseData = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // 错误响应
    //     ESP_LOGW(TAG, "Unknown command received: 0x%02X", cmd);
    //     break;
    // }
    // // 发送响应数据
    // if (!responseData.empty())
    // {
    //     sendFrame(responseData);
    //     ESP_LOGI(TAG, "Sent response for command 0x%02X, %d bytes", cmd, responseData.size());
    // }
}

// 十六进制字符串转ASCII字符（支持带/不带空格，例："48 65 6C 6C 6F" -> "Hello"）
std::string UartSender::HexStringToAscii(const std::string &hex_str)
{
    std::string result;
    std::stringstream ss(hex_str);
    std::string byte_str;

    while (std::getline(ss, byte_str, ' '))
    { // 处理带空格的十六进制字符串
        if (byte_str.size() != 2)
        {
            ESP_LOGW(TAG, "Invalid hex byte: %s", byte_str.c_str());
            continue;
        }

        char byte = static_cast<char>(std::stoi(byte_str, nullptr, 16));
        result += byte;
    }

    return result;
}

// 十六进制字节数组转ASCII字符（直接转换字节为对应ASCII字符）
std::string UartSender::HexBytesToAscii(const std::vector<uint8_t> &hex_bytes)
{
    std::string result;
    result.reserve(hex_bytes.size()); // 预分配内存优化性能

    for (uint8_t byte : hex_bytes)
    {
        // 检查字节是否在ASCII可打印范围内（0x20~0x7E）
        if (byte >= 0x20 && byte <= 0x7E)
        {
            result += static_cast<char>(byte);
        }
        else
        {
            result += '.'; // 不可打印字符用点号替代
            ESP_LOGD(TAG, "Non-ASCII byte: 0x%02X", byte);
        }
    }

    return result;
}

// 处理单个字节的帧解析
void UartSender::processByte(uint8_t byte)
{
    // printf("byte = %d (0x%02X)\n", byte, (unsigned char)byte);
    switch (frame_state_)
    {
    case FrameState::IDLE:
        // 寻找帧起始标记 0x3A
        if (byte == 0x3A)
        {
            printf("byte == 0x3A\n");
            frame_state_ = FrameState::RECEIVING;
            frame_buffer_index_ = 0;
            // 起始字节不存储，从下一个字节开始存储数据
            ESP_LOGD(TAG, "Frame started");
        }
        break;

    case FrameState::RECEIVING:
        // 接收数据，直到遇到结束标记 0x7E
        if (byte == 0x7E)
        {
            printf("11byte == 0x7E\n");
            // 帧结束，处理完整帧
            frame_state_ = FrameState::IDLE;

            uint16_t n = (frame_buffer_[3] << 8) | frame_buffer_[4]; // n是长度段的值，需要与数据域的长度相加校验
            printf("n = %d\n", n);

            // 至少需要有数据和CRC校验值
            if (frame_buffer_index_ >= 5)
            { // 确保有足够的数据（至少5字节）
                // 提取接收到的CRC值（最后一个字节）
                uint8_t receivedCRC = frame_buffer_[frame_buffer_index_ - 1];

                // 计算数据部分的CRC值（不包括CRC字节本身）
                uint8_t calculatedCRC = calculateCRC(frame_buffer_, frame_buffer_index_ - 1); // 解析数据部分

                int lendflag = 0; // 长度校验标志位

                int start_index = 4;                     // 第五字节之后的第一个字节索引（索引从0开始，第五字节是索引4，之后是索引5+）  
                int crc_index = frame_buffer_index_ - 1; // CRC字节索引（最后一个字节）
                int Length_validation = 0;               // 长度校验

                if (start_index < crc_index)
                {
                    // 计算字节个数：crc_index - start_index - 1 + 1？
                    // 示例：索引5到索引n-2（CRC前一字节），个数为 (n-2 -5 + 1) = n-6
                    // 注意：CRC字节本身不包含在内，需排除
                    int byte_count = crc_index - start_index - 1; // 从索引5到CRC前一字节（索引crc_index-1）
                    Length_validation = byte_count + 8;
                    ESP_LOGI(TAG, "第五字节之后到CRC前的字节个数: %d", byte_count);
                }
                else
                {
                    ESP_LOGW(TAG, "数据不足，无法提取有效字节");
                }

                if(Length_validation == n)
                {
                    // 长度校验通过
                    ESP_LOGI(TAG, "长度校验通过");
                    lendflag = 1;
                    
                }
                else
                {
                    ESP_LOGE(TAG, "长度校验失败！");
                    printf("Length_validation = %d\n", Length_validation);
                    printf("n = %d\n", n);
                    lendflag = 0;

                }

                // 比较CRC值
                if (receivedCRC == calculatedCRC)
                {
                    if(lendflag == 1)
                    {
                            static int lendflagasdasdsadsa = 1; // 长度校验标志位
                            ESP_LOGI(TAG, "CRC check passed! Received: 0x%02X, Calculated: 0x%02X",
                            receivedCRC, calculatedCRC);

                            // 获取data[2]的值（即接收到的第3个字节）
                            if (frame_buffer_index_ >= 5)
                            {
                                uint8_t cmd = frame_buffer_[1];  // data[1]是帧缓冲区的第2个字节
                                uint8_t cmd4 = frame_buffer_[5]; // data[4]是帧缓冲区的第5个字节

                                ESP_LOGI(TAG, "Received command: 0x%02X", cmd);
                                ESP_LOGI(TAG, "Received command4: 0x%02X", cmd4);

                                // 根据cmd值发送不同的自定义数据
                                sendResponseByCommand(cmd, cmd4);
                            }

                            // 调用回调函数，传递原始字节数据I（不包括CRC字节）
                            if (receive_callback_)
                            {
                                //printf("receive_callback_ is not null\n");
                                std::vector<uint8_t> frameData(frame_buffer_, frame_buffer_ + frame_buffer_index_ - 1);
                                receive_callback_(frameData); // 触发回调
                            }
                        lendflag = 0; // 重置标志位
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "CRC check failed! Received: 0x%02X, Calculated: 0x%02X",
                             receivedCRC, calculatedCRC);
                }
            }
            else
            {
                ESP_LOGE(TAG, "Frame too short to contain valid data and CRC");
            }

            ESP_LOGD(TAG, "Frame completed, %d bytes", frame_buffer_index_);
            frame_buffer_index_ = 0;
            //printf("6666666666\n");
        }
        else
        {
            // 存储数据
            if (frame_buffer_index_ < BUF_SIZE - 1)
            {
                frame_buffer_[frame_buffer_index_++] = byte;
            }
            else
            {
                // 缓冲区溢出，重置解析器
                ESP_LOGE(TAG, "Frame buffer overflow!");
                resetFrameParser();
            }
        }
        break;

    default:
        // 未知状态，重置解析器
        resetFrameParser();
        break;
    }
}

// 重置帧解析器状态
void UartSender::resetFrameParser()
{
    frame_state_ = FrameState::IDLE;
    frame_buffer_index_ = 0;
}

