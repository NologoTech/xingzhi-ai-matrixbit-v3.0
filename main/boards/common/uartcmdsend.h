#ifndef __UART_H__
#define __UART_H__

#include "driver/uart.h"
#include <functional>
#include <vector>
#include <string>
#include "driver/usb_serial_jtag.h" // 新增USB串口驱动头文件

// 配置UART参数
#define ECHO_TEST_TXD (48)
#define ECHO_TEST_RXD (47)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define BUF_SIZE (1024)
#define UART_RX_TIMEOUT 100  // 接收超时时间(ms)
//#define GPIO_DETECT_PIN GPIO_NUM_3 // 使用ESP-IDF的GPIO编号宏 //定义检测gpio3

class UartSender {
public:
    esp_timer_handle_t clock_timer_handle_ = nullptr;

    // 初始化 UART 并创建单例
    static void Initialize(uart_port_t uart_num, const uart_config_t& uart_config);
    
    // 获取单例引用
    static UartSender& GetInstance();


        // 添加静态成员声明
    static std::string ascii_str_part1;
    static std::string ascii_str_part2;


    // 发送数据指令
    void sendNumber(int number);

    //发送机器狗相关指令(自定义指令)  新增：实时发送自定义数据（带CRC校验）
    void sendRealTimeData(const std::vector<uint8_t>& realTimeData);
    
    // 发送字符串数据
    void sendString(const std::string& send_data);
    
    // 获取当前 UART 端口号
    uart_port_t getUartNum() const { return uart_num_; }

    // 注册接收回调函数
    void setReceiveCallback(std::function<void(const std::vector<uint8_t>&)> callback);

    //注册回发函数
    void sendFrame(const std::vector<uint8_t>& data);

    // 允许用户自定义发送数据，数据格式与接收数据一致 回发函数
    void sendCustomData(const std::vector<uint8_t>& customData);

    // 根据指令发送回应数据
    void sendResponseByCommand(uint8_t cmd,uint8_t cmd4);

    // 检查并处理接收到的数据
    void checkForReceivedData();

    // 计算CRC校验码
    uint8_t calculateCRC(const uint8_t* data, size_t length);

    // 初始化GPIO引脚
    void initGpio();

    void StartNetwork3();


    //wifi用户名，密码处理
    // 十六进制字符串转ASCII字符（例："48656C6C6F" -> "Hello"）
    static std::string HexStringToAscii(const std::string& hex_str);
    
    // 十六进制字节数组转字符（例：{0x48, 0x65, 0x6C, 0x6C, 0x6F} -> "Hello"）
    static std::string HexBytesToAscii(const std::vector<uint8_t>& hex_bytes);





private:
    // 帧解析状态
    enum class FrameState {
        IDLE,           // 等待起始标记
        RECEIVING,      // 接收数据
        CHECKSUM,       // 校验和验证(如果需要)
    };



    // 接收回调函数
    std::function<void(const std::vector<uint8_t>&)> receive_callback_;
    
    // 私有构造函数
    UartSender(uart_port_t uart_num, const uart_config_t& uart_config);
    
    // 私有析构函数
    ~UartSender();                                                     //卸载uart驱动并释放实例内存
    
    // 删除拷贝构造和赋值运算符，确保单例
    UartSender(const UartSender&) = delete;
    UartSender& operator=(const UartSender&) = delete;
    
    // 静态实例指针
    static UartSender* instance_;
    
    // UART 配置参数               
    uart_port_t uart_num_;
    uart_config_t uart_config_;

    // 接收缓冲区和状态
    char rx_buffer_[BUF_SIZE];
    int rx_buffer_index_;
    uint32_t last_rx_time_;
    
    // 帧解析相关
    FrameState frame_state_;
    uint8_t frame_buffer_[BUF_SIZE];  // 使用uint8_t数组存储原始字节数据
    int frame_buffer_index_;

    // std::string ascii_str_part1;
    // std::string ascii_str_part2;

    // 私有方法
    void processByte(uint8_t byte);
    void resetFrameParser();

    // USB SERIAL JTAG相关成员（可选，如需统一管理）
    static usb_serial_jtag_driver_config_t usb_config_;
    static uint8_t* usb_rx_buffer_;
};

#endif // __UART_H__