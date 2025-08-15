/*
 * MCP Server Implementation
 * Reference: https://modelcontextprotocol.io/specification/2024-11-05
 */



#include "mcp_server.h"
#include <esp_log.h>
#include <esp_app_desc.h>
#include <algorithm>
#include <cstring>
#include <esp_pthread.h>
#include "application.h"
#include "display.h"
#include "board.h"
#include "rp2040iic.h"  // 确保Rp2040类的声明被引入

#define TAG "MCP"

#define DEFAULT_TOOLCALL_STACK_SIZE 6144

// McpServer::McpServer() {
// }
McpServer::McpServer() : current_motor_direction("stop"),
                         current_motor2_direction("stop") 
{ // 初始化
}

McpServer::~McpServer() {
    for (auto tool : tools_) {
        delete tool;
    }
    tools_.clear();
}
static const std::vector<uint8_t> AVAILABLE_SERVO_IDS = {3, 4,7, 9, 12, 26, 27, 28};
void McpServer::AddCommonTools() {

    // 马达控制相关宏定义（修正最大PWM值为255）
    #define MOTOR_PIN_FWD 17    // 正转控制引脚P17
    #define MOTOR_PIN_REV 18    // 反转控制引脚P18
    #define MOTOR2_PIN_FWD 19   // 第二马达正转控制引脚P19
    #define MOTOR2_PIN_REV 22   // 第二马达反转控制引脚P22
    #define MAX_PWM_VALUE 255   // 最大PWM值（0-255）
    #define MIN_PWM_VALUE 1     // 最小有效PWM值（避免过低导致启动失败）


    // auto rp2040 = Rp2040::getInstance(i2c_bus_, 0x55); // 正确：用指针接收返回值
    // 在添加 self.servo.set_angle 工具前

    auto rp2040 = Rp2040::getInstance();
    if (rp2040 == nullptr) {
        ESP_LOGE(TAG, "Rp2040 instance is null, cannot add servo tool");
        return; // 或处理错误
    }

// 新增变量用于跟踪马达当前方向状态
std::string current_motor_direction = "stop"; // 初始状态为停止
// 在类成员变量中添加第二马达状态跟踪
std::string current_motor2_direction = "stop"; // 第二马达初始状态为停止

// 修改马达运动控制工具
AddTool("self.motor.set_motion",
    "控制马达的运动状态，包括正转、反转和停止，并可设置转动速度。\n"
    "参数说明：\n"
    "  `direction`: 运动方向，必须为 'forward'（正转）、'reverse'（反转）或 'stop'（停止）\n"
    "  `speed`: 转动速度（仅在direction为'forward'或'reverse'时必填），范围1-255\n"
    "使用示例：\n"
    "  正转（速度128）：{\"direction\": \"forward\", \"speed\": 128}\n"
    "  反转（速度200）：{\"direction\": \"reverse\", \"speed\": 200}\n"
    "  停止：{\"direction\": \"stop\"}",
    PropertyList({
        Property("direction", kPropertyTypeString),
        Property("speed", kPropertyTypeInteger, MIN_PWM_VALUE, MIN_PWM_VALUE, MAX_PWM_VALUE)  // 可选参数（停止时无需）
    }),
    [rp2040, this](const PropertyList& properties) -> ReturnValue {
        std::string direction = properties["direction"].value<std::string>();
        
        // 验证方向参数合法性
        if (direction != "forward" && direction != "reverse" && direction != "stop") {
            return "{\"success\": false, \"message\": \"无效的方向参数，可选值：forward, reverse, stop\"}";
        }

        // 处理停止状态
        if (direction == "stop") {
            rp2040->etPwmOutput(MOTOR_PIN_FWD, 0);
            rp2040->etPwmOutput(MOTOR_PIN_REV, 0);
             this->current_motor_direction = "stop"; 
            return "{\"success\": true, \"message\": \"马达已停止\"}";
        }

        // 非停止状态必须检查速度参数
        if (properties.find("speed") == nullptr){
            return "{\"success\": false, \"message\": \"正转/反转时必须指定speed参数（1-255）\"}";
        }
        
        int speed = properties["speed"].value<int>();
        
        // 验证速度范围
        if (speed < MIN_PWM_VALUE || speed > MAX_PWM_VALUE) {
            return "{\"success\": false, \"message\": \"速度必须在1-255范围内\"}";
        }

        // 根据方向设置PWM输出并更新状态
        if (direction == "forward") {
            rp2040->etPwmOutput(MOTOR_PIN_FWD, speed);
            rp2040->etPwmOutput(MOTOR_PIN_REV, 0);
            this->current_motor_direction = "forward"; 
            return "{\"success\": true, \"message\": \"马达正转中，速度：" + std::to_string(speed) + "\"}";
        } else {  // reverse
            rp2040->etPwmOutput(MOTOR_PIN_REV, speed);
            rp2040->etPwmOutput(MOTOR_PIN_FWD, 0);
            this->current_motor_direction = "reverse";
            return "{\"success\": true, \"message\": \"马达反转中，速度：" + std::to_string(speed) + "\"}";
        }
    });


// 修改马达速度单独调节工具
AddTool("self.motor.adjust_speed",
    "在马达运行中单独调节速度，不改变当前转动方向。\n"
    "使用前提：马达必须处于正转或反转状态（非停止）\n"
    "参数说明：\n"
    "  `speed`: 新速度值，范围1-255\n"
    "返回结果：当前运动方向及新速度",
    PropertyList({
        Property("speed", kPropertyTypeInteger, MIN_PWM_VALUE, MAX_PWM_VALUE)
    }),
        [rp2040, this](const PropertyList& properties) -> ReturnValue {
        int new_speed = properties["speed"].value<int>();
        
        // 检查马达是否处于运行状态（通过跟踪的状态变量）
        if (this->current_motor_direction == "stop") {
            return "{\"success\": false, \"message\": \"马达未运行，请先使用self.motor.set_motion启动\"}";
        }

        // 验证速度范围
        if (new_speed < MIN_PWM_VALUE || new_speed > MAX_PWM_VALUE) {
            return "{\"success\": false, \"message\": \"速度必须在1-255范围内\"}";
        }
        
        // 直接根据当前方向设置对应IO口的PWM值
        if (this->current_motor_direction == "forward") {
            rp2040->etPwmOutput(MOTOR_PIN_FWD, new_speed);
        } else { // reverse
            rp2040->etPwmOutput(MOTOR_PIN_REV, new_speed);
        }
        return "{\"success\": true, \"direction\": \"" + this->current_motor_direction + "\", \"new_speed\": " + std::to_string(new_speed) + ", \"message\": \"速度已更新\"}";
    });

    // 添加第二马达运动控制工具
AddTool("self.motor2.set_motion",
    "控制第二马达的运动状态，包括正转、反转和停止，并可设置转动速度。\n"
    "参数说明：\n"
    "  `direction`: 运动方向，必须为 'forward'（正转）、'reverse'（反转）或 'stop'（停止）\n"
    "  `speed`: 转动速度（仅在direction为'forward'或'reverse'时必填），范围1-255\n"
    "使用示例：\n"
    "  正转（速度128）：{\"direction\": \"forward\", \"speed\": 128}\n"
    "  反转（速度200）：{\"direction\": \"reverse\", \"speed\": 200}\n"
    "  停止：{\"direction\": \"stop\"}",
    PropertyList({
        Property("direction", kPropertyTypeString),
        Property("speed", kPropertyTypeInteger, MIN_PWM_VALUE, MIN_PWM_VALUE, MAX_PWM_VALUE)
    }),
    [rp2040, this](const PropertyList& properties) -> ReturnValue {
        std::string direction = properties["direction"].value<std::string>();
        
        // 验证方向参数合法性
        if (direction != "forward" && direction != "reverse" && direction != "stop") {
            return "{\"success\": false, \"message\": \"无效的方向参数，可选值：forward, reverse, stop\"}";
        }

        // 处理停止状态
        if (direction == "stop") {
            rp2040->etPwmOutput(MOTOR2_PIN_FWD, 0);
            rp2040->etPwmOutput(MOTOR2_PIN_REV, 0);
            this->current_motor2_direction = "stop"; 
            return "{\"success\": true, \"message\": \"第二马达已停止\"}";
        }

        // 非停止状态必须检查速度参数
        if (properties.find("speed") == nullptr){
            return "{\"success\": false, \"message\": \"正转/反转时必须指定speed参数（1-255）\"}";
        }
        
        int speed = properties["speed"].value<int>();
        
        // 验证速度范围
        if (speed < MIN_PWM_VALUE || speed > MAX_PWM_VALUE) {
            return "{\"success\": false, \"message\": \"速度必须在1-255范围内\"}";
        }

        // 根据方向设置PWM输出并更新状态
        if (direction == "forward") {
            rp2040->etPwmOutput(MOTOR2_PIN_FWD, speed);
            rp2040->etPwmOutput(MOTOR2_PIN_REV, 0);
            this->current_motor2_direction = "forward"; 
            return "{\"success\": true, \"message\": \"第二马达正转中，速度：" + std::to_string(speed) + "\"}";
        } else {  // reverse
            rp2040->etPwmOutput(MOTOR2_PIN_REV, speed);
            rp2040->etPwmOutput(MOTOR2_PIN_FWD, 0);
            this->current_motor2_direction = "reverse";
            return "{\"success\": true, \"message\": \"第二马达反转中，速度：" + std::to_string(speed) + "\"}";
        }
    });


    // 添加第二马达速度单独调节工具
AddTool("self.motor2.adjust_speed",
    "在第二马达运行中单独调节速度，不改变当前转动方向。\n"
    "使用前提：第二马达必须处于正转或反转状态（非停止）\n"
    "参数说明：\n"
    "  `speed`: 新速度值，范围1-255\n"
    "返回结果：当前运动方向及新速度",
    PropertyList({
        Property("speed", kPropertyTypeInteger, MIN_PWM_VALUE, MAX_PWM_VALUE)
    }),
        [rp2040, this](const PropertyList& properties) -> ReturnValue {
        int new_speed = properties["speed"].value<int>();
        
        // 检查第二马达是否处于运行状态
        if (this->current_motor2_direction == "stop") {
            return "{\"success\": false, \"message\": \"第二马达未运行，请先使用self.motor2.set_motion启动\"}";
        }

        // 验证速度范围
        if (new_speed < MIN_PWM_VALUE || new_speed > MAX_PWM_VALUE) {
            return "{\"success\": false, \"message\": \"速度必须在1-255范围内\"}";
        }
        
        // 直接根据当前方向设置对应IO口的PWM值
        if (this->current_motor2_direction == "forward") {
            rp2040->etPwmOutput(MOTOR2_PIN_FWD, new_speed);
        } else { // reverse
            rp2040->etPwmOutput(MOTOR2_PIN_REV, new_speed);
        }
        return "{\"success\": true, \"direction\": \"" + this->current_motor2_direction + "\", \"new_speed\": " + std::to_string(new_speed) + ", \"message\": \"第二马达速度已更新\"}";
    });



    // 新增：同时控制多个马达的工具
AddTool("self.motors.set_multiple_motion",
    "同时控制多个马达的运动状态（支持马达1和马达2），包括正转、反转和停止，并可分别设置速度。\n"
    "参数说明：\n"
    "  `motors`: JSON数组，每个元素为包含马达配置的对象，结构如下：\n"
    "    - `motor_id`: 马达ID，必须为1（对应原motor）或2（对应原motor2）\n"
    "    - `direction`: 运动方向，必须为 'forward'（正转）、'reverse'（反转）或 'stop'（停止）\n"
    "    - `speed`: 转动速度（仅在direction为'forward'或'reverse'时必填），范围1-255\n"
    "使用示例：\n"
    "  [{\"motor_id\":1, \"direction\":\"forward\", \"speed\":128}, {\"motor_id\":2, \"direction\":\"reverse\", \"speed\":200}]\n"
    "  [{\"motor_id\":1, \"direction\":\"stop\"}, {\"motor_id\":2, \"direction\":\"stop\"}]",
    PropertyList({
        Property("motors", kPropertyTypeString)  // 用字符串接收JSON数组
    }),
    [rp2040, this](const PropertyList& properties) -> ReturnValue {
        std::string motors_json = properties["motors"].value<std::string>();
        cJSON* motors_array = cJSON_Parse(motors_json.c_str());
        
        // 校验输入是否为有效JSON数组
        if (!cJSON_IsArray(motors_array)) {
            cJSON_Delete(motors_array);
            return "{\"success\": false, \"message\": \"无效的'motors'格式，需传入JSON数组\"}";
        }
        
        // 存储每个马达的控制结果
        std::vector<std::string> success_list;
        std::vector<std::string> fail_list;
        int array_size = cJSON_GetArraySize(motors_array);
        
        // 遍历数组中的每个马达配置
        for (int i = 0; i < array_size; ++i) {
            cJSON* motor_obj = cJSON_GetArrayItem(motors_array, i);
            if (!cJSON_IsObject(motor_obj)) {
                fail_list.push_back("第" + std::to_string(i+1) + "个元素不是对象");
                continue;
            }
            
            // 解析motor_id、direction和speed
            cJSON* id_json = cJSON_GetObjectItem(motor_obj, "motor_id");
            cJSON* dir_json = cJSON_GetObjectItem(motor_obj, "direction");
            cJSON* speed_json = cJSON_GetObjectItem(motor_obj, "speed");
            
            // 校验基础参数
            if (!cJSON_IsNumber(id_json) || !cJSON_IsString(dir_json)) {
                fail_list.push_back("第" + std::to_string(i+1) + "个元素缺少'motor_id'（数字）或'direction'（字符串）");
                continue;
            }
            
            int motor_id = id_json->valueint;
            std::string direction = dir_json->valuestring;
            
            // 校验马达ID（仅支持1和2）
            if (motor_id != 1 && motor_id != 2) {
                fail_list.push_back("马达ID " + std::to_string(motor_id) + " 无效，仅支持1或2");
                continue;
            }
            
            // 校验方向参数
            if (direction != "forward" && direction != "reverse" && direction != "stop") {
                fail_list.push_back("马达" + std::to_string(motor_id) + "方向无效，可选值：forward, reverse, stop");
                continue;
            }
            
            // 处理停止状态（无需速度）
            if (direction == "stop") {
                if (motor_id == 1) {
                    rp2040->etPwmOutput(MOTOR_PIN_FWD, 0);
                    rp2040->etPwmOutput(MOTOR_PIN_REV, 0);
                    this->current_motor_direction = "stop";
                } else {
                    rp2040->etPwmOutput(MOTOR2_PIN_FWD, 0);
                    rp2040->etPwmOutput(MOTOR2_PIN_REV, 0);
                    this->current_motor2_direction = "stop";
                }
                success_list.push_back("马达" + std::to_string(motor_id) + "已停止");
                continue;
            }
            
            // 非停止状态必须校验速度参数
            if (!cJSON_IsNumber(speed_json)) {
                fail_list.push_back("马达" + std::to_string(motor_id) + "正转/反转时必须指定speed参数");
                continue;
            }
            int speed = speed_json->valueint;
            if (speed < MIN_PWM_VALUE || speed > MAX_PWM_VALUE) {
                fail_list.push_back("马达" + std::to_string(motor_id) + "速度无效，需在1-255范围内");
                continue;
            }
            
            // 根据马达ID和方向设置PWM并更新状态
            if (motor_id == 1) {
                if (direction == "forward") {
                    rp2040->etPwmOutput(MOTOR_PIN_FWD, speed);
                    rp2040->etPwmOutput(MOTOR_PIN_REV, 0);
                    this->current_motor_direction = "forward";
                } else {
                    rp2040->etPwmOutput(MOTOR_PIN_REV, speed);
                    rp2040->etPwmOutput(MOTOR_PIN_FWD, 0);
                    this->current_motor_direction = "reverse";
                }
                success_list.push_back("马达1" + direction + "，速度：" + std::to_string(speed));
            } else {  // motor_id == 2
                if (direction == "forward") {
                    rp2040->etPwmOutput(MOTOR2_PIN_FWD, speed);
                    rp2040->etPwmOutput(MOTOR2_PIN_REV, 0);
                    this->current_motor2_direction = "forward";
                } else {
                    rp2040->etPwmOutput(MOTOR2_PIN_REV, speed);
                    rp2040->etPwmOutput(MOTOR2_PIN_FWD, 0);
                    this->current_motor2_direction = "reverse";
                }
                success_list.push_back("马达2" + direction + "，速度：" + std::to_string(speed));
            }
        }
        
        cJSON_Delete(motors_array);  // 释放JSON资源
        
        // 构建返回结果
        std::string result = "{\"success\":";
        result += (fail_list.empty() ? "true" : "false");
        result += ", \"total_motors\":" + std::to_string(array_size);
        result += ", \"success_count\":" + std::to_string(success_list.size());
        result += ", \"fail_count\":" + std::to_string(fail_list.size());
        
        if (!success_list.empty()) {
            result += ", \"success_details\": [";
            for (size_t i = 0; i < success_list.size(); ++i) {
                result += "\"" + success_list[i] + "\"";
                if (i != success_list.size() - 1) result += ", ";
            }
            result += "]";
        }
        
        if (!fail_list.empty()) {
            result += ", \"fail_details\": [";
            for (size_t i = 0; i < fail_list.size(); ++i) {
                result += "\"" + fail_list[i] + "\"";
                if (i != fail_list.size() - 1) result += ", ";
            }
            result += "]";
        }
        
        result += "}";
        return result;
    });



    // To speed up the response time, we add the common tools to the beginning of
    // the tools list to utilize the prompt cache.
    // Backup the original tools list and restore it after adding the common tools.
    auto original_tools = std::move(tools_);
    auto& board = Board::GetInstance();
    // auto1& Rp2040 = Rp2040::GetInstance();

    AddTool("self.get_device_status",
        "Provides the real-time information of the device, including the current status of the audio speaker, screen, battery, network, etc.\n"
        "Use this tool for: \n"
        "1. Answering questions about current condition (e.g. what is the current volume of the audio speaker?)\n"
        "2. As the first step to control the device (e.g. turn up / down the volume of the audio speaker, etc.)",
        PropertyList(),
        [&board](const PropertyList& properties) -> ReturnValue {
            return board.GetDeviceStatusJson();
        });


    AddTool("self.audio_speaker.set_volume", 
        "Set the volume of the audio speaker. If the current volume is unknown, you must call `self.get_device_status` tool first and then call this tool.",
        PropertyList({
            Property("volume", kPropertyTypeInteger, 0, 100)
        }), 
        [&board](const PropertyList& properties) -> ReturnValue {
            auto codec = board.GetAudioCodec();
            codec->SetOutputVolume(properties["volume"].value<int>());
            return true;
        });


// 添加设置单个舵机角度的工具（已添加7号舵机支持）
AddTool("self.servo.set_angle", 
    "Set the angle of a specific servo motor. \n"
    "Available servo IDs: 3, 4, 7, 9, 12, 26, 27, 28 (other IDs are unavailable).\n"  // 添加了7
    "Note: Angle range is 0-180 degrees.",
    PropertyList({
        Property("servo_id", kPropertyTypeInteger, 3, 28),  // 覆盖所有可能的ID范围
        Property("angle", kPropertyTypeInteger, 0, 180),
    }),
    [rp2040, &AVAILABLE_SERVO_IDS](const PropertyList& properties) -> ReturnValue  {
        uint8_t servo_id = static_cast<uint8_t>(properties["servo_id"].value<int>());
        uint8_t angle = static_cast<uint8_t>(properties["angle"].value<int>());
        
        // 检查舵机ID是否在可用列表中（现在包含7号）
        if (std::find(AVAILABLE_SERVO_IDS.begin(), AVAILABLE_SERVO_IDS.end(), servo_id) == AVAILABLE_SERVO_IDS.end()) {
            std::string msg = "{\"success\": false, \"message\": \"Servo ID not available. Use ";
            for (size_t i = 0; i < AVAILABLE_SERVO_IDS.size(); ++i) {
                msg += std::to_string(AVAILABLE_SERVO_IDS[i]);
                if (i != AVAILABLE_SERVO_IDS.size() - 1) msg += ", ";
            }
            msg += "\"}";
            return msg;
        }
        
        // 检查角度范围
        if (angle < 0 || angle > 180) {
            return "{\"success\": false, \"message\": \"Angle must be between 0 and 180 degrees\"}";
        }
        
        bool result = rp2040->SetServoAngle(servo_id, angle);
        if (result) {
            return "{\"success\": true, \"message\": \"Servo " + std::to_string(servo_id) + " angle set to " + std::to_string(angle) + "\"}";
        } else {
            return "{\"success\": false, \"message\": \"Failed to set servo " + std::to_string(servo_id) + " angle\"}";
        }
    });


// 添加同时设置多个舵机角度的工具（已添加7号舵机支持）
AddTool("self.servo.set_multiple_angles", 
    "Set angles for multiple servo motors simultaneously. \n"
    "Available servo IDs: 3, 4, 7, 9, 12, 26, 27, 28 (other IDs are unavailable).\n"  // 添加了7
    "Args:\n"
    "  `servos`: A JSON array of objects, each containing \"servo_id\" (int) and \"angle\" (int, 0-180).\n"
    "Example: [{\"servo_id\":3, \"angle\":90}, {\"servo_id\":7, \"angle\":45}]\n"  // 示例中添加了7号
    "Note: Controlling too many servos at once may cause power issues. Max 5 servos recommended.",
    PropertyList({
        Property("servos", kPropertyTypeString)  // 用字符串接收JSON数组
    }),
    [rp2040, AVAILABLE_SERVO_IDS](const PropertyList& properties) -> ReturnValue{
        
        std::string servos_json = properties["servos"].value<std::string>();
        cJSON* servos_array = cJSON_Parse(servos_json.c_str());
        
        // 校验输入是否为有效JSON数组
        if (!cJSON_IsArray(servos_array)) {
            cJSON_Delete(servos_array);
            return "{\"success\": false, \"message\": \"Invalid 'servos' format. Use a JSON array\"}";
        }
        
        // 存储每个舵机的控制结果
        std::vector<std::string> success_list;
        std::vector<std::string> fail_list;
        
        // 遍历数组中的每个舵机
        int array_size = cJSON_GetArraySize(servos_array);
        for (int i = 0; i < array_size; ++i) {
            cJSON* servo_obj = cJSON_GetArrayItem(servos_array, i);
            if (!cJSON_IsObject(servo_obj)) {
                fail_list.push_back("Item " + std::to_string(i) + ": not an object");
                continue;
            }
            
            // 解析servo_id和angle
            cJSON* id_json = cJSON_GetObjectItem(servo_obj, "servo_id");
            cJSON* angle_json = cJSON_GetObjectItem(servo_obj, "angle");
            if (!cJSON_IsNumber(id_json) || !cJSON_IsNumber(angle_json)) {
                fail_list.push_back("Item " + std::to_string(i) + ": missing 'servo_id' or 'angle'");
                continue;
            }
            
            uint8_t servo_id = static_cast<uint8_t>(id_json->valueint);
            uint8_t angle = static_cast<uint8_t>(angle_json->valueint);
            
            // 校验舵机ID（现在会包含7号的检查）
            if (std::find(AVAILABLE_SERVO_IDS.begin(), AVAILABLE_SERVO_IDS.end(), servo_id) == AVAILABLE_SERVO_IDS.end()) {
                fail_list.push_back("Servo " + std::to_string(servo_id) + ": ID not available");
                continue;
            }
            
            // 校验角度
            if (angle < 0 || angle > 180) {
                fail_list.push_back("Servo " + std::to_string(servo_id) + ": angle out of range (0-180)");
                continue;
            }
            
            // 执行角度设置
            bool result = rp2040->SetServoAngle(servo_id, angle);
            if (result) {
                success_list.push_back("Servo " + std::to_string(servo_id) + " set to " + std::to_string(angle));
            } else {
                fail_list.push_back("Servo " + std::to_string(servo_id) + ": failed to set angle");
            }
        }
        
        cJSON_Delete(servos_array);  // 释放JSON资源
        
        // 构建返回结果
        std::string result = "{\"success\":";
        result += (fail_list.empty() ? "true" : "false");
        result += ", \"success_count\":" + std::to_string(success_list.size());
        result += ", \"fail_count\":" + std::to_string(fail_list.size());
        
        if (!success_list.empty()) {
            result += ", \"success_details\": [";
            for (size_t i = 0; i < success_list.size(); ++i) {
                result += "\"" + success_list[i] + "\"";
                if (i != success_list.size() - 1) result += ", ";
            }
            result += "]";
        }
        
        if (!fail_list.empty()) {
            result += ", \"fail_details\": [";
            for (size_t i = 0; i < fail_list.size(); ++i) {
                result += "\"" + fail_list[i] + "\"";
                if (i != fail_list.size() - 1) result += ", ";
            }
            result += "]";
        }
        
        result += "}";
        return result;
    });


    auto backlight = board.GetBacklight();
    if (backlight) {
        AddTool("self.screen.set_brightness",
            "Set the brightness of the screen.",
            PropertyList({
                Property("brightness", kPropertyTypeInteger, 0, 100)
            }),
            [backlight](const PropertyList& properties) -> ReturnValue {
                uint8_t brightness = static_cast<uint8_t>(properties["brightness"].value<int>());
                backlight->SetBrightness(brightness, true);
                return true;
            });
    }


    auto display = board.GetDisplay();
    if (display && !display->GetTheme().empty()) {
        AddTool("self.screen.set_theme",
            "Set the theme of the screen. The theme can be `light` or `dark`.",
            PropertyList({
                Property("theme", kPropertyTypeString)
            }),
            [display](const PropertyList& properties) -> ReturnValue {
                display->SetTheme(properties["theme"].value<std::string>().c_str());
                return true;
            });
    }


    auto camera = board.GetCamera();
    if (camera) {
        AddTool("self.camera.take_photo",
            "Take a photo and explain it. Use this tool after the user asks you to see something.\n"
            "Args:\n"
            "  `question`: The question that you want to ask about the photo.\n"
            "Return:\n"
            "  A JSON object that provides the photo information.",
            PropertyList({
                Property("question", kPropertyTypeString)
            }),
            [camera](const PropertyList& properties) -> ReturnValue {
                if (!camera->Capture()) {
                    return "{\"success\": false, \"message\": \"Failed to capture photo\"}";
                }
                auto question = properties["question"].value<std::string>();
                return camera->Explain(question);
            });
    }

    // Restore the original tools list to the end of the tools list
    tools_.insert(tools_.end(), original_tools.begin(), original_tools.end());
}

void McpServer::AddTool(McpTool* tool) {
    // Prevent adding duplicate tools
    if (std::find_if(tools_.begin(), tools_.end(), [tool](const McpTool* t) { return t->name() == tool->name(); }) != tools_.end()) {
        ESP_LOGW(TAG, "Tool %s already added", tool->name().c_str());
        return;
    }

    ESP_LOGI(TAG, "Add tool: %s", tool->name().c_str());
    tools_.push_back(tool);
}

void McpServer::AddTool(const std::string& name, const std::string& description, const PropertyList& properties, std::function<ReturnValue(const PropertyList&)> callback) {
    AddTool(new McpTool(name, description, properties, callback));
}

void McpServer::ParseMessage(const std::string& message) {
    cJSON* json = cJSON_Parse(message.c_str());
    if (json == nullptr) {
        ESP_LOGE(TAG, "Failed to parse MCP message: %s", message.c_str());
        return;
    }
    ParseMessage(json);
    cJSON_Delete(json);
}

// void McpServer::ParseCapabilities(const cJSON* capabilities) {
//     auto vision = cJSON_GetObjectItem(capabilities, "vision");
//     if (cJSON_IsObject(vision)) {
//         auto url = cJSON_GetObjectItem(vision, "url");
//         auto token = cJSON_GetObjectItem(vision, "token");
//         if (cJSON_IsString(url)) {
//             auto camera = Board::GetInstance().GetCamera();
//             if (camera) {
//                 std::string url_str = std::string(url->valuestring);
//                 std::string token_str;
//                 if (cJSON_IsString(token)) {
//                     token_str = std::string(token->valuestring);
//                 }
//                 camera->SetExplainUrl(url_str, token_str);
//             }
//         }
//     }
// }

void McpServer::ParseCapabilities(const cJSON* capabilities) {
    auto vision = cJSON_GetObjectItem(capabilities, "vision");
    if (cJSON_IsObject(vision)) {
        auto url = cJSON_GetObjectItem(vision, "url");
        auto token = cJSON_GetObjectItem(vision, "token");
        if (cJSON_IsString(url)) {
            auto camera = Board::GetInstance().GetCamera();
            if (camera) {
                std::string url_str = std::string(url->valuestring);
                std::string token_str;
                if (cJSON_IsString(token)) {
                    token_str = std::string(token->valuestring);
                }
                camera->SetExplainUrl(url_str, token_str);
            }
        }
    }
}

void McpServer::ParseMessage(const cJSON* json) {
    // Check JSONRPC version
    auto version = cJSON_GetObjectItem(json, "jsonrpc");
    if (version == nullptr || !cJSON_IsString(version) || strcmp(version->valuestring, "2.0") != 0) {
        ESP_LOGE(TAG, "Invalid JSONRPC version: %s", version ? version->valuestring : "null");
        return;
    }
    
    // Check method
    auto method = cJSON_GetObjectItem(json, "method");
    if (method == nullptr || !cJSON_IsString(method)) {
        ESP_LOGE(TAG, "Missing method");
        return;
    }
    
    auto method_str = std::string(method->valuestring);
    if (method_str.find("notifications") == 0) {
        return;
    }
    
    // Check params
    auto params = cJSON_GetObjectItem(json, "params");
    if (params != nullptr && !cJSON_IsObject(params)) {
        ESP_LOGE(TAG, "Invalid params for method: %s", method_str.c_str());
        return;
    }

    auto id = cJSON_GetObjectItem(json, "id");
    if (id == nullptr || !cJSON_IsNumber(id)) {
        ESP_LOGE(TAG, "Invalid id for method: %s", method_str.c_str());
        return;
    }
    auto id_int = id->valueint;
    
    if (method_str == "initialize") {
        if (cJSON_IsObject(params)) {
            auto capabilities = cJSON_GetObjectItem(params, "capabilities");
            if (cJSON_IsObject(capabilities)) {
                ParseCapabilities(capabilities);
            }
        }
        auto app_desc = esp_app_get_description();
        std::string message = "{\"protocolVersion\":\"2024-11-05\",\"capabilities\":{\"tools\":{}},\"serverInfo\":{\"name\":\"" BOARD_NAME "\",\"version\":\"";
        message += app_desc->version;
        message += "\"}}";
        ReplyResult(id_int, message);
    } else if (method_str == "tools/list") {
        std::string cursor_str = "";
        if (params != nullptr) {
            auto cursor = cJSON_GetObjectItem(params, "cursor");
            if (cJSON_IsString(cursor)) {
                cursor_str = std::string(cursor->valuestring);
            }
        }
        GetToolsList(id_int, cursor_str);
    } else if (method_str == "tools/call") {
        if (!cJSON_IsObject(params)) {
            ESP_LOGE(TAG, "tools/call: Missing params");
            ReplyError(id_int, "Missing params");
            return;
        }
        auto tool_name = cJSON_GetObjectItem(params, "name");
        if (!cJSON_IsString(tool_name)) {
            ESP_LOGE(TAG, "tools/call: Missing name");
            ReplyError(id_int, "Missing name");
            return;
        }
        auto tool_arguments = cJSON_GetObjectItem(params, "arguments");
        if (tool_arguments != nullptr && !cJSON_IsObject(tool_arguments)) {
            ESP_LOGE(TAG, "tools/call: Invalid arguments");
            ReplyError(id_int, "Invalid arguments");
            return;
        }
        auto stack_size = cJSON_GetObjectItem(params, "stackSize");
        if (stack_size != nullptr && !cJSON_IsNumber(stack_size)) {
            ESP_LOGE(TAG, "tools/call: Invalid stackSize");
            ReplyError(id_int, "Invalid stackSize");
            return;
        }
        DoToolCall(id_int, std::string(tool_name->valuestring), tool_arguments, stack_size ? stack_size->valueint : DEFAULT_TOOLCALL_STACK_SIZE);
    } else {
        ESP_LOGE(TAG, "Method not implemented: %s", method_str.c_str());
        ReplyError(id_int, "Method not implemented: " + method_str);
    }
}

void McpServer::ReplyResult(int id, const std::string& result) {
    std::string payload = "{\"jsonrpc\":\"2.0\",\"id\":";
    payload += std::to_string(id) + ",\"result\":";
    payload += result;
    payload += "}";
    Application::GetInstance().SendMcpMessage(payload);
}

void McpServer::ReplyError(int id, const std::string& message) {
    std::string payload = "{\"jsonrpc\":\"2.0\",\"id\":";
    payload += std::to_string(id);
    payload += ",\"error\":{\"message\":\"";
    payload += message;
    payload += "\"}}";
    Application::GetInstance().SendMcpMessage(payload);
}

void McpServer::GetToolsList(int id, const std::string& cursor) {
    const int max_payload_size = 8000;
    std::string json = "{\"tools\":[";
    
    bool found_cursor = cursor.empty();
    auto it = tools_.begin();
    std::string next_cursor = "";
    
    while (it != tools_.end()) {
        // 如果我们还没有找到起始位置，继续搜索
        if (!found_cursor) {
            if ((*it)->name() == cursor) {
                found_cursor = true;
            } else {
                ++it;
                continue;
            }
        }
        
        // 添加tool前检查大小
        std::string tool_json = (*it)->to_json() + ",";
        if (json.length() + tool_json.length() + 30 > max_payload_size) {
            // 如果添加这个tool会超出大小限制，设置next_cursor并退出循环
            next_cursor = (*it)->name();
            break;
        }
        
        json += tool_json;
        ++it;
    }
    
    if (json.back() == ',') {
        json.pop_back();
    }
    
    if (json.back() == '[' && !tools_.empty()) {
        // 如果没有添加任何tool，返回错误
        ESP_LOGE(TAG, "tools/list: Failed to add tool %s because of payload size limit", next_cursor.c_str());
        ReplyError(id, "Failed to add tool " + next_cursor + " because of payload size limit");
        return;
    }

    if (next_cursor.empty()) {
        json += "]}";
    } else {
        json += "],\"nextCursor\":\"" + next_cursor + "\"}";
    }
    
    ReplyResult(id, json);
}

void McpServer::DoToolCall(int id, const std::string& tool_name, const cJSON* tool_arguments, int stack_size) {
    auto tool_iter = std::find_if(tools_.begin(), tools_.end(), 
                                 [&tool_name](const McpTool* tool) { 
                                     return tool->name() == tool_name; 
                                 });
    
    if (tool_iter == tools_.end()) {
        ESP_LOGE(TAG, "tools/call: Unknown tool: %s", tool_name.c_str());
        ReplyError(id, "Unknown tool: " + tool_name);
        return;
    }

    PropertyList arguments = (*tool_iter)->properties();
    try {
        for (auto& argument : arguments) {
            bool found = false;
            if (cJSON_IsObject(tool_arguments)) {
                auto value = cJSON_GetObjectItem(tool_arguments, argument.name().c_str());
                if (argument.type() == kPropertyTypeBoolean && cJSON_IsBool(value)) {
                    argument.set_value<bool>(value->valueint == 1);
                    found = true;
                } else if (argument.type() == kPropertyTypeInteger && cJSON_IsNumber(value)) {
                    argument.set_value<int>(value->valueint);
                    found = true;
                } else if (argument.type() == kPropertyTypeString && cJSON_IsString(value)) {
                    argument.set_value<std::string>(value->valuestring);
                    found = true;
                }
            }

            if (!argument.has_default_value() && !found) {
                ESP_LOGE(TAG, "tools/call: Missing valid argument: %s", argument.name().c_str());
                ReplyError(id, "Missing valid argument: " + argument.name());
                return;
            }
        }
    } catch (const std::exception& e) {  // 捕获所有std::exception子类
    ESP_LOGE(TAG, "tools/call: %s", e.what());
    ReplyError(id, e.what());
    return;
    } 
    catch (const std::runtime_error& e) {
        ESP_LOGE(TAG, "tools/call: %s", e.what());
        ReplyError(id, e.what());
        return;
    }

    // Start a task to receive data with stack size
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.thread_name = "tool_call";
    cfg.stack_size = stack_size;
    cfg.prio = 1;
    esp_pthread_set_cfg(&cfg);

    // Use a thread to call the tool to avoid blocking the main thread
    tool_call_thread_ = std::thread([this, id, tool_iter, arguments = std::move(arguments)]() {
        try {
            ReplyResult(id, (*tool_iter)->Call(arguments));
        } catch (const std::runtime_error& e) {
            ESP_LOGE(TAG, "tools/call: %s", e.what());
            ReplyError(id, e.what());
        }
    });
    tool_call_thread_.detach();
}