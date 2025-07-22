#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

// -------------------------- 舵机相关定义 --------------------------
struct ServoInfo {
  Servo servo;       // 舵机对象
  int pin;           // 控制引脚
  int currentAngle;  // 当前角度
  int targetAngle;   // 目标角度
  bool isUpdating;   // 是否需要更新（转动中）
} servos[5] = {      // 5个舵机的初始化参数
  {Servo(), 3, 0, 0, false},   // 舵机1：引脚3
  {Servo(), 12, 0, 0, false},  // 舵机2：引脚12
  {Servo(), 9, 0, 0, false},   // 舵机3：引脚9
  {Servo(), 4, 0, 0, false},   // 舵机4：引脚4
  {Servo(), 7, 0, 0, false}    // 舵机5：引脚7
};

// -------------------------- 灯带相关定义 --------------------------
#define LED_PIN     7       // WS2812数据引脚
#define LED_COUNT   30      // LED数量
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// -------------------------- PWM相关定义 --------------------------
#define PWM_START_ADDR 0x20              // PWM寄存器起始地址
#define PWM_END_ADDR 0x30                // PWM寄存器结束地址
#define PWM_CHANNEL_COUNT (PWM_END_ADDR - PWM_START_ADDR + 1)  // 17个通道

// PWM通道对应的物理引脚（根据实际硬件修改）
const int pwm_pins[PWM_CHANNEL_COUNT] = {
  0, 1, 2, 5, 6, 8, 10, 11, 13, 14, 15, 16, 17, 18, 19, 22, 23
};  // 索引0-16对应通道0-16（地址0x20-0x30）

// -------------------------- 其他硬件定义 --------------------------
#define ANALOG_PIN  29      // 光敏电阻模拟输入引脚
#define TRIGGER_PIN 25      // 触发控制引脚

// -------------------------- I2C相关定义 --------------------------
#define SLAVE_ID 0x55                   // I2C从机地址
#define TRIGGER_REGISTER 0x66           // 触发寄存器地址
#define TRIGGER_VALUE 0x01              // 触发值
#define LED_CONTROL_R 0x12              // 红灯控制地址
#define LED_CONTROL_G 0x13              // 绿灯控制地址
#define LED_CONTROL_B 0x14              // 蓝灯控制地址
#define LIGHT_SENSOR_ADDR 0x15          // 光敏电阻值存储地址
// 舵机控制地址（对应5个舵机）
#define SERVO1_ADDR 0x33
#define SERVO2_ADDR 0x3C
#define SERVO3_ADDR 0x39
#define SERVO4_ADDR 0x34
#define SERVO5_ADDR 0x37

// -------------------------- 全局变量 --------------------------
volatile uint8_t address = 0;                  // I2C通信地址
volatile uint8_t memory_map[256] = {0};        // 地址映射表（0-255）
int analogValue = 0;                           // 光敏电阻读取值

// -------------------------- I2C接收函数（主机发送数据时触发） --------------------------
void receiveEvent(int howMany) {
  static bool isAddress = true;  // 标志：当前接收的是地址还是数据

  while (Wire.available() > 0) {
    if (isAddress) {
      // 第一步：接收地址
      address = Wire.read();
      Serial.print("接收地址: 0x");
      Serial.println(address, HEX);
      isAddress = false;  // 下一次接收数据
    } else {
      // 第二步：接收数据并处理
      uint8_t data = Wire.read();
      memory_map[address] = data;  // 存储数据到地址映射表

      // PWM通道控制（处理主机SetPwmOutput()写入）
      if (address >= PWM_START_ADDR && address <= PWM_END_ADDR) {
        int channel = address - PWM_START_ADDR;  // 计算通道号（0-16）
        if (channel >= 0 && channel < PWM_CHANNEL_COUNT) {
          // 输出PWM信号到对应引脚（0-255对应0-3.3V）
          analogWrite(pwm_pins[channel], data);
          Serial.printf("PWM通道%d（地址0x%02X，引脚%d）设置为%d\n", 
                        channel, address, pwm_pins[channel], data);
        }
      }

      // 处理舵机控制（根据地址分配给对应舵机）
      if (address == SERVO1_ADDR) {
        servos[0].targetAngle = constrain(data, 0, 180);  // 限制角度0-180
        servos[0].isUpdating = true;
        Serial.print("舵机1目标角度: ");
        Serial.println(servos[0].targetAngle);
      } else if (address == SERVO2_ADDR) {
        servos[1].targetAngle = constrain(data, 0, 180);
        servos[1].isUpdating = true;
        Serial.print("舵机2目标角度: ");
        Serial.println(servos[1].targetAngle);
      } else if (address == SERVO3_ADDR) {
        servos[2].targetAngle = constrain(data, 0, 180);
        servos[2].isUpdating = true;
        Serial.print("舵机3目标角度: ");
        Serial.println(servos[2].targetAngle);
      } else if (address == SERVO4_ADDR) {
        servos[3].targetAngle = constrain(data, 0, 180);
        servos[3].isUpdating = true;
        Serial.print("舵机4目标角度: ");
        Serial.println(servos[3].targetAngle);
      } else if (address == SERVO5_ADDR) {
        servos[4].targetAngle = constrain(data, 0, 180);
        servos[4].isUpdating = true;
        Serial.print("舵机5目标角度: ");
        Serial.println(servos[4].targetAngle);
      }

      // 处理触发信号（地址0x66接收0x01时触发）
      if (address == TRIGGER_REGISTER && data == TRIGGER_VALUE) {
        digitalWrite(TRIGGER_PIN, LOW);
        delay(10);  // 10ms低电平
        digitalWrite(TRIGGER_PIN, HIGH);
        Serial.println("触发信号：GPIO25已翻转");
      }

      // 处理LED灯带控制（RGB三色合并）
      if (address == LED_CONTROL_R || address == LED_CONTROL_G || address == LED_CONTROL_B) {
        uint8_t r = memory_map[LED_CONTROL_R];
        uint8_t g = memory_map[LED_CONTROL_G];
        uint8_t b = memory_map[LED_CONTROL_B];
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(r, g, b));
        }
        strip.show();
        Serial.print("LED更新为RGB(");
        Serial.print(r);
        Serial.print(",");
        Serial.print(g);
        Serial.print(",");
        Serial.print(b);
        Serial.println(")");
      }

      Serial.print("数据0x");
      Serial.print(data, HEX);
      Serial.print("存储到地址0x");
      Serial.println(address, HEX);
      address++;  // 地址递增（下次存储到下一个地址）
    }
  }
  isAddress = true;  // 重置标志，准备下一次接收
}

// -------------------------- I2C请求函数（主机读取数据时触发） --------------------------
void requestEvent() {
  if (address < 256) {
    // 特殊处理：光敏电阻值（16位，分两个地址存储）
    if (address == LIGHT_SENSOR_ADDR) {
      Wire.write(memory_map[LIGHT_SENSOR_ADDR]);       // 低8位
      Wire.write(memory_map[LIGHT_SENSOR_ADDR + 1]);   // 高8位
      address = LIGHT_SENSOR_ADDR + 2;  // 跳过下一个地址
    } else {
      // 普通地址：返回1字节数据
      Wire.write(memory_map[address]);
      address++;
    }
  } else {
    // 无效地址返回0
    Wire.write(0);
    Serial.println("无效地址，返回0");
  }
}

// -------------------------- 舵机同步转动核心函数 --------------------------
void updateAllServos() {
  for (int i = 0; i < 5; i++) {  // 遍历5个舵机
    if (servos[i].isUpdating) {  // 如果需要更新角度
      int diff = servos[i].targetAngle - servos[i].currentAngle;
      if (diff != 0) {
        int step = (diff > 0) ? 1 : -1;
        servos[i].currentAngle += step;
        servos[i].servo.write(servos[i].currentAngle);
      } else {
        servos[i].isUpdating = false;
      }
    }
  }
}

// -------------------------- 初始化函数 --------------------------
void setup() {
  // 初始化PWM引脚（设置为输出模式）
  for (int i = 0; i < PWM_CHANNEL_COUNT; i++) {
    pinMode(pwm_pins[i], OUTPUT);
    analogWrite(pwm_pins[i], 0);  // 初始PWM值为0
    Serial.printf("PWM通道%d初始化，引脚%d\n", i, pwm_pins[i]);
  }

  

  // 舵机初始化
  for (int i = 0; i < 5; i++) {
    servos[i].servo.attach(servos[i].pin,500,2500);
    // 校准脉冲范围（根据舵机手册调整，示例：500-2500对应0-180度）
    // servos[i].servo.writ(0);  // 0度对应脉冲
    // delay(500);
    // servos[i].servo.write(180); // 180度对应脉冲
    // delay(500);
    // servos[i].servo.write(0);
    // servos[i].currentAngle = 0;
  }

  // 其他硬件初始化
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, HIGH);
  strip.begin();
  strip.clear();
  strip.show();

  // I2C初始化
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin(SLAVE_ID);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // 串口初始化
  Serial.begin(115200);
  Serial.println("系统初始化完成（I2C从机地址0x55）");
  Serial.printf("PWM通道初始化完成（0-%d通道，地址0x%02X-0x%02X）\n", 
                PWM_CHANNEL_COUNT-1, PWM_START_ADDR, PWM_END_ADDR);
}

// -------------------------- 主循环（非阻塞设计） --------------------------
void loop() {
  // 1. 同步更新所有舵机角度
  updateAllServos();

  // 2. 读取光敏电阻值（16位，分高低位存储）
  analogValue = analogRead(ANALOG_PIN);
  memory_map[LIGHT_SENSOR_ADDR] = analogValue & 0xFF;
  memory_map[LIGHT_SENSOR_ADDR + 1] = (analogValue >> 8) & 0xFF;

  delay(5);
}
