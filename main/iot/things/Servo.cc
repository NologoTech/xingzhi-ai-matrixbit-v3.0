#include "iot/thing.h"
#include "board.h"
#include "audio_codec.h"

#include <driver/gpio.h>
#include <esp_log.h>

#define TAG "Sevor"

namespace iot {

class Sevor : public Thing {
public:
    Sevor() : Thing("Sevor", "舵机的角度设置") {
        // 定义设备可以被远程执行的指令
        methods_.AddMethod("TurnOn", "打开灯", ParameterList(), [this](const ParameterList&) {
            ESP_LOGE(TAG,"打开灯");
        });

        methods_.AddMethod("TurnOff", "关闭灯", ParameterList(), [this](const ParameterList&) {
            ESP_LOGE(TAG,"打开灯");
        });
    }
};

} 
DECLARE_THING(Sevor);
