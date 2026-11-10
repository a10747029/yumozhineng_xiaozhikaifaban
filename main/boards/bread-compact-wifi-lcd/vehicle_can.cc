#include "vehicle_can.h"

#include "config.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_twai.h>
#include <esp_twai_onchip.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {
constexpr char kTag[] = "VehicleCan";

std::string DescribeValue(const char* label, const VehicleCanValue& value, int64_t now_us,
                          const std::string& text) {
    std::string result(label);
    result += "：";
    if (value.received_us < 0) {
        return result + "尚未收到数据\n";
    }
    if (!value.valid) {
        result += "最近的数据无效";
    } else {
        result += text;
    }
    result += "（最后接收于" + std::to_string((now_us - value.received_us) / 1000) + "毫秒前）\n";
    return result;
}
}  // namespace

struct VehicleCanCallback {
    static bool IRAM_ATTR OnReceive(twai_node_handle_t node, const twai_rx_done_event_data_t*,
                                    void* arg);
};

bool VehicleCanCallback::OnReceive(twai_node_handle_t node, const twai_rx_done_event_data_t*,
                                   void* arg) {
    auto* self = static_cast<VehicleCan*>(arg);
    VehicleCan::Frame item = {};
    twai_frame_t frame = {};
    frame.buffer = item.bytes;
    frame.buffer_len = sizeof(item.bytes);
    if (twai_node_receive_from_isr(node, &frame) != ESP_OK) {
        self->receive_errors_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (frame.header.ide || frame.header.rtr || frame.header.fdf ||
        (frame.header.id != 0x777 && frame.header.id != 0x666 && frame.header.id != 0x555)) {
        return false;
    }
    item.id = frame.header.id;
    item.length = frame.header.dlc;
    item.received_us = esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(self->queue_, &item, &woken) != pdTRUE) {
        self->dropped_frames_.fetch_add(1, std::memory_order_relaxed);
    }
    return woken == pdTRUE;
}

bool VehicleCan::Start() {
    queue_ = xQueueCreate(64, sizeof(Frame));
    if (queue_ == nullptr) {
        ESP_LOGE(kTag, "Failed to create CAN receive queue");
        return false;
    }

    twai_onchip_node_config_t config = {};
    config.io_cfg.tx = VEHICLE_CAN_TX_PIN;
    config.io_cfg.rx = VEHICLE_CAN_RX_PIN;
    config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    config.bit_timing.bitrate = 500000;
    config.flags.enable_listen_only = true;
    twai_node_handle_t node = nullptr;
    esp_err_t err = twai_new_node_onchip(&config, &node);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "TWAI node creation failed: %s", esp_err_to_name(err));
        vQueueDelete(queue_);
        queue_ = nullptr;
        return false;
    }
    twai_event_callbacks_t callbacks = {};
    callbacks.on_rx_done = VehicleCanCallback::OnReceive;
    err = twai_node_register_event_callbacks(node, &callbacks, this);
    if (err == ESP_OK) {
        err = twai_node_enable(node);
    }
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "TWAI callback registration or start failed: %s", esp_err_to_name(err));
        ESP_ERROR_CHECK_WITHOUT_ABORT(twai_node_delete(node));
        vQueueDelete(queue_);
        queue_ = nullptr;
        return false;
    }
    if (xTaskCreate(TaskEntry, "vehicle_can", 4096, this, 5, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create CAN receiver task");
        if (twai_node_disable(node) == ESP_OK) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(twai_node_delete(node));
            vQueueDelete(queue_);
            queue_ = nullptr;
        } else {
            ESP_LOGE(kTag, "Failed to disable CAN node after task creation failure");
        }
        return false;
    }
    started_ = true;
    ESP_LOGI(kTag, "CAN listening at 500 kbit/s, TX=%d RX=%d", VEHICLE_CAN_TX_PIN,
             VEHICLE_CAN_RX_PIN);
    return true;
}

void VehicleCan::TaskEntry(void* arg) {
    static_cast<VehicleCan*>(arg)->ReceiveTask();
    vTaskDelete(nullptr);
}

void VehicleCan::ReceiveTask() {
    for (;;) {
        Frame frame = {};
        const bool received = xQueueReceive(queue_, &frame, pdMS_TO_TICKS(1000)) == pdTRUE;
        const uint32_t errors = receive_errors_.exchange(0, std::memory_order_relaxed);
        const uint32_t dropped = dropped_frames_.exchange(0, std::memory_order_relaxed);
        if (errors || dropped) {
            ESP_LOGW(kTag, "CAN receive errors: %lu, dropped frames: %lu",
                     static_cast<unsigned long>(errors), static_cast<unsigned long>(dropped));
        }
        if (!received) {
            continue;
        }
        bool valid;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            valid = data_.Update(frame.id, frame.bytes, frame.length, frame.received_us);
        }
        if (!valid) {
            ESP_LOGW(kTag, "Invalid CAN payload for ID 0x%03lx, DLC=%u",
                     static_cast<unsigned long>(frame.id), frame.length);
        }
    }
}

std::string VehicleCan::GetReport() {
    if (!started_) {
        return "CAN接收服务启动失败，无法查询车辆数据。";
    }

    VehicleCanData snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot = data_;
    }
    const int64_t now_us = esp_timer_get_time();
    const char* turn = snapshot.turn.value == 1   ? "左转"
                       : snapshot.turn.value == 2 ? "右转"
                                                  : "转向灯关闭";
    return "以下为最近接收到的CAN数据，时间较久可能已过期；车速单位未指定。\n" +
           DescribeValue("车速", snapshot.speed, now_us, std::to_string(snapshot.speed.value)) +
           DescribeValue("车辆电量", snapshot.battery, now_us,
                         std::to_string(snapshot.battery.value) + "%") +
           DescribeValue("转向灯", snapshot.turn, now_us, turn);
}
