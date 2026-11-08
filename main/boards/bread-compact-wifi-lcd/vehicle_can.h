#pragma once

#include "vehicle_can_data.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <atomic>
#include <mutex>
#include <string>

class VehicleCan {
public:
    bool Start();
    std::string GetReport();

private:
    friend struct VehicleCanCallback;

    struct Frame {
        uint32_t id;
        uint16_t length;
        uint8_t bytes[8];
        int64_t received_us;
    };

    static void TaskEntry(void* arg);
    void ReceiveTask();

    QueueHandle_t queue_ = nullptr;
    std::atomic<uint32_t> receive_errors_{0};
    std::atomic<uint32_t> dropped_frames_{0};
    std::mutex mutex_;
    VehicleCanData data_;
    bool started_ = false;
};
