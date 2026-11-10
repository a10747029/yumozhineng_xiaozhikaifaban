#pragma once

#include <cstddef>
#include <cstdint>

struct VehicleCanValue {
    uint16_t value = 0;
    int64_t received_us = -1;
    bool valid = false;
};

struct VehicleCanData {
    VehicleCanValue speed;
    VehicleCanValue battery;
    VehicleCanValue turn;

    // Called only for standard data frames. Returns false for malformed relevant frames.
    bool Update(uint32_t id, const uint8_t* data, size_t length, int64_t now_us) {
        VehicleCanValue* target = nullptr;
        size_t expected_length = 1;
        switch (id) {
            case 0x777:
                target = &speed;
                expected_length = 2;
                break;
            case 0x666:
                target = &battery;
                break;
            case 0x555:
                target = &turn;
                break;
            default:
                return true;
        }

        target->received_us = now_us;
        if (length != expected_length || data == nullptr) {
            target->valid = false;
            return false;
        }
        if (id == 0x777) {
            target->value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        } else if (id == 0x666) {
            target->value = data[0] > 100 ? 100 : data[0];
        } else {
            target->value = data[0];
            if (data[0] > 2) {
                target->valid = false;
                return false;
            }
        }
        target->valid = true;
        return true;
    }
};
