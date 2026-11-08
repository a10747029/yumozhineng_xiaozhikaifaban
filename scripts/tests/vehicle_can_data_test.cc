#include "vehicle_can_data.h"

#include <cassert>
#include <cstdint>

int main() {
    VehicleCanData data;
    assert(!data.speed.valid && data.speed.received_us == -1);
    assert(!data.battery.valid && data.battery.received_us == -1);
    assert(!data.turn.valid && data.turn.received_us == -1);

    const uint8_t speed[] = {0x01, 0x02};
    assert(data.Update(0x777, speed, 2, 100));
    assert(data.speed.valid && data.speed.value == 258 && data.speed.received_us == 100);
    const uint8_t max_speed[] = {0xff, 0xff};
    assert(data.Update(0x777, max_speed, 2, 101));
    assert(data.speed.value == 65535);
    assert(!data.Update(0x777, speed, 1, 102));
    assert(!data.speed.valid && data.speed.received_us == 102);
    assert(data.Update(0x777, speed, 2, 103) && data.speed.valid);

    const uint8_t battery[] = {0, 100, 101, 255};
    for (unsigned i = 0; i < 4; ++i) {
        assert(data.Update(0x666, &battery[i], 1, 200 + i));
        assert(data.battery.valid);
        assert(data.battery.value == (i == 0 ? 0 : 100));
    }
    assert(!data.Update(0x666, nullptr, 1, 205));
    assert(!data.battery.valid);
    assert(!data.Update(0x666, battery, 2, 206));

    const uint8_t turn[] = {0, 1, 2, 3, 255};
    for (unsigned i = 0; i < 3; ++i) {
        assert(data.Update(0x555, &turn[i], 1, 300 + i));
        assert(data.turn.valid && data.turn.value == i);
    }
    assert(!data.Update(0x555, &turn[3], 1, 303));
    assert(!data.turn.valid && data.turn.received_us == 303);
    assert(!data.Update(0x555, &turn[4], 1, 304));
    assert(data.Update(0x555, turn, 1, 305) && data.turn.value == 0 && data.turn.valid);
    assert(!data.Update(0x555, turn, 0, 306));
    assert(!data.turn.valid);

    assert(data.Update(0x123, speed, 2, 999));
    assert(data.speed.received_us == 103 && data.battery.received_us == 206 &&
           data.turn.received_us == 306);
}
