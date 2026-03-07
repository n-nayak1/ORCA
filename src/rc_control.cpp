#include "rc_control.hpp"

namespace RCControl {

const uint8_t channelPins[NUM_CHANNELS] = {12, 14, 26, 27, 32, 33};

volatile uint16_t receiverValue[NUM_CHANNELS] = {
    1500, 1500, 1000, 1500, 1000, 1000
};

volatile bool newData[NUM_CHANNELS] = {
    false, false, false, false, false, false
};

static volatile uint32_t riseTimeUs[NUM_CHANNELS] = {
    0, 0, 0, 0, 0, 0
};

static volatile uint32_t lastUpdateUs[NUM_CHANNELS] = {
    0, 0, 0, 0, 0, 0
};

void IRAM_ATTR channelISR(void* arg) {
    const int ch = static_cast<int>(reinterpret_cast<intptr_t>(arg));
    const uint8_t pin = channelPins[ch];

    const bool level = digitalRead(pin);
    const uint32_t nowUs = micros();

    if (level) {
        riseTimeUs[ch] = nowUs;
    } else {
        const uint32_t widthUs = nowUs - riseTimeUs[ch];

        if (widthUs >= 800 && widthUs <= 2500) {
            receiverValue[ch] = static_cast<uint16_t>(widthUs);
            newData[ch] = true;
            lastUpdateUs[ch] = nowUs;
        }
    }
}

void begin() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
        pinMode(channelPins[i], INPUT_PULLUP);

        attachInterruptArg(
            digitalPinToInterrupt(channelPins[i]),
            channelISR,
            reinterpret_cast<void*>(static_cast<intptr_t>(i)),
            CHANGE
        );
    }
}

uint16_t readChannelRaw(int ch) {
    if (ch < 0 || ch >= NUM_CHANNELS) return 0;

    noInterrupts();
    uint16_t value = receiverValue[ch];
    interrupts();

    return value;
}

uint16_t readChannelClamped(int ch, uint16_t minUs, uint16_t maxUs) {
    uint16_t value = readChannelRaw(ch);
    if (value < minUs) value = minUs;
    if (value > maxUs) value = maxUs;
    return value;
}

bool hasNewData(int ch) {
    if (ch < 0 || ch >= NUM_CHANNELS) return false;

    noInterrupts();
    bool flag = newData[ch];
    interrupts();

    return flag;
}

void clearNewData(int ch) {
    if (ch < 0 || ch >= NUM_CHANNELS) return;

    noInterrupts();
    newData[ch] = false;
    interrupts();
}

float readChannelPercent(int ch, uint16_t minUs, uint16_t maxUs) {
    const uint16_t value = readChannelClamped(ch, minUs, maxUs);
    return 100.0f * (static_cast<float>(value - minUs) /
                     static_cast<float>(maxUs - minUs));
}

float readChannelUnit(int ch, uint16_t centerUs, uint16_t halfRangeUs) {
    const uint16_t value = readChannelRaw(ch);
    float x = (static_cast<float>(value) - static_cast<float>(centerUs)) /
              static_cast<float>(halfRangeUs);

    if (x > 1.0f) x = 1.0f;
    if (x < -1.0f) x = -1.0f;

    return x;
}

uint32_t getLastUpdateUs(int ch) {
    if (ch < 0 || ch >= NUM_CHANNELS) return 0;

    noInterrupts();
    uint32_t t = lastUpdateUs[ch];
    interrupts();

    return t;
}

bool isChannelFresh(int ch, uint32_t timeoutUs) {
    if (ch < 0 || ch >= NUM_CHANNELS) return false;

    const uint32_t last = getLastUpdateUs(ch);
    if (last == 0) return false;

    const uint32_t now = micros();
    return (now - last) <= timeoutUs;
}

bool isReceiverActive(uint32_t timeoutUs) {
    // At minimum, throttle channel must be fresh
    return isChannelFresh(2, timeoutUs);
}

} // namespace RCControl