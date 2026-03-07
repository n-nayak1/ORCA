#pragma once
#include <Arduino.h>

namespace RCControl {

constexpr int NUM_CHANNELS = 6;

extern const uint8_t channelPins[NUM_CHANNELS];
extern volatile uint16_t receiverValue[NUM_CHANNELS];
extern volatile bool newData[NUM_CHANNELS];

void begin();

uint16_t readChannelRaw(int ch);
uint16_t readChannelClamped(int ch, uint16_t minUs = 1000, uint16_t maxUs = 2000);

bool hasNewData(int ch);
void clearNewData(int ch);

float readChannelPercent(int ch, uint16_t minUs = 1000, uint16_t maxUs = 2000);
float readChannelUnit(int ch, uint16_t centerUs = 1500, uint16_t halfRangeUs = 500);

// Failsafe helpers
uint32_t getLastUpdateUs(int ch);
bool isChannelFresh(int ch, uint32_t timeoutUs = 50000);  // default 50 ms
bool isReceiverActive(uint32_t timeoutUs = 50000);

} // namespace RCControl