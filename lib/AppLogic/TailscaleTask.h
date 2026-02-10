#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

class PipelineContext;

#define VPN_CONNECTED_BIT BIT0

namespace TailscaleTask {
void start(PipelineContext &ctx, UBaseType_t priority = 3, BaseType_t core = 0);
EventGroupHandle_t eventGroup();
bool isConnected();
const char *vpnIpStr();
uint8_t peerCount();
} // namespace TailscaleTask
