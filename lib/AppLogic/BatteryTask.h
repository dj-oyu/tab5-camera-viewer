#pragma once

#include <freertos/FreeRTOS.h>

class PipelineContext;

namespace BatteryTask {
void start(PipelineContext &ctx, UBaseType_t priority = 1, BaseType_t core = 1);
}
