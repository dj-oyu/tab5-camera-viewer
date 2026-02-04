#pragma once

#include <freertos/FreeRTOS.h>

class PipelineContext;

namespace DetectionTask {
void start(PipelineContext &ctx, UBaseType_t priority = 4, BaseType_t core = 1);
}
