#pragma once

#include <freertos/FreeRTOS.h>

class PipelineContext;

namespace DecodeTask {
void start(PipelineContext &ctx, UBaseType_t priority = 5, BaseType_t core = 1);
}
