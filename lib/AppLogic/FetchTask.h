#pragma once

#include <freertos/FreeRTOS.h>

class PipelineContext;

namespace FetchTask {
void start(PipelineContext &ctx, UBaseType_t priority = 6, BaseType_t core = 1);
}
